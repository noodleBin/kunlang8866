/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

#include <algorithm>
#include <memory>
#include <string>

#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/common/planning_context.h"

namespace century {
namespace planning {

using century::common::Status;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;

int PathReuseDecider::reusable_path_counter_ = 0;
int PathReuseDecider::total_path_counter_ = 0;
bool PathReuseDecider::path_reusable_ = false;
namespace {
constexpr int kWaitCycle = -2;  // wait 2 cycle
constexpr double kPathBoundsDeciderResolution = 0.5;
constexpr double kMinPlanTrajTimeLength = 5.0;
}  // namespace

PathReuseDecider::PathReuseDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FLAGS_enable_anchor_lane_change_path) {
    reference_line_info->set_lane_change_path_reusable(false);
    return Status::OK();
  }
  auto* path_status = injector_->planning_context()->mutable_planning_status();
  auto* lane_change_status = path_status->mutable_change_lane();

  if ((!reference_line_info->IsChangeLanePath() &&
       reference_line_info->IsAdcPostureStraight()) ||
      !reference_line_info->GetIsClearToChangeLane() ||
      !lane_change_status->is_success_change_lane_path() ||
      path_status->path_decider().will_pass_merge_lane_area() ||
      VaildPathLable::LANE_CHANGE != injector_->last_path_label_) {
    reference_line_info->set_lane_change_path_reusable(false);
    ADEBUG << "[Reuse Lane Change Path] not reusable.";
    lane_change_status->set_is_success_change_lane_path(false);
    return Status::OK();
  }
  if (!TrimLastPath(frame, reference_line_info)) {
    reference_line_info->set_lane_change_path_reusable(false);
    AINFO << "[Reuse Lane Change Path] fail to TrimLastPath!";
    return Status::OK();
  }

  reference_line_info->set_lane_change_path_reusable(true);
  ADEBUG << "[Reuse Lane Change Path] path reusable";
  return Status::OK();
}

Status PathReuseDecider::OriginProcess(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& config = config_.path_reuse_decider_config();
  if (!config.reuse_path()) {
    ADEBUG << "skipping reusing path: conf";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  // skip path reuse if not in LANE_FOLLOW_SCENARIO
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  if (ScenarioConfig::LANE_FOLLOW != scenario_type ||
      reference_line_info->path_data().is_valid_path_reference()) {
    ADEBUG << "skipping reusing path, is_valid_path_reference: "
           << reference_line_info->path_data().is_valid_path_reference();
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  // active path reuse during change_lane only
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  ADEBUG << "lane change status: " << lane_change_status->ShortDebugString();

  // skip path reuse if not in_change_lane
  if (lane_change_status->status() != ChangeLaneStatus::IN_CHANGE_LANE &&
      !config.enable_reuse_path_in_lane_follow()) {
    ADEBUG << "skipping reusing path: not in lane_change";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  /*count total_path_ when in_change_lane && reuse_path*/
  ++total_path_counter_;

  /*reuse path when in non_change_lane reference line or
    optimization succeeded in change_lane reference line
  */
  if (reference_line_info->IsChangeLanePath() &&
      !lane_change_status->is_current_opt_succeed()) {
    reference_line_info->set_path_reusable(false);
    ADEBUG << "reusable_path_counter[" << reusable_path_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    ADEBUG << "Stop reusing path when optimization failed on change lane path";
    return Status::OK();
  }

  // stop reusing current path:
  // 1. replan path
  // 2. collision
  // 3. failed to trim previous path
  // 4. speed optimization failed on previous path
  bool speed_optimization_successful = false;
  const auto& history_frame = injector_->frame_history()->Latest();
  if (history_frame) {
    const auto history_trajectory_type =
        history_frame->reference_line_info().front().trajectory_type();
    speed_optimization_successful =
        (history_trajectory_type != ADCTrajectory::SPEED_FALLBACK);
  }

  if (path_reusable_) {
    if (!frame->current_frame_planned_trajectory().is_replan() &&
        speed_optimization_successful && IsCollisionFree(reference_line_info) &&
        TrimHistoryPath(frame, reference_line_info)) {
      ADEBUG << "reuse path";
      ++reusable_path_counter_;  // count reusable path
    } else {
      // stop reuse path
      ADEBUG << "stop reuse path";
      path_reusable_ = false;
    }
  } else {
    // F -> T
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    const int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    const bool ignore_blocking_obstacle =
        IsIgnoredBlockingObstacle(reference_line_info);
    ADEBUG << "counter[" << front_static_obstacle_cycle_counter
           << "] IsIgnoredBlockingObstacle[" << ignore_blocking_obstacle << "]";
    // stop reusing current path:
    // 1. blocking obstacle disappeared or moving far away
    // 2. trimming successful
    // 3. no statical obstacle collision.
    if ((front_static_obstacle_cycle_counter <= kWaitCycle ||
         ignore_blocking_obstacle) &&
        speed_optimization_successful && IsCollisionFree(reference_line_info) &&
        TrimHistoryPath(frame, reference_line_info)) {
      // enable reuse path
      ADEBUG << "reuse path: front_blocking_obstacle ignorable";
      path_reusable_ = true;
      ++reusable_path_counter_;
    }
  }

  reference_line_info->set_path_reusable(path_reusable_);
  ADEBUG << "reusable_path_counter[" << reusable_path_counter_
         << "] total_path_counter[" << total_path_counter_ << "]";
  return Status::OK();
}

bool PathReuseDecider::IsIgnoredBlockingObstacle(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  static constexpr double kSDistBuffer = 30.0;  // meter
  static constexpr int kTimeBuffer = 3;         // second
  // vehicle speed
  double adc_speed = injector_->vehicle_state()->linear_velocity();
  double final_s_buffer = std::max(kSDistBuffer, kTimeBuffer * adc_speed);
  // current vehicle s position
  common::SLPoint adc_position_sl;
  GetADCSLPoint(reference_line, &adc_position_sl);
  // blocking obstacle start s
  double blocking_obstacle_start_s;
  if (GetBlockingObstacleS(reference_line_info, &blocking_obstacle_start_s) &&
      // distance to blocking obstacle
      (blocking_obstacle_start_s - adc_position_sl.s() > final_s_buffer)) {
    ADEBUG << "blocking obstacle distance: "
           << blocking_obstacle_start_s - adc_position_sl.s();
    return true;
  } else {
    return false;
  }
}

bool PathReuseDecider::GetBlockingObstacleS(
    ReferenceLineInfo* const reference_line_info, double* blocking_obstacle_s) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // get blocking obstacle ID (front_static_obstacle_id)
  const std::string& blocking_obstacle_ID =
      mutable_path_decider_status->front_static_obstacle_id();
  const IndexedList<std::string, Obstacle>& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  const auto* blocking_obstacle = indexed_obstacles.Find(blocking_obstacle_ID);

  if (blocking_obstacle == nullptr) {
    return false;
  }

  const auto& obstacle_sl = blocking_obstacle->PerceptionSLBoundary();
  *blocking_obstacle_s = obstacle_sl.start_s();
  ADEBUG << "blocking obstacle distance: " << obstacle_sl.start_s();
  return true;
}

void PathReuseDecider::GetADCSLPoint(const ReferenceLine& reference_line,
                                     common::SLPoint* adc_position_sl) {
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_position, adc_position_sl);
}

bool PathReuseDecider::IsCollisionFree(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  static constexpr double kMinObstacleArea = 1e-4;
  const double kSBuffer = 0.5;
  static constexpr int kNumExtraTailBoundPoint = 21;
  // current vehicle sl position
  common::SLPoint adc_position_sl;
  GetADCSLPoint(reference_line, &adc_position_sl);

  // get history path
  const auto& history_frame = injector_->frame_history()->Latest();
  if (!history_frame) {
    return false;
  }
  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  // path end point
  common::SLPoint path_end_position_sl;
  common::math::Vec2d path_end_position = {history_path.back().x(),
                                           history_path.back().y()};
  reference_line.XYToSL(path_end_position, &path_end_position_sl);

  // current obstacles
  std::vector<Polygon2d> obstacle_polygons;
  for (auto obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // filtered all non-static objects and virtual obstacle
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      if (!obstacle->IsStatic()) {
        ADEBUG << "SPOT a dynamic obstacle";
      }
      if (obstacle->IsVirtual()) {
        ADEBUG << "SPOT a virtual obstacle";
      }
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    // Ignore obstacles behind ADC
    if ((obstacle_sl.end_s() < adc_position_sl.s() - kSBuffer) ||
        // Ignore too small obstacles.
        (obstacle_sl.end_s() - obstacle_sl.start_s()) *
                (obstacle_sl.end_l() - obstacle_sl.start_l()) <
            kMinObstacleArea) {
      continue;
    }

    for (size_t i = 0; i < history_path.size(); ++i) {
      common::SLPoint path_position_sl;
      common::math::Vec2d path_position = {history_path[i].x(),
                                           history_path[i].y()};
      reference_line.XYToSL(path_position, &path_position_sl);
      if (path_end_position_sl.s() - path_position_sl.s() <=
          kNumExtraTailBoundPoint * kPathBoundsDeciderResolution) {
        break;
      }
      if (path_position_sl.s() < adc_position_sl.s() - kSBuffer) {
        continue;
      }
      const auto& vehicle_box =
          common::VehicleConfigHelper::Instance()->GetBoundingBox(
              history_path[i]);
      SLBoundary ADC_sl_boundary_path;
      if (!reference_line.GetSLBoundary(vehicle_box, &ADC_sl_boundary_path)) {
        AERROR << "Failed to get the ADC sl boundary from vehicle box onto "
                  "reference_line";
        return false;
      }

      bool no_overlap =
          ((ADC_sl_boundary_path.end_s() < obstacle_sl.start_s() ||
            ADC_sl_boundary_path.start_s() >
                obstacle_sl.end_s()) ||  // longitudinal
           (ADC_sl_boundary_path.end_l() +
                    FLAGS_static_obstacle_nudge_l_buffer <
                obstacle_sl.start_l() ||
            ADC_sl_boundary_path.start_l() -
                    FLAGS_static_obstacle_nudge_l_buffer >
                obstacle_sl.end_l()));  // lateral

      if (!no_overlap) {
        // for debug
        ADEBUG << "s distance to end point:" << path_end_position_sl.s();
        ADEBUG << "s distance to end point:" << path_position_sl.s();
        ADEBUG << "collision: [" << i << "]"
               << ", history_path[i].x(): " << std::setprecision(9)
               << history_path[i].x() << ", history_path[i].y()"
               << std::setprecision(9) << history_path[i].y();
        return false;
      }
    }
  }

  return true;
}

// check the length of the path
bool PathReuseDecider::NotShortPath(const DiscretizedPath& current_path) {
  // TODO(shu): use gflag
  static constexpr double kShortPathThreshold = 60;
  return current_path.size() >= kShortPathThreshold;
}

bool PathReuseDecider::TrimHistoryPath(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const auto& history_frame = injector_->frame_history()->Latest();
  if (!history_frame) {
    ADEBUG << "no history frame";
    return false;
  }

  const common::TrajectoryPoint history_planning_start_point =
      history_frame->PlanningStartPoint();
  common::PathPoint history_init_path_point =
      history_planning_start_point.path_point();
  ADEBUG << "history_init_path_point x:[" << std::setprecision(9)
         << history_init_path_point.x() << "], y["
         << history_init_path_point.y() << "], s: ["
         << history_init_path_point.s() << "]";

  const common::TrajectoryPoint planning_start_point =
      frame->PlanningStartPoint();
  common::PathPoint init_path_point = planning_start_point.path_point();
  ADEBUG << "init_path_point x:[" << std::setprecision(9) << init_path_point.x()
         << "], y[" << init_path_point.y() << "], s: [" << init_path_point.s()
         << "]";

  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  DiscretizedPath trimmed_path;
  common::SLPoint adc_position_sl;  // current vehicle sl position
  GetADCSLPoint(reference_line, &adc_position_sl);
  ADEBUG << "adc_position_sl.s(): " << adc_position_sl.s();

  size_t path_start_index = 0;

  for (size_t i = 0; i < history_path.size(); ++i) {
    // find previous init point
    if (history_path[i].s() > 0) {
      path_start_index = i;
      break;
    }
  }
  ADEBUG << "!!!path_start_index[" << path_start_index << "]";

  // get current s=0
  common::SLPoint init_path_position_sl;
  reference_line.XYToSL(init_path_point, &init_path_position_sl);
  bool inserted_init_point = false;

  for (size_t i = path_start_index; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};

    reference_line.XYToSL(path_position, &path_position_sl);

    double updated_s = path_position_sl.s() - init_path_position_sl.s();
    // insert init point
    if (updated_s > 0 && !inserted_init_point) {
      trimmed_path.emplace_back(init_path_point);
      trimmed_path.back().set_s(0);
      inserted_init_point = true;
    }

    trimmed_path.emplace_back(history_path[i]);

    // if (i < 50) {
    //   ADEBUG << "path_point:[" << i << "]" << updated_s;
    //   path_position_sl.s();
    //   ADEBUG << std::setprecision(9) << "path_point:[" << i << "]"
    //          << "x: [" << history_path[i].x() << "], y:[" <<
    //          history_path[i].y()
    //          << "]. s[" << history_path[i].s() << "]";
    // }
    trimmed_path.back().set_s(updated_s);
  }

  ADEBUG << "trimmed_path[0]: " << trimmed_path.front().s();
  ADEBUG << "[END] trimmed_path.size(): " << trimmed_path.size();

  if (!NotShortPath(trimmed_path)) {
    ADEBUG << "short path: " << trimmed_path.size();
    return false;
  }

  // set path
  auto path_data = reference_line_info->mutable_path_data();
  ADEBUG << "previous path_data size: " << history_path.size();
  path_data->SetReferenceLine(&reference_line);
  ADEBUG << "previous path_data size: " << path_data->discretized_path().size();
  path_data->SetDiscretizedPath(std::move(trimmed_path));
  ADEBUG << "not short path: " << trimmed_path.size();
  ADEBUG << "current path size: "
         << reference_line_info->path_data().discretized_path().size();

  return true;
}

bool PathReuseDecider::TrimLastPath(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const DiscretizedPath& last_discretized_path =
      injector_->last_path_data().discretized_path();
  if (last_discretized_path.empty()) {
    AERROR << "last path data is empty, can not trim path.";
    return false;
  }

  size_t path_start_index = 0;
  for (size_t i = 0; i < last_discretized_path.size(); ++i) {
    if (last_discretized_path[i].s() > 0.0) {
      path_start_index = i;
      break;
    }
  }

  const common::TrajectoryPoint& planning_start_point =
      frame->PlanningStartPoint();
  const common::PathPoint& init_path_point = planning_start_point.path_point();
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  common::SLPoint init_path_position_sl;
  reference_line.XYToSL(init_path_point, &init_path_position_sl);
  adc_frenet_s_ = init_path_position_sl.s();
  DiscretizedPath trimmed_discretized_path;
  bool inserted_init_point = false;
  for (size_t i = path_start_index; i < last_discretized_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {last_discretized_path[i].x(),
                                         last_discretized_path[i].y()};

    reference_line.XYToSL(path_position, &path_position_sl);

    double updated_s = path_position_sl.s() - init_path_position_sl.s();
    // insert init point
    if (updated_s > 0 && !inserted_init_point) {
      trimmed_discretized_path.emplace_back(init_path_point);
      trimmed_discretized_path.back().set_s(0);
      inserted_init_point = true;
    }
    trimmed_discretized_path.emplace_back(last_discretized_path[i]);
    trimmed_discretized_path.back().set_s(updated_s);
  }

  if (ShortLaneChangePath(reference_line_info, trimmed_discretized_path)) {
    AINFO << "lane change path is short";
    return false;
  }
  ADEBUG << "current path_data size: " << trimmed_discretized_path.size();

  // set path
  auto* path_data = reference_line_info->mutable_path_data();
  path_data->set_path_label("regular/lanechange");
  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(std::move(trimmed_discretized_path));
  path_data->set_blocking_obstacle_id("");
  return true;
}

bool PathReuseDecider::ShortLaneChangePath(
    ReferenceLineInfo* const reference_line_info, const DiscretizedPath& path) {
  if (path.empty()) {
    AERROR << "Trimmed path size empty!";
    return true;
  }
  const auto& config = config_.path_reuse_decider_config();
  double adc_v = std::fabs(injector_->vehicle_state()->linear_velocity());
  double dec_t = config.min_lane_change_traj_time_length();
  double dec_a = config.allow_max_deceleration_for_lane_change_path();
  double allow_min_length =
      adc_v * dec_t - 0.5 * std::fabs(dec_a) * dec_t * dec_t;

  if (path.back().s() < allow_min_length) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace century
