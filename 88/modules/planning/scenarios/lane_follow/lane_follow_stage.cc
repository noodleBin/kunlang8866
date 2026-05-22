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

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/count_appeared_obstacles.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "modules/planning/tasks/deciders/path_decider/path_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

namespace century {
namespace planning {
namespace scenario {
namespace lane_follow {

using century::common::ErrorCode;
using century::common::SLPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::util::PointFactory;
using century::cyber::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
constexpr double kReferenceLineStaticObsCost = 1e3;
constexpr double kMaxNumOfReferenceline = 2;
constexpr int kSpeedFailTimeThreshold = 3;
constexpr double kLargeDistance = 1000.0;
}  // namespace

LaneFollowStage::LaneFollowStage(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {}

void LaneFollowStage::CheckEnableAutoBorrow(
    ReferenceLineInfo* reference_line_info) {
  bool is_adc_in_lane = reference_line_info->IsAdcCoverInLane();
  // AINFO << "is_adc_in_lane = " << is_adc_in_lane;
  bool is_in_lanechage = util::IsLaneChange(injector_->planning_context());
  // AINFO << "is_in_lanechage = " << is_in_lanechage;
  bool is_in_borrow = util::IsLaneBorrow(injector_->planning_context());
  // AINFO << "is_in_borrow = " << is_in_borrow;

  if (!injector_->is_auto_state_) {
    injector_->is_last_auto_state_ = false;
    injector_->auto_state_count_ = 0;
    // borrow for auto borrow ,and manual to lane ,so no borrow
    if (injector_->borrow_response().has_response() &&
        injector_->borrow_response().block_obs_id() == "auto borrow" &&
        is_adc_in_lane) {
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::UNTREATED);
      borrow_response.set_block_obs_id("");
      borrow_response.set_has_response(false);
      injector_->set_borrow_response(borrow_response);
    }
  }
  if (!injector_->is_last_auto_state_ && injector_->is_auto_state_) {
    AINFO << "last no auto state , now auto";
    // injector_->auto_state_count_++;
    injector_->auto_state_count_ = 100;
    injector_->auto_state_count_ = std::min(injector_->auto_state_count_, 100);
    injector_->is_last_auto_state_ = true;
  }
  // AINFO << "injector_->auto_state_count_ = " << injector_->auto_state_count_;
  if (!is_adc_in_lane && !is_in_lanechage && !is_in_borrow &&
      injector_->auto_state_count_ == 100) {
    AINFO << "need to accept borrow";
    planning::BorrowResponse borrow_response;
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id("auto borrow");
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
    injector_->auto_state_count_ = 0;
  } else {
    injector_->auto_state_count_ = 0;
    // injector_->auto_state_count_--;
    // injector_->auto_state_count_ = std::max(injector_->auto_state_count_,0);
  }
  if (injector_->borrow_response().response_type() ==
      planning::ResponseType::ACCEPT) {
    // AINFO<<"in accept model";
    injector_->auto_state_count_ = 0;
  } else {
    // AINFO<<"no accept model";
  }
  // AINFO<<"injector_->auto_state_count_ = "<<injector_->auto_state_count_;
}
void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();

  const auto path_decision = reference_line_info->path_decision();
  for (const auto obstacle : path_decision->obstacles().Items()) {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->set_slow_breaking_tag(obstacle->SlowBreakingTag());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

Stage::StageStatus LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();

  unsigned int count = 0;

  injector_->is_multi_reference_line_ = false;
  if (frame->mutable_reference_line_info()->size() >= kMaxNumOfReferenceline) {
    injector_->is_multi_reference_line_ = true;
  }
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line.";
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      break;
    }

    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    SetCommonStatus(cur_status);

    if (cur_status.ok()) {
      if (reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is lane change ref.";
        ADEBUG << "FLAGS_enable_smarter_lane_change: "
               << FLAGS_enable_smarter_lane_change;
        if (reference_line_info.Cost() < kPathOptimizationFallbackCost &&
            LaneChangeDecider::IsClearToChangeLane(
                &reference_line_info,
                injector_->is_adc_in_expressway_junction_)) {
          // If the path and speed optimization succeed on target lane while
          // under smart lane-change or IsClearToChangeLane under older version
          has_drivable_reference_line = true;
          reference_line_info.SetDrivable(true);
          LaneChangeDecider::UpdatePreparationDistance(
              reference_line_info.GetIsClearToChangeLane(), frame,
              &reference_line_info, injector_->planning_context());
          ADEBUG << "\tclear for lane change";
        } else {
          LaneChangeDecider::UpdatePreparationDistance(
              false, frame, &reference_line_info,
              injector_->planning_context());
          reference_line_info.SetDrivable(false);
          ADEBUG << "\tlane change failed";
        }
      } else {
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
      }
    } else {
      reference_line_info.SetDrivable(false);
    }
  }

  return has_drivable_reference_line ? StageStatus::RUNNING
                                     : StageStatus::ERROR;
}

Status LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();

  double check_s = reference_line_info->AdcSlBoundary().end_s();
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    const std::string msg = "Fail to get reference point.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  const auto& waypoint = ref_point.lane_waypoints().front();
  hdmap::Lane_LaneType lane_type = waypoint.lane->lane().type();
  reference_line_info->setLaneType(lane_type);
  if (lane_type == hdmap::Lane::PLAY_STREET) {
    injector_->is_in_play_street = true;
  } else {
    injector_->is_in_play_street = false;
  }

  if (FLAGS_enable_auto_borrow) {
    CheckEnableAutoBorrow(reference_line_info);
  }
  //  Counting the consecutive occurrences of obstacles perceived.
  AppearedObstacle::CountAppearedObstacles(
      injector_, reference_line_info->path_decision()->obstacles().Items());
  auto ret = Status::OK();
  for (auto* task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    ret = task->Execute(frame, reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      const std::string msg = "Failed task[" + task->Name() + "],";
      ret.merge_error_message_inv(msg);
      if (task->Name() != injector_->GetTaskFailureName()) {
        injector_->SetTaskFailureName(task->Name());
      }
      break;
    }
  }

  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  return CheckTrajectoryResultIsOK(ret, planning_start_point, frame,
                                   reference_line_info);
}

common::Status LaneFollowStage::CheckTrajectoryResultIsOK(
    const common::Status& ret,
    const common::TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!ret.ok()) {
    PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
    injector_->AddTaskFailureCount();
    injector_->SetPoorStatusOfTaskFailure();
  } else {
    injector_->ResetTaskFailureInfo();
    injector_->ClearPoorStatusOfTaskFailure();
  }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    Status ret_stat(ErrorCode::PLANNING_ERROR, msg);
    ret_stat.merge_error_message(ret);
    return ret_stat;
  }

  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }

  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      Status ret_stat(ErrorCode::PLANNING_ERROR, msg);
      ret_stat.merge_error_message(ret);
      return ret_stat;
    }
  }

  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  auto ret_stat = Status::OK();
  ret_stat.merge_error_message(ret);
  return ret_stat;
}

void LaneFollowStage::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // path and speed fall back
  if (reference_line_info->path_data().Empty()) {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    if (!RetrieveLastFramePathProfile(
            reference_line_info, frame,
            reference_line_info->mutable_path_data())) {
      const auto& candidate_path_data =
          reference_line_info->GetCandidatePathData();
      for (const auto& path_data : candidate_path_data) {
        if (path_data.path_label().find("self") != std::string::npos) {
          *reference_line_info->mutable_path_data() = path_data;
          AERROR << "Use current frame self lane path as fallback ";
          break;
        }
      }
    }
  }

  AERROR << "Speed fallback due to algorithm failure";
  *reference_line_info->mutable_speed_data() =
      SpeedProfileGenerator::GenerateFallbackSpeed(injector_->ego_info());

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }
}

void LaneFollowStage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  if (std::fabs(adc_s) > kLargeDistance) {
    AERROR << "adc with ref line has large distance.";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const double max_s = reference_line.Length();
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

bool LaneFollowStage::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* path_data) {
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR
        << "Last frame doesn't succeed, fail to retrieve last frame path data";
    return false;
  }

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();

  path_data->SetDiscretizedPath(last_frame_discretized_path);
  const auto adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return false;
  }
  AERROR << "Use last frame good path to do speed fallback";
  return true;
}

SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace century
