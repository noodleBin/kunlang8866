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

#include "modules/planning/traffic_rules/crosswalk.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "cyber/time/clock.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::common::Status;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
using century::hdmap::CrosswalkInfoConstPtr;
using century::hdmap::HDMapUtil;
using century::hdmap::PathOverlap;

namespace {
constexpr double kLateralBuffer = 0.3;
constexpr double kLongitudeBuffer = 0.5;  // m
}  // namespace

Crosswalk::Crosswalk(const TrafficRuleConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    return Status::OK();
  }

  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto* mutable_crosswalk_status = injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_crosswalk();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  CrosswalkToStop crosswalks_to_stop;

  // read crosswalk_stop_timer from saved status
  CrosswalkStopTimer crosswalk_stop_timer;
  std::unordered_map<std::string, double> stop_times;
  for (const auto& stop_time : mutable_crosswalk_status->stop_time()) {
    stop_times.emplace(stop_time.obstacle_id(), stop_time.stop_timestamp_sec());
  }
  crosswalk_stop_timer.emplace(mutable_crosswalk_status->crosswalk_id(),
                               stop_times);

  const auto& finished_crosswalks =
      mutable_crosswalk_status->finished_crosswalk();

  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    if (nullptr == crosswalk_ptr) {
      ADEBUG << "Failed get crosswalk ptr by id["
             << crosswalk_overlap->object_id << "].";
      continue;
    }
    std::string crosswalk_id = crosswalk_ptr->id().id();

    // skip crosswalk if master vehicle body already passes the stop line
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.crosswalk().min_pass_s_distance()) {
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }

    // check if crosswalk already finished
    if (finished_crosswalks.end() != std::find(finished_crosswalks.begin(),
                                               finished_crosswalks.end(),
                                               crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id << "] crosswalk_end_s["
             << crosswalk_overlap->end_s << "] finished already";
      continue;
    }

    const double stop_deceleration = util::GetADCStopDeceleration(
        injector_->vehicle_state(), adc_front_edge_s,
        crosswalk_overlap->start_s);

    std::vector<std::string> pedestrians;
    MakeDecisionsForObstacle(
        reference_line_info, crosswalk_ptr, stop_deceleration, adc_front_edge_s,
        crosswalk_id, crosswalk_overlap, &crosswalk_stop_timer, &pedestrians);

    if (!pedestrians.empty()) {
      crosswalks_to_stop.emplace_back(crosswalk_overlap, pedestrians);
      ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
    }
  }

  BuildStopDecisionAndUpdateCrosswalkStatus(
      frame, reference_line_info, crosswalks_to_stop, &crosswalk_stop_timer,
      mutable_crosswalk_status);

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
}

void Crosswalk::MakeDecisionsForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::CrosswalkInfoConstPtr crosswalk_ptr,
    const double stop_deceleration, const double adc_front_edge_s,
    const std::string& crosswalk_id,
    const hdmap::PathOverlap* crosswalk_overlap,
    CrosswalkStopTimer* crosswalk_stop_timer,
    std::vector<std::string>* const pedestrians) {
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    bool stop = CheckStopForObstacle(reference_line_info, crosswalk_ptr,
                                     *obstacle, stop_deceleration);

    const std::string& obstacle_id = obstacle->Id();
    const PerceptionObstacle& perception_obstacle = obstacle->Perception();
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    // update stop timestamp on static pedestrian for watch timer
    const double kStartWatchTimerDistance = 40.0;
    if (stop && crosswalk_overlap->start_s - adc_front_edge_s <=
                    kStartWatchTimerDistance) {
      // check on stop timer for static pedestrians/bicycles
      // if NOT on_lane ahead of adc
      const double kMaxStopSpeed = 0.3;
      auto obstacle_speed = obstacle->speed();
      if (obstacle_speed <= kMaxStopSpeed) {
        if ((*crosswalk_stop_timer)[crosswalk_id].count(obstacle_id) < 1) {
          // add timestamp
          ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                 << "] timestamp[" << Clock::NowInSeconds() << "]";
          (*crosswalk_stop_timer)[crosswalk_id].insert(
              {obstacle_id, Clock::NowInSeconds()});
        } else {
          double stop_time = Clock::NowInSeconds() -
                             (*crosswalk_stop_timer)[crosswalk_id][obstacle_id];
          ADEBUG << "stop_time: obstacle_id[" << obstacle_id << "] stop_time["
                 << stop_time << "]";
          if (stop_time >= config_.crosswalk().stop_timeout()) {
            stop = false;
          }
        }
      }
    }

    if (stop) {
      pedestrians->push_back(obstacle_id);
      ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    } else {
      ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
  }
}

void Crosswalk::BuildStopDecisionAndUpdateCrosswalkStatus(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const CrosswalkToStop& crosswalks_to_stop,
    CrosswalkStopTimer* crosswalk_stop_timer,
    CrosswalkStatus* mutable_crosswalk_status) {
  double min_s = std::numeric_limits<double>::max();
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    // build stop decision
    const auto* crosswalk_overlap = crosswalk_to_stop.first;
    ADEBUG << "BuildStopDecision: crosswalk[" << crosswalk_overlap->object_id
           << "] start_s[" << crosswalk_overlap->start_s << "]";
    std::string virtual_obstacle_id =
        CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
    util::BuildStopDecision(virtual_obstacle_id, crosswalk_overlap->start_s,
                            config_.crosswalk().stop_distance(),
                            StopReasonCode::STOP_REASON_CROSSWALK,
                            crosswalk_to_stop.second,
                            TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                            frame, reference_line_info);

    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      min_s = crosswalk_to_stop.first->start_s;
    }
  }

  if (firsts_crosswalk_to_stop) {
    // update CrosswalkStatus
    std::string crosswalk_id = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk_id);
    mutable_crosswalk_status->clear_stop_time();
    for (const auto& timer : (*crosswalk_stop_timer)[crosswalk_id]) {
      // remove old data
      if (Clock::NowInSeconds() - timer.second >
          config_.crosswalk().ignore_time()) {
        continue;
      }
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      stop_time->set_obstacle_id(timer.first);
      stop_time->set_stop_timestamp_sec(timer.second);
      ADEBUG << "UPDATE stop_time: id[" << crosswalk_id << "] obstacle_id["
             << timer.first << "] stop_timestamp[" << timer.second << "]";
    }

    // update CrosswalkStatus.finished_crosswalk
    mutable_crosswalk_status->clear_finished_crosswalk();
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: " << crosswalk_overlap->object_id;
      }
    }
  }
}

bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::CheckStopForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const CrosswalkInfoConstPtr crosswalk_ptr, const Obstacle& obstacle,
    const double stop_deceleration) {
  CHECK_NOTNULL(reference_line_info);

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

  // Now car obstacle need to make crosswalk decision
  // if (CheckObstacleType(obstacle_type, obstacle.Id(), obstacle_type_name)) {
  //   return false;
  // }

  if (ExpandCrosswalkPolygon(reference_line_info, obstacle, crosswalk_ptr)) {
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();

  common::SLPoint obstacle_sl_point;
  reference_line.XYToSL(perception_obstacle.position(), &obstacle_sl_point);
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle_sl_boundary.start_l()),
               std::fabs(obstacle_sl_boundary.end_l()));

  const bool is_on_lane = reference_line.IsOnLane(obstacle_sl_boundary);
  const bool is_on_road = reference_line.IsOnRoad(obstacle_sl_boundary);
  const bool is_path_cross = !obstacle.reference_line_st_boundary().IsEmpty();
  const bool yield_pedestrian =
      PerceptionObstacle::PEDESTRIAN == obstacle_type && obstacle.IsStatic();

  ADEBUG << "obstacle_id[" << obstacle.Id() << "] type[" << obstacle_type_name
         << "] crosswalk_id[" << crosswalk_ptr->id().id() << "] obstacle_l["
         << obstacle_sl_point.l() << "] obstacle_l_distance["
         << obstacle_l_distance << "] is_on_lane[" << is_on_lane
         << "] is_on_road[" << is_on_road << "] is_path_cross[" << is_path_cross
         << "]";

  if (adc_sl_boundary.end_s() >=
      obstacle_sl_boundary.end_s() + kLongitudeBuffer) {
    ADEBUG << "skip: obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] adc_sl_boundary.end_s()["
           << adc_sl_boundary.end_s() << "] obstacle_sl_boundary.end_s()["
           << obstacle_sl_boundary.end_s()
           << "]: adc front edge has exceeded obstacle point.";
    return false;
  }

  // check obstacle direction, Filter out obstacles that have no impact
  if (!yield_pedestrian &&
      !CheckObstacleDirection(reference_line_info, obstacle)) {
    return false;
  }

  bool stop = false;
  if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
    //     STOP only if paths crosses
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle.Id() << "] type["
             << obstacle_type_name << "] crosswalk_id["
             << crosswalk_ptr->id().id() << "]";
    }
  } else if (obstacle_l_distance <=
             config_.crosswalk().stop_strict_l_distance()) {
    if ((is_on_road && is_path_cross) || yield_pedestrian) {
      // (2) when l_distance <= strict_l_distance + on_road
      //     always STOP
      if (obstacle_sl_point.s() > adc_sl_boundary.end_s()) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle.Id() << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_sl_boundary.end_s()[ " << adc_sl_boundary.end_s()
               << "] crosswalk_id[" << crosswalk_ptr->id().id() << "] ON_ROAD";
      }
    } else {
      // (3) when l_distance <= strict_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle.Id() << "] type["
               << obstacle_type_name << "] crosswalk_id["
               << crosswalk_ptr->id().id() << "] PATH_CRSOSS";
      } else {
        // (4) when l_distance <= strict_l_distance
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        CheckStopForPedestrian(obstacle, crosswalk_ptr->id().id(), &stop);
      }
    }
  } else {
    // (4) when l_distance is between loose_l and strict_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle.Id()
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id["
           << crosswalk_ptr->id().id() << "] USE_PREVIOUS_DECISION";
  }

  CheckStopDeceleration(stop_deceleration, obstacle_l_distance,
                        crosswalk_ptr->id().id(), &stop);

  return stop;
}

bool Crosswalk::CheckObstacleType(const PerceptionObstacle::Type& obstacle_type,
                                  const std::string& obstacle_id,
                                  const std::string& obstacle_type_name) {
  // check type
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return true;
  }

  return false;
}

bool Crosswalk::ExpandCrosswalkPolygon(
    ReferenceLineInfo* const reference_line_info, const Obstacle& obstacle,
    const hdmap::CrosswalkInfoConstPtr crosswalk_ptr) {
  // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  const auto& obs_sl_boundary = obstacle.PerceptionSLBoundary();

  SLBoundary crosswalk_sl_boundary;
  if (!reference_line_info->reference_line().GetSLBoundary(
          crosswalk_ptr->crosswalk().polygon(), &crosswalk_sl_boundary)) {
    AERROR << "Failed get crosswalk sl boundary.";
    return true;
  }

  bool in_expanded_crosswalk =
      (!(obs_sl_boundary.start_l() > crosswalk_sl_boundary.end_l() ||
         obs_sl_boundary.end_l() < crosswalk_sl_boundary.start_l())) &&
      (!(obs_sl_boundary.end_s() <
             crosswalk_sl_boundary.start_s() -
                 config_.crosswalk().expand_s_distance() ||
         obs_sl_boundary.start_s() >
             crosswalk_sl_boundary.end_s() +
                 config_.crosswalk().expand_s_distance()));

  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] crosswalk_id["
           << crosswalk_ptr->id().id() << "]: not in crosswalk expanded area";
    return true;
  }

  return false;
}

void Crosswalk::CheckStopForPedestrian(const Obstacle& obstacle,
                                       const std::string& crosswalk_id,
                                       bool* const stop) {
  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  auto obstacle_v = Vec2d(0.0, 0.0);
  if (perception_obstacle.has_velocity()) {
    if (obstacle.speed() != 0.0) {
      if (!(std::isnan(perception_obstacle.velocity().x()) ||
            std::isnan(perception_obstacle.velocity().y()))) {
        obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                           perception_obstacle.velocity().y());
      }
    }
  }
  const auto adc_path_point =
      Vec2d(injector_->ego_info()->start_point().path_point().x(),
            injector_->ego_info()->start_point().path_point().y());
  const auto obstacle_position = Vec2d(perception_obstacle.position().x(),
                                       perception_obstacle.position().y());
  auto obs_to_adc = adc_path_point - obstacle_position;
  const double kEpsilon = 1e-6;
  if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
    *stop = true;
    ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "] MOVING_TOWARD_ADC";
  }
}

void Crosswalk::CheckStopDeceleration(const double stop_deceleration,
                                      const double obstacle_l_distance,
                                      const std::string& crosswalk_id,
                                      bool* const stop) {
  // check stop_deceleration
  if (*stop) {
    if (stop_deceleration >= config_.crosswalk().max_stop_deceleration()) {
      if (obstacle_l_distance > config_.crosswalk().stop_strict_l_distance()) {
        // SKIP when stop_deceleration is too big but safe to ignore
        *stop = false;
      }
      AWARN << "crosswalk_id[" << crosswalk_id << "] stop_deceleration["
            << stop_deceleration << "]";
    }
  }
}

bool Crosswalk::CheckObstacleDirection(
    ReferenceLineInfo* const reference_line_info, const Obstacle& obstacle) {
  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

  auto moving_type = util::GetMovingObstacleType(
      &obstacle, reference_line_info->vehicle_state(),
      reference_line_info->reference_line());

  if (util::LEFT_CROSSING != moving_type &&
      util::RIGHT_CROSSING != moving_type) {
    ADEBUG << "skip: obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] moving_type["
           << util::GetMovingObstacleTypeName(moving_type)
           << "]: non crossing behavior.";
    return false;
  }

  if (util::LEFT_CROSSING == moving_type &&
      adc_sl_boundary.start_l() >
          obstacle_sl_boundary.end_l() + kLateralBuffer) {
    ADEBUG << "skip: obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] moving_type["
           << util::GetMovingObstacleTypeName(moving_type)
           << "] adc_sl_boundary.start_l[" << adc_sl_boundary.start_l()
           << "] obstacle_sl_boundary.end_l[" << obstacle_sl_boundary.end_l()
           << "]: obstacle is left crossing behavior, but has moved away from "
              "adc.";
    return false;
  }

  if (util::RIGHT_CROSSING == moving_type &&
      obstacle_sl_boundary.start_l() - kLateralBuffer >
          adc_sl_boundary.end_l()) {
    ADEBUG << "skip: obstacle_id[" << obstacle.Id() << "] type["
           << obstacle_type_name << "] moving_type["
           << util::GetMovingObstacleTypeName(moving_type)
           << "] adc_sl_boundary.end_l[" << adc_sl_boundary.end_l()
           << "] obstacle_sl_boundary.start_l["
           << obstacle_sl_boundary.start_l()
           << "]: obstacle is right crossing behavior, but has moved away from "
              "adc.";
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace century
