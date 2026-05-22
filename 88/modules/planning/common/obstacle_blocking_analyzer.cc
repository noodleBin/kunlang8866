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

#include "modules/planning/common/obstacle_blocking_analyzer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

using century::common::VehicleConfigHelper;
using century::hdmap::HDMapUtil;
using century::perception::PerceptionObstacle;

constexpr double kAdcDistanceThreshold = 35.0;  // unit: m
constexpr double kObstaclesDistanceThreshold = 15.0;

bool IsNonmovableObstacle(const ReferenceLineInfo& reference_line_info,
                          const Obstacle& obstacle) {
  ADEBUG << "obstacle = " << obstacle.Id();
  bool is_tire_lifter = obstacle.Perception().type() ==
                        perception::PerceptionObstacle::WHEELCRANE;
  double consider_length = kAdcDistanceThreshold;
  ADEBUG << "is_tire_lifter = " << is_tire_lifter;
  if (is_tire_lifter) {
    consider_length = FLAGS_wheelcrane_consider_distance;
  }
  ADEBUG<<"consider_length = "<<consider_length;
  // Obstacle is far away.
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  if (obstacle.PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + consider_length) {
    ADEBUG << " - It is too far ahead and we are not so sure of its status.";
    return false;
  }

  // Obstacle is parked obstacle.
  if (IsParkedVehicle(reference_line_info.reference_line(), &obstacle)) {
    ADEBUG << "It is Parked and NON-MOVABLE.";
    return true;
  }

  if (FLAGS_consider_obstacle_blocked) {
    // Obstacle is blocked by others too.
    for (const auto* other_obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (other_obstacle->Id() == obstacle.Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle.PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle.PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle.PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kObstaclesDistanceThreshold) {
        continue;
      }

      // TODO(All): Fix the segmentation bug for large vehicles, otherwise
      // the follow line will be problematic.
      ADEBUG << " - It is blocked by others, and will move later.";
      return false;
    }
  }

  ADEBUG << "IT IS NON-MOVABLE!";
  return true;
}

bool IsNearJunctionObstacleCanPass(const ReferenceLineInfo& reference_line_info,
                                   const Obstacle& obstacle,
                                   double junction_start_s) {
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  if (0 &&
      (obs_sl.start_l() < adc_center_l - FLAGS_block_adc_center_view_buffer &&
       obs_sl.end_l() > adc_center_l + FLAGS_block_adc_center_view_buffer)) {
    ADEBUG << "near junction obstacle block the view of adc, not sidepass";
    return false;
  }

  std::vector<const Obstacle*> risk_obstacles;
  for (const auto* other_obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (other_obstacle->Id() == obstacle.Id()) {
      continue;
    }
    if (other_obstacle->IsVirtual() || !other_obstacle->IsStatic()) {
      continue;
    }
    if (PerceptionObstacle::VEHICLE != other_obstacle->Perception().type()) {
      continue;
    }
    const auto& other_obs_sl = other_obstacle->PerceptionSLBoundary();
    if (junction_start_s < other_obs_sl.start_s() ||
        other_obs_sl.start_s() < adc_sl.end_s()) {
      continue;
    }
    common::SLPoint sl_point;
    sl_point.set_l((other_obs_sl.start_l() + other_obs_sl.end_l()) * 0.5);
    sl_point.set_s((other_obs_sl.start_s() + other_obs_sl.end_s()) * 0.5);
    if (!reference_line_info.reference_line().IsOnLane(sl_point)) {
      continue;
    }
    risk_obstacles.emplace_back(other_obstacle);
    ADEBUG << "near junction other obstacle: " << other_obstacle->Id()
           << " in lane before blocking obs, not sidepass";
    return false;
  }
  if (risk_obstacles.empty()) {
    return true;
  }

  double min_start_l = std::numeric_limits<double>::max();
  double max_end_l = std::numeric_limits<double>::lowest();
  for (const auto* obs : risk_obstacles) {
    const auto& obs_sl = obs->PerceptionSLBoundary();
    min_start_l = std::fmin(min_start_l, obs_sl.start_l());
    max_end_l = std::fmax(max_end_l, obs_sl.end_l());

    if (min_start_l < -FLAGS_block_adc_center_view_buffer &&
        max_end_l > FLAGS_block_adc_center_view_buffer) {
      ADEBUG << "near junction other obstacle: " << obs->Id()
             << " take the center of lane, not sidepass"
             << " min_start_l: " << min_start_l << " max_end_l: " << max_end_l;
      return false;
    }
  }

  return true;
}

// This is the side-pass condition for every obstacle.
// TODO(all): if possible, transform as many function parameters into GFLAGS.
bool IsBlockingObstacleToSidePass(const Frame& frame, const Obstacle* obstacle,
                                  double block_obstacle_min_speed,
                                  double min_front_sidepass_distance,
                                  bool enable_obstacle_blocked_check) {
  // Get the necessary info.
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const PathDecision& path_decision = reference_line_info.path_decision();
  ADEBUG << "Evaluating Obstacle: " << obstacle->Id();

  // Obstacle is virtual.
  if (obstacle->IsVirtual()) {
    ADEBUG << " - It is virtual.";
    return false;
  }

  // Obstacle is moving.
  if (!obstacle->IsStatic() || obstacle->speed() > block_obstacle_min_speed) {
    ADEBUG << " - It is non-static.";
    return false;
  }

  // Obstacle is behind ADC.
  if (obstacle->PerceptionSLBoundary().start_s() <= adc_sl_boundary.end_s()) {
    ADEBUG << " - It is behind ADC.";
    return false;
  }

  // Obstacle is far away.
  static constexpr double kAdcDistanceSidePassThreshold = 15.0;
  if (obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceSidePassThreshold) {
    ADEBUG << " - It is too far ahead.";
    return false;
  }

  // Obstacle is too close.
  if (adc_sl_boundary.end_s() + min_front_sidepass_distance >
      obstacle->PerceptionSLBoundary().start_s()) {
    ADEBUG << " - It is too close to side-pass.";
    return false;
  }

  // Obstacle is not blocking our path.
  if (!IsBlockingDrivingPathObstacle(reference_line, obstacle)) {
    ADEBUG << " - It is not blocking our way.";
    return false;
  }

  // Obstacle is blocked by others too.
  if (enable_obstacle_blocked_check &&
      !IsParkedVehicle(reference_line, obstacle)) {
    for (const auto* other_obstacle : path_decision.obstacles().Items()) {
      if (other_obstacle->Id() == obstacle->Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      }

      // TODO(All): Fix the segmentation bug for large vehicles, otherwise
      // the follow line will be problematic.
      ADEBUG << " - It is blocked by others, too.";
      return false;
    }
  }

  ADEBUG << "IT IS BLOCKING!";
  return true;
}

double GetDistanceBetweenADCAndObstacle(const Frame& frame,
                                        const Obstacle* obstacle) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  double distance_between_adc_and_obstacle =
      obstacle->PerceptionSLBoundary().start_s() - adc_sl_boundary.end_s();
  return distance_between_adc_and_obstacle;
}

bool IsBlockingDrivingPathObstacle(const ReferenceLine& reference_line,
                                   const Obstacle* obstacle) {
  const double driving_width =
      reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  ADEBUG << " (driving width = " << driving_width
         << ", adc_width = " << adc_width << ")";
  if (driving_width > adc_width + FLAGS_static_obstacle_nudge_l_buffer +
                          FLAGS_side_pass_driving_width_l_buffer) {
    // TODO(jiacheng): make this a GFLAG:
    // side_pass_context_.scenario_config_.min_l_nudge_buffer()
    ADEBUG << "It is NOT blocking our path.";
    return false;
  }

  ADEBUG << "It is blocking our path.";
  return true;
}

bool IsParkedVehicle(const ReferenceLine& reference_line,
                     const Obstacle* obstacle) {
  if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  double max_road_right_width = 0.0;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = road_right_width;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = std::max(max_road_right_width, road_right_width);
  bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                         max_road_right_width - 0.1;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  auto obstacle_box = obstacle->PerceptionBoundingBox();
  HDMapUtil::BaseMapPtr()->GetLanes(
      common::util::PointFactory::ToPointENU(obstacle_box.center().x(),
                                             obstacle_box.center().y()),
      std::min(obstacle_box.width(), obstacle_box.length()), &lanes);
  bool is_on_parking_lane = false;
  if (lanes.size() == 1 &&
      lanes.front()->lane().type() == century::hdmap::Lane::PARKING) {
    is_on_parking_lane = true;
  }

  bool is_parked = is_on_parking_lane || is_at_road_edge;
  return is_parked && obstacle->IsStatic();
}

}  // namespace planning
}  // namespace century
