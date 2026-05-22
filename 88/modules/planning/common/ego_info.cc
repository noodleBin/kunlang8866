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
 * @file speed_limit.cc
 **/

#include "modules/planning/common/ego_info.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace century {
namespace planning {

using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;

EgoInfo::EgoInfo() {
  ego_vehicle_config_ = common::VehicleConfigHelper::GetConfig();
}

bool EgoInfo::Update(const common::TrajectoryPoint& start_point,
                     const common::VehicleState& vehicle_state) {
  set_start_point(start_point);
  set_vehicle_state(vehicle_state);
  CalculateEgoBox(vehicle_state);
  return true;
}

void EgoInfo::CalculateEgoBox(const common::VehicleState& vehicle_state) {
  const auto& param = ego_vehicle_config_.vehicle_param();
  ADEBUG << "param: " << param.DebugString();

  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) * 0.5,
      (param.left_edge_to_center() - param.right_edge_to_center()) * 0.5);

  Vec2d position(vehicle_state.x(), vehicle_state.y());
  Vec2d center(position + vec_to_center.rotate(vehicle_state.heading()));

  ego_box_ =
      Box2d(center, vehicle_state.heading(), param.length(), param.width());
}

void EgoInfo::Clear() {
  start_point_.Clear();
  vehicle_state_.Clear();
  front_clear_distance_ = FLAGS_default_front_clear_distance;
}

// TODO(all): remove this function and "front_clear_distance" related.
// It doesn't make sense when:
// 1. the heading is not necessaries align with the road
// 2. the road is not necessaries straight
void EgoInfo::CalculateFrontObstacleClearDistance(
    const std::vector<const Obstacle*>& obstacles) {
  Vec2d position(vehicle_state_.x(), vehicle_state_.y());

  const auto& param = ego_vehicle_config_.vehicle_param();
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) * 0.5,
      (param.left_edge_to_center() - param.right_edge_to_center()) * 0.5);

  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading()));

  Vec2d unit_vec_heading = Vec2d::CreateUnitVec2d(vehicle_state_.heading());

  // Due to the error of ego heading, only short range distance is meaningful
  static constexpr double kDistanceThreshold = 50.0;
  static constexpr double buffer = 0.1;  // in meters
  const double impact_region_length =
      param.length() + buffer + kDistanceThreshold;
  Box2d ego_front_region(center + unit_vec_heading * kDistanceThreshold * 0.5,
                         vehicle_state_.heading(), impact_region_length,
                         param.width() + buffer);

  for (const auto& obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    double dist = 0.0;
    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type) {
      const auto& ego_front_polygon = Polygon2d(ego_front_region);
      const auto& obs_polygon = obstacle->PerceptionPolygon();
      if (!ego_front_polygon.HasOverlap(obs_polygon)) {
        continue;
      }
      dist = obs_polygon.DistanceTo(ego_front_region);
    } else {
      if (!ego_front_region.HasOverlap(obstacle->PerceptionBoundingBox())) {
        continue;
      }
      dist = ego_box_.center().DistanceTo(
                 obstacle->PerceptionBoundingBox().center()) -
             ego_box_.diagonal() * 0.5;
    }

    if (front_clear_distance_ < 0.0 || dist < front_clear_distance_) {
      front_clear_distance_ = dist;
    }
  }
}

}  // namespace planning
}  // namespace century
