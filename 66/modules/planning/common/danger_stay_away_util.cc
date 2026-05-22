/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/planning/common/danger_stay_away_util.h"

#include <cmath>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace century {
namespace planning {
namespace {

using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::perception::PerceptionObstacle;

constexpr double kDangerLonBuffer = 3.0;
constexpr double kDangerLatBuffer = 1.5;
constexpr double kUnknownMovingSpeed = 0.1;
constexpr double kIgvLength = 15.0;
constexpr double kIgvWidth = 3.2;
constexpr double kStackerRealLength = 8.1;
constexpr double kStackerRealWidth = 4.4;
constexpr double kStackerFrontToLocPoint = 0.8;

std::string NormalizeStackerId(std::string stacker_id) {
  const std::vector<std::string> prefixes = {"stacker_", "wheelcrane_",
                                             "wheel_crane_"};
  for (const auto& prefix : prefixes) {
    if (stacker_id.find(prefix) == 0) {
      stacker_id.erase(0, prefix.size());
      break;
    }
  }
  return stacker_id;
}

bool IsOperationTaskType(const routing::TaskType task_type) {
  return routing::RAILWAY_OPERATIONAREA_DYNAMIC == task_type ||
         routing::YARD_OPERATIONAREA_DYNAMIC == task_type ||
         routing::TINY_ADJUSTMENT_FRONT == task_type ||
         routing::TINY_ADJUSTMENT_BACK == task_type ||
         routing::TINY_ADJUSTMENT_RIGHT == task_type ||
         routing::TINY_ADJUSTMENT_LEFT == task_type ||
         routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 == task_type ||
         routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 == task_type ||
         routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 == task_type ||
         routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 == task_type;
}

bool IsCurrentOperationStacker(const LocalView& local_view,
                               const std::string& stacker_id) {
  if (nullptr == local_view.routing) {
    return false;
  }
  const auto& routing_request = local_view.routing->routing_request();
  if (!IsOperationTaskType(routing_request.task_type()) ||
      routing_request.operation_stacker_id().empty()) {
    return false;
  }
  return NormalizeStackerId(stacker_id) ==
         NormalizeStackerId(routing_request.operation_stacker_id());
}

Box2d BuildEgoDangerBox(const common::VehicleState& vehicle_state) {
  return VehicleConfigHelper::GetBoundingBox(
      vehicle_state.x(), vehicle_state.y(), vehicle_state.heading(),
      kDangerLonBuffer, kDangerLatBuffer);
}

Box2d BuildIgvBox(const VehicleInfo& vehicle_info) {
  return Box2d({vehicle_info.x(), vehicle_info.y()}, vehicle_info.heading(),
               kIgvLength, kIgvWidth);
}

Box2d BuildStackerBox(const StackerInfo& stacker_info) {
  const auto& point = stacker_info.stacker_point();
  const double heading = point.heading();
  Box2d stacker_box({point.x(), point.y()}, heading, kStackerRealLength,
                    kStackerRealWidth);
  const double shift_distance =
      kStackerRealLength * 0.5 - kStackerFrontToLocPoint;
  const Vec2d shift_vec{-shift_distance * std::cos(heading),
                        -shift_distance * std::sin(heading)};
  stacker_box.Shift(shift_vec);
  return stacker_box;
}

bool BuildObstaclePolygon(const PerceptionObstacle& obstacle,
                          Polygon2d* const polygon) {
  if (obstacle.polygon_point_size() >= 3) {
    std::vector<Vec2d> points;
    points.reserve(obstacle.polygon_point_size());
    for (const auto& point : obstacle.polygon_point()) {
      points.emplace_back(point.x(), point.y());
    }
    *polygon = Polygon2d(points);
    return true;
  }
  if (obstacle.has_position() && obstacle.length() > 0.0 &&
      obstacle.width() > 0.0) {
    Box2d box({obstacle.position().x(), obstacle.position().y()},
              obstacle.theta(), obstacle.length(), obstacle.width());
    *polygon = Polygon2d(box);
    return true;
  }
  return false;
}

bool ObstacleHasOverlapWithBox(const PerceptionObstacle& obstacle,
                               const Box2d& box) {
  Polygon2d obstacle_polygon;
  const Polygon2d box_polygon(box);
  if (BuildObstaclePolygon(obstacle, &obstacle_polygon)) {
    return box_polygon.HasOverlap(obstacle_polygon);
  }
  if (obstacle.has_position()) {
    return box.IsPointIn({obstacle.position().x(), obstacle.position().y()});
  }
  return false;
}

bool IsVehicleMatchedWithIgv(const PerceptionObstacle& obstacle,
                             const LocalView& local_view) {
  if (nullptr == local_view.v2x_info) {
    return false;
  }
  for (const auto& vehicle_info : local_view.v2x_info->vehicle_info()) {
    if (!local_view.v2x_info->ego_id().empty() &&
        vehicle_info.id() == local_view.v2x_info->ego_id()) {
      continue;
    }
    if (ObstacleHasOverlapWithBox(obstacle, BuildIgvBox(vehicle_info))) {
      AINFO << "DangerStayAway skip IGV vehicle, obstacle_id="
            << obstacle.id() << ", igv_id=" << vehicle_info.id();
      return true;
    }
  }
  return false;
}

bool FindMatchedStackerInfo(const PerceptionObstacle& obstacle,
                            const LocalView& local_view,
                            StackerInfo* const matched_stacker_info) {
  if (nullptr == local_view.stackers_info) {
    return false;
  }
  for (const auto& stacker_info : local_view.stackers_info->stacker_info()) {
    if (stacker_info.stacker_type() != StackerType::STACKER &&
        stacker_info.stacker_type() != StackerType::FORKLIFT_STACKER) {
      continue;
    }
    if (ObstacleHasOverlapWithBox(obstacle, BuildStackerBox(stacker_info))) {
      matched_stacker_info->CopyFrom(stacker_info);
      return true;
    }
  }
  return false;
}

bool IsDangerObstacleType(const PerceptionObstacle& obstacle,
                          const LocalView& local_view) {
  switch (obstacle.type()) {
    case PerceptionObstacle::PEDESTRIAN:
      return true;
    case PerceptionObstacle::UNKNOWN:
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      return obstacle.has_velocity() &&
             std::hypot(obstacle.velocity().x(), obstacle.velocity().y()) >
                 kUnknownMovingSpeed;
    case PerceptionObstacle::VEHICLE:
      return !IsVehicleMatchedWithIgv(obstacle, local_view);
    case PerceptionObstacle::STACKER:
    case PerceptionObstacle::FORKLIFT_STACKER: {
      StackerInfo matched_stacker_info;
      const bool matched_stacker =
          FindMatchedStackerInfo(obstacle, local_view, &matched_stacker_info);
      if (matched_stacker &&
          IsCurrentOperationStacker(local_view,
                                    matched_stacker_info.stacker_id())) {
        AINFO << "DangerStayAway skip current operation stacker, obstacle_id="
              << obstacle.id() << ", stacker_id="
              << matched_stacker_info.stacker_id();
        return false;
      }
      return true;
    }
    default:
      return false;
  }
}

}  // namespace

bool NeedDangerStayAwayDisplay(const LocalView& local_view,
                               const common::VehicleState& vehicle_state) {
  if (nullptr == local_view.prediction_obstacles) {
    return false;
  }
  const Box2d danger_box = BuildEgoDangerBox(vehicle_state);
  for (const auto& prediction_obstacle :
       local_view.prediction_obstacles->prediction_obstacle()) {
    if (!prediction_obstacle.has_perception_obstacle()) {
      continue;
    }
    const auto& obstacle = prediction_obstacle.perception_obstacle();
    if (!IsDangerObstacleType(obstacle, local_view)) {
      continue;
    }
    if (ObstacleHasOverlapWithBox(obstacle, danger_box)) {
      AINFO << "DangerStayAway display triggered by obstacle_id="
            << obstacle.id() << ", type=" << obstacle.type();
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace century
