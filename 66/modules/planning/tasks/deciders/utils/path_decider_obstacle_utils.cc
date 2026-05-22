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

#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

#include <limits>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
static HysteresisInterval obstacles_interval(
    FLAGS_static_obstacle_speed_threshold,
    FLAGS_static_obstacle_speed_hysteresis_relative_lower_limit,
    FLAGS_static_obstacle_speed_hysteresis_relative_upper_limit, 100UL);
static std::mutex hyspeed_mutex;

namespace {
constexpr double kMinObstacleArea = 1e-4;
constexpr double kMaxIgnoreObstacleArea = 50.0;
}  // namespace

bool IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }
  // Obstacle should not be moving obstacle.
  double hy_speed = 0.0;
  {
    std::lock_guard<std::mutex> lock(hyspeed_mutex);
    hy_speed = obstacles_interval.HyValue(obstacle, obstacle.speed());
  }
  if (hy_speed > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }
  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

bool ObstacleSizeUnNormal(const Obstacle& obstacle) {
  const auto& obstacle_sl = obstacle.PerceptionSLBoundary();
  if (((obstacle_sl.end_s() - obstacle_sl.start_s()) *
           (obstacle_sl.end_l() - obstacle_sl.start_l()) <
       kMinObstacleArea) ||
      ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
           (obstacle_sl.end_l() - obstacle_sl.start_l()) >
       kMaxIgnoreObstacleArea)) {
    return true;
  }
  return false;
}

bool ComputeSLBoundaryIntersection(const SLBoundary& sl_boundary,
                                   const double s, double* ptr_l_min,
                                   double* ptr_l_max) {
  *ptr_l_min = std::numeric_limits<double>::max();
  *ptr_l_max = -std::numeric_limits<double>::max();

  // invalid polygon
  if (sl_boundary.boundary_point_size() < 3) {
    return false;
  }

  bool has_intersection = false;
  for (auto i = 0; i < sl_boundary.boundary_point_size(); ++i) {
    auto j = (i + 1) % sl_boundary.boundary_point_size();
    const auto& p0 = sl_boundary.boundary_point(i);
    const auto& p1 = sl_boundary.boundary_point(j);

    if (common::util::WithinBound<double>(std::fmin(p0.s(), p1.s()),
                                          std::fmax(p0.s(), p1.s()), s)) {
      has_intersection = true;
      auto l = common::math::lerp<double>(p0.l(), p0.s(), p1.l(), p1.s(), s);
      if (l < *ptr_l_min) {
        *ptr_l_min = l;
      }
      if (l > *ptr_l_max) {
        *ptr_l_max = l;
      }
    }
  }
  return has_intersection;
}

double InterpolationLookUp(const double x, const double x_lower,
                           const double x_upper, const double y_lower,
                           const double y_upper) {
  if (std::fabs(x_upper - x_lower) <= 1.0e-6) {
    AERROR << "input x axis diff is too small";
    return y_lower;
  }
  if (x <= x_lower) {
    return y_lower;
  } else if (x >= x_upper) {
    return y_upper;
  } else {
    return y_lower + (x - x_lower) * (y_upper - y_lower) / (x_upper - x_lower);
  }
}

}  // namespace planning
}  // namespace century
