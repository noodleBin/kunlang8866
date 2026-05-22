/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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
#include "modules/planning/common/util/math_util.h"

#include <utility>

#include "modules/common/math/math_utils.h"

namespace century {
namespace planning {
namespace util {

std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle) {
  double x_diff = input_world_coord.first - obj_world_coord.first;
  double y_diff = input_world_coord.second - obj_world_coord.second;
  double rho = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  double theta = std::atan2(y_diff, x_diff) - obj_world_angle;

  return std::make_pair(std::cos(theta) * rho, std::sin(theta) * rho);
}

double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle) {
  return common::math::NormalizeAngle(input_world_angle - obj_world_angle);
}

std::vector<common::math::Vec2d> ConvexHull(
    std::vector<common::math::Vec2d> points) {
  if (points.size() <= 1) {
    return points;
  }

  std::sort(points.begin(), points.end(),
            [](const common::math::Vec2d& a, const common::math::Vec2d& b) {
              return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
            });

  std::vector<common::math::Vec2d> hull;
  // upper convex hull
  for (const auto& p : points) {
    while (hull.size() >= 2) {
      const common::math::Vec2d& a = hull[hull.size() - 2];
      const common::math::Vec2d& b = hull[hull.size() - 1];
      if ((b.x() - a.x()) * (p.y() - a.y()) -
              (b.y() - a.y()) * (p.x() - a.x()) <=
          0)
        break;
      hull.pop_back();
    }
    hull.emplace_back(p);
  }

  // upper conver hull
  size_t lower_size = hull.size();
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    const auto& p = *it;
    while (hull.size() > lower_size) {
      const common::math::Vec2d& a = hull[hull.size() - 2];
      const common::math::Vec2d& b = hull[hull.size() - 1];
      if ((b.x() - a.x()) * (p.y() - a.y()) -
              (b.y() - a.y()) * (p.x() - a.x()) <=
          0)
        break;
      hull.pop_back();
    }
    hull.emplace_back(p);
  }

  hull.pop_back();
  return hull;
}

bool FindLineIntersection(double x1, double y1, double heading1, double x2,
                          double y2, double heading2, double& out_x,
                          double& out_y, const bool is_heading_rad) {
  constexpr double EPSILON = 1e-9;
  double heading1_rad = heading1;
  double heading2_rad = heading2;
  if (!is_heading_rad) {
    heading1_rad = heading1 * M_PI / 180.0;
    heading2_rad = heading2 * M_PI / 180.0;
  }

  // compute unit vector
  double va_x = std::cos(heading1_rad);
  double va_y = std::sin(heading1_rad);
  double vb_x = std::cos(heading2_rad);
  double vb_y = std::sin(heading2_rad);
  // calculate vector error: Y = b - a
  double Yx = x2 - x1;
  double Yy = y2 - y1;
  // calculate determinant: m = -va_x * vb_y + va_y * vb_x
  double m = -va_x * vb_y + va_y * vb_x;
  // Check if the lines are parallel or overlapping
  if (std::fabs(m) < EPSILON) {
    return false;
  }
  // Using Kramer's Law to Solve Parameters t
  double t = (vb_x * Yy - vb_y * Yx) / m;

  // Calculate intersection coordinates: P = a + t * va
  out_x = x1 + t * va_x;
  out_y = y1 + t * va_y;
  return true;
}

}  // namespace util
}  // namespace planning
}  // namespace century
