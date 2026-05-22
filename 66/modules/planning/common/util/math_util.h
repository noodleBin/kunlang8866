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
#pragma once

#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"

namespace century {
namespace planning {
namespace util {

// Helper function to convert world coordinates to relative coordinates
// around the obstacle of interest.
std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle);

double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle);

std::vector<century::common::math::Vec2d> ConvexHull(
    std::vector<common::math::Vec2d> points);

/**
 * @brief calculate the intersection of two lines
 * @param x1 first point x
 * @param y1 first point y
 * @param heading1_deg first point heading degree
 * @param x2 second point x
 * @param y2 second point y
 * @param heading2_deg second point heading degree
 * @param is_heading_rad is heading radian
 * @param[out] out_x intersection x
 * @param[out] out_y intersection y
 * @return true ,the two lines intersect and the intersection exists
 * @return false two lines parallel or overlap
 */
bool FindLineIntersection(double x1, double y1, double heading1_deg, double x2,
                          double y2, double heading2_deg, double& out_x,
                          double& out_y, const bool is_heading_rad = true);

}  // namespace util
}  // namespace planning
}  // namespace century
