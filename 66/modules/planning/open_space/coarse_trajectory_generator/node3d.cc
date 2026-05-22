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

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

#include "absl/strings/str_cat.h"

namespace century {
namespace planning {

using century::common::math::Box2d;

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double>& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  traversed_x_.emplace_back(x);
  traversed_y_.emplace_back(y);
  traversed_phi_.emplace_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  int_index_ = CalculateIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(const std::vector<double>& traversed_x,
               const std::vector<double>& traversed_y,
               const std::vector<double>& traversed_phi,
               const std::vector<double>& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();
  CHECK_EQ(traversed_x.size(), traversed_y.size());
  CHECK_EQ(traversed_x.size(), traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  int_index_ = CalculateIndex(x_grid_, y_grid_, phi_grid_);
  step_size_ = traversed_x.size();
}

void Node3d::SetTraversed(const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          const std::vector<double>& phis,
                          const std::vector<double>& XYbounds,
                          const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(xs.size(), ys.size());
  CHECK_EQ(xs.size(), phis.size());
  // update traversed_x_, traversed_y_, traversed_phi_
  traversed_x_ = xs;
  traversed_y_ = ys;
  traversed_phi_ = phis;

  // update x_, y_, phi_
  x_ = xs.back();
  y_ = ys.back();
  phi_ = phis.back();

  // update x_grid_, y_grid_, phi_grid_
  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  // update step_size_
  step_size_ = xs.size();
}

Box2d Node3d::GetBoundingBox(const common::VehicleParam& vehicle_param_,
                             const double x, const double y, const double phi) {
  double ego_length = vehicle_param_.length();
  double ego_width = vehicle_param_.width();
  double shift_distance =
      ego_length * 0.5 - vehicle_param_.back_edge_to_center();
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

Box2d Node3d::GetBoundingBoxWithBuffer(
    const common::VehicleParam& vehicle_param_, const double x, const double y,
    const double phi, const double extra_length, const double extra_width) {
  double ego_length = vehicle_param_.length() + extra_length;
  double ego_width = vehicle_param_.width() + extra_width;
  double shift_distance =
      ego_length * 0.5 - vehicle_param_.back_edge_to_center();
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

Box2d Node3d::GetObsCostBoxWithBuffer(
    const common::VehicleParam& vehicle_param_, const double x, const double y,
    const double phi, const double extra_length, const double extra_width) {
  double scan_length = vehicle_param_.length() + extra_length * 2;
  double scan_width = (vehicle_param_.width() * 0.5) + extra_width;
  double shift_distance =
      scan_length * 0.5 - vehicle_param_.back_edge_to_center();
  Box2d scan_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, scan_length, scan_width);
  return scan_box;
}

double Node3d::DistanceTo(const Node3d& other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Node3d::DistanceSquareTo(const Node3d& other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

bool Node3d::operator==(const Node3d& right) const {
  return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  return absl::StrCat(x_grid, "_", y_grid, "_", phi_grid);
}

uint64_t Node3d::CalculateIndex(int x_grid, int y_grid, int phi_grid) {
  uint64_t x_grid_temp = std::max(0, std::min(65535, x_grid));
  uint64_t y_grid_temp = std::max(0, std::min(65535, y_grid));
  uint64_t phi_grid_temp = std::max(0, std::min(65535, phi_grid));

  return (static_cast<uint64_t>(x_grid_temp) << 32) |
         (static_cast<uint64_t>(y_grid_temp) << 16) |
         static_cast<uint64_t>(phi_grid_temp);
}

std::string Node3d::GetIndexString() const {
  int x_grid = static_cast<int>((int_index_ >> 32) & 0xFFFF);
  int y_grid = static_cast<int>((int_index_ >> 16) & 0xFFFF);
  int phi_grid = static_cast<int>(int_index_ & 0xFFFF);
  return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
         std::to_string(phi_grid);
}

}  // namespace planning
}  // namespace century
