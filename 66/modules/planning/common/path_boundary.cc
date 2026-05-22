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

#include "modules/planning/common/path_boundary.h"

namespace century {
namespace planning {

PathBoundary::PathBoundary(const double start_s, const double delta_s,
                           std::vector<std::pair<double, double>> path_boundary)
    : start_s_(start_s),
      delta_s_(delta_s),
      boundary_(std::move(path_boundary)) {}

double PathBoundary::start_s() const { return start_s_; }

double PathBoundary::delta_s() const { return delta_s_; }

void PathBoundary::set_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& PathBoundary::boundary() const {
  return boundary_;
}

void PathBoundary::set_label(const std::string& label) { label_ = label; }

const std::string& PathBoundary::label() const { return label_; }

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
  blocking_obstacle_id_ = obs_id;
}

const std::string& PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

std::pair<double, double> PathBoundary::GetBoundLimit() const {
  std::pair<double, double> bound_limit;
  double max_start_l = std::numeric_limits<double>::lowest();
  double min_end_l = std::numeric_limits<double>::max();
  for (size_t i = 0; i < boundary_.size(); ++i) {
    if (max_start_l < boundary_[i].first) {
      max_start_l = boundary_[i].first;
    }
    if (min_end_l > boundary_[i].second) {
      min_end_l = boundary_[i].second;
    }
  }
  bound_limit.first = max_start_l;
  bound_limit.second = min_end_l;
  return bound_limit;
}

}  // namespace planning
}  // namespace century
