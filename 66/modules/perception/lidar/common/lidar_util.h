/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include <chrono>
#include <Eigen/Dense>

namespace century {
namespace perception {
namespace lidar {


bool GetNearest(const std::vector<std::pair<double, Eigen::Affine3d>>& v,
                           double ts, Eigen::Affine3d& chosen_pose, double& chosen_time, bool time_show=false) {
  if (v.empty()) return false;

  auto it = std::lower_bound(
    v.begin(), v.end(), ts,
    [](const std::pair<double, Eigen::Affine3d>& p, double value) {
        return p.first > value;
    }
  );

  if (it == v.begin()) {
    chosen_time = it->first;
    chosen_pose = it->second;
  }
  if (it == v.end()) {
    chosen_time = v.back().first;
    chosen_pose = v.back().second;
  }

  auto it_prev = it - 1;
  double t_le = it->first;      // <= ts
  double t_gt = it_prev->first; // > ts

  if (fabs(t_le - ts) < fabs(t_gt - ts)) {
    chosen_time = t_le;
    chosen_pose = it->second;
  } else {
    chosen_time = t_gt;
    chosen_pose = it_prev->second;
  }

  if (time_show) {
    std::cout << std::fixed << std::setprecision(6)
              << "[v max ts] " << v.front().first 
              << ", [v min ts] " << v.back().first << std::endl;
    std::cout << "[selected] time = " << std::fixed << std::setprecision(6)
              << chosen_time << std::endl;
  }
  if (fabs(chosen_time - ts) > 0.1) {
      std::cout << "[warning] large time difference: " 
                << fabs(it->first - ts) << std::endl;
      return false;
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
