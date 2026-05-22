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
 */

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning_status.pb.h"

#include "cyber/common/macros.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace century {
namespace planning {

struct RouteLaneInfo {
  std::string lane_id;
  bool can_exit = false;
};

class PlanningContext {
 public:
  PlanningContext() = default;

  void Clear();
  void Init();

  /*
   * please put all status info inside PlanningStatus for easy maintenance.
   * do NOT create new struct at this level.
   * */
  const PlanningStatus& planning_status() const { return planning_status_; }
  PlanningStatus* mutable_planning_status() { return &planning_status_; }
  const std::vector<std::vector<RouteLaneInfo>>& route_lane_info() const {
    return route_lane_info_;
  }
  std::vector<std::vector<RouteLaneInfo>>* mutable_route_lane_info() {
    return &route_lane_info_;
  }
  const std::unordered_map<std::string, std::pair<size_t, size_t>>&
  route_lane_index() const {
    return route_lane_index_;
  }
  std::unordered_map<std::string, std::pair<size_t, size_t>>*
  mutable_route_lane_index() {
    return &route_lane_index_;
  }
  const std::vector<double>& merge_lane_lateral_l() const {
    return merge_lane_lateral_l_;
  }
  std::vector<double>* mutable_merge_lane_lateral_l() {
    return &merge_lane_lateral_l_;
  }

 private:
  PlanningStatus planning_status_;
  std::vector<std::vector<RouteLaneInfo>> route_lane_info_;
  std::unordered_map<std::string, std::pair<size_t, size_t>> route_lane_index_;
  std::vector<double> merge_lane_lateral_l_;
};

}  // namespace planning
}  // namespace century
