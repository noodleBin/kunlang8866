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
 * @file obstacle_stabilization_for_teb_speed.h
 **/

#pragma once

#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace century {
namespace planning {

class ObstacleStabilizationForTEBSpeed {
 public:
  // sequence number + obstacle latitude distance
  using LatDistance = std::pair<uint32_t, double>;

 private:
  struct ObstacleInfo {
    ObstacleInfo(bool is_left_obs, LatDistance lat_dis, double s)
        : is_left(is_left_obs), last_s(s) {
      // history_distances.push(lat_dis);
      history_distances.push_back(lat_dis);
    }
    bool is_left;
    double last_s;
    std::deque<LatDistance> history_distances;
  };

 public:
  ObstacleStabilizationForTEBSpeed() : capacity_(50UL) { container_.clear(); }
  explicit ObstacleStabilizationForTEBSpeed(size_t capacity)
      : capacity_(capacity) {
    container_.clear();
  }
  virtual ~ObstacleStabilizationForTEBSpeed() {}
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return container_.size(); }
  size_t GetCapacity() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  double GetObstacleDistance(const std::string& obs_id, double lat_dis,
                             bool is_left_obs, double obs_s);

 private:
  bool IsObstacleKeepMoving(const std::vector<double>& diff_dis);
  size_t ClearObsoleteElements();

 private:
  std::unordered_map<std::string, ObstacleInfo> container_;
  size_t capacity_;
  static uint32_t sequence_num_;
};

}  // namespace planning
}  // namespace century
