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
 * @file obstacle_speed_distance_history.h
 **/

#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

class ObstacleSpeedDistanceHistory {
 private:
  struct ObstacleInfo {
    ObstacleInfo(uint32_t seq_num, common::math::Vec2d value, double timestamp)
        : last_seq_num(seq_num) {
      values.emplace_back(value, timestamp);
    }
    uint32_t last_seq_num;
    std::deque<std::pair<common::math::Vec2d, double>> values;
  };

 public:
  explicit ObstacleSpeedDistanceHistory(size_t capacity) : capacity_(capacity) {
    container_.clear();
  }
  virtual ~ObstacleSpeedDistanceHistory() {}

  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return container_.size(); }
  size_t GetCapacitySize() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  bool CorrectObstacleSpeed(const Obstacle& obs, double* const corrected_speed);

 private:
  size_t ClearObsoleteElements();

 private:
  std::unordered_map<std::string, ObstacleInfo> container_;
  size_t capacity_;
  static uint32_t sequence_num_;
};

}  // namespace planning
}  // namespace century
