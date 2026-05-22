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
 * @file obstacle_history_diff_value.h
 **/

#pragma once

#include <deque>
#include <string>
#include <unordered_map>
#include <limits>
#include <vector>

#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

class ObstacleHistoryDiffValue {
 public:
  enum MovingState {
    UNKNOWN_MOVING = 0,
    STABLE_MOVING,
    SLOWER_MOVING,
    FASTER_MOVING
  };

 private:
  struct ObstacleInfo {
    ObstacleInfo(uint32_t seq_num, double value)
        : last_seq_num(seq_num),
          relative_speed(std::numeric_limits<double>::max()) {
      values.push_back(value);
    }
    uint32_t last_seq_num;
    std::deque<double> values;
    // for already get the state in current sequence_num_
    double relative_speed;
  };

 public:
  explicit ObstacleHistoryDiffValue(size_t capacity) : capacity_(capacity) {
    container_.clear();
  }

  virtual ~ObstacleHistoryDiffValue() {}
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return container_.size(); }
  size_t GetCapacitySize() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  MovingState GetObstacleMoveStateByDiffS(const Obstacle& obs,
                                          const SLBoundary& adc_sl,
                                          double* relative_speed);
  size_t ClearObsoleteElements();

 private:
  MovingState GetMoveStateByRelSpeed(double relative_speed);

 private:
  std::unordered_map<std::string, ObstacleInfo> container_;
  size_t capacity_;
  static uint32_t sequence_num_;
};

}  // namespace planning
}  // namespace century
