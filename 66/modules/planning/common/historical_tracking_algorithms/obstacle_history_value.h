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
 * @file obstacle_history_value.h
 **/

#pragma once

#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

class ObstacleHistoryValue {
 private:
  enum MoveNearState {
    NO_MOVE_NEAR = 0,
    SLOWER_MOVE_NEAR,
    NORMAL_MOVE_NEAR,
    FASTER_MOVE_NEAR
  };

  struct ObstacleInfo {
    ObstacleInfo(uint32_t seq_num, double value)
        : appear_count(1U),
          has_normal_stoped(false),
          has_slower_stoped(false),
          lost_move_near_number(std::numeric_limits<uint32_t>::max()),
          last_seq_num(seq_num) {
      values.push_back(value);
    }
    uint32_t appear_count;
    bool has_normal_stoped;
    bool has_slower_stoped;
    uint32_t lost_move_near_number;
    uint32_t last_seq_num;
    std::deque<double> values;
  };

 public:
  explicit ObstacleHistoryValue(size_t capacity) : capacity_(capacity) {
    container_.clear();
  }

  virtual ~ObstacleHistoryValue() {}
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return container_.size(); }
  size_t GetCapacitySize() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  bool IsMoveNearToPathByDiffL(const Obstacle& obs,
                               const common::FrenetFramePoint& frenet_point,
                               const bool in_common_junction);
  bool IsMoveNearToPathByDiffL(const bool loose_constraint, const Obstacle& obs,
                               const common::FrenetFramePoint& frenet_point);

 private:
  size_t ClearObsoleteElements();
  bool IsNearToPath(const double average_delta_value,
                    const std::vector<double>& delta_values,
                    const bool has_normal_stoped,
                    const bool in_common_junction);
  bool IsNearToPathSlower(const double average_delta_value,
                          const std::vector<double>& delta_values,
                          const bool has_slower_stoped);
  bool GetNearToPathState(
      const bool normal_move_near, const bool slower_move_near,
      std::unordered_map<std::string, ObstacleInfo>::iterator it);
  bool IsNearToPath(const bool loose_constraint,
                    const double average_delta_value,
                    const std::vector<double>& delta_values,
                    const bool has_normal_stoped);
  uint32_t CountLargeValueTimes(const std::vector<double>& delta_value,
                                uint32_t already_appear_count, double comp_val);

 private:
  std::unordered_map<std::string, ObstacleInfo> container_;
  size_t capacity_;
  static uint32_t sequence_num_;
};

}  // namespace planning
}  // namespace century
