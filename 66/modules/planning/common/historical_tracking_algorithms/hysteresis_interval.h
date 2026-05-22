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
 * @file hysteresis_interval.h
 **/

#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

class HysteresisInterval {
 public:
  enum ValueLevel { NO_INIT_LEVEL = 0, HIGHER_LEVEL, LOWER_LEVEL };

 private:
  struct ObstacleInfo {
    ObstacleInfo(ValueLevel value_lev, uint32_t seq_num, double s)
        : value_level(value_lev), last_seq_num(seq_num), obs_s(s) {}
    ValueLevel value_level;
    uint32_t last_seq_num;
    double obs_s;
  };

 public:
  HysteresisInterval() = delete;
  HysteresisInterval(double anchor, double range, size_t capacity = 50UL)
      : anchor_value_(anchor), capacity_(capacity) {
    lower_limit_ = anchor_value_ - range / 2;
    upper_limit_ = anchor_value_ + range / 2;
    container_.clear();
  }
  HysteresisInterval(double anchor, double relative_lower,
                     double relative_upper, size_t capacity)
      : anchor_value_(anchor),
        lower_limit_(anchor + relative_lower),
        upper_limit_(anchor + relative_upper),
        capacity_(capacity) {
    if (lower_limit_ > anchor_value_) {
      lower_limit_ = anchor_value_;
      AWARN << "the lower limit value is too large.";
    }
    if (upper_limit_ < anchor_value_) {
      upper_limit_ = anchor_value_;
      AWARN << "the upper limit value is too small.";
    }
    container_.clear();
  }
  virtual ~HysteresisInterval() {}
  void SetAnchorLimits(double anchor, double relative_lower,
                       double relative_upper);
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return container_.size(); }
  size_t GetCapacity() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  double HyValue(const Obstacle& obs, double val);
  double HyValue(const std::string id, double val);
  // SetValueLevel function is order to change value level (higher or lower) by
  // id. Usually used when this value level cannot be determined only by it's
  // historical value.
  void SetValueLevel(const std::string id, ValueLevel value_lev);

 private:
  size_t ClearObsoleteElements();
  double HyValueInternal(const std::string id, double val, double obs_s);

 private:
  std::unordered_map<std::string, ObstacleInfo> container_;
  double anchor_value_;
  double lower_limit_;
  double upper_limit_;
  size_t capacity_;
  static uint32_t sequence_num_;
};

}  // namespace planning
}  // namespace century
