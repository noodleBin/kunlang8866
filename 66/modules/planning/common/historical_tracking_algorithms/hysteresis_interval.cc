/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or a是greed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file hysteresis_interval.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"

#include <algorithm>
#include <string>
#include <vector>

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5;
constexpr uint32_t kHighNumObsoleteSeqNum = 2;
constexpr double kEpsilon = 1e-10;
}  // namespace

uint32_t HysteresisInterval::sequence_num_ = 0;

void HysteresisInterval::SetAnchorLimits(double anchor, double relative_lower,
                                         double relative_upper) {
  anchor_value_ = anchor;
  lower_limit_ = anchor + relative_lower;
  upper_limit_ = anchor + relative_upper;
  if (lower_limit_ > anchor_value_) {
    lower_limit_ = anchor_value_;
    AWARN << "the lower limit value is too large.";
  }
  if (upper_limit_ < anchor_value_) {
    upper_limit_ = anchor_value_;
    AWARN << "the upper limit value is too small.";
  }
}

// This function is order to change value level (higher or lower) by id.
// Usually used when this value level cannot be determined only by it's
// historical value.
void HysteresisInterval::SetValueLevel(
    const std::string id, HysteresisInterval::ValueLevel value_lev) {
  auto it = container_.find(id);
  if (it != container_.end()) {
    (*it).second.value_level = value_lev;
  } else {
    ObstacleInfo obs_info(value_lev, sequence_num_, 0.0);
    container_.insert({id, obs_info});
  }
}

double HysteresisInterval::HyValue(const Obstacle& obs, double val) {
  return HyValueInternal(std::to_string(obs.PerceptionId()), val,
                         obs.PerceptionSLBoundary().start_s());
}

double HysteresisInterval::HyValue(const std::string id, double val) {
  return HyValueInternal(id, val, 0.0);
}

double HysteresisInterval::HyValueInternal(const std::string id, double val,
                                           double obs_s) {
  double ret = val;
  auto it = container_.find(id);
  if (it != container_.end()) {
    if ((*it).second.last_seq_num != sequence_num_) {
      (*it).second.obs_s = obs_s;
      (*it).second.last_seq_num = sequence_num_;
      if (val >= upper_limit_) {
        (*it).second.value_level = HIGHER_LEVEL;
      } else if (val <= lower_limit_) {
        (*it).second.value_level = LOWER_LEVEL;
      } else {
        // keep value_level state
      }
    }
    switch ((*it).second.value_level) {
      case HIGHER_LEVEL:
        ret = std::max(anchor_value_ + kEpsilon, val);
        break;
      case LOWER_LEVEL:
        ret = std::min(anchor_value_ - kEpsilon, val);
        break;
      default:
        ret = val;
        break;
    }
  } else {
    ADEBUG << "container_ size before clear: " << container_.size();
    auto erased_num = ClearObsoleteElements();
    ADEBUG << "erased number:" << erased_num;
    ValueLevel level = (val > anchor_value_) ? HIGHER_LEVEL : LOWER_LEVEL;
    ObstacleInfo obs_info(level, sequence_num_, obs_s);
    container_.insert({id, obs_info});
  }
  return ret;
}

size_t HysteresisInterval::ClearObsoleteElements() {
  double obsolete_seq_num = kLowNumObsoleteSeqNum;
  if (container_.size() < capacity_ / 2) {
    ADEBUG << "the container capacity is enough, so deleting nothing.";
    return 0UL;
  } else if (container_.size() < 3 * capacity_ / 4) {
    obsolete_seq_num = kLowNumObsoleteSeqNum;
  } else {
    obsolete_seq_num = kHighNumObsoleteSeqNum;
  }
  size_t erase_num = 0;

  std::vector<std::string> to_remove;
  for (const auto& item : container_) {
    if (sequence_num_ - item.second.last_seq_num > obsolete_seq_num) {
      to_remove.push_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += container_.erase(key);
  }

  if (container_.size() >= capacity_) {
    auto it_max_dist = container_.begin();
    for (auto item = container_.begin(); item != container_.end(); ++item) {
      if (item->second.obs_s > it_max_dist->second.obs_s) {
        it_max_dist = item;
      }
    }
    if (container_.find(it_max_dist->first) != container_.end()) {
      erase_num += container_.erase(it_max_dist->first);
    }
    AWARN << "Container is full, erasing an item according to the maximal "
             "obs_s.";
  }
  return erase_num;
}

}  // namespace planning
}  // namespace century
