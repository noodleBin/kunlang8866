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
 * @file obstacle_history_diff_value.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_diff_value.h"

#include <algorithm>

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr size_t kAverageNumber = 3UL;
constexpr size_t kFilterSize = 8UL;
constexpr double kDeltaTime = 0.1;
constexpr double kInitRelSpeed = 1e10;
}  // namespace

uint32_t ObstacleHistoryDiffValue::sequence_num_ = 0;

ObstacleHistoryDiffValue::MovingState
ObstacleHistoryDiffValue::GetMoveStateByRelSpeed(double relative_speed) {
  MovingState ret = STABLE_MOVING;
  if (relative_speed > FLAGS_stable_speed_buffer) {
    ret = FASTER_MOVING;
  } else if (relative_speed < -FLAGS_stable_speed_buffer) {
    ret = SLOWER_MOVING;
  } else {
    ret = STABLE_MOVING;
  }
  return ret;
}
ObstacleHistoryDiffValue::MovingState
ObstacleHistoryDiffValue::GetObstacleMoveStateByDiffS(const Obstacle& obs,
                                                      const SLBoundary& adc_sl,
                                                      double* relative_speed) {
  MovingState ret = UNKNOWN_MOVING;
  const auto& obs_sl = obs.PerceptionSLBoundary();
  const double adc_center_s = 0.5 * (adc_sl.start_s() + adc_sl.end_s());
  const double obs_center_s = 0.5 * (obs_sl.start_s() + obs_sl.end_s());
  // Calculate the difference s value of the obstacle and the adc.
  const double diff_s = obs_center_s - adc_center_s;
  // Get the id of the obstacle.
  const std::string id = std::to_string(obs.PerceptionId());
  // Check if the container has the obstacle.
  auto it = container_.find(id);
  *relative_speed = kInitRelSpeed;
  if (it != container_.end()) {
    // Check if the sequence number of the obstacle is different from the
    // last time.
    if (it->second.last_seq_num != sequence_num_) {
      // Add the new value to the values of the obstacle.
      it->second.values.push_back(diff_s);
      // Check if the number of values is greater than the maximum record
      // times for start up.
      if (it->second.values.size() > FLAGS_max_record_times_for_diff_s) {
        // Remove the first element.
        it->second.values.pop_front();
      }
      // Calculate the average delta value.
      double average_delta_value = 0.0;
      if (it->second.values.size() >= kFilterSize) {
        double front_value = 0.0;  // old
        for (auto it_value = it->second.values.begin();
             it_value != it->second.values.begin() + kAverageNumber;
             ++it_value) {
          front_value += *it_value;
        }
        double rear_value = 0.0;  // new
        for (auto rit_value = it->second.values.rbegin();
             rit_value != it->second.values.rbegin() + kAverageNumber;
             ++rit_value) {
          rear_value += *rit_value;
        }
        average_delta_value =
            (rear_value - front_value) /
            (kAverageNumber * (it->second.values.size() - kAverageNumber));
        *relative_speed = average_delta_value / kDeltaTime;
        ret = GetMoveStateByRelSpeed(*relative_speed);
      }
      // Print the values and delta values.
      ADEBUG << "values size: " << it->second.values.size()
             << ", average_delta_value: " << average_delta_value;
      uint32_t i = 0U;
      for (auto rit_value = it->second.values.rbegin();
           rit_value != it->second.values.rend(); ++rit_value, ++i) {
        ADEBUG << "values(" << i << "): " << *rit_value;
      }
      // Update the last sequence number.
      it->second.last_seq_num = sequence_num_;
    } else {
      ADEBUG << "has get before in same sequence num ";
      *relative_speed = it->second.relative_speed;
      if (*relative_speed < kInitRelSpeed) {
        ret = GetMoveStateByRelSpeed(*relative_speed);
      }
    }
  } else {
    // Create a new element.
    ADEBUG << "new element";
    ObstacleInfo obs_info(sequence_num_, diff_s);
    container_.emplace(id, obs_info);
  }

  // Print the return value.
  ADEBUG << "ret = " << ret;
  return ret;
}

size_t ObstacleHistoryDiffValue::ClearObsoleteElements() {
  size_t erase_num = 0;
  std::vector<std::string> to_remove;
  for (const auto& item : container_) {
    if (sequence_num_ != item.second.last_seq_num) {
      to_remove.emplace_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += container_.erase(key);
  }
  return erase_num;
}

}  // namespace planning
}  // namespace century
