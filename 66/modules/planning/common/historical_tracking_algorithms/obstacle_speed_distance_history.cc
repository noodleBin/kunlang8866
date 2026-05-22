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
 * @file obstacle_speed_distance_history.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_speed_distance_history.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <string>

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr uint32_t kMinValuesSize = 4U;
constexpr size_t kAverageNumber = 3UL;
constexpr size_t kFilterSize = 1UL;
constexpr double kEpsilon = 1.0e-10;
}  // namespace

uint32_t ObstacleSpeedDistanceHistory::sequence_num_ = 0;

bool ObstacleSpeedDistanceHistory::CorrectObstacleSpeed(
    const Obstacle& obs, double* const corrected_speed) {
  CHECK_NOTNULL(corrected_speed);
  bool is_need_correct_speed = false;
  // The maximum record times for start up.
  const uint32_t max_record_times_for_correct_speed =
      std::max(kMinValuesSize, FLAGS_max_record_times_for_correct_speed);

  const common::math::Vec2d curr_center(obs.Perception().position().x(),
                                        obs.Perception().position().y());
  const double curr_timestamp = obs.Perception().timestamp();
  // Get the id of the obstacle.
  const std::string id = std::to_string(obs.PerceptionId());
  // Check if the container has the obstacle.
  auto it = container_.find(id);
  double average_speed = 0.0;
  if (it != container_.end()) {
    // Check if the sequence number of the obstacle is different from the
    // last time.
    if (it->second.last_seq_num != sequence_num_) {
      // Add the new value to the values of the obstacle.
      it->second.values.emplace_back(curr_center, curr_timestamp);
      // Check if the number of values is greater than the maximum record
      // times for start up.
      if (it->second.values.size() >
          max_record_times_for_correct_speed + kFilterSize) {
        // Remove the first element.
        it->second.values.pop_front();
      }
      // Calculate the average speed according to obstacle enu position.
      if (it->second.values.size() >= kMinValuesSize + kFilterSize) {
        common::math::Vec2d front_ave_point;
        double front_time = 0.0;
        for (auto it_value = it->second.values.begin() + kFilterSize;
             it_value !=
             it->second.values.begin() + kAverageNumber + kFilterSize;
             ++it_value) {
          front_ave_point += it_value->first;
          front_time += it_value->second;
        }
        front_ave_point /= static_cast<double>(kAverageNumber);
        front_time /= static_cast<double>(kAverageNumber);

        common::math::Vec2d rear_ave_point;
        double rear_time = 0.0;
        for (auto rit_value = it->second.values.rbegin();
             rit_value != it->second.values.rbegin() + kAverageNumber;
             ++rit_value) {
          rear_ave_point += rit_value->first;
          rear_time += rit_value->second;
        }
        rear_ave_point /= static_cast<double>(kAverageNumber);
        rear_time /= static_cast<double>(kAverageNumber);
        average_speed = rear_ave_point.DistanceTo(front_ave_point) /
                        (rear_time - front_time + kEpsilon);
        is_need_correct_speed = std::abs(obs.speed() - average_speed) >
                                FLAGS_speed_correct_threshold;
      }
      // Print the values.
      ADEBUG << ", values size: " << it->second.values.size()
             << ", average_speed: " << average_speed;
      uint32_t i = 0U;
      for (auto rit_value = it->second.values.rbegin();
           rit_value != it->second.values.rend(); ++rit_value, ++i) {
        ADEBUG << "values(" << i << "): time[" << std::fixed
               << std::setprecision(3) << rit_value->second << "], pos["
               << rit_value->first.x() << ", " << rit_value->first.y() << "]";
      }
      // Update the last sequence number.
      it->second.last_seq_num = sequence_num_;
    }
  } else {
    // Clear the obsolete elements.
    auto erased_num = ClearObsoleteElements();
    ADEBUG << "erased number:" << erased_num;
    // Create a new element.
    ObstacleInfo obs_info(sequence_num_, curr_center, curr_timestamp);
    container_.emplace(id, obs_info);
  }
  if (is_need_correct_speed) {
    *corrected_speed = average_speed;
  }
  // Print the return value.
  ADEBUG << "is_need_correct_speed = " << is_need_correct_speed
         << ", corrected_speed = " << *corrected_speed;
  return is_need_correct_speed;
}

size_t ObstacleSpeedDistanceHistory::ClearObsoleteElements() {
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
