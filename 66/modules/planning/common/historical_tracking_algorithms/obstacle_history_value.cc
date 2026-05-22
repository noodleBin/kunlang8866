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
 * @file obstacle_history_value.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_value.h"

#include <algorithm>
#include <array>
#include <string>

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr uint32_t kDeltaValuesSize = 5U;
constexpr size_t kAverageNumber = 3UL;
constexpr size_t kFilterSize = 3UL;
}  // namespace

uint32_t ObstacleHistoryValue::sequence_num_ = 0;

uint32_t ObstacleHistoryValue::CountLargeValueTimes(
    const std::vector<double>& delta_values, uint32_t already_appear_count,
    double comp_val) {
  uint32_t ret = 0U;
  for (uint32_t i = 0U; i < already_appear_count; ++i) {
    ret += static_cast<uint32_t>(delta_values[i] > comp_val);
  }
  return ret;
}

bool ObstacleHistoryValue::IsNearToPath(const bool loose_constraint,
                                        const double average_delta_value,
                                        const std::vector<double>& delta_values,
                                        const bool has_normal_stoped) {
  bool ret = false;
  if (loose_constraint) {
    if (has_normal_stoped) {
      // ret = average_delta_value >
      // FLAGS_average_step_distance_for_already_stop;
      ret = average_delta_value >
            FLAGS_average_step_distance_for_already_stop_loose_constraint;
    } else {
      ret = average_delta_value >
                FLAGS_first_level_average_step_distance_loose_constraint &&
            (CountLargeValueTimes(
                 delta_values, kDeltaValuesSize,
                 FLAGS_first_level_average_step_distance_loose_constraint) >=
                 kDeltaValuesSize - 1U ||
             CountLargeValueTimes(
                 delta_values, kDeltaValuesSize - 1U,
                 FLAGS_second_level_average_step_distance_loose_constraint) >=
                 kDeltaValuesSize - 2U);
    }
  } else {
    if (has_normal_stoped) {
      ret = average_delta_value > FLAGS_average_step_distance_for_already_stop;
    } else {
      ret = average_delta_value > FLAGS_first_level_average_step_distance &&
            (CountLargeValueTimes(delta_values, kDeltaValuesSize,
                                  FLAGS_first_level_average_step_distance) >=
                 kDeltaValuesSize - 1U ||
             CountLargeValueTimes(delta_values, kDeltaValuesSize - 1U,
                                  FLAGS_second_level_average_step_distance) >=
                 kDeltaValuesSize - 2U);
    }
  }

  return ret;
}

bool ObstacleHistoryValue::IsNearToPath(const double average_delta_value,
                                        const std::vector<double>& delta_values,
                                        const bool has_normal_stoped,
                                        const bool in_common_junction) {
  bool ret = false;
  ADEBUG << "in_common_junction: " << in_common_junction;
  if (has_normal_stoped) {
    if (in_common_junction) {
      ret = average_delta_value > FLAGS_average_step_value_for_already_stop;
    } else {
      ret = average_delta_value >
            FLAGS_average_step_value_for_already_stop_tightly;
    }
  } else {
    if (in_common_junction) {
      ret = average_delta_value > FLAGS_average_step_value &&
            (CountLargeValueTimes(delta_values, kDeltaValuesSize,
                                  FLAGS_four_times_step_value) >=
                 kDeltaValuesSize - 1U ||
             CountLargeValueTimes(delta_values, kDeltaValuesSize - 1U,
                                  FLAGS_three_times_step_value) >=
                 kDeltaValuesSize - 2U);
    } else {
      ret = average_delta_value > FLAGS_average_step_value_tightly &&
            (CountLargeValueTimes(delta_values, kDeltaValuesSize,
                                  FLAGS_four_times_step_value_tightly) >=
                 kDeltaValuesSize - 1U ||
             CountLargeValueTimes(delta_values, kDeltaValuesSize - 1U,
                                  FLAGS_three_times_step_value_tightly) >=
                 kDeltaValuesSize - 2U);
    }
  }
  return ret;
}

bool ObstacleHistoryValue::IsNearToPathSlower(
    const double average_delta_value, const std::vector<double>& delta_values,
    const bool has_slower_stoped) {
  bool ret = false;
  if (has_slower_stoped) {
    ret = average_delta_value >
          FLAGS_average_step_value_for_already_stop_slower_near;
  } else {
    ret = average_delta_value > FLAGS_average_step_value_slower_near &&
          (CountLargeValueTimes(delta_values, kDeltaValuesSize,
                                FLAGS_four_times_step_value_slower_near) >=
               kDeltaValuesSize - 1U ||
           CountLargeValueTimes(delta_values, kDeltaValuesSize - 1U,
                                FLAGS_three_times_step_value_slower_near) >=
               kDeltaValuesSize - 2U);
  }
  return ret;
}

bool ObstacleHistoryValue::GetNearToPathState(
    const bool normal_move_near, const bool slower_move_near,
    std::unordered_map<std::string,
                       ObstacleHistoryValue::ObstacleInfo>::iterator it) {
  MoveNearState move_near_state = NO_MOVE_NEAR;
  if (slower_move_near && normal_move_near) {
    move_near_state = NORMAL_MOVE_NEAR;
  } else if (slower_move_near && !normal_move_near) {
    move_near_state = SLOWER_MOVE_NEAR;
  } else if (!slower_move_near && normal_move_near) {
    AWARN << "Keep Move Near Error: obstacle has normal move near, but not has "
             "slower move near!";
    move_near_state = NORMAL_MOVE_NEAR;
  }

  bool is_near_to_path = false;
  if (it != container_.end()) {
    if ((NORMAL_MOVE_NEAR == move_near_state) ||
        (SLOWER_MOVE_NEAR == move_near_state &&
         it->second.lost_move_near_number < FLAGS_lost_keep_move_near_times)) {
      is_near_to_path = true;
      it->second.lost_move_near_number = 0U;
    } else if (SLOWER_MOVE_NEAR == move_near_state &&
               it->second.lost_move_near_number >=
                   FLAGS_lost_keep_move_near_times) {
      is_near_to_path = false;
      it->second.lost_move_near_number = FLAGS_lost_keep_move_near_times;
    } else if (NO_MOVE_NEAR == move_near_state) {
      is_near_to_path = false;
      it->second.lost_move_near_number =
          it->second.lost_move_near_number >= FLAGS_lost_keep_move_near_times
              ? FLAGS_lost_keep_move_near_times
              : it->second.lost_move_near_number + 1U;
    }
    ADEBUG << "Obs Id: " << it->first;
    ADEBUG << "is_near_to_path: " << is_near_to_path;
    ADEBUG << "it->second.lost_move_near_number: "
           << it->second.lost_move_near_number;
  }
  return is_near_to_path;
}

bool ObstacleHistoryValue::IsMoveNearToPathByDiffL(
    const Obstacle& obs, const common::FrenetFramePoint& frenet_point,
    const bool in_common_junction) {
  bool normal_move_near = false, slower_move_near = false;
  // The maximum record times for start up.
  const uint32_t max_record_times_for_start_up =
      std::max(kDeltaValuesSize, FLAGS_max_record_times_for_start_up);
  // Get the SL boundary of the obstacle.
  const auto& obs_sl = obs.PerceptionSLBoundary();
  // Calculate the difference l value of the obstacle and the path according to
  // obstacle side.
  const double diff_l =
      (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l()
          ? obs_sl.start_l() - frenet_point.l()
          : frenet_point.l() - obs_sl.end_l();
  // Get the id of the obstacle.
  const std::string id = std::to_string(obs.PerceptionId());
  // Check if the container has the obstacle.
  auto it = container_.find(id);
  if (it != container_.end()) {
    // Check if the sequence number of the obstacle is different from the
    // last time.
    if (it->second.last_seq_num != sequence_num_) {
      // Initialize the delta values.
      std::vector<double> delta_values(kDeltaValuesSize, 0.0);
      // Calculate the delta values.
      uint32_t i = 1U;
      for (auto rit_value = it->second.values.rbegin();
           i < kDeltaValuesSize && rit_value != (it->second.values.rend() - 1);
           ++rit_value, ++i) {
        delta_values[i] = *(rit_value + 1) - *rit_value;
      }
      // Calculate the delta values of the first element.
      delta_values[0] = *(it->second.values.rbegin()) - diff_l;
      // Add the new value to the values of the obstacle.
      it->second.values.push_back(diff_l);
      // Check if the number of values is greater than the maximum record
      // times for start up.
      if (it->second.values.size() >
          max_record_times_for_start_up + kFilterSize) {
        // Remove the first element.
        it->second.values.pop_front();
      }
      // Calculate the average delta value.
      double average_delta_value = 0.0;
      if (it->second.values.size() >= kDeltaValuesSize + kFilterSize) {
        double front_value = 0.0;
        for (auto it_value = it->second.values.begin() + kFilterSize;
             it_value !=
             it->second.values.begin() + kAverageNumber + kFilterSize;
             ++it_value) {
          front_value += *it_value;
        }
        double rear_value = 0.0;
        for (auto rit_value = it->second.values.rbegin();
             rit_value != it->second.values.rbegin() + kAverageNumber;
             ++rit_value) {
          rear_value += *rit_value;
        }
        average_delta_value = (front_value - rear_value) /
                              (kAverageNumber * (it->second.values.size() -
                                                 kAverageNumber - kFilterSize));
      }
      // Print the values and delta values.
      ADEBUG << ", values size: " << it->second.values.size()
             << ", average_delta_value: " << average_delta_value
             << ", has normal stoped: " << it->second.has_normal_stoped
             << ", has slower stoped: " << it->second.has_slower_stoped;
      i = 0U;
      for (auto rit_value = it->second.values.rbegin();
           rit_value != it->second.values.rend(); ++rit_value, ++i) {
        ADEBUG << "values(" << i << "): " << *rit_value;
      }
      for (i = 0U; i < kDeltaValuesSize; ++i) {
        ADEBUG << "delta_values(" << i << "): " << delta_values[i];
      }
      // Check if the obstacle is near to the path.
      normal_move_near =
          IsNearToPath(average_delta_value, delta_values,
                       it->second.has_normal_stoped, in_common_junction);
      slower_move_near = IsNearToPathSlower(average_delta_value, delta_values,
                                            it->second.has_slower_stoped);
      // Update the stoped flag.
      it->second.has_normal_stoped |= normal_move_near;
      it->second.has_slower_stoped |= slower_move_near;
      // Update the last sequence number.
      it->second.last_seq_num = sequence_num_;
    }
  } else {
    // Clear the obsolete elements.
    auto erased_num = ClearObsoleteElements();
    ADEBUG << "erased number:" << erased_num;
    // Create a new element.
    ObstacleInfo obs_info(sequence_num_, diff_l);
    container_.emplace(id, obs_info);
  }
  // Print the return value.
  ADEBUG << "normal_move_near = " << normal_move_near
         << ", slower_move_near = " << slower_move_near;
  return GetNearToPathState(normal_move_near, slower_move_near, it);
}

bool ObstacleHistoryValue::IsMoveNearToPathByDiffL(
    const bool loose_constraint, const Obstacle& obs,
    const common::FrenetFramePoint& frenet_point) {
  bool ret = false;
  const uint32_t max_record_times_for_start_up =
      std::max(kDeltaValuesSize, FLAGS_max_record_times_for_start_up);
  const auto& obs_sl = obs.PerceptionSLBoundary();
  const bool is_left_obs =
      (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
  const double diff_l = is_left_obs ? obs_sl.start_l() - frenet_point.l()
                                    : frenet_point.l() - obs_sl.end_l();
  // ADEBUG << "diff_l = " << diff_l;
  const std::string id = std::to_string(obs.PerceptionId());
  auto it = container_.find(id);
  if (it != container_.end()) {
    // ADEBUG << "in container : it->second.last_seq_num = "
    //        << it->second.last_seq_num;
    if (it->second.last_seq_num != sequence_num_) {
      std::vector<double> delta_values(kDeltaValuesSize, 0.0);
      uint32_t i = 1U;
      for (auto rit_value = it->second.values.rbegin();
           i < kDeltaValuesSize && rit_value != (it->second.values.rend() - 1);
           ++rit_value, ++i) {
        delta_values[i] = *(rit_value + 1) - *rit_value;
        // ADEBUG << "delta_values[" << i << "] = " << delta_values[i];
      }
      delta_values[0] = *(it->second.values.rbegin()) - diff_l;
      // ADEBUG << "elta_values[0] = " << delta_values[0];
      it->second.values.push_back(diff_l);
      if (it->second.values.size() >
          max_record_times_for_start_up + kFilterSize) {
        it->second.values.pop_front();
      }
      double average_delta_value = 0.0;
      if (it->second.values.size() >= kDeltaValuesSize + kFilterSize) {
        double front_value = 0.0;
        for (auto it_value = it->second.values.begin() + kFilterSize;
             it_value !=
             it->second.values.begin() + kAverageNumber + kFilterSize;
             ++it_value) {
          front_value += *it_value;
        }
        double rear_value = 0.0;
        for (auto rit_value = it->second.values.rbegin();
             rit_value != it->second.values.rbegin() + kAverageNumber;
             ++rit_value) {
          rear_value += *rit_value;
        }
        average_delta_value = (front_value - rear_value) /
                              (kAverageNumber * (it->second.values.size() -
                                                 kAverageNumber - kFilterSize));
      }
      ADEBUG << ", values size: " << it->second.values.size()
             << ", average_delta_value: " << average_delta_value
             << ", has normal stoped: " << it->second.has_normal_stoped;
      i = 0U;
      for (auto rit_value = it->second.values.rbegin();
           rit_value != it->second.values.rend(); ++rit_value, ++i) {
        ADEBUG << "values(" << i << "): " << *rit_value;
      }
      for (i = 0U; i < kDeltaValuesSize; ++i) {
        ADEBUG << "delta_values(" << i << "): " << delta_values[i];
      }
      ret = IsNearToPath(loose_constraint, average_delta_value, delta_values,
                         it->second.has_normal_stoped);
      it->second.has_normal_stoped |= ret;
      it->second.last_seq_num = sequence_num_;
    }
  } else {
    auto erased_num = ClearObsoleteElements();
    ADEBUG << "erased number:" << erased_num;
    ObstacleInfo obs_info(sequence_num_, diff_l);
    ADEBUG << "first add , sequence_num_ =  " << sequence_num_;
    container_.emplace(id, obs_info);
  }
  ADEBUG << "ret = " << ret;
  return ret;
}

size_t ObstacleHistoryValue::ClearObsoleteElements() {
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
