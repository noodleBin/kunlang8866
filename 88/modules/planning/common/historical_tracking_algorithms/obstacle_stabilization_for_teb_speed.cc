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
 * @file obstacle_stabilization_for_teb_speed.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_stabilization_for_teb_speed.h"

#include <algorithm>
#include <string>
#include <vector>

#include "cyber/common/log.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr uint32_t kHistorySeqNum = 5U;
constexpr uint32_t kObsoleteDiffSeqNum = 10U;

constexpr double kEpsilon = 1e-10;
}  // namespace

uint32_t ObstacleStabilizationForTEBSpeed::sequence_num_ = 0;

bool ObstacleStabilizationForTEBSpeed::IsObstacleKeepMoving(
    const std::vector<double>& diff_dis) {
  if (diff_dis.size() < 3UL) {
    ADEBUG << "diff_dis size is " << diff_dis.size();
    return false;
  }
  size_t keep_close_num = 0UL, keep_close_buffer_num = 0UL, keep_away_num = 0UL,
         keep_away_buffer_num = 0UL;
  for (const auto dis : diff_dis) {
    if (dis > 0.0) {
      ++keep_close_num;
      ++keep_close_buffer_num;
    } else if (dis > -0.2) {
      ++keep_close_buffer_num;
    }
    if (dis < 0.0) {
      ++keep_away_num;
      ++keep_away_buffer_num;
    } else if (dis < 0.2) {
      ++keep_away_buffer_num;
    }
  }
  ADEBUG << "keep_close_num: " << keep_close_num
         << ",\n\tkeep_close_buffer_num: " << keep_close_buffer_num
         << ",\n\tkeep_away_num: " << keep_away_num
         << ",\n\tkeep_away_buffer_num: " << keep_away_buffer_num;
  ADEBUG << "diff_dis size: " << diff_dis.size();
  bool is_keep_close =
      (keep_close_num >= diff_dis.size() ||
       (keep_close_buffer_num >= diff_dis.size() && keep_close_num >= 4UL));

  bool is_keep_away =
      (keep_away_num >= diff_dis.size() ||
       (keep_away_buffer_num >= diff_dis.size() && keep_away_num >= 4UL));
  ADEBUG << "is_keep_close: " << is_keep_close
         << ", is_keep_away: " << is_keep_away;
  return is_keep_close || is_keep_away;
}

double ObstacleStabilizationForTEBSpeed::GetObstacleDistance(
    const std::string& id, double lat_dis, bool is_left_obs, double obs_s) {
  std::string perception_id = id;
  if (std::string::npos != id.find("_0")) {
    perception_id.erase(perception_id.length() - 2);
  }
  double ret = lat_dis;
  auto it = container_.find(perception_id);
  if (it != container_.end()) {
    if ((*it).second.is_left == is_left_obs) {
      std::vector<double> diff_dis;
      std::vector<double> recent_dis;
      double last_dis = lat_dis;
      recent_dis.push_back(last_dis);
      const auto& his_distances = (*it).second.history_distances;

      for (auto item = his_distances.rbegin(); item != his_distances.rend();
           ++item) {
        if (sequence_num_ - item->first <= kHistorySeqNum) {
          // positive diff_dis item: keep close obstacle.
          diff_dis.push_back(-(last_dis - item->second));
          recent_dis.push_back(item->second);
          last_dis = item->second;
        } else {
          break;
        }
      }
      uint32_t i = 0U;
      for (auto rit_diff = diff_dis.rbegin(); rit_diff != diff_dis.rend();
           ++rit_diff, ++i) {
        ADEBUG << "diff distance(" << i << "): " << *rit_diff;
      }
      i = 0U;
      for (auto rit_dis = diff_dis.rbegin(); rit_dis != diff_dis.rend();
           ++rit_dis, ++i) {
        ADEBUG << "distance(" << i << "): " << *rit_dis;
      }
      if (diff_dis.empty() || IsObstacleKeepMoving(diff_dis)) {
        ret = lat_dis;
      } else {
        auto min_item = std::min_element(recent_dis.begin(), recent_dis.end());
        if (recent_dis.end() != min_item) {
          ret = *min_item;
        } else {
          ret = lat_dis;
        }
      }
      (*it).second.history_distances.emplace_back(sequence_num_, lat_dis);
    } else {
      (*it).second = ObstacleInfo(
          is_left_obs, std::make_pair(sequence_num_, lat_dis), obs_s);
    }
    if (sequence_num_ - it->second.history_distances.front().first >
        kObsoleteDiffSeqNum) {
      it->second.history_distances.pop_front();
    }
  } else {
    ADEBUG << "container_ size before clear: " << container_.size();
    auto erased_num = ClearObsoleteElements();
    ADEBUG << "erased number:" << erased_num;
    ObstacleInfo obs_info(is_left_obs, std::make_pair(sequence_num_, lat_dis),
                          obs_s);
    container_.insert({perception_id, obs_info});
  }
  return ret;
}

size_t ObstacleStabilizationForTEBSpeed::ClearObsoleteElements() {
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
    if (sequence_num_ - item.second.history_distances.back().first >
        obsolete_seq_num) {
      to_remove.push_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += container_.erase(key);
  }

  if (container_.size() >= capacity_) {
    auto it_max_dist = container_.begin();
    for (auto item = container_.begin(); item != container_.end(); ++item) {
      if (item->second.last_s > it_max_dist->second.last_s) {
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
