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

#include "modules/planning/common/count_appeared_obstacles.h"

namespace century {
namespace planning {
namespace AppearedObstacle {
namespace {
constexpr size_t kMaxAppearObstaclesNum = 100UL;
constexpr uint32_t kReserveSequenceNum = 5U;
}  // namespace

// Counting the consecutive occurrences of obstacles perceived.
// not thread-safe.
void CountAppearedObstacles(std::shared_ptr<DependencyInjector> injector,
                            const std::vector<const Obstacle*>& obstacle_list) {
  auto ptr_obstacle_appear_counts = injector->GetPtrAppearedObstacles();
  auto sequence_num = injector->GetSequenceNum();
  ADEBUG << "before processor obstacle_appear_counts size: "
         << ptr_obstacle_appear_counts->size();
  for (const auto ptr_obs : obstacle_list) {
    if (ptr_obs->IsVirtual() || ptr_obs->IsIgnore()) {
      continue;
    }
    auto it = ptr_obstacle_appear_counts->find(ptr_obs->Id());
    auto obs_typ = ptr_obs->Perception().type();
    bool is_unknown_type =
        perception::PerceptionObstacle::UNKNOWN == obs_typ ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_typ ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_typ;
    if (ptr_obstacle_appear_counts->end() == it) {
      // When the obstacle_appear_counts capacity has not reached the maximum
      // limit, add the new obstacle to the near list
      if (ptr_obstacle_appear_counts->size() < kMaxAppearObstaclesNum) {
        if (is_unknown_type) {
          ptr_obstacle_appear_counts->insert(
              {ptr_obs->Id(), {1UL, sequence_num, false}});
        } else {
          ptr_obstacle_appear_counts->insert(
              {ptr_obs->Id(), {1UL, sequence_num, true}});
        }
      }
    } else {
      // If the obstacle already exists, update the record value
      // Note: It will only be updated once in a planning cycle
      auto& first_element = std::get<0>(it->second);
      auto& second_element = std::get<1>(it->second);
      auto& third_element = std::get<2>(it->second);
      if (sequence_num != second_element) {
        ++first_element;
        second_element = sequence_num;
        third_element |= !is_unknown_type;
      }
    }
    // If the number of elements in the current obstacle_appear_counts reach
    // 3/4 of the maximum capacity, clean up obsolete elements
    if (ptr_obstacle_appear_counts->size() > kMaxAppearObstaclesNum * 3 / 4) {
      std::vector<std::string> to_remove;
      for (const auto& item : *ptr_obstacle_appear_counts) {
        if (sequence_num - std::get<1>(item.second) > kReserveSequenceNum) {
          to_remove.push_back(item.first);
        }
      }
      for (const auto& key : to_remove) {
        ptr_obstacle_appear_counts->erase(key);
      }
    }
  }
  ADEBUG << "after processor obstacle_appear_counts size: "
         << ptr_obstacle_appear_counts->size();
}
}  // namespace AppearedObstacle
}  // namespace planning
}  // namespace century
