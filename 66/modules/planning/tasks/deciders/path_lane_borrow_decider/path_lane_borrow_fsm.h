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

#pragma once

#include <functional>
#include <list>
#include <map>
#include <memory>
#include <utility>

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_action.h"
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_condition.h"

namespace century {
namespace planning {

class LaneBorrowFsm {
 public:
  LaneBorrowFsm() = default;

  //
  // Initialize the FSM, including transition conditions and actions
  //
  void InitLaneBorrowFsm(const TaskConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector);
  void ExecuteFsm(Frame* const frame,
                  ReferenceLineInfo* const reference_line_info);

 private:
  LaneBorrowStatus UpdateStatus(const LaneBorrowStatus& pre_status,
                                Frame* const frame,
                                ReferenceLineInfo* const reference_line_info);

 private:
  LaneBorrowStatus status_;
  std::shared_ptr<LaneBorrowFsmCondition> fsm_condition_;
  std::shared_ptr<LaneBorrowFsmAction> fsm_action_;

  using Condition = std::function<bool(
      Frame* const frame, ReferenceLineInfo* const reference_line_info)>;
  using Action = std::function<void(
      Frame* const frame, ReferenceLineInfo* const reference_line_info)>;
  using Conditions = std::list<std::pair<LaneBorrowStatus, Condition>>;
  using Actions = std::map<LaneBorrowStatus, Action>;
  std::map<LaneBorrowStatus, Actions> actions_;
  std::map<LaneBorrowStatus, Conditions> conditions_;
};

}  // namespace planning
}  // namespace century
