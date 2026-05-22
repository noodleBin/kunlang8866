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

#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_action.h"
#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_condition.h"

namespace century {
namespace planning {

class OverTakeFsm {
 public:
  OverTakeFsm();

  /**
   * Initialize the FSM, including transition conditions and actions
   */
  void InitOverTakeFsm(const TaskConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector);
  void ExecuteFsm(Frame* const frame,
                  ReferenceLineInfo* const reference_line_info);
  const OverTakeStatus GetCurrentStatus() const { return status_; }

 private:
  OverTakeStatus UpdateStatus(const OverTakeStatus& pre_status,
                              Frame* const frame,
                              ReferenceLineInfo* const reference_line_info);

 private:
  OverTakeStatus status_;
  std::shared_ptr<FsmCondition> fsm_condition_;
  std::shared_ptr<FsmAction> fsm_action_;

  using Condition = std::function<bool(
      Frame* const frame, ReferenceLineInfo* const reference_line_info)>;
  using Action = std::function<void(
      Frame* const frame, ReferenceLineInfo* const reference_line_info)>;
  using Conditions = std::list<std::pair<OverTakeStatus, Condition>>;
  using Actions = std::map<OverTakeStatus, Action>;
  std::map<OverTakeStatus, Actions> actions_;
  std::map<OverTakeStatus, Conditions> conditons_;
};

}  // namespace planning
}  // namespace century
