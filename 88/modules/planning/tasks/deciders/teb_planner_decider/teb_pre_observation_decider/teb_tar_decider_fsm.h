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

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/tar_action.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/tar_condition.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"
namespace century {
namespace planning {

class TEBTarDeciderFsm {
 public:
  TEBTarDeciderFsm() = default;
  ~TEBTarDeciderFsm() = default;

  /**
   * Initialize the FSM, including transition conditions and actions
   */
  void InitTarFsm(const TaskConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector);
  void ExecuteFsm(const std::shared_ptr<TarVehicleInfo>& tar,
                  Frame* const frame);
  void ExecuteFsmRude(const std::shared_ptr<TarVehicleInfo> tar,
                      Frame* const frame);

 private:
  TEBTarStatus UpdateStatus(const TEBTarStatus& pre_status,
                            const std::shared_ptr<TarVehicleInfo>& tar,
                            Frame* const frame);

  TEBTarStatus status_;
  std::shared_ptr<TarFsmCondition> fsm_condition_;
  std::shared_ptr<TarFsmAction> fsm_action_;

  using Condition = std::function<bool(
      const std::shared_ptr<TarVehicleInfo>& tar, Frame* const frame)>;
  using Action = std::function<void(Frame* const frame)>;
  using Conditions = std::list<std::pair<TEBTarStatus, Condition>>;
  using Actions = std::map<TEBTarStatus, Action>;
  std::map<TEBTarStatus, Actions> actions_;
  std::map<TEBTarStatus, Conditions> conditons_;
  std::mutex frame_mutex_;
};

}  // namespace planning
}  // namespace century
