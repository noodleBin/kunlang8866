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
#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"

namespace century {
namespace planning {
class TarFsmAction {
 public:
  TarFsmAction() = default;
  void Normal2Creep(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::CREEP);
  }
  void Normal2Stop(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::STOP);
  }
  void Creep2Normal(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::NORMAL);
  }
  void Creep2Stop(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::STOP);
  }
  void Stop2Normal(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::NORMAL);
  }
  void Stop2Yield(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::YIELD);
  }
  void Yield2Fail(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::FAIL);
  }
  void Yield2Stop(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::STOP);
  }
  void Normal2Normal(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::NORMAL);
  }
  void Creep2Creep(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::CREEP);
  }
  void Stop2Stop(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::STOP);
  }
  void Yield2Yield(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::YIELD);
  }
  void Fail2Fail(Frame* const frame) {
    frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::FAIL);
  }
  void DoSelf() {}
};
}  // namespace planning
}  // namespace century
