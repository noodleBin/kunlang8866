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

#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_fsm.h"

namespace century {
namespace planning {

OverTakeStatus TickCheck::state_ = OverTakeStatus::DEFAULT;
uint64_t TickCheck::tick_ = 0;

using std::placeholders::_1;
using std::placeholders::_2;

OverTakeFsm::OverTakeFsm() {}

void OverTakeFsm::InitOverTakeFsm(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector) {
  status_ = OverTakeStatus::DEFAULT;

  fsm_condition_ = std::make_shared<FsmCondition>(config, injector);
  fsm_action_ = std::make_shared<FsmAction>(config, injector);

  std::map<OverTakeStatus, Conditions> conds = {
      {OverTakeStatus::DEFAULT,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmCondition::Default2Default, fsm_condition_, _1, _2)},
           {OverTakeStatus::PREPARE,
            std::bind(&FsmCondition::Default2Prepare, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::PREPARE,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmCondition::Prepare2Default, fsm_condition_, _1, _2)},
           {OverTakeStatus::TURN,
            std::bind(&FsmCondition::Prepare2Turn, fsm_condition_, _1, _2)},
           {OverTakeStatus::PREPARE,
            std::bind(&FsmCondition::Prepare2Prepare, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::TURN,
       {
           {OverTakeStatus::RETURN,
            std::bind(&FsmCondition::Turn2Return, fsm_condition_, _1, _2)},
           {OverTakeStatus::OVERTAKE,
            std::bind(&FsmCondition::Turn2Overtake, fsm_condition_, _1, _2)},
           {OverTakeStatus::TURN,
            std::bind(&FsmCondition::Turn2Turn, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::OVERTAKE,
       {
           {OverTakeStatus::FINISH,
            std::bind(&FsmCondition::Overtake2Finish, fsm_condition_, _1, _2)},
           {OverTakeStatus::FAIL,
            std::bind(&FsmCondition::Overtake2Fail, fsm_condition_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmCondition::Overtake2Return, fsm_condition_, _1, _2)},
           {OverTakeStatus::OVERTAKE,
            std::bind(&FsmCondition::Overtake2Overtake, fsm_condition_, _1,
                      _2)},
       }},
      {OverTakeStatus::RETURN,
       {
           {OverTakeStatus::FINISH,
            std::bind(&FsmCondition::Return2Finish, fsm_condition_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmCondition::Return2Return, fsm_condition_, _1, _2)},
           {OverTakeStatus::HOLD,
            std::bind(&FsmCondition::Return2Hold, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::HOLD,
       {
           {OverTakeStatus::FAIL,
            std::bind(&FsmCondition::Hold2Fail, fsm_condition_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmCondition::Hold2Return, fsm_condition_, _1, _2)},
           {OverTakeStatus::HOLD,
            std::bind(&FsmCondition::Hold2Hold, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::FINISH,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmCondition::Finish2Default, fsm_condition_, _1, _2)},
           {OverTakeStatus::FINISH,
            std::bind(&FsmCondition::Finish2Finish, fsm_condition_, _1, _2)},
       }},
      {OverTakeStatus::FAIL,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmCondition::Fail2Default, fsm_condition_, _1, _2)},
           {OverTakeStatus::FAIL,
            std::bind(&FsmCondition::Fail2Fail, fsm_condition_, _1, _2)},
       }}};

  std::map<OverTakeStatus, Actions> acts = {
      {OverTakeStatus::DEFAULT,
       {
           {OverTakeStatus::PREPARE,
            std::bind(&FsmAction::Default2Prepare, fsm_action_, _1, _2)},
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmAction::Default2Default, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::PREPARE,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmAction::Prepare2Default, fsm_action_, _1, _2)},
           {OverTakeStatus::TURN,
            std::bind(&FsmAction::Prepare2Turn, fsm_action_, _1, _2)},
           {OverTakeStatus::PREPARE,
            std::bind(&FsmAction::Prepare2Prepare, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::TURN,
       {
           {OverTakeStatus::RETURN,
            std::bind(&FsmAction::Turn2Return, fsm_action_, _1, _2)},
           {OverTakeStatus::OVERTAKE,
            std::bind(&FsmAction::Turn2Overtake, fsm_action_, _1, _2)},
           {OverTakeStatus::TURN,
            std::bind(&FsmAction::Turn2Turn, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::OVERTAKE,
       {
           {OverTakeStatus::FAIL,
            std::bind(&FsmAction::Overtake2Fail, fsm_action_, _1, _2)},
           {OverTakeStatus::FINISH,
            std::bind(&FsmAction::Overtake2Finish, fsm_action_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmAction::Overtake2Return, fsm_action_, _1, _2)},
           {OverTakeStatus::OVERTAKE,
            std::bind(&FsmAction::Overtake2Overtake, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::RETURN,
       {
           {OverTakeStatus::FINISH,
            std::bind(&FsmAction::Return2Finish, fsm_action_, _1, _2)},
           {OverTakeStatus::HOLD,
            std::bind(&FsmAction::Return2Hold, fsm_action_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmAction::Return2Return, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::HOLD,
       {
           {OverTakeStatus::FAIL,
            std::bind(&FsmAction::Hold2Fail, fsm_action_, _1, _2)},
           {OverTakeStatus::RETURN,
            std::bind(&FsmAction::Hold2Return, fsm_action_, _1, _2)},
           {OverTakeStatus::HOLD,
            std::bind(&FsmAction::Hold2Hold, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::FINISH,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmAction::Finish2Default, fsm_action_, _1, _2)},
           {OverTakeStatus::FINISH,
            std::bind(&FsmAction::Finish2Finish, fsm_action_, _1, _2)},
       }},
      {OverTakeStatus::FAIL,
       {
           {OverTakeStatus::DEFAULT,
            std::bind(&FsmAction::Fail2Default, fsm_action_, _1, _2)},
           {OverTakeStatus::FAIL,
            std::bind(&FsmAction::Fail2Fail, fsm_action_, _1, _2)},
       }}};

  conditons_.swap(conds);
  actions_.swap(acts);
  AINFO << "[JJZ][FSM] InitOverTakeFsm sucess!";
  return;
}

OverTakeStatus OverTakeFsm::UpdateStatus(
    const OverTakeStatus& pre_status, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  for (auto& cond : conditons_[pre_status]) {
    if (cond.second(frame, reference_line_info)) {
      return cond.first;
    }
  }
  return OverTakeStatus::FAIL;
}

void OverTakeFsm::ExecuteFsm(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  actions_[status_][status_](frame, reference_line_info);
  OverTakeStatus current_state =
      UpdateStatus(status_, frame, reference_line_info);
  if (current_state != status_) {
    actions_[status_][current_state](frame, reference_line_info);
  }
  // AINFO << "[JJZ][OverTake][FSM] pre_state: " << static_cast<int>(status_)
  //       << ", current_state: " << static_cast<int>(current_state);
  status_ = current_state;
  TickCheck::Set(status_);
  frame->SetOvertakeReportState(Transfer2ReportOvertakeState(status_));
  return;
}

}  // namespace planning
}  // namespace century
