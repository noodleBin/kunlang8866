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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_fsm.h"

namespace century {
namespace planning {

LaneBorrowStatus LaneBorrowTickCheck::state_ = LaneBorrowStatus::DEFAULT;
uint64_t LaneBorrowTickCheck::tick_ = 0;

using std::placeholders::_1;
using std::placeholders::_2;

void LaneBorrowFsm::InitLaneBorrowFsm(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector) {
  const auto& lane_borrow_status = injector->planning_context()
                                       ->planning_status()
                                       .lane_borrow()
                                       .lane_borrow_status();
  status_ = static_cast<LaneBorrowStatus>(lane_borrow_status);

  fsm_condition_ = std::make_shared<LaneBorrowFsmCondition>(config, injector);
  fsm_action_ = std::make_shared<LaneBorrowFsmAction>(config, injector);

  std::map<LaneBorrowStatus, Conditions> conds = {
      {LaneBorrowStatus::DEFAULT,
       {
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmCondition::Default2Default,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::PREPARE,
            std::bind(&LaneBorrowFsmCondition::Default2Prepare,
                      fsm_condition_, _1, _2)},
       }},
      {LaneBorrowStatus::PREPARE,
       {
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmCondition::Prepare2Default,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmCondition::Prepare2LeftBorrow,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmCondition::Prepare2RightBorrow,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::PREPARE,
            std::bind(&LaneBorrowFsmCondition::Prepare2Prepare,
                      fsm_condition_, _1, _2)},
       }},
      {LaneBorrowStatus::LEFTBORROW,
       {
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmCondition::LeftBorrow2Return,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmCondition::LeftBorrow2LeftBorrow,
                      fsm_condition_, _1, _2)},
       }},
      {LaneBorrowStatus::RIGHTBORROW,
       {
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmCondition::RightBorrow2Return,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmCondition::RightBorrow2RightBorrow,
                      fsm_condition_, _1, _2)},
       }},
      {LaneBorrowStatus::RETURN,
       {
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmCondition::Return2LeftBorrow,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmCondition::Return2RightBorrow,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::FINISH,
            std::bind(&LaneBorrowFsmCondition::Return2Finish,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmCondition::Return2Return,
                      fsm_condition_, _1, _2)},
       }},
      {LaneBorrowStatus::FINISH,
       {
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmCondition::Finish2Default,
                      fsm_condition_, _1, _2)},
           {LaneBorrowStatus::FINISH,
            std::bind(&LaneBorrowFsmCondition::Finish2Finish,
                      fsm_condition_, _1, _2)},
       }}};

  std::map<LaneBorrowStatus, Actions> acts = {
      {LaneBorrowStatus::DEFAULT,
       {
           {LaneBorrowStatus::PREPARE,
            std::bind(&LaneBorrowFsmAction::Default2Prepare,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmAction::Default2Default,
                      fsm_action_, _1, _2)},
       }},
      {LaneBorrowStatus::PREPARE,
       {
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmAction::Prepare2Default,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmAction::Prepare2LeftBorrow,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmAction::Prepare2RightBorrow,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::PREPARE,
            std::bind(&LaneBorrowFsmAction::Prepare2Prepare,
                      fsm_action_, _1, _2)},
       }},
      {LaneBorrowStatus::LEFTBORROW,
       {
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmAction::LeftBorrow2Return,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmAction::LeftBorrow2LeftBorrow,
                      fsm_action_, _1, _2)},
       }},
      {LaneBorrowStatus::RIGHTBORROW,
       {
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmAction::RightBorrow2Return,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmAction::RightBorrow2RightBorrow,
                      fsm_action_, _1, _2)},
       }},
      {LaneBorrowStatus::RETURN,
       {
           {LaneBorrowStatus::LEFTBORROW,
            std::bind(&LaneBorrowFsmAction::Return2LeftBorrow,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::RIGHTBORROW,
            std::bind(&LaneBorrowFsmAction::Return2RightBorrow,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::FINISH,
            std::bind(&LaneBorrowFsmAction::Return2Finish,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::RETURN,
            std::bind(&LaneBorrowFsmAction::Return2Return,
                      fsm_action_, _1, _2)},
       }},
      {LaneBorrowStatus::FINISH,
       {
           {LaneBorrowStatus::DEFAULT,
            std::bind(&LaneBorrowFsmAction::Finish2Default,
                      fsm_action_, _1, _2)},
           {LaneBorrowStatus::FINISH,
            std::bind(&LaneBorrowFsmAction::Finish2Finish,
                      fsm_action_, _1, _2)},
       }}};

  conditions_.swap(conds);
  actions_.swap(acts);
  AINFO << "[FSM][LaneBorrow] InitLaneBorrowFsm sucess!"
        << " Initial status: " << Status2String(status_);
  return;
}

LaneBorrowStatus LaneBorrowFsm::UpdateStatus(
    const LaneBorrowStatus& pre_status, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  for (auto& cond : conditions_[pre_status]) {
    if (cond.second(frame, reference_line_info)) {
      return cond.first;
    }
  }
  return LaneBorrowStatus::DEFAULT;
}

void LaneBorrowFsm::ExecuteFsm(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  // if has two stacker ,face new stacker ,need update borrow state
  if(reference_line_info->IsNewStackerSesponse()){
    AINFO<<"IS NEW STACKER INIT BORROW STATE";
      status_ = LaneBorrowStatus::DEFAULT;
  }
  // need reset borrow satte for igv
  if (reference_line_info->IsNeedResetBorrowState()) {
    status_ = LaneBorrowStatus::DEFAULT;
  }

  actions_[status_][status_](frame, reference_line_info);
  LaneBorrowStatus current_state =
      UpdateStatus(status_, frame, reference_line_info);
  if (current_state != status_) {
    actions_[status_][current_state](frame, reference_line_info);
  }
  AINFO << "[LaneBorrow][FSM] pre_state: " << Status2String(status_)
        << ", current_state: " << Status2String(current_state);
  status_ = current_state;
  LaneBorrowTickCheck::Set(status_);
//   frame->SetLaneBorrowReportState(Transfer2ReportLaneBorrowState(status_));
  return;
}

}  // namespace planning
}  // namespace century
