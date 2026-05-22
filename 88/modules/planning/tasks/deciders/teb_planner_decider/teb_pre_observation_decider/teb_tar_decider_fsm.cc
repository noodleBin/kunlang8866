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
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_decider_fsm.h"
namespace century {
namespace planning {
namespace {
using std::placeholders::_1;
using std::placeholders::_2;
}  // namespace
TEBTarStatus TarTickCheck::tar_state_ = TEBTarStatus::NORMAL;
uint64_t TarTickCheck::tar_tick_ = 0;
uint64_t TarTickCheck::tar_big_tick_ = 0;

void TEBTarDeciderFsm::InitTarFsm(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector) {
  status_ = TEBTarStatus::NORMAL;
  fsm_condition_ = std::make_shared<TarFsmCondition>();
  fsm_action_ = std::make_shared<TarFsmAction>();
  std::map<TEBTarStatus, Conditions> conds = {
      {TEBTarStatus::NORMAL,
       {
           {TEBTarStatus::CREEP,
            std::bind(&TarFsmCondition::Normal2Creep, fsm_condition_, _1, _2)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmCondition::Normal2Stop, fsm_condition_, _1, _2)},
       }},
      {TEBTarStatus::CREEP,
       {
           {TEBTarStatus::NORMAL,
            std::bind(&TarFsmCondition::Creep2Normal, fsm_condition_, _1, _2)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmCondition::Creep2Stop, fsm_condition_, _1, _2)},
       }},
      {TEBTarStatus::STOP,
       {
           {TEBTarStatus::NORMAL,
            std::bind(&TarFsmCondition::Stop2Normal, fsm_condition_, _1, _2)},
           {TEBTarStatus::YIELD,
            std::bind(&TarFsmCondition::Stop2Yield, fsm_condition_, _1, _2)},
       }},
      {TEBTarStatus::YIELD,
       {
           {TEBTarStatus::FAIL,
            std::bind(&TarFsmCondition::Yield2Fail, fsm_condition_, _1, _2)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmCondition::Yield2Stop, fsm_condition_, _1, _2)},
       }},
  };
  // {
  //     TEBTarStatus::FAIL,
  //     {{TEBTarStatus::NORMAL,
  //       std::bind(&TarFsmCondition:, fsm_condition_, _1, _2)}},
  // },
  std::map<TEBTarStatus, Actions> acts = {
      {TEBTarStatus::NORMAL,
       {
           {TEBTarStatus::NORMAL,
            std::bind(&TarFsmAction::Normal2Normal, fsm_action_, _1)},
           {TEBTarStatus::CREEP,
            std::bind(&TarFsmAction::Normal2Creep, fsm_action_, _1)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmAction::Normal2Stop, fsm_action_, _1)},
       }},
      {TEBTarStatus::CREEP,
       {
           {TEBTarStatus::CREEP,
            std::bind(&TarFsmAction::Creep2Creep, fsm_action_, _1)},
           {TEBTarStatus::NORMAL,
            std::bind(&TarFsmAction::Creep2Normal, fsm_action_, _1)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmAction::Creep2Stop, fsm_action_, _1)},
       }},
      {TEBTarStatus::STOP,
       {
           {TEBTarStatus::STOP,
            std::bind(&TarFsmAction::Stop2Stop, fsm_action_, _1)},
           {TEBTarStatus::NORMAL,
            std::bind(&TarFsmAction::Stop2Normal, fsm_action_, _1)},
           {TEBTarStatus::YIELD,
            std::bind(&TarFsmAction::Stop2Yield, fsm_action_, _1)},
       }},
      {TEBTarStatus::YIELD,
       {
           {TEBTarStatus::FAIL,
            std::bind(&TarFsmAction::Yield2Fail, fsm_action_, _1)},
           {TEBTarStatus::YIELD,
            std::bind(&TarFsmAction::Yield2Yield, fsm_action_, _1)},
           {TEBTarStatus::STOP,
            std::bind(&TarFsmAction::Yield2Stop, fsm_action_, _1)},
       }},
      {TEBTarStatus::FAIL,
       {
           {TEBTarStatus::FAIL,
            std::bind(&TarFsmAction::Fail2Fail, fsm_action_, _1)},
       }},
  };
  conditons_.swap(conds);
  actions_.swap(acts);
  AINFO << "[dzq][FSM] TARFsm sucess!";
}

TEBTarStatus TEBTarDeciderFsm::UpdateStatus(
    const TEBTarStatus& pre_status, const std::shared_ptr<TarVehicleInfo>& tar,
    Frame* const frame) {
  for (auto& cond : conditons_[pre_status]) {
    // Attention(zhiqiang.ding): conditions may have overlap.
    if (cond.second(tar, frame)) {
      return cond.first;
    }
  }
  return TEBTarStatus::FAIL;
}

void TEBTarDeciderFsm::ExecuteFsm(const std::shared_ptr<TarVehicleInfo>& tar,
                                  Frame* const frame) {
  actions_[status_][status_](frame);
  TEBTarStatus current_state = UpdateStatus(status_, tar, frame);
  if (current_state != status_) {
    actions_[status_][current_state](frame);
  }
  AINFO << "[dzq][TEBTAR][FSM] pre_state: " << static_cast<uint8_t>(status_)
        << ", current_state: " << static_cast<uint8_t>(current_state);
  status_ = current_state;
  TarTickCheck::Set(status_);
  // report
  //   frame->SetOvertakeReportState(Transfer2ReportOvertakeState(status_));
  return;
}

void TEBTarDeciderFsm::ExecuteFsmRude(const std::shared_ptr<TarVehicleInfo> tar,
                                      Frame* const frame) {
  TEBTarStatus pre_status = status_;
  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);
    // TODO(zhiqiang.ding): add keep status action.
    fsm_action_->DoSelf();
    actions_[pre_status][status_](frame);
    // merge conditions with actions.
    switch (pre_status) {
      case TEBTarStatus::NORMAL: {
        AINFO << "ExecuteFsmRude "
              << "Normal";
        if (fsm_condition_->Normal2Creep(tar, frame)) {
          status_ = TEBTarStatus::CREEP;
          fsm_action_->Normal2Creep(frame);
          // break;
        }
        if (fsm_condition_->Normal2Stop(tar, frame)) {
          status_ = TEBTarStatus::STOP;
          fsm_action_->Normal2Stop(frame);
          break;
        }

        break;
      }
      case TEBTarStatus::CREEP: {
        AINFO << "ExecuteFsmRude "
              << "CREEP";
        if (fsm_condition_->Creep2Stop(tar, frame)) {
          status_ = TEBTarStatus::STOP;
          fsm_action_->Creep2Stop(frame);
          break;
        }
        if (fsm_condition_->Creep2Normal(tar, frame)) {
          status_ = TEBTarStatus::NORMAL;
          fsm_action_->Creep2Normal(frame);
          break;
        }
        break;
      }
      case TEBTarStatus::STOP: {
        AINFO << "ExecuteFsmRude "
              << "STOP";
        if (fsm_condition_->Stop2Normal(tar, frame)) {
          status_ = TEBTarStatus::NORMAL;
          fsm_action_->Stop2Normal(frame);
          break;
        }
        if (fsm_condition_->Stop2Yield(tar, frame)) {
          status_ = TEBTarStatus::YIELD;
          fsm_action_->Stop2Yield(frame);
          break;
        }
        break;
      }
      case TEBTarStatus::YIELD: {
        AINFO << "ExecuteFsmRude "
              << "YIELD";
        if (fsm_condition_->Yield2Fail(tar, frame)) {
          status_ = TEBTarStatus::FAIL;
          fsm_action_->Yield2Fail(frame);
          break;
        }
        if (fsm_condition_->Yield2Stop(tar, frame)) {
          status_ = TEBTarStatus::STOP;
          fsm_action_->Yield2Stop(frame);
          break;
        }
        break;
      }
      case TEBTarStatus::FAIL: {
        // TODO(zhiqiang.ding): report.
        break;
      }
      default: {
        AERROR << "Exception error occurs in TarFsm!";
        status_ = TEBTarStatus::NORMAL;
        break;
      }
    }
  }
  AINFO << "tar status: " << static_cast<int>(status_);
  AINFO << "frame tar status: "
        << static_cast<int>(frame->open_space_info().tar_status());
  if (status_ == TEBTarStatus::FAIL) {
    return;
  }
  TarTickCheck::Set(status_);
  AINFO << "tick: " << (+TarTickCheck::Tick(status_));
  if (status_ != TEBTarStatus::NORMAL &&
      kNanNormalTarStatusTimeMax < TarTickCheck::Tick(status_)) {
    AERROR << "Tar fsm fail due to timeout.";
    TarTickCheck::Set(TEBTarStatus::NORMAL);  // just clear
    status_ = TEBTarStatus::FAIL;
    return;
  }
  if (status_ != TEBTarStatus::NORMAL &&
      TarTickCheck::BigTick() > kTarStatusTimeMax) {
    AERROR << "Tar fsm fail due to timeout.";
    TarTickCheck::Set(TEBTarStatus::NORMAL);  // just clear
    status_ = TEBTarStatus::FAIL;
    // report.
  }
  return;
}

}  // namespace planning
}  // namespace century
