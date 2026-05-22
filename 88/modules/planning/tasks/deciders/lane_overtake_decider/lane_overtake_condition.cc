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

#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_condition.h"

#include <memory>
#include <string>

#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::cyber::Clock;

namespace {
constexpr int kFinishStatusKeepTimes = 10;
constexpr int kFailStatusKeepTimes = 30;
}  // namespace

FsmCondition::FsmCondition(const TaskConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {}

bool FsmCondition::Default2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& overtake_info =
      injector_->planning_context()->planning_status().overtake();
  const auto& config = config_.lane_overtake_decider_config();
  auto check_times = overtake_info.is_vehicle_type()
                         ? config.slow_vehicle_type_obs_total_times()
                         : config.slow_obs_total_times();
  if (overtake_info.slow_obs_keep_check_times() < check_times) {
    return true;
  }
  return false;
}

bool FsmCondition::Default2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& overtake_info =
      injector_->planning_context()->planning_status().overtake();
  const auto& config = config_.lane_overtake_decider_config();
  auto check_times = overtake_info.is_vehicle_type()
                         ? config.slow_vehicle_type_obs_total_times()
                         : config.slow_obs_total_times();
  if (overtake_info.slow_obs_keep_check_times() >= check_times) {
    return true;
  }
  return false;
}

bool FsmCondition::Prepare2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& overtake_info =
      injector_->planning_context()->planning_status().overtake();
  auto& overtake_obs_id = overtake_info.overtake_obstacle_id();
  const auto* overtake_obs =
      reference_line_info->path_decision()->Find(overtake_obs_id);
  if (!overtake_obs) {
    std::string overtake_obs_perception_id =
        std::to_string(overtake_info.overtake_obstacle_perception_id());
    overtake_obs =
        reference_line_info->path_decision()->Find(overtake_obs_perception_id);
  }
  const auto& config = config_.lane_overtake_decider_config();
  auto check_times = overtake_info.is_vehicle_type()
                         ? config.slow_vehicle_type_obs_total_times()
                         : config.slow_obs_total_times();
  if (!overtake_obs ||
      overtake_info.slow_obs_keep_check_times() < check_times) {
    return true;
  }
  return false;
}

bool FsmCondition::Prepare2Turn(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  cancel_turn_count_ = 0;
  auto& overtake_info =
      injector_->planning_context()->planning_status().overtake();
  return overtake_info.history_overtake_signal() &&
         util::IsLaneChange(injector_->planning_context());
}

bool FsmCondition::Prepare2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  return true;
}

bool FsmCondition::Turn2Return(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  if (reference_line_info->IsOriginalRefLine(true) &&
      !reference_line_info->AdcIsOnRouteLane(injector_->planning_context())) {
    AINFO << "route lane change when turn, need cancel overtake!";
    return true;
  }
  if (!util::IsLaneChange(injector_->planning_context())) {
    mutable_overtake->set_stop_change_lane(false);
    return false;
  }
  if (BlockStopForLong(frame, reference_line_info)) {
    return true;
  }
  const auto& config = config_.lane_overtake_decider_config();
  bool slow_obs_cancel = (mutable_overtake->slow_obs_keep_check_times() <
                          -config.cancel_turn_slow_obs_total_times());
  bool safe_check_cancel =
      (mutable_overtake->safe_check_times() < -FLAGS_lc_safe_check_times);
  bool need_cancel = (slow_obs_cancel || safe_check_cancel);
  ADEBUG << "slow_obs_cancel: " << slow_obs_cancel
         << ", safe_check_cancel: " << safe_check_cancel
         << ", need_cancel: " << need_cancel;
  if (!need_cancel) {
    mutable_overtake->set_stop_change_lane(false);
    return false;
  }
  if (!mutable_overtake->stop_change_lane()) {
    mutable_overtake->set_stop_change_lane(true);
    mutable_overtake->set_turn_timestamp(Clock::NowInSeconds());
  }

  for (const auto& reference_line_info : frame->reference_line_info()) {
    if (reference_line_info.IsChangeLanePath()) {
      if (reference_line_info.GetIsClearToChangeLane() && slow_obs_cancel) {
        cancel_turn_count_ = 0;
        mutable_overtake->set_stop_change_lane(false);
        return false;
      }
    } else {
      if (reference_line_info.IsAdcPostureStraight() && safe_check_cancel) {
        return true;
      }
      if (reference_line_info.GetIsClearToChangeLane()) {
        ++cancel_turn_count_;
      } else {
        cancel_turn_count_ = 0;
      }
    }
  }

  double start_turn_timestamp = mutable_overtake->turn_timestamp();
  double time_diff = Clock::NowInSeconds() - start_turn_timestamp;
  if (cancel_turn_count_ > config.cancel_turn_safe_count() ||
      time_diff > config.allow_turn_sustain_time()) {
    cancel_turn_count_ = 0;
    return true;
  }
  return false;
}

bool FsmCondition::Turn2Overtake(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  cancel_turn_count_ = 0;
  return !util::IsLaneChange(injector_->planning_context());
}

bool FsmCondition::Turn2Turn(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  return true;
}

bool FsmCondition::Overtake2Finish(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  if ((!mutable_overtake->history_overtake_signal() &&
       frame->reference_line_info().size() > 1) ||
      reference_line_info->AdcIsOnRouteLane(injector_->planning_context())) {
    mutable_overtake->set_urgency_lane_change(true);
    mutable_overtake->set_overtake_obstacle_id("");
    mutable_overtake->set_overtake_obstacle_perception_id(0);
    return true;
  }
  return false;
}

bool FsmCondition::Overtake2Fail(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  return false;
}

bool FsmCondition::Overtake2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  return util::IsLaneChange(injector_->planning_context());
}

bool FsmCondition::Overtake2Overtake(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  return true;
}

bool FsmCondition::Return2Finish(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  return !util::IsLaneChange(injector_->planning_context());
}

bool FsmCondition::Return2Hold(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  return false;
}

bool FsmCondition::Return2Return(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  if (frame->reference_line_info().size() < 2) {
    auto* mutable_overtake = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_overtake();
    mutable_overtake->set_overtake_obstacle_id("");
    mutable_overtake->set_overtake_obstacle_perception_id(0);
  }
  return true;
}

bool FsmCondition::Hold2Fail(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  return false;
}

bool FsmCondition::Hold2Return(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  return false;
}

bool FsmCondition::Hold2Hold(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  return true;
}

bool FsmCondition::Finish2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (TickCheck::Tick(OverTakeStatus::FINISH) >= kFinishStatusKeepTimes) {
    return true;
  }
  return false;
}

bool FsmCondition::Finish2Finish(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_overtake_obstacle_id("");
  mutable_overtake->set_overtake_obstacle_perception_id(0);
  return true;
}

bool FsmCondition::Fail2Default(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  if (TickCheck::Tick(OverTakeStatus::FAIL) >= kFailStatusKeepTimes) {
    return true;
  }
  return false;
}

bool FsmCondition::Fail2Fail(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  return true;
}

bool FsmCondition::BlockStopForLong(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  const auto& config = config_.lane_overtake_decider_config();
  auto adc_v = reference_line_info->vehicle_state().linear_velocity();
  if (adc_v > config.block_stop_velocity_threshold()) {
    mutable_overtake->set_start_stop_time(Clock::NowInSeconds());
    return false;
  }
  double start_stop_time = mutable_overtake->start_stop_time();
  double time_diff = Clock::NowInSeconds() - start_stop_time;
  for (const auto& reference_line : frame->reference_line_info()) {
    if (!reference_line.IsChangeLanePath()) {
      if (reference_line.IsAdcCenterInLane() &&
          time_diff > config.block_stop_total_time_threshold()) {
        AINFO << "Adc block stop for too long, need cancel overtake";
        return true;
      }
    }
  }
  return false;
}

}  // namespace planning
}  // namespace century
