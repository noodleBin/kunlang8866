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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_condition.h"

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
}  // namespace

LaneBorrowFsmCondition::LaneBorrowFsmCondition(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {}

bool LaneBorrowFsmCondition::Default2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (util::IsLaneChange(injector_->planning_context())) {
    // return true;
  }
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  AINFO << "condition " << __func__
        << "  laneborrow obstacle id "
        << laneborrow_info.laneborrow_obstacle_id()
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  is_in_crowd_traffic "
        << path_decider_status.is_in_crowd_traffic();
  // no obstacle | no neighbour lane to borrow
  if (laneborrow_info.laneborrow_obstacle_id().empty() ||
      !laneborrow_info.is_necessary_to_laneborrow() ||
      path_decider_status.is_in_crowd_traffic()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Default2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (util::IsLaneChange(injector_->planning_context())) {
    // return false;
  }
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  AINFO << "condition " << __func__
        << "  laneborrow obstacle id "
        << laneborrow_info.laneborrow_obstacle_id()
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  is_in_crowd_traffic "
        << path_decider_status.is_in_crowd_traffic();
  if (!laneborrow_info.laneborrow_obstacle_id().empty() &&
      laneborrow_info.is_necessary_to_laneborrow() &&
      !path_decider_status.is_in_crowd_traffic()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Prepare2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (util::IsLaneChange(injector_->planning_context())) {
    // return true;
  }
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  AINFO << "condition " << __func__
        << "  slow obs disappear check times "
        << laneborrow_info.slow_obs_disappear_check_times()
        << "  is_in_crowd_traffic "
        << path_decider_status.is_in_crowd_traffic();
  if (laneborrow_info.slow_obs_disappear_check_times() >
      config_.path_lane_borrow_decider_config().slow_obs_threshold_times() ||
      path_decider_status.is_in_crowd_traffic()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Prepare2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  AINFO << "condition " << __func__
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  laneborrow_direction "
        << laneborrow_info.laneborrow_direction()
        << "  is_left_borrow_safe "
        << laneborrow_info.is_left_borrow_safe()
        << "  slow_obs_keep_check_times "
        << laneborrow_info.slow_obs_keep_check_times();
  if ((laneborrow_info.is_necessary_to_laneborrow() ||
       laneborrow_info.is_need_to_laneborrow_in_play_street() ||
       laneborrow_info.is_need_laneborrow_in_auxiliary_road()) &&
      laneborrow_info.slow_obs_keep_check_times() >
          config_.path_lane_borrow_decider_config().slow_obs_threshold_times() &&
      LaneborrowStatus::LEFT_BORROW == laneborrow_info.laneborrow_direction() &&
      laneborrow_info.is_left_borrow_safe()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Prepare2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  AINFO << "condition " << __func__
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  laneborrow_direction "
        << laneborrow_info.laneborrow_direction()
        << "  is_right_borrow_safe "
        << laneborrow_info.is_right_borrow_safe()
        << "  slow_obs_keep_check_times "
        << laneborrow_info.slow_obs_keep_check_times();
  if ((laneborrow_info.is_necessary_to_laneborrow() ||
       laneborrow_info.is_need_to_laneborrow_in_play_street()) &&
      laneborrow_info.slow_obs_keep_check_times() >
          config_.path_lane_borrow_decider_config().slow_obs_threshold_times() &&
      LaneborrowStatus::RIGHT_BORROW == laneborrow_info.laneborrow_direction() &&
      laneborrow_info.is_right_borrow_safe()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Prepare2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO << "condition " << __func__;
  return true;
}

bool LaneBorrowFsmCondition::LeftBorrow2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  AINFO << "condition " << __func__
        << "  slow_obs_disappear_check_times "
        << laneborrow_info.slow_obs_disappear_check_times()
        << "  is_able_to_self_borrow "
        << laneborrow_info.is_able_to_self_borrow();

  if (FLAGS_enable_near_junction_laneborrow &&
      path_decider.is_adc_near_junction()) {
    bool exit_leftborrow_near_junction =
        (laneborrow_info.is_able_to_self_borrow() ||
         laneborrow_info.force_to_exit_laneborrow()) &&
        !laneborrow_info.is_need_remote_request();
    if (exit_leftborrow_near_junction) {
      AINFO << "transit [LeftBorrow] to [Return] near junction";
      return true;
    }
  } else if (injector_->is_auxiliary_road_) {
    bool exit_leftborrow_in_auxiliary_road =
        laneborrow_info.is_able_to_self_borrow();
    if (exit_leftborrow_in_auxiliary_road) {
      AINFO << "transit [LeftBorrow] to [Return] in auxiliary road";
      return true;
    }
  } else if (laneborrow_info.slow_obs_disappear_check_times() >
             config_.path_lane_borrow_decider_config()
                 .slow_obs_threshold_times() &&
             laneborrow_info.is_able_to_self_borrow()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::LeftBorrow2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO << "condition " << __func__;
  return true;
}

bool LaneBorrowFsmCondition::RightBorrow2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  AINFO << "condition " << __func__
        << "  slow_obs_disappear_check_time "
        << laneborrow_info.slow_obs_disappear_check_times()
        << "  is_able_to_self_borrow "
        << laneborrow_info.is_able_to_self_borrow();
  if (FLAGS_enable_near_junction_laneborrow &&
      path_decider.is_adc_near_junction()) {
    bool exit_leftborrow_near_junction =
        (laneborrow_info.is_able_to_self_borrow() ||
         laneborrow_info.force_to_exit_laneborrow()) &&
        !laneborrow_info.is_need_remote_request();
    if (exit_leftborrow_near_junction) {
      AINFO << "transit [RightBorrow] to [Return] near junction";
      return true;
    }
  } else if (injector_->is_auxiliary_road_) {
    bool exit_rightborrow_in_auxiliary_road =
        laneborrow_info.is_able_to_self_borrow();
    if (exit_rightborrow_in_auxiliary_road) {
      AINFO << "transit [RightBorrow] to [Return] in auxiliary road";
      return true;
    }
  } else if (laneborrow_info.slow_obs_disappear_check_times() >
                 config_.path_lane_borrow_decider_config()
                     .slow_obs_threshold_times() &&
             laneborrow_info.is_able_to_self_borrow()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::RightBorrow2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO << "condition " << __func__;
  return true;
}

bool LaneBorrowFsmCondition::Return2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  AINFO << "condition " << __func__
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  laneborrow_direction "
        << laneborrow_info.laneborrow_direction()
        << "  is_left_borrow_safe "
        << laneborrow_info.is_left_borrow_safe()
        << "  slow_obs_keep_check_times "
        << laneborrow_info.slow_obs_keep_check_times();
  if (laneborrow_info.slow_obs_keep_check_times() >
          config_.path_lane_borrow_decider_config()
              .slow_obs_threshold_times() &&
      laneborrow_info.is_necessary_to_laneborrow() &&
      LaneborrowStatus::LEFT_BORROW ==
          laneborrow_info.laneborrow_direction() &&
      laneborrow_info.is_left_borrow_safe()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Return2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  AINFO << "condition " << __func__
        << "  is_necessary_to_laneborrow "
        << laneborrow_info.is_necessary_to_laneborrow()
        << "  laneborrow_direction "
        << laneborrow_info.laneborrow_direction()
        << "  is_right_borrow_safe "
        << laneborrow_info.is_right_borrow_safe()
        << "  slow_obs_keep_check_times "
        << laneborrow_info.slow_obs_keep_check_times();
  if (laneborrow_info.slow_obs_keep_check_times() >
          config_.path_lane_borrow_decider_config()
              .slow_obs_threshold_times() &&
      laneborrow_info.is_necessary_to_laneborrow() &&
      LaneborrowStatus::RIGHT_BORROW ==
          laneborrow_info.laneborrow_direction() &&
      laneborrow_info.is_right_borrow_safe()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Return2Finish(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto& laneborrow_info =
      injector_->planning_context()->planning_status().lane_borrow();
  AINFO << "condition " << __func__
        << "  slow_obs_disappear_check_times "
        << laneborrow_info.slow_obs_disappear_check_times()
        << "  is_adc_posture_straight "
        << reference_line_info->IsAdcPostureStraight();
  if (laneborrow_info.slow_obs_disappear_check_times() >
          config_.path_lane_borrow_decider_config()
              .slow_obs_threshold_times() &&
      reference_line_info->IsAdcPostureStraight()) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmCondition::Return2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO << "condition " << __func__;
  return true;
}

bool LaneBorrowFsmCondition::Finish2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO << "condition " << __func__
        << "  tick " << LaneBorrowTickCheck::Tick(LaneBorrowStatus::FINISH);

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  const auto& reference_point =
      reference_line_info->reference_line().GetReferencePoint(center_s);
  double ref_heading = reference_point.heading();
  double adc_heading = reference_line_info->vehicle_state().heading();
  double diff_heading =
      std::fabs(common::math::NormalizeAngle(ref_heading - adc_heading));
  double diff_heading_between_adc_and_startpoint =
      std::fabs(common::math::NormalizeAngle(
          reference_line_info->InitPointHeading() - adc_heading));
  // AINFO << " reference_line_info->InitPointHeading() = "
  //       << reference_line_info->InitPointHeading();
  // AINFO << "center_l  = " << center_l;
  // AINFO << "diff_heading = " << diff_heading;
  // AINFO << "diff_heading_between_adc_and_startpoint = "
  //       << diff_heading_between_adc_and_startpoint;
  bool is_in_ref_line = std::fabs(center_l) < FLAGS_lateral_error 
                        && diff_heading < FLAGS_same_heading 
                        && diff_heading_between_adc_and_startpoint < FLAGS_same_heading;
  bool is_stop = reference_line_info->vehicle_state().linear_velocity()<0.1;
  bool can_change_state = is_in_ref_line || is_stop;
  // In the borrow state, manual control is performed to shake back to the obstacle.
  // There is no state machine for exiting the row when switching to diagonal mode, 
  // so it automatically exits when a new parking state is needed
  if (LaneBorrowTickCheck::Tick(LaneBorrowStatus::FINISH) >=
          kFinishStatusKeepTimes &&
      can_change_state) {
    AINFO<<"FINISH TRUE";
    return true;
  }
  AINFO<<"no finish";
  return false;
}

bool LaneBorrowFsmCondition::Finish2Finish(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  AINFO "condition " << __func__;
  return true;
}

}  // namespace planning
}  // namespace century
