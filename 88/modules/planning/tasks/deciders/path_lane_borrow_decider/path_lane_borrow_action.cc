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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_action.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::cyber::Clock;
using century::perception::PerceptionObstacle;
using century::planning::ReferenceLineInfo;
using century::planning_internal::LaneBorrowDebug;

namespace {
constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kLonDistance = 10.0;
constexpr double kCheckStepDistance = 2.0;
constexpr int kUseSelfLaneCount = 6;
constexpr double kSlowObsDistance = 10.0;
constexpr double kEpsilon = 0.001;
constexpr double kSameDirectionThr = M_PI * 0.5;
constexpr int32_t kForceToExitLaneBorrowCount = 10;
constexpr double kFrontObsMaxDistance = 60.0;
constexpr double kFrontObsMaxDistanceInJunction = 60.0;
constexpr double kMaxExtraSafeCheckTime = 4.0;
constexpr double kTrafficCrowdRatio = 0.2;
constexpr double kBorrowSpeed = 6.0;
constexpr double kSafeBorrowLateralDistance = 1.0;
constexpr double kTireStackerHeight = 6.0;
constexpr double kWheelCraneLength = 20.0;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kPassBuffer = 1.0;
}  // namespace

LaneBorrowFsmAction::LaneBorrowFsmAction(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {}

void LaneBorrowFsmAction::Default2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  injector_->borro_obs_name_.clear();
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  // initialize laneborrow
  mutable_laneborrow->set_slow_obs_keep_check_times(0);
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::DEFAULT);
  mutable_laneborrow->set_laneborrow_obstacle_id("");
  mutable_laneborrow->set_laneborrow_obstacle_perception_id(0);
  mutable_laneborrow->set_is_left_borrowable(false);
  mutable_laneborrow->set_is_right_borrowable(false);
  mutable_laneborrow->set_is_in_laneborrow_status(false);
  mutable_laneborrow->set_is_left_borrow_safe(false);
  mutable_laneborrow->set_is_right_borrow_safe(false);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::NO_BORROW);
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  reference_line_info->set_is_path_lane_borrow(false);

  // clear injector_->obs_start_moving_
  injector_->ClearStartMovingObstacles();

  // laneborrow is_left_borrowable / is_right_borrowable
  // need_keep_self_path_straight path_decider  front_static_obstacle_id /
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);

  // laneborrow is_left_borrow_safe / is_right_borrow_safe
  // is_necessary_to_laneborrow path_decider  is_in_path_laneborrow_scenario
  // decided_side_pass_direction
  bool is_necessary_to_laneborrow =
      IsNecessaryToLaneBorrow(frame, reference_line_info);
  if (FLAGS_enable_auto_borrow) {
    if (injector_->borrow_response().has_has_response()) {
      if (injector_->borrow_response().block_obs_id() == "auto borrow") {
        if (ref_static_obs_.empty()) {
          planning::BorrowResponse borrow_response;
          borrow_response.set_response_type(planning::ResponseType::UNTREATED);
          borrow_response.set_block_obs_id("");
          borrow_response.set_has_response(false);
          injector_->set_borrow_response(borrow_response);
        }
      }
    }
  }
  mutable_laneborrow->set_is_necessary_to_laneborrow(
      is_necessary_to_laneborrow);

  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
  mutable_path_decider_status->set_goback_near_junction_check_count(0);
  mutable_path_decider_status->set_is_need_goback_reference_lane(false);

  // laneborrow  laneborrow_obstacle_id / laneborrow_obstacle_perception_id
  //             slow_obs_keep_check_times
  if (!FindLaneBorrowObstacle(reference_line_info)) {
    mutable_path_decider_status->set_goleft_near_junction_check_count(0);
    mutable_path_decider_status->set_is_need_go_left_lane(false);
    has_checked_red_light_ = false;
    PrintDebugInfo(LaneBorrowDebug::DEFAULT, reference_line_info);
    return;
  }

  CheckLaneBorrowInPlayStreet(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::DEFAULT, reference_line_info);

  AINFO << "action " << __func__ << "  laneborrow_obstacle_id "
        << mutable_laneborrow->laneborrow_obstacle_id()
        << "  is_necessary_to_laneborrow "
        << mutable_laneborrow->is_necessary_to_laneborrow()
        << "  is_left_borrowable " << mutable_laneborrow->is_left_borrowable()
        << "  is_right_borrowable " << mutable_laneborrow->is_right_borrowable()
        << "  is_left_borrow_safe " << mutable_laneborrow->is_left_borrow_safe()
        << "  is_right_borrow_safe "
        << mutable_laneborrow->is_right_borrow_safe();

  return;
}

void LaneBorrowFsmAction::Default2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::PREPARE);
  mutable_laneborrow->set_slow_obs_keep_check_times(0);
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_need_exit_laneborrow_in_auxiliary_road(false);
  mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(false);
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Prepare2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::DEFAULT);
  mutable_laneborrow->set_laneborrow_obstacle_id("");
  mutable_laneborrow->set_lane_borrow_lane_id("");
  frame->UpdateLaneBorrowLaneId("");
  mutable_laneborrow->set_laneborrow_obstacle_perception_id(0);
  mutable_laneborrow->set_slow_obs_keep_check_times(0);
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_path_decider()
      ->set_goleft_near_junction_check_count(0);
  has_checked_red_light_ = false;
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Prepare2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_is_in_laneborrow_status(true);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::LEFT_BORROW);
  mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(false);
  reference_line_info->set_is_path_lane_borrow(true);
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_path_decider_status->set_goleft_near_junction_check_count(0);
  mutable_path_decider_status->set_is_need_go_left_lane(false);
  has_checked_red_light_ = false;
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          mutable_laneborrow->laneborrow_obstacle_id());
  if (nullptr != blocking_obstacle) {
    injector_->borro_obs_name_ = mutable_laneborrow->laneborrow_obstacle_id();
    injector_->borrow_obs_consider_l_ =
        blocking_obstacle->PerceptionSLBoundary().end_l();
  }
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Prepare2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_is_in_laneborrow_status(true);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::RIGHT_BORROW);
  reference_line_info->set_is_path_lane_borrow(true);
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          mutable_laneborrow->laneborrow_obstacle_id());
  if (nullptr != blocking_obstacle) {
    injector_->borro_obs_name_ = mutable_laneborrow->laneborrow_obstacle_id();
    injector_->borrow_obs_consider_l_ =
        blocking_obstacle->PerceptionSLBoundary().start_l();
  }
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Prepare2Prepare(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // laneborrow   is_left_borrowable / is_right_borrowable
  // need_keep_left_path_straight path_decider   front_static_obstacle_id
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);

  // laneborrow   is_left_borrow_safe / is_right_borrow_safe
  // is_necessary_to_laneborrow path_decider   is_in_path_laneborrow_scenario
  bool is_necessary_to_laneborrow =
      IsNecessaryToLaneBorrow(frame, reference_line_info);

  // laneborrow   laneborrow_obstacle_id  slow_obs_keep_check_times
  FindLaneBorrowObstacle(reference_line_info);

  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  // laneborrow  slow_obs_disappear_check_times
  if (IsObsDisappearOrFarAway(reference_line_info)) {
    auto check_times = mutable_laneborrow->slow_obs_disappear_check_times();
    mutable_laneborrow->set_slow_obs_disappear_check_times(++check_times);
    AINFO << "obs disappear count: "
          << mutable_laneborrow->slow_obs_disappear_check_times();
    PrintDebugInfo(LaneBorrowDebug::PREPARE, reference_line_info);
    return;
  } else {
    mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  }

  if (mutable_laneborrow->lane_borrow_lane_id().empty()) {
    const std::string lane_borrow_id = reference_line_info->Lanes().Id();
    // mutable_laneborrow->set_lane_borrow_lane_id(lane_borrow_id);
    frame->UpdateLaneBorrowLaneId(mutable_laneborrow->lane_borrow_lane_id());
  }

  reference_line_info->set_is_path_lane_borrow(true);
  if (injector_->is_auxiliary_road_) {
    CheckLaneBorrowInAuxiliaryRoad(frame, reference_line_info);
    PrintDebugInfo(LaneBorrowDebug::PREPARE, reference_line_info);
    return;
  }

  // is_necessary_to_laneborrow / laneborrow_direction
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_laneborrow->set_is_necessary_to_laneborrow(
      is_necessary_to_laneborrow);
  if (is_necessary_to_laneborrow &&
      mutable_path_decider_status->decided_side_pass_direction_size() > 0) {
    if (mutable_path_decider_status->decided_side_pass_direction()[0] ==
        PathDeciderStatus::LEFT_BORROW) {
      mutable_laneborrow->set_laneborrow_direction(
          LaneborrowStatus::LEFT_BORROW);
    } else if (mutable_path_decider_status->decided_side_pass_direction()[0] ==
               PathDeciderStatus::RIGHT_BORROW) {
      mutable_laneborrow->set_laneborrow_direction(
          LaneborrowStatus::RIGHT_BORROW);
    }
  }

  CheckLaneBorrowInPlayStreet(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::PREPARE, reference_line_info);
  AINFO << "action " << __func__ << "  is_necessary_to_laneborrow "
        << mutable_laneborrow->is_necessary_to_laneborrow()
        << "  laneborrow_direction "
        << mutable_laneborrow->laneborrow_direction()
        << "  slow_obs_keep_check_times "
        << mutable_laneborrow->slow_obs_keep_check_times()
        << "  slow_obs_disappear_check_times "
        << mutable_laneborrow->slow_obs_disappear_check_times();
  return;
}

void LaneBorrowFsmAction::LeftBorrow2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::RETURN);
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_need_exit_laneborrow_in_auxiliary_road(false);
  reference_line_info->set_is_path_lane_borrow(true);

  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_force_to_exit_laneborrow_count(0);
  mutable_path_decider_status->set_goleft_near_junction_check_count(0);
  mutable_path_decider_status->set_is_need_go_left_lane(false);
  has_checked_red_light_ = false;
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::LeftBorrow2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // laneborrow  is_left_borrowable / is_right_borrowable
  // need_keep_self_path_straight path_decider  front_static_obstacle_id
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);

  // laneborrow  laneborrow_obstacle_id  slow_obs_keep_check_times
  FindLaneBorrowObstacle(reference_line_info);
  reference_line_info->set_is_path_lane_borrow(true);
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::LEFTBORROW);
  // laneborrow  slow_obs_disappear_check_times
  if (IsObsDisappearOrFarAway(reference_line_info)) {
    auto check_times = mutable_laneborrow->slow_obs_disappear_check_times();
    mutable_laneborrow->set_slow_obs_disappear_check_times(++check_times);
  } else {
    mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  }

  if (FLAGS_enable_near_junction_laneborrow &&
      mutable_path_decider_status->is_adc_near_junction()) {
    CheckNearJunctionExitLaneBorrow(frame, reference_line_info);
  } else if (injector_->is_auxiliary_road_) {
    CheckAuxiliaryRoadExitLaneBorrow(frame, reference_line_info);
  } else {
    // check left laneborrow
    bool is_left_borrowable = true;
    CheckLeftLaneBorrowable(*reference_line_info, &is_left_borrowable);
    if (mutable_path_decider_status->left_reverse_unsafe() &&
        reference_line_info->IsAdcLocatedInLane()) {
      is_left_borrowable = false;
      AINFO << "left reverse unsafe, need cancel borrow!";
    }
    mutable_laneborrow->set_is_left_borrowable(is_left_borrowable);
    mutable_laneborrow->set_is_right_borrowable(false);
    mutable_laneborrow->set_is_left_borrow_safe(is_left_borrowable);
    mutable_laneborrow->set_is_right_borrow_safe(false);
    if (injector_->adc_in_junction_info_.first) {
      double remain_dis = 0.0;
      reference_line_info->GetRemainDistanceToRoutingEndPoint(
          CheckDirection::LEFT, injector_->planning_context(), &remain_dis);
      if (remain_dis < config_.path_lane_borrow_decider_config()
                           .in_junction_routing_check_remain_dis()) {
        mutable_path_decider_status->set_is_need_goback_reference_lane(true);
      }
    }
  }
  mutable_laneborrow->set_is_able_to_self_borrow(
      AbleToUseSelfLane(*reference_line_info));
  IsBorrowNeedRemoteRequest(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::LEFTBORROW, reference_line_info);
  AINFO << "action " << __func__ << "  slow_obs_disappear_check_times "
        << mutable_laneborrow->slow_obs_disappear_check_times()
        << "  is_left_borrowable " << mutable_laneborrow->is_left_borrowable()
        << "  is_left_borrowable " << mutable_laneborrow->is_left_borrow_safe()
        << "  is_able_to_self_borrow "
        << mutable_laneborrow->is_able_to_self_borrow();
  return;
}

void LaneBorrowFsmAction::RightBorrow2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::RETURN);
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  reference_line_info->set_is_path_lane_borrow(true);
  IsBorrowNeedRemoteRequest(reference_line_info);
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::RightBorrow2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // laneborrow  is_left_borrowable / is_right_borrowable
  // need_keep_self_path_straight path_decider  front_static_obstacle_id
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);

  // laneborrow  laneborrow_obstacle_id  slow_obs_keep_check_times
  FindLaneBorrowObstacle(reference_line_info);
  reference_line_info->set_is_path_lane_borrow(true);
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_lane_borrow_status(LaneborrowStatus::RIGHTBORROW);
  // laneborrow  slow_obs_disappear_check_times
  if (IsObsDisappearOrFarAway(reference_line_info)) {
    auto check_times = mutable_laneborrow->slow_obs_disappear_check_times();
    mutable_laneborrow->set_slow_obs_disappear_check_times(++check_times);
  } else {
    mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  }

  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  if (FLAGS_enable_near_junction_laneborrow &&
      mutable_path_decider_status->is_adc_near_junction()) {
    CheckNearJunctionExitLaneBorrow(frame, reference_line_info);
  } else if (injector_->is_auxiliary_road_) {
    CheckAuxiliaryRoadExitLaneBorrow(frame, reference_line_info);
  } else {
    // check right laneborrow
    bool is_right_borrowable = true;
    CheckRightLaneBorrowable(*reference_line_info, &is_right_borrowable);
    mutable_laneborrow->set_is_right_borrowable(is_right_borrowable);
    mutable_laneborrow->set_is_left_borrowable(false);
    mutable_laneborrow->set_is_right_borrow_safe(is_right_borrowable);
    mutable_laneborrow->set_is_left_borrow_safe(false);
  }
  mutable_laneborrow->set_is_able_to_self_borrow(
      AbleToUseSelfLane(*reference_line_info));
  IsBorrowNeedRemoteRequest(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::RIGHTBORROW, reference_line_info);
  AINFO << "action " << __func__ << "  slow_obs_disappear_check_times "
        << mutable_laneborrow->slow_obs_disappear_check_times()
        << "  is_right_borrowable " << mutable_laneborrow->is_right_borrowable()
        << "  is_right_borrow_safe "
        << mutable_laneborrow->is_right_borrow_safe();
  return;
}

void LaneBorrowFsmAction::Return2LeftBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_is_in_laneborrow_status(true);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::LEFT_BORROW);
  reference_line_info->set_is_path_lane_borrow(true);
  IsBorrowNeedRemoteRequest(reference_line_info);
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Return2RightBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_is_in_laneborrow_status(true);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::RIGHT_BORROW);
  reference_line_info->set_is_path_lane_borrow(true);
  IsBorrowNeedRemoteRequest(reference_line_info);
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Return2Finish(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_is_in_laneborrow_status(false);
  mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::NO_BORROW);
  reference_line_info->set_is_path_lane_borrow(false);

  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
  AINFO << "action " << __func__;
  return;
}

void LaneBorrowFsmAction::Return2Return(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // laneborrow  is_left_borrowable / is_right_borrowable
  // need_keep_self_path_straight path_decider  front_static_obstacle_id
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  // laneborrow  laneborrow_obstacle_id  slow_obs_keep_check_times
  if (FindLaneBorrowObstacle(reference_line_info)) {
    mutable_laneborrow->set_slow_obs_disappear_check_times(0);
    auto check_times = mutable_laneborrow->slow_obs_keep_check_times();
    mutable_laneborrow->set_slow_obs_keep_check_times(++check_times);

    // laneborrow  is_left_borrow_safe / is_right_borrow_safe
    // is_necessary_to_laneborrow path_decider  is_in_path_laneborrow_scenario
    // decided_side_pass_direction
    bool is_necessary_to_laneborrow =
        IsNecessaryToLaneBorrow(frame, reference_line_info);
    if (injector_->auto_state_count_ == 100 && FLAGS_enable_auto_borrow) {
      is_necessary_to_laneborrow = true;
    }
    mutable_laneborrow->set_is_necessary_to_laneborrow(
        is_necessary_to_laneborrow);

    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    // mutable_path_decider_status->clear_decided_side_pass_direction();
    //  NEED TO ADD RIGHT BORROW
    if (is_necessary_to_laneborrow &&
        mutable_path_decider_status->decided_side_pass_direction_size() > 0) {
      if (mutable_path_decider_status->decided_side_pass_direction()[0] ==
          PathDeciderStatus::LEFT_BORROW) {
        mutable_laneborrow->set_laneborrow_direction(
            LaneborrowStatus::LEFT_BORROW);
      } else if (mutable_path_decider_status
                     ->decided_side_pass_direction()[0] ==
                 PathDeciderStatus::RIGHT_BORROW) {
        mutable_laneborrow->set_laneborrow_direction(
            LaneborrowStatus::RIGHT_BORROW);
      }
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      reference_line_info->set_is_path_lane_borrow(true);
    }
  } else {
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
    reference_line_info->set_is_path_lane_borrow(true);
  }

  // laneborrow  slow_obs_disappear_check_times
  if (IsObsDisappearOrFarAway(reference_line_info)) {
    auto check_times = mutable_laneborrow->slow_obs_disappear_check_times();
    mutable_laneborrow->set_slow_obs_disappear_check_times(++check_times);
  } else {
    mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  }
  IsBorrowNeedRemoteRequest(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::RETURN, reference_line_info);
  AINFO << "action " << __func__ << "  is_adc_posture_straight "
        << reference_line_info->IsAdcPostureStraight();
  return;
}

void LaneBorrowFsmAction::Finish2Default(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_slow_obs_disappear_check_times(0);
  mutable_laneborrow->set_slow_obs_keep_check_times(0);
  mutable_laneborrow->set_laneborrow_obstacle_id("");
  mutable_laneborrow->set_laneborrow_obstacle_perception_id(0);
  mutable_laneborrow->set_is_in_laneborrow_status(false);
  reference_line_info->set_is_path_lane_borrow(false);
  injector_->ClearStartMovingObstacles();
  AINFO << "action " << __func__;
  AINFO << "clear borrow response.";
  planning::BorrowResponse borrow_response;
  borrow_response.set_response_type(planning::ResponseType::UNTREATED);
  borrow_response.set_block_obs_id("");
  borrow_response.set_has_response(false);
  injector_->set_borrow_response(borrow_response);
  injector_->borrow_response().clear_block_obs_id();
  AINFO << "injector_->borrow_response().clear_block_obs_id() ="
        << injector_->borrow_response().block_obs_id();
  // injector_->borrow_response().clear_response_type();
  if (injector_->borrow_response().response_type() ==
      planning::ResponseType::ACCEPT) {
    AINFO << "clear response_type faild";
  }
  return;
}

void LaneBorrowFsmAction::Finish2Finish(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // laneborrow  is_left_borrowable / is_right_borrowable
  // need_keep_self_path_straight path_decider  front_static_obstacle_id
  // is_in_crowd_traffic
  CheckLaneBorrowInfo(reference_line_info);
  PrintDebugInfo(LaneBorrowDebug::FINISH, reference_line_info);
  AINFO << "action " << __func__;
  return;
}

bool LaneBorrowFsmAction::IsObsDisappearOrFarAway(
    ReferenceLineInfo* const reference_line_info) {
  const std::string laneborrow_obstacle_id = injector_->planning_context()
                                                 ->planning_status()
                                                 .lane_borrow()
                                                 .laneborrow_obstacle_id();
  // laneborrow obstacle id is empty
  if (laneborrow_obstacle_id.empty()) {
    ADEBUG << "Laneborrow obstacle disappeared.";
    return true;
  }

  // invalid obstacle
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          laneborrow_obstacle_id);
  if (nullptr == blocking_obstacle) {
    ADEBUG << "Laneborrow obstacle pointer is nullptr.";
    return true;
  }

  // vehicle polygon
  auto adc_sl = reference_line_info->AdcSlBoundary();
  // obstacle polygon
  auto obstacle_sl = blocking_obstacle->PerceptionSLBoundary();
  // AINFO << "adc_sl.start_s " << adc_sl.start_s();
  // AINFO << "obstacle_sl.end_s " << obstacle_sl.end_s();
  //
  if (obstacle_sl.start_s() - adc_sl.end_s() >
      config_.path_lane_borrow_decider_config().obs_far_away_safe_distance()) {
    AINFO << "Laneborrow obstacle goes far away.";
    return true;
  }
  if (adc_sl.start_s() - obstacle_sl.end_s() >
      config_.path_lane_borrow_decider_config().pass_by_safe_distance()) {
    AINFO << "Bypass Laneborrow obstacle.";
    return true;
  }
  return false;
}

bool LaneBorrowFsmAction::IsNecessaryToLaneBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  double adc_v = std::fabs(injector_->vehicle_state()->linear_velocity());
  if (adc_v > kBorrowSpeed) {
    return false;
  }

  if (FLAGS_enable_borrow_request) {
    if (injector_->borrow_response().response_type() ==
        planning::ResponseType::REFUSE) {
      AINFO << "REFUSE  BORROW";
      const Obstacle* laneborrow_obstacle =
          reference_line_info->path_decision()->obstacles().Find(
              injector_->borrow_response().block_obs_id());
      if (laneborrow_obstacle != nullptr) {
        const auto& obs_sl = laneborrow_obstacle->PerceptionSLBoundary();
        const auto& adc_sl = reference_line_info->AdcSlBoundary();
        if (adc_sl.start_s() > obs_sl.end_s()) {
          AINFO << "clear borrow response.";
          planning::BorrowResponse borrow_response;
          borrow_response.set_response_type(planning::ResponseType::UNTREATED);
          borrow_response.set_block_obs_id("");
          borrow_response.set_has_response(false);
          injector_->set_borrow_response(borrow_response);
          injector_->borrow_response().clear_block_obs_id();
          AINFO << "injector_->borrow_response().clear_block_obs_id() ="
                << injector_->borrow_response().block_obs_id();
        }
      } else {
        AINFO << "clear borrow response.";
        planning::BorrowResponse borrow_response;
        borrow_response.set_response_type(planning::ResponseType::UNTREATED);
        borrow_response.set_block_obs_id("");
        borrow_response.set_has_response(false);
        injector_->set_borrow_response(borrow_response);
        injector_->borrow_response().clear_block_obs_id();
        AINFO << "injector_->borrow_response().clear_block_obs_id() ="
              << injector_->borrow_response().block_obs_id();
      }
      return false;
    } else if (injector_->borrow_response().response_type() ==
               planning::ResponseType::ACCEPT) {
      reference_line_info->SetNeedDiagonal(true);
      AINFO << "ACCEPT BORROW = "
            << injector_->borrow_response().block_obs_id();
    } else {
      // AINFO << "DEFAULT  BORROW";
      return false;
    }
  }
  // pass stacker but no enter borrow,face obs,need check safe
  if (injector_->borrow_response().has_response() &&
      injector_->borrow_response().block_obs_id() == "pass_stacker_obs" &&
      !injector_->pass_stacker_response().has_response() &&
      !util::IsLaneBorrow(injector_->planning_context())) {
    AINFO << "pass stacker and no need borrow ,clear borrow response .";
    planning::BorrowResponse borrow_response;
    borrow_response.set_response_type(planning::ResponseType::UNTREATED);
    borrow_response.set_block_obs_id("");
    borrow_response.set_has_response(false);
    injector_->set_borrow_response(borrow_response);
    injector_->borrow_response().clear_block_obs_id();
  }

  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();

  const std::string blocking_obstacle_id =
      mutable_path_decider_status->front_static_obstacle_id();
  if (blocking_obstacle_id.empty() &&injector_->borrow_response()
          .has_response() &&
      injector_->borrow_response().block_obs_id() ==
          "STOP_REASON_WHEEL_CRANE") {
    if (blocking_obstacle_id.empty()) {
      // bo blocking obs
      std::string nearst_wheel_crane_id = "";
      double min_wheel_crane_start_s = std::numeric_limits<double>::max();
      bool has_wheel_crane = false;
      double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
      double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
      for (const auto* obstacle :
           reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle->Perception().type() !=
            perception::PerceptionObstacle::WHEELCRANE) {
          continue;
        }
        const auto& obs_sl = obstacle->PerceptionSLBoundary();
        if (obs_sl.start_s() - adc_end_s > FLAGS_wheelcrane_consider_distance) {
          AINFO << "large distance ,no consider";
          continue;
        }
        if (obs_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
          AINFO << "back stacker ,no consider";
          continue;
        }
        has_wheel_crane = true;
        if (obs_sl.start_s() < min_wheel_crane_start_s) {
          min_wheel_crane_start_s = obs_sl.start_s();
          nearst_wheel_crane_id = obstacle->Id();
        }
      }
      // no wheel crane
      if (!has_wheel_crane) {
        AINFO << "pass wheel crane and no need borrow ,clear borrow response .";
        planning::BorrowResponse borrow_response;
        borrow_response.set_response_type(planning::ResponseType::UNTREATED);
        borrow_response.set_block_obs_id("");
        borrow_response.set_has_response(false);
        injector_->set_borrow_response(borrow_response);
        injector_->borrow_response().clear_block_obs_id();
      }
    }
  }

  // single lane
  if (!HasSingleReferenceLine(*frame)) {
    AINFO << "single reference line";
    // return false;
  }
  if(reference_line_info->IsChangeLanePath()){
    AINFO << "change lane path  reference line";
    return false;
  }

  if (!IsWithinSidePassingSpeedADC()) {
    AINFO << "No laneborrow reason: IsWithinLaneBorrowSpeedADC";
    return false;
  }

  if (!IsBlockingObstacleFarFromIntersection(frame, reference_line_info)) {
    AINFO << "No laneborrow reason: IsBlockingObstacleFarFromIntersection";
    return false;
  }

  if (!IsBlockingObstacleWithinDestination(*reference_line_info)) {
    AINFO << "No laneborrow reason: IsBlockingObstacleWithinDestination";
    return false;
  }

  if (!IsSidePassableObstacle(*reference_line_info)) {
    AINFO << "No laneborrow reason: IsSidePassableObstacle";
    return false;
  }

  if (mutable_path_decider_status->is_in_crowd_traffic()) {
    AINFO << "No laneborrow reason: in crowd traffic";
    return false;
  }

  // check left lane borrow safe, right lane borrow safe
  CheckIsSafeToLaneBorrow(*reference_line_info);
  bool is_left_borrow_safe = injector_->planning_context()
                                 ->planning_status()
                                 .lane_borrow()
                                 .is_left_borrow_safe();
  bool is_right_borrow_safe = injector_->planning_context()
                                  ->planning_status()
                                  .lane_borrow()
                                  .is_right_borrow_safe();
  AINFO << "is_left_borrow_safe " << is_left_borrow_safe;
  AINFO << "is_right_borrow_safe " << is_right_borrow_safe;
  if (!(is_left_borrow_safe || is_right_borrow_safe)) {
    AINFO << "Unsafe to left/right borrow.";
    return false;
  }

  if (FLAGS_enable_noborrow_nearobstacle &&
      hdmap::Lane::PLAY_STREET != reference_line_info->GetLaneType() &&
      IsNearObstacle(*reference_line_info)) {
    AINFO << "No laneborrow reason: IsNearObstacle";
    return false;
  }

  if (!IsLaneTypeSupportLaneBorrow(*reference_line_info)) {
    AINFO << "No laneborrow reason: IsLaneTypeSupportLaneBorrow";
    return false;
  }

  return true;
}

bool LaneBorrowFsmAction::HasSingleReferenceLine(const Frame& frame) {
  return 1 == frame.reference_line_info().size();
}

bool LaneBorrowFsmAction::IsWithinSidePassingSpeedADC() {
  return AdcSpeedStatus::SPEED_LOWER == injector_->GetAdcSpeedStatus();
}

bool LaneBorrowFsmAction::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const std::string laneborrow_obstacle_id = injector_->planning_context()
                                                 ->planning_status()
                                                 .path_decider()
                                                 .front_static_obstacle_id();
  if (laneborrow_obstacle_id.empty()) {
    AERROR << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* laneborrow_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          laneborrow_obstacle_id);
  if (nullptr == laneborrow_obstacle) {
    AERROR << "Blocking obstacle is no longer there.";
    return false;
  }

  double laneborrow_obstacle_start_s =
      laneborrow_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle begins at s = " << laneborrow_obstacle_start_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  if (laneborrow_obstacle_start_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

bool LaneBorrowFsmAction::IsBlockingObstacleFarFromIntersection(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    AINFO << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          blocking_obstacle_id);
  if (nullptr == blocking_obstacle) {
    AINFO << "Blocking obstacle is no longer there.";
    return false;
  }

  // Get blocking obstacle's s.
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

  // remain distance to laneborrow
  double remain_distane = CheckBorrowRemainDistance(reference_line_info);
  double distance = closest_stop_sign_start_s_ - blocking_obstacle_s;
  if (FLAGS_enable_near_junction_laneborrow) {
    auto* mutable_path_decider = injector_->planning_context()
                                     ->mutable_planning_status()
                                     ->mutable_path_decider();
    bool is_obs_near_junction = false;
    bool has_truck_near_junction = false;
    CheckObsNearJunction(distance, blocking_obstacle, &is_obs_near_junction,
                         &has_truck_near_junction);
    mutable_path_decider->set_is_obs_near_junction(is_obs_near_junction);
    mutable_path_decider->set_has_truck_near_junction(has_truck_near_junction);
    if (has_truck_near_junction) {
      CheckIsNeedPassTruck(reference_line_info);
    } else {
      mutable_path_decider->set_truck_check_count(0);
      if (is_obs_near_junction || injector_->adc_in_junction_info_.first) {
        CheckIsNeedGotoLeftNeighborLane(frame, reference_line_info);
      } else {
        mutable_path_decider->set_goleft_near_junction_check_count(0);
      }
    }
  } else if (distance < config_.path_lane_borrow_decider_config()
                            .far_from_intersection_distance_threshold() &&
             remain_distane > 0.0) {
    return false;
  }

  return true;
}

bool LaneBorrowFsmAction::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const std::string laneborrow_obstacle_id = injector_->planning_context()
                                                 ->planning_status()
                                                 .path_decider()
                                                 .front_static_obstacle_id();
  if (laneborrow_obstacle_id.empty()) {
    AERROR << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* laneborrow_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          laneborrow_obstacle_id);
  if (nullptr == laneborrow_obstacle) {
    AERROR << "Blocking obstacle is no longer there.";
    return false;
  }

  if (!IsNonmovableObstacle(reference_line_info, *laneborrow_obstacle)) {
    return false;
  }

  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  if ((mutable_path_decider->has_truck_near_junction() ||
       mutable_path_decider->is_obs_near_junction() ||
       injector_->adc_in_junction_info_.first) &&
      !mutable_path_decider->is_need_go_left_lane()) {
    return false;
  }

  return true;
}

bool LaneBorrowFsmAction::IsLaneTypeSupportLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->clear_decided_side_pass_direction();
  if (FLAGS_enable_near_junction_laneborrow &&
      (mutable_path_decider_status->has_truck_near_junction() ||
       mutable_path_decider_status->is_obs_near_junction() ||
       injector_->adc_in_junction_info_.first)) {
    // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
    mutable_path_decider_status->add_decided_side_pass_direction(
        PathDeciderStatus::LEFT_BORROW);
    ADEBUG << "[Junction] switch from self borrow to LANE BORROW.";
    return true;
  } else if (mutable_path_decider_status->decided_side_pass_direction()
                 .empty()) {
    bool left_borrowable = false;
    bool right_borrowable = false;
    CheckBothLaneBorrowable(reference_line_info, &left_borrowable,
                            &right_borrowable);
    AINFO << "away from junction, left borrowable: " << left_borrowable
          << ", right borrowable: " << right_borrowable;
    if (!left_borrowable && !right_borrowable) {
      // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      return false;
    } else {
      // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      // can right and left borrowable,if stacker in lane left ,right borrow
      // first.
      bool has_block_obs = false;
      const std::string laneborrow_obstacle_id =
          mutable_path_decider_status->front_static_obstacle_id();
      AINFO << "laneborrow_obstacle_id = " << laneborrow_obstacle_id;
      if (laneborrow_obstacle_id.empty()) {
        AERROR << "There is no blocking obstacle.";
        has_block_obs = false;
      }
      const Obstacle* laneborrow_obstacle =
          reference_line_info.path_decision().obstacles().Find(
              laneborrow_obstacle_id);
      if (nullptr == laneborrow_obstacle) {
        AERROR << "Blocking obstacle is no longer there.";
        has_block_obs = false;
      }
      has_block_obs = true;
      bool use_right_borrow_first = false;
      if (has_block_obs) {
        AINFO << "laneborrow_obstacle->Perception().type() = "
              << laneborrow_obstacle->Perception().type();
        bool is_tire_lifter =
            laneborrow_obstacle->Perception().type() ==
                perception::PerceptionObstacle::WHEELCRANE;
        auto tire_lifter_boundary = laneborrow_obstacle->PerceptionSLBoundary();
        bool is_satcker = laneborrow_obstacle->Perception().type() ==
                              perception::PerceptionObstacle::STACKER |
                          laneborrow_obstacle->Perception().type() ==
                              perception::PerceptionObstacle::FORKLIFT_STACKER;
        double tire_lifter_center_l =
            (tire_lifter_boundary.start_l() + tire_lifter_boundary.end_l()) *
            0.5;
        AINFO << "tire_lifter_center_l = " << tire_lifter_center_l;
        AINFO << "is_satcker = " << is_satcker;
        if ((is_tire_lifter || is_satcker) && tire_lifter_center_l > 0.0) {
          use_right_borrow_first = true;
        }
      }
      if (has_block_obs) {
        double curr_road_left_width = 0.0;
        double curr_road_right_width = 0.0;
        double past_road_left_width = kDefaultLaneWidth * 0.5;
        double past_road_right_width = kDefaultLaneWidth * 0.5;
        const auto& obs_sl = laneborrow_obstacle->PerceptionSLBoundary();
        double curr_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
        const auto& veh_param =
            common::VehicleConfigHelper::GetConfig().vehicle_param();
        if (!reference_line_info.reference_line().GetRoadWidth(
                curr_s, &curr_road_left_width, &curr_road_right_width)) {
          // AWARN << "Failed to get lane width at s = " << curr_s;
          curr_road_left_width = past_road_left_width;
          curr_road_right_width = past_road_right_width;
        } else {
          past_road_left_width = curr_road_left_width;
          past_road_right_width = curr_road_right_width;
        }
        double left_space = past_road_left_width;
        double right_space = curr_road_right_width;
        bool no_left_space_to_borrow =
            (left_space - obs_sl.end_l()) < (veh_param.width() + kPassBuffer);
        bool has_right_space_to_borrow = (obs_sl.start_l() - (-right_space)) >
                                         (veh_param.width() + kPassBuffer);
        if (no_left_space_to_borrow && has_right_space_to_borrow) {
          use_right_borrow_first = true;
        }
      }
      AINFO << "use_right_borrow_first = " << use_right_borrow_first;
      if (use_right_borrow_first) {
        AINFO << "right_borrowable = " << right_borrowable;
        if (right_borrowable && injector_->planning_context()
                                    ->planning_status()
                                    .lane_borrow()
                                    .is_right_borrow_safe()) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        } else if (left_borrowable && injector_->planning_context()
                                          ->planning_status()
                                          .lane_borrow()
                                          .is_left_borrow_safe()) {
          bool is_tire_lifter = laneborrow_obstacle->Perception().type() ==
                                perception::PerceptionObstacle::WHEELCRANE;
          if (is_tire_lifter) {
            return false;
          }
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        }
      } else {
        if (left_borrowable && injector_->planning_context()
                                   ->planning_status()
                                   .lane_borrow()
                                   .is_left_borrow_safe()) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        } else if (right_borrowable && injector_->planning_context()
                                           ->planning_status()
                                           .lane_borrow()
                                           .is_right_borrow_safe()) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        }
      }
    }
  }
  return true;
}

void LaneBorrowFsmAction::CheckBothLaneBorrowable(
    const ReferenceLineInfo& reference_line_info,
    bool* const is_left_borrowable, bool* const is_right_borrowable) {
  *is_left_borrowable = true;
  *is_right_borrowable = true;
  double lookforward_distance = GetForwardCheckDistance(reference_line_info);
  double check_s = reference_line_info.AdcSlBoundary().end_s();

  while (check_s < lookforward_distance) {
    auto ref_point =
        reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *is_left_borrowable = false;
      *is_right_borrowable = false;
      return;
    }

    const auto& waypoint = ref_point.lane_waypoints().front();
    if (*is_left_borrowable) {
      const auto lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (!IsLaneBoundaryTypeBorrowable(lane_boundary_type)) {
        *is_left_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
      if (IsNeighbourLaneEmpty(waypoint, LaneborrowStatus::LEFT_BORROW)) {
        *is_left_borrowable = false;
        AINFO << "left FORWARD/REVERSE lane is empty, not borrowable";
      }
    }
    if (*is_right_borrowable) {
      const auto lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (!IsLaneBoundaryTypeBorrowable(lane_boundary_type) ||
          waypoint.lane->lane().right_boundary().gap()) {
        *is_right_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
      if (IsNeighbourLaneEmpty(waypoint, LaneborrowStatus::RIGHT_BORROW)) {
        *is_right_borrowable = false;
        AINFO << "right FORWARD/REVERSE lane is empty, not borrowable";
      }
    }
    check_s += kCheckStepDistance;
  }
  return;
}

void LaneBorrowFsmAction::CheckLeftLaneBorrowable(
    const ReferenceLineInfo& reference_line_info,
    bool* const is_left_borrowable) {
  *is_left_borrowable = true;
  double lookforward_distance = GetForwardCheckDistance(reference_line_info);
  double check_s = reference_line_info.AdcSlBoundary().end_s();

  while (check_s < lookforward_distance) {
    auto ref_point =
        reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *is_left_borrowable = false;
      return;
    }

    const auto& waypoint = ref_point.lane_waypoints().front();
    if (*is_left_borrowable) {
      const auto lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (!IsLaneBoundaryTypeBorrowable(lane_boundary_type)) {
        *is_left_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
      if (IsNeighbourLaneEmpty(waypoint, LaneborrowStatus::LEFT_BORROW)) {
        *is_left_borrowable = false;
        AINFO << "left FORWARD/REVERSE lane is empty, not borrowable";
      }
    }
    check_s += kCheckStepDistance;
  }
  return;
}

void LaneBorrowFsmAction::CheckRightLaneBorrowable(
    const ReferenceLineInfo& reference_line_info,
    bool* const is_right_borrowable) {
  *is_right_borrowable = true;
  double lookforward_distance = GetForwardCheckDistance(reference_line_info);
  double check_s = reference_line_info.AdcSlBoundary().end_s();

  while (check_s < lookforward_distance) {
    auto ref_point =
        reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *is_right_borrowable = false;
      return;
    }

    const auto& waypoint = ref_point.lane_waypoints().front();
    if (*is_right_borrowable) {
      const auto lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (!IsLaneBoundaryTypeBorrowable(lane_boundary_type) ||
          waypoint.lane->lane().right_boundary().gap()) {
        *is_right_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
      if (IsNeighbourLaneEmpty(waypoint, LaneborrowStatus::RIGHT_BORROW)) {
        *is_right_borrowable = false;
        AINFO << "right FORWARD/REVERSE lane is empty, not borrowable";
      }
    }
    check_s += kCheckStepDistance;
  }
  return;
}

bool LaneBorrowFsmAction::IsLaneBoundaryTypeBorrowable(
    const hdmap::LaneBoundaryType::Type& lane_boundary_type) {
  if (hdmap::LaneBoundaryType::CURB == lane_boundary_type ||
      hdmap::LaneBoundaryType::UNKNOWN == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_YELLOW == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_WHITE == lane_boundary_type ||
      hdmap::LaneBoundaryType::DOUBLE_YELLOW == lane_boundary_type) {
    return false;
  }
  return true;
}

bool LaneBorrowFsmAction::IsNeighbourLaneEmpty(
    const hdmap::LaneWaypoint& waypoint,
    const LaneborrowStatus::LaneborrowDirection& direction) {
  bool is_in_borrow = injector_->planning_context()
                          ->planning_status()
                          .lane_borrow()
                          .is_in_laneborrow_status();
  if (LaneborrowStatus::LEFT_BORROW == direction && !is_in_borrow &&
      waypoint.lane->lane().left_neighbor_forward_lane_id().empty() &&
      (waypoint.lane->lane().left_neighbor_reverse_lane_id().empty() ||
       not_borrow_reverse_)) {
    return true;
  }
  if (LaneborrowStatus::RIGHT_BORROW == direction && !is_in_borrow &&
      waypoint.lane->lane().right_neighbor_forward_lane_id().empty() &&
      (waypoint.lane->lane().right_neighbor_reverse_lane_id().empty() ||
       not_borrow_reverse_)) {
    return true;
  }
  return false;
}

bool LaneBorrowFsmAction::IsLeftNeighbourForwardLaneEmpty(
    ReferenceLineInfo* const reference_line_info) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& ref_point =
      reference_line_info->reference_line().GetNearestReferencePoint(
          adc_sl.end_s());
  const auto& waypoint = ref_point.lane_waypoints().front();
  return waypoint.lane->lane().left_neighbor_forward_lane_id().empty();
}

bool LaneBorrowFsmAction::IsNearObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (nullptr == blocking_obstacle) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double blocking_obstacle_l =
      blocking_obstacle->PerceptionSLBoundary().end_l();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_l_max =
      std::max(std::fabs(adc_end_l), std::fabs(adc_start_l));
  auto stop_distance = blocking_obstacle->MinRadiusStopDistance(
      common::VehicleConfigHelper::GetConfig().vehicle_param(), adc_l_max);
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  AINFO << "stop_distance = " << stop_distance;
  if (blocking_obstacle_s - adc_end_s > 0.0 &&
      blocking_obstacle_s - adc_end_s < stop_distance &&
      blocking_obstacle_l > adc_end_l) {
    return true;
  }
  return false;
}

void LaneBorrowFsmAction::CheckIsSafeToLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  if (!mutable_laneborrow->is_left_borrowable() &&
      !mutable_laneborrow->is_right_borrowable()) {
    mutable_laneborrow->set_is_left_borrow_safe(false);
    mutable_laneborrow->set_is_right_borrow_safe(false);
    return;
  }
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  hdmap::Id left_neighbor_lane_id, right_neighbor_lane_id;
  double left_neighbor_lane_width = 0.0, right_neighbor_lane_width = 0.0;
  if (!reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl_boundary.end_s(),
          &left_neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl_boundary.end_s(),
          &left_neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, adc_sl_boundary.end_s(),
          &right_neighbor_lane_id, &right_neighbor_lane_width) &&
      !reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightReverse, adc_sl_boundary.end_s(),
          &right_neighbor_lane_id, &right_neighbor_lane_width)) {
    mutable_laneborrow->set_is_left_borrow_safe(false);
    mutable_laneborrow->set_is_right_borrow_safe(false);
    return;
  }
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  const auto& lane_width = reference_line_info.GetLaneWidthBaseOnAdcCenter();
  double curr_lane_left_width = lane_width.first;
  double curr_lane_right_width = lane_width.second;
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double past_road_left_width = curr_lane_left_width;
  double past_road_right_width = curr_lane_right_width;
  if (!reference_line_info.reference_line().GetRoadWidth(
          adc_sl.end_s(), &curr_road_left_width, &curr_road_right_width)) {
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  left_neighbor_lane_width = curr_road_left_width - curr_lane_left_width;
  right_neighbor_lane_width = curr_road_right_width - curr_lane_right_width;
  double adc_start_s = adc_sl_boundary.start_s();
  double adc_end_s = adc_sl_boundary.end_s();
  double adc_v = std::abs(injector_->vehicle_state()->linear_velocity());
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double left_width = lane_width.first;
  double right_width = lane_width.second;
  const auto& config = config_.path_lane_borrow_decider_config();
  mutable_laneborrow->set_is_left_borrow_safe(true);
  mutable_laneborrow->set_is_right_borrow_safe(true);
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (nullptr == obstacle) {
      continue;
    }
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle->IsStatic() &&
        util::WithPassObsSideBySide(reference_line_info, obstacle_sl,
                                    veh_param.width())) {
      continue;
    }
    if (!util::HasCommon(obstacle_sl.start_l(), obstacle_sl.end_l(), left_width,
                         left_width + left_neighbor_lane_width) &&
        !util::HasCommon(obstacle_sl.start_l(), obstacle_sl.end_l(),
                         -right_width,
                         -(right_width + right_neighbor_lane_width))) {
      continue;
    }
    double obstacle_moving_direction = 0.0;
    if (!obstacle->HasTrajectory()) {
      obstacle_moving_direction = obstacle->Perception().theta();
    } else {
      obstacle_moving_direction = obstacle->SpeedHeading();
    }
    double vehicle_moving_direction = injector_->vehicle_state()->heading();
    double heading_difference = std::abs(common::math::NormalizeAngle(
        obstacle_moving_direction - vehicle_moving_direction));
    bool is_same_direction = heading_difference < kSameDirectionThr;
    double forward_safe_distance_left = 0.0, forward_safe_distance_right = 0.0;
    double backward_safe_distance_left = 0.0,
           backward_safe_distance_right = 0.0;
    // set a safe distance based on direction
    if (is_same_direction) {
      double safe_check_time_left =
          config.safe_check_time_same_direction() +
          GetExtraSafeCheckTime(reference_line_info,
                                LaneborrowStatus::LEFT_BORROW);
      forward_safe_distance_left =
          std::fmax(config.forward_safe_distance_same_direction(),
                    ((adc_v - obstacle->speed()) * safe_check_time_left));
      backward_safe_distance_left =
          std::fmax(config.backward_safe_distance_same_direction(),
                    ((obstacle->speed() - adc_v) * safe_check_time_left));
      double safe_check_time_right =
          config.safe_check_time_same_direction() +
          GetExtraSafeCheckTime(reference_line_info,
                                LaneborrowStatus::RIGHT_BORROW);
      forward_safe_distance_right =
          std::fmax(config.forward_safe_distance_same_direction(),
                    (adc_v - obstacle->speed()) * safe_check_time_right);
      backward_safe_distance_right =
          std::fmax(config.backward_safe_distance_same_direction(),
                    (obstacle->speed() - adc_v) * safe_check_time_right);
    } else {
      double safe_check_time_left =
          config.safe_check_time_opposite_direction() +
          GetExtraSafeCheckTime(reference_line_info,
                                LaneborrowStatus::LEFT_BORROW);
      forward_safe_distance_left =
          std::fmax(config.forward_safe_distance_opposite_direction(),
                    ((adc_v + obstacle->speed()) * safe_check_time_left));
      backward_safe_distance_left =
          config.backward_safe_distance_opposite_direction();
      double safe_check_time_right =
          config.safe_check_time_opposite_direction() +
          GetExtraSafeCheckTime(reference_line_info,
                                LaneborrowStatus::RIGHT_BORROW);
      forward_safe_distance_right =
          std::fmax(config.forward_safe_distance_opposite_direction(),
                    ((adc_v + obstacle->speed()) * safe_check_time_right));
      backward_safe_distance_right =
          config.backward_safe_distance_opposite_direction();
    }
    if ((obstacle_sl.start_l() + obstacle_sl.end_l()) * 0.5 > 0.0) {
      bool is_in_left_return_status = false;
      bool is_adc_left_obs = false;
      if (mutable_laneborrow->lane_borrow_status() ==
          LaneborrowStatus::RETURN) {
        is_in_left_return_status = true;
      }
      if (obstacle_sl.start_l() > adc_sl_boundary.end_l()) {
        is_adc_left_obs = true;
      }
      if (is_in_left_return_status && is_adc_left_obs) {
        continue;
      }
      if (mutable_laneborrow->is_left_borrowable()) {
        if (util::HasCommon(obstacle_sl.start_s(), obstacle_sl.end_s(),
                            adc_start_s - backward_safe_distance_left,
                            adc_end_s + forward_safe_distance_left) &&
            obstacle->IsStatic()) {
          if (obstacle_sl.start_l() - veh_param.width() -
                      kSafeBorrowLateralDistance >
                  -right_width ||
              left_width + left_neighbor_lane_width - obstacle_sl.end_l() <
                  veh_param.width()) {
            mutable_laneborrow->set_is_left_borrow_safe(false);
          }
        }
      } else {
        mutable_laneborrow->set_is_left_borrow_safe(false);
      }
    } else {
      bool is_in_right_return_status = false;
      bool is_adc_right_obs = false;
      if (mutable_laneborrow->lane_borrow_status() ==
          LaneborrowStatus::RETURN) {
        is_in_right_return_status = true;
      }
      if (obstacle_sl.end_l() < adc_sl_boundary.start_l()) {
        is_adc_right_obs = true;
      }
      if (is_in_right_return_status && is_adc_right_obs) {
        continue;
      }
      if (mutable_laneborrow->is_right_borrowable()) {
        if (util::HasCommon(obstacle_sl.start_s(), obstacle_sl.end_s(),
                            adc_start_s - backward_safe_distance_right,
                            adc_end_s + forward_safe_distance_right) &&
            obstacle->IsStatic()) {
          if (obstacle_sl.start_l() +
                      (right_width + right_neighbor_lane_width) <
                  veh_param.width() ||
              -right_width - obstacle_sl.end_l() > veh_param.width()) {
            mutable_laneborrow->set_is_right_borrow_safe(false);
          }
        }
      } else {
        mutable_laneborrow->set_is_right_borrow_safe(false);
      }
    }
  }
}

double LaneBorrowFsmAction::GetForwardCheckDistance(
    const ReferenceLineInfo& reference_line_info) {
  double blocking_obstacle_s = 0.0;
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    AERROR << "There is no blocking obstacle.";
  } else {
    const Obstacle* blocking_obstacle =
        reference_line_info.path_decision().obstacles().Find(
            blocking_obstacle_id);
    if (nullptr == blocking_obstacle) {
      AERROR << "Blocking obstacle is no longer there.";
    } else {
      blocking_obstacle_s = blocking_obstacle->PerceptionSLBoundary().end_s();
    }
  }
  bool is_in_borrow = injector_->planning_context()
                          ->planning_status()
                          .lane_borrow()
                          .is_in_laneborrow_status();
  double check_forward_dis =
      (is_in_borrow && !reference_line_info.IsAdcPostureStraight())
          ? config_.path_lane_borrow_decider_config()
                .in_borrow_check_forward_distance()
          : config_.path_lane_borrow_decider_config().check_forward_distance();
  // adc front 20m.
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  double lookforward_distance =
      std::min(check_s + check_forward_dis,
               reference_line_info.reference_line().Length());
  // blocked obstacle after 10m.
  if (!is_in_borrow && blocking_obstacle_s > check_s) {
    lookforward_distance =
        std::max(blocking_obstacle_s + kLonDistance, lookforward_distance);
  }
  return lookforward_distance;
}

bool LaneBorrowFsmAction::AbleToUseSelfLane(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // reference lane obstacles
  std::vector<const Obstacle*> ref_obstacles;
  // ref_obstacles.insert(ref_obstacles.end(), ref_lane_veh_.begin(),
  //                      ref_lane_veh_.end());
  ref_obstacles.insert(ref_obstacles.end(), ref_static_obs_.begin(),
                       ref_static_obs_.end());
  // sort obstacle
  std::sort(ref_obstacles.begin(), ref_obstacles.end(),
            [](const Obstacle* a, const Obstacle* b) {
              return a->PerceptionSLBoundary().start_s() <
                     b->PerceptionSLBoundary().start_s();
            });
  // vehicle polygon
  auto adc_sl = reference_line_info.AdcSlBoundary();
  for (auto ob : ref_obstacles) {
    if (ob->PerceptionSLBoundary().end_s() >=
        adc_sl.start_s() -
            config_.path_lane_borrow_decider_config().pass_by_safe_distance()) {
      AINFO << "obstacle " << ob->Id() << '\t'
            << "is ahead of vehicle, cannot return !!!";
      mutable_path_decider_status->set_able_to_use_self_lane_counter(0);
      return false;
    }
    if (!ob->IsStatic()) {
      double t = GetExtraSafeCheckTime(reference_line_info,
                                       LaneborrowStatus::NO_BORROW);
      double adc_v = std::fabs(injector_->vehicle_state()->linear_velocity());
      double reference_line_heading = reference_line_info.GetHeadingByS(
          (ob->PerceptionSLBoundary().start_s() +
           ob->PerceptionSLBoundary().end_s()) *
          0.5);
      double obs_v =
          ob->speed() * cos(common::math::NormalizeAngle(
                            ob->SpeedHeading() - reference_line_heading));
      double check_s = std::fmax((adc_v - obs_v) * t,
                                 config_.path_lane_borrow_decider_config()
                                     .obs_far_away_safe_distance());
      if (util::WithinBound(ob->PerceptionSLBoundary().start_s(),
                            adc_sl.end_s(), adc_sl.end_s() + check_s)) {
        AINFO << "obstacle " << ob->Id() << '\t'
              << "is dynamic, cannot return !!!";
        mutable_path_decider_status->set_able_to_use_self_lane_counter(0);
        return false;
      }
    }
  }
  mutable_path_decider_status->set_able_to_use_self_lane_counter(
      mutable_path_decider_status->able_to_use_self_lane_counter() + 1);
  return mutable_path_decider_status->able_to_use_self_lane_counter() >=
         kUseSelfLaneCount;
}

bool LaneBorrowFsmAction::FindLaneBorrowObstacle(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  const std::string blocking_obstacle_id = injector_->planning_context()
                                               ->planning_status()
                                               .path_decider()
                                               .front_static_obstacle_id();
  AINFO << "blocking_obstacle_id " << blocking_obstacle_id;
  ADEBUG << "mutable_laneborrow->laneborrow_obstacle_id() = "
        << mutable_laneborrow->laneborrow_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    mutable_laneborrow->set_laneborrow_obstacle_id("");
    mutable_laneborrow->set_laneborrow_obstacle_perception_id(0);
    mutable_laneborrow->set_slow_obs_keep_check_times(0);
    ADEBUG << "There is no blocking obstacle, don't laneborrow.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          blocking_obstacle_id);

  if (nullptr == blocking_obstacle) {
    ADEBUG<<"NO blocking_obstacle_id";
    mutable_laneborrow->set_laneborrow_obstacle_id("");
    mutable_laneborrow->set_laneborrow_obstacle_perception_id(0);
    mutable_laneborrow->set_slow_obs_keep_check_times(0);
    return false;
  }
  if (std::strcmp(blocking_obstacle->Id().c_str(),
                  mutable_laneborrow->laneborrow_obstacle_id().c_str()) != 0) {
    ADEBUG<<"NO SET CHECK TIMES";
    mutable_laneborrow->set_slow_obs_keep_check_times(0);
    mutable_laneborrow->set_laneborrow_obstacle_id(blocking_obstacle->Id());
    mutable_laneborrow->set_laneborrow_obstacle_perception_id(
        blocking_obstacle->PerceptionId());
  } else {
    mutable_laneborrow->set_slow_obs_keep_check_times(
        mutable_laneborrow->slow_obs_keep_check_times() + 1);
  }
  return true;
}

bool LaneBorrowFsmAction::IsBorrowNeedRemoteRequest(
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (!path_decider_status.is_in_path_lane_borrow_scenario()) {
    return false;
  }
  if (reference_line_info->IsAdcLocatedInLane()) {
    return false;
  }
  const auto& config = config_.path_lane_borrow_decider_config();
  if (path_decider_status.lane_borrow_path_fail_times() >
      config.borrow_path_fail_remote_report_threshold()) {
    reference_line_info->SetLaneBorrowFailRequest(true);
    return true;
  }
  return false;
}

void LaneBorrowFsmAction::PrintDebugInfo(
    const planning_internal::LaneBorrowDebug::LaneBorrowFsmStatus&
        lane_borrow_status,
    ReferenceLineInfo* const reference_line_info) {
  auto* laneborrow_debug = reference_line_info->mutable_debug()
                               ->mutable_planning_data()
                               ->mutable_valid_path_info();
  laneborrow_debug->set_laneborrow_fsm_status(lane_borrow_status);
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  std::string msg = "[Away]";
  if (mutable_path_decider_status->is_adc_near_junction() ||
      mutable_path_decider_status->is_obs_near_junction()) {
    msg = "[Near]";
  }
  if (injector_->adc_in_junction_info_.first) {
    msg = "[In]";
  }
  AINFO << msg << "[Junction] adc/obs near junction: "
        << mutable_path_decider_status->is_adc_near_junction() << ","
        << mutable_path_decider_status->is_obs_near_junction()
        << ". adc current/last in junction: "
        << injector_->adc_in_junction_info_.first << ","
        << injector_->adc_in_junction_info_.second;
  if (config_.path_lane_borrow_decider_config()
          .enable_truck_negative_laneborrow() &&
      mutable_path_decider_status->has_truck_near_junction()) {
    AINFO << "[Truck] has truck near junction, id: "
          << mutable_path_decider_status->front_static_obstacle_id();
  }
  AINFO << "able_to_use_self_lane_counter: "
        << mutable_path_decider_status->able_to_use_self_lane_counter()
        << ", is_in_path_lane_borrow_scenario: "
        << mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

void LaneBorrowFsmAction::CheckLaneBorrowInPlayStreet(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const auto& adc_slboundary = reference_line_info->AdcSlBoundary();
  auto lane_type = reference_line_info->GetLaneType();
  if (hdmap::Lane::PLAY_STREET == lane_type) {
    AINFO << "ADC is in play street now.";
    double adc_center_l =
        (adc_slboundary.start_l() + adc_slboundary.end_l()) * 0.5;
    hdmap::Id neighbor_lane_id;
    double left_lane_width = 0.0;
    double right_lane_width = 0.0;
    bool has_left_lane =
        reference_line_info->GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::LeftForward, adc_slboundary.start_s(),
            &neighbor_lane_id, &left_lane_width)
            ? true
            : (reference_line_info->GetNeighborLaneInfo(
                   ReferenceLineInfo::LaneType::LeftReverse,
                   adc_slboundary.start_s(), &neighbor_lane_id,
                   &left_lane_width)
                   ? true
                   : false);
    bool has_right_forward_lane = reference_line_info->GetNeighborLaneInfo(
        ReferenceLineInfo::LaneType::RightForward, adc_slboundary.start_s(),
        &neighbor_lane_id, &right_lane_width);
    const auto& curr_lane_width =
        reference_line_info->GetLaneWidthBaseOnAdcCenter();
    if (!reference_line_info->IsAdcCenterInLane() &&
        (has_left_lane || has_right_forward_lane) &&
        mutable_path_decider_status->decided_side_pass_direction().empty()) {
      if (adc_center_l > 0.0 &&
          adc_slboundary.end_l() < (curr_lane_width.first + left_lane_width)) {
        mutable_laneborrow->set_is_left_borrowable(true);
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::LEFT_BORROW);
        mutable_laneborrow->set_is_need_to_laneborrow_in_play_street(true);
        // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      } else if (adc_center_l < 0.0 &&
                 adc_slboundary.start_l() >
                     (-curr_lane_width.second - right_lane_width)) {
        mutable_laneborrow->set_is_right_borrowable(true);
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::RIGHT_BORROW);
        mutable_laneborrow->set_is_need_to_laneborrow_in_play_street(true);
        // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      } else {
        mutable_laneborrow->set_is_need_to_laneborrow_in_play_street(false);
        // mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      }
    }
  } else {
    mutable_laneborrow->set_is_need_to_laneborrow_in_play_street(false);
  }
}

void LaneBorrowFsmAction::CheckLaneBorrowInfo(
    ReferenceLineInfo* const reference_line_info) {
  // compute remain distance for laneborrow
  CheckBorrowRemainDistance(reference_line_info);
  // check if the vehicle is in auxiliary road
  CheckAuxiliaryRoadStatus(reference_line_info);
  // check if the vehicle is near junction or in junction
  if (FLAGS_enable_near_junction_laneborrow) {
    CheckAdcDistance(reference_line_info);
  }
  // get static obstacles in left lane, right lane, reference lane
  // get vehicle in left lane, right lane, reference lane
  CheckVehicleObsInFrontOfAdc(reference_line_info);
  // check traffic
  CheckTrafficStatus(reference_line_info);
  has_checked_obs_in_front_ = false;
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->set_need_keep_self_path_straight(false);
}

double LaneBorrowFsmAction::CheckBorrowRemainDistance(
    ReferenceLineInfo* const reference_line_info) {
  // remain distance for laneborrow
  double remain_distane =
      std::fmin(reference_line_info->GetRemainDistanceToSolidLine(),
                reference_line_info->GetRemainDistanceToJunction());
  if (0 && remain_distane >
               reference_line_info->reference_line().Length() + kEpsilon) {
    remain_distane = reference_line_info->GetRemainDistanceForBack(
        injector_->planning_context());
  }
  const auto& config = config_.path_lane_borrow_decider_config();
  // flag for laneborrow reverse neighbour lane
  not_borrow_reverse_ =
      (remain_distane < config.near_junction_not_borrow_reverse_distance());
  // flag for keep straight near junction
  injector_->near_junction_keep_straight_ =
      (remain_distane < config.near_junction_distance_check_borrow());
  // calculate stop sign position
  if (remain_distane <
      reference_line_info->reference_line().Length() + kEpsilon) {
    closest_stop_sign_start_s_ =
        remain_distane + reference_line_info->AdcSlBoundary().end_s();
  } else {
    closest_stop_sign_start_s_ = std::numeric_limits<double>::max();
  }
  ADEBUG << "remain_distane: " << remain_distane
         << ", not_borrow_reverse_: " << not_borrow_reverse_
         << ", near_junction_keep_straight_: "
         << injector_->near_junction_keep_straight_
         << ", closest_stop_sign_start_s: " << closest_stop_sign_start_s_;

  return remain_distane;
}

void LaneBorrowFsmAction::CheckAdcDistance(
    ReferenceLineInfo* const reference_line_info) {
  // remain distance for laneborrow
  double to_solid_line_distance =
      std::fmin(reference_line_info->GetRemainDistanceToSolidLine(),
                reference_line_info->GetRemainDistanceToJunction());
  if (to_solid_line_distance >
      reference_line_info->reference_line().Length() + kEpsilon) {
    to_solid_line_distance = std::numeric_limits<double>::max();
  }
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  const auto& config = config_.path_lane_borrow_decider_config();
  // flag for near junction
  bool is_adc_near_junction =
      to_solid_line_distance < config.adc_to_solid_line_distance_threshold() &&
      !injector_->is_auxiliary_road_ && !injector_->is_in_play_street &&
      !reference_line_info->IsADCLocatedInMergeLane();
  mutable_path_decider->set_is_adc_near_junction(is_adc_near_junction);
  mutable_path_decider->set_is_obs_near_junction(false);

  // near junction && laneborrow scenario
  if (to_solid_line_distance < config.obs_to_solid_line_distance_threshold() ||
      (is_adc_near_junction &&
       util::IsLaneBorrow(injector_->planning_context()))) {
    mutable_path_decider->set_has_truck_near_junction(false);
    mutable_path_decider->set_truck_check_count(0);
  }

  // vehicle polygon
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  injector_->adc_in_junction_info_.second =
      injector_->adc_in_junction_info_.first;
  injector_->adc_in_junction_info_.first =
      reference_line_info->AdcIsOnOverlapJunction() &&
      !reference_line_info->IsInEitherSolidLine(adc_sl) &&
      !reference_line_info->IsADCLocatedInMergeLane() &&
      !injector_->is_auxiliary_road_;
  if (injector_->adc_in_junction_info_.first) {
    mutable_path_decider->set_is_adc_near_junction(false);
    mutable_path_decider->set_has_truck_near_junction(false);
  }

  if (injector_->adc_in_junction_info_.first !=
      injector_->adc_in_junction_info_.second) {
    mutable_path_decider->set_goleft_near_junction_check_count(0);
    mutable_path_decider->set_goback_near_junction_check_count(0);
    mutable_path_decider->set_truck_check_count(0);
    mutable_path_decider->set_is_need_go_left_lane(false);
    mutable_path_decider->set_is_need_goback_reference_lane(false);
  }

  if (!mutable_path_decider->is_adc_near_junction() &&
      !injector_->adc_in_junction_info_.first) {
    last_obs_keep_check_threshold_ = 0;
  }
}

void LaneBorrowFsmAction::CheckObsNearJunction(
    const double distance, const Obstacle* const block_obs,
    bool* const is_obs_near_junction, bool* const has_truck_near_junction) {
  CHECK_NOTNULL(is_obs_near_junction);
  CHECK_NOTNULL(has_truck_near_junction);

  const auto& config = config_.path_lane_borrow_decider_config();
  *is_obs_near_junction =
      distance < config.obs_to_solid_line_distance_threshold() &&
      !injector_->adc_in_junction_info_.first &&
      !injector_->is_auxiliary_road_ && !injector_->is_in_play_street;
  *has_truck_near_junction =
      config.enable_truck_negative_laneborrow() &&
      PerceptionObstacle::VEHICLE == block_obs->Perception().type() &&
      block_obs->Perception().height() > config.truck_height_threshold() &&
      distance < config.truck_distance_threshold() &&
      distance >= config.obs_to_solid_line_distance_threshold() &&
      !injector_->adc_in_junction_info_.first;
}

void LaneBorrowFsmAction::CheckTrafficStatus(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  // initialize
  mutable_path_decider->set_is_in_crowd_traffic(false);
  const auto& config = config_.path_lane_borrow_decider_config();
  // not allow laneborrow in crowd traffic
  if (!config.enable_crowd_traffic_laneborrow()) {
    return;
  }

  const auto& count_limit = config.in_crowd_traffic_check_count_limit();
  if (mutable_path_decider->is_adc_near_junction() ||
      mutable_path_decider->is_obs_near_junction() ||
      injector_->adc_in_junction_info_.first || injector_->is_in_play_street) {
    mutable_path_decider->set_crowd_traffic_check_count(-count_limit);
    return;
  }

  const auto single_lane_car_number_limit =
      config.crowd_traffic_car_number_limit();

  //
  if ((left_static_obs_.size() >=
           static_cast<size_t>(single_lane_car_number_limit) ||
       left_lane_veh_.size() >=
           static_cast<size_t>(single_lane_car_number_limit)) &&
      (ref_static_obs_.size() >=
           static_cast<size_t>(single_lane_car_number_limit) ||
       ref_lane_veh_.size() >=
           static_cast<size_t>(single_lane_car_number_limit)) &&
      (right_static_obs_.size() >=
           static_cast<size_t>(single_lane_car_number_limit) ||
       right_lane_veh_.size() >=
           static_cast<size_t>(single_lane_car_number_limit))) {
    mutable_path_decider->set_is_in_crowd_traffic(true);
    mutable_path_decider->set_crowd_traffic_check_count(count_limit);
    AINFO << "[CrowdTraffic] ADC is in crowd traffic.";
    return;
  }
  std::vector<const Obstacle*> veh_obs_ahead;
  // veh_obs_ahead.insert(veh_obs_ahead.end(), left_static_obs_.begin(),
  //                      left_static_obs_.end());
  // veh_obs_ahead.insert(veh_obs_ahead.end(), ref_static_obs_.begin(),
  //                      ref_static_obs_.end());
  // veh_obs_ahead.insert(veh_obs_ahead.end(), right_static_obs_.begin(),
  //                      right_static_obs_.end());
  veh_obs_ahead.insert(veh_obs_ahead.end(), left_lane_veh_.begin(),
                       left_lane_veh_.end());
  veh_obs_ahead.insert(veh_obs_ahead.end(), ref_lane_veh_.begin(),
                       ref_lane_veh_.end());
  veh_obs_ahead.insert(veh_obs_ahead.end(), right_lane_veh_.begin(),
                       right_lane_veh_.end());
  if (veh_obs_ahead.size() <= 1) {
    mutable_path_decider->set_crowd_traffic_check_count(-count_limit);
    return;
  }
  std::sort(veh_obs_ahead.begin(), veh_obs_ahead.end(),
            [](const Obstacle* a, const Obstacle* b) {
              return a->PerceptionSLBoundary().start_s() <
                     b->PerceptionSLBoundary().start_s();
            });

  double car_gap = config.crowd_traffic_car_gap_distance_limit();
  int in_crowd_traffic = 0, in_clear_traffic = 0;
  for (size_t i = 0; i < veh_obs_ahead.size() - 1; ++i) {
    if (veh_obs_ahead[i]->Id() == veh_obs_ahead[i + 1]->Id()) continue;
    auto car1_sl = veh_obs_ahead[i]->PerceptionSLBoundary();
    car1_sl.set_start_s(car1_sl.start_s() - 0.5 * car_gap);
    car1_sl.set_end_s(car1_sl.end_s() + 0.5 * car_gap);
    auto car2_sl = veh_obs_ahead[i + 1]->PerceptionSLBoundary();
    car2_sl.set_start_s(car2_sl.start_s() - 0.5 * car_gap);
    car2_sl.set_end_s(car2_sl.end_s() + 0.5 * car_gap);

    if (util::IsLongitudinalOverlap(car1_sl, car2_sl) &&
        !util::IsLateralOverlap(car1_sl, car2_sl) &&
        ((veh_obs_ahead[i + 1]->PerceptionSLBoundary().start_s() -
          reference_line_info->AdcSlBoundary().end_s()) <
         config.crowd_traffic_overlap_car_lookforward_distance())) {
      AINFO << "[CrowdTraffic] car(" << veh_obs_ahead[i]->Id()
            << ") parallel wiht car(" << veh_obs_ahead[i + 1]->Id() << ").";
      ++in_crowd_traffic;
    } else {
      ++in_clear_traffic;
    }
  }
  if (0 == in_crowd_traffic ||
      in_crowd_traffic / (in_crowd_traffic + in_clear_traffic) <
          kTrafficCrowdRatio) {
    auto crowd_count = mutable_path_decider->crowd_traffic_check_count();
    mutable_path_decider->set_crowd_traffic_check_count(
        std::max(--crowd_count, -count_limit));
  } else {
    AINFO << "traffic ratio "
          << in_crowd_traffic / (in_crowd_traffic + in_clear_traffic);
    auto crowd_count = mutable_path_decider->crowd_traffic_check_count();
    mutable_path_decider->set_crowd_traffic_check_count(
        std::min(++crowd_count, count_limit));
  }
  if (mutable_path_decider->crowd_traffic_check_count() > 0) {
    mutable_path_decider->set_is_in_crowd_traffic(true);
    AINFO << "[CrowdTraffic] ADC is in crowd traffic.";
  }
}

void LaneBorrowFsmAction::CheckNearJunctionExitLaneBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_lane_borrow = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_lane_borrow();
  bool is_force_to_exit_laneborrow =
      IsForcedToExitLaneBorrow(reference_line_info);
  bool is_need_remote_request = IsBorrowNeedRemoteRequest(reference_line_info);
  mutable_lane_borrow->set_force_to_exit_laneborrow(
      is_force_to_exit_laneborrow);
  mutable_lane_borrow->set_is_need_remote_request(is_need_remote_request);

  CheckIsNeedGobackReferenceLane(frame, reference_line_info);
}

bool LaneBorrowFsmAction::IsForcedToExitLaneBorrow(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto check_time = mutable_path_decider->force_to_exit_laneborrow_count();
  bool exit_borrow_in_solid_line =
      is_obs_in_front.first && is_obs_in_front.second &&
      reference_line_info->IsLocatedInSolidLine(adc_sl);
  bool exit_borrow_in_reference_lane =
      !is_obs_in_front.second && reference_line_info->IsAdcCenterInLane();
  if (exit_borrow_in_solid_line || exit_borrow_in_reference_lane) {
    mutable_path_decider->set_force_to_exit_laneborrow_count(++check_time);
  } else {
    mutable_path_decider->set_force_to_exit_laneborrow_count(0);
  }
  return mutable_path_decider->force_to_exit_laneborrow_count() >=
         kForceToExitLaneBorrowCount;
}

void LaneBorrowFsmAction::CheckIsNeedGobackReferenceLane(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  bool has_left_routing_lane = reference_line_info->HasLeftNeighborRoutingLane(
      "goback", injector_->planning_context());
  mutable_path_decider->set_has_left_neighbor_routing_lane(
      has_left_routing_lane);
  bool safe_to_goback = reference_line_info->GetIsClearToChangeLane();
  if (!has_left_routing_lane) {
    mutable_path_decider->set_is_need_goback_reference_lane(true);
    mutable_path_decider->set_is_need_go_left_lane(false);
    AINFO << "[goback] has no left routing lane, need go back reference lane.";
    return;
  }
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  double to_solid_line_distance =
      std::fmin(reference_line_info->GetRemainDistanceToSolidLine(),
                reference_line_info->GetRemainDistanceToJunction());
  auto check_time = mutable_path_decider->goback_near_junction_check_count();
  if (!is_obs_in_front.second && safe_to_goback &&
      (is_obs_in_front.first || to_solid_line_distance > 0.0)) {
    mutable_path_decider->set_goback_near_junction_check_count(++check_time);
  } else {
    AINFO << "[goback] front obs in left/current lane: "
          << is_obs_in_front.first << "," << is_obs_in_front.second
          << ". safe to goback: " << safe_to_goback
          << ". distance(>0.0?): " << to_solid_line_distance;
    mutable_path_decider->set_goback_near_junction_check_count(
        std::max(--check_time, 0));
  }
  int32_t obs_keep_check_threshold =
      GetKeepCheckCountLimit(frame, reference_line_info);
  if (safe_to_goback &&
      mutable_path_decider->goback_near_junction_check_count() >=
          obs_keep_check_threshold) {
    mutable_path_decider->set_is_need_goback_reference_lane(true);
    mutable_path_decider->set_is_need_go_left_lane(false);
    mutable_path_decider->set_goleft_near_junction_check_count(0);
  } else {
    mutable_path_decider->set_is_need_goback_reference_lane(false);
  }
  AINFO << "[goback] count: "
        << mutable_path_decider->goback_near_junction_check_count()
        << ", count limit: " << obs_keep_check_threshold
        << ", force_to_exit_laneborrow: "
        << injector_->planning_context()
               ->planning_status()
               .lane_borrow()
               .force_to_exit_laneborrow()
        << ", is_need_goback_reference_lane: "
        << mutable_path_decider->is_need_goback_reference_lane();
}

void LaneBorrowFsmAction::CheckIsNeedGotoLeftNeighborLane(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  bool has_left_lane =
      injector_->adc_in_junction_info_.first
          ? !IsLeftNeighbourForwardLaneEmpty(reference_line_info)
          : reference_line_info->HasLeftNeighborRoutingLane(
                "goleft", injector_->planning_context());
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_has_left_neighbor_routing_lane(has_left_lane);
  CheckIsSafeToLaneBorrow(*reference_line_info);
  bool safe_to_left_lane = injector_->planning_context()
                               ->planning_status()
                               .lane_borrow()
                               .is_left_borrow_safe();
  auto check_time = mutable_path_decider->goleft_near_junction_check_count();
  if (!is_obs_in_front.first && has_left_lane && is_obs_in_front.second &&
      safe_to_left_lane) {
    mutable_path_decider->set_goleft_near_junction_check_count(++check_time);
  } else {
    AINFO << "[go left] front obs in left/current lane: "
          << is_obs_in_front.first << "," << is_obs_in_front.second
          << ". has_left_lane: " << has_left_lane
          << ". safe to go left: " << safe_to_left_lane;
    mutable_path_decider->set_goleft_near_junction_check_count(
        std::max(--check_time, 0));
  }
  int32_t obs_keep_check_threshold =
      GetKeepCheckCountLimit(frame, reference_line_info);
  if (safe_to_left_lane &&
      mutable_path_decider->goleft_near_junction_check_count() >=
          obs_keep_check_threshold) {
    mutable_path_decider->set_is_need_go_left_lane(true);
    mutable_path_decider->set_is_need_goback_reference_lane(false);
    mutable_path_decider->set_goback_near_junction_check_count(0);
  } else {
    mutable_path_decider->set_is_need_go_left_lane(false);
  }
  AINFO << "[go left] count: "
        << mutable_path_decider->goleft_near_junction_check_count()
        << ", count limit: " << obs_keep_check_threshold
        << ", is_need_go_left_lane: "
        << mutable_path_decider->is_need_go_left_lane();
}

void LaneBorrowFsmAction::CheckIsNeedPassTruck(
    ReferenceLineInfo* const reference_line_info) {
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  bool has_left_lane = false;
  CheckLeftLaneBorrowable(*reference_line_info, &has_left_lane);
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_has_left_neighbor_routing_lane(has_left_lane);
  CheckIsSafeToLaneBorrow(*reference_line_info);
  bool safe_to_left_lane = injector_->planning_context()
                               ->planning_status()
                               .lane_borrow()
                               .is_left_borrow_safe();
  auto check_time = mutable_path_decider->truck_check_count();
  if (!is_obs_in_front.first && has_left_lane && is_obs_in_front.second &&
      safe_to_left_lane) {
    mutable_path_decider->set_truck_check_count(++check_time);
  } else {
    AINFO << "[truck] front obs in left/current lane: " << is_obs_in_front.first
          << "," << is_obs_in_front.second
          << ". has_left_lane: " << has_left_lane
          << ". safe to go left: " << safe_to_left_lane;
    mutable_path_decider->set_truck_check_count(std::max(--check_time, 0));
  }
  const auto& config = config_.path_lane_borrow_decider_config();
  int32_t obs_keep_check_threshold = config.truck_keep_check_time_threshold();
  if (safe_to_left_lane &&
      mutable_path_decider->truck_check_count() >= obs_keep_check_threshold) {
    mutable_path_decider->set_is_need_go_left_lane(true);
    mutable_path_decider->set_is_need_goback_reference_lane(false);
    mutable_path_decider->set_goback_near_junction_check_count(0);
  } else {
    mutable_path_decider->set_is_need_go_left_lane(false);
  }
  AINFO << "[truck] count: " << mutable_path_decider->truck_check_count()
        << ", count limit: " << obs_keep_check_threshold
        << ", is_need_go_left_lane: "
        << mutable_path_decider->is_need_go_left_lane();
}

std::pair<bool, bool> LaneBorrowFsmAction::CheckIsObsInFrontOfAdc(
    ReferenceLineInfo* const reference_line_info) {
  if (has_checked_obs_in_front_) {
    return is_obs_in_front_;
  }

  const auto& path_decision = reference_line_info->path_decision();
  const auto& indexed_obstacles = path_decision->obstacles();
  auto obstacle_list = indexed_obstacles.Items();
  std::vector<const Obstacle*> left_lane_obstacle;
  std::vector<const Obstacle*> ref_lane_obstacle;
  const auto& config = config_.path_lane_borrow_decider_config();
  std::pair<bool, bool> is_obs_in_front = std::make_pair(false, false);

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  hdmap::Id neighbor_lane_id;
  double left_neighbor_lane_width = 0.0;
  if (!reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width)) {
    AINFO
        << "reference lane id: "
        << reference_line_info->LocateLaneInfo(adc_sl.end_s())->lane().id().id()
        << " has no left neighbor lane.";
    is_obs_in_front.first = true;
    has_checked_obs_in_front_ = true;
    is_obs_in_front_ = is_obs_in_front;
    return is_obs_in_front;
  }

  for (const auto* obstacle : obstacle_list) {
    if (nullptr == obstacle) {
      continue;
    }
    if (obstacle->IsVirtual()) {
      continue;
    }

    if (obstacle->IsStatic()) {
      const auto& lane_width =
          reference_line_info->GetLaneWidthBaseOnAdcCenter();
      double left_width = lane_width.first;
      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      const auto& veh_param =
          common::VehicleConfigHelper::GetConfig().vehicle_param();
      double half_width = veh_param.width() * 0.5;
      double check_distance = injector_->adc_in_junction_info_.first
                                  ? kFrontObsMaxDistanceInJunction
                                  : kFrontObsMaxDistance;
      if ((obs_sl.end_l() < left_width + left_neighbor_lane_width -
                                config.left_neighbor_lane_obs_buffer()) &&
          obs_sl.start_l() >
              left_width + config.left_neighbor_lane_obs_buffer() &&
          obs_sl.end_s() > adc_sl.end_s() &&
          (obs_sl.start_s() - adc_sl.end_s()) < check_distance) {
        left_lane_obstacle.emplace_back(obstacle);
      }
      if (obs_sl.end_l() < left_width - config.reference_lane_obs_buffer() &&
          obs_sl.start_l() > -half_width - config.reference_lane_obs_buffer() &&
          obs_sl.end_s() > adc_sl.end_s() &&
          (obs_sl.start_s() - adc_sl.end_s()) < kFrontObsMaxDistance) {
        ref_lane_obstacle.emplace_back(obstacle);
      }
    }
  }
  AINFO << "There are " << left_lane_obstacle.size()
        << " obs in the left neighbor lane, " << ref_lane_obstacle.size()
        << " obs in the reference lane.";

  is_obs_in_front.first = !left_lane_obstacle.empty();
  is_obs_in_front.second = !ref_lane_obstacle.empty();
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_has_obs_in_reference_lane(is_obs_in_front.second);
  has_checked_obs_in_front_ = true;
  is_obs_in_front_ = is_obs_in_front;
  return is_obs_in_front;
}

void LaneBorrowFsmAction::CheckVehicleObsInFrontOfAdc(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  // initialize is_left_borrowable / is_right_borrowable
  mutable_laneborrow->set_is_left_borrowable(true);
  mutable_laneborrow->set_is_right_borrowable(true);
  left_lane_veh_.clear();
  ref_lane_veh_.clear();
  right_lane_veh_.clear();
  left_static_obs_.clear();
  ref_static_obs_.clear();
  right_static_obs_.clear();
  ref_lane_obs_.clear();
  // vehicle polygon
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  //
  hdmap::Id left_neighbor_lane_id, right_neighbor_lane_id;
  double left_neighbor_lane_width = 0.0, right_neighbor_lane_width = 0.0;
  bool left_lane_exist = true, right_lane_exist = true;
  // check neighour left lane
  if (!reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl.end_s(),
          &left_neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl.end_s(),
          &left_neighbor_lane_id, &left_neighbor_lane_width)) {
    mutable_laneborrow->set_is_left_borrowable(false);
    mutable_laneborrow->set_is_left_borrow_safe(false);
    left_lane_exist = false;
  }
  // check neighour right lane
  if (!reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, adc_sl.end_s(),
          &right_neighbor_lane_id, &right_neighbor_lane_width) &&
      !reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightReverse, adc_sl.end_s(),
          &right_neighbor_lane_id, &right_neighbor_lane_width)) {
    mutable_laneborrow->set_is_right_borrowable(false);
    mutable_laneborrow->set_is_right_borrow_safe(false);
    right_lane_exist = false;
  }

  // left neighour lane is not exist, right neighour lane is not exist
  if (!(left_lane_exist || right_lane_exist)) {
    mutable_laneborrow->set_is_necessary_to_laneborrow(false);
    mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::NO_BORROW);
    reference_line_info->set_is_path_lane_borrow(false);
    return;
  }
  auto obstacle_list =
      reference_line_info->path_decision()->obstacles().Items();
  const auto& config = config_.path_lane_borrow_decider_config();
  double distance_to_signal = reference_line_info->GetRemainDistanceToSignal();

  for (const auto* obstacle : obstacle_list) {
    if (nullptr == obstacle) {
      continue;
    }
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    const auto& lane_width = reference_line_info->GetLaneWidthByS(
        (obs_sl.start_s() + obs_sl.end_s()) * 0.5);
    double left_width = lane_width.first;
    double right_width = lane_width.second;
    double check_distance = injector_->adc_in_junction_info_.first
                                ? kFrontObsMaxDistanceInJunction
                                : kFrontObsMaxDistance;
    if (left_lane_exist &&
        util::HasCommon(obs_sl.start_l(), obs_sl.end_l(), left_width,
                        left_width + left_neighbor_lane_width) &&
        util::HasCommon(
            obs_sl.start_s(), obs_sl.end_s(),
            adc_sl.start_s() - config.pass_by_safe_distance(),
            adc_sl.end_s() + std::fmin(check_distance, distance_to_signal))) {
      if (PerceptionObstacle::VEHICLE == obstacle->Perception().type()) {
        left_lane_veh_.emplace_back(obstacle);
      }
      if (obstacle->IsStatic() ||
          obstacle->speed() < config.slow_vehicle_speed_limit()) {
        left_static_obs_.emplace_back(obstacle);
      }
    }
    bool is_tire_lifter = obstacle->Perception().type() ==
                              perception::PerceptionObstacle::WHEELCRANE;
    if (obstacle->Perception().type() == PerceptionObstacle::STACKER ||
        obstacle->Perception().type() == PerceptionObstacle::FORKLIFT_STACKER ||
        is_tire_lifter) {
      right_width = right_width + FLAGS_stacker_borrow_lateral_distance;
      left_width = left_width + FLAGS_stacker_borrow_lateral_distance;
    }
    double borrow_consider_obs = kFrontObsMaxDistance;
    if (is_tire_lifter) {
      borrow_consider_obs = FLAGS_wheelcrane_consider_distance;
    }
    if (util::HasCommon(obs_sl.start_l(), obs_sl.end_l(), -right_width,
                        left_width) &&
        util::HasCommon(obs_sl.start_s(), obs_sl.end_s(),
                        adc_sl.start_s() - config.pass_by_safe_distance(),
                        adc_sl.end_s() + std::fmin(borrow_consider_obs,
                                                   distance_to_signal))) {
      if (PerceptionObstacle::VEHICLE == obstacle->Perception().type()) {
        ref_lane_veh_.emplace_back(obstacle);
      }
      if (obstacle->IsStatic() ||
          obstacle->speed() < config.slow_vehicle_speed_limit()) {
        ref_static_obs_.emplace_back(obstacle);
      }
    }
    if (util::HasCommon(obs_sl.start_l(), obs_sl.end_l(),
                        -right_width - FLAGS_slef_borrow_buffer,
                        left_width + FLAGS_slef_borrow_buffer) &&
        util::HasCommon(obs_sl.start_s(), obs_sl.end_s(),
                        adc_sl.start_s() - config.pass_by_safe_distance(),
                        adc_sl.end_s() + std::fmin(kFrontObsMaxDistance,
                                                   distance_to_signal))) {
      if (obstacle->IsStatic() ||
          obstacle->speed() < config.slow_vehicle_speed_limit()) {
        ref_lane_obs_.emplace_back(obstacle);
      }
    }

    // right neighour lane
    if (right_lane_exist &&
        util::HasCommon(obs_sl.start_l(), obs_sl.end_l(), -right_width,
                        -(right_width + right_neighbor_lane_width)) &&
        util::HasCommon(
            obs_sl.start_s(), obs_sl.end_s(),
            adc_sl.start_s() - config.pass_by_safe_distance(),
            adc_sl.end_s() + std::fmin(check_distance, distance_to_signal))) {
      if (PerceptionObstacle::VEHICLE == obstacle->Perception().type()) {
        right_lane_veh_.emplace_back(obstacle);
      }
      if (obstacle->IsStatic() ||
          obstacle->speed() < config.slow_vehicle_speed_limit()) {
        right_static_obs_.emplace_back(obstacle);
      }
    }
  }

  AINFO << left_static_obs_.size() << " static obstacles in the left lane, "
        << ref_static_obs_.size() << " static obstacles in the reference lane, "
        << right_static_obs_.size() << " static obstacles in the right lane, "
        << left_lane_veh_.size() << " static vehicles in the left lane, "
        << ref_lane_veh_.size() << " static vehicles in the reference lane, "
        << right_lane_veh_.size() << " static vehicles in the right lane.";
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_front_static_obstacle_id("");
  std::sort(ref_static_obs_.begin(), ref_static_obs_.end(),
            [](const Obstacle* a, const Obstacle* b) {
              return a->PerceptionSLBoundary().start_s() <
                     b->PerceptionSLBoundary().start_s();
            });
  std::sort(ref_lane_obs_.begin(), ref_lane_obs_.end(),
            [](const Obstacle* a, const Obstacle* b) {
              return a->PerceptionSLBoundary().start_s() <
                     b->PerceptionSLBoundary().start_s();
            });
  std::vector<std::string> ref_obs_ids;
  for (auto ob : ref_lane_obs_) {
    ref_obs_ids.push_back(ob->Id());
  }
  reference_line_info->SetRefObstacleIds(ref_obs_ids);
  // borrrow for wheelcrane first
  bool is_get_wheel_crane = false;
  for (auto ob : ref_lane_obs_) {
    bool is_wheel_scrane =
        ob->Perception().type() == perception::PerceptionObstacle::WHEELCRANE;
    if (!is_wheel_scrane) {
      continue;
    }
    double far_away_distance = FLAGS_wheelcrane_consider_distance;
    bool is_big_wheel_crane = ob->Perception().length() > kWheelCraneLength ||
                              ob->Perception().width() > kWheelCraneLength;
    if (util::WithinBound(ob->PerceptionSLBoundary().end_s(), adc_sl.start_s(),
                          adc_sl.end_s() + far_away_distance) &&
        (!is_get_wheel_crane || is_big_wheel_crane)) {
      mutable_path_decider_status->set_front_static_obstacle_id(ob->Id());
      is_get_wheel_crane = true;
    }
  }
  if (is_get_wheel_crane) {
    return;
  }
  for (auto ob : ref_static_obs_) {
    if (util::WithinBound(
            ob->PerceptionSLBoundary().end_s(), adc_sl.start_s(),
            adc_sl.end_s() + config.obs_far_away_safe_distance())) {
      AINFO << "laneborrow obstacle id " << ob->Id();
      mutable_path_decider_status->set_front_static_obstacle_id(ob->Id());
      return;
    }
  }
}

bool LaneBorrowFsmAction::IsBlockingObstacleTruck(
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info->path_decision()->obstacles().Find(
          blocking_obstacle_id);
  if (nullptr == blocking_obstacle) {
    return false;
  }
  if (PerceptionObstacle::VEHICLE == blocking_obstacle->Perception().type() &&
      blocking_obstacle->Perception().height() >
          config_.path_lane_borrow_decider_config().truck_height_threshold()) {
    AINFO << "blocking obs: " << blocking_obstacle_id
          << " is a truck, height: " << blocking_obstacle->Perception().height()
          << ", height limit: "
          << config_.path_lane_borrow_decider_config().truck_height_threshold();
    return true;
  }
  return false;
}

int32_t LaneBorrowFsmAction::GetKeepCheckCountLimit(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  const auto& config = config_.path_lane_borrow_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  bool adc_in_solid_line = reference_line_info->IsInEitherSolidLine(adc_sl);
  // baseline waiting time
  int32_t obs_keep_check_threshold =
      injector_->adc_in_junction_info_.first
          ? config.in_junction_obs_keep_check_time_threshold()
          : config.near_junction_obs_keep_check_time_threshold();
  if (adc_in_solid_line) {
    double distance_to_signal =
        reference_line_info->GetRemainDistanceToSignal();
    obs_keep_check_threshold =
        distance_to_signal < config.distance_to_signal_limit_level_1()
            ? config.in_solid_line_check_time_limit_level_1()
            : (distance_to_signal < config.distance_to_signal_limit_level_2()
                   ? config.in_solid_line_check_time_limit_level_2()
                   : config.in_solid_line_check_time_limit_level_3());
  }
  // more than one vehicle
  int veh_count = 0;
  if (!injector_->adc_in_junction_info_.first) {
    veh_count = reference_line_info->IsAdcCenterInLane()
                    ? ref_lane_veh_.size()
                    : left_lane_veh_.size();
  }
  obs_keep_check_threshold +=
      std::max(veh_count - 1, 0) * config.waiting_time_for_one_more_car();
  // has truck in front
  if (IsBlockingObstacleTruck(reference_line_info) &&
      !injector_->adc_in_junction_info_.first) {
    obs_keep_check_threshold += config.extra_waiting_time_for_truck();
  }
  // red light
  bool is_red_light = perception::TrafficLight::RED == frame->signal_color_;
  bool first_time_red_light = is_red_light && !has_checked_red_light_;
  if (first_time_red_light && adc_in_solid_line) {
    obs_keep_check_threshold += config.extra_waiting_time_for_red_light();
    has_checked_red_light_ = true;
  }
  obs_keep_check_threshold =
      std::max(obs_keep_check_threshold, last_obs_keep_check_threshold_);
  obs_keep_check_threshold =
      std::min(obs_keep_check_threshold, config.max_waiting_time());
  last_obs_keep_check_threshold_ = obs_keep_check_threshold;
  return obs_keep_check_threshold;
}

void LaneBorrowFsmAction::CheckAuxiliaryRoadStatus(
    ReferenceLineInfo* const reference_line_info) {
  // vehicle polygon
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  // single lane flag
  injector_->is_single_lane_ =
      !reference_line_info->HasNeighborLane(adc_sl.end_s());
  // auxiliary road flag
  injector_->is_auxiliary_road_ =
      FLAGS_enable_separate_auxiliary_road_borrow
          ? reference_line_info->IsADCLocatedInAuxiliaryRoad()
          : injector_->is_single_lane_;
  reference_line_info->SetIsAuxiliaryRoad(injector_->is_auxiliary_road_);
  // AINFO << "[AuxiliaryRoad] adc in auxiliary_road: "
  //       << injector_->is_auxiliary_road_
  //       << ", is single lane: " << injector_->is_single_lane_;
}

void LaneBorrowFsmAction::CheckLaneBorrowInAuxiliaryRoad(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->clear_decided_side_pass_direction();
  if (!FindLaneBorrowObstacle(reference_line_info) ||
      IsLeftNeighbourForwardLaneEmpty(reference_line_info)) {
    mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(false);
    return;
  }

  bool adc_in_junction = reference_line_info->AdcIsOnOverlapJunction() &&
                         !reference_line_info->IsADCInCurbArea();
  const auto& config = config_.path_lane_borrow_decider_config();
  CheckIsSafeToLaneBorrow(*reference_line_info);
  const bool is_left_borrow_safe = mutable_laneborrow->is_left_borrow_safe();

  if (adc_in_junction) {
    AINFO << "[AuxiliaryRoad] adc in auxiliary road junction now.";
    double remain_left_lane_length = 0.0;
    reference_line_info->GetRemainDistanceToRoutingEndPoint(
        CheckDirection::LEFT, injector_->planning_context(),
        &remain_left_lane_length);
    double length_limit =
        config.auxiliary_road_in_junction_left_lane_length_limit();
    bool is_necessary_to_borrow_in_junction =
        remain_left_lane_length > length_limit &&
        HasSingleReferenceLine(*frame) && IsWithinSidePassingSpeedADC() &&
        is_left_borrow_safe;
    if (is_necessary_to_borrow_in_junction) {
      mutable_path_decider->add_decided_side_pass_direction(
          PathDeciderStatus::LEFT_BORROW);
      // mutable_path_decider->set_is_in_path_lane_borrow_scenario(true);
      mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(true);
    } else {
      AINFO << "[AuxiliaryRoad] no laneborrow in junction, left lane length: "
            << remain_left_lane_length << ", limit: " << length_limit
            << ", safe to borrow: " << is_left_borrow_safe;
      mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(false);
    }
    return;
  }

  double distance_to_curb_area =
      std::fmin(reference_line_info->GetRemainDistanceToCurbArea(),
                reference_line_info->GetRemainDistanceToSignal());
  double distance_limit = config.auxiliary_road_distance_to_curb_limit();
  bool is_necessary_to_borrow = distance_to_curb_area > distance_limit &&
                                HasSingleReferenceLine(*frame) &&
                                IsWithinSidePassingSpeedADC() &&
                                is_left_borrow_safe;
  if (is_necessary_to_borrow) {
    mutable_path_decider->add_decided_side_pass_direction(
        PathDeciderStatus::LEFT_BORROW);
    // mutable_path_decider->set_is_in_path_lane_borrow_scenario(true);
    mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(true);
  } else {
    AINFO << "[AuxiliaryRoad] no laneborrow, distance_to_curb_area: "
          << distance_to_curb_area << ", limit: " << distance_limit
          << ", safe to borrow: " << is_left_borrow_safe;
    mutable_laneborrow->set_is_need_laneborrow_in_auxiliary_road(false);
  }
}

void LaneBorrowFsmAction::CheckAuxiliaryRoadExitLaneBorrow(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  double remain_distance = 0.0;
  reference_line_info->GetRemainDistanceToRoutingEndPoint(
      CheckDirection::LEFT, injector_->planning_context(), &remain_distance);
  remain_distance = std::fmin(
      remain_distance, reference_line_info->GetRemainDistanceToCurbArea());
  remain_distance = std::fmin(remain_distance,
                              reference_line_info->GetRemainDistanceToSignal());
  bool adc_in_junction = reference_line_info->AdcIsOnOverlapJunction() &&
                         !reference_line_info->IsADCInCurbArea();
  const auto& config = config_.path_lane_borrow_decider_config();
  double distance_limit =
      adc_in_junction
          ? config.auxiliary_road_in_junction_left_lane_length_limit()
          : config.auxiliary_road_distance_to_curb_limit();
  if (remain_distance < distance_limit) {
    AINFO << "[AuxiliaryRoad] need exit laneborrow, remain distance: "
          << remain_distance << ", limit: " << distance_limit;
    mutable_laneborrow->set_need_exit_laneborrow_in_auxiliary_road(true);
  } else {
    mutable_laneborrow->set_need_exit_laneborrow_in_auxiliary_road(false);
  }
}

double LaneBorrowFsmAction::GetExtraSafeCheckTime(
    const ReferenceLineInfo& reference_line_info,
    const LaneborrowStatus::LaneborrowDirection direction) {
  // lateral velocity estimate
  double adc_lateral_v_estimate =
      config_.path_lane_borrow_decider_config()
          .safe_check_adc_lateral_velocity_estimate();
  // current lane width
  const auto& lane_width = reference_line_info.GetLaneWidthBaseOnAdcCenter();
  // left lane width
  double left_half_width = lane_width.first;
  // right lane width
  double right_half_width = lane_width.second;
  // vehicle polygon
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  // vehicle center lateral position
  double adc_center_l = 0.5 * (adc_sl.start_l() + adc_sl.end_l());
  // duration for enter neighbour lane
  double extra_safe_check_time = 0.0;
  if (LaneborrowStatus::LEFT_BORROW == direction) {
    extra_safe_check_time = adc_center_l > left_half_width
                                ? 0.0
                                : std::fmin((left_half_width - adc_center_l) /
                                                adc_lateral_v_estimate,
                                            kMaxExtraSafeCheckTime);
  } else if (LaneborrowStatus::RIGHT_BORROW == direction) {
    extra_safe_check_time = adc_center_l < -right_half_width
                                ? 0.0
                                : std::fmin((adc_center_l + right_half_width) /
                                                adc_lateral_v_estimate,
                                            kMaxExtraSafeCheckTime);
  } else {
    if (util::WithinBound(adc_center_l, -right_half_width, left_half_width)) {
      extra_safe_check_time = 0.0;
    } else {
      if (adc_center_l - left_half_width > 0.0) {
        extra_safe_check_time =
            std::fmin((adc_center_l - left_half_width) / adc_lateral_v_estimate,
                      kMaxExtraSafeCheckTime);
      } else if (adc_center_l < -right_half_width) {
        extra_safe_check_time = std::fmin(
            (-right_half_width - adc_center_l) / adc_lateral_v_estimate,
            kMaxExtraSafeCheckTime);
      }
    }
  }
  ADEBUG << "extra_safe_check_time: " << extra_safe_check_time;
  return extra_safe_check_time;
}

}  // namespace planning
}  // namespace century
