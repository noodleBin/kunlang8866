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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::common::Status;
using century::common::VehicleConfigHelper;
using century::perception::PerceptionObstacle;

namespace {
constexpr double kEpsilon = 1e-3;
constexpr double kLonDistance = 10.0;
constexpr double kSameDirectionThr = M_PI * 0.5;
constexpr double kStepCheckDistance = 2.0;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr int32_t kForceToExitLaneBorrowCount = 10;
constexpr double kFrontObsMaxDistance = 40.0;
}  // namespace

PathLaneBorrowDecider::PathLaneBorrowDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  lane_borrow_fsm_ = std::make_shared<LaneBorrowFsm>();
  lane_borrow_fsm_->InitLaneBorrowFsm(config, injector);
}

Status PathLaneBorrowDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // skip path_lane_borrow_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    // for debug
    AINFO << "skip due to reusing path";
    return Status::OK();
  }
  reference_line_info->set_is_path_lane_borrow(false);
  if (reference_line_info->lane_change_path_reusable()) {
    AINFO << "skip due to reusing LANE CHANGE anchor path";
    return Status::OK();
  }
  /*
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();

  // By default, don't borrow any lane.
  injector_->is_auxiliary_road_ = false;
  // Check if lane-borrowing is needed, if so, borrow lane.
  if (!FLAGS_allow_lane_borrow_fsm &&
      IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
    if (mutable_path_decider->lane_borrow_lane_id().empty()) {
      std::string lane_borrow_id = reference_line_info->Lanes().Id();
      mutable_path_decider->set_lane_borrow_lane_id(lane_borrow_id);
      frame->UpdateLaneBorrowLaneId(lane_borrow_id);
    }
  } else if (!CheckAdcPostureIsCorrect()) {
    if (mutable_path_decider->lane_borrow_lane_id().empty()) {
      std::string lane_borrow_id = reference_line_info->Lanes().Id();
      mutable_path_decider->set_lane_borrow_lane_id(lane_borrow_id);
      frame->UpdateLaneBorrowLaneId(lane_borrow_id);
    }
  } else if (0 == std::strcmp(
                      reference_line_info->Lanes().Id().c_str(),
                      mutable_path_decider->lane_borrow_lane_id().c_str()) &&
             reference_line_info->IsAdcLocatedInLane()) {
    std::string lane_borrow_id = "";
    mutable_path_decider->set_lane_borrow_lane_id(lane_borrow_id);
    frame->UpdateLaneBorrowLaneId(lane_borrow_id);
  }
  */
  if (FLAGS_allow_lane_borrow_fsm) {
    lane_borrow_fsm_->ExecuteFsm(frame, reference_line_info);
    if (!FLAGS_enable_near_junction_laneborrow &&
        !util::IsLaneChange(injector_->planning_context())) {
      IsStopNearIntersectionLongTime(*reference_line_info);
    }
  }
  return Status::OK();
}

bool PathLaneBorrowDecider::CheckAdcPostureIsCorrect() {
  if (!FLAGS_enable_extend_path_bound_base_adc_posture) {
    return true;
  }
  if (util::IsLaneChange(injector_->planning_context())) {
    return true;
  }
  auto heading_diff = reference_line_info_->GetAdcHeadingDiffWithRefLine();
  if (reference_line_info_->IsAdcLocatedInLane() &&
      std::fabs(heading_diff) < FLAGS_adc_posture_correct_check_heading_diff) {
    return true;
  }
  AINFO << "adc posture is not correct when not in lane borrow/change";
  return false;
}

bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();

  const auto& adc_slboundary = reference_line_info.AdcSlBoundary();
  auto lane_type = reference_line_info.GetLaneType();
  if (!mutable_path_decider_status->is_in_path_lane_borrow_scenario() &&
      hdmap::Lane::PLAY_STREET == lane_type) {
    double adc_center_l =
        (adc_slboundary.start_l() + adc_slboundary.end_l()) * 0.5;
    if (!reference_line_info.IsAdcCenterInLane() &&
        mutable_path_decider_status->decided_side_pass_direction().empty()) {
      if (adc_center_l > 0.0) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::LEFT_BORROW);
      } else {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::RIGHT_BORROW);
      }
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      return true;
    }
  }
  CheckBorrowRemainDistance();

  AINFO << "is in lane_borrow:  "
        << mutable_path_decider_status->is_in_path_lane_borrow_scenario();
  AINFO << "able_to_use_self_lane_counter :  "
        << mutable_path_decider_status->able_to_use_self_lane_counter();

  if (FLAGS_enable_near_junction_laneborrow) {
    CheckAdcDistance(reference_line_info);
  }
  CheckTrafficStatus(reference_line_info);

  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // If the last decision does not grow the path, the decision will disappear,
    // resulting in the need to switch to selfborow and then enter the
    // laneborrow; Or there is no obstacle avoidance decision in the next frame
    if (FLAGS_enable_near_junction_laneborrow &&
        mutable_path_decider_status->is_adc_near_junction()) {
      CheckNearJunctionExitLaneBorrow(reference_line_info);
    } else {
      CheckIsNeedExitLaneBorrow(reference_line_info);
    }
  } else {
    CheckIsCouldEnterLaneBorrow(frame, reference_line_info);
  }
  if (!FLAGS_enable_near_junction_laneborrow &&
      !util::IsLaneChange(injector_->planning_context())) {
    IsStopNearIntersectionLongTime(reference_line_info);
  }

  AINFO << "[Junction] adc/obs near junction: "
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

  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

double PathLaneBorrowDecider::CheckBorrowRemainDistance() {
  double remain_distane =
      std::fmin(reference_line_info_->GetRemainDistanceToSolidLine(),
                reference_line_info_->GetRemainDistanceToJunction());
  if (0 && remain_distane >
               reference_line_info_->reference_line().Length() + kEpsilon) {
    remain_distane = reference_line_info_->GetRemainDistanceForBack(
        injector_->planning_context());
  }
  const auto& config = config_.path_lane_borrow_decider_config();
  not_borrow_reverse_ =
      (remain_distane < config.near_junction_not_borrow_reverse_distance());
  injector_->near_junction_keep_straight_ =
      (remain_distane < config.near_junction_distance_check_borrow());
  if (remain_distane <
      reference_line_info_->reference_line().Length() + kEpsilon) {
    closest_stop_sign_start_s_ =
        remain_distane + reference_line_info_->AdcSlBoundary().end_s();
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

bool PathLaneBorrowDecider::CheckIsNeedExitLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  bool left_borrowable;
  bool right_borrowable;
  CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
  AINFO << "left_borrowable: " << left_borrowable
        << ", right_borrowable: " << right_borrowable;
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->clear_decided_side_pass_direction();
  if (left_borrowable) {
    mutable_path_decider_status->add_decided_side_pass_direction(
        PathDeciderStatus::LEFT_BORROW);
  }
  if (right_borrowable) {
    mutable_path_decider_status->add_decided_side_pass_direction(
        PathDeciderStatus::RIGHT_BORROW);
  }
  if (!right_borrowable && !left_borrowable) {
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
  }
  // If originally borrowing neighbor lane:
  if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6 &&
      !IsBorrowNeedRemoteRequest(reference_line_info)) {
    // If have been able to use self-lane for some time, then switch to
    // non-lane-borrowing.
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
    mutable_path_decider_status->clear_decided_side_pass_direction();
    AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
  }
  if (mutable_path_decider_status->is_in_crowd_traffic() &&
      reference_line_info.IsAdcLocatedInLane()) {
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
    mutable_path_decider_status->clear_decided_side_pass_direction();
    AINFO << "[CrowdTraffic] Exit laneborrow due to crowd traffic.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

/**
 * About near/in junction laneborrow,
 * @param IsBlockingObstacleFarFromIntersection used to determine whether to go
 * left (is_need_go_left_lane), always return true;
 * @param IsSidePassableObstacle return true when is_need_go_left_lane is true;
 * @param IsLaneTypeSupportBorrow return true when near or in junction.
 **/
bool PathLaneBorrowDecider::CheckIsCouldEnterLaneBorrow(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  if (!HasSingleReferenceLine(frame)) {
    return false;
  }

  if (!IsWithinSidePassingSpeedADC()) {
    AINFO << "IsWithinLaneBorrowSpeedADC";
    return false;
  }

  if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {
    AINFO << "IsBlockingObstacleFarFromIntersection";
    return false;
  }

  if (!IsBlockingObstacleWithinDestination(reference_line_info)) {
    AINFO << "NO need laneborrow for obs is close to destination";
    return false;
  }

  if (!IsSidePassableObstacle(reference_line_info)) {
    AINFO << "NO need laneborrow for no sidepassing obstacle.";
    return false;
  }

  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .is_in_crowd_traffic()) {
    AINFO << "No need laneborrow for crowd traffic.";
    return false;
  }

  CheckIsSafeToLaneBorrow(reference_line_info);
  bool is_left_borrow_safe = injector_->planning_context()
                                 ->planning_status()
                                 .lane_borrow()
                                 .is_left_borrow_safe();
  //because right borrow no consider left back obs.
  if (!is_left_borrow_safe && FLAGS_allow_lane_borrow_fsm) {
    ADEBUG << "Unsafe to left borrow.";
    return false;
  }

  if (FLAGS_enable_noborrow_nearobstacle &&
      hdmap::Lane::PLAY_STREET != reference_line_info.GetLaneType() &&
      IsNearObstacle(reference_line_info)) {
    return false;
  }

  if (!IsLaneTypeSupportBorrow(reference_line_info)) {
    AINFO << "lane type not support lane borrow.";
    return false;
  }

  AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  return true;
}

bool PathLaneBorrowDecider::IsLaneTypeSupportBorrow(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  if (FLAGS_enable_near_junction_laneborrow &&
      (mutable_path_decider_status->has_truck_near_junction() ||
       mutable_path_decider_status->is_obs_near_junction() ||
       injector_->adc_in_junction_info_.first)) {
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
    mutable_path_decider_status->add_decided_side_pass_direction(
        PathDeciderStatus::LEFT_BORROW);
    ADEBUG << "[Junction] switch from self borrow to LANE BORROW.";
    return true;
  } else if (path_decider_status.decided_side_pass_direction().empty()) {
    bool left_borrowable;
    bool right_borrowable;
    CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
    if (!left_borrowable && !right_borrowable) {
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      return false;
    } else {
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
      if (left_borrowable) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::LEFT_BORROW);
      }
      if (right_borrowable) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::RIGHT_BORROW);
      }
    }
  }
  return true;
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
//                 lane-change.
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
  return 1 == frame.reference_line_info().size();
}

bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC() {
  const auto& adc_speed_status = injector_->GetAdcSpeedStatus();
  if (AdcSpeedStatus::SPEED_LOWER == adc_speed_status) {
    return true;
  } else {
    return false;
  }
}

bool PathLaneBorrowDecider::IsWithinSuitableDistance(
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

  const auto& adc_v = reference_line_info_->vehicle_state().linear_velocity();
  double ttc_distance =
      std::abs(adc_v - blocking_obstacle->speed()) * FLAGS_lane_borrow_ttc_time;
  double suitable_distance = 0.0;
  if (PerceptionObstacle::PEDESTRIAN ==
          blocking_obstacle->Perception().type() ||
      PerceptionObstacle::BICYCLE == blocking_obstacle->Perception().type()) {
    suitable_distance =
        std::fmin(ttc_distance, FLAGS_lane_borrow_distance_pedestrian);
  } else if (PerceptionObstacle::VEHICLE ==
             blocking_obstacle->Perception().type()) {
    suitable_distance = std::fmin(ttc_distance, FLAGS_lane_borrow_distance_car);
  } else {
    suitable_distance =
        std::fmin(ttc_distance, FLAGS_lane_borrow_distance_unknown);
  }

  double actual_distance = blocking_obstacle->PerceptionSLBoundary().start_s() -
                           reference_line_info.AdcSlBoundary().end_s();
  AINFO << "suitable_distance = " << suitable_distance
        << " actual_distance = " << actual_distance;
  if (suitable_distance < actual_distance) {
    return false;
  } else {
    return true;
  }
}

bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (nullptr == blocking_obstacle) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (nullptr == blocking_obstacle) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // Get blocking obstacle's s.
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

  double remain_distane = CheckBorrowRemainDistance();
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
        CheckIsNeedGotoLeftNeighborLane(reference_line_info);
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

bool PathLaneBorrowDecider::IsSidePassableObstacle(
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

  if (!IsNonmovableObstacle(reference_line_info, *blocking_obstacle)) {
    return false;
  }

  const auto& config = config_.path_lane_borrow_decider_config();
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  if (FLAGS_enable_near_junction_laneborrow) {
    if ((mutable_path_decider->has_truck_near_junction() ||
         mutable_path_decider->is_obs_near_junction() ||
         injector_->adc_in_junction_info_.first) &&
        !mutable_path_decider->is_need_go_left_lane()) {
      return false;
    }
  } else if (injector_->near_junction_keep_straight_) {
    if (IsNearJunctionObstacleCanPass(reference_line_info, *blocking_obstacle,
                                      closest_stop_sign_start_s_)) {
      ++injector_->near_junction_borrow_check_times_;
    } else {
      ADEBUG << "Near Junction block obstacle: " << blocking_obstacle_id
             << " not Can Pass";
      injector_->near_junction_borrow_check_times_ = 0;
    }
    if (injector_->near_junction_borrow_check_times_ <
        config.near_junction_obstacle_check_times()) {
      return false;
    }
  } else {
    injector_->near_junction_borrow_check_times_ = 0;
  }

  return true;
}

void PathLaneBorrowDecider::CheckNearJunctionExitLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  CheckIsNeedGobackReferenceLane(reference_line_info);
  bool is_force_to_exit_laneborrow =
      IsForcedToExitLaneBorrow(reference_line_info);
  if (((mutable_path_decider->is_need_goback_reference_lane() &&
        mutable_path_decider->able_to_use_self_lane_counter() >= 6) ||
       is_force_to_exit_laneborrow) &&
      !IsBorrowNeedRemoteRequest(reference_line_info)) {
    mutable_path_decider->set_is_in_path_lane_borrow_scenario(false);
    mutable_path_decider->set_force_to_exit_laneborrow_count(0);
    mutable_path_decider->clear_decided_side_pass_direction();
    AINFO << "[Junction] switch from lane borrow to SELF BORROW.";
  }
}

bool PathLaneBorrowDecider::IsForcedToExitLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  auto check_time = mutable_path_decider->force_to_exit_laneborrow_count();
  bool exit_borrow_in_solid_line =
      is_obs_in_front.first && is_obs_in_front.second &&
      reference_line_info_->IsLocatedInSolidLine(adc_sl);
  bool exit_borrow_in_reference_lane =
      !is_obs_in_front.second && reference_line_info_->IsAdcCenterInLane();
  if (exit_borrow_in_solid_line || exit_borrow_in_reference_lane) {
    mutable_path_decider->set_force_to_exit_laneborrow_count(++check_time);
  } else {
    mutable_path_decider->set_force_to_exit_laneborrow_count(0);
  }
  return mutable_path_decider->force_to_exit_laneborrow_count() >=
         kForceToExitLaneBorrowCount;
}

void PathLaneBorrowDecider::CheckIsNeedGobackReferenceLane(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  bool has_left_routing_lane = reference_line_info_->HasLeftNeighborRoutingLane(
      "goback", injector_->planning_context());
  mutable_path_decider->set_has_left_neighbor_routing_lane(
      has_left_routing_lane);
  bool safe_to_goback = reference_line_info.GetIsClearToChangeLane();
  if (!has_left_routing_lane) {
    mutable_path_decider->set_is_need_goback_reference_lane(true);
    mutable_path_decider->set_is_need_go_left_lane(false);
    AINFO << "[goback] has no left routing lane, need go back reference lane.";
    return;
  }
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  double to_solid_line_distance =
      std::fmin(reference_line_info_->GetRemainDistanceToSolidLine(),
                reference_line_info_->GetRemainDistanceToJunction());
  auto check_time = mutable_path_decider->goback_near_junction_check_count();
  if (!is_obs_in_front.second && safe_to_goback &&
      (is_obs_in_front.first || to_solid_line_distance > 0.0)) {
    mutable_path_decider->set_goback_near_junction_check_count(++check_time);
  } else {
    AINFO << "[goback] front obs in left/current lane: "
          << is_obs_in_front.first << "," << is_obs_in_front.second
          << ". safe to goback: " << safe_to_goback
          << ". distance: " << to_solid_line_distance;
    mutable_path_decider->set_goback_near_junction_check_count(
        std::max(--check_time, 0));
  }
  int32_t obs_keep_check_threshold =
      GetKeepCheckCountLimit(reference_line_info);
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

void PathLaneBorrowDecider::CheckIsNeedGotoLeftNeighborLane(
    const ReferenceLineInfo& reference_line_info) {
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  bool has_left_lane = reference_line_info_->HasLeftNeighborRoutingLane(
      "goleft", injector_->planning_context());
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_has_left_neighbor_routing_lane(has_left_lane);
  CheckIsSafeToLaneBorrow(reference_line_info);
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
      GetKeepCheckCountLimit(reference_line_info);
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

void PathLaneBorrowDecider::CheckIsNeedPassTruck(
    const ReferenceLineInfo& reference_line_info) {
  const auto& is_obs_in_front = CheckIsObsInFrontOfAdc(reference_line_info);
  bool has_left_lane = false;
  bool has_right_lane = false;
  CheckLaneBorrow(reference_line_info, &has_left_lane, &has_right_lane);
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_has_left_neighbor_routing_lane(has_left_lane);
  CheckIsSafeToLaneBorrow(reference_line_info);
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

bool PathLaneBorrowDecider::IsAdcInLeftNeighborLane(
    const ReferenceLineInfo& reference_line_info) {
  double adc_heading = injector_->vehicle_state()->heading();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  const auto& lane_width = reference_line_info.GetLaneWidthBaseOnAdcCenter();
  double left_width = lane_width.first;
  double heading_threshold = config_.path_lane_borrow_decider_config()
                                 .adc_goto_neighbor_lane_heading_threshold() *
                             kDegreesToRadians;
  if ((adc_heading >= heading_threshold && adc_sl.end_l() > left_width) ||
      (std::fabs(adc_heading) < heading_threshold &&
       adc_center_l > left_width) ||
      (adc_heading <= -heading_threshold && adc_sl.start_l() > left_width)) {
    return true;
  }
  return false;
}

std::pair<bool, bool> PathLaneBorrowDecider::CheckIsObsInFrontOfAdc(
    const ReferenceLineInfo& reference_line_info) {
  if (has_checked_obs_in_front_) {
    return is_obs_in_front_;
  }

  const auto& path_decision = reference_line_info.path_decision();
  const auto& indexed_obstacles = path_decision.obstacles();
  auto obstacle_list = indexed_obstacles.Items();
  std::vector<const Obstacle*> left_lane_obstacle;
  std::vector<const Obstacle*> ref_lane_obstacle;
  const auto& config = config_.path_lane_borrow_decider_config();
  std::pair<bool, bool> is_obs_in_front = std::make_pair(false, false);

  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  hdmap::Id neighbor_lane_id;
  double left_neighbor_lane_width = 0.0;
  if (!reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width)) {
    AINFO
        << "reference lane id: "
        << reference_line_info.LocateLaneInfo(adc_sl.end_s())->lane().id().id()
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
          reference_line_info.GetLaneWidthBaseOnAdcCenter();
      double left_width = lane_width.first;
      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      const auto& veh_param =
          common::VehicleConfigHelper::GetConfig().vehicle_param();
      double half_width = veh_param.width() * 0.5;
      if ((obs_sl.start_l() < left_width + left_neighbor_lane_width -
                                  config.left_neighbor_lane_obs_buffer()) &&
          obs_sl.end_l() >
              left_width + config.left_neighbor_lane_obs_buffer() &&
          obs_sl.end_s() > adc_sl.end_s() &&
          (obs_sl.start_s() - adc_sl.end_s()) < kFrontObsMaxDistance) {
        left_lane_obstacle.emplace_back(obstacle);
      }
      if (obs_sl.start_l() < left_width - config.reference_lane_obs_buffer() &&
          obs_sl.end_l() > -half_width - config.reference_lane_obs_buffer() &&
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

void PathLaneBorrowDecider::CheckVehicleObsInFrontOfAdc(
    const ReferenceLineInfo& reference_line_info,
    std::vector<const Obstacle*>* const left_lane_veh,
    std::vector<const Obstacle*>* const ref_lane_veh) {
  CHECK_NOTNULL(left_lane_veh);
  CHECK_NOTNULL(ref_lane_veh);
  left_lane_veh->clear();
  ref_lane_veh->clear();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  hdmap::Id neighbor_lane_id;
  double left_neighbor_lane_width = 0.0;
  if (!reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width) &&
      !reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl.end_s(),
          &neighbor_lane_id, &left_neighbor_lane_width)) {
    AINFO
        << "reference lane id: "
        << reference_line_info.LocateLaneInfo(adc_sl.end_s())->lane().id().id()
        << " has no left neighbor lane.";
    return;
  }

  auto obstacle_list = reference_line_info.path_decision().obstacles().Items();
  const auto& config = config_.path_lane_borrow_decider_config();
  for (const auto* obstacle : obstacle_list) {
    if (nullptr == obstacle) {
      continue;
    }
    if (obstacle->IsVirtual()) {
      continue;
    }

    if ((obstacle->IsStatic() ||
         obstacle->speed() < FLAGS_static_obstacle_speed_threshold) &&
        PerceptionObstacle::VEHICLE == obstacle->Perception().type()) {
      const auto& lane_width =
          reference_line_info.GetLaneWidthBaseOnAdcCenter();
      double left_width = lane_width.first;
      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      if ((obs_sl.start_l() < left_width + left_neighbor_lane_width -
                                  config.left_lane_veh_buffer()) &&
          obs_sl.end_l() > left_width + config.left_lane_veh_buffer() &&
          obs_sl.end_s() > adc_sl.end_s() &&
          (obs_sl.start_s() - adc_sl.end_s()) < kFrontObsMaxDistance) {
        left_lane_veh->emplace_back(obstacle);
      }
      if (obs_sl.start_l() < left_width - config.reference_lane_veh_buffer() &&
          obs_sl.end_l() > -config.reference_lane_veh_buffer() &&
          obs_sl.end_s() > adc_sl.end_s() &&
          (obs_sl.start_s() - adc_sl.end_s()) < kFrontObsMaxDistance) {
        ref_lane_veh->emplace_back(obstacle);
      }
    }
  }
}

void PathLaneBorrowDecider::CheckAdcDistance(
    const ReferenceLineInfo& reference_line_info) {
  has_checked_obs_in_front_ = false;
  double to_solid_line_distance =
      std::fmin(reference_line_info_->GetRemainDistanceToSolidLine(),
                reference_line_info_->GetRemainDistanceToJunction());
  if (to_solid_line_distance >
      reference_line_info.reference_line().Length() + kEpsilon) {
    to_solid_line_distance = std::numeric_limits<double>::max();
  }
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  const auto& config = config_.path_lane_borrow_decider_config();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  injector_->is_auxiliary_road_ =
      !reference_line_info_->HasNeighborLane(adc_sl.end_s());
  AINFO << "is adc in auxiliary_road: " << injector_->is_auxiliary_road_;
  bool is_adc_near_junction =
      to_solid_line_distance < config.adc_to_solid_line_distance_threshold() &&
      !injector_->is_auxiliary_road_ && !injector_->is_in_play_street;
  mutable_path_decider->set_is_adc_near_junction(is_adc_near_junction);

  if (to_solid_line_distance < config.obs_to_solid_line_distance_threshold() ||
      (is_adc_near_junction &&
       util::IsLaneBorrow(injector_->planning_context()))) {
    mutable_path_decider->set_has_truck_near_junction(false);
    mutable_path_decider->set_truck_check_count(0);
  }

  injector_->adc_in_junction_info_.second =
      injector_->adc_in_junction_info_.first;
  injector_->adc_in_junction_info_.first =
      reference_line_info.AdcIsOnOverlapJunction() &&
      !reference_line_info_->IsInEitherSolidLine(adc_sl);
  if (injector_->adc_in_junction_info_.first) {
    mutable_path_decider->set_is_obs_near_junction(false);
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
}

void PathLaneBorrowDecider::CheckObsNearJunction(
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

void PathLaneBorrowDecider::CheckTrafficStatus(
    const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_path_decider();
  mutable_path_decider->set_is_in_crowd_traffic(false);
  const auto& config = config_.path_lane_borrow_decider_config();
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

  std::vector<const Obstacle*> left_lane_veh;
  std::vector<const Obstacle*> ref_lane_veh;
  CheckVehicleObsInFrontOfAdc(reference_line_info, &left_lane_veh,
                              &ref_lane_veh);
  const auto single_lane_car_number_limit =
      config.crowd_traffic_car_number_limit();
  if (left_lane_veh.size() >=
          static_cast<size_t>(single_lane_car_number_limit) &&
      ref_lane_veh.size() >=
          static_cast<size_t>(single_lane_car_number_limit)) {
    mutable_path_decider->set_is_in_crowd_traffic(true);
    mutable_path_decider->set_crowd_traffic_check_count(count_limit);
    return;
  }
  std::vector<const Obstacle*> veh_obs_ahead;
  veh_obs_ahead.insert(veh_obs_ahead.end(), left_lane_veh.begin(),
                       left_lane_veh.end());
  veh_obs_ahead.insert(veh_obs_ahead.end(), ref_lane_veh.begin(),
                       ref_lane_veh.end());
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
  bool is_crowd_traffic = false;
  for (size_t i = 0; i < veh_obs_ahead.size() - 1; ++i) {
    auto car1_sl = veh_obs_ahead[i]->PerceptionSLBoundary();
    car1_sl.set_start_s(car1_sl.start_s() - 0.5 * car_gap);
    car1_sl.set_end_s(car1_sl.end_s() + 0.5 * car_gap);
    auto car2_sl = veh_obs_ahead[i + 1]->PerceptionSLBoundary();
    car2_sl.set_start_s(car2_sl.start_s() - 0.5 * car_gap);
    car2_sl.set_end_s(car2_sl.end_s() + 0.5 * car_gap);

    if (util::IsLongitudinalOverlap(car1_sl, car2_sl) &&
        !util::IsLateralOverlap(car1_sl, car2_sl) &&
        ((veh_obs_ahead[i + 1]->PerceptionSLBoundary().start_s() -
          reference_line_info.AdcSlBoundary().end_s()) <
         config.crowd_traffic_overlap_car_lookforward_distance())) {
      AINFO << "[CrowdTraffic] car(" << veh_obs_ahead[i]->Id()
            << ") parallel wiht car(" << veh_obs_ahead[i + 1]->Id() << ").";
      is_crowd_traffic = true;
      auto crowd_count = mutable_path_decider->crowd_traffic_check_count();
      mutable_path_decider->set_crowd_traffic_check_count(
          std::min(++crowd_count, count_limit));
      break;
    }
  }
  if (!is_crowd_traffic) {
    auto crowd_count = mutable_path_decider->crowd_traffic_check_count();
    mutable_path_decider->set_crowd_traffic_check_count(
        std::max(--crowd_count, -count_limit));
  }
  if (mutable_path_decider->crowd_traffic_check_count() > 0) {
    mutable_path_decider->set_is_in_crowd_traffic(true);
    AINFO << "[CrowdTraffic] ADC is in crowd traffic.";
  }
}

void PathLaneBorrowDecider::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* const left_neighbor_lane_borrowable,
    bool* const right_neighbor_lane_borrowable) {
  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  // when in overtaking, not lane borrow
  if (injector_->planning_context()
          ->planning_status()
          .overtake()
          .is_in_overtake_status()) {
    *left_neighbor_lane_borrowable = false;
    *right_neighbor_lane_borrowable = false;
    return;
  }

  // Keep lane borrow decision stable same as last frame
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider_status.is_in_path_lane_borrow_scenario() &&
      (!reference_line_info.IsAdcPostureStraight() ||
       path_decider_status.will_pass_merge_lane_area())) {
    *left_neighbor_lane_borrowable = false;
    *right_neighbor_lane_borrowable = false;
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (PathDeciderStatus::LEFT_BORROW == lane_borrow_direction) {
        *left_neighbor_lane_borrowable = true;
      } else if (PathDeciderStatus::RIGHT_BORROW == lane_borrow_direction) {
        *right_neighbor_lane_borrowable = true;
      }
    }
    if (injector_->adc_in_junction_info_.first) {
      AINFO << "no need check solid line when adc is in junction";
      return;
    }
  }
  double lookforward_distance = GetForwardCheckDistance(reference_line_info);
  double check_s = reference_line_info.AdcSlBoundary().end_s();

  while (check_s < lookforward_distance) {
    if (!(*left_neighbor_lane_borrowable) &&
        !(*right_neighbor_lane_borrowable)) {
      AINFO << "Neither left nor right boundry can borrow";
      return;
    }
    auto ref_point =
        reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }

    CheckLaneBoundaryTypeIsBorrowable(ref_point, check_s,
                                      left_neighbor_lane_borrowable,
                                      right_neighbor_lane_borrowable);
    auto check_lane_info = reference_line_info.LocateLaneInfo(check_s);
    if (check_lane_info && check_lane_info->lane().is_merge()) {
      AINFO << "curr check_s: " << check_s
            << " is merge lane, always support borrow!";
      break;
    }
    check_s += 2.0;
  }
}

double PathLaneBorrowDecider::GetForwardCheckDistance(
    const ReferenceLineInfo& reference_line_info) {
  double blocking_obstacle_s = 0.0;
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
  } else {
    const Obstacle* blocking_obstacle =
        reference_line_info.path_decision().obstacles().Find(
            blocking_obstacle_id);
    if (nullptr == blocking_obstacle) {
      ADEBUG << "Blocking obstacle is no longer there.";
    } else {
      blocking_obstacle_s = blocking_obstacle->PerceptionSLBoundary().end_s();
    }
  }
  bool is_in_borrow = path_decider_status.is_in_path_lane_borrow_scenario();
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

void PathLaneBorrowDecider::CheckLaneBoundaryTypeIsBorrowable(
    const ReferencePoint& ref_point, const double check_s,
    bool* const left_neighbor_lane_borrowable,
    bool* const right_neighbor_lane_borrowable) {
  const auto& waypoint = ref_point.lane_waypoints().front();
  bool is_in_borrow = injector_->planning_context()
                          ->planning_status()
                          .path_decider()
                          .is_in_path_lane_borrow_scenario();

  if (*left_neighbor_lane_borrowable) {
    const auto lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
    if (LaneBoundTypeIsUnPassable(lane_boundary_type)) {
      *left_neighbor_lane_borrowable = false;
    }
    ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
           << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    if (*left_neighbor_lane_borrowable && !is_in_borrow &&
        waypoint.lane->lane().left_neighbor_forward_lane_id().empty() &&
        (waypoint.lane->lane().left_neighbor_reverse_lane_id().empty() ||
         not_borrow_reverse_)) {
      *left_neighbor_lane_borrowable = false;
      AINFO << "left FORWARD/REVERSE lane is empty, not borrowable";
    }
  }
  if (*right_neighbor_lane_borrowable) {
    const auto lane_boundary_type = hdmap::RightBoundaryType(waypoint);
    if (LaneBoundTypeIsUnPassable(lane_boundary_type) ||
        waypoint.lane->lane().right_boundary().gap()) {
      *right_neighbor_lane_borrowable = false;
    }
    ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
           << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    if (*right_neighbor_lane_borrowable && !is_in_borrow &&
        waypoint.lane->lane().right_neighbor_forward_lane_id().empty() &&
        (waypoint.lane->lane().right_neighbor_reverse_lane_id().empty() ||
         not_borrow_reverse_)) {
      *right_neighbor_lane_borrowable = false;
      AINFO << "right FORWARD/REVERSE lane is empty, not borrowable";
    }
  }
}

bool PathLaneBorrowDecider::LaneBoundTypeIsUnPassable(
    const hdmap::LaneBoundaryType::Type& lane_boundary_type) {
  if (hdmap::LaneBoundaryType::CURB == lane_boundary_type ||
      hdmap::LaneBoundaryType::UNKNOWN == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_YELLOW == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_WHITE == lane_boundary_type ||
      hdmap::LaneBoundaryType::DOUBLE_YELLOW == lane_boundary_type) {
    return true;
  }
  return false;
}

bool PathLaneBorrowDecider::IsNearObstacle(
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
      VehicleConfigHelper::GetConfig().vehicle_param(), adc_l_max);
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  AINFO << "stop_distance = " << stop_distance;
  if (blocking_obstacle_s - adc_end_s > 0.0 &&
      blocking_obstacle_s - adc_end_s < stop_distance &&
      blocking_obstacle_l > adc_end_l) {
    return true;
  }
  return false;
}

bool PathLaneBorrowDecider::IsBorrowNeedRemoteRequest(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (!path_decider_status.is_in_path_lane_borrow_scenario()) {
    return false;
  }
  if (reference_line_info.IsAdcLocatedInLane()) {
    return false;
  }
  const auto& config = config_.path_lane_borrow_decider_config();
  if (path_decider_status.lane_borrow_path_fail_times() >
      config.borrow_path_fail_remote_report_threshold()) {
    reference_line_info_->SetLaneBorrowFailRequest(true);
    return true;
  }
  return false;
}

bool PathLaneBorrowDecider::IsStopNearIntersectionLongTime(
    const ReferenceLineInfo& reference_line_info) {
  const auto& adc_v = reference_line_info.vehicle_state().linear_velocity();
  const auto& config = config_.path_lane_borrow_decider_config();
  if (std::fabs(adc_v) > kEpsilon) {
    stop_start_time_ = Clock::NowInSeconds();
    return false;
  }
  if (reference_line_info.IsAdcLocatedInLane()) {
    return false;
  }
  if (Clock::NowInSeconds() - stop_start_time_ <
      config.stop_near_intersection_time_threshold()) {
    return false;
  }
  auto remain_dis = reference_line_info_->GetRemainDistanceForBack(
      injector_->planning_context());
  if (remain_dis > config.stop_near_intersection_distance_threshold()) {
    return false;
  }
  reference_line_info_->SetLaneBorrowFailRequest(true);
  ADEBUG << "adc is stop near intersection for too long time";
  return true;
}

void PathLaneBorrowDecider::CheckIsSafeToLaneBorrow(
    const ReferenceLineInfo& reference_line_info) {
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  double adc_start_s = adc_sl_boundary.start_s();
  double adc_end_s = adc_sl_boundary.end_s();
  double adc_v = std::abs(injector_->vehicle_state()->linear_velocity());

  const auto& lane_width = reference_line_info.GetLaneWidthBaseOnAdcCenter();
  double left_width = lane_width.first;
  double right_width = lane_width.second;
  double neighbor_lane_width = left_width + right_width;

  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  const auto& config = config_.path_lane_borrow_decider_config();

  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }

    if (obstacle->IsVirtual()) {
      ADEBUG << "skip virtual obstacle";
      continue;
    }

    if (obstacle->IsStatic()) {
      ADEBUG << "skip static obstacle";
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.start_l() > (left_width + neighbor_lane_width) ||
        obstacle_sl.end_l() < left_width) {
      ADEBUG << "skip obstacle not in the left neighbor lane";
      continue;
    }

    // judge the directiion of the obstacle.
    double obstacle_moving_direction = 0.0;
    if (!obstacle->HasTrajectory()) {
      obstacle_moving_direction = obstacle->Perception().theta();
    } else {
      obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
    }
    double vehicle_moving_direction = injector_->vehicle_state()->heading();
    double heading_difference = std::abs(common::math::NormalizeAngle(
        obstacle_moving_direction - vehicle_moving_direction));
    bool is_same_direction = heading_difference < kSameDirectionThr;

    double forward_safe_distance = 0.0;
    double backward_safe_distance = 0.0;
    // set a safe distance based on direction
    if (is_same_direction) {
      forward_safe_distance =
          std::fmax(config.forward_safe_distance_same_direction(),
                    ((adc_v - obstacle->speed()) *
                     config.safe_check_time_same_direction()));
      backward_safe_distance =
          std::fmax(config.backward_safe_distance_same_direction(),
                    ((obstacle->speed() - adc_v) *
                     config.safe_check_time_same_direction()));
    } else {
      forward_safe_distance =
          std::fmax(config.forward_safe_distance_opposite_direction(),
                    ((adc_v + obstacle->speed()) *
                     config.safe_check_time_opposite_direction()));
      backward_safe_distance =
          config.backward_safe_distance_opposite_direction();
    }

    if ((obstacle_sl.end_s() > adc_start_s - backward_safe_distance) &&
        (obstacle_sl.start_s() < adc_end_s + forward_safe_distance)) {
      mutable_laneborrow->set_is_left_borrow_safe(false);
      AINFO << "Unsafe to left borrow, obs_id : " << obstacle->Id();
      return;
    }
  }
  mutable_laneborrow->set_is_left_borrow_safe(true);
}

bool PathLaneBorrowDecider::IsBlockingObstacleTruck(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
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

int32_t PathLaneBorrowDecider::GetKeepCheckCountLimit(
    const ReferenceLineInfo& reference_line_info) {
  const auto& config = config_.path_lane_borrow_decider_config();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  bool adc_in_solid_line = reference_line_info_->IsInEitherSolidLine(adc_sl);
  int32_t obs_keep_check_threshold =
      injector_->adc_in_junction_info_.first
          ? config.in_junction_obs_keep_check_time_threshold()
          : config.near_junction_obs_keep_check_time_threshold();
  if (adc_in_solid_line) {
    double distance_to_signal =
        reference_line_info_->GetRemainDistanceToSignal();
    obs_keep_check_threshold =
        distance_to_signal < config.distance_to_signal_limit_level_1()
            ? config.in_solid_line_check_time_limit_level_1()
            : (distance_to_signal < config.distance_to_signal_limit_level_2()
                   ? config.in_solid_line_check_time_limit_level_2()
                   : config.in_solid_line_check_time_limit_level_3());
  }
  obs_keep_check_threshold =
      (IsBlockingObstacleTruck(reference_line_info) &&
       !injector_->adc_in_junction_info_.first)
          ? obs_keep_check_threshold + config.extra_waiting_time_for_truck()
          : obs_keep_check_threshold;
  bool is_red_light = perception::TrafficLight::RED == frame_->signal_color_;
  obs_keep_check_threshold =
      (is_red_light && adc_in_solid_line)
          ? obs_keep_check_threshold + config.extra_waiting_time_for_red_light()
          : obs_keep_check_threshold;
  return obs_keep_check_threshold;
}

}  // namespace planning
}  // namespace century
