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

#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_action.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::cyber::Clock;
using century::perception::PerceptionObstacle;
using century::planning::ReferenceLineInfo;

namespace {
constexpr double kEpsilon = 1e-2;
constexpr double kSafeTimeOnSameDirection = 3.0;
constexpr double kChangeLaneTime = 6.0;
constexpr double kCheckSolidLineStep = 5.0;
constexpr double kCheckMergeLineStep = 5.0;
constexpr double kOvertakeCheckDistanceBuffer = 1.5;
constexpr double kOvertakeCheckVelocityBuffer = 1.4;
constexpr double kMaxLaneChangeRemainDistance = 1000.0;
}  // namespace

FsmAction::FsmAction(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {}

void FsmAction::Default2Default(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  InitParamInfo(frame);
  if (util::IsLaneChange(injector_->planning_context())) {
    return;
  }
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider_status.is_in_path_lane_borrow_scenario() ||
      !reference_line_info->IsAdcLocatedInLane()) {
    return;
  }

  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  if (!IsRoadSupportOvertake(reference_line_info)) {
    mutable_overtake->set_slow_obs_keep_check_times(0);
    return;
  }

  double adc_to_obstacle_min_distance = std::numeric_limits<double>::max();
  const Obstacle* min_dis_obs = nullptr;
  bool is_vehicle_obs = false;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (!IsObstacleNeedToBeOvertaken(frame, reference_line_info, obstacle)) {
      continue;
    }

    double actual_distance = obstacle->PerceptionSLBoundary().start_s() -
                             reference_line_info->AdcSlBoundary().end_s();

    if (actual_distance < adc_to_obstacle_min_distance) {
      adc_to_obstacle_min_distance = actual_distance;
      min_dis_obs = obstacle;
      is_vehicle_obs = (perception::PerceptionObstacle::VEHICLE ==
                        obstacle->Perception().type());
    }
  }

  if (nullptr == min_dis_obs) {
    mutable_overtake->set_overtake_obstacle_id("");
    mutable_overtake->set_overtake_obstacle_perception_id(0);
    mutable_overtake->set_slow_obs_keep_check_times(0);
    mutable_overtake->set_is_vehicle_type(false);
    return;
  }

  if (std::strcmp(min_dis_obs->Id().c_str(),
                  mutable_overtake->overtake_obstacle_id().c_str()) != 0) {
    mutable_overtake->set_slow_obs_keep_check_times(1);
    mutable_overtake->set_overtake_obstacle_id(min_dis_obs->Id());
    mutable_overtake->set_overtake_obstacle_perception_id(
        min_dis_obs->PerceptionId());
    mutable_overtake->set_is_vehicle_type(is_vehicle_obs);
    return;
  }

  auto check_times = mutable_overtake->slow_obs_keep_check_times();
  mutable_overtake->set_slow_obs_keep_check_times(++check_times);
  return;
}

void FsmAction::Default2Prepare(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_safe_check_times(0);

  std::string overtake_obs_id = mutable_overtake->overtake_obstacle_id();
  const auto* overtake_obs =
      reference_line_info->path_decision()->Find(overtake_obs_id);
  if (nullptr != overtake_obs) {
    const auto& obstacle_sl = overtake_obs->PerceptionSLBoundary();
    const auto& adc_sl = reference_line_info->AdcSlBoundary();
    AINFO << "obstacle id[" << overtake_obs->Id() << "] type["
          << perception::PerceptionObstacle_Type_Name(
                 overtake_obs->Perception().type())
          << "] obstacle speed[" << overtake_obs->speed() << "] adc speed["
          << reference_line_info->vehicle_state().linear_velocity()
          << "] obstacle sl[" << obstacle_sl.ShortDebugString() << "] adc sl["
          << adc_sl.ShortDebugString() << "]need to overtake.";
  }

  bool is_left_overtake = true;
  if (IsSafeToOvertake(reference_line_info, is_left_overtake,
                       ReferenceLineInfo::CheckLevel::PRE_CHECK)) {
    auto safe_check_times = mutable_overtake->safe_check_times();
    mutable_overtake->set_safe_check_times(++safe_check_times);
  }
  return;
}

void FsmAction::Prepare2Default(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_is_in_overtake_status(false);
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Prepare2Turn(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_turn_timestamp(Clock::NowInSeconds());
  mutable_overtake->set_start_stop_time(Clock::NowInSeconds());
  mutable_overtake->set_slow_obs_keep_check_times(0);
  mutable_overtake->set_safe_check_times(0);
  mutable_overtake->set_is_in_overtake_status(true);
  mutable_overtake->set_urgency_lane_change(false);
  mutable_overtake->set_accelerated_mode(true);
  mutable_overtake->set_stop_change_lane(false);
  frame->UpdateLaneChangeLaneId("");
  find_solid_start_ = false;
  return;
}

void FsmAction::Prepare2Prepare(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  if (mutable_overtake->history_overtake_signal() &&
      reference_line_info->IsChangeLanePath()) {
    return;
  }
  origin_lane_id_ = reference_line_info->Lanes().Id();
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider_status.is_in_path_lane_borrow_scenario()) {
    mutable_overtake->set_slow_obs_keep_check_times(0);
    return;
  }
  if (!IsRoadSupportOvertake(reference_line_info)) {
    mutable_overtake->set_slow_obs_keep_check_times(0);
    return;
  }
  std::string overtake_obs_id = mutable_overtake->overtake_obstacle_id();
  const auto* overtake_obs =
      reference_line_info->path_decision()->Find(overtake_obs_id);
  if (!overtake_obs) {
    std::string overtake_obs_perception_id =
        std::to_string(mutable_overtake->overtake_obstacle_perception_id());
    overtake_obs =
        reference_line_info->path_decision()->Find(overtake_obs_perception_id);
  }
  if (!IsObstacleNeedToBeOvertaken(frame, reference_line_info, overtake_obs)) {
    mutable_overtake->set_slow_obs_keep_check_times(0);
    return;
  }

  bool is_left_overtake = true;
  if (IsSafeToOvertake(reference_line_info, is_left_overtake,
                       ReferenceLineInfo::CheckLevel::PRE_CHECK)) {
    auto safe_check_times = mutable_overtake->safe_check_times();
    mutable_overtake->set_safe_check_times(++safe_check_times);
  } else {
    mutable_overtake->set_safe_check_times(0);
  }

  if (mutable_overtake->safe_check_times() > FLAGS_lc_safe_check_times) {
    mutable_overtake->set_history_overtake_signal(true);
    frame->UpdateOvertakeStatus(OvertakeStatus::LEFT);
    mutable_overtake->set_is_in_overtake_status(true);
  } else {
    mutable_overtake->set_history_overtake_signal(false);
    frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  }
  return;
}

void FsmAction::Turn2Return(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_turn_timestamp(Clock::NowInSeconds());
  mutable_overtake->set_stop_change_lane(false);
  mutable_overtake->set_accelerated_mode(false);
  ResetOvertakeInfo(frame);
  frame->UpdateLaneChangeLaneId(origin_lane_id_);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  lane_change_remain_dis_ = kMaxLaneChangeRemainDistance;
  return;
}

void FsmAction::Turn2Overtake(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::OVERTAKE);
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_turn_timestamp(Clock::NowInSeconds());
  mutable_overtake->set_urgency_lane_change(false);
  mutable_overtake->set_accelerated_mode(true);
  find_solid_start_ = false;
  need_give_up_overtake_ = false;
  return;
}

void FsmAction::Turn2Turn(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  bool is_origin_lane = reference_line_info->IsOriginalRefLine(true);
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  if (!is_origin_lane && reference_line_info->IsAdcCenterInLane()) {
    mutable_overtake->set_safe_check_times(0);
    mutable_overtake->set_slow_obs_keep_check_times(0);
    return;
  }
  if (is_origin_lane && reference_line_info->IsChangeLanePath()) {
    mutable_overtake->set_safe_check_times(-FLAGS_lc_safe_check_times - 1);
    return;
  }
  if (!CheckDistanceToOverlap(reference_line_info) ||
      reference_line_info->LaneChangePathWillPassOverlap() ||
      (reference_line_info->IsChangeLanePath() &&
       (!CheckSolidLine(reference_line_info, CheckDirection::CHECK_RIGHT) ||
        !CheckRoutingEnd(reference_line_info, CheckDirection::CHECK_SELF)))) {
    mutable_overtake->set_safe_check_times(-FLAGS_lc_safe_check_times - 1);
    return;
  }
  if ((reference_line_info->IsChangeLanePath() &&
       !IsSafeToOvertake(reference_line_info, true,
                         ReferenceLineInfo::CheckLevel::CHANGE_CHECK)) ||
      is_origin_lane) {
    mutable_overtake->set_safe_check_times(-FLAGS_lc_safe_check_times - 1);
  } else {
    mutable_overtake->set_safe_check_times(0);
  }

  std::string overtake_obs_id = mutable_overtake->overtake_obstacle_id();
  auto* overtake_obs =
      reference_line_info->path_decision()->Find(overtake_obs_id);
  if (!overtake_obs) {
    std::string overtake_obs_perception_id =
        std::to_string(mutable_overtake->overtake_obstacle_perception_id());
    overtake_obs =
        reference_line_info->path_decision()->Find(overtake_obs_perception_id);
  }
  if (!IsObstacleNeedToBeOvertaken(frame, reference_line_info, overtake_obs)) {
    auto slow_obs_check_time = mutable_overtake->slow_obs_keep_check_times();
    mutable_overtake->set_slow_obs_keep_check_times(--slow_obs_check_time);
  } else {
    mutable_overtake->set_slow_obs_keep_check_times(0);
  }
  return;
}

void FsmAction::Overtake2Finish(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Overtake2Fail(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Overtake2Overtake(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info->IsChangeLanePath()) {
    return;
  }

  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  double turn_timestamp = mutable_overtake->turn_timestamp();
  double delta_t = Clock::NowInSeconds() - turn_timestamp;
  const auto& config = config_.lane_overtake_decider_config();
  if (delta_t < config.min_overtake_keep_time_for_stable()) {
    return;
  }

  lane_change_remain_dis_ = kMaxLaneChangeRemainDistance;
  if (!CheckDistanceToDestination(reference_line_info) ||
      !CheckRoutingEnd(reference_line_info, CheckDirection::CHECK_SELF)) {
    mutable_overtake->set_history_overtake_signal(true);
    frame->UpdateOvertakeStatus(OvertakeStatus::RIGHT);
    mutable_overtake->set_urgency_lane_change(true);
    return;
  }
  if (AdcWillPassMergeLane(reference_line_info)) {
    mutable_overtake->set_safe_check_times(0);
    return;
  }
  if (!FLAGS_enable_overtake_cross_junction) {
    if (!CheckSolidLine(reference_line_info, CheckDirection::CHECK_RIGHT) ||
        !CheckDistanceToOverlap(reference_line_info)) {
      mutable_overtake->set_history_overtake_signal(true);
      frame->UpdateOvertakeStatus(OvertakeStatus::RIGHT);
      mutable_overtake->set_urgency_lane_change(true);
      return;
    }
  } else if (reference_line_info->LaneChangePathWillPassSolidLine()) {
    mutable_overtake->set_safe_check_times(0);
    return;
  }

  if (reference_line_info->LaneChangePathWillPassOverlap()) {
    mutable_overtake->set_safe_check_times(0);
    return;
  }

  std::string overtake_obs_id = mutable_overtake->overtake_obstacle_id();
  auto* overtake_obs =
      reference_line_info->path_decision()->Find(overtake_obs_id);
  if (!overtake_obs) {
    std::string overtake_obs_perception_id =
        std::to_string(mutable_overtake->overtake_obstacle_perception_id());
    overtake_obs =
        reference_line_info->path_decision()->Find(overtake_obs_perception_id);
  }

  if (!HasOvertakePassTheObstacle(reference_line_info, overtake_obs)) {
    mutable_overtake->set_safe_check_times(0);
    return;
  }

  if (IsSafeToOvertake(reference_line_info, false,
                       ReferenceLineInfo::CheckLevel::PRE_CHECK)) {
    auto check_times = mutable_overtake->safe_check_times();
    mutable_overtake->set_safe_check_times(++check_times);
  } else {
    mutable_overtake->set_safe_check_times(0);
  }

  if (mutable_overtake->safe_check_times() > FLAGS_lc_safe_check_times) {
    mutable_overtake->set_history_overtake_signal(true);
    frame->UpdateOvertakeStatus(OvertakeStatus::RIGHT);
  }
  return;
}

void FsmAction::Overtake2Return(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_turn_timestamp(Clock::NowInSeconds());
  mutable_overtake->set_safe_check_times(0);
  find_solid_start_ = false;
  return;
}

void FsmAction::Return2Finish(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  ResetOvertakeInfo(frame);
  origin_lane_id_ = "";
  frame->UpdateLaneChangeLaneId(origin_lane_id_);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Return2Return(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  if (NeedCancelLaneChangeInMergeLane(frame, reference_line_info)) {
    AINFO << "[Return2Return] will pass mergelane, cancel lanechange";
    return;
  }
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  bool origin_check_stop_ok = true;

  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (reference_line_info.IsChangeLanePath()) {
      continue;
    }
    origin_check_stop_ok =
        CheckRoutingEnd(&reference_line_info, CheckDirection::CHECK_SELF);
  }

  bool need_stop = false;
  double stop_s = 0.0;
  const std::string stop_wall_id = "lane change wall";
  if (!reference_line_info->IsChangeLanePath()) {
    if (!origin_check_stop_ok &&
        !util::CompareTwoStringIsEqual(origin_lane_id_,
                                       reference_line_info->Lanes().Id())) {
      double check_s = adc_end_s + routing_end_remain_dis_;
      double limit_stop_s = check_s - FLAGS_limit_stop_wall_for_lane_change;
      stop_s = (adc_end_s > limit_stop_s) ? adc_end_s : limit_stop_s;
      AINFO << "origin lane BuildStopDecision, lane change wall, stop_s: "
            << stop_s;
      need_stop = true;
    }
  } else if (!reference_line_info->GetIsClearToChangeLane()) {
    CheckIsNeedBuildLaneChangeStop(reference_line_info, &need_stop, &stop_s);
  }
  if (need_stop) {
    const auto& planning_start_point = frame->PlanningStartPoint();
    double stop_dec_dis =
        std::fabs(planning_start_point.v() * planning_start_point.v() /
                  FLAGS_soft_deceleration_for_lane_change_stop * 0.5);
    double soft_stop_wall_s =
        adc_end_s +
        std::fmax(stop_dec_dis, FLAGS_min_stop_distance_for_lane_change_wall);
    AINFO << "adc_end_s: " << adc_end_s
          << ", soft_stop_wall_s: " << soft_stop_wall_s
          << ", stop_s(before): " << stop_s;
    stop_s = std::fmax(stop_s, soft_stop_wall_s);
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, stop_s, 0.0,
                            StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY,
                            wait_for_obstacles, "LaneOvertakeDecider", frame,
                            reference_line_info);
  }
  return;
}

void FsmAction::Return2Hold(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  return;
}

void FsmAction::Hold2Fail(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  return;
}

void FsmAction::Hold2Return(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  return;
}

void FsmAction::Hold2Hold(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  return;
}

void FsmAction::Finish2Default(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_is_in_overtake_status(false);
  mutable_overtake->set_accelerated_mode(false);
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Finish2Finish(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  return;
}

void FsmAction::Fail2Default(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_is_in_overtake_status(false);
  ResetOvertakeInfo(frame);
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  return;
}

void FsmAction::Fail2Fail(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  return;
}

bool FsmAction::IsRoadSupportOvertake(
    ReferenceLineInfo* const reference_line_info) {
  lane_change_remain_dis_ = kMaxLaneChangeRemainDistance;
  if (hdmap::Lane::PLAY_STREET == reference_line_info->GetLaneType()) {
    return false;
  }
  if (AdcWillPassMergeLane(reference_line_info)) {
    return false;
  }
  if (!reference_line_info->AdcIsOnRouteLane(injector_->planning_context())) {
    return false;
  }
  if (reference_line_info->LaneChangePathWillPassOverlap()) {
    return false;
  }
  if (!CheckDistanceToOverlap(reference_line_info)) {
    return false;
  }
  if (!CheckDistanceToDestination(reference_line_info)) {
    return false;
  }
  if (!CheckSolidLine(reference_line_info, CheckDirection::CHECK_LEFT)) {
    return false;
  }
  if (!CheckRoutingEnd(reference_line_info, CheckDirection::CHECK_LEFT)) {
    return false;
  }
  return true;
}

bool FsmAction::IsObstacleNeedToBeOvertaken(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const Obstacle* obstacle) const {
  if (!obstacle || obstacle->IsVirtual()) {
    AINFO << "obstacle is nullptr or virtual, not overtake";
    return false;
  }
  const auto& config = config_.lane_overtake_decider_config();
  bool is_vehicle_obs = (perception::PerceptionObstacle::VEHICLE ==
                         obstacle->Perception().type());
  if (!config.enable_overtake_vehicle() && is_vehicle_obs) {
    AINFO << "not overtake vehicle type obs";
    return false;
  }

  // 1. check static obstacle and adc speed status
  if (!util::IsLaneChange(injector_->planning_context()) &&
      obstacle->speed() < FLAGS_static_obstacle_speed_threshold &&
      obstacle->IsStatic()) {
    if (!config.enable_overtake_static_obstacle()) {
      return false;
    }
    const auto& adc_speed_status = injector_->GetAdcSpeedStatus();
    if (AdcSpeedStatus::SPEED_LOWER == adc_speed_status ||
        AdcSpeedStatus::SPEED_NONE == adc_speed_status) {
      return false;
    }
  }

  // 2. check obstacle is blocking lane
  if (!ObstacleIsBlocking(frame, obstacle)) {
    AINFO << "obstacle: " << obstacle->Id() << " no block, no need overtake";
    return false;
  }

  // 3. check obstacle's distance is too far
  if (ObstacleIsFarFromAdc(reference_line_info, obstacle)) {
    AINFO << "obstacle: " << obstacle->Id() << " is far away, no need overtake";
    return false;
  }

  // 4. check obstacle's speed is too fast
  if (ObstacleIsTooFast(reference_line_info, obstacle)) {
    AINFO << "obstacle: " << obstacle->Id() << " is fast, no need overtake";
    return false;
  }

  double remain_dis =
      std::fmin(lane_change_remain_dis_, routing_end_remain_dis_);
  if (remain_dis <
      GetMinOvertakeNeededDistance(reference_line_info, obstacle)) {
    AINFO << "remain_dis: " << remain_dis
          << " is less than min_overtake_dis, not overtake";
    return false;
  }

  if (!CheckIsForwoardMovingObstacle(reference_line_info, obstacle)) {
    AINFO << "obstacle: " << obstacle->Id()
          << " is not forward moving, no need overtake";
    return false;
  }
  return true;
}

bool FsmAction::ObstacleIsFarFromAdc(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle* obstacle) const {
  const auto& config = config_.lane_overtake_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
  double distance = obstacle_sl.start_s() - adc_sl.end_s();
  double dis_lower = config.min_distance_check_threshold();
  double dis_upper = config.max_distance_check_threshold();
  if (util::IsLaneChange(injector_->planning_context())) {
    dis_lower = config.min_safe_follow_distance();
    dis_upper += kOvertakeCheckDistanceBuffer;
  }
  if (distance < dis_lower || distance > dis_upper) {
    return true;
  }

  bool is_vehicle_obs = (perception::PerceptionObstacle::VEHICLE ==
                         obstacle->Perception().type());
  double ratio =
      (FLAGS_enable_overtake_speed_up) ? FLAGS_overtake_speed_up_ratio : 1.0;
  double adc_max_v = std::fmin(FLAGS_planning_upper_speed_limit * ratio,
                               FLAGS_overtake_upper_speed_limit);
  double diff_v = adc_max_v - obstacle->speed();
  double pre_change_ttc = is_vehicle_obs ? config.pre_change_ttc_for_vehicle()
                                         : config.pre_change_ttc();
  if (distance > diff_v * pre_change_ttc &&
      distance > adc_max_v * config.pre_change_hwt()) {
    AINFO << "obstacle: " << obstacle->Id() << " is not in risk area"
          << ", distance: " << distance << ", TTC: " << diff_v * pre_change_ttc
          << ", HWT: " << adc_max_v * config.pre_change_hwt();
    return true;
  }
  return false;
}

bool FsmAction::ObstacleIsTooFast(ReferenceLineInfo* const reference_line_info,
                                  const Obstacle* obstacle) const {
  const auto& config = config_.lane_overtake_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_s = 0.5 * (adc_sl.start_s() + adc_sl.end_s());
  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(adc_center_s);
  double speed_buffer = util::IsLaneChange(injector_->planning_context())
                            ? kOvertakeCheckVelocityBuffer
                            : 0.0;
  double adc_v =
      std::fabs(reference_line_info->vehicle_state().linear_velocity());
  bool is_vehicle_obs = (perception::PerceptionObstacle::VEHICLE ==
                         obstacle->Perception().type());
  if (is_vehicle_obs &&
      obstacle->speed() > config.overtake_velocity_limit_for_vehicle()) {
    return true;
  }
  double coef_of_road_limit =
      is_vehicle_obs ? config.overtake_coef_of_road_limit_for_vehicle()
                     : config.overtake_coef_of_road_limit();
  if (obstacle->speed() > (speed_limit * coef_of_road_limit + speed_buffer) ||
      obstacle->speed() > config.obs_faster_than_adc_ratio() * adc_v) {
    return true;
  }
  return false;
}

bool FsmAction::ObstacleIsBlocking(Frame* const frame,
                                   const Obstacle* obstacle) const {
  bool has_single_line = (1 == frame->reference_line_info().size());
  auto obs_id = obstacle->Id();
  for (const auto& ref_info : frame->reference_line_info()) {
    if (has_single_line) {
      return ref_info.IsObstacleBlockAdc(obstacle);
    }
    const auto* obs_info = ref_info.path_decision().Find(obs_id);
    if (!obs_info) {
      continue;
    }
    const auto& obs_sl = obs_info->PerceptionSLBoundary();
    if (ref_info.IsChangeLanePath()) {
      common::SLPoint sl_point;
      sl_point.set_s((obs_sl.start_s() + obs_sl.end_s()) * 0.5);
      sl_point.set_l((obs_sl.start_l() + obs_sl.end_l()) * 0.5);
      if (ref_info.reference_line().IsOnLane(sl_point)) {
        return false;
      }
      continue;
    }
    const auto& lane_width = ref_info.GetLaneWidthBaseOnAdcCenter();
    double left_space = lane_width.first - obs_sl.end_l();
    double right_space = lane_width.second + obs_sl.start_l();
    double remain_space = std::fmax(left_space, right_space);
    const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    if (remain_space <
        veh_param.width() + FLAGS_obstacle_max_lat_buffer_public_road) {
      return true;
    }
  }
  return false;
}

bool FsmAction::IsSafeToOvertake(
    ReferenceLineInfo* const reference_line_info, const bool& is_turn_left,
    const ReferenceLineInfo::CheckLevel& check_level) {
  // reference line is change lane
  if (reference_line_info->IsChangeLanePath()) {
    return reference_line_info->GetIsClearToChangeLane();
  }
  // get neighbor lane width
  double neighbor_lane_width = 0.0;
  if (!GetNeighborLaneWidth(reference_line_info, is_turn_left,
                            &neighbor_lane_width)) {
    return false;
  }

  // get lane check bound l
  const auto& lane_width = reference_line_info->GetLaneWidthBaseOnAdcCenter();
  double left_bound_l = is_turn_left ? lane_width.first + neighbor_lane_width
                                     : -lane_width.second;
  double right_bound_l = is_turn_left
                             ? lane_width.first
                             : -(lane_width.second + neighbor_lane_width);

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    double cutin_buffer =
        reference_line_info->GetObstacleCheckCuttinBuffer(obstacle);
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.start_l() > left_bound_l - cutin_buffer ||
        obstacle_sl.end_l() < right_bound_l + cutin_buffer) {
      continue;
    }

    if (is_turn_left &&
        obstacle->speed() < FLAGS_static_obstacle_speed_threshold &&
        reference_line_info->IsObstacleBlockAdc(obstacle)) {
      AINFO << "turn left, there has static obs: " << obstacle->Id()
            << " block adc";
      return false;
    }

    if (!reference_line_info->LaneChangeSafeCheck(check_level, obstacle)) {
      AINFO << "obstacle: " << obstacle->Id() << " safe check NOT OK";
      return false;
    }
  }
  return true;
}

bool FsmAction::GetNeighborLaneWidth(
    ReferenceLineInfo* const reference_line_info, const bool& is_turn_left,
    double* const neighbor_lane_width) {
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();

  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_info->reference_line().GetLaneFromS(
      (ego_start_s + ego_end_s) * 0.5, &lanes);
  if (lanes.empty() || nullptr == lanes.front()) {
    return false;
  }

  const auto& lane = lanes.front()->lane();
  century::hdmap::Id lane_id;
  if (is_turn_left) {  // get left lane id
    if (lane.left_neighbor_forward_lane_id_size() > 0) {
      lane_id = lane.left_neighbor_forward_lane_id(0);
    } else {
      AINFO << " Current lane has no left neighbor lane! ";
      return false;
    }
  } else {  // get right lane id
    if (lane.right_neighbor_forward_lane_id_size() > 0) {
      lane_id = lane.right_neighbor_forward_lane_id(0);
    } else {
      AINFO << " Current lane has no right neighbor lane! ";
      return false;
    }
  }

  if (!lane_id.has_id()) {
    AINFO << "Get neighbor lane error!";
    return false;
  }
  *neighbor_lane_width =
      century::hdmap::HDMapUtil::BaseMap().GetLaneById(lane_id)->GetWidth(
          (ego_start_s + ego_end_s) * 0.5);

  return true;
}

bool FsmAction::CheckIsForwoardMovingObstacle(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle* obstacle) const {
  util::MovingObstacleType moving_obstacle_type = util::GetMovingObstacleType(
      obstacle, reference_line_info->vehicle_state(),
      reference_line_info->reference_line());

  if (util::STRAIGHT_FORWARD == moving_obstacle_type ||
      util::NO_MOVING == moving_obstacle_type) {
    return true;
  }
  return false;
}

void FsmAction::ResetOvertakeInfo(Frame* const frame) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_history_overtake_signal(false);
  mutable_overtake->set_slow_obs_keep_check_times(0);
  mutable_overtake->set_safe_check_times(0);
  return;
}

bool FsmAction::CheckDistanceToOverlap(
    ReferenceLineInfo* const reference_line_info) {
  // check the distance between the stop sign and ADC is enough to overtake
  for (const auto& overlap : reference_line_info->FirstEncounteredOverlaps()) {
    if (overlap.first != ReferenceLineInfo::OverlapType::STOP_SIGN &&
        overlap.first != ReferenceLineInfo::OverlapType::SIGNAL &&
        overlap.first != ReferenceLineInfo::OverlapType::PNC_JUNCTION) {
      continue;
    }
    double dis_to_overlap =
        overlap.second.start_s - reference_line_info->AdcSlBoundary().end_s();
    lane_change_remain_dis_ =
        std::fmin(lane_change_remain_dis_, dis_to_overlap);
    if (dis_to_overlap < FLAGS_overtake_mindis_to_stopsign_threshold) {
      AINFO << "there left " << dis_to_overlap
            << "m to overlap: " << static_cast<int>(overlap.first)
            << ", not overtake";
      return false;
    }
  }
  return true;
}

bool FsmAction::CheckDistanceToDestination(
    ReferenceLineInfo* const reference_line_info) {
  double dis_to_dest = reference_line_info->SDistanceToDestination();
  lane_change_remain_dis_ = std::fmin(lane_change_remain_dis_, dis_to_dest);
  if (dis_to_dest < config_.lane_overtake_decider_config()
                        .forbid_overtake_distance_to_destination()) {
    AINFO << "close to SDistanceToDestination: " << dis_to_dest
          << ", not overtake";
    return false;
  }
  return true;
}

bool FsmAction::CheckSolidLine(ReferenceLineInfo* const reference_line_info,
                               const CheckDirection& check_direction) {
  if (CheckDirection::NO_CHECK == check_direction) {
    return true;
  }
  const auto& reference_line = reference_line_info->reference_line();
  double check_s = reference_line_info->AdcSlBoundary().end_s();
  if (find_solid_start_) {
    common::SLPoint point_sl;
    double start_point_s = std::numeric_limits<double>::lowest();
    if (reference_line.XYToSL(solid_start_point_, &point_sl)) {
      start_point_s = point_sl.s();
    }
    if (start_point_s < reference_line_info->AdcSlBoundary().start_s()) {
      find_solid_start_ = false;
    } else {
      double remain_dis = start_point_s - check_s;
      AINFO << "there left " << remain_dis
            << "m to solid line, do not overtake";
      lane_change_remain_dis_ = std::fmin(lane_change_remain_dis_, remain_dis);
      return false;
    }
  }

  const double lookforward_distance =
      std::fmin(check_s + FLAGS_overtake_mindis_to_stopsign_threshold,
                reference_line.Length());
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  bool check_both = (CheckDirection::CHECK_BOTH == check_direction);
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      return false;
    }

    const auto& waypoint = ref_point.lane_waypoints().front();
    if (CheckDirection::CHECK_LEFT == check_direction) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
    } else {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
    }
    if (!LaneBoundaryTypeCheckOK(reference_line_info, lane_boundary_type,
                                 check_s)) {
      return false;
    }

    if (check_both) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (!LaneBoundaryTypeCheckOK(reference_line_info, lane_boundary_type,
                                   check_s)) {
        return false;
      }
    }

    check_s += kCheckSolidLineStep;
  }
  return true;
}

bool FsmAction::LaneBoundaryTypeCheckOK(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::LaneBoundaryType::Type& lane_boundary_type,
    const double check_s) {
  if (hdmap::LaneBoundaryType::CURB == lane_boundary_type ||
      hdmap::LaneBoundaryType::UNKNOWN == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_YELLOW == lane_boundary_type ||
      hdmap::LaneBoundaryType::SOLID_WHITE == lane_boundary_type) {
    double remain_dis = check_s - reference_line_info->AdcSlBoundary().end_s();
    AINFO << "there left " << remain_dis
          << "m to laneType: " << static_cast<int>(lane_boundary_type)
          << ", do not overtake";
    lane_change_remain_dis_ = std::fmin(lane_change_remain_dis_, remain_dis);
    if (!find_solid_start_) {
      common::SLPoint slp;
      slp.set_s(check_s);
      slp.set_l(0.0);
      if (reference_line_info->reference_line().SLToXY(slp,
                                                       &solid_start_point_)) {
        find_solid_start_ = true;
      }
    }
    return false;
  }
  return true;
}

bool FsmAction::CheckRoutingEnd(ReferenceLineInfo* const reference_line_info,
                                const CheckDirection& check_direction) {
  PassageType passage_type = PassageType::SELF;
  switch (check_direction) {
    case CheckDirection::CHECK_SELF:
      passage_type = PassageType::SELF;
      break;
    case CheckDirection::CHECK_LEFT:
      passage_type = PassageType::LEFT;
      break;
    case CheckDirection::CHECK_RIGHT:
      passage_type = PassageType::RIGHT;
      break;
    default:
      break;
  }

  routing_end_remain_dis_ = kMaxLaneChangeRemainDistance;
  common::PointENU routing_end;
  if (reference_line_info->GetRoutePassageEndPoint(
          injector_->planning_context(), passage_type, &routing_end)) {
    common::SLPoint sl_point;
    common::math::Vec2d xy_point = {routing_end.x(), routing_end.y()};
    if (reference_line_info->reference_line().XYToSL(xy_point, &sl_point)) {
      double distance_to_passage_end =
          sl_point.s() - reference_line_info->AdcSlBoundary().end_s();
      routing_end_remain_dis_ = std::fmax(0.0, distance_to_passage_end);
      if (distance_to_passage_end <
              FLAGS_overtake_mindis_to_stopsign_threshold &&
          reference_line_info->reference_line().IsOnLane(sl_point)) {
        AINFO << "CheckRoutingEnd not ok, distance_to_passage_end: "
              << distance_to_passage_end;
        return false;
      }
    }
  }
  return true;
}

double FsmAction::GetMinOvertakeNeededDistance(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle* obstacle) const {
  double ratio =
      (FLAGS_enable_overtake_speed_up) ? FLAGS_overtake_speed_up_ratio : 1.0;
  double adc_max_v = std::fmin(FLAGS_planning_upper_speed_limit * ratio,
                               FLAGS_overtake_upper_speed_limit);
  double diff_v = adc_max_v - obstacle->speed();
  if (diff_v < 0.0) {
    return std::numeric_limits<double>::max();
  }
  diff_v = std::fmax(diff_v, kEpsilon);
  double distance = obstacle->PerceptionSLBoundary().start_s() -
                    reference_line_info->AdcSlBoundary().end_s();
  double obs_length = obstacle->PerceptionSLBoundary().end_s() -
                      obstacle->PerceptionSLBoundary().start_s();
  double adc_length =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length();
  const auto& config = config_.lane_overtake_decider_config();
  double total_distance = (distance + obs_length + adc_length +
                           obstacle->speed() * config.lane_change_hwt());
  double chase_time = total_distance / diff_v;
  double min_need_overtake_dis = (config.fixed_overtake_spend_time() +
                                  chase_time + FLAGS_lane_change_total_time) *
                                 adc_max_v;
  AINFO << "min_need_overtake_dis: " << min_need_overtake_dis;
  return min_need_overtake_dis;
}

bool FsmAction::HasOvertakePassTheObstacle(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  double turn_timestamp = mutable_overtake->turn_timestamp();
  double delta_t = Clock::NowInSeconds() - turn_timestamp;
  const auto& config = config_.lane_overtake_decider_config();

  if (nullptr == obstacle) {
    if (delta_t < config.max_overtake_spend_time()) {
      return false;
    }
  } else if (mutable_overtake->accelerated_mode() || !need_give_up_overtake_) {
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    double passed_dis =
        reference_line_info->AdcSlBoundary().start_s() - obstacle_sl.end_s();
    double dropped_dis =
        obstacle_sl.start_s() - reference_line_info->AdcSlBoundary().end_s();
    double adc_v = reference_line_info->vehicle_state().linear_velocity();
    double delta_v = adc_v - obstacle->speed();
    if (passed_dis < config.overtake_return_dis()) {
      if (delta_v >
          obstacle->speed() * config.adc_faster_than_obstacle_ratio()) {
        return false;
      }
      double check_drop =
          delta_v * (kChangeLaneTime + kSafeTimeOnSameDirection) +
          obstacle->speed() * config.lane_change_hwt() +
          delta_v * delta_v / FLAGS_slowdown_profile_deceleration * 0.5;
      if (dropped_dis < check_drop &&
          delta_t < config.max_overtake_spend_time()) {
        return false;
      }
      if (passed_dis < 0.0 && delta_v < 0.0) {
        mutable_overtake->set_accelerated_mode(false);
        need_give_up_overtake_ = true;
      }
    }
  }
  return true;
}

bool FsmAction::AdcWillPassMergeLane(
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider_status.will_pass_merge_lane_area()) {
    AINFO << "[Overtake] adc will pass merge lane, not turn/return.";
    return true;
  }

  double check_s = reference_line_info->AdcSlBoundary().end_s();
  while (check_s < reference_line_info->reference_line().Length()) {
    auto curr_lane_info = reference_line_info->LocateLaneInfo(check_s);
    if (nullptr == curr_lane_info) {
      break;
    }
    if (curr_lane_info->lane().is_merge()) {
      AINFO << "[Overtake] curr ref will pass merge lane, not turn/return.";
      return true;
    }
    if (NeighborLaneIsMerge(reference_line_info, check_s)) {
      AINFO << "[Overtake] neighbor ref will pass merge lane, not turn/return.";
      return true;
    }
    check_s += kCheckMergeLineStep;
  }
  return false;
}

bool FsmAction::NeighborLaneIsMerge(
    ReferenceLineInfo* const reference_line_info, const double check_s) {
  hdmap::Id neighbor_lane_id;
  double neighbor_lane_width;
  if (reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, check_s, &neighbor_lane_id,
          &neighbor_lane_width)) {
    auto ptr_neighbor_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
    if (ptr_neighbor_lane && ptr_neighbor_lane->lane().is_merge()) {
      ADEBUG << "left forward neighbor lane is merge";
      return true;
    }
  }
  if (reference_line_info->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, check_s, &neighbor_lane_id,
          &neighbor_lane_width)) {
    auto ptr_neighbor_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
    if (ptr_neighbor_lane && ptr_neighbor_lane->lane().is_merge()) {
      ADEBUG << "right forward neighbor lane is merge";
      return true;
    }
  }
  return false;
}

bool FsmAction::NeedCancelLaneChangeInMergeLane(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .will_pass_merge_lane_area() &&
      !reference_line_info->IsAdcCenterInLane()) {
    frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
    frame->UpdateLaneChangeLaneId("");
    return true;
  }
  return false;
}

void FsmAction::CheckIsNeedBuildLaneChangeStop(
    ReferenceLineInfo* const reference_line_info, bool* const need_stop,
    double* const stop_s) {
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  lane_change_remain_dis_ =
      std::fmin(routing_end_remain_dis_, kMaxLaneChangeRemainDistance);
  bool check_stop_ok =
      CheckDistanceToDestination(reference_line_info) &&
      CheckSolidLine(reference_line_info, CheckDirection::CHECK_LEFT) &&
      CheckDistanceToOverlap(reference_line_info);
  double lane_change_remain_dis =
      std::fmin(lane_change_remain_dis_, kMaxLaneChangeRemainDistance);
  if (!check_stop_ok ||
      (lane_change_remain_dis > 0.0 &&
       lane_change_remain_dis < FLAGS_overtake_mindis_to_stopsign_threshold)) {
    double check_s = adc_end_s + lane_change_remain_dis;
    double limit_stop_s = check_s - FLAGS_limit_stop_wall_for_lane_change;
    *stop_s = (adc_end_s > limit_stop_s) ? adc_end_s : limit_stop_s;

    if (lane_change_remain_dis < FLAGS_brake_buffer_for_lane_change +
                                     FLAGS_limit_stop_wall_for_lane_change) {
      // do nothing
    } else if (lane_change_remain_dis <
               FLAGS_preview_brake_distance_for_lane_change +
                   0.5 * FLAGS_brake_buffer_for_lane_change) {
      *stop_s = check_s - FLAGS_brake_buffer_for_lane_change;
    } else {
      *stop_s = check_s - FLAGS_preview_brake_distance_for_lane_change;
    }

    ADEBUG << "change lane BuildStopDecision, lane change wall, stop_s: "
           << *stop_s;
    *need_stop = true;
  }
}

void FsmAction::InitParamInfo(Frame* const frame) {
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  mutable_overtake->set_is_in_overtake_status(false);
  mutable_overtake->set_history_overtake_signal(false);
  mutable_overtake->set_urgency_lane_change(false);
  mutable_overtake->set_accelerated_mode(false);
  mutable_overtake->set_stop_change_lane(false);
  mutable_overtake->set_safe_check_times(0);
  find_solid_start_ = false;
  frame->UpdateOvertakeStatus(OvertakeStatus::DEFAULT);
  origin_lane_id_ = "";
  frame->UpdateLaneChangeLaneId(origin_lane_id_);
}
}  // namespace planning
}  // namespace century
