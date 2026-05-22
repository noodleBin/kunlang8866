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

#include "modules/planning/tasks/deciders/risk_shift_decider/risk_shift_decider.h"

#include <algorithm>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

using century::common::SLPoint;
using century::common::Status;
using century::perception::PerceptionObstacle;

namespace {
constexpr double kEpsilon = 1e-2;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kLateralOverlapBuffer = 0.5;
constexpr double kEfficiencyCheckVelocityBuffer = 1.0;
constexpr double kEfficiencyLateralSafeBuffer = 1.0;
constexpr double kLatHysteresisBuffer = 0.3;
constexpr int kDefaultReverseCheckTimes = 10000;
}  // namespace

RiskShiftDecider::RiskShiftDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status RiskShiftDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  ReverseShiftAvoid(frame, reference_line_info);
  EfficiencyShiftBypass(frame, reference_line_info);
  return Status::OK();
}

void RiskShiftDecider::ReverseShiftAvoid(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_is_in_reverse_avoid_state(false);
  if (!config_.risk_shift_decider_config().enable_reverse_shift_avoid()) {
    return;
  }

  // 0. check is need into reverse avoid state
  bool has_risk = GetReverseRiskObstacle(reference_line_info);
  UpdateReverseShiftState(reference_line_info);

  // 1. if into reverse avoid state, then compute shift value
  if (is_in_reverse_avoid_state_ && has_risk) {
    double res = OptimizeRiskProblem(reference_line_info);
    double min_dis = std::fmin(left_front_reverse_nearest_id_.second,
                               right_front_reverse_nearest_id_.second);
    double confidence = util::GetObstacleConfidence(min_dis);
    reference_line_info->SetRiskShiftResult(res, confidence);
    AINFO << "[ReverseShiftAvoid] optimzied result is: " << res
          << ", confidence: " << confidence;
    last_lateral_risk_shift_.first = res;
    last_lateral_risk_shift_.second = confidence;
  } else if (is_in_reverse_avoid_state_) {
    reference_line_info->SetRiskShiftResult(last_lateral_risk_shift_.first,
                                            last_lateral_risk_shift_.second);
    AINFO << "[ReverseShiftAvoid] use last_lateral_risk_shift: "
          << last_lateral_risk_shift_.first
          << ", confidence: " << last_lateral_risk_shift_.second;
  } else {
    reference_line_info->SetRiskShiftResult(0.0, 1.0);
    last_lateral_risk_shift_.first = 0.0;
    last_lateral_risk_shift_.second = 0.0;
  }
  mutable_path_decider_status->set_is_in_reverse_avoid_state(
      is_in_reverse_avoid_state_);
  mutable_path_decider_status->set_left_reverse_unsafe(
      (!left_front_reverse_nearest_id_.first.empty() ||
       has_left_unsafe_sidebyside_obs_) &&
      is_in_reverse_avoid_state_);
  return;
}

void RiskShiftDecider::UpdateReverseShiftState(
    ReferenceLineInfo* const reference_line_info) {
  const auto& config = config_.risk_shift_decider_config();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  if (is_in_reverse_avoid_state_) {
    int count =
        mutable_path_decider_status->quit_reverse_shift_state_keep_times();
    if (left_front_reverse_nearest_id_.first.empty() &&
        right_front_reverse_nearest_id_.first.empty()) {
      mutable_path_decider_status->set_quit_reverse_shift_state_keep_times(
          ++count);
    } else {
      count = 0;
      mutable_path_decider_status->set_quit_reverse_shift_state_keep_times(
          count);
    }
    if (count > config.quit_reverse_shift_state_count_threshold() ||
        DisableRiskShiftInPlayStreet()) {
      is_in_reverse_avoid_state_ = false;
      mutable_path_decider_status->set_quit_reverse_shift_state_keep_times(0);
      left_front_reverse_id_.clear();
      right_front_reverse_id_.clear();
    }
    AINFO << "[Front] LFR_N: " << left_front_reverse_nearest_id_.first
          << ", RFR_N: " << right_front_reverse_nearest_id_.first
          << "; [Rear] LRF_N: " << left_rear_forward_nearest_id_.first
          << ", RRF_N: " << right_rear_forward_nearest_id_.first;
    return;
  }

  if (DisableRiskShiftInPlayStreet()) {
    AINFO << "[WARN] Not support risk shift in play street!";
    return;
  }

  for (const auto& risk_obs : left_front_reverse_id_) {
    if (risk_obs.second >= GetReverseCheckTimeBaseOnDistance(
                               reference_line_info, risk_obs.first)) {
      is_in_reverse_avoid_state_ = true;
    }
  }

  for (const auto& risk_obs : right_front_reverse_id_) {
    if (risk_obs.second >= GetReverseCheckTimeBaseOnDistance(
                               reference_line_info, risk_obs.first)) {
      is_in_reverse_avoid_state_ = true;
    }
  }

  if (is_in_reverse_avoid_state_) {
    AINFO << "[Reverse Avoid] TURN ON!";
    AINFO << "[Front] LFR_N: " << left_front_reverse_nearest_id_.first
          << ", RFR_N: " << right_front_reverse_nearest_id_.first
          << "; [Rear] LRF_N: " << left_rear_forward_nearest_id_.first
          << ", RRF_N: " << right_rear_forward_nearest_id_.first;
  }
  return;
}

bool RiskShiftDecider::GetReverseRiskObstacle(
    ReferenceLineInfo* const reference_line_info) {
  // screen out risk obstacles
  const auto& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();

  left_front_reverse_nearest_id_ = {"", std::numeric_limits<double>::max()};
  right_front_reverse_nearest_id_ = {"", std::numeric_limits<double>::max()};
  left_rear_forward_nearest_id_ = {"", std::numeric_limits<double>::max()};
  right_rear_forward_nearest_id_ = {"", std::numeric_limits<double>::max()};
  has_left_unsafe_sidebyside_obs_ = false;

  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (nullptr == obstacle || obstacle->IsVirtual()) {
      continue;
    }

    if (!CanAvoidStaticObstacle(reference_line_info, obstacle)) {
      continue;
    }

    if (CanIgnoreRearCar(reference_line_info, obstacle)) {
      continue;
    }

    if (CheckLateralIsSafe(reference_line_info, obstacle)) {
      continue;
    }

    if (CheckCollisionSafe(reference_line_info, obstacle)) {
      continue;
    }

    bool is_left_reverse_obs = false;
    bool is_right_reverse_obs = false;
    SaveRiskObstacleInfo(reference_line_info, obstacle, &is_left_reverse_obs,
                         &is_right_reverse_obs);
    UpdateReverseRiskObstacleInfo(reference_line_info, obstacle,
                                  is_left_reverse_obs, is_right_reverse_obs);
  }
  if (!left_front_reverse_nearest_id_.first.empty() ||
      !right_front_reverse_nearest_id_.first.empty()) {
    return true;
  }
  return false;
}

bool RiskShiftDecider::CanAvoidStaticObstacle(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  if (!obstacle->IsStatic() &&
      obstacle->speed() > FLAGS_static_obstacle_speed_threshold) {
    return true;
  }
  if (!config_.risk_shift_decider_config()
           .enable_risk_shift_avoid_static_obstacle()) {
    return false;
  }
  if (PerceptionObstacle::UNKNOWN == obstacle->Perception().type()) {
    return false;
  }
  if (!obstacle->IsNeedShift()) {
    return false;
  }
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  if (obs_sl.start_l() < adc_center_l) {
    return false;
  }
  ADEBUG << "can avoid static obs: " << obstacle->Id();
  return true;
}

bool RiskShiftDecider::CheckLateralIsSafe(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  double lateral_risk_safe_buffer = config.check_lateral_risk_safe_buffer();
  if (obstacle->IsLargeVehicle(config.large_vehicle_check_length())) {
    lateral_risk_safe_buffer =
        config.check_lateral_risk_safe_buffer_for_large_vehicle();
  } else if (PerceptionObstacle::VEHICLE == obstacle->Perception().type()) {
    lateral_risk_safe_buffer =
        config.check_lateral_risk_safe_buffer_for_vehicle();
  }

  if (is_in_reverse_avoid_state_) {
    lateral_risk_safe_buffer += kLatHysteresisBuffer;
  }

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  if (obs_sl.start_l() > adc_sl.end_l() + lateral_risk_safe_buffer ||
      obs_sl.end_l() < adc_sl.start_l() - lateral_risk_safe_buffer) {
    return true;
  }
  ADEBUG << "[lateral check] not safe, obstacle[" << obstacle->Id()
         << "], risk buffer: " << lateral_risk_safe_buffer;
  return false;
}

bool RiskShiftDecider::CanIgnoreRearCar(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  double obs_heading_diff =
      obstacle->HeadingDiffWithLane(reference_line_info->reference_line());
  bool is_a_rear_obs = obs_sl.end_s() < adc_sl.start_s();
  bool is_in_left_side =
      obs_sl.start_l() > adc_sl.end_l() - kLateralOverlapBuffer;
  bool is_in_right_side =
      obs_sl.end_l() < adc_sl.start_l() + kLateralOverlapBuffer;
  if (is_a_rear_obs && (std::fabs(obs_heading_diff) > M_PI_4 ||
                        (!is_in_left_side && !is_in_right_side))) {
    return true;
  }
  return false;
}

double RiskShiftDecider::GetLateralSpeedBuffer(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  double obs_lat_v = obstacle->LateralSpeed();
  double adc_lat_v = reference_line_info->GetAdcLateralVelocity();
  double delta_lat_v = adc_lat_v - obs_lat_v;
  double vetor_sign = delta_lat_v > 0.0 ? 1.0 : -1.0;
  double lat_speed_buffer = vetor_sign * delta_lat_v * delta_lat_v /
                            config.check_reverse_max_lateral_deceleration();
  ADEBUG << "obs_lat_v: " << obs_lat_v << ", adc_lat_v: " << adc_lat_v
         << ", vetor_sign: " << vetor_sign
         << ", lat_speed_buffer: " << lat_speed_buffer;
  return lat_speed_buffer;
}

bool RiskShiftDecider::CheckCollisionSafe(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();

  if (obs_sl.start_s() < adc_sl.end_s() && obs_sl.end_s() > adc_sl.start_s()) {
    if (!has_left_unsafe_sidebyside_obs_) {
      has_left_unsafe_sidebyside_obs_ = (obs_sl.start_l() > adc_sl.end_l());
    }
    ADEBUG << "[collision check] not safe, obstacle[" << obstacle->Id()
           << "] side by side with adc, not safe";
    return false;
  }
  if (obs_sl.start_s() - adc_sl.end_s() >
      config.ignore_lon_risk_reverse_obs_threshold()) {
    return true;
  }

  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_v = obstacle->LongitudinalSpeed();
  double distance = 0.0;
  double delta_v = 0.0;
  double rear_car_v = 0.0;

  if (obs_sl.start_s() > adc_sl.start_s()) {
    distance = obs_sl.start_s() - adc_sl.end_s();
    delta_v = adc_v - obs_v;
    rear_car_v = adc_v;
  } else {
    distance = adc_sl.start_s() - obs_sl.end_s();
    delta_v = obs_v - adc_v;
    rear_car_v = obs_v;
  }

  double obs_heading_diff =
      obstacle->HeadingDiffWithLane(reference_line_info->reference_line());
  bool same_direction_obs = (std::fabs(obs_heading_diff) < M_PI_2);
  double check_ttc =
      same_direction_obs ? config.cruise_ttc() : config.reverse_ttc();
  double check_hwt =
      same_direction_obs ? config.cruise_hwt() : config.reverse_hwt();

  if (distance > delta_v * check_ttc && distance > rear_car_v * check_hwt &&
      distance > config.cruise_min_lon_safe_distance()) {
    return true;
  }
  ADEBUG << "[collision check] not safe, obstacle[" << obstacle->Id()
         << "] obs_v: " << obs_v << ", adc_v: " << adc_v
         << ", same direction: " << same_direction_obs
         << ", delta_v: " << delta_v << ", rear_car_v: " << rear_car_v
         << ", distance: " << distance << ", check_ttc: " << check_ttc
         << ", check_hwt: " << check_hwt
         << ", check_min_distance: " << config.cruise_min_lon_safe_distance();
  return false;
}

void RiskShiftDecider::SaveRiskObstacleInfo(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle,
    bool* const is_left_reverse_obs, bool* const is_right_reverse_obs) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double obs_heading_diff =
      obstacle->HeadingDiffWithLane(reference_line_info->reference_line());
  bool is_in_left_side = obs_sl.end_l() > adc_sl.end_l();
  bool is_in_right_side =
      obs_sl.end_l() <
      adc_sl.start_l() + config.consider_reverse_lateral_overlap_buffer();
  bool is_forward =
      std::fabs(obs_heading_diff) <
      config.check_forward_heading_diff_degree() * kDegreesToRadians;

  if ((obs_sl.start_s() < adc_sl.end_s()) && is_forward) {
    double distance = adc_sl.start_s() - obs_sl.end_s();
    if (is_in_left_side && left_rear_forward_nearest_id_.second > distance) {
      left_rear_forward_nearest_id_.second = distance;
      left_rear_forward_nearest_id_.first = obstacle->Id();
      return;
    }
    if (is_in_right_side && right_rear_forward_nearest_id_.second > distance) {
      right_rear_forward_nearest_id_.second = distance;
      right_rear_forward_nearest_id_.first = obstacle->Id();
      return;
    }
    return;
  }

  bool is_reverse =
      std::fabs(obs_heading_diff) >
      config.check_reverse_heading_diff_degree() * kDegreesToRadians;
  if ((obs_sl.end_s() > adc_sl.start_s()) &&
      (is_reverse || EnableShiftStaticObstacle(obstacle) ||
       IsNeedConsiderSlowForwardObstacle(reference_line_info, obstacle))) {
    double distance = obs_sl.start_s() - adc_sl.end_s();
    if (is_in_left_side) {
      *is_left_reverse_obs = true;
      if (left_front_reverse_id_.find(obstacle->Id()) ==
          left_front_reverse_id_.end()) {
        left_front_reverse_id_.insert({obstacle->Id(), 1});
      } else {
        ++left_front_reverse_id_.find(obstacle->Id())->second;
      }

      if (left_front_reverse_nearest_id_.second > distance) {
        left_front_reverse_nearest_id_.second = distance;
        left_front_reverse_nearest_id_.first = obstacle->Id();
      }
      return;
    }
    if (is_in_right_side) {
      *is_right_reverse_obs = true;
      if (right_front_reverse_id_.find(obstacle->Id()) ==
          right_front_reverse_id_.end()) {
        right_front_reverse_id_.insert({obstacle->Id(), 1});
      } else {
        ++right_front_reverse_id_.find(obstacle->Id())->second;
      }
      if (right_front_reverse_nearest_id_.second > distance) {
        right_front_reverse_nearest_id_.second = distance;
        right_front_reverse_nearest_id_.first = obstacle->Id();
      }
      return;
    }
    return;
  }
  return;
}

bool RiskShiftDecider::EnableShiftStaticObstacle(const Obstacle* obstacle) {
  if (!config_.risk_shift_decider_config()
           .enable_risk_shift_avoid_static_obstacle()) {
    return false;
  }
  if (obstacle->IsStatic() ||
      obstacle->speed() < FLAGS_static_obstacle_speed_threshold) {
    return true;
  }
  return false;
}

bool RiskShiftDecider::IsNeedConsiderSlowForwardObstacle(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  if (!config.enable_risk_shift_avoid_slow_obstacle()) {
    return false;
  }

  double obs_heading_diff =
      obstacle->HeadingDiffWithLane(reference_line_info->reference_line());
  bool is_forward =
      std::fabs(obs_heading_diff) <
      config.check_forward_heading_diff_degree() * kDegreesToRadians;
  if (!is_forward) {
    return false;
  }

  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_v = obstacle->LongitudinalSpeed();
  return obs_v < adc_v + kEpsilon;
}

void RiskShiftDecider::UpdateReverseRiskObstacleInfo(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle,
    const bool is_left_reverse_obs, const bool is_right_reverse_obs) {
  if (!is_left_reverse_obs && !left_front_reverse_id_.empty() &&
      left_front_reverse_id_.find(obstacle->Id()) !=
          left_front_reverse_id_.end()) {
    --left_front_reverse_id_.find(obstacle->Id())->second;
    ADEBUG << "left obs: " << obstacle->Id() << ", disappear this time";
    if (left_front_reverse_id_.find(obstacle->Id())->second <= 0) {
      left_front_reverse_id_.erase(left_front_reverse_id_.find(obstacle->Id()));
      ADEBUG << "erase no longer exist left risk obs: " << obstacle->Id();
    }
  }

  if (!is_right_reverse_obs && !right_front_reverse_id_.empty() &&
      right_front_reverse_id_.find(obstacle->Id()) !=
          right_front_reverse_id_.end()) {
    --right_front_reverse_id_.find(obstacle->Id())->second;
    ADEBUG << "right obs: " << obstacle->Id() << ", disappear this time";
    if (right_front_reverse_id_.find(obstacle->Id())->second <= 0) {
      right_front_reverse_id_.erase(
          right_front_reverse_id_.find(obstacle->Id()));
      ADEBUG << "erase no longer exist right risk obs: " << obstacle->Id();
    }
  }
  return;
}

double RiskShiftDecider::OptimizeRiskProblem(
    ReferenceLineInfo* const reference_line_info) {
  lateral_step_ = InterpolationLookUp(
      reference_line_info->GetAdcLongitudinalVelocity(),
      FLAGS_min_speed_step_end_state_l, FLAGS_max_speed_step_end_state_l,
      FLAGS_max_step_end_state_l, FLAGS_min_step_end_state_l);
  ADEBUG << "[OptimizeRiskProblem] Get lateral step: " << lateral_step_;

  // Shift_limit_bounds: <min, max>
  std::pair<double, double> shift_limit_bounds;
  if (!GetShiftLimitBounds(reference_line_info, &shift_limit_bounds)) {
    AINFO << "Can not get bound limits, not shift!";
    return 0.0;
  }

  const auto& config = config_.risk_shift_decider_config();
  double check_l = shift_limit_bounds.first;
  double min_cost = std::numeric_limits<double>::max();
  double best_shift_value = check_l;
  while (check_l <= shift_limit_bounds.second) {
    double risk = ComputeRiskValue(reference_line_info, check_l);
    double efficiency = ComputeEfficiencyValue(reference_line_info, check_l);
    double cost = risk * config.reverse_shift_result_risk_weight() +
                  efficiency * config.reverse_shift_result_efficiency_weight();
    ADEBUG << "[OptimizeRiskProblem] check_l: " << check_l << ", risk: " << risk
           << ", efficiency: " << efficiency << ", cost: " << cost;
    if (min_cost > cost) {
      min_cost = cost;
      best_shift_value = check_l;
    }
    check_l += lateral_step_;
  }
  return best_shift_value;
}

bool RiskShiftDecider::GetShiftLimitBounds(
    ReferenceLineInfo* const reference_line_info,
    std::pair<double, double>* const shift_limit_bounds) {
  const auto& path_bounds = reference_line_info->GetCandidatePathBoundaries();
  if (path_bounds.empty()) {
    AERROR << "[GetShiftLimitBounds] path bound is empty, not shift!";
    return false;
  }
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  bool has_bounds = false;
  for (const auto& bound : path_bounds) {
    if (bound.label().find("fallback") != std::string::npos &&
        VaildPathLable::FALLBACK == injector_->last_path_label_) {
      auto bound_limit = bound.GetBoundLimit();
      shift_limit_bounds->first = bound_limit.first;
      shift_limit_bounds->second = bound_limit.second;
      has_bounds = true;
      ADEBUG << "[fallback path] min_l: " << shift_limit_bounds->first
             << ", max_l: " << shift_limit_bounds->second;
      break;
    }
    if (bound.label().find("self") != std::string::npos &&
        VaildPathLable::SELF == injector_->last_path_label_) {
      auto bound_limit = bound.GetBoundLimit();
      shift_limit_bounds->first = bound_limit.first;
      shift_limit_bounds->second = bound_limit.second;
      has_bounds = true;
      ADEBUG << "[self path] min_l: " << shift_limit_bounds->first
             << ", max_l: " << shift_limit_bounds->second;
      break;
    }
    if (bound.label().find("left") != std::string::npos &&
        VaildPathLable::LEFT_BORROW == injector_->last_path_label_) {
      shift_limit_bounds->first = adc_center_l + kEpsilon;
      shift_limit_bounds->second = bound.GetBoundLimit().second;
      has_bounds = true;
      ADEBUG << "[left path] min_l: " << shift_limit_bounds->first
             << ", max_l: " << shift_limit_bounds->second;
      break;
    }
    if (bound.label().find("right") != std::string::npos &&
        VaildPathLable::RIGHT_BORROW == injector_->last_path_label_) {
      shift_limit_bounds->first = bound.GetBoundLimit().first;
      shift_limit_bounds->second = adc_center_l - kEpsilon;
      has_bounds = true;
      ADEBUG << "[right path] min_l: " << shift_limit_bounds->first
             << ", max_l: " << shift_limit_bounds->second;
      break;
    }
  }
  return has_bounds;
}

double RiskShiftDecider::ComputeRiskValue(
    ReferenceLineInfo* const reference_line_info, const double check_l) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  double delta_l = check_l - adc_center_l;
  double delta_t = std::fabs(delta_l) / std::fmax(kEpsilon, lateral_step_);
  ADEBUG << "[ComputeRiskValue] check_l: " << check_l
         << ", adc_center_l: " << adc_center_l << ", delta_l: " << delta_l
         << ", delta_t: " << delta_t;
  return GetLeftFrontReverseRisk(reference_line_info, check_l, delta_t) *
             config.left_front_reverse_risk_weight() +
         GetRightFrontReverseRisk(reference_line_info, check_l, delta_t) *
             config.right_front_reverse_risk_weight() +
         GetLeftRearForwardRisk(reference_line_info, check_l, delta_t) *
             config.left_rear_forward_risk_weight() +
         GetRightRearForwardRisk(reference_line_info, check_l, delta_t) *
             config.right_rear_forward_risk_weight();
}

double RiskShiftDecider::GetLeftFrontReverseRisk(
    ReferenceLineInfo* const reference_line_info, const double check_l,
    const double delta_t) {
  if (left_front_reverse_nearest_id_.first.empty()) {
    return 0.0;
  }
  const auto* obs_info = reference_line_info->path_decision()->Find(
      left_front_reverse_nearest_id_.first);
  if (!obs_info) {
    return 0.0;
  }
  double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  const auto& obs_sl = obs_info->PerceptionSLBoundary();
  ADEBUG << "Need compute Left Front Reverse Risk, obs_start_l: "
         << obs_sl.start_l() << ", obs_end_l:" << obs_sl.end_l();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double obs_delta_l = obs_sl.start_l() - check_l - half_width;
  double obs_v = obs_info->LongitudinalSpeed();
  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_delta_s = std::fmax(
      0.0, obs_sl.start_s() - adc_sl.end_s() - (adc_v - obs_v) * delta_t);
  return RiskAreaEstimate(true, adc_v - obs_v, obs_v, obs_delta_l, obs_delta_s);
}

double RiskShiftDecider::GetRightFrontReverseRisk(
    ReferenceLineInfo* const reference_line_info, const double check_l,
    const double delta_t) {
  if (right_front_reverse_nearest_id_.first.empty()) {
    return 0.0;
  }
  const auto* obs_info = reference_line_info->path_decision()->Find(
      right_front_reverse_nearest_id_.first);
  if (!obs_info) {
    return 0.0;
  }
  double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  const auto& obs_sl = obs_info->PerceptionSLBoundary();
  ADEBUG << "Need compute Right Front Reverse Risk, obs_start_l: "
         << obs_sl.start_l() << ", obs_end_l:" << obs_sl.end_l();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double obs_delta_l = check_l - half_width - obs_sl.end_l();
  double obs_v = obs_info->LongitudinalSpeed();
  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_delta_s = std::fmax(
      0.0, obs_sl.start_s() - adc_sl.end_s() - (adc_v - obs_v) * delta_t);
  return RiskAreaEstimate(true, adc_v - obs_v, obs_v, obs_delta_l, obs_delta_s);
}

double RiskShiftDecider::GetLeftRearForwardRisk(
    ReferenceLineInfo* const reference_line_info, const double check_l,
    const double delta_t) {
  if (left_rear_forward_nearest_id_.first.empty()) {
    return 0.0;
  }
  const auto* obs_info = reference_line_info->path_decision()->Find(
      left_rear_forward_nearest_id_.first);
  if (!obs_info) {
    return 0.0;
  }
  double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  const auto& obs_sl = obs_info->PerceptionSLBoundary();
  ADEBUG << "Need compute Left Rear Forward Risk, obs_start_l: "
         << obs_sl.start_l() << ", obs_end_l:" << obs_sl.end_l();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double obs_delta_l = obs_sl.start_l() - check_l - half_width;
  double obs_v = obs_info->LongitudinalSpeed();
  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_delta_s =
      adc_sl.start_s() - obs_sl.end_s() - (obs_v - adc_v) * delta_t;
  return RiskAreaEstimate(false, obs_v - adc_v, obs_v, obs_delta_l,
                          obs_delta_s);
}

double RiskShiftDecider::GetRightRearForwardRisk(
    ReferenceLineInfo* const reference_line_info, const double check_l,
    const double delta_t) {
  if (right_rear_forward_nearest_id_.first.empty()) {
    return 0.0;
  }
  const auto* obs_info = reference_line_info->path_decision()->Find(
      right_rear_forward_nearest_id_.first);
  if (!obs_info) {
    return 0.0;
  }
  double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  const auto& obs_sl = obs_info->PerceptionSLBoundary();
  ADEBUG << "Need compute Right Rear Forward Risk, obs_start_l: "
         << obs_sl.start_l() << ", obs_end_l:" << obs_sl.end_l();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double obs_delta_l = check_l - obs_sl.end_l() - half_width;
  double obs_v = obs_info->LongitudinalSpeed();
  double adc_v = reference_line_info->GetAdcLongitudinalVelocity();
  double obs_delta_s =
      adc_sl.start_s() - obs_sl.end_s() - (obs_v - adc_v) * delta_t;
  return RiskAreaEstimate(true, obs_v - adc_v, obs_v, obs_delta_l, obs_delta_s);
}

double RiskShiftDecider::RiskAreaEstimate(const bool reverse_check,
                                          const double init_v,
                                          const double obs_v,
                                          const double delta_l,
                                          const double delta_s) {
  const auto& config = config_.risk_shift_decider_config();

  // lon risk func: y = a * e^x + b; a = 1 / (e^max_dec - 1), b = -a; x =
  // (v_lon)^2 / (2 * s);
  double factor_a = 1 / (std::exp(config.check_risk_max_lon_decelerate()) - 1);
  double factor_b = -factor_a;
  double factor_x = init_v * init_v * 0.5;
  auto lon_risk_func = [factor_a, factor_b, factor_x](double s) {
    return factor_a * std::exp(factor_x / s) + factor_b;
  };

  double limit_s =
      std::fmax(config.cruise_min_lon_safe_distance(),
                init_v * init_v * 0.5 / config.check_risk_max_lon_decelerate());
  double lon_risk = delta_s < limit_s
                        ? 1.0
                        : (1 / lon_risk_func(limit_s)) * lon_risk_func(delta_s);
  ADEBUG << "[lon_risk]: " << lon_risk << "; limit_s: " << limit_s
         << ", delta_s: " << delta_s << ", factor_a: " << factor_a
         << ", factor_b: " << factor_b << ", factor_x: " << factor_x
         << ", init_v: " << init_v;

  // lat risk func: y = a * (e^(1 / t) - e^(1 / max_lat_react_t)); a = (1 / e^(1
  // / min_lat_react_t) - e^(1 / max_lat_react_t)); t = l / v_lat;
  double min_react_t = std::fmax(
      kEpsilon, reverse_check ? config.min_danger_lat_reverse_react_time()
                              : config.min_danger_lat_forward_react_time());
  double max_react_t = std::fmax(
      kEpsilon, reverse_check ? config.max_safe_lat_reverse_react_time()
                              : config.max_safe_lat_forward_react_time());
  double factor_c = 1 / (std::exp(1 / min_react_t) - std::exp(1 / max_react_t));
  double factor_d = std::exp(1 / max_react_t);
  double factor_t =
      (std::fabs(obs_v) < FLAGS_static_obstacle_speed_threshold)
          ? FLAGS_static_obstacle_speed_threshold *
                config.potential_lat_steering_angle_for_static_obs()
          : std::fabs(obs_v) * std::sin(config.potential_lat_steering_angle());
  double value_rate_of_rise = config.lateral_risk_value_rate_of_rise();
  auto lat_risk_func = [factor_c, factor_d, factor_t,
                        value_rate_of_rise](double l) {
    return factor_c * (std::exp(value_rate_of_rise * factor_t / l) - factor_d);
  };

  double limit_l = std::fmax(
      config.cruise_min_lat_safe_distance(),
      factor_t * factor_t * 0.5 / config.check_risk_max_lat_decelerate());
  double safe_l =
      std::fmax(config.cruise_max_lat_safe_distance(), factor_t * max_react_t);
  double lat_disk =
      delta_l < limit_l
          ? 1.0 + (limit_l - delta_l)
          : ((delta_l > safe_l)
                 ? kEpsilon
                 : std::fmax(kEpsilon, (1 / lat_risk_func(limit_l)) *
                                           lat_risk_func(delta_l)));
  ADEBUG << "[lat_disk]: " << lat_disk << "; limit_l: " << limit_l
         << ", delta_l: " << delta_l << ", factor_c: " << factor_c
         << ", factor_d: " << factor_d << ", factor_t: " << factor_t
         << ", obs_v: " << std::fabs(obs_v) << ", safe_l: " << safe_l;

  double res = 0.0;
  if (reverse_check) {
    res = lon_risk * config.reverse_lon_risk_weight() +
          lat_disk * config.reverse_lat_risk_weight();
  } else {
    res = lon_risk * config.forward_lon_risk_weight() +
          lat_disk * config.forward_lat_risk_weight();
  }
  ADEBUG << "[RiskAreaEstimate] final risk result: " << res;
  return res;
}

double RiskShiftDecider::ComputeEfficiencyValue(
    ReferenceLineInfo* const reference_line_info, const double check_l) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  double far_from_lane_center =
      (check_l / config.keep_lane_center_lateral_ref_shift()) *
      (check_l / config.keep_lane_center_lateral_ref_shift());
  double far_from_adc_center =
      (check_l - adc_center_l) * (check_l - adc_center_l);
  return far_from_lane_center *
             config.far_from_lane_center_efficiency_weight() +
         far_from_adc_center * config.far_from_adc_center_efficiency_weight();
}

bool RiskShiftDecider::DisableRiskShiftInPlayStreet() {
  const auto& config = config_.risk_shift_decider_config();
  return injector_->is_in_play_street &&
         !config.enable_risk_shift_in_play_street();
}

int RiskShiftDecider::GetReverseCheckTimeBaseOnDistance(
    ReferenceLineInfo* const reference_line_info, std::string id) {
  const auto* obs_info = reference_line_info->path_decision()->Find(id);
  if (!obs_info) {
    return kDefaultReverseCheckTimes;
  }
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obs_info->PerceptionSLBoundary();
  double diff_dis = obs_sl.start_s() - adc_sl.end_s();
  if (diff_dis < config.min_check_turn_on_distance()) {
    ADEBUG << "[CHECK TIMES] obs: " << obs_info->Id() << ", check times: "
           << config.min_check_keep_exist_reverse_obstacle_time()
           << ", dis: " << diff_dis;
    return config.min_check_keep_exist_reverse_obstacle_time();
  }
  if (diff_dis > config.max_check_turn_on_distance()) {
    ADEBUG << "[CHECK TIMES] obs: " << obs_info->Id() << ", check times: "
           << config.max_check_keep_exist_reverse_obstacle_time()
           << ", dis: " << diff_dis;
    return config.max_check_keep_exist_reverse_obstacle_time();
  }

  double numerator =
      1.0 * (config.max_check_keep_exist_reverse_obstacle_time() -
             config.min_check_keep_exist_reverse_obstacle_time());
  double denominator = (config.max_check_turn_on_distance() -
                        config.min_check_turn_on_distance());
  double multiplier = (diff_dis - config.min_check_turn_on_distance());
  double res = (multiplier * numerator / std::fmax(denominator, kEpsilon)) +
               config.min_check_keep_exist_reverse_obstacle_time();

  ADEBUG << "[CHECK TIMES] obs: " << obs_info->Id()
         << ", check times: " << round(res) << ", dis: " << diff_dis;
  return round(res);
}

void RiskShiftDecider::EfficiencyShiftBypass(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_is_in_efficiency_bypass_state(false);
  // if (!config_.risk_shift_decider_config().enable_efficiency_shift_bypass())
  // {
  //   return;
  // }
  if (!FLAGS_enable_efficiency_shift_borrow) {
    return;
  }
  std::vector<std::string> slow_obs;
  bool has_slow_obs =
      CheckFrontSlowBlockObstacle(reference_line_info, &slow_obs);
  ADEBUG << "has_slow_obs: " << has_slow_obs
         << ", slow obs size: " << slow_obs.size();
  UpdateEfficiencyBypassState(has_slow_obs);
  SaveSlowObstacleInfo(reference_line_info, slow_obs);
  mutable_path_decider_status->set_is_in_efficiency_bypass_state(
      is_in_efficiency_bypass_state_);
}

bool RiskShiftDecider::CheckFrontSlowBlockObstacle(
    ReferenceLineInfo* const reference_line_info,
    std::vector<std::string>* const slow_obs) {
  if (CheckIsNearJunction()) {
    return false;
  }
  const auto& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  has_pedestrian_obs_ = false;
  has_sidebyside_obs_ = false;

  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (nullptr == obstacle || obstacle->IsVirtual()) {
      continue;
    }
    if (obstacle->speed() < FLAGS_static_obstacle_speed_threshold &&
        obstacle->IsStatic()) {
      continue;
    }
    if (IrrelevantObstacle(reference_line_info, obstacle)) {
      continue;
    }
    if (CheckIsCrossing(reference_line_info, obstacle)) {
      continue;
    }
    if (HasObsSideBySide(reference_line_info, obstacle)) {
      slow_obs->emplace_back(obstacle->Id());
      continue;
    }
    if (ObstacleIsNotSlow(reference_line_info, obstacle)) {
      continue;
    }
    if (ObstacleIsTooFar(reference_line_info, obstacle)) {
      continue;
    }
    if (FrontHasVehicle(reference_line_info, obstacle)) {
      return false;
    }
    if (NoNeedByPass(reference_line_info, obstacle)) {
      continue;
    }
    if (CheckObstacleType(obstacle)) {
      slow_obs->emplace_back(obstacle->Id());
    }
  }
  return !slow_obs->empty();
}

bool RiskShiftDecider::CheckIsNearJunction() {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  if (path_decider_status.is_adc_near_junction() ||
      path_decider_status.is_obs_near_junction() ||
      injector_->adc_in_junction_info_.first) {
    ADEBUG << "Near or In junction, not efficiency borrow.";
    return true;
  }
  return false;
}

bool RiskShiftDecider::IrrelevantObstacle(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  if (obs_sl.end_s() < adc_sl.start_s() || obs_sl.start_l() > adc_sl.end_l()) {
    return true;
  }
  return false;
}

bool RiskShiftDecider::CheckIsCrossing(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  double obs_heading = obstacle->PerceptionBoundingBox().heading();
  if (obstacle->HasTrajectory()) {
    obs_heading =
        obstacle->Trajectory().trajectory_point(0).path_point().theta();
  }
  double ref_heading = reference_line_info->reference_line()
                           .GetReferencePoint(obs_sl.start_s())
                           .heading();
  double heading_diff =
      std::fabs(century::common::math::AngleDiff(obs_heading, ref_heading));
  if (heading_diff > M_PI_4 && heading_diff < M_PI_4 * 3.0) {
    return true;
  }

  util::MovingObstacleType moving_obstacle_type = util::GetMovingObstacleType(
      obstacle, reference_line_info->vehicle_state(),
      reference_line_info->reference_line());
  if (util::STRAIGHT_FORWARD != moving_obstacle_type &&
      util::NO_MOVING != moving_obstacle_type) {
    return true;
  }
  return false;
}

bool RiskShiftDecider::HasObsSideBySide(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  double obs_center_l = ((obs_sl.start_l() + obs_sl.end_l()) * 0.5);
  const auto& lane_width = reference_line_info->GetLaneWidthBaseOnAdcCenter();
  if (obs_sl.end_s() > adc_sl.start_s() && obs_sl.start_s() < adc_sl.end_s() &&
      obs_center_l > -lane_width.second && obs_sl.end_l() < adc_sl.start_l()) {
    has_sidebyside_obs_ = true;
    return true;
  }
  return false;
}

bool RiskShiftDecider::ObstacleIsNotSlow(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_s = 0.5 * (adc_sl.start_s() + adc_sl.end_s());
  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(adc_center_s);
  double speed_buffer =
      is_in_efficiency_bypass_state_ ? kEfficiencyCheckVelocityBuffer : 0.0;
  double adc_v =
      std::fabs(reference_line_info->vehicle_state().linear_velocity());

  if (obstacle->speed() >
          (speed_limit * config.efficiency_shift_obs_speed_limit_coef() +
           speed_buffer) ||
      obstacle->speed() > config.obs_faster_than_adc_ratio() * adc_v) {
    ADEBUG << "[Efficiency Check] obs: " << obstacle->Id()
           << " is fast, obs_v: " << obstacle->speed();
    return true;
  }

  return false;
}

bool RiskShiftDecider::ObstacleIsTooFar(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  double distance = obs_sl.start_s() - adc_sl.end_s();
  if (distance > config.max_efficiency_bypass_distance_check_threshold()) {
    ADEBUG << "[Efficiency Check] obs: " << obstacle->Id()
           << " is far from adc, distance: " << distance;
    return true;
  }
  double adc_max_v = reference_line_info->IsAuxiliaryRoad()
                         ? FLAGS_auxiliary_road_limit_speed
                         : FLAGS_planning_upper_speed_limit;
  double diff_v = adc_max_v - obstacle->speed();

  if (distance > diff_v * config.efficidency_check_ttc() &&
      distance > adc_max_v * config.efficidency_check_hwt()) {
    ADEBUG << "[Efficiency Check] obs: " << obstacle->Id()
           << " is far from adc, diff_v: " << diff_v;
    return true;
  }
  return false;
}

bool RiskShiftDecider::FrontHasVehicle(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  if (PerceptionObstacle::VEHICLE != obstacle->Perception().type()) {
    return false;
  }

  const auto& lane_width = reference_line_info->GetLaneWidthBaseOnAdcCenter();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& vehicle_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();
  if (obs_sl.start_l() < lane_width.first &&
      obs_sl.end_l() > (-vehicle_width * 0.5) &&
      obs_sl.start_s() > adc_sl.end_s()) {
    ADEBUG << "[Vehicle Check] has front slow vehicle obs: " << obstacle->Id();
    return true;
  }
  return false;
}

bool RiskShiftDecider::NoNeedByPass(
    ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& adc_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();
  const auto& lane_width = reference_line_info->GetLaneWidthBaseOnAdcCenter();

  SLPoint obs_center_p;
  obs_center_p.set_s((obs_sl.start_s() + obs_sl.end_s()) * 0.5);
  obs_center_p.set_l((obs_sl.start_l() + obs_sl.end_l()) * 0.5);

  if (util::IsLaneBorrow(injector_->planning_context()) &&
      !reference_line_info->IsAdcLocatedInLane()) {
    if (!reference_line_info->reference_line().IsOnLane(obs_center_p)) {
      ADEBUG << "[NoNeedByPass] in lane borrow, no bypass <ON_LANE> obs: "
             << obstacle->Id();
      return true;
    }
    return false;
  }
  if (obs_sl.start_l() > adc_width * 0.5 + kEfficiencyLateralSafeBuffer ||
      obs_sl.end_l() < -adc_width * 0.5 - kEfficiencyLateralSafeBuffer) {
    ADEBUG << "[NoNeedByPass] no bypass <NO_Block_ADC> obs: " << obstacle->Id();
    return true;
  }
  if (obs_sl.end_l() > lane_width.first) {
    ADEBUG << "[NoNeedByPass] no bypass <Step_On_Line> obs: " << obstacle->Id();
    return true;
  }
  if (obs_sl.end_l() > adc_sl.end_l()) {
    ADEBUG << "[NoNeedByPass] no bypass <Left_Side> obs: " << obstacle->Id();
    return true;
  }
  if (!injector_->is_in_play_street &&
      !config.enable_radical_efficiency_shift_public_road()) {
    if (obs_sl.end_l() <
        lane_width.first - adc_width * 0.5 - kEfficiencyLateralSafeBuffer) {
      ADEBUG << "[NeedByPass] public road, need bypass <Right_Side> obs: "
             << obstacle->Id();
      return false;
    }
    if (PerceptionObstacle::PEDESTRIAN == obstacle->Perception().type()) {
      ADEBUG << "[NeedByPass] public road, need bypass <PEDESTRIAN> obs: "
             << obstacle->Id();
      return false;
    }
    if (obstacle->speed() < FLAGS_play_street_speed_limit) {
      ADEBUG << "[NeedByPass] public road, need bypass <TooSlow> obs: "
             << obstacle->Id();
      return false;
    }
    ADEBUG << "[NoNeedByPass] public road, no bypass <Block> obs: "
           << obstacle->Id();
    return true;
  }
  return false;
}

bool RiskShiftDecider::CheckObstacleType(const Obstacle* obstacle) {
  const auto& config = config_.risk_shift_decider_config();
  if (PerceptionObstacle::PEDESTRIAN == obstacle->Perception().type()) {
    has_pedestrian_obs_ = true;
  }
  if (PerceptionObstacle::BICYCLE == obstacle->Perception().type() ||
      PerceptionObstacle::PEDESTRIAN == obstacle->Perception().type() ||
      obstacle->IsSmallUnknown(config.small_unknown_check_width())) {
    return true;
  }
  ADEBUG << "[Type Check] obs: " << obstacle->Id()
         << " type: " << static_cast<int>(obstacle->Perception().type())
         << " not support bypass";
  return false;
}

void RiskShiftDecider::UpdateEfficiencyBypassState(bool has_slow_obs) {
  const auto& config = config_.risk_shift_decider_config();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  int count =
      mutable_path_decider_status->efficiency_bypass_slow_obs_keep_times();
  if (is_in_efficiency_bypass_state_) {
    if (has_slow_obs) {
      count = 0;
      mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(
          count);
    } else {
      mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(
          --count);
    }
    if (count < -config.quit_efficiency_bypass_state_count_threshold()) {
      is_in_efficiency_bypass_state_ = false;
      mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(0);
      AINFO << "[Efficiency State]: TURN OFF!";
    }
    return;
  }

  if (has_slow_obs) {
    mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(
        ++count);
  } else {
    mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(
        std::max(0, --count));
  }

  auto check_times =
      (injector_->is_in_play_street || has_pedestrian_obs_)
          ? config.check_keep_exist_slow_obstacle_time_in_playstreet()
          : config.check_keep_exist_slow_obstacle_time();
  if (util::IsLaneBorrow(injector_->planning_context())) {
    check_times = config.check_keep_exist_slow_obstacle_time_in_laneborrow();
  }

  ADEBUG << "not in bypass, has_pedestrian_obs_: " << has_pedestrian_obs_
         << ", count: " << count << ", check_times: " << check_times;

  if (count > check_times || has_sidebyside_obs_) {
    is_in_efficiency_bypass_state_ = true;
    mutable_path_decider_status->set_efficiency_bypass_slow_obs_keep_times(0);
    AINFO << "[Efficiency State]: TURN ON!";
  }
}

void RiskShiftDecider::SaveSlowObstacleInfo(
    ReferenceLineInfo* const reference_line_info,
    const std::vector<std::string> slow_obs) {
  if (!is_in_efficiency_bypass_state_) {
    return;
  }
  for (const auto& id : slow_obs) {
    auto* obs_info = reference_line_info->path_decision()->Find(id);
    if (obs_info) {
      obs_info->SetSlowCanPass(true);
      AINFO << "[Efficiency Borrow] obs: " << id << " slow can bypass";
    }
  }
  return;
}

}  // namespace planning
}  // namespace century
