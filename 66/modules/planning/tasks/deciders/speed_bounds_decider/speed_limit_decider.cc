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

/**
 * @file
 **/

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

#include <algorithm>
#include <limits>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

using century::common::SpeedPoint;
using century::common::Status;
using century::common::math::Vec2d;

namespace {
constexpr double kSpeedBuffer = 2.0;               // in meters per second
constexpr double kFirstPathDistanceBuffer = 35.0;  // m
constexpr double kDegrees = 90.0;
constexpr double kLateralBuffer = 0.5;
constexpr double kPredictionTime = 8.0;
constexpr uint32_t kMinPathStep = 1;
constexpr double kEpison = 1e-5;
constexpr double kSpeedLimitDistance = 5.0;
constexpr double kMinSpeedLimitDistance = 20.0;
constexpr double kMaxSpeedLimitDistance = 50.0;
constexpr double kMinSpeedLimit = 3.0;
constexpr double kSpeedCoeff = 0.8;
constexpr double kOvertakeBuffer = 5.0;
constexpr double kYieldBuffer = 5.0;
constexpr double kMinSpeed = 1.0;
constexpr double kHalfReserveAngle = 30;
constexpr double kSqrtTwo = std::sqrt(2.0);
constexpr double kMinConsiderLength = 10.0;
constexpr double kMaxConsiderLength = 40.0;
constexpr double kSpeedLimitForBorrow = 2.5;
constexpr double kSpeedLimitForBorrowStraightPath = 2.5;
constexpr double kSpeedLimitForReturn = 2.5;
constexpr double kMinDiffL = 0.3;
constexpr double kLimitMoveSpeed = 2.0;
constexpr double kLimitTinyAdjustSpeed = 0.6;
}  // namespace

HysteresisInterval SpeedLimitDecider::obstacles_interval_(
    FLAGS_obs_cross_angle_degree / kDegrees * M_PI_2,
    FLAGS_hy_buffer_of_cross_angle / kDegrees * M_PI_2, 20UL);

SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     ReferenceLineInfo* reference_line_info,
                                     const PlanningContext* context)
    : speed_bounds_config_(config),
      speed_limit_config_(config.speed_limit_config()),
      reference_line_info_(reference_line_info),
      planning_context_(context),
      need_accelerate_for_overtake_(
          context->planning_status().overtake().accelerated_mode()) {
  accel_ratio_for_overtake_ =
      (FLAGS_enable_overtake_speed_up && need_accelerate_for_overtake_)
          ? FLAGS_overtake_speed_up_ratio
          : 1.0;
}

Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  bool is_need_return_lane = IsNeedReturnlane();

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  uint32_t path_step =
      speed_limit_config_.closer_path_step_for_speed_limit();  // 2m
  double max_kappa_fabs = reference_line_info_->GetMaxKappaFabs();
  bool is_need_to_speed_limit_for_diagonal = false;
    for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
      auto discretized_point = discretized_path[i];
      // AINFO<<"discretized_point = "<<discretized_point.kappa();
      if(std::fabs(discretized_point.kappa()) > 0.01){
        // AINFO<<"NEED  SPEED LIMIT FOR DIAGONAL";
        is_need_to_speed_limit_for_diagonal = true;
        break;
      }
    }
  for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
    const double path_s = discretized_path[i].s();
    const double reference_line_s = frenet_path[i].s();
    if (reference_line_s > reference_line_info_->reference_line().Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_info_->reference_line().Length()
            << "]. Please debug before proceeding.";
      break;
    }

    if (path_s > kFirstPathDistanceBuffer) {
      path_step =
          speed_limit_config_.far_away_path_step_for_speed_limit();  // 4m
    }

    // (1) speed limit from map
    double speed_limit_from_reference_line =
        GetSpeedLimitFromReferenceLine(reference_line_s);
    ADEBUG << "path_s(" << path_s << ") speed_limit_from_reference_line: "
           << speed_limit_from_reference_line;

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    double kappa_temp =
        std::fabs(discretized_path[i].kappa()) >
                speed_limit_config_.min_speed_limit_kappa_threshold()
            ? max_kappa_fabs
            : fabs(discretized_path[i].kappa());
    double speed_limit_from_centripetal_acc =
        FLAGS_enable_speed_limit_max_kappa
            ? GetSpeedLimitFromCentripetalAcc(kappa_temp)
            : GetSpeedLimitFromCentripetalAcc(discretized_path[i].kappa());
    const auto& candidate_path_data =
        reference_line_info_->GetCandidatePathData();
    bool is_left_borrow =
        (std::string::npos !=
             candidate_path_data.front().path_label().find("left") &&
         std::string::npos !=
             candidate_path_data.front().path_label().find("regular"));
    bool is_right_borrow =
        (std::string::npos !=
             candidate_path_data.front().path_label().find("right") &&
         std::string::npos !=
             candidate_path_data.front().path_label().find("regular"));
    if ((is_right_borrow || is_left_borrow) && is_need_to_speed_limit_for_diagonal) {
      speed_limit_from_centripetal_acc =
          std::min(speed_limit_from_centripetal_acc, kSpeedLimitForBorrow);
    }else if((is_right_borrow || is_left_borrow) && !is_need_to_speed_limit_for_diagonal){
      speed_limit_from_centripetal_acc =
          std::min(speed_limit_from_centripetal_acc, kSpeedLimitForBorrowStraightPath);
      if(is_need_return_lane){
        // AINFO<<"need speed limit for return";
      speed_limit_from_centripetal_acc =
          std::min(speed_limit_from_centripetal_acc, kSpeedLimitForReturn);
      }
    }
    ADEBUG << "path_s(" << path_s << ") speed_limit_from_centripetal_acc: "
           << speed_limit_from_centripetal_acc;

    // (3) speed limit from obstacles
    double speed_limit_from_obstacles = GetSpeedLimitFromObstacles(
        obstacles, discretized_path[i], frenet_path[i],
        speed_limit_from_reference_line);
    ADEBUG << "path_s(" << path_s
           << ") speed_limit_from_obstacles: " << speed_limit_from_obstacles;

    // (4) speed limit from keep approaching obstacles.
    double speed_limit_from_approaching_obstacles =
        GetSpeedLimitFromApproachingObstacles(
            reference_line_info_->GetKeepApproachingObstacles(),
            discretized_path[i], speed_limit_from_reference_line);
    ADEBUG << "path_s(" << path_s
           << ") speed_limit_from_approaching_obstacles: "
           << speed_limit_from_approaching_obstacles;

    double curr_speed_limit = std::max(
        speed_limit_config_.lowest_speed(),
        std::min({speed_limit_from_reference_line,
                  speed_limit_from_centripetal_acc, speed_limit_from_obstacles,
                  speed_limit_from_approaching_obstacles}));

    if (FLAGS_enable_use_radical_decision) {
      curr_speed_limit =
          std::min(curr_speed_limit, FLAGS_play_street_speed_limit);
    }
    // (5) speed limit from diagonal
    if(is_need_to_speed_limit_for_diagonal){
      curr_speed_limit = std::min(curr_speed_limit,FLAGS_diagonal_peed_limit);
    }
    if (reference_line_info_->EnableShrinkCollisionBuffer()) {
      AINFO << "has speed limit";
      curr_speed_limit = std::min(curr_speed_limit, kLimitMoveSpeed);
    }
    if(reference_line_info_->IsTinyAdjustType()){
      curr_speed_limit = std::min(curr_speed_limit,kLimitTinyAdjustSpeed);
    }
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
    ADEBUG << "curr_speed_limit = " << curr_speed_limit;
  }
  reference_line_info_->ReleaseKeepApproachingObstacles();

  return Status::OK();
}

bool  SpeedLimitDecider::IsNeedReturnlane() const{
  double request_end_state_l = reference_line_info_->RequestEndL();
    const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_center_l =
      (adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5;
  // double ego_start_s = adc_sl_boundary.start_s();
  // double ego_end_s = adc_sl_boundary.end_s();
  // double ego_v =
  //     std::fabs(reference_line_info_->vehicle_state().linear_velocity());
  // const auto& veh_param =
  //     common::VehicleConfigHelper::GetConfig().vehicle_param();
  // double adc_half_width = veh_param.width() * 0.5;
  if (std::fabs(request_end_state_l - adc_center_l) < kMinDiffL) {
    return false;
  }
  // AINFO<<"request_end_state_l = "<<request_end_state_l;
  // AINFO<<"adc_center_l = "<<adc_center_l;

  bool is_left_self_borrow_or_right_return =
      (request_end_state_l > adc_center_l);
  bool is_right_self_borrow_or_left_return =
      (request_end_state_l < adc_center_l);
    const auto& candidate_path_data =
        reference_line_info_->GetCandidatePathData();
    bool is_in_left_borrow =
        (std::string::npos !=
             candidate_path_data.front().path_label().find("left") &&
         std::string::npos !=
             candidate_path_data.front().path_label().find("regular"));
    bool is_in_right_borrow =
        (std::string::npos !=
             candidate_path_data.front().path_label().find("right") &&
         std::string::npos !=
             candidate_path_data.front().path_label().find("regular"));
  if((is_left_self_borrow_or_right_return && is_in_right_borrow) || 
    (is_right_self_borrow_or_left_return && is_in_left_borrow)){
    return true;
  }
  return false;

}

double SpeedLimitDecider::GetFollowSpeedLimit(const SLBoundary& adc_sl_boundary,
                                              const Obstacle* obstacle) const {
  const auto& obs_sl_boundary = obstacle->PerceptionSLBoundary();
  double follow_speed_limit = FLAGS_planning_upper_speed_limit;
  double diff_s = obs_sl_boundary.start_s() - adc_sl_boundary.end_s();
  if (obstacle->IsStatic()) {
    bool is_overlap_with_static_obs =
        obs_sl_boundary.end_l() > obs_sl_boundary.start_l() - kLateralBuffer &&
        obs_sl_boundary.start_l() < obs_sl_boundary.end_l() + kLateralBuffer;
    if (!is_overlap_with_static_obs) {
      return FLAGS_planning_upper_speed_limit;
    }
    follow_speed_limit = InterpolationLookUp(
        diff_s, speed_limit_config_.min_speed_limit_longitude_distance(),
        speed_limit_config_.max_speed_limit_longitude_distance(),
        speed_limit_config_.min_follow_speed_limit(),
        FLAGS_planning_upper_speed_limit);
  } else {
    // for yield follow stop decision.
  }
  return follow_speed_limit;
}

Status SpeedLimitDecider::GetHighSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  uint32_t path_step = kMinPathStep;  // 1m

  double max_kappa_fabs = reference_line_info_->GetMaxKappaFabs();
  for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
    const double path_s = discretized_path[i].s();
    const double reference_line_s = frenet_path[i].s();
    if (reference_line_s > reference_line_info_->reference_line().Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_info_->reference_line().Length()
            << "]. Please debug before proceeding.";
      break;
    }

    // (1) speed limit from map
    double speed_limit_from_reference_line =
        GetSpeedLimitFromReferenceLine(reference_line_s);

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    double kappa_temp =
        std::fabs(discretized_path[i].kappa()) >
                speed_limit_config_.min_speed_limit_kappa_threshold()
            ? max_kappa_fabs
            : fabs(discretized_path[i].kappa());
    double speed_limit_from_centripetal_acc =
        FLAGS_enable_speed_limit_max_kappa
            ? GetSpeedLimitFromCentripetalAcc(kappa_temp)
            : GetSpeedLimitFromCentripetalAcc(discretized_path[i].kappa());

    // (3) speed limit from nudge obstacles
    // Get ADC boundary from frenet_point
    SLBoundary adc_sl_boundary_path_point;
    bool adc_sl_boundary_result = reference_line_info_->AdcSlBoundaryAt(
        discretized_path[i], &adc_sl_boundary_path_point);
    if (!adc_sl_boundary_result) {
      AERROR << "Failed to get ADC boundary from frenet_point: "
             << discretized_path[i].DebugString();
    }
    double speed_limit_from_nudge_obstacles =
        GetSpeedLimitFromNudgeObstaclesForHighSpeed(
            obstacles, adc_sl_boundary_path_point, adc_sl_boundary_result);
    // (4) speed limit from pedestrian

    double speed_limit_from_pedestrian =
        GetSpeedLimitFromPedestrianForHighSpeed(
            obstacles, adc_sl_boundary_path_point, discretized_path[i],
            adc_sl_boundary_result);

    double curr_speed_limit = 0.0;
    curr_speed_limit = std::fmax(
        speed_limit_config_.lowest_speed(),
        std::min({speed_limit_from_reference_line,
                  speed_limit_from_centripetal_acc,
                  FLAGS_enable_nudge_slowdown ? speed_limit_from_nudge_obstacles
                                              : speed_limit_from_reference_line,
                  speed_limit_from_pedestrian}));

    if (FLAGS_enable_use_radical_decision) {
      curr_speed_limit =
          std::min(curr_speed_limit, FLAGS_play_street_speed_limit);
    }

    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
    ADEBUG << "speed_limit_from_centripetal_acc = "
           << speed_limit_from_centripetal_acc;
    ADEBUG << "curr_speed_limit = " << curr_speed_limit;
  }

  return Status::OK();
}

Status SpeedLimitDecider::UpdateSpeedLimitsForStopDecision(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  uint32_t path_step = kMinPathStep;
  if (FLAGS_enable_speed_limit_for_obstacle) {
    bool need_to_slow_down_for_stop_decision = false;
    double min_stop_s = std::numeric_limits<double>::max();
    // The length of the path cannot be used here because the length of the path
    // may fluctuate.
    double max_speed_limit_distance = kMaxSpeedLimitDistance;
    for (const auto* obstacle : obstacles.Items()) {
      const auto& decision = obstacle->LongitudinalDecision();
      const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
      const auto& obs_start_s = obstacle_sl.start_s();
      if (!decision.has_stop()) {
        continue;
      }
      // only conside stop decision in front of adc.
      if (adc_sl_boundary.end_s() > min_stop_s) {
        continue;
      }
      double stop_s = obs_start_s + decision.stop().distance_s();
      if (stop_s - adc_sl_boundary.end_s() > max_speed_limit_distance) {
        continue;
      }
      if (stop_s < min_stop_s) {
        min_stop_s = stop_s;
        need_to_slow_down_for_stop_decision = true;
      }
    }
    ADEBUG << "min_stop_s = " << min_stop_s;

    double speed_limit_stop_obs = FLAGS_planning_upper_speed_limit;

    // speed limit between adc and stop point.
    if (need_to_slow_down_for_stop_decision) {
      for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
        const double path_s = discretized_path.at(i).s();
        double distance = min_stop_s - adc_sl_boundary.end_s() - path_s;
        ADEBUG << "distance=" << distance;
        if (i > speed_limit_data->speed_limit_points().size() - 1) {
          break;
        }
        // The linear interpolation here cannot use the maximum speed limit
        // because our goal is to slow down, so the target speed needs to be
        // relatively small, otherwise the previous speed limit is almost
        // unlimited
        speed_limit_stop_obs = InterpolationLookUp(
            distance, kMinSpeedLimitDistance, max_speed_limit_distance,
            kMinSpeedLimit,
            FLAGS_planning_upper_speed_limit);
        ADEBUG << "speed_limit_stop_obs = " << speed_limit_stop_obs;
        double v_upper_bound = FLAGS_planning_upper_speed_limit;
        v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
        ADEBUG << "ori speed limit = " << v_upper_bound;
        v_upper_bound = std::min(v_upper_bound, speed_limit_stop_obs);
        speed_limit_data->SetSpeedLimitByS(path_s, v_upper_bound);
      }
    }
  }
  return Status::OK();
}

Status SpeedLimitDecider::UpdateSpeedLimitsForDecision(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  UpdateSpeedLimitsForStopYieldFollowDecision(obstacles, speed_limit_data);
  UpdateSpeedLimitsForSlowdownObstacle(obstacles, speed_limit_data);
  UpdateSpeedLimitsForCrossObstacle(obstacles, speed_limit_data);
  UpdateSpeedLimitsForReverseObstacle(obstacles, speed_limit_data);

  return Status::OK();
}

Status SpeedLimitDecider::UpdateSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  for (const auto* slowdown_obstacle : obstacles.Items()) {
    if (nullptr == slowdown_obstacle) {
      continue;
    }
    // only for yield or follow
    bool is_follow_obs = false;
    bool is_yield_obs = false;
    if (slowdown_obstacle->LongitudinalDecision().has_follow()) {
      is_follow_obs = true;
    } else if (slowdown_obstacle->LongitudinalDecision().has_yield()) {
      is_yield_obs = true;
    } else {
      continue;
    }
    ADEBUG << "obs_decision = "
           << slowdown_obstacle->LongitudinalDecision().DebugString();

    const auto& obs_sl = slowdown_obstacle->PerceptionSLBoundary();
    ADEBUG << "obs_sl.start_s() = " << obs_sl.start_s();
    ADEBUG << "adc_sl_boundary.end_s() = " << adc_sl_boundary.end_s();
    double limit_end_s = obs_sl.start_s() - adc_sl_boundary.end_s();
    double min_limit_speed = FLAGS_planning_upper_speed_limit;
    if (is_follow_obs) {
      limit_end_s =
          limit_end_s +
          slowdown_obstacle->LongitudinalDecision().follow().distance_s();
      min_limit_speed = slowdown_obstacle->speed();
    } else if (is_yield_obs) {
      limit_end_s =
          limit_end_s +
          slowdown_obstacle->LongitudinalDecision().yield().distance_s();
      min_limit_speed = speed_limit_config_.min_yield_speed_limit();
    }
    ADEBUG << "limit_end_s = " << limit_end_s;
    // obstacles only in front of ADC.

    if (limit_end_s < 0.0) {
      continue;
    }
    uint32_t path_step = kMinPathStep;
    for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
      const double path_s = discretized_path.at(i).s();
      double v_upper_bound = FLAGS_planning_upper_speed_limit;
      ADEBUG << "path_s = " << path_s;
      if (i > speed_limit_data->speed_limit_points().size() - 1) {
        break;
      }
      // the excess is calculated based on the minimum yield speed or following
      // speed.
      if (path_s > limit_end_s) {
        v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);

        v_upper_bound = std::min(v_upper_bound, min_limit_speed);
        speed_limit_data->SetSpeedLimitByIndex(i, path_s, v_upper_bound);
        ADEBUG << "after speed limit = " << v_upper_bound;
      } else {
        // the following distance has already been subtracted,so use 0.0
        double limit_speed = InterpolationLookUp(
            limit_end_s - path_s, 0.0,
            speed_limit_config_.max_speed_limit_longitude_distance(),
            min_limit_speed, FLAGS_planning_upper_speed_limit);
        v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
        ADEBUG << "ori speed limit = " << v_upper_bound;
        v_upper_bound = std::min(v_upper_bound, limit_speed);
        speed_limit_data->SetSpeedLimitByIndex(i, path_s, v_upper_bound);
        ADEBUG << "after speed limit = " << v_upper_bound;
      }
    }
  }

  return Status::OK();
}

double SpeedLimitDecider::GetSpeedLimitFromReferenceLine(
    const double& reference_line_s) const {
  double speed_limit_from_reference_line =
      std::min(reference_line_info_->reference_line().GetSpeedLimitFromS(
                   reference_line_s) *
                   accel_ratio_for_overtake_,
               FLAGS_overtake_upper_speed_limit);
  if (reference_line_info_->IsAuxiliaryRoad()) {
    // AINFO << "in aux road, set max_speed_limit to 5.56m";
    speed_limit_from_reference_line = std::min(
        speed_limit_from_reference_line, FLAGS_auxiliary_road_limit_speed);
  }
  if (FLAGS_enable_high_speed) {
    speed_limit_from_reference_line = FLAGS_planning_upper_speed_limit;
  }
  return speed_limit_from_reference_line;
}

double SpeedLimitDecider::GetSpeedLimitFromCentripetalAcc(
    const double& kappa) const {
  double speed_limit_from_centripetal_acc = FLAGS_planning_upper_speed_limit;
  if (hdmap::Lane::CITY_DRIVING == reference_line_info_->GetLaneType()) {
    speed_limit_from_centripetal_acc = CalcCentripetalAccSpeedLimit(
        kappa, speed_limit_config_.max_centric_acceleration_limit_road());
    if (!FLAGS_enable_high_speed && FLAGS_enable_overtake_speed_up &&
        need_accelerate_for_overtake_) {
      speed_limit_from_centripetal_acc = FLAGS_overtake_upper_speed_limit;
    }
  } else {
    speed_limit_from_centripetal_acc = CalcCentripetalAccSpeedLimit(
        kappa,
        speed_limit_config_.max_centric_acceleration_limit_play_street());
  }

  return speed_limit_from_centripetal_acc;
}

double SpeedLimitDecider::CalcCentripetalAccSpeedLimit(
    const double& kappa, const double max_centric_acceleration_limit) const {
  double speed_limit_from_centripetal_acc = std::sqrt(
      max_centric_acceleration_limit /
      std::fmax(std::fabs(kappa), speed_limit_config_.minimal_kappa()));
  // if start up in low speed limit will no start up.
  speed_limit_from_centripetal_acc = std::max(
      speed_limit_from_centripetal_acc, speed_limit_config_.min_speed_limit());
  if (reference_line_info_->vehicle_state().linear_velocity() < kMinSpeed) {
    speed_limit_from_centripetal_acc =
        std::max(speed_limit_from_centripetal_acc,
                 speed_limit_config_.lower_speed_limit());
  }

  return speed_limit_from_centripetal_acc;
}

double SpeedLimitDecider::SetSpeedLimitForCautionObs(
    const Obstacle* ptr_obstacle, const SLBoundary& path_point_sl) const {
  const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
  double speed_limit_from_caution_obs = std::numeric_limits<double>::max();
  if (ptr_obstacle->IsCaution()) {
    reference_line_info_->SetIsLowStartUp(true);
    if (obs_sl.start_s() - speed_limit_config_.distance_behind_caution_obs() <
            path_point_sl.end_s() &&
        (obs_sl.end_s() + obs_sl.start_s()) * 0.5 > path_point_sl.end_s()) {
      speed_limit_from_caution_obs =
          speed_limit_config_.min_speed_limit_for_caution_obstacle();
    }
  }
  return speed_limit_from_caution_obs;
}

double SpeedLimitDecider::GetSpeedLimitFromObstacles(
    const IndexedList<std::string, Obstacle>& obstacles,
    const common::PathPoint& path_point,
    const common::FrenetFramePoint& frenet_point,
    const double speed_limit_from_reference_line) const {
  double speed_limit_from_obstacles =
      std::min(FLAGS_planning_upper_speed_limit * accel_ratio_for_overtake_,
               FLAGS_overtake_upper_speed_limit);

  // Get ADC boundary from frenet_point
  SLBoundary path_point_sl;
  bool sl_result =
      reference_line_info_->AdcSlBoundaryAt(path_point, &path_point_sl);
  if (!sl_result) {
    AERROR << "Failed to get ADC boundary from frenet_point: "
           << path_point.DebugString();
    return speed_limit_from_obstacles;
  }
  const double reference_line_s = frenet_point.s();
  const double path_point_l =
      0.5 * (path_point_sl.start_l() + path_point_sl.end_l());
  const double path_point_heading = path_point.theta();

  for (const auto* ptr_obstacle : obstacles.Items()) {
    if (ptr_obstacle->IsVirtual()) {
      continue;
    }
    double speed_limit_from_nudge_obstacle = std::numeric_limits<double>::max();
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    if (FLAGS_enable_nudge_slowdown &&
        ptr_obstacle->LateralDecision().has_nudge()) {
      speed_limit_from_nudge_obstacle = GetSpeedLimitFromNudgeObstacles(
          *ptr_obstacle, path_point_sl, speed_limit_from_reference_line);
    }

    double speed_limit_from_pedestrian = std::numeric_limits<double>::max();
    if (FLAGS_enable_pedestrian_slowdown &&
        hdmap::Lane::PLAY_STREET != reference_line_info_->GetLaneType() &&
        perception::PerceptionObstacle::PEDESTRIAN ==
            ptr_obstacle->Perception().type()) {
      speed_limit_from_pedestrian = GetSpeedLimitFromPedestrian(
          *ptr_obstacle, path_point_l, path_point_heading, reference_line_s);
    }

    double speed_limit_from_bicycle = std::numeric_limits<double>::max();
    if (FLAGS_enable_bicycle_slowdown &&
        hdmap::Lane::PLAY_STREET != reference_line_info_->GetLaneType() &&
        perception::PerceptionObstacle::BICYCLE ==
            ptr_obstacle->Perception().type() &&
        !ptr_obstacle->IsStatic()) {
      speed_limit_from_bicycle = GetSpeedLimitFromBicycle(
          *ptr_obstacle, path_point_l, path_point_heading, reference_line_s);
    }

    // speed limit for caution obs.
    const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
    double speed_limit_from_caution_obs =
        SetSpeedLimitForCautionObs(ptr_obstacle, path_point_sl);

    // low start up for has front obs
    bool is_in_longitudinal_range =
        obs_sl.start_s() - reference_line_info_->AdcSlBoundary().end_s() <
            speed_limit_config_.max_longitudinal_range_for_low_start_up() &&
        obs_sl.start_s() - reference_line_info_->AdcSlBoundary().end_s() > 0.0;
    bool is_in_lateral_range =
        (obs_sl.start_l() <
             path_point_sl.end_l() +
                 speed_limit_config_.max_lateral_range_for_low_start_up() &&
         obs_sl.end_l() >
             path_point_sl.start_l() -
                 speed_limit_config_.max_lateral_range_for_low_start_up());
    double diff_theta =
        std::fabs(ptr_obstacle->SpeedHeading() -
                  reference_line_info_->vehicle_state().heading());
    double buffer_radian = kHalfReserveAngle / 90.0 * M_PI_2;
    bool is_same_direction =
        std::fabs(century::common::math::NormalizeAngle(diff_theta)) <
        buffer_radian;
    if (is_in_longitudinal_range && is_in_lateral_range && !is_same_direction) {
      reference_line_info_->SetIsLowStartUp(true);
    }

    speed_limit_from_obstacles =
        std::min({speed_limit_from_obstacles, speed_limit_from_pedestrian,
                  speed_limit_from_bicycle, speed_limit_from_nudge_obstacle,
                  speed_limit_from_caution_obs});
  }

  return speed_limit_from_obstacles;
}

double SpeedLimitDecider::GetSpeedLimitFromApproachingObstacles(
    const std::vector<std::string>& approaching_obs_ids,
    const common::PathPoint& path_point,
    const double speed_limit_from_reference_line) const {
  double speed_limit_from_approaching_obstacles =
      std::min(FLAGS_planning_upper_speed_limit * accel_ratio_for_overtake_,
               FLAGS_overtake_upper_speed_limit);

  // Get ADC boundary from frenet_point
  SLBoundary path_point_sl;
  bool sl_result =
      reference_line_info_->AdcSlBoundaryAt(path_point, &path_point_sl);
  if (!sl_result) {
    AERROR << "Failed to get ADC boundary from frenet_point: "
           << path_point.DebugString();
    return speed_limit_from_approaching_obstacles;
  }
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const double path_point_l =
      0.5 * (path_point_sl.start_l() + path_point_sl.end_l());
  const double path_point_heading = path_point.theta();

  for (const auto obs_id : approaching_obs_ids) {
    ADEBUG << "approaching obstacle id: " << obs_id;
    const auto* approaching_obstacle =
        reference_line_info_->path_decision()->Find(obs_id);
    if (nullptr == approaching_obstacle) {
      continue;
    }
    const SLBoundary& obstacle_sl =
        approaching_obstacle->PerceptionSLBoundary();
    if ((path_point_sl.end_s() <
         obstacle_sl.start_s() -
             speed_limit_config_.speed_limit_advance_dis_for_approach_obs()) ||
        (path_point_sl.end_s() >
         obstacle_sl.end_s() -
             speed_limit_config_.speed_limit_backward_dis_for_approach_obs())) {
      continue;
    }
    double speed_limit_for_appro_obs = std::numeric_limits<double>::max();
    double obs_path_l = std::abs(obstacle_sl.end_l() - path_point_l) <
                                std::abs(obstacle_sl.start_l() - path_point_l)
                            ? obstacle_sl.end_l() - path_point_l
                            : obstacle_sl.start_l() - path_point_l;
    const auto& vehicle_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
    double obs_path_abs_l = std::abs(obs_path_l) - vehicle_half_width;

    // if diff l is bigger than diff s, than ignore the obstacle(don't do speed
    // limit).
    ADEBUG << "obs_path_l: " << obs_path_l
           << " obs_path_abs_l: " << obs_path_abs_l
           << " vehicle_half_width: " << vehicle_half_width;
    ADEBUG << "obstacle_sl.end_s(): " << obstacle_sl.end_s()
           << " adc_sl.end_s(): " << adc_sl.end_s();
    if (obs_path_abs_l < 0.0 || obstacle_sl.end_s() < adc_sl.end_s() ||
        obs_path_abs_l > speed_limit_config_.aspect_range_ratio() *
                             (obstacle_sl.end_s() - adc_sl.end_s())) {
      continue;
    }

    const auto& velocity = approaching_obstacle->Perception().velocity();
    const double obs_lat_speed =
        std::abs(common::math::Vec2d::CreateUnitVec2d(path_point_heading)
                     .CrossProd(Vec2d(velocity.x(), velocity.y())));
    ADEBUG << "obs_lat_speed: " << obs_lat_speed;
    if (obs_path_abs_l < 1.0) {
      speed_limit_for_appro_obs = std::max(
          1.0, kSqrtTwo / (kSqrtTwo - 1.0) * std::sqrt(obs_path_abs_l) +
                   (kSqrtTwo - 2.0) / (kSqrtTwo - 1.0));
      ADEBUG << "speed limit for (lat close) appro obs(" << obs_id << ")"
             << speed_limit_for_appro_obs;
    } else {
      double min_dis = speed_limit_config_.min_distance_for_fast_approach_obs();
      double max_dis = speed_limit_config_.max_distance_for_fast_approach_obs();
      double min_limit_v =
          speed_limit_config_.min_speed_limit_ratio_for_approach_obs() *
          FLAGS_planning_upper_speed_limit;
      double max_limit_v = speed_limit_config_.max_speed_limit_ratio() *
                           FLAGS_planning_upper_speed_limit;
      if (obs_lat_speed < speed_limit_config_.lower_approach_obs_speed()) {
        min_dis = speed_limit_config_.min_distance_lower_for_approach_obs();
        max_dis = speed_limit_config_.max_distance_lower_for_approach_obs();
      } else if (obs_lat_speed <
                 speed_limit_config_.upper_approach_obs_speed()) {
        min_dis = speed_limit_config_.min_distance_middle_for_approach_obs();
        max_dis = speed_limit_config_.max_distance_middle_for_approach_obs();
      } else if (obs_lat_speed <
                 speed_bounds_config_.speed_of_faster_move_obstacle()) {
        min_dis = speed_limit_config_.min_distance_upper_for_approach_obs();
        max_dis = speed_limit_config_.max_distance_upper_for_approach_obs();
      }
      speed_limit_for_appro_obs = InterpolationLookUp(
          obs_path_abs_l, min_dis, max_dis, min_limit_v, max_limit_v);
      ADEBUG << "dis = " << obs_path_abs_l
             << ", obs_lat_speed = " << obs_lat_speed;
      ADEBUG << "speed limit for appro obs(" << obs_id << ")"
             << speed_limit_for_appro_obs;
    }
    speed_limit_from_approaching_obstacles = std::min(
        speed_limit_from_approaching_obstacles, speed_limit_for_appro_obs);
  }

  return speed_limit_from_approaching_obstacles;
}

double SpeedLimitDecider::GetSpeedLimitFromNudgeObstacles(
    const Obstacle& obstacle, const SLBoundary& path_point_sl,
    const double speed_limit_from_reference_line) const {
  double speed_limit_from_nudge_obstacle = std::numeric_limits<double>::max();
  /* ref line:
   * -------------------------------
   *    start_s   end_s
   * ------|  adc   |---------------
   * ------------|  obstacle |------
   */
  const SLBoundary& obstacle_sl = obstacle.PerceptionSLBoundary();
  bool has_longitudinal_overlap =
      path_point_sl.end_s() >=
          obstacle_sl.start_s() - speed_limit_config_.obs_nudge_s_threshold() &&
      path_point_sl.start_s() <=
          obstacle_sl.end_s() + speed_limit_config_.obs_nudge_s_threshold();

  if (has_longitudinal_overlap) {
    const auto& nudge_decision = obstacle.LateralDecision().nudge();

    double distance_diff =
        nudge_decision.type() == ObjectNudge::LEFT_NUDGE
            ? std::fabs(path_point_sl.start_l() - obstacle_sl.end_l())
            : std::fabs(path_point_sl.end_l() - obstacle_sl.start_l());
    double nudge_speed_ratio = 1.0;
    // TODO(all): dynamic obstacles do not have nudge decision
    if (obstacle.IsStatic()) {
      nudge_speed_ratio = InterpolationLookUp(
          distance_diff, speed_limit_config_.very_close_nudge_threshold(),
          speed_limit_config_.slightly_close_nudge_threshold(),
          speed_limit_config_.static_obs_nudge_speed_ratio_very(), 1.0);
    } else {
      nudge_speed_ratio = InterpolationLookUp(
          distance_diff, speed_limit_config_.very_close_nudge_threshold(),
          speed_limit_config_.slightly_close_nudge_threshold(),
          speed_limit_config_.dynamic_obs_nudge_speed_ratio_very(), 1.0);
    }
    speed_limit_from_nudge_obstacle =
        nudge_speed_ratio * speed_limit_from_reference_line;
  }
  return speed_limit_from_nudge_obstacle;
}

double SpeedLimitDecider::MakeSpeedLimitForDynamicPedestrian(
    const Obstacle& obstacle, const double path_point_heading,
    const double obs_path_l) const {
  double speed_limit_pedestrian = std::numeric_limits<double>::max();
  const auto& velocity = obstacle.Perception().velocity();
  const auto& acceleration = obstacle.Perception().acceleration();
  const double obs_lat_speed =
      std::abs(common::math::Vec2d::CreateUnitVec2d(path_point_heading)
                   .CrossProd(Vec2d(velocity.x(), velocity.y())));
  const double obs_lat_acc =
      std::abs(common::math::Vec2d::CreateUnitVec2d(path_point_heading)
                   .CrossProd(Vec2d(acceleration.x(), acceleration.y())));
  const auto& near_path_acc = std::max(obs_lat_acc, 0.0);

  ADEBUG << "obs_lat_speed: " << obs_lat_speed
         << ", obs_lat_acc: " << obs_lat_acc;
  const double arrive_speed =
      std::sqrt(obs_lat_speed * obs_lat_speed +
                2.0 * near_path_acc * std::abs(obs_path_l));
  const auto& vehicle_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  double obs_path_abs_l = std::abs(std::abs(obs_path_l) - vehicle_half_width);
  if (obs_path_abs_l < speed_limit_config_.max_dynamic_ped_distance_faster()) {
    double min_dis = speed_limit_config_.min_dynamic_ped_distance();
    double max_dis = speed_limit_config_.max_dynamic_ped_distance();
    double min_limit_v = speed_limit_config_.min_dynamic_ped_speed_limit();
    double max_limit_v = speed_limit_config_.max_speed_limit_ratio() *
                         FLAGS_planning_upper_speed_limit;
    if (arrive_speed < speed_limit_config_.lower_pedestrian_speed()) {
      min_dis = speed_limit_config_.min_dynamic_ped_distance_lower();
      max_dis = speed_limit_config_.max_dynamic_ped_distance_lower();
      min_limit_v = speed_limit_config_.min_dynamic_ped_speed_limit_lower();
    } else if (arrive_speed < speed_limit_config_.upper_pedestrian_speed()) {
      min_dis = speed_limit_config_.min_dynamic_ped_distance_middle();
      max_dis = speed_limit_config_.max_dynamic_ped_distance_middle();
      min_limit_v = speed_limit_config_.min_dynamic_ped_speed_limit_middle();
    } else if (arrive_speed > speed_limit_config_.faster_pedestrian_speed()) {
      min_dis = speed_limit_config_.min_dynamic_ped_distance_faster();
      max_dis = speed_limit_config_.max_dynamic_ped_distance_faster();
      min_limit_v = speed_limit_config_.min_dynamic_ped_speed_limit_faster();
    }
    speed_limit_pedestrian = InterpolationLookUp(
        obs_path_abs_l, min_dis, max_dis, min_limit_v, max_limit_v);
    ADEBUG << "dis = " << obs_path_abs_l
           << ", obs_lat_speed = " << obs_lat_speed
           << ", arrive_speed = " << arrive_speed;
    ADEBUG << "speed_limit_pedestrian = " << speed_limit_pedestrian;
  }
  return speed_limit_pedestrian;
}

double SpeedLimitDecider::GetSpeedLimitFromPedestrian(
    const Obstacle& obstacle, const double path_point_l,
    const double path_point_heading, const double reference_line_s) const {
  double speed_limit_pedestrian = std::numeric_limits<double>::max();

  const double adc_speed =
      reference_line_info_->vehicle_state().linear_velocity();
  const double adc_s =
      reference_line_info_->path_data().frenet_frame_path().front().s();
  const SLBoundary& obstacle_sl = obstacle.PerceptionSLBoundary();

  // speed limits are only applied near pedestrians
  bool too_far_away_obstacle =
      adc_speed * speed_limit_config_.pedestrian_speed_limit_time() <
      obstacle_sl.start_s() - adc_s;
  bool do_speed_limit =
      ((obstacle_sl.start_s() > reference_line_s) &&
       (obstacle_sl.start_s() <
        reference_line_s +
            speed_limit_config_.speed_limit_advance_distance())) ||
      ((obstacle_sl.end_s() > reference_line_s) &&
       (obstacle_sl.end_s() <
        reference_line_s + speed_limit_config_.speed_limit_advance_distance()));
  if (!too_far_away_obstacle && do_speed_limit) {
    double obs_path_l = std::abs(obstacle_sl.end_l() - path_point_l) <
                                std::abs(obstacle_sl.start_l() - path_point_l)
                            ? obstacle_sl.end_l() - path_point_l
                            : obstacle_sl.start_l() - path_point_l;
    double sign_obs_path_l = obs_path_l < 0.0 ? -1.0 : 1.0;
    const auto& vehicle_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
    double obs_path_abs_l = std::abs(std::abs(obs_path_l) - vehicle_half_width);

    ADEBUG << "obstacle id = " << obstacle.Id() << ", obstacle type = "
           << (obstacle.IsStatic() ? "static" : "dynamic")
           << ", obstacle speed = " << obstacle.speed()
           << ", HasTrajectory: " << obstacle.HasTrajectory()
           << ", path_point_l = " << path_point_l
           << ", obs_path_l = " << obs_path_l;
    ADEBUG << "Perception heading: " << obstacle.Perception().theta();
    ADEBUG << "SpeedHeading: " << obstacle.SpeedHeading();

    bool obs_near_to_path = false;
    if (obstacle.speed() > FLAGS_min_dynamic_obstacle_speed) {
      double obs_heading = obstacle.SpeedHeading();
      double diff_angle = std::fabs(common::math::AngleDiff(
          path_point_heading, obs_heading + M_PI_2 * sign_obs_path_l));
      double obs_cross_angle = FLAGS_obs_cross_angle_degree / kDegrees * M_PI_2;
      obs_near_to_path =
          obstacles_interval_.HyValue(obstacle, diff_angle) < obs_cross_angle;
      ADEBUG << "diff_angle = " << diff_angle
             << ", path_point_heading = " << path_point_heading
             << ", sign_obs_path_l = " << sign_obs_path_l
             << ", obs_heading = " << obs_heading;
    }
    ADEBUG << "obs_near_to_path = " << obs_near_to_path;

    if (obs_near_to_path) {
      ADEBUG << "Dynamic speed limit.";
      speed_limit_pedestrian = MakeSpeedLimitForDynamicPedestrian(
          obstacle, path_point_heading, obs_path_l);
    } else {
      ADEBUG << "Static speed limit.";
      if (obs_path_abs_l < speed_limit_config_.max_static_ped_distance()) {
        speed_limit_pedestrian = InterpolationLookUp(
            obs_path_abs_l, speed_limit_config_.min_static_ped_distance(),
            speed_limit_config_.max_static_ped_distance(),
            speed_limit_config_.min_static_ped_speed_limit(),
            speed_limit_config_.max_speed_limit_ratio() *
                FLAGS_planning_upper_speed_limit);
      }
    }
  }
  return speed_limit_pedestrian;
}

double SpeedLimitDecider::GetSpeedLimitFromBicycle(
    const Obstacle& obstacle, const double path_point_l,
    const double path_point_heading, const double reference_line_s) const {
  double speed_limit_bicycle = std::numeric_limits<double>::max();

  const double adc_speed =
      reference_line_info_->vehicle_state().linear_velocity();
  const double adc_s =
      reference_line_info_->path_data().frenet_frame_path().front().s();
  const SLBoundary& obstacle_sl = obstacle.PerceptionSLBoundary();

  // speed limits are only applied near bicycles
  bool too_far_away_obstacle =
      adc_speed * speed_limit_config_.bicycle_speed_limit_time() <
      obstacle_sl.start_s() - adc_s;
  bool do_speed_limit =
      ((obstacle_sl.start_s() > reference_line_s) &&
       (obstacle_sl.start_s() <
        reference_line_s +
            speed_limit_config_.speed_limit_advance_distance_for_bicycle())) ||
      ((obstacle_sl.end_s() > reference_line_s) &&
       (obstacle_sl.end_s() <
        reference_line_s +
            speed_limit_config_.speed_limit_advance_distance_for_bicycle()));
  if (!too_far_away_obstacle && do_speed_limit) {
    double obs_path_l = std::abs(obstacle_sl.end_l() - path_point_l) <
                                std::abs(obstacle_sl.start_l() - path_point_l)
                            ? obstacle_sl.end_l() - path_point_l
                            : obstacle_sl.start_l() - path_point_l;
    const auto& vehicle_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
    double obs_path_abs_l = std::abs(std::abs(obs_path_l) - vehicle_half_width);

    ADEBUG << "Static speed limit.";
    if (obs_path_abs_l < speed_limit_config_.max_bicycle_distance()) {
      speed_limit_bicycle = InterpolationLookUp(
          obs_path_abs_l, speed_limit_config_.min_bicycle_distance(),
          speed_limit_config_.max_bicycle_distance(),
          speed_limit_config_.min_speed_limit_ratio_for_bicycle() *
              FLAGS_planning_upper_speed_limit,
          speed_limit_config_.max_speed_limit_ratio() *
              FLAGS_planning_upper_speed_limit);
    }
  }
  return speed_limit_bicycle;
}

double SpeedLimitDecider::GetSpeedLimitFromNudgeObstaclesForHighSpeed(
    const IndexedList<std::string, Obstacle>& obstacles,
    const SLBoundary& adc_sl_boundary_path_point,
    const bool adc_sl_boundary_result) const {
  double speed_limit_from_nudge_obstacles = FLAGS_planning_upper_speed_limit;
  double min_lateral_diff = FLAGS_lateral_ignore_buffer;
  std::string min_lateral_obs_id = "";
  bool is_need_nudge_speed_limit = false;

  for (const auto* ptr_obstacle : obstacles.Items()) {
    const SLBoundary& obstacle_sl_boundary =
        ptr_obstacle->PerceptionSLBoundary();
    // Get ADC boundary from path point failed need break, not to do nudge
    // speed limit
    if (!adc_sl_boundary_result) {
      break;
    }

    if (!FLAGS_enable_nudge_slowdown) {
      break;
    }

    if (ptr_obstacle->IsVirtual()) {
      continue;
    }

    if (!ptr_obstacle->LateralDecision().has_nudge()) {
      continue;
    }

    /* ref line:
     * -------------------------------
     *    start_s   end_s
     * ------|  adc   |---------------
     * ------------|  obstacle |------
     */

    bool no_longitudinal_overlap =
        adc_sl_boundary_path_point.end_s() <
            obstacle_sl_boundary.start_s() -
                speed_limit_config_.obs_nudge_s_threshold() ||
        adc_sl_boundary_path_point.start_s() >
            obstacle_sl_boundary.end_s() +
                speed_limit_config_.obs_nudge_s_threshold();

    if (no_longitudinal_overlap) {
      continue;
    }

    const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

    double distance_diff =
        nudge_decision.type() == ObjectNudge::LEFT_NUDGE
            ? std::fabs(adc_sl_boundary_path_point.start_l() -
                        obstacle_sl_boundary.end_l())
            : std::fabs(adc_sl_boundary_path_point.end_l() -
                        obstacle_sl_boundary.start_l());
    if (distance_diff < min_lateral_diff) {
      is_need_nudge_speed_limit = true;
      min_lateral_diff = distance_diff;
      min_lateral_obs_id = ptr_obstacle->Id();
    }
  }
  ADEBUG << "min_lateral_diff = " << min_lateral_diff;
  if (is_need_nudge_speed_limit) {
    const auto& obstacle_nudge =
        reference_line_info_->path_decision()->Find(min_lateral_obs_id);
    if (perception::PerceptionObstacle::PEDESTRIAN ==
        obstacle_nudge->Perception().type()) {
      speed_limit_from_nudge_obstacles = InterpolationLookUp(
          min_lateral_diff,
          speed_limit_config_.min_nudge_obs_lateral_distance(),
          speed_limit_config_.max_nudge_pedestrian_lateral_distance(),
          speed_limit_config_.min_nudge_obs_speed_limit(),
          FLAGS_planning_upper_speed_limit);
    } else if (perception::PerceptionObstacle::BICYCLE ==
               obstacle_nudge->Perception().type()) {
      speed_limit_from_nudge_obstacles = InterpolationLookUp(
          min_lateral_diff,
          speed_limit_config_.min_nudge_obs_lateral_distance(),
          speed_limit_config_.max_nudge_bicycle_lateral_distance(),
          speed_limit_config_.min_nudge_obs_speed_limit(),
          FLAGS_planning_upper_speed_limit);
    } else if (perception::PerceptionObstacle::VEHICLE ==
               obstacle_nudge->Perception().type()) {
      speed_limit_from_nudge_obstacles = InterpolationLookUp(
          min_lateral_diff,
          speed_limit_config_.min_nudge_obs_lateral_distance(),
          speed_limit_config_.max_nudge_vehicle_lateral_distance(),
          speed_limit_config_.min_nudge_obs_speed_limit(),
          FLAGS_planning_upper_speed_limit);
    } else {
      // unknown
      speed_limit_from_nudge_obstacles = InterpolationLookUp(
          min_lateral_diff,
          speed_limit_config_.min_nudge_obs_lateral_distance(),
          speed_limit_config_.max_nudge_unknown_lateral_distance(),
          speed_limit_config_.min_nudge_obs_speed_limit(),
          FLAGS_planning_upper_speed_limit);
    }

    ADEBUG << "speed_limit_from_nudge_obstacles_change = "
           << speed_limit_from_nudge_obstacles;
  }
  return speed_limit_from_nudge_obstacles;
}

double SpeedLimitDecider::GetSpeedLimitFromPedestrianForHighSpeed(
    const IndexedList<std::string, Obstacle>& obstacles,
    const SLBoundary& adc_sl_boundary_path_point,
    const common::PathPoint path_point, bool adc_sl_boundary_result) const {
  double speed_limit_from_pedestrian = std::numeric_limits<double>::max();
  const double path_s = path_point.s();
  const double reference_line_s = path_point.s();
  const double path_point_heading = path_point.theta();
  double obs_cross_angle = FLAGS_obs_cross_angle_degree / kDegrees * M_PI_2;
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  if (hdmap::Lane::PLAY_STREET != reference_line_info_->GetLaneType() &&
      FLAGS_enable_pedestrian_slowdown) {
    double adc_path_l = 0.0;
    if (adc_sl_boundary_result) {
      adc_path_l = 0.5 * (adc_sl_boundary_path_point.start_l() +
                          adc_sl_boundary_path_point.end_l());
    }
    const double adc_speed =
        reference_line_info_->vehicle_state().linear_velocity();
    for (const auto* ptr_obstacle : obstacles.Items()) {
      double speed_limit_obs = std::numeric_limits<double>::max();
      const SLBoundary& obstacle_sl = ptr_obstacle->PerceptionSLBoundary();
      // Not do pedestrian speed limit in play street
      if (ptr_obstacle->IsVirtual() ||
          perception::PerceptionObstacle::PEDESTRIAN !=
              ptr_obstacle->Perception().type()) {
        continue;
      }
      if (adc_speed * speed_limit_config_.pedestrian_speed_limit_time() <
          obstacle_sl.start_s() - frenet_path.front().s()) {
        continue;
      }
      // speed limits are only applied near pedestrians
      bool do_speed_limit =
          ((obstacle_sl.start_s() > reference_line_s) &&
           (obstacle_sl.start_s() <
            reference_line_s +
                speed_limit_config_.speed_limit_advance_distance())) ||
          ((obstacle_sl.end_s() > reference_line_s) &&
           (obstacle_sl.end_s() <
            reference_line_s +
                speed_limit_config_.speed_limit_advance_distance()));
      if (!do_speed_limit) {
        continue;
      }
      double obs_path_l = std::abs(obstacle_sl.end_l() - adc_path_l) <
                                  std::abs(obstacle_sl.start_l() - adc_path_l)
                              ? obstacle_sl.end_l() - adc_path_l
                              : obstacle_sl.start_l() - adc_path_l;
      double sign_obs_path_l = obs_path_l < 0.0 ? -1.0 : 1.0;
      const auto& vehicle_half_width =
          common::VehicleConfigHelper::GetConfig().vehicle_param().width() *
          0.5;
      double obs_path_abs_l =
          std::abs(std::abs(obs_path_l) - vehicle_half_width);

      ADEBUG << "=================================";
      ADEBUG << "obstacle id = " << ptr_obstacle->Id();
      ADEBUG << "obstacle type = "
             << (ptr_obstacle->IsStatic() ? "static" : "dynamic");
      ADEBUG << "adc_path_l = " << adc_path_l;
      ADEBUG << "obs_path_l = " << obs_path_l;

      bool obs_near_to_path = false;
      if (ptr_obstacle->HasTrajectory()) {
        double obs_heading =
            ptr_obstacle->Trajectory().trajectory_point(0).path_point().theta();
        double diff_angle = std::fabs(common::math::AngleDiff(
            path_point_heading, obs_heading + M_PI_2 * sign_obs_path_l));
        obs_near_to_path = obstacles_interval_.HyValue(
                               *ptr_obstacle, diff_angle) < obs_cross_angle;
        ADEBUG << "path_point_heading = " << path_point_heading;
        ADEBUG << "sign_obs_path_l = " << sign_obs_path_l;
        ADEBUG << "obs_heading = " << obs_heading;
      }
      ADEBUG << "obs_near_to_path = " << obs_near_to_path;

      if (!ptr_obstacle->IsStatic() && obs_near_to_path) {
        speed_limit_obs = InterpolationLookUp(
            obs_path_abs_l, speed_limit_config_.min_dynamic_ped_distance(),
            speed_limit_config_.max_dynamic_ped_distance(),
            speed_limit_config_.min_dynamic_ped_speed_limit(),
            FLAGS_planning_upper_speed_limit);
      } else {
        speed_limit_obs = InterpolationLookUp(
            obs_path_abs_l,
            speed_limit_config_.min_nudge_obs_lateral_distance(),
            speed_limit_config_.max_nudge_pedestrian_lateral_distance(),
            speed_limit_config_.min_nudge_obs_speed_limit(),
            FLAGS_planning_upper_speed_limit);
      }
      speed_limit_from_pedestrian =
          std::min(speed_limit_obs, speed_limit_from_pedestrian);
      ADEBUG << "speed_limit_obs(" << path_s << "): " << speed_limit_obs;
    }
  }
  ADEBUG << "speed_limit_from_pedestrian(" << path_s
         << "): " << speed_limit_from_pedestrian;
  return speed_limit_from_pedestrian;
}
double SpeedLimitDecider::GetNearstCrossDistance() const {
  double nearst_cross_distance = std::numeric_limits<double>::infinity();
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& slowdown_obstacles = reference_line_info_->GetSlowdownObstacles();
  const auto& frenet_frame_path =
      reference_line_info_->path_data().frenet_frame_path();
  const double adc_s =
      (adc_sl_boundary.end_s() + adc_sl_boundary.start_s()) * 0.5;
  const auto& reference_speed_data = reference_line_info_->speed_data();
  // obs can reach distance in path,get neast distance.
  for (const auto& cross_slowdown_obstacle_id :
       reference_line_info_->GetCrossSlowdownObstacles()) {
    const auto* cross_slowdown_obstacle =
        reference_line_info_->path_decision()->Find(cross_slowdown_obstacle_id);
    if (nullptr == cross_slowdown_obstacle) {
      continue;
    }
    // has overtake decision no limit speed.
    if (cross_slowdown_obstacle->LongitudinalDecision().has_overtake()) {
      continue;
    }
    const auto& obs_sl = cross_slowdown_obstacle->PerceptionSLBoundary();
    if (obs_sl.end_s() < adc_sl_boundary.end_s()) {
      continue;
    }
    const auto& velocity = cross_slowdown_obstacle->Perception().velocity();
    const auto& frenet_point = frenet_frame_path.GetNearestPoint(obs_sl);
    const auto& path_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            (obs_sl.start_s() + obs_sl.end_s()) * 0.5 - adc_s);
    const double obstacle_longitudinal_speed =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta())
            .InnerProd(Vec2d(velocity.x(), velocity.y()));
    const double obstacle_lateral_speed =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta())
            .CrossProd(Vec2d(velocity.x(), velocity.y()));
    // get lateral distance.
    double lat_distance_between_adc_and_obs =
        std::max(std::fabs(adc_sl_boundary.start_l() - obs_sl.end_l()),
                 std::fabs(adc_sl_boundary.end_l() - obs_sl.start_l()));

    // calculate the longitudinal distance that requires speed limit based
    // on the speed and lateral distance of the obstacle and the
    // longitudinal speed.
    double limit_min_distance = 0.0;
    double limit_max_distance = 0.0;
    double collision_time = 0.0;
    double limit_end_s = 0.0;

    collision_time = lat_distance_between_adc_and_obs /
                     std::max(std::fabs(obstacle_lateral_speed), kEpison);
    // obs can't reach path break.
    if (collision_time > kPredictionTime) {
      continue;
    }
    SpeedPoint sp_end;
    double relative_time =
        collision_time - speed_limit_config_.cross_limit_speed_time_buffer();
    if (perception::PerceptionObstacle::PEDESTRIAN ==
        cross_slowdown_obstacle->Perception().type()) {
      relative_time = collision_time -
                      speed_limit_config_.cross_limit_speed_time_buffer() * 2;
    }
    relative_time =
        std::min(std::max(0.0, relative_time),
                 reference_line_info_->st_graph_data().total_time_by_conf());
    reference_speed_data.EvaluateByTime(relative_time, &sp_end);
    const double path_s_end = sp_end.s() + adc_s;

    if (obstacle_longitudinal_speed < 0.0) {
      limit_min_distance =
          std::max(frenet_point.s() +
                       collision_time * obstacle_longitudinal_speed - adc_s,
                   0.0);
      limit_max_distance = frenet_point.s() - adc_s;
      limit_end_s =
          frenet_point.s() + collision_time * obstacle_longitudinal_speed;
    } else {
      limit_max_distance = frenet_point.s() +
                           collision_time * obstacle_longitudinal_speed - adc_s;
      limit_end_s =
          frenet_point.s() + collision_time * obstacle_longitudinal_speed;
      limit_min_distance = std::max(frenet_point.s() - adc_s, 0.0);
    }
    ADEBUG << "limit_max_distance = " << limit_max_distance;
    if (limit_end_s < path_s_end &&
        FLAGS_enable_no_slow_down_for_overtake_obstacle) {
      continue;
    }
    if (limit_min_distance < nearst_cross_distance) {
      nearst_cross_distance = limit_min_distance;
    }
  }
  return nearst_cross_distance;
}
void SpeedLimitDecider::UpdateSpeedLimitsForCrossObstacle(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& slowdown_obstacles = reference_line_info_->GetSlowdownObstacles();

  if (!reference_line_info_->GetCrossSlowdownObstacles().empty()) {
    double nearst_cross_distance = std::numeric_limits<double>::infinity();

    nearst_cross_distance = GetNearstCrossDistance();
    ADEBUG << "nearst_cross_distance = " << nearst_cross_distance;
    int index = -1;
    // speed limit all point
    uint32_t path_step_cross =
        speed_limit_config_.closer_path_step_for_speed_limit();
    for (uint32_t i = 0; i < discretized_path.size(); i += path_step_cross) {
      index++;
      const double path_s = discretized_path.at(i).s();
      if (path_s > kFirstPathDistanceBuffer) {
        path_step_cross =
            speed_limit_config_.far_away_path_step_for_speed_limit();
      }
      if (static_cast<uint32_t>(index) >
          speed_limit_data->speed_limit_points().size() - 1) {
        break;
      }

      double v_upper_bound = FLAGS_planning_upper_speed_limit;
      v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
      ADEBUG << "ori speed limit = " << v_upper_bound;

      // nearst_cross_distance
      ADEBUG << "path_s = " << path_s;
      double speed_limit_cross_obs = InterpolationLookUp(
          nearst_cross_distance - path_s - kYieldBuffer, 0.0,
          speed_limit_config_.max_cross_path_distance(),
          speed_limit_config_.min_cross_limit_speed(),
          FLAGS_planning_upper_speed_limit);
      ADEBUG << "speed_limit_cross_obs = " << speed_limit_cross_obs;
      v_upper_bound = std::min(v_upper_bound, speed_limit_cross_obs);
      speed_limit_data->SetSpeedLimitByS(path_s, v_upper_bound);
    }
  }
}
void SpeedLimitDecider::UpdateSpeedLimitsForReverseObstacle(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_frame_path =
      reference_line_info_->path_data().frenet_frame_path();
  const double adc_speed =
      reference_line_info_->vehicle_state().linear_velocity();
  const double adc_s =
      (adc_sl_boundary.end_s() + adc_sl_boundary.start_s()) * 0.5;
  if (!reference_line_info_->GetReverseSlowdownObstacles().empty()) {
    const auto& reverse_slow_down_obstacles =
        reference_line_info_->GetReverseSlowdownObstacles();
    for (const auto& reverse_slowdown_obstacle_id :
         reverse_slow_down_obstacles) {
      const auto* reverse_slowdown_obstacle =
          reference_line_info_->path_decision()->Find(
              reverse_slowdown_obstacle_id);
      if (nullptr == reverse_slowdown_obstacle) {
        continue;
      }
      const auto& obs_sl = reverse_slowdown_obstacle->PerceptionSLBoundary();
      ADEBUG << "obs_sl = " << obs_sl.start_l() << "   " << obs_sl.end_l();
      // obstacles only in front of ADC.
      if (obs_sl.start_s() < adc_sl_boundary.end_s()) {
        continue;
      }
      const auto& velocity = reverse_slowdown_obstacle->Perception().velocity();
      ADEBUG << "obs_v = " << reverse_slowdown_obstacle->speed();
      const auto& frenet_point = frenet_frame_path.GetNearestPoint(obs_sl);
      const auto& path_point =
          reference_line_info_->path_data().GetPathPointWithPathS(
              (obs_sl.start_s() + obs_sl.end_s()) * 0.5 - adc_s);
      ADEBUG << "obs_heading = "
             << reverse_slowdown_obstacle->Perception().theta();
      const double obstacle_longitudinal_speed =
          common::math::Vec2d::CreateUnitVec2d(path_point.theta())
              .InnerProd(Vec2d(velocity.x(), velocity.y()));

      double lon_distance_between_adc_and_obs =
          obs_sl.start_s() - adc_sl_boundary.end_s();
      ADEBUG << "lon_distance_between_adc_and_obs = "
             << lon_distance_between_adc_and_obs;

      // calculate the longitudinal distance that requires speed limit based
      // on the speed and lateral distance of the obstacle and the
      // longitudinal speed.
      double limit_start_s = 0.0;
      double limit_end_s = 0.0;

      ADEBUG << "frenet_point.s() = " << frenet_point.s();
      ADEBUG << "limit_start_s = " << limit_start_s
             << "           limit_end_s = " << limit_end_s;
      uint32_t path_step_reverse =
          speed_limit_config_.closer_path_step_for_speed_limit();
      int index = -1;
      for (uint32_t i = 0; i < discretized_path.size();
           i += path_step_reverse) {
        index++;
        const double path_s = discretized_path.at(i).s();
        ADEBUG << "path_s = " << path_s;
        if (path_s > kFirstPathDistanceBuffer) {
          path_step_reverse =
              speed_limit_config_.far_away_path_step_for_speed_limit();
        }
        if (static_cast<uint32_t>(index) >
            speed_limit_data->speed_limit_points().size() - 1) {
          break;
        }
        // before yield wall need limit to 2m/s.
        double min_yield_speed = InterpolationLookUp(
            lon_distance_between_adc_and_obs, kMinConsiderLength,
            kMaxConsiderLength, speed_limit_config_.min_reverse_limit_speed(),
            FLAGS_planning_upper_speed_limit);
        // AINFO << "min_yield_speed = " << min_yield_speed;
        double speed_limit_reverse_obs = InterpolationLookUp(
            lon_distance_between_adc_and_obs - path_s -
                std::fabs(obstacle_longitudinal_speed) * 2,
            0.0,
            (std::fabs(obstacle_longitudinal_speed) + adc_speed) *
                kPredictionTime,
            min_yield_speed, FLAGS_planning_upper_speed_limit);
        double v_upper_bound = FLAGS_planning_upper_speed_limit;
        v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
        ADEBUG << "ori speed limit = " << v_upper_bound;
        v_upper_bound = std::min(v_upper_bound, speed_limit_reverse_obs);
        ADEBUG << "AFTER speed limit = " << v_upper_bound;
        speed_limit_data->SetSpeedLimitByS(path_s, v_upper_bound);
      }
    }
  }
}
void SpeedLimitDecider::UpdateSpeedLimitsForSlowdownObstacle(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  uint32_t path_step = speed_limit_config_.closer_path_step_for_speed_limit();
  const auto& slowdown_obstacles = reference_line_info_->GetSlowdownObstacles();
  const auto& frenet_frame_path =
      reference_line_info_->path_data().frenet_frame_path();
  const double adc_s =
      (adc_sl_boundary.end_s() + adc_sl_boundary.start_s()) * 0.5;

  if (!slowdown_obstacles.empty() && FLAGS_enable_slowdown_for_obstacle) {
    for (const auto& slowdown_obstacle_id : slowdown_obstacles) {
      const auto* slowdown_obstacle =
          reference_line_info_->path_decision()->Find(slowdown_obstacle_id);
      if (nullptr == slowdown_obstacle) {
        continue;
      }
      const auto& obs_sl = slowdown_obstacle->PerceptionSLBoundary();
      // obstacles only in front of ADC.
      // obstacles only in right of ADC.
      if ((obs_sl.start_s() < adc_sl_boundary.end_s()) ||
          (adc_sl_boundary.start_l() < obs_sl.end_l())) {
        continue;
      }

      const auto& velocity = slowdown_obstacle->Perception().velocity();
      const auto& frenet_point = frenet_frame_path.GetNearestPoint(obs_sl);
      const auto& path_point =
          reference_line_info_->path_data().GetPathPointWithPathS(
              (obs_sl.start_s() + obs_sl.end_s()) * 0.5 - adc_s);
      const double obstacle_longitudinal_speed =
          common::math::Vec2d::CreateUnitVec2d(path_point.theta())
              .InnerProd(Vec2d(velocity.x(), velocity.y()));
      const double obstacle_lateral_speed =
          common::math::Vec2d::CreateUnitVec2d(path_point.theta())
              .CrossProd(Vec2d(velocity.x(), velocity.y()));

      double lat_distance_between_adc_and_obs =
          adc_sl_boundary.start_l() - obs_sl.end_l();

      // calculate the longitudinal distance that requires speed limit based
      // on the speed and lateral distance of the obstacle and the
      // longitudinal speed.
      double limit_start_s = 0.0;
      double limit_end_s = 0.0;
      double collision_time = 0.0;

      collision_time = lat_distance_between_adc_and_obs /
                       std::max(obstacle_lateral_speed, kEpison);
      // can't reach path or deviation from path.
      if (collision_time > kPredictionTime && obstacle_lateral_speed < 0.0) {
        continue;
      }
      if (obstacle_longitudinal_speed < 0.0) {
        limit_start_s =
            std::max(frenet_point.s() +
                         collision_time * obstacle_longitudinal_speed - adc_s,
                     0.0);
        limit_end_s = frenet_point.s() - adc_s;
      } else {
        limit_end_s = frenet_point.s() +
                      collision_time * obstacle_longitudinal_speed - adc_s;
        limit_start_s = std::max(frenet_point.s() - adc_s, 0.0);
      }
      int index = -1;
      for (uint32_t i = 0; i < discretized_path.size(); i += path_step) {
        index++;
        const double path_s = discretized_path.at(i).s();
        if (path_s > kFirstPathDistanceBuffer) {
          path_step = speed_limit_config_.far_away_path_step_for_speed_limit();
        }
        // 5m before  need to speed limit.
        if (path_s < limit_start_s - kSpeedLimitDistance) {
          continue;
        }
        if (path_s > limit_end_s) {
          break;
        }
        if (static_cast<uint32_t>(index) >
            speed_limit_data->speed_limit_points().size() - 1) {
          break;
        }
        double v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
        v_upper_bound =
            std::min(v_upper_bound, 0.5 * FLAGS_planning_upper_speed_limit);
        speed_limit_data->SetSpeedLimitByS(path_s, v_upper_bound);
      }
    }
  }
}

void SpeedLimitDecider::UpdateSpeedLimitsForStopYieldFollowDecision(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  uint32_t path_step = speed_limit_config_.closer_path_step_for_speed_limit();

  std::string close_obs_id_stop = "";
  std::string close_obs_id_yield = "";
  std::string close_obs_id_follow = "";
  GetCloseStopYieldFollowObstacleId(obstacles, &close_obs_id_stop,
                                    &close_obs_id_yield, &close_obs_id_follow);
  const auto* stop_obs_to_limit_speed = obstacles.Find(close_obs_id_stop);
  double min_s_for_stop = -1.0;
  if (nullptr != stop_obs_to_limit_speed) {
    min_s_for_stop = stop_obs_to_limit_speed->path_st_boundary().min_s();
  }
  const auto* yield_obs_to_limit_speed = obstacles.Find(close_obs_id_yield);
  double min_s_for_yield = -1.0;
  double max_s_for_yield = -1.0;
  if (nullptr != yield_obs_to_limit_speed) {
    min_s_for_yield = yield_obs_to_limit_speed->path_st_boundary().min_s();
    max_s_for_yield = yield_obs_to_limit_speed->path_st_boundary().max_s();
  }
  const auto* follow_obs_to_limit_speed = obstacles.Find(close_obs_id_follow);
  double min_s_for_follow = -1.0;
  if (nullptr != follow_obs_to_limit_speed) {
    min_s_for_follow = follow_obs_to_limit_speed->path_st_boundary().min_s();
  }
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  size_t index = 0;
  for (uint32_t i = 0; i < discretized_path.size(); i += path_step, ++index) {
    const double path_s = discretized_path.at(i).s();
    if (path_s > kFirstPathDistanceBuffer) {
      path_step = speed_limit_config_.far_away_path_step_for_speed_limit();
    }
    if (index > speed_limit_data->speed_limit_points().size() - 1UL) {
      break;
    }
    double v_upper_bound = speed_limit_data->GetSpeedLimitByS(path_s);
    if (nullptr != stop_obs_to_limit_speed) {
      if (path_s < min_s_for_stop) {
        const double distance = min_s_for_stop - path_s;
        v_upper_bound =
            std::min(v_upper_bound, GetSpeedLimitFromStopObstacle(distance));
      }
    }
    if (nullptr != yield_obs_to_limit_speed) {
      // 5m before YIELD_WALL need to speed limit.
      if (path_s < max_s_for_yield &&
          path_s > min_s_for_yield - kSpeedLimitDistance) {
        v_upper_bound =
            std::min(v_upper_bound, GetSpeedLimitFromYieldObstacle(path_s));
      }
    }
    if (nullptr != follow_obs_to_limit_speed) {
      if (path_s < min_s_for_follow) {
        const double distance = min_s_for_follow - path_s;
        v_upper_bound = std::min(
            v_upper_bound, GetSpeedLimitFromFollowObstacle(
                               distance, follow_obs_to_limit_speed->speed()));
      }
    }
    speed_limit_data->SetSpeedLimitByS(path_s, v_upper_bound);
  }
}

void SpeedLimitDecider::GetCloseStopYieldFollowObstacleId(
    const IndexedList<std::string, Obstacle>& obstacles,
    std::string* const close_obs_id_stop, std::string* const close_obs_id_yield,
    std::string* const close_obs_id_follow) const {
  double close_key_point_s_stop = std::numeric_limits<double>::max();
  double close_key_point_s_yield = std::numeric_limits<double>::max();
  double close_key_point_s_follow = std::numeric_limits<double>::max();

  bool is_need_follow_speed_limit =
      FLAGS_enable_follow_slowdown &&
      hdmap::Lane::PLAY_STREET != reference_line_info_->GetLaneType() &&
      !util::IsOvertake(planning_context_) &&
      !util::IsLaneChange(planning_context_);
  for (const auto* ptr_obs : obstacles.Items()) {
    if (!ptr_obs->HasLongitudinalDecision() ||
        !ptr_obs->is_path_st_boundary_initialized()) {
      continue;
    }
    if (FLAGS_enable_stop_slowdown &&
        ptr_obs->LongitudinalDecision().has_stop()) {
      double key_point_s_stop =
          ptr_obs->path_st_boundary().min_s() +
          ptr_obs->LongitudinalDecision().stop().distance_s();
      if (close_key_point_s_stop > key_point_s_stop) {
        close_key_point_s_stop = key_point_s_stop;
        *close_obs_id_stop = ptr_obs->Id();
      }
    }
    if (FLAGS_enable_yield_slowdown &&
        ptr_obs->LongitudinalDecision().has_yield()) {
      double key_point_s_yield =
          ptr_obs->path_st_boundary().min_s() +
          ptr_obs->LongitudinalDecision().yield().distance_s();
      if (close_key_point_s_yield > key_point_s_yield) {
        close_key_point_s_yield = key_point_s_yield;
        *close_obs_id_yield = ptr_obs->Id();
      }
    }
    if (is_need_follow_speed_limit &&
        ptr_obs->LongitudinalDecision().has_follow()) {
      double key_point_s_follow =
          ptr_obs->path_st_boundary().min_s() +
          ptr_obs->LongitudinalDecision().follow().distance_s();
      if (close_key_point_s_follow > key_point_s_follow) {
        close_key_point_s_follow = key_point_s_follow;
        *close_obs_id_follow = ptr_obs->Id();
      }
    }
  }
}

double SpeedLimitDecider::GetSpeedLimitFromYieldObstacle(
    const double path_s) const {
  double v_upper_bound = std::numeric_limits<double>::max();
  double half_adc_decel = std::fabs(common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_deceleration() *
                                    0.5);
  const double adc_speed =
      reference_line_info_->vehicle_state().linear_velocity();
  double decelerate_s = adc_speed * adc_speed / (2.0 * half_adc_decel);
  // able to slow down to 0.0 within path_s.
  if (decelerate_s <= path_s) {
    v_upper_bound = speed_limit_config_.yield_speed_limit_ratio() *
                    FLAGS_planning_upper_speed_limit;
  } else {
    // Unable to slow down to 0.0 within path_s.
    v_upper_bound = std::sqrt(
        std::fabs(adc_speed * adc_speed - 2 * half_adc_decel * path_s));
    ADEBUG << "v_upper_bound =" << v_upper_bound;
  }
  return v_upper_bound;
}

double SpeedLimitDecider::GetSpeedLimitFromStopObstacle(
    const double distance) const {
  double stop_speed_limit = InterpolationLookUp(
      distance, speed_limit_config_.min_speed_limit_longitude_distance(),
      speed_limit_config_.max_speed_limit_longitude_distance_for_stop(),
      speed_limit_config_.min_stop_speed_limit(),
      FLAGS_planning_upper_speed_limit);
  return stop_speed_limit;
}

double SpeedLimitDecider::GetSpeedLimitFromFollowObstacle(
    const double distance, const double obs_speed) const {
  double follow_speed_limit = InterpolationLookUp(
      distance, speed_limit_config_.min_speed_limit_longitude_distance(),
      speed_limit_config_.max_speed_limit_longitude_distance_for_follow(),
      speed_limit_config_.min_follow_speed_limit(),
      FLAGS_planning_upper_speed_limit);
  return std::max(
      follow_speed_limit,
      obs_speed * speed_limit_config_.min_follow_obstacle_speed_ratio());
}

}  // namespace planning
}  // namespace century
