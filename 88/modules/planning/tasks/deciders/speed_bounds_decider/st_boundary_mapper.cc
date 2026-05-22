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

#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::PathPoint;
using century::common::PointENU;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
using century::hdmap::HDMapUtil;
using century::hdmap::Junction;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::Lane;
using century::hdmap::PathOverlap;
using century::perception::PerceptionObstacle;

namespace {
constexpr double kLongitudinalTimeToCollision = 3.0;
constexpr double kNegtiveTimeThreshold = -1.0;
constexpr double kStopBuffer = 4.0;
constexpr double kHighVelocity = 5.0;
constexpr double kLowVelocity = 3.0;
constexpr double kMaxLateralBuffer = 0.6;
constexpr double kSecondaryLateralBuffer = 0.4;
constexpr double kMinLateralBuffer = 0.2;
constexpr double kLateralBufferForYiedObstacle = 0.3;
constexpr double kHysteresisDistance = 0.5;
constexpr double kDistanceOutLane = 1.5;
constexpr double kHalfLaneWidth = 1.75;
constexpr double kDistanceOfAdcFront = 15;
constexpr double kMinVelocity = 1.0;
constexpr double kSpeedDiffBuffer = 1.7;
constexpr double kLongObsLength = 8.0;
constexpr double kMinObstacleTrajectoryStepDistance = 0.3;
constexpr double kMaxObstacleTrajectoryStepDistance = 3.0;
constexpr size_t kMinObstacleTrajectorySize = 10UL;
constexpr double kCollisionPathMagnifyRatio = 1.1;
constexpr double kMinCollisionPathLen = 35.0;
constexpr double kMinReverseVelocity = -1.0;
constexpr double kStopReverseVelocity = -0.05;
constexpr double kShortTrajectoryRelativeTime = 4.0;
constexpr double kMinTrajectoryRelativeTime = 2.0;
constexpr double kMinPercentOfReverseSpeed = 0.1;
constexpr double kEpison = 1e-5;
constexpr double kObstacleLengthRatio = 0.25;
constexpr double kLatBuffer = 1.0;
constexpr double kIgnoreObstacleSpeedCoeff = 1.5;
constexpr double kComfortableAcc = 3.0;
constexpr double kBreakBuff = 1.0;
constexpr double kMinLateralDiff = 0.5;
constexpr size_t kPathStep = 2UL;
constexpr double kMinKappa = 0.005;
constexpr double kConsiderLatBuffer = 1.0;
constexpr double kTrajectoryRelativeTime = 8.0;
constexpr double kMaxTrajectoryRelativeTime = 10.0;
constexpr double kSecondVelocity = 2.0;
constexpr double kLowSpeed = 1.0;
constexpr double kDegrees = 90.0;
constexpr double kMaxDegrees = 180.0;
constexpr double kMinRelativeTime = 1.0;
constexpr double kMinDiffTheta = 0.1;
constexpr size_t kHyCapacity = 200UL;
constexpr double kMaxOvertakeLongitudinalBuffer = 20.0;
constexpr double kRelativeTimeCoeff = 1.0;
constexpr double kMaxRelativeTime = 8.0;
constexpr int kOneStep = 1;
constexpr double kQuadrantAngle = 90.0;
constexpr double kMaxAngle = 360.0;
constexpr double kOctupleAngle = 45.0;
constexpr double kDiffHeading = 45.0;
constexpr size_t kDefaultNumPoint = 50;
constexpr double kComfortDecel = 0.5;
constexpr double kMinConsiderLength = 10.0;
constexpr double kStraightAngle = 10.0;
constexpr double kSpeedCoeff = 1.2;
constexpr double kLateralBufferForAdc = 0.5;
constexpr double kMinSameAngle = 0.0;
constexpr double kObsHighSpeed = 5.0;
constexpr double kConsiderLonDistance = 30.0;
constexpr size_t kMinPathStep = 1;
constexpr double kTrajectoryTime = 5.0;
constexpr double kTrajectoryTimeForLowSpeed = 3.0;
constexpr double kLowBicycleSpeed = 4.0;
constexpr double kLowDecel = 1.0;
constexpr double kMinHeadingDiff = 5.0;
constexpr double kMinAngleDiff = 2.0;
constexpr double kStopWallBuffer = 2.0;
constexpr double kLowDiffLForNoOverlap = -0.2;
constexpr double kUpperfortableDecel = 0.0;
constexpr double kMaxDiffLForNoOverlap = 0.0;
constexpr double kComfortableDecel = 3.0;
constexpr size_t kHyCapacityForSpeed = 50UL;
constexpr double kMaxSpeedForBicycle = 4.0;
constexpr double kLowSpeedForBicycle = 3.5;
constexpr double kUpperSpeedForBicycle = 4.5;
constexpr double kLateralBufferForBicycle = 3.0;
constexpr size_t kAddedIdSize = 3UL;
constexpr char kAddedIdPrefix[3] = "_b";
constexpr double kEpsilon = 1e-30;
constexpr uint32_t kTrajectoryPointsTimes = 7;
constexpr double kTireStackerHeight = 6.0;
constexpr double kNormalBuffer = 0.3;
}  // namespace

static HysteresisInterval adc_obstacles_l_diff_interval(kMaxDiffLForNoOverlap,
                                                        kLowDiffLForNoOverlap,
                                                        kUpperfortableDecel,
                                                        kHyCapacity);
static HysteresisInterval obstacles_speed_interval(kMaxSpeedForBicycle,
                                                   kLowSpeedForBicycle,
                                                   kUpperSpeedForBicycle,
                                                   kHyCapacityForSpeed);

STBoundaryMapper::STBoundaryMapper(
    const SLBoundary& adc_sl_boundary, const SpeedBoundsDeciderConfig& config,
    const ReferenceLine& reference_line, const PathData& path_data,
    const SpeedLimit& speed_limit, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector,
    ReferenceLineInfo* reference_line_info, Frame* const frame)
    : adc_sl_boundary_(adc_sl_boundary),
      speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      speed_limit_(speed_limit),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector),
      reference_line_info_(reference_line_info),
      frame_(frame) {
  if (!reference_line_.GetLaneWidth(adc_sl_boundary_.start_s(),
                                    &adc_lane_left_width_,
                                    &adc_lane_right_width_)) {
    adc_lane_left_width_ = kHalfLaneWidth;
    adc_lane_right_width_ = kHalfLaneWidth;
  }
  diagonal_heading_ = reference_line_info->vehicle_state().heading();
}

void STBoundaryMapper::SetInnerJunctionStatus() const {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  common::PointENU vehicle_state_point;
  Vec2d car_position;
  vehicle_state_point.set_x(injector_->vehicle_state()->vehicle_state().x());
  vehicle_state_point.set_y(injector_->vehicle_state()->vehicle_state().y());
  CHECK_NOTNULL(base_map_ptr);
  injector_->is_adc_in_junction_ = false;
  if (base_map_ptr->GetJunctions(vehicle_state_point, 1.0, &junctions) == 0) {
    if (junctions.size() > 0) {
      car_position.set_x(injector_->vehicle_state()->vehicle_state().x());
      car_position.set_y(injector_->vehicle_state()->vehicle_state().y());
      if (JudgeCarInTrafficeLightInnerJunction(&junctions, car_position,
                                               &junction)) {
        injector_->is_adc_in_junction_ = true;
      } else {
        ADEBUG << "No in traffic_inter junction";
      }
    } else {
      ADEBUG << "Fail to get junctions from base_map.";
    }
  }
  ADEBUG << "injector_->is_adc_in_junction_ ="
         << injector_->is_adc_in_junction_;
}

bool STBoundaryMapper::IgnoreFrontHighSpeedObstacle(
    const Obstacle& obstacle,
    const util::MovingObstacleType moving_obstacle_type) {
  bool ret = false;
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  const auto& obstacle_type = obstacle.Perception().type();
  bool is_front_car =
      (obs_sl.start_s() - adc_sl_boundary_.end_s() >
       speed_bounds_config_.min_skip_obs_distance()) &&
      (perception::PerceptionObstacle::VEHICLE == obstacle_type);
  bool is_front_obs =
      (obs_sl.start_s() - adc_sl_boundary_.end_s() >
       speed_bounds_config_.min_skip_obs_distance_for_no_vehicle()) &&
      (perception::PerceptionObstacle::VEHICLE != obstacle_type);
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  bool is_high_speed_obs =
      obstacle.speed() > speed_bounds_config_.min_skip_obs_speed();
  bool is_lane_borrow = util::IsLaneBorrow(injector_->planning_context());
  bool Is_change_lane = util::IsLaneChange(injector_->planning_context());
  if ((is_front_car || is_front_obs) &&
      obstacle.speed() > adc_speed * FLAGS_ignore_obstacle_speed_coeff &&
      !(is_lane_borrow || Is_change_lane) && is_high_speed_obs) {
    const double obs_theta = obstacle.Perception().theta();
    const double adc_heading =
        injector_->vehicle_state()->vehicle_state().heading();
    double theta_diff =
        century::common::math::NormalizeAngle(obs_theta - adc_heading);
    double relative_speed_s =
        obstacle.speed() * std::cos(theta_diff) - adc_speed;
    if (relative_speed_s > kSpeedDiffBuffer &&
        // At present, the acceleration information of obstacles is not
        // stable, so acceleration judgment will not to do temporarily.
        // obstacle.acceleration() >
        //     -FLAGS_ignore_obstacle_acceleration_buffer &&
        (util::LEFT_FORWARD == moving_obstacle_type ||
         util::RIGHT_FORWARD == moving_obstacle_type ||
         util::STRAIGHT_FORWARD == moving_obstacle_type)) {
      ADEBUG << "obs_perception_id[" << obstacle.Perception().id() << "] type["
             << PerceptionObstacle_Type_Name(obstacle_type)
             << "] is faster than ADC and is driving in the same direction. "
                "IGNORE it.";
      ret = true;
    }
  }
  return ret;
}

bool STBoundaryMapper::IgnoreObstacleForRadicalDecision(
    const Obstacle& obstacle) {
  bool ret = false;
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_) {
    const double adc_center_s =
        (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
    ADEBUG << "obstacle Id  = " << obstacle.Id()
           << "\nobs_end_s = " << obs_sl.end_s()
           << ", adc_center_s = " << adc_center_s
           << "\nobs_end_l = " << obs_sl.end_l()
           << ", obs_start_l = " << obs_sl.start_l()
           << ", adc_start_l = " << adc_sl_boundary_.start_l()
           << ", adc_end_l=" << adc_sl_boundary_.end_l();
    if ((obs_sl.end_s() < adc_center_s) ||
        (!(obs_sl.end_l() > adc_sl_boundary_.start_l() - kLatBuffer &&
           obs_sl.start_l() < adc_sl_boundary_.end_l() + kLatBuffer) &&
         obs_sl.end_s() < adc_sl_boundary_.end_s())) {
      ADEBUG << "skip obstacle : " << obstacle.Id();
      ret = true;
    }
  }

  // skip fast obstacles driving in the same direction.
  // Use the heading angle diff to judge whether the obstacle ahead is
  // driving in the same direction.
  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_) {
    FLAGS_ignore_obstacle_speed_coeff =
        FLAGS_ignore_obstacle_speed_coeff_radical;
  } else {
    FLAGS_ignore_obstacle_speed_coeff = kIgnoreObstacleSpeedCoeff;
  }
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  double break_distance =
      adc_speed * adc_speed * 0.5 / kComfortableAcc + kBreakBuff;
  // ADEBUG << "break_distance = " << break_distance;
  // Get obstacle moving type
  util::MovingObstacleType moving_obstacle_type = util::GetMovingObstacleType(
      &obstacle, injector_->vehicle_state()->vehicle_state(), reference_line_);
  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_ &&
      (obs_sl.start_s() - adc_sl_boundary_.end_s() > break_distance) &&
      obstacle.speed() > adc_speed * FLAGS_ignore_obstacle_speed_coeff &&
      (util::LEFT_FORWARD == moving_obstacle_type ||
       util::RIGHT_FORWARD == moving_obstacle_type ||
       util::STRAIGHT_FORWARD == moving_obstacle_type)) {
    // ADEBUG << "obs_perception_id[" << obs_perception_id << "] type["
    //        << PerceptionObstacle_Type_Name(obstacle_type)
    //        << "] is faster than ADC and is driving in the same direction. "
    //           "IGNORE it.";
    ret = true;
  }
  return ret;
}

bool STBoundaryMapper::IsNeedToIgnoreObstacles(
    const Obstacle& obstacle, const bool is_adc_high_road_right_beginning,
    const bool is_high_right_in_intersection, const bool is_need_to_turn) {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  // skip backside obstacles.
  double obs_front_s =
      obstacle.IsStatic()
          ? obs_sl.end_s() + FLAGS_min_stop_distance_obstacle
          : obs_sl.end_s() +
                std::max(FLAGS_min_stop_distance_obstacle,
                         kLongitudinalTimeToCollision * obstacle.speed());

  bool is_can_skip_obs = false;
  if (!obstacle.IsStatic()) {
    is_can_skip_obs =
        IsCanSkipObstacle(obstacle, is_adc_high_road_right_beginning);
    // AINFO << "is_can_skip_obs = " << is_can_skip_obs;
    // straight lane, skip back side obs
    if (is_high_right_in_intersection && !is_need_to_turn &&
        obs_sl.end_s() < adc_sl_boundary_.start_s()) {
      is_can_skip_obs = true;
      // AINFO << "skip";
      return true;
    }
  }

  if (is_can_skip_obs) {
    // ADEBUG << "can skip back obstacle, id[" << obs_perception_id << "]";
    obs_front_s = obs_sl.end_s();
  }
  // AINFO << "OBS_SOEED = " << obstacle.speed();
  const auto& obstacle_type = obstacle.Perception().type();
  if (obs_front_s < adc_sl_boundary_.start_s()) {
    // ADEBUG << "obs_perception_id[" << obs_perception_id << "] type["
    //        << PerceptionObstacle_Type_Name(obstacle_type)
    //        << "] behind ADC. SKIP";
    // ADEBUG << "obs_front_s = " << obs_front_s;
    // ADEBUG << ":adc_sl_boundary_.end_s() = " << adc_sl_boundary_.end_s();
    return true;
  }

  // skip obstacles ahead the maximum path distance.
  if (obs_sl.start_s() - adc_sl_boundary_.end_s() >
      FLAGS_look_forward_long_distance) {
    // ADEBUG << "obs_perception_id[" << obs_perception_id << "] type["
    //        << PerceptionObstacle_Type_Name(obstacle_type)
    //        << "] far in front of ADC. SKIP";
    return true;
  }

  const double adc_speed = injector_->vehicle_state()->linear_velocity();

  util::MovingObstacleType moving_obstacle_type = util::GetMovingObstacleType(
      &obstacle, injector_->vehicle_state()->vehicle_state(), reference_line_);

  if (IgnoreObstacleForRadicalDecision(obstacle)) {
    return true;
  }

  if (IgnoreFrontHighSpeedObstacle(obstacle, moving_obstacle_type)) {
    return true;
  }
  // can skip obs if height is too large
  if(obstacle.IsHigherObs()){
    AINFO<<"IGNORE obstacle because it is higher than IGV.";
    return true;
  }
  if (hdmap::Lane::PLAY_STREET == reference_line_info_->GetLaneType() &&
      perception::PerceptionObstacle::PEDESTRIAN == obstacle_type &&
      !obstacle.IsStatic()) {
    double distance_buffer =
        InterpolationLookUp(adc_speed, speed_bounds_config_.adc_min_speed(),
                            speed_bounds_config_.adc_max_speed(),
                            speed_bounds_config_.obstacle_min_distance(),
                            speed_bounds_config_.obstacle_max_distance());
    if (obs_sl.end_l() < adc_sl_boundary_.start_l() - distance_buffer ||
        obs_sl.start_l() > adc_sl_boundary_.end_l() + distance_buffer) {
      // ADEBUG << "Outside the security buffer,skip. obstacle id:["
      //        << obs_perception_id << "],distance buffer: [" <<
      //        distance_buffer
      //        << "],ADC speed:[" << adc_speed << "], ADC start_l:["
      //        << adc_sl_boundary_.start_l() << "], ADC end_l: ["
      //        << adc_sl_boundary_.end_l() << "], obstacle start_l:["
      //        << obs_sl.start_l() << "], obstacle end_l:[" << obs_sl.end_l()
      //        << "].";
      return true;
    }
  }
  return false;
}

double STBoundaryMapper::GetOvertakeSBuffer(
    const Obstacle& obstacle, const bool is_minway_entry_boundary) {
  double obs_speed = std::fabs(obstacle.speed());
  double adc_speed = std::fabs(injector_->vehicle_state()->linear_velocity());
  double min_speed = 0.0;
  // If the speed of the ADC is higher than that of the OBS, overtaking will
  //  be given priority, and the overtaking buffer will be slightly smaller.
  if (adc_speed > kSpeedCoeff * obs_speed) {
    min_speed = obs_speed;
  } else {
    min_speed = adc_speed;
  }
  double overtake_distance_s = InterpolationLookUp(
      min_speed, FLAGS_play_street_speed_limit,
      FLAGS_planning_upper_speed_limit, FLAGS_min_overtake_longitude_buffer,
      FLAGS_max_overtake_longitude_buffer);
  // AINFO << "speed overtake buffer =" << overtake_distance_s;
  if (FLAGS_enable_yield_for_min_lateral_obs) {
    // no lateral overlap and in cutin or cross obs.
    const double adc_s =
        (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;

    const auto& obs_sl_boundary = obstacle.PerceptionSLBoundary();
    const auto& path_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) * 0.5 -
            adc_s);

    Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                                vehicle_param_.back_edge_to_center()) *
                                   0.5,
                               (vehicle_param_.left_edge_to_center() -
                                vehicle_param_.right_edge_to_center()) *
                                   0.5);
    ego_center_map_frame.SelfRotate(path_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

    // Compute the ADC bounding box.
    Box2d adc_box(ego_center_map_frame, path_point.theta(),
                  vehicle_param_.length(), vehicle_param_.width());

    // collision detection based on SLBoundary.
    SLBoundary adc_sl_boundary;
    if (!reference_line_.GetSLBoundary(adc_box, &adc_sl_boundary)) {
      adc_sl_boundary.set_start_s(adc_sl_boundary_.start_s());
      adc_sl_boundary.set_end_s(adc_sl_boundary_.end_s());
      adc_sl_boundary.set_start_l(adc_sl_boundary_.start_l());
      adc_sl_boundary.set_end_l(adc_sl_boundary_.end_l());
    }

    double lateral_diff =
        std::max(obs_sl_boundary.start_l() - adc_sl_boundary.end_l(),
                 adc_sl_boundary.start_l() - obs_sl_boundary.end_l());
    ADEBUG << "obs start_l = " << obs_sl_boundary.start_l()
           << ", obs end_l = " << obs_sl_boundary.end_l()
           << ", adc start_l = " << adc_sl_boundary_.start_l()
           << ", adc end_l = " << adc_sl_boundary_.end_l();

    double buffer_for_cross_obs = 0.0;
    bool is_front_obs = adc_s < obs_sl_boundary.end_s();
    ADEBUG << "lateral_diff = " << lateral_diff
           << ", overtake_distance_s = " << overtake_distance_s
           << ", is_front_obs = " << is_front_obs;
    if (is_minway_entry_boundary && is_front_obs) {
      // save computing power,so written hear.
      // if is close to path and has lateral speed,no overtake.
      if (lateral_diff < speed_bounds_config_.min_overtake_lateral_buffer()) {
        buffer_for_cross_obs = kMaxOvertakeLongitudinalBuffer;
      } else {
        buffer_for_cross_obs = InterpolationLookUp(
            lateral_diff, speed_bounds_config_.min_overtake_lateral_buffer(),
            speed_bounds_config_.max_overtake_lateral_buffer(),
            FLAGS_max_overtake_longitude_buffer,
            FLAGS_min_overtake_longitude_buffer);
      }
    } else {
      buffer_for_cross_obs = 0.0;
    }
    overtake_distance_s = overtake_distance_s + buffer_for_cross_obs;
    ADEBUG << "buffer_for_cross_obs = " << buffer_for_cross_obs
           << ", overtake_distance_s = " << overtake_distance_s;
  } else {
    overtake_distance_s = 0.0;
  }
  // reverse obs and lonoverlap ,prefer overtake.
  double obs_center_s = (obstacle.PerceptionSLBoundary().start_s() +
                         obstacle.PerceptionSLBoundary().end_s()) *
                        0.5;
  const auto& ref_point = reference_line_.GetReferencePoint(obs_center_s);
  const auto& obs_velocity = obstacle.Perception().velocity();
  double obs_lon_speed =
      common::math::Vec2d::CreateUnitVec2d(ref_point.heading())
          .InnerProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  if (obs_lon_speed < 0.0 && adc_sl_boundary_.end_s() > obs_sl.start_s()) {
    // AINFO << "prefer overtake";
    overtake_distance_s = 0.0;
  }
  // ADEBUG << "obs_reverse_speed_ = " << obs_reverse_speed_;
  return overtake_distance_s;
}

void STBoundaryMapper::GetLaneHeading(
    const std::vector<hdmap::LaneSegment>& lane_segments,
    const hdmap::LaneInfoConstPtr& adc_lane, const int adc_lane_precessor_index,
    const int adc_lane_sucessor_index, double* precessor_lane_heading,
    double* sucessor_lane_heading,
    common::math::Vec2d* sucessor_lane_first_point,
    common::math::Vec2d* sucessor_lane_second_point) {
  if (adc_lane_precessor_index >= 0) {
    const auto& precessor_lane = lane_segments[adc_lane_precessor_index];
    *precessor_lane_heading =
        precessor_lane.lane->headings().back() * kQuadrantAngle / M_PI_2;
    ADEBUG << "precessor_lane.id = " << precessor_lane.lane->lane().id().id()
           << ", precessor_lane_heading = " << *precessor_lane_heading;
    injector_->adc_precessor_lane_heading_ = *precessor_lane_heading;
  } else {
    // if adc first has no precessor lane.
    if (injector_->adc_precessor_lane_heading_ > kMaxAngle) {
      injector_->adc_precessor_lane_heading_ =
          adc_lane->headings().front() * kQuadrantAngle / M_PI_2;
      ADEBUG << "use adc lane front heading = "
             << injector_->adc_precessor_lane_heading_;
    }
  }
  ADEBUG << "injector_->adc_precessor_lane_heading_ = "
         << injector_->adc_precessor_lane_heading_;
  if ((adc_lane_sucessor_index) < static_cast<int>(lane_segments.size())) {
    const auto& sucessor_lane = lane_segments[adc_lane_sucessor_index];
    ADEBUG << "sucessor_lane.id = " << sucessor_lane.lane->lane().id().id();

    *sucessor_lane_heading =
        sucessor_lane.lane->headings().front() * kQuadrantAngle / M_PI_2;
    sucessor_lane_first_point->set_x(sucessor_lane.lane->points().front().x());
    sucessor_lane_first_point->set_y(sucessor_lane.lane->points().front().y());
    sucessor_lane_second_point->set_x(
        sucessor_lane.lane->points().front().x() +
        std::cos(*sucessor_lane_heading * M_PI_2 / kQuadrantAngle));
    sucessor_lane_second_point->set_y(
        sucessor_lane.lane->points().front().y() +
        std::sin(*sucessor_lane_heading * M_PI_2 / kQuadrantAngle));
  } else {
    *sucessor_lane_heading =
        adc_lane->headings().back() * kQuadrantAngle / M_PI_2;
    sucessor_lane_first_point->set_x(adc_lane->points().back().x());
    sucessor_lane_first_point->set_y(adc_lane->points().back().y());
    sucessor_lane_second_point->set_x(
        adc_lane->points().back().x() +
        std::cos(*sucessor_lane_heading * M_PI_2 / kQuadrantAngle));
    sucessor_lane_second_point->set_y(
        adc_lane->points().back().y() +
        std::sin(*sucessor_lane_heading * M_PI_2 / kQuadrantAngle));
  }
  ADEBUG << "sucessor_lane_heading = " << *sucessor_lane_heading;
}

bool STBoundaryMapper::CheckHighRightOnIntersection(bool* at_right_merage_lane,
                                                    bool* at_left_merage_lane,
                                                    bool* is_direct_turn) {
  bool is_high_right_in_intersection = false;
  if (reference_line_info_->IsAdcInCommonJunction()) {
    double adc_lane_heading = 0.0;
    const auto& adc_lane =
        reference_line_info_->LocateLaneInfo(adc_sl_boundary_.end_s());
    const auto& adc_lane_headings_size = adc_lane->headings().size();
    size_t adc_lane_headings_half_index =
        static_cast<size_t>((adc_lane_headings_size - 1) * 0.5);
    ADEBUG << "adc_lane_id = " << adc_lane->id().id()
           << ", adc_lane_headings_size = " << adc_lane_headings_size
           << ", adc_lane_headings_half_index = "
           << adc_lane_headings_half_index;
    adc_lane_heading = adc_lane->headings()[adc_lane_headings_half_index] *
                       kQuadrantAngle / M_PI_2;

    ADEBUG << "adc_lane_heading = " << adc_lane_heading
           << ", adc_lane_front_heading = "
           << adc_lane->headings().front() * kQuadrantAngle / M_PI_2
           << ", adc_lane_end_heading = "
           << adc_lane->headings().back() * kQuadrantAngle / M_PI_2;
    const auto& adc_lane_index =
        reference_line_info_->reference_line().map_path().GetLaneIndexFromS(
            adc_sl_boundary_.end_s());

    const auto& lane_segments =
        reference_line_info_->reference_line().map_path().lane_segments();
    const auto adc_lane_sucessor_index = adc_lane_index.id + kOneStep;
    const auto adc_lane_precessor_index = adc_lane_index.id - kOneStep;
    common::math::Vec2d sucessor_lane_first_point, sucessor_lane_second_point;
    double sucessor_lane_heading = 0.0, precessor_lane_heading = 0.0;
    ADEBUG << "adc_lane_sucessor_index = " << adc_lane_sucessor_index
           << ", adc_lane_precessor_index = " << adc_lane_precessor_index;
    GetLaneHeading(lane_segments, adc_lane, adc_lane_precessor_index,
                   adc_lane_sucessor_index, &precessor_lane_heading,
                   &sucessor_lane_heading, &sucessor_lane_first_point,
                   &sucessor_lane_second_point);

    double sucessor_and_adc_lane_diff =
        sucessor_lane_heading - adc_lane_heading;
    double adc_and_precessor_lane_diff =
        adc_lane_heading - injector_->adc_precessor_lane_heading_;
    double sucessor_and_precessor_lane_diff =
        sucessor_lane_heading - injector_->adc_precessor_lane_heading_;

    if (std::fabs(sucessor_and_adc_lane_diff) <
            speed_bounds_config_.min_angle_diff() &&
        std::fabs(adc_and_precessor_lane_diff) <
            speed_bounds_config_.min_angle_diff() &&
        std::fabs(sucessor_and_precessor_lane_diff) <
            speed_bounds_config_.min_angle_diff()) {
      is_high_right_in_intersection = true;
    }
    // if no straignt lane,check is merage area or direct turn area.
    if (!is_high_right_in_intersection) {
      // check is direct turn lane:
      double adc_lane_front_heading =
          adc_lane->headings().front() * kQuadrantAngle / M_PI_2;
      double adc_lane_back_heading =
          adc_lane->headings().back() * kQuadrantAngle / M_PI_2;
      at_turn_area_in_junction_ =
          std::fabs(adc_lane_front_heading - adc_lane_back_heading) >
          kDiffHeading;
      ADEBUG << "at_turn_area_in_junction_ = " << at_turn_area_in_junction_;
      at_left_turn_area_in_junction_ =
          common::math::NormalizeAngle(adc_lane->headings().back() -
                                       adc_lane->headings().front()) > M_PI_4;
      at_right_turn_area_in_junction_ =
          common::math::NormalizeAngle(adc_lane->headings().back() -
                                       adc_lane->headings().front()) < -M_PI_4;
      // AINFO << "adc_lane_back_heading = " << adc_lane_back_heading;
      // AINFO << "adc_lane_front_heading = " << adc_lane_front_heading;
      // AINFO << "at_left_turn_area_in_junction_ = "
      //       << at_left_turn_area_in_junction_;
      // AINFO << "at_RIGHT_turn_area_in_junction_ ="
      //       << at_right_turn_area_in_junction_;
      // AINFO << "sucessor_and_adc_lane_diff = " << sucessor_and_adc_lane_diff
      //       << ", adc_and_precessor_lane_diff = " <<
      //       adc_and_precessor_lane_diff
      //       << ", sucessor_and_precessor_lane_diff = "
      //       << sucessor_and_precessor_lane_diff;
      // in direct turn lane.

      *is_direct_turn = at_turn_area_in_junction_;
      // AINFO << "*is_direct_turn = " << *is_direct_turn;
      if (!at_turn_area_in_junction_) {
        // in merage lane.
        const common::math::Vec2d adc_lane_first_point(
            adc_lane->points().front().x(), adc_lane->points().front().y());
        double dx = sucessor_lane_first_point.x() - adc_lane_first_point.x();
        double dy = sucessor_lane_first_point.y() - adc_lane_first_point.y();
        double heading_low_right_lane = std::atan2(dy, dx) * kQuadrantAngle /
                                        M_PI_2;  // calculate the heading

        double side = common::math::CrossProd(adc_lane_first_point,
                                              sucessor_lane_first_point,
                                              sucessor_lane_second_point);
        ADEBUG << "heading_low_right_lane = " << heading_low_right_lane
               << ", side = " << side;
        // right merage lane.
        if (side > 0.0) {
          *at_right_merage_lane = true;
          ADEBUG << "in lane left ,turn right ";
        } else {
          // left merage lane.
          *at_left_merage_lane = true;
          ADEBUG << "in lane right ,turn left ";
        }
      }
    }
  } else {
    is_high_right_in_intersection = false;
    injector_->adc_precessor_lane_heading_ = std::numeric_limits<double>::max();
  }
  return is_high_right_in_intersection;
}

bool STBoundaryMapper::IsReachTurnLane(bool* reach_right_lane,
                                       bool* reach_left_lane) {
  bool is_reach_turn_lane = false;
  bool is_path_kappa_left_turn = false;
  bool is_path_kappa_right_turn = false;
  int count_num = 0;
  double kappa_sum = 0.0;
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  for (size_t i = 0; i < discretized_path.size(); ++i) {
    const auto& frenet_point = frenet_path[i];
    const auto& reference_point =
        reference_line_info_->reference_line().GetReferencePoint(
            frenet_point.s());
    // ADEBUG << "reference_point.kappa = " << reference_point.kappa();
    double ref_point_kappa = reference_point.kappa();
    // AINFO << "ref_point_kappa = " << ref_point_kappa;
    // AINFO << "first_turn_point_s_ = " << first_turn_point_s_;
    if (frenet_point.s() > first_turn_point_s_) {
      // AINFO << "break s =" << discretized_path[i].s();
      break;
    }
    // check path first turn
    if (ref_point_kappa > kMinKappa) {
      // AINFO << "LEFT_TURN";
      ++count_num;
      kappa_sum += ref_point_kappa;
      is_path_kappa_left_turn = true;
      break;
    } else if (ref_point_kappa < -kMinKappa) {
      // AINFO << "RIGHT_TURN";
      ++count_num;
      kappa_sum += ref_point_kappa;
      is_path_kappa_right_turn = true;
      break;
    } else {
      // AINFO << "NO_TURN";
    }
  }

  const auto& adc_lane =
      reference_line_info_->LocateLaneInfo(adc_sl_boundary_.end_s());
  adc_lane_heading_diff_ =
      common::math::NormalizeAngle(
          (adc_lane->headings().back() - adc_lane->headings().front())) *
      kQuadrantAngle / M_PI_2;
  // AINFO << "adc_lane_heading_diff_ = " << adc_lane_heading_diff_;
  const auto& adc_lane_index =
      reference_line_info_->reference_line().map_path().GetLaneIndexFromS(
          adc_sl_boundary_.end_s());

  const auto& lane_segments =
      reference_line_info_->reference_line().map_path().lane_segments();
  const auto adc_lane_sucessor_index = adc_lane_index.id + kOneStep;
  if ((adc_lane_sucessor_index) < static_cast<int>(lane_segments.size())) {
    const auto& sucessor_lane = lane_segments[adc_lane_sucessor_index];
    // AINFO << "sucessor_lane.id = " << sucessor_lane.lane->lane().id().id();

    double sucessor_lane_heading = sucessor_lane.lane->headings().front();
    double successor_lane_back_heading = sucessor_lane.lane->headings().back();
    successor_lane_heading_diff_ =
        common::math::NormalizeAngle(successor_lane_back_heading -
                                     sucessor_lane_heading) *
        kQuadrantAngle / M_PI_2;
    // AINFO << "successor_lane_heading_diff_ = " <<
    // successor_lane_heading_diff_;
  } else {
    successor_lane_heading_diff_ = 0.0;
  }
  // need path has kappa and lane has turn to make sure turn direct.
  if (is_path_kappa_left_turn &&
      (adc_lane_heading_diff_ > kDiffHeading ||
       successor_lane_heading_diff_ > kDiffHeading)) {
    *reach_left_lane = true;
    is_reach_turn_lane = true;
    // AINFO << "path_kappa_left_turn";
  }
  if (is_path_kappa_right_turn &&
      (adc_lane_heading_diff_ < -kDiffHeading ||
       successor_lane_heading_diff_ < -kDiffHeading)) {
    *reach_right_lane = true;
    is_reach_turn_lane = true;
    // AINFO << "path_kappa_right_turn";
  }
  return is_reach_turn_lane;
}

void STBoundaryMapper::SetRoadRightInfo(
    const bool is_high_right_in_intersection, const bool at_left_merage_lane,
    const bool at_right_merage_lane, bool* is_direct_turn,
    bool* is_need_to_turn) {
  if (reference_line_info_->IsAdcInCommonJunction()) {
    // intersection and no high right.
    if (!(*is_direct_turn)) {
      at_left_turn_ = at_left_merage_lane;
      at_right_turn_ = at_right_merage_lane;
      *is_need_to_turn = path_turn_right_ || path_turn_left_ ||
                         at_right_turn_ || at_left_turn_;
    } else {
      at_left_turn_ = at_left_turn_area_in_junction_;
      at_right_turn_ = at_right_turn_area_in_junction_;
      *is_need_to_turn = path_turn_right_ || path_turn_left_ ||
                         at_right_turn_ || at_left_turn_;
    }

    reference_line_info_->SetIsInHighRoadRightLaneOnIntersection(
        is_high_right_in_intersection, at_left_turn_, *is_direct_turn);
  } else {
    // no intersection get merage area or turn area.
    bool reach_right_lane = false;
    bool reach_left_lane = false;
    bool is_reach_turn_lane =
        IsReachTurnLane(&reach_right_lane, &reach_left_lane);
    if (is_reach_turn_lane) {
      at_left_turn_ = reach_left_lane;
      at_right_turn_ = reach_right_lane;
      *is_need_to_turn = path_turn_right_ || path_turn_left_ ||
                         at_right_turn_ || at_left_turn_;
      *is_direct_turn = true;
    }
    // no direct turn and no in merage
    ADEBUG << "is_in_merage_ = " << is_in_merage_;
    if ((at_left_turn_ || at_right_turn_) && !is_in_merage_) {
      is_reach_turn_lane = true;
    }
    ADEBUG << "is_reach_turn_lane = " << is_reach_turn_lane;
    reference_line_info_->SetIsReachTurnLane(is_reach_turn_lane, at_left_turn_,
                                             at_right_turn_);
  }

  ADEBUG << "is_direct_turn = " << *is_direct_turn
         << ", is_need_to_turn = " << *is_need_to_turn
         << ", path_turn_right = " << path_turn_right_
         << ", path_turn_left = " << path_turn_left_
         << ", at_right_turn = " << at_right_turn_
         << ", at_left_turn = " << at_left_turn_;
  reference_line_info_->SetTurnInfo(*is_direct_turn, *is_need_to_turn,
                                    at_left_turn_, at_right_turn_,
                                    path_turn_left_, path_turn_right_);
}

Status STBoundaryMapper::PreCheckForComputeSTBoundary(
    PathDecision* path_decision) {
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  if (nullptr == path_decision) {
    const std::string msg = "The path_decision pointer is null.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status STBoundaryMapper::MakeCloseStopObstacleDecision(
    const ObjectDecisionType& closest_stop_decision,
    Obstacle* const closest_stop_obstacle) {
  if (closest_stop_obstacle) {
    double key_point_s =
        closest_stop_obstacle->PerceptionSLBoundary().start_s() +
        closest_stop_decision.stop().distance_s();

    ADEBUG << "obstacle id: " << closest_stop_obstacle->Id() << ", start_s: "
           << closest_stop_obstacle->PerceptionSLBoundary().start_s()
           << ", distance_s: " << closest_stop_decision.stop().distance_s()
           << ", key_point_s: " << key_point_s
           << ", adc_end_s: " << adc_sl_boundary_.end_s();
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    if (adc_sl_boundary_.end_s() >
            key_point_s + speed_bounds_config_.s_buffer_to_enable_estop() &&
        adc_speed < speed_bounds_config_.max_speed_to_enable_estop()) {
      // need emergency stop.
      reference_line_info_->SetEStopStatus(true, "stop fence is behind ADC.");
    }

    bool success =
        MapStopDecision(closest_stop_obstacle, closest_stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) {
  // Sanity checks.
  CHECK_GT(planning_max_time_, 0.0);

  const auto pre_check_ret = PreCheckForComputeSTBoundary(path_decision);
  if (!pre_check_ret.ok()) {
    return pre_check_ret;
  }

  CalcuMaxCollisionPathLength();

  // check path is on lane,no use remove.
  // bool is_path_on_lane = IsPathOnHighRight();
  bool is_path_on_lane = false, at_right_merage_lane = false,
       at_left_merage_lane = false;

  // check adc is high in intersection.
  bool is_direct_turn = false;
  bool is_high_right_in_intersection = CheckHighRightOnIntersection(
      &at_right_merage_lane, &at_left_merage_lane, &is_direct_turn);

  // AINFO << "is_high_right_in_intersection = " <<
  // is_high_right_in_intersection;

  // path is need to turn or road need to turn
  bool is_need_to_turn = IsNeedToTurn(is_path_on_lane);
  // straight lane（ <3 degreee） is_high_right_in_intersection = true;
  // merage lane（3 -10 degreee）
  // direct turn lane（ >3 degreee）

  SetRoadRightInfo(is_high_right_in_intersection, at_left_merage_lane,
                   at_right_merage_lane, &is_direct_turn, &is_need_to_turn);

  bool is_adc_high_road_right_beginning = false;
  if (!is_need_to_turn) {
    is_adc_high_road_right_beginning = true;
  }

  // Judge is in traffic_inter junction
  SetInnerJunctionStatus();

  // only first to calculate
  StopForRightTurn(path_decision);

  // only first to calculate ,no consider laneborrow return back side obs.
  CanSkipObstaclesInLaneborrowReturn(path_decision);
  // cal diagonal_heading_
  const auto& center_s =
      (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
  const auto& reference_point =
      reference_line_info_->reference_line().GetReferencePoint(center_s);
  diagonal_heading_ = reference_point.heading();
  // AINFO<<"diagonal_heading_ = "<<diagonal_heading_;
  if(reference_line_info_->IsInDiagonalRoad()){
    diagonal_heading_ = reference_line_info_->DiagonalRoadHeading();
    // AINFO<<"diagonal_heading_==="<<diagonal_heading_;
  }

  // Go through every obstacle.
  Obstacle* closest_stop_obstacle = nullptr;
  ObjectDecisionType closest_stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (ptr_obstacle == nullptr) {
      // ADEBUG << "Current obstacle pointer is null.";
      continue;
    }

    if (!skip_high_speed_bicycle_ids_.empty() &&
        PerceptionObstacle::BICYCLE == ptr_obstacle->Perception().type() &&
        (std::find(skip_high_speed_bicycle_ids_.begin(),
                   skip_high_speed_bicycle_ids_.end(),
                   ptr_obstacle_item->Id()) !=
         skip_high_speed_bicycle_ids_.end())) {
      // AINFO << "skip high speed bicycle,bcause no reach or can overtake.";
      continue;
    }
    if (!skip_back_laneborrow_obs_ids_.empty() &&
        (std::find(skip_back_laneborrow_obs_ids_.begin(),
                   skip_back_laneborrow_obs_ids_.end(),
                   ptr_obstacle_item->Id()) !=
         skip_back_laneborrow_obs_ids_.end())) {
      // AINFO << "skip high speed bicycle,bcause no reach or can overtake.";
      continue;
    }
    if (IsNeedToIgnoreObstacles(*ptr_obstacle, is_adc_high_road_right_beginning,
                                is_high_right_in_intersection,
                                is_need_to_turn)) {
      continue;
    }

    // If no longitudinal decision has been made, then plot it onto ST-graph.

    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    // If there is a longitudinal decision, then fine-tune boundary.
    const auto& decision = ptr_obstacle->LongitudinalDecision();
    // ADEBUG << "No mapping for ignore decision: " << decision.DebugString();
    if (decision.has_stop()) {
      // 1. Store the closest stop fence info.
      GetClosestStopDecision(ptr_obstacle, decision, &closest_stop_obstacle,
                             &closest_stop_decision, &min_stop_s);

    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }
  for (const auto& obs : ready_to_add_obstalces_) {
    path_decision->AddObstacle(obs);
  }
  return MakeCloseStopObstacleDecision(closest_stop_decision,
                                       closest_stop_obstacle);
}

bool STBoundaryMapper::IsReachRightTurn(double* first_turn_s) {
  const auto& lane_segments =
      reference_line_info_->reference_line().map_path().lane_segments();
  bool has_turn = false;
  for (const auto& lane_segment : lane_segments) {
    const auto& lane_trun_type = lane_segment.lane->lane().turn();
    // AINFO << "lane_trun_type = " <<
    // hdmap::Lane_LaneTurn_Name(lane_trun_type);
    if (hdmap::Lane::NO_TURN != lane_trun_type) {
      has_turn = true;
      break;
    }
  }
  if (!has_turn) {
    return false;
  }

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  size_t path_step = kMinPathStep;
  for (size_t i = 0; i < discretized_path.size(); i += path_step) {
    const auto& frenet_point = frenet_path[i];
    // adc front 0-30m
    // AINFO << "discretized_path[i].s() = " << discretized_path[i].s();
    if (discretized_path[i].s() > kConsiderLonDistance) {
      break;
    }
    // use map information
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    reference_line_.GetLaneFromS(frenet_point.s(), &lanes);
    if (!lanes.empty() && lanes.front() != nullptr) {
      const auto& lane = lanes.front()->lane();
      if (lane.has_turn()) {
        if (Lane::LEFT_TURN == lane.turn()) {
          // ADEBUG << "LEFT_TURN";
        } else if (Lane::RIGHT_TURN == lane.turn()) {
          *first_turn_s = discretized_path[i].s();
          AINFO << "REACHING  RIGHT TURN";
          return true;
        } else if (Lane::U_TURN == lane.turn()) {
          // ADEBUG << "U_TURN";
        } else {
          // ADEBUG << "no turn";
        }
      }
    }
  }
  return false;
}

void STBoundaryMapper::GetDangerousHighSpeedBicycle(
    PathDecision* path_decision) {
  if (!FLAGS_enable_yield_for_high_speed_bicycle) {
    return;
  }
  // AINFO << "===start test skip high speed bicycle===";
  double first_turn_s = std::numeric_limits<double>::max();
  if (!IsReachRightTurn(&first_turn_s)) {
    // AINFO << "NO REACHING RIGHT TURN";
    return;
  }

  first_turn_s = (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5 +
                 first_turn_s;
  // AINFO << "first_turn_s = " << first_turn_s;

  // AINFO << "check has bicycle in range";
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (ptr_obstacle == nullptr) {
      // ADEBUG << "Current obstacle pointer is null.";
      continue;
    }
    if (PerceptionObstacle::BICYCLE != ptr_obstacle->Perception().type()) {
      continue;
    }
    if (ptr_obstacle->IsStatic()) {
      continue;
    }

    double obstacle_end_s = ptr_obstacle->PerceptionSLBoundary().end_s(),
           obs_end_l = ptr_obstacle->PerceptionSLBoundary().end_l(),
           adc_start_l = adc_sl_boundary_.start_l();
    std::string hy_obs_id = std::to_string(ptr_obstacle->PerceptionId());
    double diff_l = adc_start_l - obs_end_l, hy_diff_l = diff_l;
    // AINFO << "diff_l_before = " << diff_l;
    hy_diff_l = adc_obstacles_l_diff_interval.HyValue(hy_obs_id, diff_l);
    // AINFO << "diff_l_after = " << hy_diff_l;
    if (hy_diff_l < 0.0 || hy_diff_l > kLateralBufferForBicycle) {
      // AINFO << "obstacle_" << ptr_obstacle->Id() << " is no right of ADC";
      continue;
    }

    if (obstacle_end_s < first_turn_s - kConsiderLonDistance ||
        obstacle_end_s > first_turn_s) {
      // AINFO << "obstacle_" << ptr_obstacle->Id() << " faraway from right turn
      // ";
      continue;
    }
    // roi : right & turn point back 0-30.
    double obstacle_speed = ptr_obstacle->LongitudinalSpeed(),
           obstacle_acceleration = ptr_obstacle->LongitudianlAcc();
    double obstacle_reach_s =
        obstacle_end_s + obstacle_speed * kTrajectoryTime +
        0.5 * obstacle_acceleration * kTrajectoryTime * kTrajectoryTime;
    double hy_speed =
        obstacles_speed_interval.HyValue(hy_obs_id, obstacle_speed);
    // ADEBUG << "obstacle_speed = " << obstacle_speed;
    // ADEBUG << "hy_speed = " << hy_speed;
    if (hy_speed < kLowBicycleSpeed) {
      obstacle_reach_s =
          obstacle_end_s + obstacle_speed * kTrajectoryTimeForLowSpeed +
          0.5 * obstacle_acceleration * kTrajectoryTimeForLowSpeed *
              kTrajectoryTimeForLowSpeed;
    }

    // if obs can't reach turn ,no consider.
    if (obstacle_reach_s < first_turn_s) {
      // AINFO << "obstacle_" << ptr_obstacle->Id() << " no reache right turn";
      skip_high_speed_bicycle_ids_.push_back(ptr_obstacle_item->Id());
      continue;
    }
    // AINFO << "dangerous obstacle id = " << ptr_obstacle_item->Id();
    // add obs which has collision risk.
    dangerous_high_speed_bicycle_ids_.push_back(ptr_obstacle_item->Id());
  }

  // AINFO << "dangerous_high_speed_bicycle_ids_.size = "
  //  << dangerous_high_speed_bicycle_ids_.size();
  if (!dangerous_high_speed_bicycle_ids_.empty()) {
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    double adc_breaking_distance = adc_speed * adc_speed * 0.5 / kLowDecel;
    // adc can break no enter turn,advance braking.
    if (first_turn_s - adc_sl_boundary_.end_s() > adc_breaking_distance) {
      // AINFO << "ADC is in straight road";
      const std::string stop_wall_id = "yield_high_speed_bicycle_stop_wall_1";
      std::vector<std::string> wait_for_obstacles;

      util::BuildStopDecision(stop_wall_id, first_turn_s, 0.0,
                              StopReasonCode::STOP_REASON_LOW_RIGHT,
                              wait_for_obstacles, "StopReasonADCLowRight",
                              frame_, reference_line_info_);
    } else {
      // AINFO << "ADC is in dangerous turn road";
      double stop_s = std::numeric_limits<double>::max();
      if (!GetNeastStopS(path_decision, dangerous_high_speed_bicycle_ids_,
                         &stop_s)) {
        AINFO << "dangerous high speed bicycle no collision with adc.";
        return;
      }

      // AINFO << "stop_s = " << stop_s;
      const std::string stop_wall_id = "yield_high_speed_bicycle_stop_wall_2";
      std::vector<std::string> wait_for_obstacles;

      util::BuildStopDecision(stop_wall_id, stop_s, 0.0,
                              StopReasonCode::STOP_REASON_LOW_RIGHT,
                              wait_for_obstacles, "StopReasonADCLowRight",
                              frame_, reference_line_info_);
    }
  }
}

bool STBoundaryMapper::GetNeastStopS(
    PathDecision* path_decision,
    std::vector<std::string> dangerous_high_speed_bicycle_ids,
    double* first_stop_s) {
  double min_s = std::numeric_limits<double>::max();
  bool get_neast_stop_s = false;
  for (const auto& id : dangerous_high_speed_bicycle_ids) {
    Obstacle* obstacle = path_decision->Find(id);
    if (obstacle == nullptr) {
      AERROR << "Current obstacle pointer is null.";
      continue;
    }
    double stop_s = std::numeric_limits<double>::max();
    if (GetStopS(*obstacle, &stop_s)) {
      AINFO << "get stop_s  = " << stop_s;
      if (stop_s < min_s) {
        min_s = stop_s;
        get_neast_stop_s = true;
      }
    }
  }
  *first_stop_s = min_s;
  if (get_neast_stop_s) {
    return true;
  } else {
    return false;
  }
}

bool STBoundaryMapper::GetStopS(const Obstacle& obstacle,
                                double* first_stop_s) {
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  size_t path_step = kMinPathStep;
  for (size_t i = 0; i < discretized_path.size(); i += path_step) {
    const auto& frenet_point = frenet_path[i];
    if (discretized_path[i].s() > kConsiderLonDistance) {
      break;
    }
    // Starting from the first point of the vehicle's position, find the point
    // in front that satisfies parking and is not a collision.
    double backward_distance = vehicle_param_.front_edge_to_center();
    double adc_heading = discretized_path[i].theta();
    double adc_x =
        discretized_path[i].x() + backward_distance * std::cos(adc_heading);
    double adc_y =
        discretized_path[i].y() + backward_distance * std::sin(adc_heading);
    double obs_x = obstacle.Perception().position().x();
    double obs_y = obstacle.Perception().position().y();
    double dx = adc_x - obs_x;
    double dy = adc_y - obs_y;
    double heading_obs_to_adc = std::atan2(dy, dx);  // calculate the heading

    double diff_theta_between_adc_and_obs = common::math::NormalizeAngle(
        obstacle.SpeedHeading() - heading_obs_to_adc);
    // AINFO << "heading_obs_to_adc =               "
    //       << heading_obs_to_adc * kDegrees / M_PI_2
    //       << "\n obstacle.SpeedHeading =          "
    //       << obstacle.SpeedHeading() * kDegrees / M_PI_2
    //       << "\n diff_theta_between_adc_and_obs = "
    //       << diff_theta_between_adc_and_obs * kDegrees / M_PI_2
    //       << " \n adc_heading=                    "
    //       << injector_->vehicle_state()->heading() * kDegrees / M_PI_2;

    if (std::fabs(diff_theta_between_adc_and_obs * kDegrees / M_PI_2) <
        kMinHeadingDiff) {
      AINFO << "get yield s ,break";

      // get prediction neast point
      double min_distance = std::numeric_limits<double>::max();
      int nearst_point_index = 0;
      for (int j = 0; j < obstacle.Trajectory().trajectory_point().size();
           j++) {
        const auto& trajectory_point =
            obstacle.Trajectory().trajectory_point(j);
        double obs_x = trajectory_point.path_point().x();
        double obs_y = trajectory_point.path_point().y();
        double diff_x = adc_x - obs_x;
        double diff_y = adc_y - obs_y;
        double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (distance < min_distance) {
          min_distance = distance;
          nearst_point_index = j;
        }
      }
      double min_d = std::numeric_limits<double>::max();
      // according obs prediction neast point ,get collision path point.
      for (size_t k = 0; k < discretized_path.size(); ++k) {
        const auto& point = discretized_path[k];
        double diff_x = point.x() - obstacle.Trajectory()
                                        .trajectory_point(nearst_point_index)
                                        .path_point()
                                        .x();
        double diff_y = point.y() - obstacle.Trajectory()
                                        .trajectory_point(nearst_point_index)
                                        .path_point()
                                        .y();
        double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (distance < min_d) {
          min_d = distance;
          // TODO(zongxingguo): change buffer.
          *first_stop_s = frenet_point.s() - kStopWallBuffer;
          if (adc_sl_boundary_.end_s() > frenet_point.s()) {
            AINFO << "can overtake";
            skip_high_speed_bicycle_ids_.push_back(obstacle.Id());
            return false;
          }
          AINFO << "nearst_point_index = " << nearst_point_index;
          AINFO << "k = " << k;
          AINFO << "GET STOP S = " << *first_stop_s;
          return true;
        }
      }
    }
  }
  // if no build stop wall ,ignore
  skip_high_speed_bicycle_ids_.push_back(obstacle.Id());
  return false;
}

bool STBoundaryMapper::IsRerutnReferenceLine(const bool is_left_borrow,
                                             const bool is_right_borrow) {
  const auto refer_point =
      reference_line_info_->reference_line().GetNearestReferencePoint(
          adc_sl_boundary_.end_s());
  double adc_ref_heading = refer_point.heading();
  // AINFO << "adc_ref_heading = " << adc_ref_heading;
  double adc_heading = injector_->vehicle_state()->heading();
  // AINFO << "adc_heading = " << adc_heading;
  double diff_heading =
      common::math::NormalizeAngle(adc_heading - adc_ref_heading);
  // AINFO << "diff_heading = " << diff_heading;
  double diff_angle = std::fabs(diff_heading * kDegrees / M_PI_2);
  // AINFO << "diff_angle = " << diff_angle;
  bool is_face_to_ref_line = false;
  if (is_left_borrow && ((diff_heading < 0.0 && diff_angle > kMinAngleDiff) ||
                         reference_line_info_->GetNeedTurnRight())) {
    is_face_to_ref_line = true;
  }
  if (is_right_borrow && diff_heading > 0.0 && diff_angle > kMinAngleDiff) {
    is_face_to_ref_line = true;
  }
  // AINFO << "is_face_to_ref_line = " << is_face_to_ref_line;
  return is_face_to_ref_line;
}

void STBoundaryMapper::CanSkipObstaclesInLaneborrowReturn(
    PathDecision* path_decision) {
  if (!FLAGS_enable_skip_back_side_in_laneborrow_return) {
    return;
  }
  // intersection may have no borrow.
  if (reference_line_info_->IsAdcInCommonJunction()) {
    return;
  }
  bool adc_in_ref_lane = reference_line_info_->IsAdcCenterInLane();
  // adc in ref lane , normal decision-making,can skip.
  if (adc_in_ref_lane) {
    return;
  }
  const auto& speed_data = reference_line_info_->speed_data();
  // only first collision compute
  if (speed_data.empty()) {
    bool is_lane_borrow = util::IsLaneBorrow(injector_->planning_context());
    if (!is_lane_borrow) {
      // AINFO << "NOT LANE BORROW";
      return;
    }
    // borrow and no in lane,so check use l.
    bool is_left_borrow = reference_line_info_->AdcSlBoundary().start_l() > 0.0;
    bool is_right_borrow = reference_line_info_->AdcSlBoundary().end_l() < 0.0;
    // AINFO << "is_left_borrow = " << is_left_borrow;
    // AINFO << "is_right_borrow = " << is_right_borrow;

    double adc_start_s = adc_sl_boundary_.start_s();

    // adc in laneborrow return stage.
    if (!IsRerutnReferenceLine(is_left_borrow, is_right_borrow)) {
      return;
    }

    double curr_lane_left_width = kHalfLaneWidth;
    double curr_lane_right_width = kHalfLaneWidth;
    if (!reference_line_.GetLaneWidth(adc_start_s, &curr_lane_left_width,
                                      &curr_lane_right_width)) {
      curr_lane_left_width = kHalfLaneWidth;
      curr_lane_right_width = kHalfLaneWidth;
    }

    for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
      Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
      if (ptr_obstacle == nullptr) {
        // ADEBUG << "Current obstacle pointer is null.";
        continue;
      }
      if (ptr_obstacle->IsStatic()) {
        continue;
      }

      double obstacle_end_s = ptr_obstacle->PerceptionSLBoundary().end_s();

      // all back obs ,and type is car
      if (obstacle_end_s > adc_start_s ||
          !PerceptionObstacle::VEHICLE ==
              ptr_obstacle_item->Perception().type()) {
        continue;
      }

      double obs_start_l = ptr_obstacle->PerceptionSLBoundary().start_l();
      double obs_end_l = ptr_obstacle->PerceptionSLBoundary().end_l();
      double obs_center_l = (obs_start_l + obs_end_l) * 0.5;

      // left borrow ,skip right back obs.
      if (is_left_borrow && obs_center_l < curr_lane_left_width &&
          obs_center_l > -curr_lane_right_width) {
        skip_back_laneborrow_obs_ids_.push_back(ptr_obstacle_item->Id());
      }
      // right borrow,skip left back obs.
      if (is_right_borrow && obs_center_l > -curr_lane_right_width &&
          obs_center_l < curr_lane_left_width) {
        skip_back_laneborrow_obs_ids_.push_back(ptr_obstacle_item->Id());
      }
      // fallback path ,no skip
    }
  }
}

void STBoundaryMapper::StopForRightTurn(PathDecision* path_decision) {
  const auto& speed_data = reference_line_info_->speed_data();
  // AINFO << "speed_data.size() = " << speed_data.size();
  if (speed_data.empty()) {
    GetDangerousHighSpeedBicycle(path_decision);
  }
}

bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  double st_stop_s = 0.0;
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();

  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),
              planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());
  boundary.set_obs_v(stop_obstacle->speed());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

Obstacle* STBoundaryMapper::AddReadyBoundaryObstacle(const Obstacle& obstacle,
                                                     size_t idx) {
  ready_to_add_obstalces_.push_back(obstacle);
  auto& new_obs = ready_to_add_obstalces_.back();
  auto obs_id = new_obs.Id();
  std::string add_idx = kAddedIdPrefix + std::to_string(idx);
  auto pos = obs_id.find('_');
  if (std::string::npos != pos) {
    obs_id.insert(pos, add_idx);
  } else {
    obs_id += add_idx;
  }
  new_obs.SetId(obs_id);
  return &new_obs;
}

void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) {
  if (FLAGS_use_st_drivable_boundary) {
    return;
  }
  std::vector<std::vector<STPoint>> lower_points_list;
  std::vector<std::vector<STPoint>> upper_points_list;
  if (!GetOverlapMultiBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                     &upper_points_list, &lower_points_list)) {
    ADEBUG << "NO COLLISION";
    return;
  }

  ADEBUG << "obstacle->Id() = " << obstacle->Id();

  for (size_t idx = 0UL; idx < lower_points_list.size(); ++idx) {
    auto& lower_points = lower_points_list[idx];
    auto& upper_points = upper_points_list[idx];
    ADEBUG << "collision index = " << idx;
    for (const auto& st_pt : lower_points) {
      ADEBUG << "collision point: [" << st_pt.t() << ", " << st_pt.s() << "]";
    }
    auto* cur_obs = obstacle;
    if (idx >= 1UL) {
      cur_obs = AddReadyBoundaryObstacle(*obstacle, idx);
    }
    if (lower_points_list.size() > 1UL) {
      frame_->AddBoundary(obstacle->Id(), cur_obs->Id(),
                          lower_points.front().t(), lower_points.back().t());
    }
    // if in st graph at first, it can't overtake.
    const bool is_minway_entry_boundary = lower_points.front().t() > 0.0;
    double overtake_distance_s =
        GetOvertakeSBuffer(*cur_obs, is_minway_entry_boundary);

    // only extend upper.
    // less two collision point ,can get boundary
    auto boundary = STBoundary::CreateInstance(lower_points, upper_points)
                        .ExpandBySForOvertake(overtake_distance_s);
    // AINFO<<"st_boudnary max t = "<<boundary.upper_right_point().t();
    boundary.set_id(cur_obs->Id());

    // TODO(all): potential bug here.
    const auto& prev_st_boundary = cur_obs->path_st_boundary();
    const auto& ref_line_st_boundary = cur_obs->reference_line_st_boundary();
    if (!prev_st_boundary.IsEmpty()) {
      boundary.SetBoundaryType(prev_st_boundary.boundary_type());
    } else if (!ref_line_st_boundary.IsEmpty()) {
      boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
    }
    boundary.set_obs_v(cur_obs->speed());
    cur_obs->set_path_st_boundary(boundary);
    SkipObsForExceedCollisionPoint(cur_obs);
  }
}

void STBoundaryMapper::SkipObsForExceedCollisionPoint(Obstacle* obstacle) {
  if (!exceed_collision_obs_ids_.empty() &&
      !obstacle->path_st_boundary().IsEmpty()) {
    const auto& boundary = obstacle->path_st_boundary();
    if (std::find(exceed_collision_obs_ids_.begin(),
                  exceed_collision_obs_ids_.end(),
                  obstacle->Id()) != exceed_collision_obs_ids_.end()) {
      double mint = boundary.min_t();
      double maxt = boundary.max_t();
      // AINFO << "mint = " << mint << "    maxt = " << maxt;
      size_t trajectory_size = obstacle->Trajectory().trajectory_point().size();
      double prediction_time = obstacle->Trajectory()
                                   .trajectory_point(trajectory_size - 1)
                                   .relative_time();
      // AINFO << "prediction_time = " << prediction_time;
      prediction_time = prediction_time > 0.0 ? prediction_time : kEpison;
      size_t first_collision_index =
          static_cast<size_t>(mint / (prediction_time) * (trajectory_size - 1));
      size_t second_collision_index =
          static_cast<size_t>(maxt / (prediction_time)*trajectory_size - 1);
      // AINFO << "first_collision_index = " << first_collision_index
      //       << "     second_collision_index = " << second_collision_index;
      const auto& first_collision_obs =
          obstacle->Trajectory().trajectory_point(first_collision_index);
      const auto& second_collision_obs =
          obstacle->Trajectory().trajectory_point(second_collision_index);
      common::math::Vec2d obs_first_point, obs_second_point, adc_front_point;
      double adc_heading = injector_->vehicle_state()->heading();
      double lon_buffer = 0.0;
      // TODO(zongxingguo): if can't fast breaking ,so skip to overtake;
      // double adc_v = injector_->vehicle_state()->linear_velocity();
      // double max_decel = adc_v / mint;
      // AINFO << "max_decel = " << max_decel;
      // if (max_decel > 4.0) {
      //   lon_buffer = adc_v * adc_v * 0.5 / 4.0;
      // }
      // AINFO << "lon_buffer = " << lon_buffer;
      double backward_distance =
          vehicle_param_.front_edge_to_center() + lon_buffer;
      double adc_front_x = injector_->vehicle_state()->x() +
                           backward_distance * std::cos(adc_heading),
             adc_front_y = injector_->vehicle_state()->y() +
                           backward_distance * std::sin(adc_heading);
      obs_first_point.set_x(first_collision_obs.path_point().x());
      obs_first_point.set_y(first_collision_obs.path_point().y());
      obs_second_point.set_x(second_collision_obs.path_point().x());
      obs_second_point.set_y(second_collision_obs.path_point().y());
      adc_front_point.set_x(adc_front_x);
      adc_front_point.set_y(adc_front_y);
      double obs_side = common::math::CrossProd(
          adc_front_point, obs_first_point, obs_second_point);
      common::math::LineSegment2d linesegment(obs_first_point,
                                              obs_second_point);
      double front_adc_to_traject_distance =
          linesegment.DistanceTo(adc_front_point);
      // AINFO << "front_adc_to_traject_distance = "
      //       << front_adc_to_traject_distance;
      double obs_half_width = obstacle->PerceptionBoundingBox().half_width();
      // AINFO << "obs_half_width = " << obs_half_width;
      bool is_adc_exceed_cross_obs = false;
      // AINFO << "obs_side = " << obs_side;
      const auto& obs_sl = obstacle->PerceptionSLBoundary();
      if (obs_sl.end_l() < adc_sl_boundary_.start_l()) {
        is_adc_exceed_cross_obs = obs_side < 0.0;
      }
      if (obs_sl.start_l() > adc_sl_boundary_.end_l()) {
        is_adc_exceed_cross_obs = obs_side > 0.0;
      }
      if (std::fabs(front_adc_to_traject_distance) < obs_half_width) {
        // AINFO << "near trajectory";
        is_adc_exceed_cross_obs = true;
      }
      // AINFO << "is_adc_exceed_cross_obs = " << is_adc_exceed_cross_obs;
      if (is_adc_exceed_cross_obs) {
        // AINFO << "erase st boundary";
        obstacle->EraseStBoundary();
      }
    }
  }
}

bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<size_t>* begin_overlap_position_in_st_points,
    std::vector<size_t>* end_overlap_position_in_st_points,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points) {
  // Sanity checks.
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  DCHECK(begin_overlap_position_in_st_points->empty());
  DCHECK(end_overlap_position_in_st_points->empty());
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }
  // AINFO << "CHECK collision id = " << obstacle.Id();

  CalcuLBufferForOverlapCheck(obstacle);

  // Draw the given obstacle on the ST-graph.
  // path_points.front().theta() need to change to ref_point heading.
  double obs_center_s = (obstacle.PerceptionSLBoundary().start_s() +
                         obstacle.PerceptionSLBoundary().end_s()) *
                        0.5;
  const auto& ref_point = reference_line_.GetReferencePoint(obs_center_s);
  GetObstacleReverseAndLateralSpeed(obstacle, ref_point.heading());

  bool is_need_break = false;
  if (FLAGS_enable_slow_down_for_cross_obstacle) {
    bool is_can_skip = false;
    is_need_break |=
        GetCrossObstacleStateForOverlapCheck(obstacle, &is_can_skip);
    if (is_can_skip) {
      ADEBUG << "skip obs for cross obs = " << obstacle.Id();
      return false;
    }
    ADEBUG << "is_cross_obs_in_intersection = " << is_need_break;
  }
  if (FLAGS_enable_slow_down_for_reverse_obstacle) {
    is_need_break |= GetReverseObstacleStateForOverlapCheck(obstacle);
    ADEBUG << "is_reverse_obs = " << is_need_break;
  }

  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  double curr_lane_left_width = 0.0;
  double curr_lane_right_width = 0.0;
  if (!reference_line_.GetLaneWidth(obs_sl.start_s(), &curr_lane_left_width,
                                    &curr_lane_right_width)) {
    curr_lane_left_width = kHalfLaneWidth;
    curr_lane_right_width = kHalfLaneWidth;
  }

  // no consider yield when out lane
  bool is_threated_static_obstacle =
      obs_sl.end_l() + kLateralBufferForYiedObstacle + kDistanceOutLane <
      -curr_lane_right_width;

  ADEBUG << "obstacle.Id()= " << obstacle.Id()
         << ", is_threated_static_obstacle=" << is_threated_static_obstacle
         << ", obs_sl.end_l()=" << obs_sl.end_l()
         << "adc_start_l=" << adc_sl_boundary_.start_l();
  bool is_traffic_light_green = true;
  bool is_consider_right_obstacle_speed =
      is_threated_static_obstacle && injector_->is_adc_in_junction_ &&
      is_traffic_light_green && FLAGS_enable_skip_motion_obstacle;
  ADEBUG << "is_consider_right_obstacle_speed = "
         << is_consider_right_obstacle_speed;

  if (is_consider_right_obstacle_speed) {
    reference_line_info_->AddSlowdownObstacle(obstacle.Id());
  }
  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point().empty() ||
      std::fabs(obstacle.speed()) < FLAGS_max_stop_speed ||
      is_consider_right_obstacle_speed) {
    OverlapCheckForStaticObstacle(path_points, obstacle, upper_points,
                                  lower_points);
  } else {
    // For those with predicted trajectories (moving obstacles):
    // 1. Subsample to reduce computation time.
    SubsamplePathPoint(path_points);
    // 2. Go through every point of the predicted obstacle trajectory.
    uint32_t step = GetObsStepForOverlapCheck(obstacle);
    ADEBUG << "trajectory_point size: " << trajectory.trajectory_point_size();
    ADEBUG << "trajectory_point step size: " << step;

    // rough search the overlap
    std::vector<int> begin_overlap_index;
    std::vector<int> end_overlap_index;
    RoughSearchOverlap(obstacle, step, is_need_break, &begin_overlap_index,
                       &end_overlap_index, begin_overlap_position_in_st_points,
                       end_overlap_position_in_st_points, upper_points,
                       lower_points);
    DCHECK_EQ(begin_overlap_index.size(), end_overlap_index.size());

    // fine search the overlap
    FineSearchOverlap(obstacle, step, is_need_break, begin_overlap_index,
                      end_overlap_index, begin_overlap_position_in_st_points,
                      end_overlap_position_in_st_points, upper_points,
                      lower_points);
  }

  // Sanity checks and return.
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

bool STBoundaryMapper::GetOverlapMultiBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<std::vector<STPoint>>* upper_points_list,
    std::vector<std::vector<STPoint>>* lower_points_list) {
  DCHECK(upper_points_list->empty());
  DCHECK(lower_points_list->empty());
  std::vector<STPoint> upper_points, lower_points;
  std::vector<size_t> begin_overlap_position_in_st_points;
  std::vector<size_t> end_overlap_position_in_st_points;
  GetOverlapBoundaryPoints(
      path_points, obstacle, &begin_overlap_position_in_st_points,
      &end_overlap_position_in_st_points, &upper_points, &lower_points);
  // std::vector<std::vector<STPoint>> upper_points_list, lower_points_list;
  for (const auto& st_pt : lower_points) {
    ADEBUG << "collision point: [" << st_pt.t() << ", " << st_pt.s() << "]";
  }
  for (size_t i = 0; i < begin_overlap_position_in_st_points.size(); ++i) {
    size_t begin_idx = begin_overlap_position_in_st_points[i];
    size_t end_idx = end_overlap_position_in_st_points[i];
    upper_points_list->emplace_back();
    upper_points_list->back().assign(upper_points.begin() + begin_idx,
                                     upper_points.begin() + end_idx);
    lower_points_list->emplace_back();
    lower_points_list->back().assign(lower_points.begin() + begin_idx,
                                     lower_points.begin() + end_idx);
  }
  // add no static obs but no trajectory collision.
  if (begin_overlap_position_in_st_points.empty() && !lower_points.empty()) {
    upper_points_list->push_back(upper_points);

    lower_points_list->push_back(lower_points);

    return (upper_points.size() >= 1 && lower_points.size() >= 1);
  }

  ADEBUG << "lower_points_list->size(): " << lower_points_list->size();
  ADEBUG << "upper_points_list->size(): " << upper_points_list->size();

  return (lower_points_list->size() >= 1 && upper_points_list->size() >= 1);
}

void STBoundaryMapper::GetObstacleReverseAndLateralSpeed(
    const Obstacle& obstacle, const double ref_heading) {
  const auto& obs_velocity = obstacle.Perception().velocity();
  obs_reverse_speed_ =
      common::math::Vec2d::CreateUnitVec2d(ref_heading)
          .InnerProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
  // ADEBUG << "obs_reverse_speed_ = " << obs_reverse_speed_;
  if (std::fabs(obstacle.speed()) > kEpison) {
    obs_lon_speed_ratio_ = std::fabs(obs_reverse_speed_ / obstacle.speed());
  }
  obs_lateral_speed_ =
      common::math::Vec2d::CreateUnitVec2d(ref_heading)
          .CrossProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
  if (std::fabs(obstacle.speed()) > kEpison) {
    obs_lat_speed_ratio_ = std::fabs(obs_lateral_speed_ / obstacle.speed());
  }
  // ADEBUG << "obs_lateral_speed_ = " << obs_lateral_speed_;
  double diff_heading = std::fabs(common::math::NormalizeAngle(
                            obstacle.SpeedHeading() - ref_heading)) *
                        kDegrees / M_PI_2;

  // ADEBUG << "diff_heading = " << diff_heading;
  is_vertical_obs_ = false;

  if (diff_heading > speed_bounds_config_.min_vertical_degree() &&
      diff_heading < speed_bounds_config_.max_vertical_degree()) {
    is_vertical_obs_ = true;
  }
  is_opposite_obs_ = false;
  if (diff_heading > speed_bounds_config_.min_opposite_degree()) {
    is_opposite_obs_ = true;
  }
  // ADEBUG << "is_vertical_obs = " << is_vertical_obs_
  //        << "    is_opposite_obs = " << is_opposite_obs_;
}

bool STBoundaryMapper::GetCrossObstacleStateForOverlapCheck(
    const Obstacle& obstacle, bool* is_can_skip) {
  if (std::fabs(obs_lateral_speed_) < speed_bounds_config_.min_cross_speed()) {
    return false;
  }
  bool is_need_break = false;
  // adc in common junction
  bool is_adc_in_common_junction =
      reference_line_info_->IsAdcInCommonJunction();
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  // cross path
  bool is_left_cross_obs =
           obs_sl.end_l() < adc_sl_boundary_.start_l() &&
           obs_lateral_speed_ > speed_bounds_config_.min_cross_speed(),
       is_right_cross_obs =
           obs_sl.start_l() > adc_sl_boundary_.end_l() &&
           obs_lateral_speed_ < -speed_bounds_config_.min_cross_speed(),
       is_cross_obs =
           is_vertical_obs_ && (is_left_cross_obs || is_right_cross_obs),
       is_face_to_path_obs = (is_left_cross_obs || is_right_cross_obs);
  // AINFO << "is_face_to_path_obs = " << is_face_to_path_obs;

  // is obs can reach path in lateral direction.
  //  kMaxTrajectoryRelativeTime need to large
  //  TrajectoryRelativeTime,otherwise, there will be critical issues.
  bool is_cross_path_relative_time =
      std::max(fabs(obstacle.PerceptionSLBoundary().start_l()),
               fabs(obstacle.PerceptionSLBoundary().end_l())) <
      std::fabs(obs_lateral_speed_) * kMaxTrajectoryRelativeTime;
  double obs_cross_relative_time = 0.0;

  // obs in front adc and vertical cross
  if (is_cross_obs) {
    PathDecision* const path_decision = reference_line_info_->path_decision();
    Obstacle* mutable_obstacle = path_decision->Find(obstacle.Id());
    mutable_obstacle->SetIsCross(true);
    double adc_half_width = vehicle_param_.width() * 0.5;
    obs_cross_relative_time =
        (std::min(fabs(obstacle.PerceptionSLBoundary().start_l()),
                  fabs(obstacle.PerceptionSLBoundary().end_l())) -
         adc_half_width) /
            std::fabs(obs_lateral_speed_) -
        speed_bounds_config_.min_obs_cross_relative_time();
  }
  // AINFO << "obs_cross_relative_time = " << obs_cross_relative_time;

  double adc_front_buffer =
      injector_->vehicle_state()->linear_velocity() * obs_cross_relative_time;
  // obs in back or no vertical cross or no green
  bool is_back_side_obs = obs_sl.end_s() < adc_sl_boundary_.start_s();
  if (obs_cross_relative_time < kEpison || is_back_side_obs ||
      !reference_line_info_->NoGreenInJunction()) {
    adc_front_buffer = 0.0;
  }

  // AINFO << "adc_front_buffer = " << adc_front_buffer;
  double adc_heading = injector_->vehicle_state()->heading(),
         obs_heading = obstacle.SpeedHeading();
  double backward_distance =
      vehicle_param_.front_edge_to_center() + adc_front_buffer;

  double adc_front_x = injector_->vehicle_state()->x() +
                       backward_distance * std::cos(adc_heading),
         adc_front_y = injector_->vehicle_state()->y() +
                       backward_distance * std::sin(adc_heading);
  double obs_x_first = obstacle.Perception().position().x(),
         obs_y_first = obstacle.Perception().position().y();
  double obs_x_second =
             obstacle.Perception().position().x() + 1.0 * std::cos(obs_heading),
         obs_y_second =
             obstacle.Perception().position().y() + 1.0 * std::sin(obs_heading);

  common::math::Vec2d obs_first_point, obs_second_point, adc_front_point;
  obs_first_point.set_x(obs_x_first);
  obs_first_point.set_y(obs_y_first);
  obs_second_point.set_x(obs_x_second);
  obs_second_point.set_y(obs_y_second);
  adc_front_point.set_x(adc_front_x);
  adc_front_point.set_y(adc_front_y);
  double obs_side = common::math::CrossProd(adc_front_point, obs_first_point,
                                            obs_second_point);

  bool is_adc_exceed_cross_obs = false;
  // AINFO << "obs_side = " << obs_side;
  // AINFO << "is_left_cross_obs = " << is_left_cross_obs;
  // AINFO << "is_right_cross_obs = " << is_right_cross_obs;
  if (is_left_cross_obs) {
    is_adc_exceed_cross_obs = obs_side < 0.0;
  }
  if (is_right_cross_obs) {
    is_adc_exceed_cross_obs = obs_side > 0.0;
  }
  // AINFO << "is_adc_exceed_cross_obs = " << is_adc_exceed_cross_obs;
  IsCanSkipForAdcExceedCollisionPoint(obstacle, is_adc_in_common_junction,
                                      is_adc_exceed_cross_obs,
                                      is_face_to_path_obs, is_cross_obs,
                                      is_cross_path_relative_time, is_can_skip);
  // AINFO << "is_can_skip = " << is_can_skip;
  //  ADEBUG << "is_adc_in_common_junction = " << is_adc_in_common_junction;
  //  ADEBUG << "is_cross_obs = " << is_cross_obs;
  //  ADEBUG << "is_cross_path_relative_time = " << is_cross_path_relative_time;
  return is_need_break;
}

void STBoundaryMapper::IsCanSkipForAdcExceedCollisionPoint(
    const Obstacle& obstacle, bool is_adc_in_common_junction,
    bool is_adc_exceed_cross_obs, bool is_face_to_path_obs, bool is_cross_obs,
    bool is_cross_path_relative_time, bool* is_can_skip) {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  // back obs ,adc front ecxeed collision point,skip.
  bool is_back_obs = obs_sl.end_s() < adc_sl_boundary_.start_s();
  // AINFO << "is_back_obs = " << is_back_obs;
  if (is_back_obs && is_adc_in_common_junction && is_adc_exceed_cross_obs &&
      is_face_to_path_obs) {
    // AINFO << "skip obs for back cross obs. ";
    *is_can_skip = true;
  }

  // overlap side obs  ,adc front ecxeed collision point,slow breking.
  // we can skip yet,but now slow breaking.
  // bool is_side_obs = obs_sl.end_s() < adc_sl_boundary_.end_s() &&
  //                    obs_sl.end_s() > adc_sl_boundary_.start_s();
  // if (is_side_obs && is_adc_in_common_junction && !is_adc_exceed_cross_obs &&
  //     is_face_to_path_obs) {
  //   // ADEBUG << "slow breaking obs for back cross obs and cut obs
  //   trajectory.";
  // }
  // front obs ,adc front ecxeed collision point and vertical cross obs,skip.
  bool is_front_obs = obs_sl.end_s() > adc_sl_boundary_.end_s();
  // AINFO << "is_front_obs = " << is_front_obs;
  if (is_front_obs) {
    if (is_adc_in_common_junction && is_adc_exceed_cross_obs && is_cross_obs &&
        is_cross_path_relative_time) {
      // AINFO << "SKIP OBS FOR EXCEED OBS";
      *is_can_skip = true;
    }

    bool no_lateral_overlap =
        obs_sl.end_l() + kLateralBufferForAdc < adc_sl_boundary_.start_l() ||
        obs_sl.start_l() - kLateralBufferForAdc > adc_sl_boundary_.end_l();
    // face to path ,adc front ecxeed collision point.skip.
    // only take effect in obs trajectory is straight or no trajectory.
    if (is_adc_exceed_cross_obs && is_face_to_path_obs && no_lateral_overlap) {
      IsCanSkipForlinearTrajectory(obstacle, is_can_skip);
    }

    // front; no_lateral_overlap; maby curve prediction trajectory; no face
    // path.
    if (no_lateral_overlap && !is_face_to_path_obs) {
      // AINFO << "skip no face obs";
      IsCanSkipForlinearTrajectory(obstacle, is_can_skip);
    }
  }
}

void STBoundaryMapper::IsCanSkipForlinearTrajectory(const Obstacle& obstacle,
                                                    bool* is_can_skip) {
  if (obstacle.HasTrajectory() &&
      !obstacle.Trajectory().trajectory_point().empty()) {
    const auto& trajectory = obstacle.Trajectory();
    size_t tra_point_size =
        static_cast<size_t>(trajectory.trajectory_point_size());
    double max_theta_diff = 0.0;
    double max_theta = std::numeric_limits<double>::lowest();
    double min_theta = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tra_point_size; ++i) {
      double heading_temp = trajectory.trajectory_point(i).path_point().theta();
      if (heading_temp > max_theta) {
        max_theta = heading_temp;
      }
      if (heading_temp < min_theta) {
        min_theta = heading_temp;
      }
      // AINFO << "   heading_temp = " << heading_temp;
    }
    max_theta_diff =
        std::fabs(common::math::NormalizeAngle((max_theta - min_theta)) *
                  kQuadrantAngle / M_PI_2);
    // AINFO << "max_theta_diff = " << max_theta_diff;
    if (max_theta_diff < kStraightAngle) {
      // AINFO << "SKIP OBS FOR EXCEED front OBS";
      *is_can_skip = true;
    } else {
      exceed_collision_obs_ids_.emplace_back(obstacle.Id());
    }
  } else {
    // AINFO << "SKIP OBS FOR EXCEED front OBS";
    *is_can_skip = true;
  }
}

bool STBoundaryMapper::GetReverseObstacleStateForOverlapCheck(
    const Obstacle& obstacle) {
  bool is_need_break = false;
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  bool is_on_lane = reference_line_info_->reference_line().IsOnADCLane(
      adc_sl_boundary_, obs_sl);
  bool is_in_range = obs_sl.start_s() - adc_sl_boundary_.end_s() > 0.0 &&
                     obs_sl.start_s() - adc_sl_boundary_.end_s() <
                         (std::fabs(obs_reverse_speed_) +
                          injector_->vehicle_state()->linear_velocity()) *
                             kTrajectoryRelativeTime;
  bool is_reverse_obs =
      is_opposite_obs_ &&
      obs_reverse_speed_ < speed_bounds_config_.min_reverse_speed();
  bool is_no_car =
      !(PerceptionObstacle::VEHICLE == obstacle.Perception().type());
  // can use in playstreet.
  if (hdmap::Lane::PLAY_STREET == reference_line_info_->GetLaneType()) {
    is_no_car = true;
  }
  // is_no_car = true;

  bool is_facing_path = false;
  bool is_overlap_obs = false;
  double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const auto& ref_point =
      reference_line_info_->path_data().GetPathPointWithPathS(
          obs_center_s -
          (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5);
  common::SLPoint ref_sl_point;
  if (!reference_line_.XYToSL(ref_point, &ref_sl_point)) {
    return false;
  }
  double adc_half_width = vehicle_param_.width() * 0.5;
  bool is_right_obs = obs_sl.end_l() < ref_sl_point.l() - adc_half_width;
  bool is_left_obs = obs_sl.start_l() > ref_sl_point.l() + adc_half_width;
  if (is_left_obs || is_right_obs) {
    ADEBUG << "obs_lateral_speed_ = " << obs_lateral_speed_;
    if (is_left_obs) {
      is_facing_path = obs_lateral_speed_ < 0.0;
    } else {
      is_facing_path = obs_lateral_speed_ > 0.0;
    }

  } else {
    is_overlap_obs = true;
  }
  ADEBUG << "is_facing_path = " << is_facing_path;
  ADEBUG << "is_overlap_obs = " << is_overlap_obs;
  ADEBUG << "is_on_lane = " << is_on_lane;
  ADEBUG << "is_in_range = " << is_in_range;
  ADEBUG << "is_reverse_obs = " << is_reverse_obs;
  const bool is_low_speed_obs = obstacle.speed() < kLowSpeed;
  if (is_on_lane && is_in_range && is_reverse_obs && is_no_car &&
      (is_overlap_obs || is_facing_path) && !is_low_speed_obs) {
    is_need_break = true;
    PathDecision* const path_decision = reference_line_info_->path_decision();
    Obstacle* mutable_obstacle = path_decision->Find(obstacle.Id());
    mutable_obstacle->SetIsReverse(true);
    reference_line_info_->AddReverseSlowdownObstacle(obstacle.Id());
    ADEBUG << "REVERSE OBS id = " << obstacle.Id();
  }
  return is_need_break;
}

void STBoundaryMapper::SubsamplePathPoint(
    const std::vector<PathPoint>& path_points) {
  if (path_points.size() > 2 * kDefaultNumPoint) {
    const auto ratio = path_points.size() / kDefaultNumPoint;
    std::vector<PathPoint> sampled_path_points;
    for (size_t i = 0; i < path_points.size(); ++i) {
      if (i % ratio == 0) {
        sampled_path_points.emplace_back(path_points[i]);
      }
    }
    subsample_discretized_path_ =
        DiscretizedPath(std::move(sampled_path_points));
  } else {
    subsample_discretized_path_ = DiscretizedPath(path_points);
  }
}

uint32_t STBoundaryMapper::GetObsStepForOverlapCheck(const Obstacle& obstacle) {
  const auto obs_type = obstacle.Perception().type();
  const auto& trajectory = obstacle.Trajectory();
  bool is_unknown_obs =
      perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type;
  double diff_dis_of_trajectory = std::numeric_limits<double>::max();
  if (static_cast<size_t>(trajectory.trajectory_point_size()) >
      kMinObstacleTrajectorySize) {
    auto diff_x = trajectory.trajectory_point(1).path_point().x() -
                  trajectory.trajectory_point(0).path_point().x();
    auto diff_y = trajectory.trajectory_point(1).path_point().y() -
                  trajectory.trajectory_point(0).path_point().y();
    diff_dis_of_trajectory = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    // if obs has large speed and trajectory point in one position,
    // diff_dis_of_trajectory maybe tiny ,cause step large.
    double cal_diff_dis_of_trajectory_use_speed =
        std::fabs(obstacle.speed() * 0.1);
    // AINFO << "cal_diff_dis_of_trajectory_use_speed = "
    //       << cal_diff_dis_of_trajectory_use_speed;
    diff_dis_of_trajectory =
        std::max(diff_dis_of_trajectory, cal_diff_dis_of_trajectory_use_speed);
  }
  diff_dis_of_trajectory = std::max(diff_dis_of_trajectory, kEpison);
  // AINFO << "diff_dis_of_trajectory = " << diff_dis_of_trajectory;
  // default step = 0.3
  double min_step_distance = kMinObstacleTrajectoryStepDistance;
  if (!is_unknown_obs) {
    min_step_distance =
        obstacle.PerceptionBoundingBox().length() * kObstacleLengthRatio;
  }
  min_step_distance =
      std::min(min_step_distance, kMaxObstacleTrajectoryStepDistance);
  ADEBUG << "obstacle(" << obstacle.Id()
         << ") len:" << obstacle.PerceptionBoundingBox().length()
         << "width:" << obstacle.PerceptionBoundingBox().width();
  uint32_t step = 1U;
  if (diff_dis_of_trajectory <= min_step_distance) {
    // the step distance will be between min_step_distance and
    // 2*min_step_distance
    step = static_cast<uint32_t>(
        std::floor(2.0 * min_step_distance / diff_dis_of_trajectory));
    // AINFO << "step = " << step;
  }
  // step no overseed trajectory.size
  step = std::min(
      static_cast<uint32_t>(trajectory.trajectory_point().size() - 1), step);
  // if has 50 point.0-49,so loop to end point ,we need 7 times.
  if (step > kTrajectoryPointsTimes) {
    step =
        step % kTrajectoryPointsTimes == 0
            ? step
            : ((step - 1) / kTrajectoryPointsTimes) * kTrajectoryPointsTimes;
    // AINFO << "new step = " << step;
  }
  return step;
}

void STBoundaryMapper::RoughSearchOverlap(
    const Obstacle& obstacle, const uint32_t step, const bool is_need_break,
    std::vector<int>* const begin_overlap_index,
    std::vector<int>* const end_overlap_index,
    std::vector<size_t>* const begin_overlap_position_in_st_points,
    std::vector<size_t>* const end_overlap_position_in_st_points,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points) {
  OverlapStatus overlap_status_last = NEED_CONTINUE;
  const auto& trajectory = obstacle.Trajectory();
  int k = 0;
  int i = 0;
  bool is_last_point = false;
  while (i < trajectory.trajectory_point_size()) {
    if (is_last_point) {
      break;
    }
    const auto& trajectory_point = trajectory.trajectory_point(i);
    if (static_cast<uint32_t>(i) >= step) {
      const auto& trajectory_point_before =
          trajectory.trajectory_point(i - step);
      ADEBUG << "diff trajectory(" << i << "): ["
             << trajectory_point.path_point().x() -
                    trajectory_point_before.path_point().x()
             << ", "
             << trajectory_point.path_point().y() -
                    trajectory_point_before.path_point().y()
             << "]"
             << "   heading  = " << trajectory_point.path_point().theta();
    }
    // AINFO << "i = " << i;
    STPoint lower_point, upper_point;
    OverlapStatus overlap_status = DetectOverlapByObstacleTrajectory(
        obstacle, trajectory_point, is_need_break, &lower_point, &upper_point);
    if (NEED_BREAK == overlap_status) {
      // AINFO << "NEED_BREAK = ";
      ++k;
    }
    if (FIND_OVERLAP == overlap_status) {
      // AINFO << "FIND_OVERLAP";
      lower_points->push_back(lower_point);
      upper_points->push_back(upper_point);
    } else if (NEED_BREAK == overlap_status && k > 1) {
      // AINFO << "NEED_BREAK ";
      break;
    }
    if (IsFindOvelap(overlap_status) && !IsFindOvelap(overlap_status_last)) {
      // now find overlap, last step no find overlap.
      // record the begin overlap index
      begin_overlap_index->push_back(i);
      // record the begin ovelap position in lower_points
      // AINFO << "record the begin ovelap position in lower_points = "
      //       << lower_points->size() - 1UL;
      begin_overlap_position_in_st_points->push_back(lower_points->size() -
                                                     1UL);
    } else if (!IsFindOvelap(overlap_status) &&
               IsFindOvelap(overlap_status_last)) {
      // last step find overlap, now no find overlap.
      // record the end overlap index
      end_overlap_index->push_back(i);
      // record the end ovelap position in lower_points
      // AINFO << "record the end ovelap position in lower_points = "
      //       << lower_points->size();
      // we need update in finesearch.beacuse this position no correct position.
      // this position no collision.
      end_overlap_position_in_st_points->push_back(lower_points->size());
    } else {
      // do nothing
    }
    overlap_status_last = overlap_status;
    // if 50 point ,step = 30 ,only has circle 2 times,and 31-49point no check.
    i += step;
    if (i >= trajectory.trajectory_point_size()) {
      i = trajectory.trajectory_point_size() - 1;
      is_last_point = true;
    }
  }
  // if only one collision point in roughsearch.
  // maybe only begin_overlap or only end_overlap,
  if (begin_overlap_index->size() != end_overlap_index->size()) {
    end_overlap_index->push_back(trajectory.trajectory_point_size());
    end_overlap_position_in_st_points->push_back(lower_points->size());
  }
}

void STBoundaryMapper::FineSearchOverlap(
    const Obstacle& obstacle, const uint32_t step, const bool is_need_break,
    const std::vector<int>& begin_overlap_index,
    const std::vector<int>& end_overlap_index,
    std::vector<size_t>* const begin_overlap_position_in_st_points,
    std::vector<size_t>* const end_overlap_position_in_st_points,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points) {
  const auto& trajectory = obstacle.Trajectory();
  // maybe has collision in two palce in one path.default size = 1.
  for (size_t idx = 0; idx < begin_overlap_index.size(); ++idx) {
    // AINFO << "idx = " << idx;
    // AINFO << "begin_overlap_index[idx] = " << begin_overlap_index[idx];
    int begin_min =
        std::max(begin_overlap_index[idx] - static_cast<int>(step) + 1, 0);
    // begin_min is first collision point,
    // need move one step forward.if near trajectory first point ,use trajectory
    // first point .
    // AINFO << "first collision begin_min = " << begin_min;
    for (int j = begin_min; j < begin_overlap_index[idx]; ++j) {
      // AINFO << "j = " << j;
      const auto& trajectory_point = trajectory.trajectory_point(j);
      STPoint lower_point, upper_point;
      OverlapStatus overlap_status = DetectOverlapByObstacleTrajectory(
          obstacle, trajectory_point, is_need_break, &lower_point,
          &upper_point);
      if (IsFindOvelap(overlap_status)) {
        // AINFO << "find overlap index= " << j;
        auto it_lower = lower_points->begin();
        auto it_upper = upper_points->begin();
        std::advance(it_lower, (*begin_overlap_position_in_st_points)[idx]);
        std::advance(it_upper, (*begin_overlap_position_in_st_points)[idx]);
        if (it_lower != lower_points->end() &&
            lower_point.t() >= it_lower->t()) {
          ADEBUG << "1lower_point.t:" << lower_point.t();
        }
        // AINFO << "lower_point.t() = " << lower_point.t();
        lower_points->insert(it_lower, lower_point);
        upper_points->insert(it_upper, upper_point);
        // if we insert ,end overlap position need update.
        ++(*end_overlap_position_in_st_points)[idx];
        size_t pos_idx = idx + 1UL;
        while (pos_idx < begin_overlap_index.size()) {
          ++(*begin_overlap_position_in_st_points)[pos_idx];
          ++(*end_overlap_position_in_st_points)[pos_idx];
          ++pos_idx;
        }
        break;
      }
    }
    int end_min =
        std::max(end_overlap_index[idx] - static_cast<int>(step) + 1, 0);
    // AINFO << "back collision end_min = " << end_min;
    for (int j = end_overlap_index[idx] - 1; j >= end_min; --j) {
      // AINFO << "j = " << j;
      const auto& trajectory_point = trajectory.trajectory_point(j);
      STPoint lower_point, upper_point;
      OverlapStatus overlap_status = DetectOverlapByObstacleTrajectory(
          obstacle, trajectory_point, is_need_break, &lower_point,
          &upper_point);
      if (IsFindOvelap(overlap_status)) {
        // AINFO << "find overlap index= " << j;
        auto it_lower = lower_points->begin();
        auto it_upper = upper_points->begin();
        std::advance(it_lower, (*end_overlap_position_in_st_points)[idx]);
        std::advance(it_upper, (*end_overlap_position_in_st_points)[idx]);
        if (lower_point.t() > (it_lower - 1)->t()) {
          lower_points->insert(it_lower, lower_point);
          upper_points->insert(it_upper, upper_point);
          // need update end overlap position,in rough maybe no get correct end
          // overlap position.
          //(*end_overlap_position_in_st_points)[idx] = lower_points->size();
          ++(*end_overlap_position_in_st_points)[idx];
          size_t pos_idx = idx + 1UL;
          while (pos_idx < begin_overlap_index.size()) {
            ++(*begin_overlap_position_in_st_points)[pos_idx];
            ++(*end_overlap_position_in_st_points)[pos_idx];
            ++pos_idx;
          }
        } else {
          ADEBUG << "2lower_point.t:" << lower_point.t();
        }
        break;
      }
    }
  }
}

void STBoundaryMapper::OverlapCheckForStaticObstacle(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points) {
  // For those with no predicted trajectories, just map the obstacle's
  // current position to ST-graph and always assume it's static.
  if (!obstacle.IsStatic()) {
    AWARN << "Non-static obstacle[" << obstacle.Id()
          << "] has NO prediction trajectory."
          << obstacle.Perception().ShortDebugString();
  }
  const auto& obs_sl_boundary = obstacle.PerceptionSLBoundary();
  double obs_max_s = obs_sl_boundary.end_s();
  double obs_min_s = obs_sl_boundary.start_s();
  double adc_len = adc_sl_boundary_.end_s() - adc_sl_boundary_.start_s();
  double center_s =
      (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;

  const auto& obs_polygon = obstacle.PerceptionPolygon();
  const Box2d& obs_box = obstacle.PerceptionBoundingBox();
  const auto obs_type = obstacle.Perception().type();
  for (const auto& curr_point_on_path : path_points) {
    if (curr_point_on_path.s() > planning_max_distance_) {
      break;
    }
    // AINFO << "curr_point_on_path.s() = " << curr_point_on_path.s();
    // Only the projection of obstacles on the reference line is selected for
    // collision detection
    if (curr_point_on_path.s() + center_s < obs_min_s - adc_len) {
      // AINFO << "CONTINUE";
      continue;
    } else if (curr_point_on_path.s() + center_s > obs_max_s + adc_len) {
      // AINFO << "BREAK";
      break;
    }
    bool is_overlap = false;
    // turn area, use sl collision for vehicle and stacker.
    // need change no ue sl colliion
    // if ((perception::PerceptionObstacle::STACKER == obs_type ||
    //      perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type ||
    //      perception::PerceptionObstacle::VEHICLE == obs_type) &&
    //     (at_left_turn_ || at_right_turn_)) {
    if(0){
      is_overlap = CheckStackerObsOverlap(curr_point_on_path, obs_sl_boundary,
                                          obs_polygon);
    } else if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type) {
        if((at_left_turn_ || at_right_turn_)){
          // is_overlap = CheckStackerObsOverlap(curr_point_on_path,
          // obs_sl_boundary,
          //                                 obs_polygon);
          is_overlap = CheckUnknownObsOverlap(curr_point_on_path,
                                              obs_sl_boundary, obs_polygon);
        }else{
            is_overlap = CheckUnknownObsOverlap(curr_point_on_path, obs_sl_boundary,
                                          obs_polygon);
        }
    } else {
      is_overlap = CheckOverlap(curr_point_on_path, obs_polygon);
    }
    if (is_overlap) {
      // AINFO << "HAS COLLISION";
        const double backward_distance = -vehicle_param_.front_edge_to_center();
        const double forward_distance = vehicle_param_.length() +
                                        vehicle_param_.width() +
                                        obs_box.length() + obs_box.width();
      double low_s = std::fmax(0.0, curr_point_on_path.s() + backward_distance);
        double high_s = std::fmin(planning_max_distance_,
                                  curr_point_on_path.s() + forward_distance);
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_max_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_max_time_);
        break;
    }
  }
}

void STBoundaryMapper::CalcuLBufferForOverlapCheck(const Obstacle& obstacle) {
  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();
  l_buffer_ = ChangeLaneStatus::IN_CHANGE_LANE == planning_status->status()
                  ? speed_bounds_config_.lane_change_obstacle_nudge_l_buffer()
                  : FLAGS_nonstatic_obstacle_nudge_l_buffer;
  const auto obs_type = obstacle.Perception().type();
  // stitic car need add buffer，0.3
  if ((perception::PerceptionObstacle::VEHICLE == obs_type &&
       obstacle.IsStatic()) ||
      perception::PerceptionObstacle::STACKER == obs_type ||
      perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
    l_buffer_ = l_buffer_ + FLAGS_car_type_lateral_buffer;
  }
  bool is_tire_lifter =
            obstacle.Perception().type() ==
                perception::PerceptionObstacle::WHEELCRANE;
  if(is_tire_lifter){
    // todo tire_lifer collision
    l_buffer_ = l_buffer_ + FLAGS_car_type_lateral_buffer;
    // l_buffer_ = 1.0;
  }
  if (injector_->enable_shrink_collision_buffer_) {
    l_buffer_ = kNormalBuffer;
  }
  // AINFO << "adc_sl_boundary_.start_l() = " << adc_sl_boundary_.start_l();
  // AINFO << "obs end_l = " << obstacle.PerceptionSLBoundary().end_l();
  if (perception::PerceptionObstacle::PEDESTRIAN == obs_type &&
      planning_status->status() != ChangeLaneStatus::IN_CHANGE_LANE &&
      !injector_->is_in_play_street) {
    double adc_velocity = injector_->vehicle_state()->linear_velocity();
    adc_velocity = std::fabs(adc_velocity);
    if (adc_velocity > kHighVelocity) {
      l_buffer_ = kMaxLateralBuffer;
    } else if (adc_velocity > kLowVelocity && adc_velocity < kHighVelocity) {
      l_buffer_ = kSecondaryLateralBuffer;
    } else if (adc_velocity < kLowVelocity) {
      l_buffer_ = kMinLateralBuffer;
    }
  }

  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_) {
    l_buffer_ *= 0.5;
  }
  if (FLAGS_allow_narrow_pass&& injector_->is_adc_in_gate_junction_) {
    l_buffer_ = 0.0;
  }
  AINFO << "l_buffer_ = " << l_buffer_;
}

STBoundaryMapper::OverlapStatus STBoundaryMapper::FoundFineOverlap(
    const Obstacle& obstacle, const Polygon2d& obs_polygon,
    const Box2d& obs_box, const double trajectory_point_time,
    const double path_s, const double step_length,
    STPoint* const ptr_lower_point, STPoint* const ptr_upper_point) const {
  const double backward_distance = -step_length;
  const double forward_distance = vehicle_param_.length() +
                                  vehicle_param_.width() + obs_box.length() +
                                  obs_box.width();
  bool is_long_obs =
      (obs_box.width() > kLongObsLength) || (obs_box.length() > kLongObsLength);

  const double default_min_step =
      speed_bounds_config_.default_min_tune_step();  // in meters

  const double fine_tuning_step_length =
      std::fmin(default_min_step,
                subsample_discretized_path_.Length() / kDefaultNumPoint);
  ADEBUG << "fine_tuning_step_length=" << fine_tuning_step_length;
  const double default_min_step_for_high_s =
      speed_bounds_config_.default_second_tune_step();  // in meters

  const double fine_tuning_step_length_for_high_s =
      std::fmin(default_min_step_for_high_s,
                subsample_discretized_path_.Length() / kDefaultNumPoint);
  ADEBUG << "fine_tuning_step_length_for_high_s = "
         << fine_tuning_step_length_for_high_s;
  bool find_low = false;
  bool find_high = false;
  double low_s = std::fmax(0.0, path_s + backward_distance);
  double high_s = std::fmin(subsample_discretized_path_.Length(),
                            path_s + forward_distance);
  const auto obs_type = obstacle.Perception().type();
  bool is_unknown_obs =
      perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type;
  // Keep shrinking by the resolution bidirectionally until finally
  // locating the tight upper and lower bounds.
  bool has_overlap = true;
  while (low_s < high_s) {
    if (find_low && find_high) {
      break;
    }
    if (!find_low) {
      const auto& point_low = subsample_discretized_path_.Evaluate(
          low_s + subsample_discretized_path_.front().s());
      if (is_unknown_obs) {
        has_overlap = CheckOverlap(point_low, obs_polygon);
      } else {
        has_overlap = CheckOverlap(point_low, obs_box);
      }
      if (!has_overlap) {
        low_s += fine_tuning_step_length;
      } else {
        find_low = true;
      }
    }
    if (!find_high) {
      if (FLAGS_enable_no_shrink_upper_bound && !is_long_obs) {
        find_high = true;
      } else {
        const auto& point_high = subsample_discretized_path_.Evaluate(
            high_s + subsample_discretized_path_.front().s());
        if (is_unknown_obs) {
          has_overlap = CheckOverlap(point_high, obs_polygon);
        } else {
          has_overlap = CheckOverlap(point_high, obs_box);
        }
        if (!has_overlap) {
          high_s -= fine_tuning_step_length_for_high_s;
        } else {
          find_high = true;
        }
      }
    }
  }
  OverlapStatus overlap_status = NEED_CONTINUE;
  if (find_high && find_low) {
    *ptr_lower_point = STPoint(low_s - speed_bounds_config_.point_extension(),
                               trajectory_point_time);
    *ptr_upper_point = STPoint(high_s + speed_bounds_config_.point_extension(),
                               trajectory_point_time);
    overlap_status = FIND_OVERLAP;
  }
  return overlap_status;
}

STBoundaryMapper::OverlapStatus
STBoundaryMapper::DetectOverlapByObstacleTrajectory(
    const Obstacle& obstacle, const common::TrajectoryPoint& trajectory_point,
    const bool is_need_break, STPoint* const ptr_lower_point,
    STPoint* const ptr_upper_point) const {
  CHECK_GT(kDefaultNumPoint, 0UL);
  OverlapStatus overlap_status = NEED_CONTINUE;
  double trajectory_point_time = trajectory_point.relative_time();
  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_ &&
      obs_lon_speed_ratio_ > kMinPercentOfReverseSpeed &&
      obs_reverse_speed_ < kStopReverseVelocity &&
      trajectory_point_time >
          speed_bounds_config_.min_trajectory_relative_time()) {
    overlap_status = NEED_BREAK;
    return overlap_status;
  }
  // high speed use large relative time
  // double adc_v = injector_->vehicle_state()->linear_velocity();
  // double trajectory_relative_time =
  //     std::min(std::floor(kRelativeTimeCoeff * adc_v), kMaxRelativeTime);
  // double min_trajectory_relative_time =
  //     std::max(speed_bounds_config_.min_trajectory_relative_time(),
  //              trajectory_relative_time);
  // reverse or cross obs
  // if (trajectory_point_time > min_trajectory_relative_time && is_need_break)
  // {
  if (trajectory_point_time >
          speed_bounds_config_.min_trajectory_relative_time() &&
      is_need_break) {
    overlap_status = NEED_BREAK;
    reference_line_info_->SetIsNeedToSpeedLimit(true);
    ADEBUG << "break";
    return overlap_status;
  }
  // if adc in low speed can break more trajectory.
  if (trajectory_point_time >
          speed_bounds_config_.min_trajectory_relative_time() &&
      is_need_break &&
      injector_->vehicle_state()->linear_velocity() < kSecondVelocity) {
    overlap_status = NEED_BREAK;
    reference_line_info_->SetIsNeedToSpeedLimit(true);
    ADEBUG << "break";
    return overlap_status;
  }
  // reverse obstacle no on lane or no in range.
  if (obs_lon_speed_ratio_ > speed_bounds_config_.percent_of_speed() &&
      obs_reverse_speed_ < speed_bounds_config_.min_reverse_speed() &&
      trajectory_point_time >
          speed_bounds_config_.min_trajectory_relative_time() * 2) {
    overlap_status = NEED_BREAK;
    return overlap_status;
  }
  if (trajectory_point_time < kNegtiveTimeThreshold) {
    return overlap_status;
  }

  Polygon2d obs_polygon;
  Box2d obs_box;
  const auto obs_type = obstacle.Perception().type();
  bool is_unknown_obs =
      perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type;
  if (is_unknown_obs) {
    obs_polygon = obstacle.GetPolygon(trajectory_point);
  } else {
    obs_box = obstacle.GetBoundingBox(trajectory_point);
  }

  const double step_length = vehicle_param_.front_edge_to_center();
  // AINFO << "subsample_discretized_path_.Length() = "
  //       << subsample_discretized_path_.Length();
  auto path_len =
      std::min(max_collision_path_len_, subsample_discretized_path_.Length());
  // Go through every point of the ADC's path.
  for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
    const auto curr_adc_path_point = subsample_discretized_path_.Evaluate(
        path_s + subsample_discretized_path_.front().s());
    bool has_overlap = false;
    if (is_unknown_obs) {
      has_overlap = CheckOverlap(curr_adc_path_point, obs_polygon);
    } else {
      has_overlap = CheckOverlap(curr_adc_path_point, obs_box);
    }
    if (has_overlap) {
      // Found overlap, start searching with higher resolution
      overlap_status = FoundFineOverlap(
          obstacle, obs_polygon, obs_box, trajectory_point_time, path_s,
          step_length, ptr_lower_point, ptr_upper_point);
      break;
    }
  }

  return overlap_status;
}

double CountBoundaryScore(std::pair<double, double> ref_boundary_time,
                          std::pair<double, double> boundary_time) {
  if (ref_boundary_time.first > boundary_time.second ||
      ref_boundary_time.second < boundary_time.first) {
    return -std::min(std::abs(ref_boundary_time.first - boundary_time.second),
                     std::abs(ref_boundary_time.second - boundary_time.first));
  }
  return std::min(std::abs(ref_boundary_time.first - boundary_time.second),
                  std::abs(ref_boundary_time.second - boundary_time.first)) /
             std::max(
                 std::abs(ref_boundary_time.first - boundary_time.second),
                 std::abs(ref_boundary_time.second - boundary_time.first)) +
         kEpsilon;
}

bool STBoundaryMapper::SelectStPoints(
    const std::string& obs_id,
    std::vector<std::vector<STPoint>>* upper_points_list,
    std::vector<std::vector<STPoint>>* lower_points_list,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points) {
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  const auto& boundary_map = frame_->GetBoundaryMap();
  auto base_id = obs_id;
  size_t pos = base_id.find(kAddedIdPrefix);
  if (std::string::npos != pos) {
    base_id.erase(pos, kAddedIdSize);
  }
  switch (lower_points_list->size()) {
    case 0UL:
      break;
    case 1UL:
      *upper_points = std::move(upper_points_list->front());
      *lower_points = std::move(lower_points_list->front());
      break;
    default:
      auto range = boundary_map.equal_range(base_id);
      std::string boundary_obs_id;
      double first_time = 0.0, last_time = 0.0;
      for (auto it = range.first; it != range.second; ++it) {
        std::tie(boundary_obs_id, first_time, last_time) = it->second;
        if (boundary_obs_id == obs_id) {
          ADEBUG << "boundary_obs_id: " << boundary_obs_id;
          ADEBUG << "boundary time range = [" << first_time << ", " << last_time
                 << "]";
          break;
        }
      }
      using Pair = std::pair<double, size_t>;
      struct comp {
        bool operator()(const Pair& a, const Pair& b) const {
          return a.first < b.first;
        }
      };

      std::priority_queue<Pair, std::vector<Pair>, comp> score_que;
      for (size_t idx = 0UL; idx < lower_points_list->size(); ++idx) {
        auto& cur_lower_points = (*lower_points_list)[idx];
        double score = CountBoundaryScore(
            {first_time, last_time},
            {cur_lower_points.front().t(), cur_lower_points.back().t()});
        score_que.emplace(score, idx);
      }
      auto matched_index = score_que.top().second;
      *upper_points = std::move((*upper_points_list)[matched_index]);
      *lower_points = std::move((*lower_points_list)[matched_index]);
      break;
  }
  return !lower_points->empty();
}

void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) {
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } else {
    std::vector<std::vector<STPoint>> lower_points_list;
    std::vector<std::vector<STPoint>> upper_points_list;
    if (!GetOverlapMultiBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                       &upper_points_list,
                                       &lower_points_list)) {
      return;
    }
    if (!SelectStPoints(obstacle->Id(), &upper_points_list, &lower_points_list,
                        &upper_points, &lower_points)) {
      return;
    }
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    // TODO(zongxingguo): take piecewise_jerk_speed_optimizer here.
    characteristic_length = std::fabs(decision.follow().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    if (FLAGS_enable_yield_for_min_lateral_obs) {
      boundary = STBoundary::CreateInstance(lower_points, upper_points)
                     .ExpandBySForOvertake(characteristic_length);
    }
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  boundary.set_obs_v(obstacle->speed());
  obstacle->set_path_st_boundary(boundary);
}
bool STBoundaryMapper::JudgeCarInTrafficeLightInnerJunction(
    std::vector<JunctionInfoConstPtr>* junctions, const Vec2d& car_position,
    JunctionInfoConstPtr* target_junction) const {
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    return false;
  }
  for (size_t i = 0; i < junction_num; ++i) {
    if (Junction::TRAFFICLIGHT_INTER == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: TRAFFICLIGHT_INTER";
    } else if (Junction::UNKNOWN == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: UNKNOWN";
    } else if (Junction::IN_ROAD == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: IN_ROAD";
    } else if (Junction::CROSS_ROAD == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: CROSS_ROAD";
    } else if (Junction::MAIN_SIDE == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: MAIN_SIDE";
    } else if (Junction::DEAD_END == junctions->at(i)->junction().type()) {
      ADEBUG << "Type: DEAD_END";
    }
    if (Junction::TRAFFICLIGHT_INTER == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(car_position)) {
        *target_junction = junctions->at(i);
        return true;
      } else {
        continue;
      }
    } else {
      continue;
    }
  }
  return false;
}
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  double adc_theta = path_point.theta();
    // AINFO<<"path_point.s = "<<path_point.s();
  if (reference_line_info_->NeedDiagonal()) {
    adc_theta = diagonal_heading_;
        double center_s = (adc_sl_boundary_.start_s()+adc_sl_boundary_.end_s())*0.5;
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if ((reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s))) {
        // AINFO<<" no use diagonal heading";
        adc_theta = path_point.theta();
      }else{
        // AINFO<<"  use diagonal heading";
      }
    }
  }
  ego_center_map_frame.SelfRotate(adc_theta);
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_theta, vehicle_param_.length(),
                vehicle_param_.width() + l_buffer_ * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Polygon2d& obs_polygon) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  double adc_theta = path_point.theta();
  // AINFO<<"path_point.s = "<<path_point.s();
      double center_s = (adc_sl_boundary_.start_s()+adc_sl_boundary_.end_s())*0.5;
  if (reference_line_info_->NeedDiagonal()) {
    adc_theta = diagonal_heading_;
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if ((reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s))) {
        // AINFO<<" no use diagonal heading";
        adc_theta = path_point.theta();
      }else{
        // AINFO<<"  use diagonal heading";
      }
    }
  }
  ego_center_map_frame.SelfRotate(adc_theta);
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_theta, vehicle_param_.length(),
                vehicle_param_.width() + l_buffer_ * 2);
  // Check whether ADC bounding box overlaps with obstacle polygon.
  return obs_polygon.HasOverlap(Polygon2d(adc_box));
}

bool STBoundaryMapper::CheckStackerObsOverlap(
    const common::PathPoint& path_point,
    const planning::SLBoundary& obs_sl_boundary,
    const common::math::Polygon2d& obs_polygon) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  double adc_theta = path_point.theta();
  if (reference_line_info_->NeedDiagonal()) {
    adc_theta = diagonal_heading_;
      // AINFO<<"path_point.s = "<<path_point.s();
          double center_s = (adc_sl_boundary_.start_s()+adc_sl_boundary_.end_s())*0.5;
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if ((reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s))) {
        // AINFO<<" no use diagonal heading";
        adc_theta = path_point.theta();
      }else{
        // AINFO<<"  use diagonal heading";
      }
    }
  }
  // AINFO << "adc_theta = " << adc_theta;
  ego_center_map_frame.SelfRotate(adc_theta);
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_theta, vehicle_param_.length(),
                vehicle_param_.width() + l_buffer_ * 2);

  // collision detection based on SLBoundary.
  SLBoundary adc_sl_boundary;
  if (!reference_line_.GetSLBoundary(adc_box, &adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << adc_box.DebugString();
    return false;
  }
  bool no_overlap =
      ((adc_sl_boundary.end_s() < obs_sl_boundary.start_s() ||
        adc_sl_boundary.start_s() > obs_sl_boundary.end_s()) ||  // longitudinal
       (adc_sl_boundary.end_l() < obs_sl_boundary.start_l() ||
        adc_sl_boundary.start_l()  >
            obs_sl_boundary.end_l()));  // lateral
  if (no_overlap) {
      // AINFO << "There is no overlap. adc_sl_boundary.start_s() = "
      //       << adc_sl_boundary.start_s()
      //       << ", adc_sl_boundary.end_s() = " << adc_sl_boundary.end_s()
      //       << ", obs_sl_boundary.start_s() = " << obs_sl_boundary.start_s()
      //       << ", obs_sl_boundary.end_s() = " << obs_sl_boundary.end_s()
      //       << ", adc_sl_boundary.start_l() = " << adc_sl_boundary.start_l()
      //       << ", adc_sl_boundary.end_l() = " << adc_sl_boundary.end_l()
      //       << ", obs_sl_boundary.start_l() = " << obs_sl_boundary.start_l()
      //       << ", obs_sl_boundary.end_l() = " << obs_sl_boundary.end_l();
    return false;
  } else {
    // only use rough collision check.
    if (((adc_sl_boundary.end_s() > obs_sl_boundary.start_s() &&
          adc_sl_boundary.start_s() <
              obs_sl_boundary.end_s()) &&  // longitudinal
         (adc_sl_boundary.end_l()  > obs_sl_boundary.start_l() ||
          adc_sl_boundary.start_l()  < obs_sl_boundary.end_l()))) {
      // AINFO << "There is an overlap. adc_sl_boundary.start_s() = "
      //       << adc_sl_boundary.start_s()
      //       << ", adc_sl_boundary.end_s() = " << adc_sl_boundary.end_s()
      //       << ", obs_sl_boundary.start_s() = " << obs_sl_boundary.start_s()
      //       << ", obs_sl_boundary.end_s() = " << obs_sl_boundary.end_s()
      //       << ", adc_sl_boundary.start_l() = " << adc_sl_boundary.start_l()
      //       << ", adc_sl_boundary.end_l() = " << adc_sl_boundary.end_l()
      //       << ", obs_sl_boundary.start_l() = " << obs_sl_boundary.start_l()
      //       << ", obs_sl_boundary.end_l() = " << obs_sl_boundary.end_l();
      return true;
    }
  }
  return obs_polygon.HasOverlap(common::math::Polygon2d(adc_box));
}

bool STBoundaryMapper::CheckUnknownObsOverlap(
    const common::PathPoint& path_point,
    const planning::SLBoundary& obs_sl_boundary,
    const common::math::Polygon2d& obs_polygon) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  double adc_theta = path_point.theta();
  if (reference_line_info_->NeedDiagonal()) {
    adc_theta = diagonal_heading_;
    double center_s = (adc_sl_boundary_.start_s()+adc_sl_boundary_.end_s())*0.5;
      // AINFO<<"path_point.s = "<<path_point.s();
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if ((reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s))) {
        // AINFO<<" no use diagonal heading";
        adc_theta = path_point.theta();
      }else{
        // AINFO<<"  use diagonal heading";
      }
    }
  }
  // AINFO << "adc_theta = " << adc_theta;
  ego_center_map_frame.SelfRotate(adc_theta);
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_theta, vehicle_param_.length(),
                vehicle_param_.width() + l_buffer_ * 2);

  // collision detection based on SLBoundary.
  SLBoundary adc_sl_boundary;
  if (!reference_line_.GetSLBoundary(adc_box, &adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << adc_box.DebugString();
    return false;
  }
  // need add s buffer.
  bool no_overlap =
      ((adc_sl_boundary.end_s() < obs_sl_boundary.start_s() ||
        adc_sl_boundary.start_s() > obs_sl_boundary.end_s()) ||  // longitudinal
       (adc_sl_boundary.end_l() + l_buffer_ < obs_sl_boundary.start_l() ||
        adc_sl_boundary.start_l() - l_buffer_ >
            obs_sl_boundary.end_l()));  // lateral
  if (no_overlap) {
    return false;
  }

  ADEBUG << "There is an overlap. adc_sl_boundary.start_s() = "
         << adc_sl_boundary.start_s()
         << ", adc_sl_boundary.end_s() = " << adc_sl_boundary.end_s()
         << ", obs_sl_boundary.start_s() = " << obs_sl_boundary.start_s()
         << ", obs_sl_boundary.end_s() = " << obs_sl_boundary.end_s()
         << ", adc_sl_boundary.start_l() = " << adc_sl_boundary.start_l()
         << ", adc_sl_boundary.end_l() = " << adc_sl_boundary.end_l()
         << ", obs_sl_boundary.start_l() = " << obs_sl_boundary.start_l()
         << ", obs_sl_boundary.end_l() = " << obs_sl_boundary.end_l();

  // collision detection based on Box2d.
  return obs_polygon.HasOverlap(common::math::Polygon2d(adc_box));
}

void STBoundaryMapper::GetClosestStopDecision(
    Obstacle* current_stop_obstacle,
    const ObjectDecisionType& current_stop_decision,
    Obstacle** const closest_stop_obstacle,
    ObjectDecisionType* const closest_stop_decision,
    double* const min_stop_s) const {
  CHECK_NOTNULL(current_stop_obstacle);
  CHECK_NOTNULL(closest_stop_obstacle);
  CHECK_NOTNULL(closest_stop_decision);
  CHECK_NOTNULL(min_stop_s);

  /**
   * Because the STOP decision is added to the obstacle,
   * `decision.stop().distance_s()` is a negative value which indicates that
   * the ADC should stop at `decision.stop().distance_s()`meters behind the
   * obstacle. If `stop_s` is already behind the ADC, we try to
   * decrease `stop_s` to determine whether `stop_s` is in front of the ADC. If
   * `stop_s` is still behind the ADC,  we try to  add a new STOP decision.
   * If the obstacle is already behind the ADC, just ignore it.
   *
   */
  const auto& obstacle_sl = current_stop_obstacle->PerceptionSLBoundary();
  auto obs_start_s = obstacle_sl.start_s();
  auto obs_end_s = obstacle_sl.end_s();
  auto adc_start_s = adc_sl_boundary_.start_s();
  auto adc_end_s = adc_sl_boundary_.end_s();
  double stop_s = obs_start_s + current_stop_decision.stop().distance_s() +
                  kHysteresisDistance;
  ObjectDecisionType modified_stop_decision = current_stop_decision;
  bool has_large_stop_distance_to_slow_break =
      current_stop_decision.stop().distance_s() < -kStopBuffer;
  // The buffer distance is not enough for a normal STOP.
  if (stop_s < adc_end_s && has_large_stop_distance_to_slow_break &&
      FLAGS_enable_modify_stop_distance) {
    AWARN << "stop reason " << current_stop_decision.stop().reason_code()
          << "id = " << current_stop_obstacle->Id();
    AWARN << "Invalid normal stop decision. not stop at behind of current "
             "position. stop_s = "
          << stop_s << ",  stop distance buffer = "
          << -current_stop_decision.stop().distance_s();
    AWARN << "ADC position: (start_s, end_s) = (" << adc_start_s << ", "
          << adc_end_s << "). ";
    AWARN << "Obstacle position: (start_s, end_s) = (" << obs_start_s << ", "
          << obs_end_s << "). ";
    AWARN << "First try to reduce the stop distance by half. ";
    // stop_decision_length is a positive value.
    double stop_decision_length =
        -0.5 * current_stop_decision.stop().distance_s();
    stop_s = obs_start_s - stop_decision_length;
    // The buffer distance is still not enough for a normal STOP.
    if (stop_s < adc_end_s) {
      if (adc_start_s > obs_end_s + FLAGS_min_stop_distance_obstacle) {
        ADEBUG << "The obstacle is already behind the ADC and ignored. ";
        return;
      } else {
        stop_decision_length =
            std::max(obs_start_s - adc_end_s - kStopBuffer, kStopBuffer);
        AWARN << "Invalid stop decsion. stop_s < adc_end_s. stop_s = " << stop_s
              << ". Second try to stop" << stop_decision_length
              << "meters behind the obstacle.";
        stop_s = obs_start_s - stop_decision_length;
      }
    }

    const auto& stop_point = reference_line_.GetReferencePoint(stop_s);
    double stop_heading = reference_line_.GetReferencePoint(stop_s).heading();

    auto* modified_stop = modified_stop_decision.mutable_stop();
    // distance_s must be a negative value.
    modified_stop->set_distance_s(-stop_decision_length);
    modified_stop->set_stop_heading(stop_heading);
    modified_stop->mutable_stop_point()->set_x(stop_point.x());
    modified_stop->mutable_stop_point()->set_y(stop_point.y());
    modified_stop->mutable_stop_point()->set_z(0.0);

    AWARN << "Dangerous stop decision! stop_s = " << stop_s
          << ",  current ADC's position is: (start_s, end_s) = (" << adc_start_s
          << ", " << adc_end_s
          << ").  the obstacle's position is: (start_s, end_s) = ("
          << obs_start_s << ", " << obs_end_s << ").";
  }

  if (stop_s < *min_stop_s) {
    *closest_stop_obstacle = current_stop_obstacle;
    *closest_stop_decision = std::move(modified_stop_decision);
    *min_stop_s = stop_s;
  }
}
void STBoundaryMapper::CalcuMaxCollisionPathLength() {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  double center_s =
      0.5 * (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s());
  double speed_limit_from_ref_line =
      reference_line_.GetSpeedLimitFromS(center_s);
  auto max_speed_limit = speed_limit_.GetMaxSpeedLimitBySRange(
      0.0, speed_limit_from_ref_line * planning_max_time_);
  double t_acc =
      (max_speed_limit - adc_speed) / vehicle_param_.max_acceleration();
  t_acc = std::max(0.0, std::min(planning_max_time_, t_acc));
  double s_acc = 0.5 * (max_speed_limit + adc_speed) * t_acc;
  double s_uniform = max_speed_limit * (planning_max_time_ - t_acc);
  max_collision_path_len_ = std::max(
      kCollisionPathMagnifyRatio * (s_acc + s_uniform), kMinCollisionPathLen);
  ADEBUG << "center_s: " << center_s;
  ADEBUG << "max_speed_limit:" << max_speed_limit << ", adc_speed:" << adc_speed
         << ", s_acc:" << s_acc << ", s_uniform" << s_uniform;
  ADEBUG << "max_collision_path_len = " << max_collision_path_len_;
}

bool STBoundaryMapper::IsPathOnHighRight() const {
  bool is_adc_high_road_right_beginning_in_reference_line = true;
  bool is_adc_high_road_right_beginning_out_reference_line = true;
  bool is_adc_high_road_right_beginning = false;

  // maby map no type or no all point has type ,so can't believe alone
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
    if (path_pt_s >
        max_collision_path_len_ +
            (adc_sl_boundary_.end_s() + adc_sl_boundary_.start_s()) * 0.5) {
      break;
    }

    if ((path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
         path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE ||
         path_pt_type == PathData::PathPointType::UNKNOWN) &&
        is_adc_high_road_right_beginning_in_reference_line) {
      is_adc_high_road_right_beginning_in_reference_line = false;
      ADEBUG << "has neighbor lane path.";
    }
    if (path_pt_type == PathData::PathPointType::IN_LANE &&
        is_adc_high_road_right_beginning_out_reference_line) {
      is_adc_high_road_right_beginning_out_reference_line = false;
      ADEBUG << "has reference lane path.";
    }
  }
  if (path_data_.path_point_decision_guide().empty()) {
    ADEBUG << "no path point decision guide.";
    is_adc_high_road_right_beginning_in_reference_line = false;
    is_adc_high_road_right_beginning_out_reference_line = false;
  }
  if (is_adc_high_road_right_beginning_out_reference_line &&
      !is_adc_high_road_right_beginning_in_reference_line) {
    ADEBUG << "hight road right,in neighbor lane.";
    is_adc_high_road_right_beginning = true;
  }
  if (is_adc_high_road_right_beginning_in_reference_line &&
      !is_adc_high_road_right_beginning_out_reference_line) {
    ADEBUG << "hight road right,in reference lane.";
    is_adc_high_road_right_beginning = true;
  }
  if (!is_adc_high_road_right_beginning_out_reference_line &&
      !is_adc_high_road_right_beginning_in_reference_line) {
    ADEBUG << "low road right.";
    is_adc_high_road_right_beginning = false;
  }

  return is_adc_high_road_right_beginning;
}

bool STBoundaryMapper::IsNeedToTurn(const bool is_path_onlane) {
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  const double adc_s =
      (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
  double init_adc_l = path_data_.frenet_frame_path().front().l();
  bool is_need_to_turn = false, is_path_in_turning = false,
       at_turn_area = false;

  path_turn_right_ = false;
  path_turn_left_ = false;

  at_left_turn_ = false;
  at_right_turn_ = false;
  // is too large,need to test more
  size_t path_step = kPathStep;  // 2m
  double adc_v = injector_->vehicle_state()->linear_velocity(),
         comfort_slow_down_length =
             std::fabs(adc_v * adc_v * 0.5 / kComfortDecel);
  double consider_s = std::max(std::min(max_collision_path_len_ + adc_s,
                                        comfort_slow_down_length + adc_s),
                               kMinConsiderLength + adc_s);
  first_turn_point_s_ = consider_s;
  // AINFO << "max_collision_path_len_ = " << max_collision_path_len_;
  // AINFO << "consider_s = " << consider_s;
  for (size_t i = 0; i < discretized_path.size(); i += path_step) {
    const auto& frenet_point = frenet_path[i];
    if (frenet_point.s() > consider_s) {
      break;
    }
    double temp_diff = frenet_point.l() - init_adc_l;
    // check path is turn or nor;
    if (std::fabs(temp_diff) > kMinLateralDiff && !is_path_in_turning) {
      if (temp_diff < 0.0) {
        path_turn_right_ = true;
        is_path_in_turning = true;
      } else {
        path_turn_left_ = true;
        is_path_in_turning = true;
      }
    }
    if (!at_turn_area) {
      // use map information
      std::vector<hdmap::LaneInfoConstPtr> lanes;
      reference_line_.GetLaneFromS(frenet_point.s(), &lanes);
      if (!lanes.empty() && lanes.front() != nullptr) {
        const auto& lane = lanes.front()->lane();
        if (!is_in_merage_) {
          is_in_merage_ = lane.is_merge();
        }
        if (lane.has_turn()) {
          if (Lane::LEFT_TURN == lane.turn()) {
            at_turn_area = true;
            at_left_turn_ = true;
            // ADEBUG << "LEFT_TURN";
          } else if (Lane::RIGHT_TURN == lane.turn()) {
            at_turn_area = true;
            at_right_turn_ = true;
            // ADEBUG << "RIGHT_TURN";
          } else if (Lane::U_TURN == lane.turn()) {
            // ADEBUG << "U_TURN";
          } else {
            // ADEBUG << "no turn";
          }
        }
      }

    } else {
      // in laneborrow ,we no need to consider the road turn,on consider path
      // turn direction
      continue;
    }
    //
    if (is_path_in_turning && at_turn_area) {
      break;
    }
  }

  if (is_path_in_turning || at_turn_area) {
    is_need_to_turn = true;
  }

  return is_need_to_turn;
}

bool STBoundaryMapper::GetRoadRightRelativeObstacle(
    const Obstacle& obstacle, const bool is_left_side_obs,
    const bool is_right_side_obs, const bool is_high_right) const {
  static HysteresisInterval adc_obstacles_heading_interval(
      speed_bounds_config_.max_diff_angle_for_same_orientation(),
      speed_bounds_config_.hy_buffer_lower_for_same_orientation(),
      speed_bounds_config_.hy_buffer_upper_for_same_orientation(), kHyCapacity);
  bool is_in_lateral_range = false;
  std::string hy_obs_id = std::to_string(obstacle.PerceptionId());
  // ADEBUG << "obstacle.SpeedHeading() = " << obstacle.SpeedHeading()
  //        << "   injector_->vehicle_state()->heading() = "
  //        << injector_->vehicle_state()->heading();
  const double diff_heading =
      common::math::NormalizeAngle(obstacle.SpeedHeading() -
                                   injector_->vehicle_state()->heading()) *
      kDegrees / M_PI_2;
  double hy_diff_heading = diff_heading;
  bool is_face_to_path = false;
  if (diff_heading > 0.0) {
    hy_obs_id += "_a";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, diff_heading);
    if (is_right_side_obs) {
      is_face_to_path = true;
    }
  } else {
    hy_obs_id += "_b";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, -diff_heading);
    if (is_left_side_obs) {
      is_face_to_path = true;
    }
  }

  // ADEBUG << "ID = " << obstacle.Id()
  //        << "       is_face_to_path = " << is_face_to_path;
  // ADEBUG << "is_left_side_obs = " << is_left_side_obs
  //        << "    is_right_side_obs = " << is_right_side_obs;
  //  hy_diff_heading is not negative value.
  // ADEBUG << "hy_obs_id: " << hy_obs_id << ", diff_heading: " << diff_heading
  //        << " degree, hy_diff_heading: " << hy_diff_heading << " degree";
  const bool is_same_orientation =
      hy_diff_heading <
      speed_bounds_config_.max_diff_angle_for_same_orientation();
  const auto& obstacle_sl = obstacle.PerceptionSLBoundary();
  if (is_left_side_obs) {
    is_in_lateral_range =
        obstacle_sl.start_l() - adc_sl_boundary_.end_l() < kConsiderLatBuffer &&
        obstacle_sl.start_l() - adc_sl_boundary_.end_l() > 0.0;
  } else if (is_right_side_obs) {
    is_in_lateral_range =
        adc_sl_boundary_.start_l() - obstacle_sl.end_l() < kConsiderLatBuffer &&
        adc_sl_boundary_.start_l() - obstacle_sl.end_l() > 0.0;
  }
  bool is_in_longitude_range = false;
  if (obstacle_sl.end_s() > adc_sl_boundary_.start_s() &&
      obstacle_sl.end_s() < adc_sl_boundary_.end_s()) {
    is_in_longitude_range = true;
  }
  // ADEBUG << "obstacle_sl.end_s() = " << obstacle_sl.end_s();
  // ADEBUG << "is_in_longitude_range = " << is_in_longitude_range;
  // ADEBUG << "is_in_lateral_range = " << is_in_lateral_range;
  // ADEBUG << "is_same_orientation = " << is_same_orientation;
  bool is_high_right_of_way_compare_obs = true;
  if ((!is_same_orientation && is_face_to_path) ||
      (is_in_lateral_range && is_in_longitude_range)) {
    is_high_right_of_way_compare_obs = false;
  }
  if (!is_high_right) {
    is_high_right_of_way_compare_obs = false;
  }
  // ADEBUG << "is_high_right_of_way_compare_obs = "
  //        << is_high_right_of_way_compare_obs;
  // ADEBUG << "at_left_turn = " << at_left_turn_
  //        << " is_left_side_obs = " << is_left_side_obs
  //        << " at_right_turn = " << at_right_turn_
  //        << " is_right_side_obs = " << is_right_side_obs;
  // ADEBUG << "at_left_turn = " << at_left_turn_
  //        << " is_left_side_obs = " << is_left_side_obs
  //        << " at_right_turn = " << at_right_turn_
  //        << " is_right_side_obs = " << is_right_side_obs;
  // adc must at turn area.
  is_high_right_of_way_compare_obs =
      CheckRoadRightInTurn(obstacle, is_left_side_obs, is_right_side_obs,
                           hy_diff_heading, is_high_right_of_way_compare_obs);
  return is_high_right_of_way_compare_obs;
}

bool STBoundaryMapper::CheckRoadRightInTurn(
    const Obstacle& obstacle, bool is_left_side_obs, bool is_right_side_obs,
    double hy_diff_heading, bool is_high_right_of_way_compare_obs) const {
  bool ret = is_high_right_of_way_compare_obs;
  const double diff_heading =
      common::math::NormalizeAngle(obstacle.SpeedHeading() -
                                   injector_->vehicle_state()->heading()) *
      kDegrees / M_PI_2;
  if (((at_left_turn_) || (at_right_turn_)) &&
      !is_high_right_of_way_compare_obs) {
    // obs to adc relative heading.
    double backward_distance = vehicle_param_.front_edge_to_center();
    double adc_heading = injector_->vehicle_state()->heading();
    double adc_x = injector_->vehicle_state()->x() +
                   backward_distance * std::cos(adc_heading);
    double adc_y = injector_->vehicle_state()->y() +
                   backward_distance * std::sin(adc_heading);
    double obs_x = obstacle.Perception().position().x();
    double obs_y = obstacle.Perception().position().y();
    double dx = adc_x - obs_x;
    double dy = adc_y - obs_y;
    double heading_obs_to_adc = std::atan2(dy, dx);  // calculate the heading

    double diff_theta_between_adc_and_obs = common::math::NormalizeAngle(
        obstacle.SpeedHeading() - heading_obs_to_adc);
    ADEBUG << "heading_obs_to_adc =               "
           << heading_obs_to_adc * kDegrees / M_PI_2
           << "\n obstacle.SpeedHeading =          "
           << obstacle.SpeedHeading() * kDegrees / M_PI_2
           << "\n diff_theta_between_adc_and_obs = "
           << diff_theta_between_adc_and_obs * kDegrees / M_PI_2
           << " \n adc_heading=                    "
           << injector_->vehicle_state()->heading() * kDegrees / M_PI_2;
    ADEBUG << "diff_heading = " << diff_heading;
    // adc in obstacle's middle or left && obs in path left && adc no face
    // obstacle's trajectory
    double min_same_angle = speed_bounds_config_.min_same_angle();
    if (obstacle.speed() > kObsHighSpeed) {
      min_same_angle = kMinSameAngle;
    }
    if (diff_theta_between_adc_and_obs * kDegrees / M_PI_2 < min_same_angle &&
        is_left_side_obs && diff_heading <= 0.0 &&
        -hy_diff_heading > -kMaxDegrees &&
        FLAGS_enable_skip_back_obstacle_in_the_same_line) {
      ret = true;
    }
    if (diff_theta_between_adc_and_obs * kDegrees / M_PI_2 > -min_same_angle &&
        is_right_side_obs && diff_heading > 0.0 &&
        hy_diff_heading < kMaxDegrees &&
        FLAGS_enable_skip_back_obstacle_in_the_same_line) {
      ret = true;
    }
  }
  return ret;
}

bool STBoundaryMapper::IsSameDirection(const Obstacle& obstacle,
                                       double path_point_theta) const {
  static HysteresisInterval adc_obstacles_heading_interval(
      speed_bounds_config_.max_diff_angle_for_same_orientation(),
      speed_bounds_config_.hy_buffer_lower_for_same_orientation(),
      speed_bounds_config_.hy_buffer_upper_for_same_orientation(), kHyCapacity);
  std::string hy_obs_id = std::to_string(obstacle.PerceptionId());
  ADEBUG << "bstacle->SpeedHeading() = " << obstacle.SpeedHeading();
  const double diff_heading =
      common::math::NormalizeAngle(obstacle.SpeedHeading() - path_point_theta) *
      kDegrees / M_PI_2;
  double hy_diff_heading = diff_heading;
  if (diff_heading > 0.0) {
    hy_obs_id += "_c";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, diff_heading);
  } else {
    hy_obs_id += "_d";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, -diff_heading);
  }
  // hy_diff_heading is not negative value.
  ADEBUG << "hy_obs_id: " << hy_obs_id << ", diff_heading: " << diff_heading
         << " degree, hy_diff_heading: " << hy_diff_heading << " degree";
  const bool is_same_orientation =
      hy_diff_heading <
      speed_bounds_config_.max_diff_angle_for_same_orientation();

  ADEBUG << "is_same_orientation = " << is_same_orientation;
  return is_same_orientation;
}

bool STBoundaryMapper::IsCanSkipObstacle(
    const Obstacle& obstacle,
    const bool& is_adc_high_road_right_beginning) const {
  const auto& obstacle_sl = obstacle.PerceptionSLBoundary();
  bool is_high_right_of_way_compare_obs = false;
  bool is_left_side_obs = obstacle_sl.start_l() > adc_sl_boundary_.end_l();
  bool is_right_side_obs = obstacle_sl.end_l() < adc_sl_boundary_.start_l();
  bool is_back_obs = obstacle_sl.end_s() < adc_sl_boundary_.start_s();
  bool is_all_in_back_side = obstacle_sl.end_s() < adc_sl_boundary_.start_s();
  // only check the obstacle which back and side of adc；
  ADEBUG << "is_back_obs = " << is_back_obs
         << "  obstacle_sl.start_l() =" << obstacle_sl.start_l()
         << "     obstacle_sl.end_l() = " << obstacle_sl.end_l();
  if (is_back_obs && util::IsLaneChange(injector_->planning_context())) {
    return true;
  }
  const auto& candidate_path_data =
      reference_line_info_->GetCandidatePathData();
  bool is_selfborrow =
      (std::string::npos !=
           candidate_path_data.front().path_label().find("self") &&
       std::string::npos !=
           candidate_path_data.front().path_label().find("regular"));
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
  if (is_back_obs && (is_left_side_obs || is_right_side_obs)) {
    // in low right，check path and road direction,need to turn
    if (!is_adc_high_road_right_beginning) {
      if (path_turn_left_ && !at_right_turn_ && is_right_side_obs) {
        // path left，road straight or left
        is_high_right_of_way_compare_obs = true;
      } else if (path_turn_right_ && !at_left_turn_ && is_left_side_obs) {
        // path right ,road straight or right
        is_high_right_of_way_compare_obs = true;
      } else if (!path_turn_left_ && at_right_turn_ && is_left_side_obs) {
        // road right，path forward or right
        is_high_right_of_way_compare_obs = true;
      } else if (!path_turn_right_ && at_left_turn_ && is_right_side_obs) {
        // road left，path forward or left
        is_high_right_of_way_compare_obs = true;
      } else if (path_turn_left_ && !at_right_turn_ && !at_left_turn_ &&
                 is_left_side_obs && is_selfborrow && is_all_in_back_side) {
        // if path is selfborrow,no consider back side obs.
        is_high_right_of_way_compare_obs = true;
      } else if (path_turn_right_ && !at_left_turn_ && !at_right_turn_ &&
                 is_right_side_obs && is_selfborrow && is_all_in_back_side) {
        is_high_right_of_way_compare_obs = true;
      } else {
        is_high_right_of_way_compare_obs = false;
      }
    }
    is_high_right_of_way_compare_obs = GetRoadRightRelativeObstacle(
        obstacle, is_left_side_obs, is_right_side_obs,
        is_high_right_of_way_compare_obs);


    // no need to turn or in turn but has diff direction.
    if (((is_adc_high_road_right_beginning &&
          !reference_line_info_->IsAdcInCommonJunction()) ||
         is_high_right_of_way_compare_obs) &&
        FLAGS_enable_skip_back_obstacles) {
      return true;
    }
  } else if (is_all_in_back_side && !(is_left_side_obs || is_right_side_obs) &&
             FLAGS_enable_skip_back_side_and_has_overlap_obs) {
    double adc_heading = injector_->vehicle_state()->vehicle_state().heading();
    double adc_ref_heading =
        reference_line_
            .GetReferencePoint(
                (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5)
            .heading();

    double adc_and_refpoint_theta_diff = std::fabs(
        century::common::math::NormalizeAngle(adc_ref_heading - adc_heading));
    if (adc_and_refpoint_theta_diff < kMinDiffTheta) {
      // in merage area or turn need to test.
      ADEBUG << " back side obs and has overlap and adc and ref_point same "
                "direction,go straight,skip obs !!!";
      return true;
    } else {
      bool adc_out_left_line = adc_sl_boundary_.end_l() > adc_lane_left_width_;
      bool adc_out_right_line =
          adc_sl_boundary_.start_l() < -adc_lane_right_width_;

      double init_adc_l = path_data_.frenet_frame_path().front().l();
      if (!adc_out_left_line && !adc_out_right_line) {
        ADEBUG << "adc no out lane";
        if (is_selfborrow) {
          ADEBUG << "adc in lane and in self borrow  and with backside obs has "
                    "overlap ,skip";
          return true;
        }
        if (is_left_borrow) {
          ADEBUG << "obstacle_sl.end_l() = " << obstacle_sl.end_l()
                 << "      adc_sl_boundary_.end_l()="
                 << adc_sl_boundary_.end_l();
          if (!(obstacle_sl.end_l() > adc_sl_boundary_.end_l())) {
            ADEBUG << "in left borrow,obs no left side obs,is right back,skip";
            return true;
          }
          if (reference_line_info_->IsOnLane(obstacle)) {
            ADEBUG << "in left borrow,has lateral overlap ,obs is onlane,skip.";
            return true;
          }

          ADEBUG
              << "need to left borrow ,adc inlane and obs in left lane,no skip";
        }
        if (is_right_borrow) {
          if (!(obstacle_sl.start_l() < adc_sl_boundary_.start_l())) {
            ADEBUG << "in right borrow,obs no right obs,is left back obs,skip";
            return true;
          }
          if (reference_line_info_->IsOnLane(obstacle)) {
            ADEBUG
                << "in right borrow,has lateral overlap ,obs is onlane,skip.";
            return true;
          }

          ADEBUG << "need to right borrow ,adc inlane and obs in right lane,no "
                    "skp.";
        }

      } else if (adc_out_left_line && !adc_out_right_line) {
        if (init_adc_l > adc_lane_left_width_) {
          ADEBUG << "ADC all in left lane";
          // need to check adc heading is toward ref lane.
          // if adc in  go back to ref lane,consider ref lane's obs.
          if (reference_line_info_->IsOnLane(obstacle)) {
            ADEBUG << "adc all in left lane,ref lane's obs , no skip.";
            return false;
          } else {
            // if adc in go to left lane, and all in left lane, no consider ref
            // lane obs.
            ADEBUG << "adc all in left lane ,left back side obs ,skip.";
            return true;
          }
        } else {
          ADEBUG << "adc no all in left lane.";
          if (!(obstacle_sl.end_l() > adc_sl_boundary_.end_l())) {
            ADEBUG << "in left borrow,obs no left side obs,is right back,skip";
            return true;
          }
          if (reference_line_info_->IsOnLane(obstacle)) {
            ADEBUG << "in left borrow,has lateral overlap ,obs is on ref "
                      "lane,skip.";
            return true;
          }
          ADEBUG
              << "need to left borrow ,adc inlane and obs in left lane,no skip";
        }
      } else if (!adc_out_left_line && adc_out_right_line) {
        if (init_adc_l < -adc_lane_right_width_) {
          ADEBUG << "adc all in right lane";
          if (reference_line_info_->IsOnLane(obstacle)) {
            ADEBUG << "adc in right lane,consider ref lane's obs.";
            return false;
          } else {
            ADEBUG << "adc in right lane ,no consider obs without ref lane";
          }
        } else {
          if (!(obstacle_sl.start_l() > adc_sl_boundary_.start_l())) {
            return true;
          }
          if (reference_line_info_->IsOnLane(obstacle)) {
            return true;
          }
        }
      }
    }
  }

  // front no skip.Directly behind already skip in reference_line_info
  return false;
}

}  // namespace planning
}  // namespace century
