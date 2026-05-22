
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
#include "modules/planning/tasks/optimizers/teb_trajectory_partition/teb_trajectory_partition.h"

#include <algorithm>
#include <memory>
#include <queue>

#include "absl/strings/str_cat.h"

#include "cyber/time/clock.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_stabilization_for_teb_speed.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {
namespace {
constexpr int kIdDistToEnd = 6;
constexpr int kCurveNum = 10;
constexpr double kBackReachS = -0.25;
constexpr double kReachS = 0.35;
constexpr double kSmoothPercent = 0.4;
constexpr double kMinSpeedLimitCurve = 0.06;
constexpr double kMaxSpeedLimitCurve = 0.26;
constexpr double kLargeDist = 100.0;
constexpr double kMinDist = 0.01;
constexpr double kEpsilon = 0.001;
constexpr double kMinSpeed = 0.35;
constexpr double kStopSpeed = 0.05;
constexpr int kIsEmptyIntValue = 0;          // int size
constexpr double kInterpolatePitch_S = 0.1;  // m
constexpr int kPolyfitCoefficient = 5;       // m
constexpr double kPolyfitEndStep = 0.01;     // m

constexpr int kControlStep = 2;
constexpr double kChangeKappa = 0.041;  //
constexpr int kBsplineOrder = 3;
constexpr int kIdRange = 4;
constexpr int kSmoothStep = 2;
constexpr double kMaxKappa = 0.25;

constexpr double kSafeBuffer = 0.2;                  // m
constexpr double kLonSafeBufferSpeedCoff = 2.0;      // 0.5
constexpr double kLatSafeBufferSpeedCoff = 0.6;      // 0.3
constexpr double kVehicleStaticThr = 0.05;           // m/s
constexpr double kSlowDownDisThr = 15.0;             // m
constexpr double kSpeedLimitRange = 1.0 + kEpsilon;  // m
constexpr double kNearCoeff = 0.8;

constexpr size_t kZero = 0;
constexpr double kRoutingEndExtendLength = 1.5;
constexpr double kPullOverHeadingThr = 0.1;
constexpr double kPullOverReachedDisThr = 0.3;

// Optimized
constexpr double kOptimizedTotalTime = 7.0;
constexpr double kOptimizedDeltaT = 0.1;
constexpr double kComfortDecel = 1.0;
constexpr double kComfortAcc = 0.5;
constexpr double kUturnComfortDecel = 0.05;
constexpr double kUturnComfortAcc = 0.2;
constexpr double kMinDecel = -6.0;
constexpr double kMaxAcc = 1.5;
constexpr double kStoppedSpeed = 0.001;
constexpr double kStoppedLength = 0.01;
constexpr double kTargetSpeed = 1.0;
constexpr double kBreakLength = 1.0;
constexpr double kCollisionBuffer = 1.0;
constexpr double kMinCollisionBuffer = 0.2;
constexpr double kCollisionLateralBuffer = 0.4;
constexpr double kCollisionLonBuffer = 0.4;
constexpr double kDepartureDistance = 1.0;
constexpr double kMinLength = 0.1;
constexpr double kPedCheckTime = 0.8;
constexpr double kDeltaT = 0.2;
constexpr size_t kMinSizePoint = 4;
constexpr size_t kOneSizePoint = 1;
constexpr size_t kTwoSizePoint = 2;
constexpr double kMinSpeedStartup = 0.15;
constexpr double kConsiderSpeedStartup = 0.20;
constexpr double kMaxDistanceToDestination = 0.3;
constexpr double kMinDistanceToDestination = 0.1;
constexpr double kMinOptimizedTotalTime = 1.0;
constexpr size_t kSearchIndexBuffer = 5UL;
constexpr double kMinSearchStepDistance = 0.5;
constexpr double kSearchDistanceRatio = 0.9;
constexpr double kMaxSearchStepRatio = 0.75;
constexpr size_t kFineSearchStep = 2UL;
constexpr double kMinTrajectoryLength = 0.01;
constexpr double kStepDistance = 0.1;
}  // namespace

using century::common::ErrorCode;
using century::common::PathPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Box2d;
using century::common::math::NormalizeAngle;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;

TEBTrajectoryPartition::TEBTrajectoryPartition(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : TrajectoryOptimizer(config, injector),
      teb_trajectory_partition_config_(
          config_.teb_trajectory_partition_config()) {
  heading_search_range_ =
      teb_trajectory_partition_config_.heading_search_range();
  heading_track_range_ = teb_trajectory_partition_config_.heading_track_range();
  distance_search_range_ =
      teb_trajectory_partition_config_.distance_search_range();
  heading_offset_to_midpoint_ =
      teb_trajectory_partition_config_.heading_offset_to_midpoint();
  lateral_offset_to_midpoint_ =
      teb_trajectory_partition_config_.lateral_offset_to_midpoint();
  longitudinal_offset_to_midpoint_ =
      teb_trajectory_partition_config_.longitudinal_offset_to_midpoint();
  vehicle_box_iou_threshold_to_midpoint_ =
      teb_trajectory_partition_config_.vehicle_box_iou_threshold_to_midpoint();
  linear_velocity_threshold_on_ego_ =
      teb_trajectory_partition_config_.switch_trace_thr_v();
  up_dist_threshold_on_ego_ =
      teb_trajectory_partition_config_.up_dist_threshold_on_ego();
  down_dist_threshold_on_ego_ =
      teb_trajectory_partition_config_.down_dist_threshold_on_ego();
  slow_down_min_speed_ = teb_trajectory_partition_config_.slow_down_min_speed();
  switch_trace_thr_s_ = teb_trajectory_partition_config_.switch_trace_thr_s();
  switch_trace_thr_dis_ =
      teb_trajectory_partition_config_.switch_trace_thr_dis();
  use_new_trajectory_partition_ =
      teb_trajectory_partition_config_.use_new_trajectory_partition();
  use_new_trajectory_speed_cal_ =
      teb_trajectory_partition_config_.use_new_trajectory_speed_cal();
  teb_trajectory_near_end_ =
      teb_trajectory_partition_config_.teb_trajectory_near_end();
  teb_max_speed_limit_ = teb_trajectory_partition_config_.teb_max_speed_limit();
  teb_min_speed_limit_ = teb_trajectory_partition_config_.teb_min_speed_limit();
  teb_min_curve_thr_ = teb_trajectory_partition_config_.teb_min_curve_thr();
  teb_max_curve_thr_ = teb_trajectory_partition_config_.teb_max_curve_thr();
  teb_max_curve_speed_limit_ =
      teb_trajectory_partition_config_.teb_max_curve_speed_limit();
  flu_up_dist_threshold_on_ego_ =
      teb_trajectory_partition_config_.flu_up_dist_threshold_on_ego();

  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param_.length();
  ego_width_ = vehicle_param_.width();
  front_to_center_ = vehicle_param_.front_edge_to_center();
  back_to_center_ = vehicle_param_.back_edge_to_center();
  shift_distance_ = ego_length_ * 0.5 - vehicle_param_.back_edge_to_center();
}

bool TEBTrajectoryPartition::CalRemainDis(
    const DiscretizedTrajectory& current_trajectory,
    const common::math::Vec2d adc_position, size_t& nearest_index,
    double& remain_dis) {
  if (current_trajectory.empty()) {
    return false;
  }
  nearest_index = current_trajectory.QueryNearestPoint(adc_position);
  AINFO << "nearest_index: " << nearest_index;
  if (current_trajectory.size() - 1 == nearest_index) {
    AERROR << "adc is near end point.";
    remain_dis = 0.0;
    return true;
  }
  float distance = 0.0;
  for (size_t i = nearest_index; i < current_trajectory.size() - 1; ++i) {
    distance += std::hypot(current_trajectory.at(i + 1).path_point().x() -
                               current_trajectory.at(i).path_point().x(),
                           current_trajectory.at(i + 1).path_point().y() -
                               current_trajectory.at(i).path_point().y());
  }
  remain_dis = distance;
  return true;
}

bool TEBTrajectoryPartition::CalSwitchToNextTrace(
    const DiscretizedTrajectory& current_trajectory,
    const canbus::Chassis::GearPosition& current_gear,
    const common::VehicleStateProvider* vehicle_state) {
  const common::math::Vec2d adc_position = {vehicle_state->x(),
                                            vehicle_state->y()};
  TrajectoryPoint end_point = (current_trajectory.at(current_trajectory.size() - 1));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  double remain_s = 10000;
  size_t nearest_index = 0;
  CalRemainDis(current_trajectory, adc_position, nearest_index, remain_s);
  double dx = end_point.path_point().x() - injector_->vehicle_state()->x();
  double dy = end_point.path_point().y() - injector_->vehicle_state()->y();
  double adc2end_dis = std::hypot(dx, dy);

  AINFO << "Switch:"
        << " remain_s:" << std::setprecision(3) << remain_s
        << " dist:" << std::setprecision(3) << adc2end_dis
        << " speed:" << std::setprecision(3) << adc_speed
        << " gear:" << current_gear << ", cur_ind == traj_end: "
        << (nearest_index == static_cast<int>(current_trajectory.size() - 1))
        << ", nearest_index: " << nearest_index
        << ", traj_size: " << current_trajectory.size();
  AINFO << "Switch:"
        << " s_thr_:" << switch_trace_thr_s_
        << " dis_thr_:" << switch_trace_thr_dis_
        << " speed_thr_:" << linear_velocity_threshold_on_ego_;

  if (remain_s < switch_trace_thr_s_ &&
      std::fabs(adc_speed) < linear_velocity_threshold_on_ego_ &&
      ((adc2end_dis < switch_trace_thr_dis_) ||
       (adc2end_dis >= switch_trace_thr_dis_ && remain_s < kEpsilon &&
        current_trajectory.size() - 1 == nearest_index))) {
    AINFO << "need switch next path!!!";
    return true;
  }
  return false;
}

Status TEBTrajectoryPartition::Process() {
  obstacle_trajectory_info_.clear();
  is_stop_trajectory_ = false;
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  const auto& open_space_status =
      injector_->planning_context()->planning_status().open_space();

  const auto& stitched_trajectory_result =
      open_space_info.stitched_trajectory_result();

  if (kIsEmptyIntValue == static_cast<int>(stitched_trajectory_result.size())) {
    AERROR << "Openspace_Planning_Result: stitched trajectorys is empty";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Openspace_Planning_Result: stitched trajectorys is empty");
  }

  auto* interpolated_trajectory_result_ptr =
      open_space_info_ptr->mutable_interpolated_trajectory_result();

  InterpolateTrajectory(stitched_trajectory_result,
                        interpolated_trajectory_result_ptr);

  bool planning_finished_has_new_traces = !open_space_status.position_init();
  auto* partitioned_trajectories =
      open_space_info_ptr->mutable_partitioned_trajectories();
  if (planning_finished_has_new_traces) {
    PartitionTrajectory(*interpolated_trajectory_result_ptr,
                        partitioned_trajectories);
    last_partitioned_trajectories_ = *partitioned_trajectories;
  } else {
    *partitioned_trajectories = last_partitioned_trajectories_;
  }

  size_t trajectories_size = partitioned_trajectories->size();
  if (trajectories_size < 1U) {
    AERROR << "error, Partitioned_Result: partitioned trajectorys is empty";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Partitioned_Result: partitioned trajectorys is empty");
  }
  AINFO << "Partitioned: trajectories_size:" << trajectories_size
        << ", planning_finished_has_new_traces: "
        << planning_finished_has_new_traces;

  UpdateVehicleInfo();

  // Choose the one to follow based on the closest partitioned
  // current_trajectory
  size_t current_trajectory_index = 0;
  size_t current_trajectory_point_index = 0;
  century::planning::DiscretizedTrajectory current_trajectory;

  bool switch_to_next_trace_flag = false;
  if (planning_finished_has_new_traces || trajectories_size < 2) {
    current_trajectory_index = 0;
    switch_to_next_trace_flag = false;
    current_trajectory =
        partitioned_trajectories->at(current_trajectory_index).first;
    auto* open_space_status_ptr = injector_->planning_context()
                                      ->mutable_planning_status()
                                      ->mutable_open_space();
    open_space_status_ptr->set_position_init(true);
  } else {
    current_trajectory_index = last_current_trajectory_index_;
    const auto& current_gear =
        partitioned_trajectories->at(current_trajectory_index).second;
    current_trajectory =
        partitioned_trajectories->at(current_trajectory_index).first;
    if (current_trajectory.size() < 1U) {
      AERROR << "error, Partitioned_Result: current_trajectory is empty";
      return Status(ErrorCode::PLANNING_ERROR,
                    "Partitioned_Result: current_trajectory is empty");
    }
    switch_to_next_trace_flag = CalSwitchToNextTrace(
        current_trajectory, current_gear, injector_->vehicle_state());
  }
  AINFO << "Partitioned Switch: switch_to_next_trace_flag:"
        << switch_to_next_trace_flag;

  if (switch_to_next_trace_flag) {
    current_trajectory_index =
        (current_trajectory_index + 1 >= trajectories_size)
            ? trajectories_size - 1
            : current_trajectory_index + 1;
  }
  current_trajectory_point_index = current_trajectory.QueryNearestPoint(
      {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()});

  last_current_trajectory_index_ = current_trajectory_index;
  AINFO << "Partitioned Switch: Finished chosen_trajectory_index:"
        << current_trajectory_index
        << " trajectory_point_index:" << current_trajectory_point_index;

  if (current_trajectory_index >= partitioned_trajectories->size()) {
    AERROR << "error, Partitioned_Result: chosen_trajectory_index_error";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Partitioned_Result: chosen_trajectory_index_error");
  }
  if (partitioned_trajectories->at(current_trajectory_index).first.size() <
      1U) {
    AERROR << "error, Partitioned_Result: chosen_trajectory_is_empty";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Partitioned_Result: chosen_trajectory_is_empty");
  }

  auto* chosen_partitioned_trajectory =
      open_space_info_ptr->mutable_chosen_partitioned_trajectory();

  auto* mutable_trajectory =
      open_space_info_ptr->mutable_stitched_trajectory_result();
  double adjust_trajectory_start_time = Clock::NowInSeconds();

  NewAdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(),
                            current_trajectory_index,
                            current_trajectory_point_index, mutable_trajectory,
                            chosen_partitioned_trajectory);

  double adjust_trajectory_end_time = Clock::NowInSeconds();
  AINFO << "adjust_trajectory_use_time = "
        << adjust_trajectory_end_time - adjust_trajectory_start_time;

  double speed_limit_start_time = Clock::NowInSeconds();
  if (FLAGS_enable_use_qp_for_teb_speed && FLAGS_enable_teb_speed_limit &&
      canbus::Chassis::GEAR_REVERSE != chosen_partitioned_trajectory->second) {
    CalcuSpeedLimit(*chosen_partitioned_trajectory);
  }
  double speed_limit_end_time = Clock::NowInSeconds();

  AINFO << "speed_limit_use_time = "
        << speed_limit_end_time - speed_limit_start_time;
  double optimized_start_time = Clock::NowInSeconds();
  OptimizeTrajectory(open_space_info.partitioned_trajectories(), 0,
                     current_trajectory_index, mutable_trajectory,
                     chosen_partitioned_trajectory);
  double optimized_end_time = Clock::NowInSeconds();
  AINFO << "optimized_use_time = " << optimized_end_time - optimized_start_time;

  return Status::OK();
}

void TEBTrajectoryPartition::InterpolateTrajectory(
    const DiscretizedTrajectory& stitched_trajectory_result,
    DiscretizedTrajectory* interpolated_trajectory) {
  if (FLAGS_use_iterative_anchoring_smoother) {
    *interpolated_trajectory = stitched_trajectory_result;
    return;
  }
  interpolated_trajectory->clear();
  size_t interpolated_pieces_num =
      teb_trajectory_partition_config_.interpolated_pieces_num();
  CHECK_GT(stitched_trajectory_result.size(), 0U);
  CHECK_GT(interpolated_pieces_num, 0U);
  size_t trajectory_to_be_partitioned_intervals_num =
      stitched_trajectory_result.size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    double relative_time_interval =
        (stitched_trajectory_result.at(i + 1).relative_time() -
         stitched_trajectory_result[i].relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    interpolated_trajectory->emplace_back(stitched_trajectory_result[i]);
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          stitched_trajectory_result[i].relative_time() +
          (static_cast<double>(j) + 1.0) * relative_time_interval;
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              stitched_trajectory_result[i],
              stitched_trajectory_result.at(i + 1), relative_time));
    }
  }
  interpolated_trajectory->emplace_back(stitched_trajectory_result.back());
}

void TEBTrajectoryPartition::UpdateVehicleInfo() {
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  ego_theta_ = vehicle_state.heading();
  ego_x_ = vehicle_state.x();
  ego_y_ = vehicle_state.y();
  ego_v_ = vehicle_state.linear_velocity();
  Box2d box({ego_x_, ego_y_}, ego_theta_, ego_length_, ego_width_);
  ego_box_ = std::move(box);
  Vec2d ego_shift_vec{shift_distance_ * std::cos(ego_theta_),
                      shift_distance_ * std::sin(ego_theta_)};
  ego_box_.Shift(ego_shift_vec);
  vehicle_moving_direction_ =
      canbus::Chassis::GEAR_REVERSE == vehicle_state.gear()
          ? NormalizeAngle(ego_theta_ + M_PI)
          : ego_theta_;
}

bool TEBTrajectoryPartition::EncodeTrajectory(
    const DiscretizedTrajectory& trajectory, std::string* const encoding) {
  if (trajectory.empty()) {
    AERROR << "Fail to encode trajectory because it is empty";
    return false;
  }
  static constexpr double encoding_origin_x = 58700.0;
  static constexpr double encoding_origin_y = 4141000.0;
  const auto& init_path_point = trajectory.front().path_point();
  const auto& last_path_point = trajectory.back().path_point();

  const int init_point_x =
      static_cast<int>((init_path_point.x() - encoding_origin_x) * 1000.0);
  const int init_point_y =
      static_cast<int>((init_path_point.y() - encoding_origin_y) * 1000.0);
  const int init_point_heading =
      static_cast<int>(init_path_point.theta() * 10000.0);
  const int last_point_x =
      static_cast<int>((last_path_point.x() - encoding_origin_x) * 1000.0);
  const int last_point_y =
      static_cast<int>((last_path_point.y() - encoding_origin_y) * 1000.0);
  const int last_point_heading =
      static_cast<int>(last_path_point.theta() * 10000.0);

  *encoding = absl::StrCat(
      init_point_x, "_", init_point_y, "_", init_point_heading, "/",
      last_point_x, "_", last_point_y, "_", last_point_heading);
  return true;
}

bool TEBTrajectoryPartition::CheckTrajTraversed(
    const std::string& trajectory_encoding_to_check) {
  const auto& open_space_status =
      injector_->planning_context()->planning_status().open_space();
  const int index_history_size =
      open_space_status.partitioned_trajectories_index_history_size();

  if (index_history_size <= 1) {
    return false;
  }
  for (int i = 0; i < index_history_size - 1; i++) {
    const auto& index_history =
        open_space_status.partitioned_trajectories_index_history(i);
    if (index_history == trajectory_encoding_to_check) {
      return true;
    }
  }
  return false;
}

void TEBTrajectoryPartition::UpdateTrajHistory(
    const std::string& chosen_trajectory_encoding) {
  auto* open_space_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_open_space();

  const auto& trajectory_history =
      injector_->planning_context()
          ->planning_status()
          .open_space()
          .partitioned_trajectories_index_history();
  if (trajectory_history.empty()) {
    open_space_status->add_partitioned_trajectories_index_history(
        chosen_trajectory_encoding);
    return;
  }
  if (*(trajectory_history.rbegin()) == chosen_trajectory_encoding) {
    return;
  }
  open_space_status->add_partitioned_trajectories_index_history(
      chosen_trajectory_encoding);
}

void TEBTrajectoryPartition::PartitionTrajectory(
    const DiscretizedTrajectory& raw_trajectory,
    std::vector<TrajGearPair>* partitioned_trajectories) {
  CHECK_NOTNULL(partitioned_trajectories);

  size_t horizon = raw_trajectory.size();
  bool is_reverse_driving = frame_->open_space_info().is_reverse();

  partitioned_trajectories->clear();
  partitioned_trajectories->emplace_back();
  TrajGearPair* current_trajectory_gear = &(partitioned_trajectories->back());

  auto* trajectory = &(current_trajectory_gear->first);
  auto* gear = &(current_trajectory_gear->second);

  AINFO << "is_reverse_driving: " << is_reverse_driving << ", gear: " << *gear
        << ", vehicle_state()->x(): " << injector_->vehicle_state()->x()
        << " | vehicle_state()->y()" << injector_->vehicle_state()->y();

  // There is a possibility of misjudgment of D gear due to the possibility of
  // considering the same coordinate point between the front and rear points
  // Decide initial gear
  const auto& first_path_point = raw_trajectory.front().path_point();
  const auto& second_path_point = raw_trajectory[1].path_point();
  double heading_angle = first_path_point.theta();
  const Vec2d init_tracking_vector(
      second_path_point.x() - first_path_point.x(),
      second_path_point.y() - first_path_point.y());
  AINFO << "first_path_point.x(): " << first_path_point.x()
        << " | first_path_point.y(): " << first_path_point.y()
        << ", second_path_point.x(): " << second_path_point.x()
        << " | second_path_point.y(): " << second_path_point.y();
  double tracking_angle = init_tracking_vector.Angle();
  *gear =
      (std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
       (M_PI_2))
          ? (is_reverse_driving ? canbus::Chassis::GEAR_REVERSE
                                : canbus::Chassis::GEAR_DRIVE)
          : (is_reverse_driving ? canbus::Chassis::GEAR_DRIVE
                                : canbus::Chassis::GEAR_REVERSE);
  AINFO << "gear: " << *gear << ", heading_angle: " << heading_angle
        << ", tracking_angle: " << tracking_angle;

  // Set accumulated distance
  Vec2d last_pos_vec(first_path_point.x(), first_path_point.y());
  double distance_s = 0.0;
  bool is_trajectory_last_point = false;

  for (size_t i = 0; i < horizon - 1; ++i) {
    const TrajectoryPoint& trajectory_point = raw_trajectory[i];
    const TrajectoryPoint& next_trajectory_point = raw_trajectory.at(i + 1);
    heading_angle = trajectory_point.path_point().theta();

    const Vec2d tracking_vector(next_trajectory_point.path_point().x() -
                                    trajectory_point.path_point().x(),
                                next_trajectory_point.path_point().y() -
                                    trajectory_point.path_point().y());
    tracking_angle = tracking_vector.Angle();
    auto cur_gear = (std::abs(common::math::NormalizeAngle(
                         tracking_angle - heading_angle)) < (M_PI_2))
                        ? (is_reverse_driving ? canbus::Chassis::GEAR_REVERSE
                                              : canbus::Chassis::GEAR_DRIVE)
                        : (is_reverse_driving ? canbus::Chassis::GEAR_DRIVE
                                              : canbus::Chassis::GEAR_REVERSE);

    if (cur_gear != *gear) {
      is_trajectory_last_point = true;
      LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                          &last_pos_vec, &distance_s, trajectory);
      partitioned_trajectories->emplace_back();
      current_trajectory_gear = &(partitioned_trajectories->back());
      current_trajectory_gear->second = cur_gear;
      distance_s = 0.0;
      is_trajectory_last_point = false;
    }

    trajectory = &(current_trajectory_gear->first);
    gear = &(current_trajectory_gear->second);

    LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                        &last_pos_vec, &distance_s, trajectory);
  }
  is_trajectory_last_point = true;
  const TrajectoryPoint& last_trajectory_point = raw_trajectory.back();
  LoadTrajectoryPoint(last_trajectory_point, is_trajectory_last_point, *gear,
                      &last_pos_vec, &distance_s, trajectory);
}

void TEBTrajectoryPartition::LoadTrajectoryPoint(
    const TrajectoryPoint& trajectory_point,
    const bool is_trajectory_last_point,
    const canbus::Chassis::GearPosition& gear, Vec2d* last_pos_vec,
    double* distance_s, DiscretizedTrajectory* current_trajectory) {
  current_trajectory->emplace_back();
  TrajectoryPoint* point = &(current_trajectory->back());
  *point = trajectory_point;
  Vec2d cur_pos_vec(trajectory_point.path_point().x(),
                    trajectory_point.path_point().y());
  *distance_s += (canbus::Chassis::GEAR_REVERSE == gear ? -1.0 : 1.0) *
                 (cur_pos_vec.DistanceTo(*last_pos_vec));
  *last_pos_vec = cur_pos_vec;
  ADEBUG << "path_point().kappa() " << trajectory_point.path_point().kappa();
  point->mutable_path_point()->set_s(*distance_s);
}

bool TEBTrajectoryPartition::NewCheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    const size_t trajectories_index, size_t* current_trajectory_index,
    size_t* current_trajectory_point_index) {
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};

  size_t adc_index = trajectory.QueryNearestPoint(adc_position);

  const DiscretizedTrajectory* trajectory_ptr = &trajectory;
  common::math::Vec2d nearest_point = {
      (&(trajectory_ptr->at(adc_index)))->path_point().x(),
      (&(trajectory_ptr->at(adc_index)))->path_point().y()};

  double projection_dis = 0.0;
  if (adc_index + 1 < trajectory_ptr->size()) {
    const common::math::Vec2d next_nearest_point = {
        (&(trajectory_ptr->at(adc_index + 1)))->path_point().x(),
        (&(trajectory_ptr->at(adc_index + 1)))->path_point().y()};
    const Vec2d projection_point =
        ComputeProjection(nearest_point, next_nearest_point, adc_position);
    projection_dis = ComputeSideLonDistanceToProjection(
        nearest_point, next_nearest_point, projection_point);
  }
  projection_dis = (canbus::Chassis::GEAR_REVERSE == gear) ? -projection_dis
                                                           : projection_dis;

  const size_t trajectory_size = trajectory.size();
  AINFO << "NewCheckReachTrajectoryEnd_trajectory_size: " << trajectory_size;

  TrajectoryPoint start_point = (trajectory.at(adc_index));
  TrajectoryPoint end_point = (trajectory.at(trajectory_size - 1));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  double dist_to_end =
      std::fabs(end_point.path_point().s() -
                (start_point.path_point().s() + projection_dis));
  double dx = end_point.path_point().x() - injector_->vehicle_state()->x();
  double dy = end_point.path_point().y() - injector_->vehicle_state()->y();
  double dist_to_end2 = std::hypot(dx, dy);

  AINFO << "dist_to_end = " << dist_to_end << " gear " << gear << "adc_speed "
        << adc_speed << "dist_to_end2 " << dist_to_end2;
  if (1 == trajectories_size) {
    AINFO << "only one trajectory";
    *current_trajectory_index = 0;
    *current_trajectory_point_index = trajectory_size - 1;
    return true;
  }
  if (canbus::Chassis::GEAR_REVERSE == gear) {
    if ((dist_to_end > -switch_trace_thr_s_ &&
         std::fabs(adc_speed) < linear_velocity_threshold_on_ego_) &&
        dist_to_end2 < switch_trace_thr_dis_) {
      if (trajectories_index + 1 >= trajectories_size) {
        *current_trajectory_index = trajectories_size - 1;
        *current_trajectory_point_index = trajectory_size - 1;
      } else {
        *current_trajectory_index = trajectories_index + 1;
        *current_trajectory_point_index = 0;
      }
      return true;
    }
  } else {
    if ((dist_to_end < switch_trace_thr_s_ &&
         std::fabs(adc_speed) < linear_velocity_threshold_on_ego_) &&
        dist_to_end2 < switch_trace_thr_dis_) {
      if (trajectories_index + 1 >= trajectories_size) {
        *current_trajectory_index = trajectories_size - 1;
        *current_trajectory_point_index = trajectory_size - 1;
      } else {
        *current_trajectory_index = trajectories_index + 1;
        *current_trajectory_point_index = 0;
      }
      return true;
    }
  }
  return false;
}

bool TEBTrajectoryPartition::CheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    const size_t trajectories_index, size_t* current_trajectory_index,
    size_t* current_trajectory_point_index) {
  // Check if have reached endpoint of trajectory
  const TrajectoryPoint& trajectory_end_point = trajectory.back();
  const size_t trajectory_size = trajectory.size();
  const PathPoint& path_end_point = trajectory_end_point.path_point();
  const double path_end_point_x = path_end_point.x();
  const double path_end_point_y = path_end_point.y();
  const Vec2d tracking_vector(ego_x_ - path_end_point_x,
                              ego_y_ - path_end_point_y);
  const double path_end_point_theta = path_end_point.theta();
  const double included_angle = frame_->vehicle_state().heading();

  const double distance_to_trajs_end =
      std::sqrt((path_end_point_x - ego_x_) * (path_end_point_x - ego_x_) +
                (path_end_point_y - ego_y_) * (path_end_point_y - ego_y_));
  const double lateral_offset =
      std::abs(distance_to_trajs_end * std::sin(included_angle));
  const double longitudinal_offset =
      std::abs(distance_to_trajs_end * std::cos(included_angle));
  const double traj_end_point_moving_direction =
      canbus::Chassis::GEAR_REVERSE == gear
          ? NormalizeAngle(path_end_point_theta + M_PI)
          : path_end_point_theta;

  const double heading_search_to_trajs_end = std::abs(NormalizeAngle(
      traj_end_point_moving_direction - vehicle_moving_direction_));

  ADEBUG << "heading_search_to_trajs_end" << heading_search_to_trajs_end;
  AINFO << "lateral_offset " << lateral_offset << "longitudinal_offset "
        << longitudinal_offset;

  // If close to the end point, start on the next trajectory
  if (lateral_offset < lateral_offset_to_midpoint_ &&
      longitudinal_offset < longitudinal_offset_to_midpoint_ &&
      std::abs(ego_v_) < kEpsilon) {
    if (trajectories_index + 1 >= trajectories_size) {
      *current_trajectory_index = trajectories_size - 1;
      *current_trajectory_point_index = trajectory_size - 1;
    } else {
      *current_trajectory_index = trajectories_index + 1;
      *current_trajectory_point_index = 0;
    }
    ADEBUG << "Reach the end of a trajectory, switching to next one";
    return true;
  }

  ADEBUG << "Vehicle did not reach end of a trajectory with conditions for "
            "lateral distance_check: "
         << (lateral_offset < lateral_offset_to_midpoint_)
         << " and actual lateral distance: " << lateral_offset
         << "; longitudinal distance_check: "
         << (longitudinal_offset < longitudinal_offset_to_midpoint_)
         << " and actual longitudinal distance: " << longitudinal_offset
         << "; heading_check: "
         << (heading_search_to_trajs_end < heading_offset_to_midpoint_)
         << " with actual heading: " << heading_search_to_trajs_end
         << "; velocity_check: "
         << (std::abs(ego_v_) < linear_velocity_threshold_on_ego_)
         << " with actual linear velocity: " << ego_v_ << "; iou_check: ";
  return false;
}

bool TEBTrajectoryPartition::UseFailSafeSearch(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const std::vector<std::string>& trajectories_encodings,
    size_t* current_trajectory_index, size_t* current_trajectory_point_index) {
  AERROR << "Trajectory partition fail, using failsafe search"
         << "distance_search_range_" << distance_search_range_;
  const size_t trajectories_size = partitioned_trajectories.size();
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      failsafe_closest_point_on_trajs;
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = partitioned_trajectories[i].first;
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0U);
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        failsafe_closest_point;

    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      const double distance = tracking_vector.Length();
      if (distance < distance_search_range_) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        failsafe_closest_point.emplace(j, iou_ratio);
      }
    }
    if (!failsafe_closest_point.empty()) {
      size_t closest_point_index = failsafe_closest_point.top().first;
      double max_iou_ratio = failsafe_closest_point.top().second;
      failsafe_closest_point_on_trajs.emplace(
          std::make_pair(i, closest_point_index), max_iou_ratio);
    }
  }
  if (failsafe_closest_point_on_trajs.empty()) {
    return false;
  } else {
    bool closest_and_not_repeated_traj_found = false;
    while (!failsafe_closest_point_on_trajs.empty()) {
      if (CheckTrajTraversed(
              trajectories_encodings[*current_trajectory_index])) {
        failsafe_closest_point_on_trajs.pop();
      } else {
        closest_and_not_repeated_traj_found = true;
        return true;
      }
    }
    if (!closest_and_not_repeated_traj_found) {
      return false;
    }

    return true;
  }
}

bool TEBTrajectoryPartition::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  const auto* last_frame = injector_->use_thread_in_play_street()
                               ? injector_->frame_teb_history()->Latest()
                               : injector_->frame_history()->Latest();
  const auto& last_gear_status =
      last_frame->open_space_info().gear_switch_states();
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  *(current_gear_status) = last_gear_status;

  if (flag_change_to_next || !current_gear_status->gear_shift_period_finished) {
    current_gear_status->gear_shift_period_finished = false;
    if (current_gear_status->gear_shift_period_started) {
      current_gear_status->gear_shift_start_time =
          Clock::Instance()->NowInSeconds();
      current_gear_status->gear_shift_position =
          partitioned_trajectories.at(current_trajectory_index).second;
      current_gear_status->gear_shift_period_started = false;
    }
    if (current_gear_status->gear_shift_period_time >
        teb_trajectory_partition_config_.gear_shift_period_duration()) {
      current_gear_status->gear_shift_period_finished = true;
      current_gear_status->gear_shift_period_started = true;
    } else {
      // send N gear to protect idle
      if (!last_frame->open_space_info().open_space_provider_success()) {
        current_gear_status->gear_shift_position =
            canbus::Chassis::GEAR_NEUTRAL;
      }
      GenerateGearShiftTrajectory(current_gear_status->gear_shift_position,
                                  gear_switch_idle_time_trajectory);
      current_gear_status->gear_shift_period_time =
          Clock::Instance()->NowInSeconds() -
          current_gear_status->gear_shift_start_time;
      return true;
    }
  }

  return true;
}

void TEBTrajectoryPartition::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t =
      teb_trajectory_partition_config_.gear_shift_max_t();
  const double gear_shift_unit_t =
      teb_trajectory_partition_config_.gear_shift_unit_t();
  for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(frame_->vehicle_state().kappa());
    point.set_relative_time(t);
    point.set_v(0.0);
    point.set_a(0.0);
    gear_switch_idle_time_trajectory->first.emplace_back(point);
  }
  ADEBUG << "gear_switch_idle_time_trajectory "
         << gear_switch_idle_time_trajectory->first.size();
  gear_switch_idle_time_trajectory->second = gear_position;
}

Eigen::VectorXd TEBTrajectoryPartition::Polyfit(
    const std::vector<PathPoint2D>& in_point, int n) {
  int size = in_point.size();
  Eigen::MatrixXd A(size, n + 1);
  Eigen::VectorXd Y(size);
  Eigen::VectorXd Result(n + 1);

  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < n + 1; ++j) {
      A(i, j) = pow(in_point[i].GetConstX(), j);
    }
    Y[i] = in_point[i].GetConstY();
  }

  Result = A.householderQr().solve(Y);

  AINFO << "Result: " << Result;
  return Result;
}

bool TEBTrajectoryPartition::PolyfitTrajectory(
    const bool is_reverse, const double step,
    DiscretizedTrajectory* polyfit_trajectory_ptr,
    DiscretizedTrajectory** current_trajectory_ptr) {
  if (!teb_trajectory_partition_config_.enable_teb_polyfit()) {
    return false;
  }
  DiscretizedTrajectory current_trajectory = **current_trajectory_ptr;

  if (current_trajectory.empty()) {
    AERROR << "Current trajectory is empty";
    return false;
  }

  polyfit_trajectory_ptr->clear();
  polyfit_trajectory_ptr->shrink_to_fit();
  std::vector<PathPoint2D> in_point_s_for_x;
  std::vector<PathPoint2D> in_point_s_for_y;
  if (current_trajectory.size() < kPolyfitCoefficient + 1) {
    return false;
  }
  double start_x = current_trajectory.front().path_point().x();
  double start_y = current_trajectory.front().path_point().y();
  double start_s = current_trajectory.front().path_point().s();
  double start_theta = current_trajectory.front().path_point().theta();

  for (size_t i = 0; i < current_trajectory.size(); ++i) {
    in_point_s_for_x.emplace_back(
        PathPoint2D((current_trajectory[i].path_point().s() - start_s),
                    (current_trajectory[i].path_point().x()) - start_x));
    in_point_s_for_y.emplace_back(
        PathPoint2D((current_trajectory[i].path_point().s() - start_s),
                    (current_trajectory[i].path_point().y()) - start_y));
  }

  Eigen::VectorXd result_sx = Polyfit(in_point_s_for_x, kPolyfitCoefficient);
  Eigen::VectorXd result_sy = Polyfit(in_point_s_for_y, kPolyfitCoefficient);

  double const_start_x = in_point_s_for_x[0].GetConstX();
  double const_end_x =
      in_point_s_for_x[in_point_s_for_x.size() - 1].GetConstX();
  bool gear_d = (const_start_x < const_end_x) ? true : false;
  double increment = gear_d ? step : -step;
  double temp_x = 0.0;
  double temp_y = 0.0;
  double s = 0.0;
  TrajectoryPoint polyfit_trajectory;
  PathPoint* path_point = polyfit_trajectory.mutable_path_point();
  for (double i = const_start_x;
       gear_d ? (i <= const_end_x) : (i >= const_end_x); i += increment) {
    temp_x = 0.0;
    temp_y = 0.0;
    for (int j = 0; j < kPolyfitCoefficient + 1; ++j) {
      temp_x += result_sx[j] * pow(i, j);
      temp_y += result_sy[j] * pow(i, j);
    }

    path_point->set_x(temp_x + start_x);
    path_point->set_y(temp_y + start_y);

    if (polyfit_trajectory_ptr->empty()) {
      path_point->set_theta(start_theta);
      path_point->set_s(s + start_s);
    } else {
      double dx = ((temp_x + start_x) -
                   polyfit_trajectory_ptr->back().path_point().x());
      double dy = ((temp_y + start_y) -
                   polyfit_trajectory_ptr->back().path_point().y());
      double theta_yaw = is_reverse ? NormalizeAngle(std::atan2(dy, dx) + M_PI)
                                    : std::atan2(dy, dx);
      path_point->set_theta(theta_yaw);
      s += increment;
      path_point->set_s(s + start_s);
    }
    polyfit_trajectory_ptr->emplace_back(polyfit_trajectory);
    if ((const_end_x - i) < std::fabs(increment)) {
      increment = gear_d ? kPolyfitEndStep : -kPolyfitEndStep;
    }
  }
  *current_trajectory_ptr = polyfit_trajectory_ptr;
  AINFO << "polyfit_trajectory_ptr->size() = "
        << polyfit_trajectory_ptr->size();
  return true;
}

void TEBTrajectoryPartition::ShrunkTrajectoryForReverse(
    common::TrajectoryPoint* start_point,
    TrajGearPair* current_partitioned_trajectory,
    DiscretizedTrajectory* trajectory) {
  bool is_get_start_point =
      GetStartPoint(current_partitioned_trajectory, start_point);
  AINFO << "is_get_start_point = " << is_get_start_point;
  double start_point_x = injector_->vehicle_state()->x();
  double start_point_y = injector_->vehicle_state()->y();
  if (0) {
    start_point_x = start_point->path_point().x();
    start_point_y = start_point->path_point().y();
  }
  AINFO << "before cut reverse trajectory point size = " << trajectory->size();
  int first_index = -1;
  AINFO << "reverse first_index = " << first_index;

  const common::math::Vec2d adc_position = {start_point_x, start_point_y};

  size_t adc_index = trajectory->QueryNearestPoint(adc_position);
  TrajectoryPoint nearst_point = (trajectory->at(adc_index));
  double adc_nearest_point_s = nearst_point.path_point().s();
  double deviation_distance =
      std::sqrt((nearst_point.path_point().x() - start_point_x) *
                    (nearst_point.path_point().x() - start_point_x) +
                (nearst_point.path_point().y() - start_point_y) *
                    (nearst_point.path_point().y() - start_point_y));
  ADEBUG << "deviation_distance = " << deviation_distance;
  // TODO(zongxingguo): if has large deviation distance ,we need stop.
  if (trajectory->size() - 1 == adc_index) {
    // TODO(zongxingguo): sliding 0.1m.
    trajectory->erase(trajectory->begin(),
                      trajectory->begin() + trajectory->size() - 3);
    AINFO << "reaching last point and shrunk to 2 point, trajectory.size = "
          << trajectory->size();
    return;
  }
  for (size_t i = 0; i < trajectory->size(); ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    if (trajectory_point->path_point().s() > adc_nearest_point_s) {
      first_index = i;
      continue;
    }
  }
  // delete passed trajectories.
  if (first_index >= 0) {
    // after s<0.0 point as first.
    first_index++;
    if (first_index >= static_cast<int>(trajectory->size() - 1)) {
      return;
    }
    trajectory->erase(trajectory->begin(), trajectory->begin() + first_index);
  }

  // interpolate the current position as the starting point.
  double dx_1 = start_point_x - trajectory->front().path_point().x(),
         dy_1 = start_point_y - trajectory->front().path_point().y(),
         dx_2 = start_point_x - trajectory->at(1).path_point().x(),
         dy_2 = start_point_y - trajectory->at(1).path_point().y();

  double vehicle_to_first_point_distance = std::sqrt(dx_1 * dx_1 + dy_1 * dy_1);
  double vehicle_to_second_point_distance =
      std::sqrt(dx_2 * dx_2 + dy_2 * dy_2);
  double dx_3 = start_point_x - trajectory->back().path_point().x();
  double dy_3 = start_point_y - trajectory->back().path_point().y();
  double start_point_to_back_point_distance =
      std::sqrt(dx_3 * dx_3 + dy_3 * dy_3);
  if (start_point_to_back_point_distance <
      vehicle_to_first_point_distance + kEpsilon) {
    AINFO << "no change start point ,reach last point.";
    return;
  }
  auto point = trajectory->front().mutable_path_point();
  point->set_x(start_point_x);
  point->set_y(start_point_y);
  point->set_s(trajectory->at(1).path_point().s() +
               vehicle_to_second_point_distance);
}

void TEBTrajectoryPartition::ShrunkTrajectory(
    common::TrajectoryPoint* start_point,
    TrajGearPair* current_partitioned_trajectory,
    DiscretizedTrajectory* trajectory) {
  bool is_get_start_point =
      GetStartPoint(current_partitioned_trajectory, start_point);

  double start_point_x = injector_->vehicle_state()->x();
  double start_point_y = injector_->vehicle_state()->y();
  ADEBUG << "is_get_start_point" << is_get_start_point;
  if (0) {
    start_point_x = start_point->path_point().x();
    start_point_y = start_point->path_point().y();
  }
  AINFO << "before cut trajectory point size = " << trajectory->size();
  int first_index = -1;
  const common::math::Vec2d start_point_position = {start_point_x,
                                                    start_point_y};
  size_t adc_index = trajectory->QueryNearestPoint(start_point_position);
  TrajectoryPoint neast_point = (trajectory->at(adc_index));
  double adc_nearest_point_s = neast_point.path_point().s();
  double deviation_distance =
      std::sqrt((neast_point.path_point().x() - start_point_x) *
                    (neast_point.path_point().x() - start_point_x) +
                (neast_point.path_point().y() - start_point_y) *
                    (neast_point.path_point().y() - start_point_y));
  ADEBUG << "deviation_distance = " << deviation_distance;
  if ((trajectory->size() - 1 == adc_index) && trajectory->size() >= 3) {
    trajectory->erase(trajectory->begin(),
                      trajectory->begin() + trajectory->size() - 3);
    double dx_to_end = start_point_x - trajectory->back().path_point().x();
    double dy_to_end = start_point_y - trajectory->back().path_point().y();
    double vehicle_to_end_point_distance =
        std::sqrt(dx_to_end * dx_to_end + dy_to_end * dy_to_end);
    if (vehicle_to_end_point_distance <= kMinDistanceToDestination) {
      auto point = trajectory->front().mutable_path_point();
      point->set_x(start_point_x);
      point->set_y(start_point_y);
      point->set_s(trajectory->at(1).path_point().s() -
                   vehicle_to_end_point_distance);
    }
    return;
  }

  for (size_t i = 0; i < trajectory->size(); ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    // TODO(zongxingguo): need to use start point or vehicle_point?
    // make sure current index point's =0.0.
    if (trajectory_point->path_point().s() < adc_nearest_point_s) {
      first_index = i;
      continue;
    }
  }
  // delete passed trajectories.
  if (first_index >= 0) {
    // after s<0.0 point as first.
    first_index++;
    AINFO << "cut trajectory = " << first_index;
    if (first_index >= static_cast<int>(trajectory->size() - 1)) {
      AINFO << "reach last point";
      return;
    }
    trajectory->erase(trajectory->begin(), trajectory->begin() + first_index);
  }
  AINFO << "after cut tarjectory point size = " << trajectory->size();
  double dx_1 = start_point_x - trajectory->front().path_point().x(),
         dy_1 = start_point_y - trajectory->front().path_point().y(),
         dx_2 = start_point_x - trajectory->at(1).path_point().x(),
         dy_2 = start_point_y - trajectory->at(1).path_point().y();
  double vehicle_to_first_point_distance = std::sqrt(dx_1 * dx_1 + dy_1 * dy_1),
         vehicle_to_second_point_distance =
             std::sqrt(dx_2 * dx_2 + dy_2 * dy_2),
         dx_3 = start_point_x - trajectory->back().path_point().x(),
         dy_3 = start_point_y - trajectory->back().path_point().y(),
         start_point_to_back_point_distance =
             std::sqrt(dx_3 * dx_3 + dy_3 * dy_3);

  if (start_point_to_back_point_distance <
      vehicle_to_first_point_distance + kEpsilon) {
    AINFO << "reach last point.";
    return;
  }
  auto point = trajectory->front().mutable_path_point();
  point->set_x(start_point_x);
  point->set_y(start_point_y);
  point->set_s(trajectory->at(1).path_point().s() -
               vehicle_to_second_point_distance);
}

double TEBTrajectoryPartition::GetTotalTimeForNoComfortStop(
    double total_length, double adc_speed, double target_v,
    double* begin_break_s, std::vector<std::pair<double, double>>* ref_s_list) {
  double need_decel = (adc_speed * adc_speed) / (total_length * 2.0);
  double total_time = adc_speed / need_decel;
  *begin_break_s = total_length;
  double delta_t = kOptimizedDeltaT;
  AINFO << "GetTotalTimeForNoComfortStop";
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    ref_s.second =
        (adc_speed * adc_speed - (adc_speed - need_decel * ref_s.first) *
                                     (adc_speed - need_decel * ref_s.first)) /
        (2 * need_decel);
    ref_v.second = adc_speed - need_decel * ref_s.first;
    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double TEBTrajectoryPartition::GetTotalTimeForUniformSpeedComfortStop(
    double total_length, double adc_speed, double target_v,
    double comfort_decel, double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  CHECK_NE(target_v, 0.0);
  CHECK_NE(comfort_decel, 0.0);
  AINFO << "GetTotalTimeForUniformSpeedComfortStop";
  double t_cruise = (total_length - comfort_decel_distance) / target_v;
  double t_rampdown = (adc_speed - target_v) / comfort_decel;
  double t_dec = target_v / comfort_decel;
  double total_time = t_cruise + t_rampdown + t_dec;
  *begin_break_s = total_length - target_v * target_v / (2 * comfort_decel);
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  double start_break_t = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_rampdown) {
      ref_s.first = i * delta_t;
      ref_s.second = (adc_speed * adc_speed -
                      (adc_speed - comfort_decel * ref_s.first) *
                          (adc_speed - comfort_decel * ref_s.first)) /
                     (2 * comfort_decel);
      current_s = ref_s.second;
      ref_v.second = adc_speed - comfort_decel * ref_s.first;
      start_break_t = ref_s.first;
    } else if (ref_s.first > t_rampdown &&
               ref_s.first <= t_cruise + t_rampdown) {
      ref_s.first = i * delta_t;
      current_s = current_s + delta_t * target_v;
      ref_s.second = current_s;
      start_break_t = ref_s.first;
      ref_v.second = target_v;
    } else {
      ref_s.first = i * delta_t;
      double diff_t = ref_s.first - start_break_t;
      ref_s.second =
          current_s + target_v * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = target_v - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double TEBTrajectoryPartition::GetTotalTimeForACCAndUniformSpeedAndComfortStop(
    double s_rest, double t_rampup, double total_length, double adc_speed,
    double target_v, double comfort_decel, double comfort_acc,
    double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  AINFO << "GetTotalTimeForACCAndUniformSpeedAndComfortStop";
  double t_cruise = s_rest / target_v;
  double t_dec = target_v / comfort_decel;
  *begin_break_s = total_length - target_v * target_v / (2 * comfort_decel);
  double total_time = t_rampup + t_cruise + t_dec;
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  double start_break_t = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_rampup) {
      ref_s.first = i * delta_t;
      ref_s.second = ((adc_speed + comfort_acc * ref_s.first) *
                          (adc_speed + comfort_acc * ref_s.first) -
                      adc_speed * adc_speed) /
                     (2 * comfort_acc);
      current_s = ref_s.second;
      ref_v.second = adc_speed + comfort_acc * ref_v.first;
      start_break_t = ref_s.first;
    } else if (ref_s.first > t_rampup && ref_s.first <= t_cruise + t_rampup) {
      ref_s.first = i * delta_t;
      current_s = current_s + delta_t * target_v;
      ref_s.second = current_s;
      start_break_t = ref_s.first;
      ref_v.second = target_v;
    } else {
      ref_s.first = i * delta_t;
      double diff_t = ref_s.first - start_break_t;
      ref_s.second =
          current_s + target_v * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = target_v - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double TEBTrajectoryPartition::GetTotalTimeForACCAndComfortStop(
    double s_rest, double t_rampup, double total_length, double adc_speed,
    double target_v, double comfort_decel, double comfort_acc,
    double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  AINFO << "GetTotalTimeForACCAndComfortStop";
  double s_rampup_rampdown = total_length - comfort_decel_distance;
  double acc_use_s = s_rampup_rampdown * 0.5;
  double v_max =
      std::sqrt(adc_speed * adc_speed + 2.0 * comfort_acc * acc_use_s);
  // AINFO << "v_max = " << v_max;
  // AINFO << "adc_speed = " << adc_speed;
  double t_acc = (v_max - adc_speed) / comfort_acc;
  double t_dec = v_max / comfort_decel;
  // AINFO << "t_acc = " << t_acc;
  // AINFO << "t_dec = " << t_dec;
  *begin_break_s = total_length - v_max * v_max / (2 * comfort_decel);

  double total_time = t_acc + t_dec;
  // AINFO << "total_time = " << total_time;
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_acc) {
      ref_s.second = ((adc_speed + comfort_acc * ref_s.first) *
                          (adc_speed + comfort_acc * ref_s.first) -
                      adc_speed * adc_speed) /
                     (2 * comfort_acc);
      current_s = ref_s.second;
      ref_v.second = adc_speed + comfort_acc * ref_v.first;
    } else {
      double diff_t = ref_s.first - t_acc;
      ref_s.second =
          current_s + v_max * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = v_max - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  // TODO(zongxingguo): else also add last point..
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double TEBTrajectoryPartition::GetTotalTimeForReverse(
    double total_length, double adc_speed, double target_v,
    double* begin_break_s, std::vector<std::pair<double, double>>* ref_s_list,
    std::vector<std::pair<double, double>>* ref_v_list,
    bool* is_get_total_time) {
  double total_time = kOptimizedTotalTime;
  double comfort_decel = kComfortDecel;
  double comfort_acc = kComfortAcc;
  double comfort_decel_distance = adc_speed * adc_speed / (comfort_decel * 2.0);
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  if (ScenarioConfig::UTURN_TEB == scenario_type) {
    comfort_decel = kUturnComfortDecel;
    comfort_acc = kUturnComfortAcc;
  }
  if (std::fabs(adc_speed) < kStoppedSpeed && total_length < kStoppedLength) {
    *is_get_total_time = false;
    return total_time;
  }
  *is_get_total_time = true;

  // breaking down now with small comfort decel.
  if (comfort_decel_distance > total_length) {
    return GetTotalTimeForNoComfortStop(total_length, adc_speed, target_v,
                                        begin_break_s, ref_s_list);
  }
  if (adc_speed > target_v) {
    // comfort decel to target speed ,and uniform speed ,and comfort stop.
    return GetTotalTimeForUniformSpeedComfortStop(
        total_length, adc_speed, target_v, comfort_decel,
        comfort_decel_distance, begin_break_s, ref_s_list);
  } else {
    CHECK_NE(comfort_acc, 0.0);
    CHECK_NE(target_v, 0.0);
    CHECK_NE(comfort_decel, 0.0);
    double t_rampup = (target_v - adc_speed) / comfort_acc;
    double t_rampdown = (target_v - adc_speed) / comfort_decel;
    double s_ramp = (adc_speed + target_v) * (t_rampup + t_rampdown) * 0.5;

    double s_rest = total_length - s_ramp - comfort_decel_distance;
    if (s_rest > 0) {
      // comfort acc to target speed ,and uniform speed ,and comfort stop.
      return GetTotalTimeForACCAndUniformSpeedAndComfortStop(
          s_rest, t_rampup, total_length, adc_speed, target_v, comfort_decel,
          comfort_acc, comfort_decel_distance, begin_break_s, ref_s_list);
    } else {
      // acc to someone speed ,and comfort stop.
      return GetTotalTimeForACCAndComfortStop(
          s_rest, t_rampup, total_length, adc_speed, target_v, comfort_decel,
          comfort_acc, comfort_decel_distance, begin_break_s, ref_s_list);
    }
  }

  AINFO << "total_time = " << total_time;
  return total_time;
}

double TEBTrajectoryPartition::GetTotalTime(
    double total_length, double adc_speed, double target_v,
    double* begin_break_s, std::vector<std::pair<double, double>>* ref_s_list,
    bool* is_get_total_time) {
  double total_time = kOptimizedTotalTime;
  double comfort_decel = kComfortDecel;
  double comfort_acc = kComfortAcc;
  double comfort_decel_distance = adc_speed * adc_speed / (comfort_decel * 2.0);
  if (total_length < kBreakLength) {
    return total_time;
  }
  if (std::fabs(adc_speed) < kStoppedSpeed && total_length < kStoppedLength) {
    AINFO << "adc_speed = " << adc_speed;
    AINFO << "total_length = " << total_length;
    AINFO << "gear is rear or stop now";
    *is_get_total_time = false;
    return total_time;
  }
  *is_get_total_time = true;
  AINFO << "comfort_decel_distance = " << comfort_decel_distance;
  AINFO << "total_length = " << total_length;

  // breaking down now with small comfort decel.
  if (comfort_decel_distance > total_length) {
    return GetTotalTimeForNoComfortStop(total_length, adc_speed, target_v,
                                        begin_break_s, ref_s_list);
  }

  if (adc_speed > target_v) {
    // comfort decel to target speed ,and uniform speed ,and comfort stop.
    return GetTotalTimeForUniformSpeedComfortStop(
        total_length, adc_speed, target_v, comfort_decel,
        comfort_decel_distance, begin_break_s, ref_s_list);
  } else {
    CHECK_NE(comfort_acc, 0.0);
    CHECK_NE(target_v, 0.0);
    CHECK_NE(comfort_decel, 0.0);
    double t_rampup = (target_v - adc_speed) / comfort_acc;
    double t_rampdown = (target_v - adc_speed) / comfort_decel;
    double s_ramp = (adc_speed + target_v) * (t_rampup + t_rampdown) * 0.5;

    double s_rest = total_length - s_ramp - comfort_decel_distance;
    if (s_rest > 0) {
      // comfort acc to target speed ,and uniform speed ,and comfort stop.
      return GetTotalTimeForACCAndUniformSpeedAndComfortStop(
          s_rest, t_rampup, total_length, adc_speed, target_v, comfort_decel,
          comfort_acc, comfort_decel_distance, begin_break_s, ref_s_list);
    } else {
      // acc to someone speed ,and comfort stop.
      return GetTotalTimeForACCAndComfortStop(
          s_rest, t_rampup, total_length, adc_speed, target_v, comfort_decel,
          comfort_acc, comfort_decel_distance, begin_break_s, ref_s_list);
    }
  }
  AINFO << "total_time = " << total_time;
  return total_time;
}

void TEBTrajectoryPartition::OptimizeReproscessForReverse(
    double delta_t, const std::vector<std::pair<double, double>>& ref_s_list,
    int num_of_knots, DiscretizedTrajectory* trajectory) {
  std::vector<double> s_vector;
  std::vector<double> ds_vector;
  for (int i = 0; i < num_of_knots; ++i) {
    s_vector.emplace_back(ref_s_list[i].second);
    ds_vector.emplace_back(ref_v_list_[i].second);
  }
  std::vector<TrajectoryPoint> backup_points;
  for (int i = 0; i < num_of_knots; ++i) {
    if (!(s_vector[i] <= trajectory->front().path_point().s())) {
      continue;
    }
    if (!(s_vector[i] >= trajectory->back().path_point().s())) {
      break;
    }
    auto comp = [](const TrajectoryPoint point, const double s) {
      return point.path_point().s() > s;
    };
    auto it_lower = std::lower_bound(trajectory->begin(), trajectory->end(),
                                     s_vector[i], comp);
    const auto& index = std::distance(trajectory->begin(), it_lower);

    double v = ds_vector[i];
    auto trajectory_point = trajectory->at(index);
    PathPoint* path_point = trajectory_point.mutable_path_point();
    // except front point
    if (index > 0) {
      const auto& start_trajectory_point = trajectory->at(index - 1);
      const auto& end_trajectory_point = trajectory->at(index);
      const auto& start_path_point = start_trajectory_point.path_point();
      const auto& end_path_point = end_trajectory_point.path_point();
      const double dist = std::abs(end_path_point.s() - start_path_point.s());

      double dx = (end_path_point.x() - start_path_point.x()) / dist,
             dy = (end_path_point.y() - start_path_point.y()) / dist,
             dtheta = (NormalizeAngle(end_path_point.theta() -
                                      start_path_point.theta())) /
                      dist,
             dkappa =
                 (end_path_point.kappa() - start_path_point.kappa()) / dist,
             ddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             dddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             ds = (end_path_point.s() - start_path_point.s()) / dist,
             dsteer = (end_trajectory_point.steer() -
                       start_trajectory_point.steer()) /
                      dist;

      double d = start_path_point.s() - s_vector[i];
      if (d < 0.0) {
        continue;
      }
      path_point->set_x(start_path_point.x() + dx * d);
      path_point->set_y(start_path_point.y() + dy * d);
      ADEBUG << dtheta << dkappa << ddkappa << dddkappa << dsteer;
      path_point->set_theta(
          NormalizeAngle(start_path_point.theta() + dtheta * d));

      path_point->set_kappa(start_path_point.kappa() + dkappa * d);
      path_point->set_dkappa(start_path_point.dkappa() + ddkappa * d);
      path_point->set_ddkappa(start_path_point.ddkappa() + dddkappa * d);
      path_point->set_s(start_path_point.s() + ds * d);
      trajectory_point.set_steer(start_trajectory_point.steer() + dsteer * d);
    }
    trajectory_point.set_v(v);
    trajectory_point.set_a(0.0);
    trajectory_point.set_relative_time(i * delta_t);

    backup_points.emplace_back(trajectory_point);
    openspace_common_->CalTrajS(&backup_points, true);
  }
  if (backup_points.size() > kOneSizePoint) {
    trajectory->clear();

  } else {
    GetStopBackupPoints(&backup_points);
  }
  for (size_t i = 0; i < backup_points.size(); ++i) {
    trajectory->emplace_back(backup_points.at(i));
  }
}

void TEBTrajectoryPartition::OptimizeReproscess(
    double delta_t, PiecewiseJerkSpeedProblem* piecewise_jerk_problem,
    int num_of_knots, DiscretizedTrajectory* trajectory) {
  const std::vector<double>& s_vector_backup = piecewise_jerk_problem->opt_x();
  const std::vector<double>& ds_vector = piecewise_jerk_problem->opt_dx();
  const std::vector<double>& dds_vector = piecewise_jerk_problem->opt_ddx();
  std::vector<double> s_vector;
  // AINFO << "trajectory_point.size = " << trajectory->size();

  for (int i = 0; i < num_of_knots; ++i) {
    s_vector.emplace_back(s_vector_backup[i] +
                       trajectory->front().path_point().s());
    // AINFO << "For t[" << i * delta_t << "], s = " << s_vector[i]
    //       << ", v = " << ds_vector[i] << ", a = " << dds_vector[i];
  }
  std::vector<TrajectoryPoint> backup_points;
  TrajectoryPoint back_point = trajectory->back();
  for (int i = 0; i < num_of_knots; ++i) {
    // AINFO << "For t[" << i * delta_t << "], s = " << s_vector[i]
    //       << ", v = " << ds_vector[i] << ", a = " << dds_vector[i];
    // s_vector[i] must small than trajectory->back().path_point().s()
    if (!(s_vector[i] >= trajectory->front().path_point().s())) {
      // AINFO << "CONTINUE";
      continue;
    }
    if (!(s_vector[i] <= trajectory->back().path_point().s())) {
      // AINFO << "break";
      break;
    }
    auto comp = [](const TrajectoryPoint point, const double s) {
      return point.path_point().s() < s;
    };
    auto it_lower = std::lower_bound(trajectory->begin(), trajectory->end(),
                                     s_vector[i], comp);
    const auto& index = std::distance(trajectory->begin(), it_lower);
    // AINFO << "index = " << index;

    double v = ds_vector[i];
    double a = dds_vector[i];
    auto trajectory_point = trajectory->at(index);
    PathPoint* path_point = trajectory_point.mutable_path_point();
    // except front point
    if (index > 0) {
      const auto& start_trajectory_point = trajectory->at(index - 1);
      const auto& end_trajectory_point = trajectory->at(index);
      const auto& start_path_point = start_trajectory_point.path_point();
      const auto& end_path_point = end_trajectory_point.path_point();
      const double dist = std::abs(end_path_point.s() - start_path_point.s());

      double dx = (end_path_point.x() - start_path_point.x()) / dist,
             dy = (end_path_point.y() - start_path_point.y()) / dist,
             dtheta = (NormalizeAngle(end_path_point.theta() -
                                      start_path_point.theta())) /
                      dist,
             dkappa =
                 (end_path_point.kappa() - start_path_point.kappa()) / dist,
             ddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             dddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             ds = (end_path_point.s() - start_path_point.s()) / dist,
             dsteer = (end_trajectory_point.steer() -
                       start_trajectory_point.steer()) /
                      dist;

      double d = s_vector[i] - start_path_point.s();
      if (d < 0.0) {
        continue;
      }
      path_point->set_x(start_path_point.x() + dx * d);
      path_point->set_y(start_path_point.y() + dy * d);
      path_point->set_theta(
          NormalizeAngle(start_path_point.theta() + dtheta * d));

      path_point->set_kappa(start_path_point.kappa() + dkappa * d);
      path_point->set_dkappa(start_path_point.dkappa() + ddkappa * d);
      path_point->set_ddkappa(start_path_point.ddkappa() + dddkappa * d);
      path_point->set_s(start_path_point.s() + ds * d);
      trajectory_point.set_steer(start_trajectory_point.steer() + dsteer * d);
    }
    // AINFO << "backup v = " << v << "   a = " << a;
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(i * delta_t);

    backup_points.emplace_back(trajectory_point);
    openspace_common_->CalTrajS(&backup_points, false);
  }
  // AINFO << "backup_points.size() = " << backup_points.size();
  if (backup_points.size() > kOneSizePoint) {
    trajectory->clear();
  } else {
    GetStopBackupPoints(&backup_points);
  }
  for (size_t i = 0; i < backup_points.size(); ++i) {
    trajectory->emplace_back(backup_points.at(i));
  }
  AddLastPoint(back_point, trajectory);
}
void TEBTrajectoryPartition::AddLastPoint(const TrajectoryPoint& back_point,
                                          DiscretizedTrajectory* trajectory) {
  if (trajectory->back().path_point().s() -
          trajectory->front().path_point().s() >
      kMinTrajectoryLength) {
    auto trajectory_point = trajectory->back();
    PathPoint* path_point = trajectory_point.mutable_path_point();
    trajectory_point.set_v(0.0);
    trajectory_point.set_a(0.0);
    path_point->set_x(back_point.path_point().x());
    path_point->set_y(back_point.path_point().y());
  }
}

void TEBTrajectoryPartition::GetStopBackupPoints(
    std::vector<TrajectoryPoint>* backup_points) {
  AERROR << "backup_points is empty";
  double delta_t = kOptimizedDeltaT;
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  for (size_t i = 0; i < kMinSizePoint; ++i) {
    TrajectoryPoint trajectory_point;
    trajectory_point.set_v(0.0);
    trajectory_point.set_a(kMinDecel);
    trajectory_point.set_relative_time(i * delta_t);
    PathPoint* path_point = trajectory_point.mutable_path_point();
    path_point->set_x(vehicle_state.x());
    path_point->set_y(vehicle_state.y());
    path_point->set_theta(vehicle_state.heading());
    path_point->set_kappa(0.0);
    path_point->set_dkappa(0.0);
    path_point->set_ddkappa(0.0);
    path_point->set_s(0.0);
    trajectory_point.set_steer(0.0);
    backup_points->emplace_back(trajectory_point);
  }
}

void TEBTrajectoryPartition::SetProblemStatus(
    const int num_of_knots, const double adc_speed, const double total_length,
    PiecewiseJerkSpeedProblem* piecewise_jerk_problem) {
  piecewise_jerk_problem->set_weight_ddx(1000.0);
  piecewise_jerk_problem->set_weight_dddx(3000.0);

  piecewise_jerk_problem->set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                         FLAGS_longitudinal_jerk_upper_bound);

  piecewise_jerk_problem->set_dx_ref(100.0, 2.0);

  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, total_length});
  std::vector<std::pair<double, double>> dx_bounds(
      num_of_knots, {0.0, std::fmax(2.0, adc_speed)});

  double min_decel = kMinDecel;
  double max_acc = kMaxAcc;
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {min_decel, max_acc});
  // end_state
  x_bounds[num_of_knots - 1] = std::make_pair(total_length, total_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  piecewise_jerk_problem->set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem->set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem->set_ddx_bounds(std::move(ddx_bounds));
}

void TEBTrajectoryPartition::UpdateTrajectoryForReverse(
    double stop_s, DiscretizedTrajectory* trajectory) {
  AINFO << "before cut trajectory point size = " << trajectory->size();
  int first_index = -1;
  for (size_t i = 0; i < trajectory->size(); ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    AINFO << "trajectory_point->path_point().s() = "
          << trajectory_point->path_point().s();
    if (trajectory_point->path_point().s() <= stop_s) {
      int index = static_cast<int>(i) - 1;
      AINFO << "index = " << index;
      first_index = index < 0 ? 0 : index;
      break;
    }
  }
  // delete passed trajectories.
  if (first_index >= 0) {
    if (first_index >= static_cast<int>(trajectory->size() - 1)) {
      return;
    }
    if (first_index > 1) {
      trajectory->erase(trajectory->begin() + first_index, trajectory->end());
    } else {
      AERROR << "creat stop trajectory point for collision";
      CreateStopTrajectory(trajectory);
    }
  }

  if (stop_s > trajectory->front().path_point().s()) {
    AERROR << "generate stop trajectory point for collision";
    CreateStopTrajectory(trajectory);
  }

  AINFO << "after cut tarjectory point size = " << trajectory->size();
}

void TEBTrajectoryPartition::UpdateTrajectory(
    double stop_s, DiscretizedTrajectory* trajectory) {
  AINFO << "before cut trajectory point size = " << trajectory->size();
  int first_index = -1;
  for (size_t i = 0; i < trajectory->size(); ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    AINFO << "trajectory_point->path_point().s() = "
          << trajectory_point->path_point().s();
    if (trajectory_point->path_point().s() >= stop_s) {
      int index = static_cast<int>(i) - 1;
      first_index = index < 0 ? 0 : index;
      break;
    }
  }
  AINFO << "first_index = " << first_index;
  // delete passed trajectories.
  if (first_index >= 0) {
    if (first_index >= static_cast<int>(trajectory->size() - 1)) {
      return;
    }
    if (first_index > 1) {
      trajectory->erase(trajectory->begin() + first_index, trajectory->end());
      AINFO << "cut trajectory = " << first_index;
    } else {
      AINFO << "create stop trajectory for init point collision";
      is_stop_trajectory_ = true;
      CreateStopTrajectory(trajectory);
    }
  }
  AINFO << "after cut tarjectory point size = " << trajectory->size();
}

void TEBTrajectoryPartition::ConvertCostmap() {
  if (!FLAGS_enable_use_costmap) {
    return;
  }
  const auto& costmap_obstacles = frame_->static_area_polygon();
  costmap_polygons_.clear();
  common::math::Polygon2d perception_polygon;

  for (const auto& obstacle : costmap_obstacles) {
    std::vector<Vec2d> polygon_points;
    for (auto pt : obstacle.second) {
      polygon_points.emplace_back(Vec2d(pt.x(), pt.y()));
    }
    if (!common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon)) {
      AERROR << "waring : ComputeConvexHull failded, continue";
      continue;
    }
    costmap_polygons_.emplace_back(perception_polygon);
  }
}

bool TEBTrajectoryPartition::CheckObstacleCollision(
    const double relative_s, const common::math::Box2d& ego_box,
    const std::vector<const Obstacle*>& obstacles) {
  const auto& adc_polygon = Polygon2d(ego_box);
  for (const auto* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type) {
      const auto& obstacle_polygon = obstacle->PerceptionPolygon();
      if (obstacle_polygon.HasOverlap(adc_polygon)) {
        AINFO << ": ["
              << "]"
              << "relative_s " << relative_s << "obstacle id " << obstacle->Id()
              << "obs_type " << obs_type;
        return true;
      }
    } else {
      const auto& box = obstacle->PerceptionBoundingBox();
      if (ego_box.HasOverlap(box)) {
        AINFO << ": ["
              << "]"
              << "relative_s " << relative_s << "obstacle id " << obstacle->Id()
              << "obs_type " << obs_type;
        return true;
      }
    }
    if (!obstacle->IsStatic()) {
      // TODO(zongxingguo): is obs in side for face adc ,no stop.
      double relative_t = 0.0;
      double check_time = perception::PerceptionObstacle::PEDESTRIAN == obs_type
                              ? kPedCheckTime
                              : FLAGS_teb_obs_prediction_time;
      while (relative_t < check_time) {
        const auto& point = obstacle->GetPointAtTime(relative_t);
        if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
            perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
            FLAGS_enable_openspace_use_polygon_plan) {
          const Polygon2d& obs_polygon = obstacle->GetPolygon(point);
          if (obs_polygon.HasOverlap(adc_polygon)) {
            AINFO << ": ["
                  << "]"
                  << "relative_s " << relative_s << "obstacle id "
                  << obstacle->Id() << "obstacle->speed() " << obstacle->speed()
                  << "obs_type " << obs_type;
            AINFO << "dynamic collision1";
            return true;
          }
        } else {
          const auto& obs_box = obstacle->GetBoundingBox(point);
          if (ego_box.HasOverlap(obs_box)) {
            AINFO << ": ["
                  << "]"
                  << "relative_s " << relative_s << "obstacle id "
                  << obstacle->Id() << "obstacle->speed() " << obstacle->speed()
                  << "obs_type " << obs_type;
            AINFO << "dynamic collision2";
            return true;
          }
        }
        relative_t += kDeltaT;
      }
    }
  }

  return false;
}

bool TEBTrajectoryPartition::GetNewStopSForReverse(
    const std::vector<const Obstacle*>& obstacles, const size_t* current_index,
    DiscretizedTrajectory* trajectory, double* stop_s) {
  const size_t point_size = trajectory->NumOfPoints();
  for (size_t i = 0; i < point_size; ++i) {
    const auto& trajectory_point = trajectory->TrajectoryPointAt(i);

    double rel_s = std::fabs(trajectory_point.path_point().s() -
                             trajectory->TrajectoryPointAt(0).path_point().s());
    double ego_theta = trajectory_point.path_point().theta();
    // last point near obs 1m ,no to last point.
    double ego_length = ego_length_ + kCollisionLonBuffer;
    double ego_width = ego_width_ + kCollisionLateralBuffer;
    if (FLAGS_enable_change_collision_buffer &&
        injector_->start_collision_flag_) {
      double lon_buffer = InterpolationLookUp(
          rel_s,
          teb_trajectory_partition_config_.distance_for_start_up_min_buffer(),
          teb_trajectory_partition_config_.distance_for_start_up_max_buffer(),
          teb_trajectory_partition_config_.min_start_up_lon_collision_buffer(),
          teb_trajectory_partition_config_.max_start_up_lon_collision_buffer());
      double lat_buffer = InterpolationLookUp(
          rel_s,
          teb_trajectory_partition_config_.distance_for_start_up_min_buffer(),
          teb_trajectory_partition_config_.distance_for_start_up_max_buffer(),
          teb_trajectory_partition_config_.min_start_up_lat_collision_buffer(),
          teb_trajectory_partition_config_.max_start_up_lat_collision_buffer());
      // AINFO << "lon_buffer = " << lon_buffer;
      // AINFO << "lat_buffer = " << lat_buffer;
      ego_length = ego_length_ + lon_buffer;
      ego_width = ego_width_ + lat_buffer;
    }

    if (point_size - 1 == i) {
      ego_length = ego_length_ + kCollisionBuffer;
    }
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);

    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    bool is_need_update_stop_s =
        CheckObstacleCollision(rel_s, ego_box, obstacles);
    // AINFO << "is_need_update_stop_s = " << is_need_update_stop_s;
    if (is_need_update_stop_s) {
      // TODO(zongxingguo): consider buffer according speed.
      if (injector_->is_in_near_goal_) {
        *stop_s = trajectory_point.path_point().s() + kMinCollisionBuffer;
      } else {
        *stop_s = trajectory_point.path_point().s() + kCollisionLonBuffer;
      }

      return true;
    }
  }

  return false;
}

bool TEBTrajectoryPartition::GetNewStopS(
    const std::vector<const Obstacle*>& obstacles, const size_t* current_index,
    DiscretizedTrajectory* trajectory, double* stop_s) {
  const size_t point_size = trajectory->NumOfPoints();
  for (size_t i = 0; i < point_size; ++i) {
    const auto& trajectory_point = trajectory->TrajectoryPointAt(i);

    double rel_s = std::fabs(trajectory_point.path_point().s() -
                             trajectory->TrajectoryPointAt(0).path_point().s());
    double ego_theta = trajectory_point.path_point().theta();
    // consider collision buffer
    double ego_length = ego_length_ + kCollisionLonBuffer;
    double ego_width = ego_width_ + kCollisionLateralBuffer;
    if (FLAGS_enable_change_collision_buffer &&
        injector_->start_collision_flag_) {
      double lon_buffer = InterpolationLookUp(
          rel_s,
          teb_trajectory_partition_config_.distance_for_start_up_min_buffer(),
          teb_trajectory_partition_config_.distance_for_start_up_max_buffer(),
          teb_trajectory_partition_config_.min_start_up_lon_collision_buffer(),
          teb_trajectory_partition_config_.max_start_up_lon_collision_buffer());
      double lat_buffer = InterpolationLookUp(
          rel_s,
          teb_trajectory_partition_config_.distance_for_start_up_min_buffer(),
          teb_trajectory_partition_config_.distance_for_start_up_max_buffer(),
          teb_trajectory_partition_config_.min_start_up_lat_collision_buffer(),
          teb_trajectory_partition_config_.max_start_up_lat_collision_buffer());
      ego_length = ego_length_ + lon_buffer;
      ego_width = ego_width_ + lat_buffer;
    }

    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    bool is_need_update_stop_s =
        CheckObstacleCollision(rel_s, ego_box, obstacles);
    AINFO << "is_need_update_stop_s = " << is_need_update_stop_s;
    if (is_need_update_stop_s) {
      // TODO(zongxingguo): consider buffer according speed.
      if (injector_->is_in_near_goal_) {
        *stop_s = std::max(
            trajectory_point.path_point().s() - kMinCollisionBuffer, kEpsilon);
      } else {
        *stop_s = std::max(trajectory_point.path_point().s() - kCollisionBuffer,
                           kEpsilon);
      }

      return true;
    }
  }

  return false;
}

bool TEBTrajectoryPartition::GetStartPoint(
    TrajGearPair* current_partitioned_trajectory,
    TrajectoryPoint* start_point) {
  // get start point
  auto* previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto& pre_trajectory =
      previous_frame->open_space_info().chosen_partitioned_trajectory().first;

  AINFO << "pre_trajectory = " << pre_trajectory.size();
  const auto& pre_gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;
  const auto pre_publish_trajectory =
      previous_frame->open_space_info().publishable_trajectory_data().first;
  PublishableTrajectory pre_trajectory_new(
      previous_frame->planning_start_time(),
      previous_frame->open_space_info().chosen_partitioned_trajectory().first);
  const auto& current_gear = current_partitioned_trajectory->second;
  // only pre and current trajectory is same gear,we get start point,else use
  // adc_speed;
  if (current_gear != pre_gear) {
    AERROR << "current_gear != pre_gear";
    return false;
  }
  AINFO << "pre_publish_trajectory: " << pre_publish_trajectory.size();
  if (is_first_plan_success && pre_trajectory.size() > 1) {
    std::vector<TrajectoryPoint> stitching_trajectory;
    const auto& current_timestamp = frame_->planning_start_time();
    double planning_cycle_time =
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);
    // TODO(zongxingguo): total time maby be long.no use position.
    //  const auto& preserved_points_num = pre_trajectory.size();
    bool replan_by_offset = true;
    bool is_poor_status = false;
    std::string replan_reason;
    TrajectoryStitcher::ComputeOpenSpaceStitchingTrajectory(
        injector_->vehicle_state()->vehicle_state(), current_timestamp,
        planning_cycle_time, FLAGS_trajectory_stitching_preserved_length,
        replan_by_offset, &pre_publish_trajectory, &stitching_trajectory,
        &replan_reason, is_poor_status);
    const auto& stitching_point = stitching_trajectory.back();
    *start_point = stitching_point;
    double stitching_point_x = stitching_point.path_point().x();
    double stitching_point_y = stitching_point.path_point().y();

    ADEBUG << "vehicle_state_x = " << std::setprecision(9)
          << injector_->vehicle_state()->vehicle_state().x()
          << ", stitching_point_x = " << std::setprecision(9)
          << stitching_point_x;
    ADEBUG << "vehicle_state_y = " << std::setprecision(9)
          << injector_->vehicle_state()->vehicle_state().y()
          << ", stitching_point_y = " << std::setprecision(9)
          << stitching_point_y;
  }
  return true;
}

void TEBTrajectoryPartition::CreateStopTrajectory(
    DiscretizedTrajectory* trajectory) {
  AERROR << " create stop trajectory.";
  double delta_t = kOptimizedDeltaT;
  std::vector<TrajectoryPoint> backup_points;
  for (size_t i = 0; i < trajectory->size(); ++i) {
    auto trajectory_point = trajectory->at(i);
    PathPoint* path_point = trajectory_point.mutable_path_point();
    if (i > 0) {
      path_point->set_x(trajectory->at(0).path_point().x());
      path_point->set_y(trajectory->at(0).path_point().y());
      path_point->set_theta(trajectory->at(0).path_point().theta());
      path_point->set_kappa(trajectory->at(0).path_point().kappa());
      path_point->set_dkappa(trajectory->at(0).path_point().dkappa());
      path_point->set_ddkappa(trajectory->at(0).path_point().ddkappa());
      path_point->set_s(trajectory->at(0).path_point().s());
      trajectory_point.set_steer(trajectory->at(0).steer());
    }
    trajectory_point.set_v(0.0);
    trajectory_point.set_a(kMinDecel);
    trajectory_point.set_relative_time(i * delta_t);

    backup_points.emplace_back(trajectory_point);
  }
  if (backup_points.size() > kOneSizePoint) {
    trajectory->clear();

  } else {
    AERROR << "backup_points is empty";
    const common::VehicleState& vehicle_state = frame_->vehicle_state();
    for (size_t i = 0; i < kMinSizePoint; ++i) {
      TrajectoryPoint trajectory_point;
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(kMinDecel);
      trajectory_point.set_relative_time(i * delta_t);
      PathPoint* path_point = trajectory_point.mutable_path_point();
      path_point->set_x(vehicle_state.x());
      path_point->set_y(vehicle_state.y());
      path_point->set_theta(vehicle_state.heading());
      path_point->set_kappa(0.0);
      path_point->set_dkappa(0.0);
      path_point->set_ddkappa(0.0);
      path_point->set_s(0.0);
      trajectory_point.set_steer(0.0);
      backup_points.emplace_back(trajectory_point);
    }
  }
  for (size_t i = 0; i < backup_points.size(); ++i) {
    // AINFO << "backup_points[" << i << "] " << backup_points.at(i).v();
    trajectory->emplace_back(backup_points.at(i));
  }
}

void TEBTrajectoryPartition::CalculateReverseTrajectory(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    DiscretizedTrajectory* unpartitioned_trajectory_result,
    TrajGearPair* current_partitioned_trajectory) {
  if (FLAGS_enable_optimized_reverse_trajectory_for_teb_speed) {
    const auto gear = &(current_partitioned_trajectory->second);
    bool is_reverse = canbus::Chassis::GEAR_REVERSE == *gear;
    AINFO << "is_reverse = " << is_reverse;
    auto trajectory = &(current_partitioned_trajectory->first);
    // only for reverse trajectory
    if (is_reverse) {
      // AINFO << "=======start optimized reverse trajectory========";
      ref_v_list_.clear();
      // delete the trajectory that has already been passed
      // TODO(zongxingguo): using interpolation.
      double adc_speed = std::fabs(
          injector_->vehicle_state()->vehicle_state().linear_velocity());
      double init_v = adc_speed;
      TrajectoryPoint start_point;
      start_point.set_v(init_v);
      GetStartPoint(current_partitioned_trajectory, &start_point);
      ShrunkTrajectoryForReverse(&start_point, current_partitioned_trajectory,
                                 trajectory);
      init_v = start_point.v();
      ConvertCostmap();
      double adc_s = trajectory->TrajectoryPointAt(0).path_point().s();
      double back_point_s = trajectory->back().path_point().s();
      double trajectory_length = adc_s - back_point_s;
      // AINFO << "before trajectory_length = " << trajectory_length;
      double new_stop_s = std::numeric_limits<double>::max();
      bool is_need_update_stop_s =
          GetNewStopSForReverse(frame_->obstacles(), &current_trajectory_index,
                                trajectory, &new_stop_s);

      if (is_need_update_stop_s) {
        // if has collision ,we must cut trajectory or stop.
        AINFO << "reverse obs has collision";
        UpdateTrajectoryForReverse(new_stop_s, trajectory);
      }

      trajectory_length = trajectory->front().path_point().s() -
                          trajectory->back().path_point().s();

      adc_speed = std::fabs(init_v);
      // AINFO << "init_v = " << init_v;
      // AINFO << "adc_speed = "
      //       << injector_->vehicle_state()->vehicle_state().linear_velocity();
      double begin_break_s = 0.0;
      std::vector<std::pair<double, double>> ref_s_list;
      std::vector<std::pair<double, double>> ref_v_list;
      bool is_get_total_time = false;
      double total_time = GetTotalTimeForReverse(
          trajectory_length, adc_speed, 1.0, &begin_break_s, &ref_s_list,
          &ref_v_list, &is_get_total_time);
      if (!is_get_total_time) {
        AINFO << "creat stop trajectory point for no get total time";
        // stop trajectory
        CreateStopTrajectory(trajectory);
      } else {
        AINFO << "total_time  =" << total_time;
        for (size_t i = 0; i < ref_s_list.size(); ++i) {
          ref_s_list[i].second =
              trajectory->at(0).path_point().s() - ref_s_list[i].second;
          double v = -ref_v_list_[i].second;
          ref_v_list_[i].second = v;
          // AINFO << "t = " << ref_s_list[i].first
          //       << " s = " << ref_s_list[i].second
          //       << "  v= " << ref_v_list_[i].second;
        }
        OptimizeReproscessForReverse(kOptimizedDeltaT, ref_s_list,
                                     ref_s_list.size(), trajectory);
      }
      AINFO << "REVERSE TRAJECTORY SIZE = " << trajectory->size();
    }
  }
}

void TEBTrajectoryPartition::IsDepartureTrajectory(
    century::planning::DiscretizedTrajectory* trajectory) {
  if (FLAGS_enable_stop_for_departure_trajectory) {
    const common::VehicleState& vehicle_state = frame_->vehicle_state();
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory->size(); ++i) {
      TrajectoryPoint* trajectory_point = &(trajectory->at(i));
      double trajectory_x = trajectory_point->path_point().x();
      double trajectory_y = trajectory_point->path_point().y();
      double distance = std::sqrt((vehicle_state.x() - trajectory_x) *
                                      (vehicle_state.x() - trajectory_x) +
                                  (vehicle_state.y() - trajectory_y) *
                                      (vehicle_state.y() - trajectory_y));
      if (distance < min_distance) {
        min_distance = distance;
      }
    }
    if (min_distance > kDepartureDistance) {
      AINFO << "vehicle departure trajectory ,stop ";
      CreateStopTrajectory(trajectory);
    }
  }
}

void TEBTrajectoryPartition::CollisionCheckForTebSpeed(
    const size_t current_trajectory_index, DiscretizedTrajectory* trajectory,
    bool* is_collision) {
  if (FLAGS_enable_collision_check_for_teb_speed) {
    ConvertCostmap();
    double new_stop_s = std::numeric_limits<double>::max();
    bool is_need_update_stop_s =
        GetNewStopS(frame_->obstacles(), &current_trajectory_index, trajectory,
                    &new_stop_s);
    // AINFO << "is_need_update_stop_s = " << is_need_update_stop_s;
    // AINFO << "new_stop_s = " << new_stop_s;

    if (is_need_update_stop_s) {
      *is_collision = true;
      // new_stop_s always large 0.0
      UpdateTrajectory(new_stop_s, trajectory);
    }
  }
}

bool TEBTrajectoryPartition::StartupForReachDestination(
    bool is_collision, DiscretizedTrajectory* trajectory) {
  double front_point_s = trajectory->front().path_point().s(),
         back_point_s = trajectory->back().path_point().s(),
         trajectory_length = back_point_s - front_point_s,
         adc_speed = std::fabs(
             injector_->vehicle_state()->vehicle_state().linear_velocity());
  // AINFO << "trajectory_length = " << trajectory_length;
  // AINFO << "adc_speed = " << adc_speed;
  // only for Actual vehicle
  if (trajectory_length < kMinDistanceToDestination ||
      trajectory_length > kMaxDistanceToDestination || is_collision ||
      adc_speed > kConsiderSpeedStartup || frame_->is_sim_control()) {
    return false;
  }
  double init_v = kMinSpeedStartup,
         init_a = init_v * init_v * 0.5 / trajectory_length,
         total_time = init_v / init_a, delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  std::vector<double> s_vector, ds_vector, dds_vector;

  for (int i = 0; i < num_of_knots; ++i) {
    s_vector.emplace_back(init_v * (i * delta_t) -
                       0.5 * init_a * (i * delta_t) * (i * delta_t) +
                       trajectory->front().path_point().s());
    ds_vector.emplace_back(init_v - init_a * (i * delta_t));
    dds_vector.emplace_back(-init_a);
  }
  std::vector<TrajectoryPoint> backup_points;
  for (int i = 0; i < num_of_knots; ++i) {
    if (!(s_vector[i] >= trajectory->front().path_point().s())) {
      continue;
    }
    if (!(s_vector[i] <= trajectory->back().path_point().s())) {
      break;
    }
    auto comp = [](const TrajectoryPoint point, const double s) {
      return point.path_point().s() < s;
    };
    auto it_lower = std::lower_bound(trajectory->begin(), trajectory->end(),
                                     s_vector[i], comp);
    const auto& index = std::distance(trajectory->begin(), it_lower);
    double v = ds_vector[i], a = dds_vector[i];
    auto trajectory_point = trajectory->at(index);
    PathPoint* path_point = trajectory_point.mutable_path_point();
    // except front point
    if (index > 0) {
      const auto& start_trajectory_point = trajectory->at(index - 1);
      const auto& end_trajectory_point = trajectory->at(index);
      const auto& start_path_point = start_trajectory_point.path_point();
      const auto& end_path_point = end_trajectory_point.path_point();
      double dist = std::abs(end_path_point.s() - start_path_point.s()),
             dx = (end_path_point.x() - start_path_point.x()) / dist,
             dy = (end_path_point.y() - start_path_point.y()) / dist,
             dtheta = (NormalizeAngle(end_path_point.theta() -
                                      start_path_point.theta())) /
                      dist,
             dkappa =
                 (end_path_point.kappa() - start_path_point.kappa()) / dist,
             ddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             dddkappa =
                 (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist,
             ds = (end_path_point.s() - start_path_point.s()) / dist,
             dsteer = (end_trajectory_point.steer() -
                       start_trajectory_point.steer()) /
                      dist,
             d = s_vector[i] - start_path_point.s();
      if (d < 0.0) {
        continue;
      }
      path_point->set_x(start_path_point.x() + dx * d);
      path_point->set_y(start_path_point.y() + dy * d);
      path_point->set_theta(
          NormalizeAngle(start_path_point.theta() + dtheta * d));

      path_point->set_kappa(start_path_point.kappa() + dkappa * d);
      path_point->set_dkappa(start_path_point.dkappa() + ddkappa * d);
      path_point->set_ddkappa(start_path_point.ddkappa() + dddkappa * d);
      path_point->set_s(start_path_point.s() + ds * d);
      trajectory_point.set_steer(start_trajectory_point.steer() + dsteer * d);
    }
    // AINFO << "backup v = " << v << "   a = " << a;
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(i * delta_t);

    backup_points.emplace_back(trajectory_point);
  }
  CheckBackupPoints(backup_points, trajectory);
  return true;
}

void TEBTrajectoryPartition::CheckBackupPoints(
    std::vector<TrajectoryPoint> backup_points,
    DiscretizedTrajectory* trajectory) {
  if (backup_points.size() > kOneSizePoint) {
    trajectory->clear();
  } else {
    GetStopBackupPoints(&backup_points);
  }
  for (size_t i = 0; i < backup_points.size(); ++i) {
    trajectory->emplace_back(backup_points.at(i));
  }
}

void TEBTrajectoryPartition::OptimizeTrajectory(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    DiscretizedTrajectory* unpartitioned_trajectory_result,
    TrajGearPair* current_partitioned_trajectory) {
  // optimized for reverse trajectory.
  double optimized_start_time = Clock::NowInSeconds();
  CalculateReverseTrajectory(partitioned_trajectories, current_trajectory_index,
                             closest_trajectory_point_index,
                             unpartitioned_trajectory_result,
                             current_partitioned_trajectory);
  double optimized_end_time = Clock::NowInSeconds();
  AINFO << "get_reverse_use_time = "
        << optimized_end_time - optimized_start_time;
  const auto gear = &(current_partitioned_trajectory->second);
  bool is_reverse = canbus::Chassis::GEAR_REVERSE == *gear;
  auto trajectory = &(current_partitioned_trajectory->first);
  // optimized for drive trajectory.
  if (FLAGS_enable_use_qp_for_teb_speed && !is_reverse) {
    double adc_speed = std::fabs(
        injector_->vehicle_state()->vehicle_state().linear_velocity());
    double init_v = adc_speed, init_a = trajectory->front().a();
    TrajectoryPoint start_point;
    start_point.set_v(init_v);
    start_point.set_a(init_a);
    GetStartPoint(current_partitioned_trajectory, &start_point);
    ShrunkTrajectory(&start_point, current_partitioned_trajectory, trajectory);
    init_v = start_point.v();
    init_a = start_point.a();
    // update new stop s according obstacle collision.
    bool is_collision = false;
    CollisionCheckForTebSpeed(current_trajectory_index, trajectory,
                              &is_collision);
    ADEBUG << "is_collision: " << is_collision;

    if (is_stop_trajectory_) {
      AINFO << "STOP TRAJECTORY NO QP";
    } else {
      // total_length if 0.0,we need to fast break.
      double total_length = trajectory->back().path_point().s() -
                            trajectory->front().path_point().s(),

             target_v = kTargetSpeed, delta_t = kOptimizedDeltaT,
             begin_break_s = 0.0;
      std::vector<std::pair<double, double>> ref_s_list;
      bool is_get_total_time = false;
      double total_time =
          GetTotalTime(total_length, init_v, target_v, &begin_break_s,
                       &ref_s_list, &is_get_total_time);
      // AINFO << "begin_break_s = " << begin_break_s;
      // AINFO << "total_time = " << total_time;
      //  Extend the planning time, otherwise due to speed limits, it may not be
      //  possible to reach the final point.
      total_time = std::ceil(total_time) + FLAGS_rescue_extra_time;
      if (total_length < kMaxDistanceToDestination) {
        total_time = std::ceil(total_time) + kMinOptimizedTotalTime;
      }
      // AINFO << "total_time = " << total_time;

      // The previous frame was the braking trajectory, and without re planning,
      // starting may not be possible.
      if (init_v < kEpsilon) {
        init_a = 0.0;
      }
      // AINFO << "init_a = " << init_a;
      // AINFO << "init_v = " << init_v;
      if (init_v < 0.0) {
        // AERROR << "error qp for no start init speed : " << init_v;
        init_v = 0.0;
      }
      std::array<double, 3> init_s = {0.0, init_v, init_a};
      int num_of_knots =
          static_cast<int>(std::max(7.0, total_time) / delta_t) + 1;
      PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                       init_s);
      // AINFO << "num_of_knots = " << num_of_knots;

      SetProblemStatus(num_of_knots, adc_speed, total_length,
                       &piecewise_jerk_problem);

      std::vector<double> x_ref;
      // std::vector<double> penalty_dx;

      std::vector<std::pair<double, double>> s_dot_bounds;
      SetSpeedLimitForQp(num_of_knots, is_get_total_time, total_length,
                         adc_speed, ref_s_list, init_s, &x_ref, &s_dot_bounds,
                         &piecewise_jerk_problem);

      piecewise_jerk_problem.set_x_ref(200, std::move(x_ref));

      if ((piecewise_jerk_problem.Optimize().ok())) {
        OptimizeReproscess(delta_t, &piecewise_jerk_problem, num_of_knots,
                           trajectory);

      } else {
        CreateStopTrajectory(trajectory);
        AERROR << "teb piecewise jerk speed faild , create stop trajectory.";
      }
    }
    if (FLAGS_enable_startup_for_reach_destination) {
      if (StartupForReachDestination(is_collision, trajectory)) {
        AINFO << "NO USE QP FOR STOP";
      }
    }
  }
  CheckTrajectory(trajectory);
}

void TEBTrajectoryPartition::SetSpeedLimitForQp(
    int num_of_knots, bool is_get_total_time, double total_length,
    double adc_speed, const std::vector<std::pair<double, double>>& ref_s_list,
    const std::array<double, 3>& init_s, std::vector<double>* x_ref,
    std::vector<std::pair<double, double>>* s_dot_bounds,
    PiecewiseJerkSpeedProblem* piecewise_jerk_problem) {
  double min_v_upper_bound = std::numeric_limits<double>::max();
  for (int i = 0; i < num_of_knots; ++i) {
    if (FLAGS_enable_teb_speed_limit && is_get_total_time) {
      size_t index = static_cast<size_t>(i) < ref_s_list.size() - 1
                         ? i
                         : ref_s_list.size() - 1;
      x_ref->emplace_back(ref_s_list[index].second);
      // AINFO << "ref_s_list[" << i
      //       << "].second = " << ref_s_list[index].second;
      double v_upper_bound =
          speed_limit_.GetSpeedLimitByS(ref_s_list[index].second);
      min_v_upper_bound =
          min_v_upper_bound < v_upper_bound ? min_v_upper_bound : v_upper_bound;
      s_dot_bounds->emplace_back(0.0, std::fmax(v_upper_bound, 0.0));
    } else {
      x_ref->emplace_back(total_length);
      s_dot_bounds->resize(static_cast<size_t>(num_of_knots));
      std::pair<double, double> value(0.0, std::fmax(2.0, adc_speed));
      std::fill(s_dot_bounds->begin(), s_dot_bounds->end(), value);
    }

    // TODO(zongxingguo): get curvature .

    // TODO(zongxingguo):  get v_upper_bound.
  }
  if (FLAGS_enable_teb_speed_limit && is_get_total_time) {
    ReconstructSpeedLimitByKinematic(min_v_upper_bound, init_s[1], init_s[2],
                                     s_dot_bounds);
    (*s_dot_bounds)[num_of_knots - 1] = std::make_pair(0.0, 0.0);
    piecewise_jerk_problem->set_dx_bounds(*s_dot_bounds);
  }
}

void TEBTrajectoryPartition::CheckTrajectory(
    DiscretizedTrajectory* trajectory) {
  IsDepartureTrajectory(trajectory);
  if (trajectory->size() < kTwoSizePoint) {
    CreateStopTrajectory(trajectory);
    AERROR << "trajectory size < 2,stop.";
  }
}

void TEBTrajectoryPartition::ReconstructSpeedLimitByKinematic(
    const double min_v_upper_bound, double init_speed, double init_acc,
    std::vector<std::pair<double, double>>* s_dot_bounds) {
  const auto& config = teb_trajectory_partition_config_;
  const double max_dec =
      config.reduction_ratio() * common::VehicleConfigHelper::GetConfig()
                                     .vehicle_param()
                                     .max_deceleration();
  const double max_jerk_lower_bound =
      config.reduction_ratio() * FLAGS_longitudinal_jerk_lower_bound;
  init_speed += config.speed_buffer();
  init_acc += config.deceleration_buffer();
  const double t_max_v_by_jerk = -init_acc / max_jerk_lower_bound;
  // setting first speed limit as the init_speed, when init_speed is bigger
  // than first speed limit
  if ((*s_dot_bounds)[0UL].second < init_speed) {
    (*s_dot_bounds)[0UL].second = init_speed;
  }
  // adjust the speed limit value according the vehicle kinematic
  for (size_t i = 1UL; i < (*s_dot_bounds).size(); ++i) {
    double& curr_v = (*s_dot_bounds)[i].second;
    double curr_t = i * kOptimizedDeltaT;
    double min_v_by_acc = init_speed + max_dec * curr_t;
    double min_v_by_jerk = init_speed + init_acc * curr_t +
                           0.5 * max_jerk_lower_bound * curr_t * curr_t;
    if (min_v_by_acc <= min_v_upper_bound &&
        (curr_t >= t_max_v_by_jerk && min_v_by_jerk <= min_v_upper_bound)) {
      // Subsequent min_v_by_acc and min_v_by_jerk are no longer greater than
      // min_v_upper_bound, so there is no need to adjust subsequent speed limit
      // value.
      break;
    }
    curr_v = std::max(curr_v, std::max(min_v_by_acc, min_v_by_jerk));
    curr_v = std::max(curr_v, 0.0);
  }
}

double TEBTrajectoryPartition::GetObstacleAndPathPointDistance(
    const Obstacle& obstacle, const PathPoint& path_point) {
  Box2d ego_box;
  const double path_point_theta = path_point.theta();
  Vec2d path_adc_point{path_point.x(), path_point.y()};
  ego_box = Box2d(path_adc_point, path_point_theta, ego_length_, ego_width_);
  Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                  shift_distance_ * std::sin(path_point_theta)};
  ego_box.Shift(shift_vec);
  double dist = 0.0;
  const auto& obs_type = obstacle.Perception().type();
  if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
      FLAGS_enable_openspace_use_polygon_plan) {
    dist = obstacle.PerceptionPolygon().DistanceTo(ego_box);
  } else {
    dist = obstacle.PerceptionBoundingBox().DistanceTo(ego_box);
  }
  return dist;
}

bool TEBTrajectoryPartition::CalcuNeastTrajectoryPoint(
    const Obstacle& obstacle, const DiscretizedTrajectory& trajectory,
    const bool is_reverse_traj, bool* is_vehicle_box_valid,
    std::vector<common::math::Box2d>* vehicle_boxs,
    std::pair<size_t, double>* neast_traj_info) {
  neast_traj_info->first = std::numeric_limits<size_t>::max();
  neast_traj_info->second = kLargeDist;
  ADEBUG << "obs id: " << obstacle.Id();
  if (trajectory.size() < 2UL) {
    return false;
  }
  double sign_coeff = is_reverse_traj ? -1.0 : 1.0;
  double delta_s =
      std::abs(trajectory[0].path_point().s() - trajectory[1].path_point().s());
  double min_center_distance = 0.5 * std::hypot(obstacle.Perception().length(),
                                                obstacle.Perception().width()) +
                               0.5 * std::hypot(ego_length_, ego_width_);
  size_t step = 1UL;
  double last_lon_dis = std::numeric_limits<double>::max();
  size_t start_index = 0UL, end_index = trajectory.size();
  for (size_t i = 0; i < trajectory.size(); i += step) {
    const auto& curr_traj = trajectory[i].path_point();
    ADEBUG << "curr_traj.s :" << curr_traj.s();
    if (sign_coeff * curr_traj.s() < 0.0) {
      continue;
    }
    const auto& obs_pos = obstacle.Perception().position();
    double lon_dis =
        sign_coeff *
        common::math::Vec2d::CreateUnitVec2d(curr_traj.theta())
            .InnerProd(common::math::Vec2d(obs_pos.x() - curr_traj.x(),
                                           obs_pos.y() - curr_traj.y()));
    ADEBUG << "lon_dis: " << lon_dis;
    if (lon_dis < 0.0) {
      end_index = std::min(i + kSearchIndexBuffer, trajectory.size());
      break;
    } else {
      start_index = std::max(i, kSearchIndexBuffer) - kSearchIndexBuffer;
      end_index = std::min(i + kSearchIndexBuffer, trajectory.size());
    }
    last_lon_dis = lon_dis;
    double step_dis = std::max(
        kMinSearchStepDistance,
        kSearchDistanceRatio *
            (lon_dis - min_center_distance -
             teb_trajectory_partition_config_.max_dis_for_speed_limit()));
    step = std::min(
        static_cast<size_t>(std::floor(step_dis / delta_s)),
        static_cast<size_t>(std::floor(
            kMaxSearchStepRatio * static_cast<double>(trajectory.size() - i))));
    step = std::max(1UL, step);
    ADEBUG << "step = " << step;
  }
  if ((last_lon_dis - 2.0 * min_center_distance) >
      teb_trajectory_partition_config_.max_dis_for_speed_limit()) {
    return false;
  }
  ADEBUG << "start_index: " << start_index << "end_index: " << end_index;
  ACHECK(end_index >= start_index);
  for (size_t i = start_index; i < end_index; i += kFineSearchStep) {
    double dist =
        GetObstacleAndPathPointDistance(obstacle, trajectory[i].path_point());
    if (dist < neast_traj_info->second) {
      neast_traj_info->first = i;
      neast_traj_info->second = dist;
    }
  }

  auto it = obstacle_trajectory_info_.find(obstacle.Id());
  if (obstacle_trajectory_info_.end() != it) {
    it->second = *neast_traj_info;
  } else {
    obstacle_trajectory_info_.emplace(obstacle.Id(), *neast_traj_info);
  }
  return true;
}

void TEBTrajectoryPartition::CalcuSpeedLimit(
    const TrajGearPair& partitioned_trajectory) {
  const auto& current_trajectory = partitioned_trajectory.first;
  const auto& gear = partitioned_trajectory.second;
  const bool is_reverse_traj = canbus::Chassis::GEAR_REVERSE == gear;
  static ObstacleStabilizationForTEBSpeed obs_stable(100UL);
  speed_limit_.InitSpeedLimit(
      0.0, current_trajectory.back().path_point().s(),
      teb_trajectory_partition_config_.s_step_for_speed_limit(),
      teb_trajectory_partition_config_.teb_max_speed_limit());
  bool is_vehicle_box_valid = false;
  std::vector<common::math::Box2d> vehicle_boxs;
  vehicle_boxs.clear();
  std::pair<size_t, double> obs_neast_traj_point_info;
  for (const auto* obstacle : frame_->open_space_roi_obstacles()) {
    if (!CalcuNeastTrajectoryPoint(*obstacle, current_trajectory,
                                   is_reverse_traj, &is_vehicle_box_valid,
                                   &vehicle_boxs, &obs_neast_traj_point_info)) {
      continue;
    }
    TrajectoryPoint neast_traj_point =
        current_trajectory[obs_neast_traj_point_info.first];
    double neast_point_s = neast_traj_point.path_point().s();

    // adc position is (0, 0)
    const Vec2d obs_pos(obstacle->Perception().position().x(),
                        obstacle->Perception().position().y());
    const Vec2d neast_traj_point_pos(neast_traj_point.path_point().x(),
                                     neast_traj_point.path_point().y());
    bool is_left_obs = neast_traj_point_pos.CrossProd(obs_pos) > 0;
    double lat_stable_dis = obs_stable.GetObstacleDistance(
        obstacle->Id(), obs_neast_traj_point_info.second, is_left_obs,
        neast_point_s);
    double limited_speed =
        teb_trajectory_partition_config_.teb_max_speed_limit();
    if (lat_stable_dis <
        teb_trajectory_partition_config_.min_dis_for_speed_limit()) {
      limited_speed = teb_trajectory_partition_config_.teb_min_speed_limit();
    } else if (lat_stable_dis <
               teb_trajectory_partition_config_.mid_dis_for_speed_limit()) {
      limited_speed = InterpolationLookUp(
          lat_stable_dis,
          teb_trajectory_partition_config_.min_dis_for_speed_limit(),
          teb_trajectory_partition_config_.mid_dis_for_speed_limit(),
          teb_trajectory_partition_config_.teb_min_speed_limit(),
          teb_trajectory_partition_config_.teb_mid_speed_limit());
    } else if (lat_stable_dis <
               teb_trajectory_partition_config_.max_dis_for_speed_limit()) {
      limited_speed = InterpolationLookUp(
          lat_stable_dis,
          teb_trajectory_partition_config_.mid_dis_for_speed_limit(),
          teb_trajectory_partition_config_.max_dis_for_speed_limit(),
          teb_trajectory_partition_config_.teb_mid_speed_limit(),
          teb_trajectory_partition_config_.teb_max_speed_limit());
    }
    speed_limit_.UpdateSpeedLimitWithRawDataBySRange(
        neast_point_s - kSpeedLimitRange, neast_point_s + kSpeedLimitRange,
        limited_speed);
  }
  const auto& speed_limit_points = speed_limit_.GetSpeedLimitRawData();
  for (const auto& limit_point : speed_limit_points) {
    ADEBUG << "obstacle speed limit point: (" << limit_point.first << ", "
           << limit_point.second << ")";
  }
  // calculate curve speed limit CalTraceCurveSpeed
  for (const auto& limit_point : speed_limit_points) {
    auto compare_s = [](const common::TrajectoryPoint& point, const double s) {
      return point.path_point().s() < s;
    };
    auto lower =
        std::lower_bound(current_trajectory.begin(), current_trajectory.end(),
                         limit_point.first, compare_s);

    if (current_trajectory.end() == lower) {
      continue;
    }
    double trajectory_point_kappa = std::fabs(lower->path_point().kappa());
    double curve_limited_v = 0.0;
    if (trajectory_point_kappa > teb_max_curve_thr_) {
      curve_limited_v = teb_max_curve_speed_limit_;
    } else {
      curve_limited_v = teb_max_speed_limit_ -
                        (teb_max_speed_limit_ - teb_max_curve_speed_limit_) *
                            (trajectory_point_kappa - teb_min_curve_thr_) /
                            (teb_max_curve_thr_ - teb_min_curve_thr_);
    }
    ADEBUG << "curve_limited_v = " << curve_limited_v;
  }

  for (const auto& limit_point : speed_limit_points) {
    ADEBUG << "total speed limit point: (" << limit_point.first << ", "
           << limit_point.second << ")";
  }
}

void TEBTrajectoryPartition::NewAdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    DiscretizedTrajectory* unpartitioned_trajectory_result,
    TrajGearPair* current_partitioned_trajectory) {
  start_c_id_ = 0;
  end_c_id_ = 0;
  const size_t partitioned_trajectories_size = partitioned_trajectories.size();
  CHECK_GT(partitioned_trajectories_size, current_trajectory_index);

  // Reassign relative time and relative s to have the closest point as origin
  // point
  *(current_partitioned_trajectory) =
      partitioned_trajectories.at(current_trajectory_index);
  auto trajectory = &(current_partitioned_trajectory->first);
  const auto gear = &(current_partitioned_trajectory->second);
  bool is_reverse = canbus::Chassis::GEAR_REVERSE == *gear;
  ADEBUG << "*gear " << *gear << ", is_reverse: " << is_reverse
         << ", current_trajectory_index " << current_trajectory_index;

  double origin_start_time = trajectory->front().relative_time();
  double origin_end_time = trajectory->back().relative_time();
  PullOverRoutingEndExtendLength(current_trajectory_index,
                                 partitioned_trajectories_size, is_reverse,
                                 trajectory);
  TraceInterpolatePolyfitSmooth(is_reverse, trajectory);

  ADEBUG << "after polyfit_trajectory_size: " << trajectory->size();
  openspace_common_->CalTheta(is_reverse, trajectory);
  ADEBUG << "after caltheta_trajectory_size: " << trajectory->size();
  CalCurve(is_reverse, trajectory);
  FillTrajectoryTime(origin_start_time, origin_end_time, trajectory);
  ADEBUG << "calcurve_trajectory_size: " << trajectory->size();

  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  ADEBUG << "after poly fit _back_point X-Y = " << std::setprecision(9)
         << trajectory->back().path_point().x() << "     "
         << std::setprecision(9) << trajectory->back().path_point().y();
  size_t adc_index = trajectory->QueryNearestPoint(adc_position);
  ADEBUG << "partition_adc_index: " << adc_index;
  size_t use_id =
      std::max(std::min(adc_index + 1, trajectory->size() - 1), kZero);
  double time_shift = trajectory->at(use_id).relative_time();
  double s_shift = trajectory->at(use_id).path_point().s();

  const double adc_speed =
      std::fabs(injector_->vehicle_state()->vehicle_state().linear_velocity());

  double adc_s = trajectory->at(use_id).path_point().s() - s_shift;

  for (size_t i = 0; i < trajectory->size(); ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));

    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);

    // If the trajectory point near the obstacles to slow down
    const PathPoint& path_point = trajectory_point->path_point();
    const double path_point_x = path_point.x();
    const double path_point_y = path_point.y();
    const double path_point_theta = path_point.theta();
    Vec2d path_point_vec{path_point_x, path_point_y};

    bool is_need_speed_limit = true;
    double frenet_obs_v = 0.0;
    if (!FLAGS_enable_use_qp_for_teb_speed) {
      frenet_obs_v =
          CalFrenetSpeedSlowDown(trajectory, i, adc_s, gear, &path_point_vec,
                                 path_point_theta, &is_need_speed_limit);
    }
    const common::VehicleState& vehicle_state = frame_->vehicle_state();
    const double vehicle_heading = vehicle_state.heading();
    const common::math::Vec2d vehicle_position = {vehicle_state.x(),
                                                  vehicle_state.y()};
    double flu_obs_limited_v = 0.0;
    if (!FLAGS_enable_use_qp_for_teb_speed) {
      flu_obs_limited_v =
          GetFluObsToVehicleSpeed(vehicle_position, vehicle_heading);
    }
    // AINFO << "flu_obs_limited_v: " << flu_obs_limited_v;
    double obs_limited_v = std::min(frenet_obs_v, flu_obs_limited_v);

    // According to the trajectory curve to slow down
    double curve_limited_v = teb_max_speed_limit_;
    curve_limited_v = CalTraceCurveSpeed(trajectory, i);

    // AINFO << "curve_limited_v " << curve_limited_v
    //       << " | trajectory_point_kappa" << trajectory_point_kappa;

    // According to the to end_point dis to slow down
    double dis_limited_v = teb_max_speed_limit_;
    dis_limited_v = CalNearEndDisSpeed(trajectory, i, s_shift);
    // AINFO << "dis_limited_v " << dis_limited_v << " | dis" << dis;

    // merge speed
    double smooth_v = std::min(obs_limited_v, curve_limited_v);
    smooth_v = std::min(dis_limited_v, smooth_v);
    if (i == use_id) {
      AINFO << "use_id";
    }
    // AINFO << "end_v: " << smooth_v << " dis_v " << dis_limited_v
    //       << " curve_v " << curve_limited_v
    //       << " frenet_v " << frenet_obs_v
    //       << "flu_v " << flu_obs_limited_v;

    // Manually adjusting trajectory speed
    if (*gear != canbus::Chassis::GEAR_REVERSE) {
      // !R
      int delta = i > use_id ? i - use_id : 1;
      smooth_v = std::min(std::fabs(adc_speed) + delta * teb_min_speed_limit_,
                          smooth_v);
      if (std::fabs(smooth_v) < teb_min_speed_limit_ && !is_need_speed_limit) {
        smooth_v = teb_min_speed_limit_;
      }
      // smooth_v = (i < adc_index) ? 0.0 : smooth_v;
      // AINFO << "D smooth_v: " << smooth_v;
      trajectory_point->set_v(smooth_v);
      if (i + 1 >= trajectory->size()) {
        trajectory_point->set_v(0.0);
      }
    } else {
      // R
      int delta = i > use_id ? i - use_id : 1;
      smooth_v = std::min(std::fabs(adc_speed) + delta * teb_min_speed_limit_,
                          smooth_v);
      if (std::fabs(smooth_v) < teb_min_speed_limit_ && !is_need_speed_limit) {
        smooth_v = -teb_min_speed_limit_;
      } else {
        smooth_v = -smooth_v;
      }
      smooth_v = std::max(smooth_v, -teb_max_speed_limit_ * 0.5);
      // smooth_v = (i < adc_index) ? 0.0 : smooth_v;
      // AINFO << "R smooth_v: " << smooth_v;
      trajectory_point->set_v(smooth_v);
      if (i + 1 >= trajectory->size()) {
        trajectory_point->set_v(0.0);
      }
    }
  }

  // AINFO << "current_trajectory_index " << current_trajectory_index;
  // AINFO << "close_trajectory_point_index " << closest_trajectory_point_index;
  // AINFO << "chose_trajectory_size " << trajectory->size();

  // Reassign relative t and s on stitched_trajectory_result for accurate next
  // frame stitching
  const size_t interpolated_pieces_num =
      teb_trajectory_partition_config_.interpolated_pieces_num();
  const size_t unpartitioned_trajectory_size =
      unpartitioned_trajectory_result->size();
  size_t index_estimate = 0;
  for (size_t i = 0; i < current_trajectory_index; ++i) {
    index_estimate += partitioned_trajectories[i].first.size();
  }
  index_estimate += closest_trajectory_point_index;
  index_estimate /= interpolated_pieces_num;
  if (index_estimate >= unpartitioned_trajectory_size) {
    index_estimate = unpartitioned_trajectory_size - 1;
  }
  time_shift =
      unpartitioned_trajectory_result->at(index_estimate).relative_time();
  s_shift =
      unpartitioned_trajectory_result->at(index_estimate).path_point().s();
  for (size_t i = 0; i < unpartitioned_trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point =
        &(unpartitioned_trajectory_result->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
}

bool TEBTrajectoryPartition::CheckFinishInitPosition(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* current_partitioned_trajectory) {
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  *(current_partitioned_trajectory) = partitioned_trajectories.at(0);
  auto trajectory = &(current_partitioned_trajectory->first);
  size_t adc_index = trajectory->QueryNearestPoint(adc_position);
  auto gear = &(current_partitioned_trajectory->second);

  const size_t trajectory_size = trajectory->size();

  TrajectoryPoint* start_point = &(trajectory->at(adc_index));
  TrajectoryPoint* end_point = &(trajectory->at(trajectory_size - 1));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  // dist_to_end has a dispersion error
  double dist_to_end =
      end_point->path_point().s() - start_point->path_point().s();
  double dx = end_point->path_point().x() - injector_->vehicle_state()->x();
  double dy = end_point->path_point().y() - injector_->vehicle_state()->y();
  double dist_to_end2 = std::hypot(dx, dy);

  TrajectoryPoint* point = &(trajectory->at(0));
  double length = end_point->path_point().s() - point->path_point().s();
  AINFO << "dist_to_end = " << dist_to_end << " *gear " << *gear << "adc_speed "
        << adc_speed << "length " << length << "dist_to_end2 " << dist_to_end2;
  if (canbus::Chassis::GEAR_REVERSE == *gear) {
    return (dist_to_end > kBackReachS || dist_to_end2 < std::fabs(kReachS)) &&
           std::fabs(adc_speed) < linear_velocity_threshold_on_ego_;
  } else {
    return (dist_to_end < longitudinal_offset_to_midpoint_ ||
            dist_to_end2 < std::fabs(longitudinal_offset_to_midpoint_)) &&
           std::fabs(adc_speed) < linear_velocity_threshold_on_ego_;
  }
  return false;
}

Vec2d TEBTrajectoryPartition::ComputeProjection(const Vec2d& a, const Vec2d& b,
                                                const Vec2d& p) {
  Vec2d ab_vec = b - a;  // Create vector from point a to b
  Vec2d ap_vec = p - a;  // Create vector from point a to p

  // Compute the projection of ap_vec onto ab_vec
  double projection_length = ap_vec.InnerProd(ab_vec) / ab_vec.LengthSquare();
  Vec2d projection_vec = ab_vec * projection_length;

  // The projection point is then `a` point plus the projection vector
  Vec2d projection_point = a + projection_vec;

  return projection_point;
}

double TEBTrajectoryPartition::ComputeSideLonDistanceToProjection(
    const Vec2d& a, const Vec2d& b, const Vec2d& projection_point) {
  Vec2d ab = b - a;
  Vec2d ap = projection_point - a;

  // Compute the signed distance from 'a' to the projection point along 'ab'
  double signed_distance = ap.InnerProd(ab) > 0 ? ap.Length() : -ap.Length();

  return signed_distance;
}

bool TEBTrajectoryPartition::NewCheckFinishInitPosition(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* current_partitioned_trajectory) {
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  // first trajectory use id 0
  *(current_partitioned_trajectory) = partitioned_trajectories.at(0);
  auto trajectory = &(current_partitioned_trajectory->first);
  size_t adc_index = trajectory->QueryNearestPoint(adc_position);
  auto gear = &(current_partitioned_trajectory->second);
  common::math::Vec2d nearest_point = {
      (&(trajectory->at(adc_index)))->path_point().x(),
      (&(trajectory->at(adc_index)))->path_point().y()};

  double projection_dis = 0.0;
  if (adc_index + 1 < trajectory->size()) {
    const common::math::Vec2d next_nearest_point = {
        (&(trajectory->at(adc_index + 1)))->path_point().x(),
        (&(trajectory->at(adc_index + 1)))->path_point().y()};
    const Vec2d projection_point =
        ComputeProjection(nearest_point, next_nearest_point, adc_position);
    projection_dis = ComputeSideLonDistanceToProjection(
        nearest_point, next_nearest_point, projection_point);
  }
  // AINFO << "projection_dis: " << projection_dis;
  projection_dis = (canbus::Chassis::GEAR_REVERSE == *gear) ? -projection_dis
                                                            : projection_dis;
  // AINFO << "gear projection_dis: " << projection_dis;

  const size_t trajectory_size = trajectory->size();
  AINFO << "NewCheckFinishInitPosition_trajectory_size: " << trajectory_size;

  TrajectoryPoint* start_point = &(trajectory->at(adc_index));
  TrajectoryPoint* end_point = &(trajectory->at(trajectory_size - 1));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  // dist_to_end has a dispersion error
  double dist_to_end = end_point->path_point().s() -
                       (start_point->path_point().s() + projection_dis);
  double dx = end_point->path_point().x() - injector_->vehicle_state()->x();
  double dy = end_point->path_point().y() - injector_->vehicle_state()->y();
  double dist_to_end2 = std::hypot(dx, dy);

  TrajectoryPoint* point = &(trajectory->at(0));
  double length = end_point->path_point().s() - point->path_point().s();
  AINFO << "dist_to_end = " << dist_to_end << " *gear " << *gear << "adc_speed "
        << adc_speed << "length " << length << "dist_to_end2 " << dist_to_end2;
  if (canbus::Chassis::GEAR_REVERSE == *gear) {
    return (dist_to_end > -switch_trace_thr_s_ &&
            std::fabs(adc_speed) < linear_velocity_threshold_on_ego_);
  } else {
    return (dist_to_end < switch_trace_thr_s_) &&
           std::fabs(adc_speed) < linear_velocity_threshold_on_ego_;
  }
  return false;
}

double TEBTrajectoryPartition::GetObstacleDist(const common::math::Vec2d& point,
                                               const double& heading) {
  Box2d ego_box(point, heading, ego_length_, ego_width_);
  Vec2d shift_vec{shift_distance_ * std::cos(heading),
                  shift_distance_ * std::sin(heading)};
  ego_box.Shift(shift_vec);

  double min_dist = kLargeDist;
  for (const auto& obstacle : frame_->open_space_roi_obstacles()) {
    common::math::Vec2d obs_point =
        Vec2d(obstacle->Perception().position().x(),
              obstacle->Perception().position().y());
    double car2obs_dis = point.DistanceTo(obs_point);
    if (car2obs_dis > FLAGS_cal_speed_filter_too_far_obs) {
      continue;
    }

    double dist = 0.0;
    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        FLAGS_enable_openspace_use_polygon_plan) {
      dist = obstacle->PerceptionPolygon().DistanceTo(ego_box);
    } else {
      dist = obstacle->PerceptionBoundingBox().DistanceTo(ego_box);
    }
    if (dist < min_dist) {
      min_dist = dist;
      if (min_dist < kMinDist) {
        ADEBUG << "RS obstacle dist too little , think block" << obstacle->Id();
        return -1.0;
      }
    }
  }

  if (FLAGS_enable_use_costmap) {
    for (const auto polygon : frame_->costmap_polygons()) {
      double dist = 0.0;
      dist = polygon.DistanceTo(ego_box);
      if (dist < min_dist) {
        min_dist = dist;
        if (min_dist < kMinDist) {
          ADEBUG << "costmap obstacle dist too little , think block";
          return -1.0;
        }
      }
    }
  }

  return min_dist;
}

double TEBTrajectoryPartition::CalFrenetSpeedSlowDown(
    century::planning::DiscretizedTrajectory* trajectory, const size_t& i,
    const double& adc_s, century::canbus::Chassis::GearPosition* const gear,
    century::common::math::Vec2d* path_point_vec, const double path_point_theta,
    bool* is_need_speed_limit) {
  double error_lat_dist = GetObstacleDist((*path_point_vec), path_point_theta);
  double lon_dis = std::fabs(trajectory->at(i).path_point().s() - adc_s);
  double frenet_obs_v = teb_max_speed_limit_;
  const double adc_speed =
      std::fabs(injector_->vehicle_state()->vehicle_state().linear_velocity());
  if (*gear != canbus::Chassis::GEAR_REVERSE) {
    if (adc_speed >= kVehicleStaticThr) {
      lon_dis = lon_dis - (front_to_center_ + kSafeBuffer +
                           adc_speed * kLonSafeBufferSpeedCoff);
      error_lat_dist =
          error_lat_dist - adc_speed * kLatSafeBufferSpeedCoff - kSafeBuffer;
    } else {
      lon_dis = lon_dis - (front_to_center_);
    }
  } else {
    if (adc_speed >= kVehicleStaticThr) {
      lon_dis = lon_dis - (back_to_center_ + kSafeBuffer +
                           adc_speed * kLonSafeBufferSpeedCoff);
      error_lat_dist =
          error_lat_dist - adc_speed * kLatSafeBufferSpeedCoff - kSafeBuffer;
    } else {
      lon_dis = lon_dis - (back_to_center_);
    }
  }
  error_lat_dist = error_lat_dist < 0.0 ? 0.0 : error_lat_dist;

  if (lon_dis < 0.0) {
    frenet_obs_v = std::min(
        (teb_max_speed_limit_ * error_lat_dist / flu_up_dist_threshold_on_ego_),
        teb_max_speed_limit_);
  } else if (lon_dis < kSlowDownDisThr) {
    frenet_obs_v = std::min(
        (teb_max_speed_limit_ * error_lat_dist / flu_up_dist_threshold_on_ego_),
        teb_max_speed_limit_);
    frenet_obs_v = frenet_obs_v > 0.0
                       ? (frenet_obs_v +
                          (teb_max_speed_limit_ * lon_dis / kSlowDownDisThr))
                       : 0.0;

    frenet_obs_v = std::min(frenet_obs_v, teb_max_speed_limit_);
  } else {
    frenet_obs_v = teb_max_speed_limit_;
    *(is_need_speed_limit) = false;
  }
  // AINFO << "v: " << frenet_obs_v << " | adc_v: " << adc_speed
  //       << " | lon_dis: " << lon_dis << " | lat_dis: " << error_lat_dist;
  return frenet_obs_v;
}

double TEBTrajectoryPartition::CalTraceCurveSpeed(
    century::planning::DiscretizedTrajectory* trajectory, const size_t& i) {
  double trajectory_point_kappa =
      std::fabs(trajectory->at(i).path_point().kappa());
  double curve_limited_v = 0.0;
  if (trajectory_point_kappa > teb_max_curve_thr_) {
    curve_limited_v = teb_max_curve_speed_limit_;
  } else {
    curve_limited_v = teb_max_speed_limit_ -
                      (teb_max_speed_limit_ - teb_max_curve_speed_limit_) *
                          (trajectory_point_kappa - teb_min_curve_thr_) /
                          (teb_max_curve_thr_ - teb_min_curve_thr_);
  }
  return curve_limited_v;
}

double TEBTrajectoryPartition::CalNearEndDisSpeed(
    century::planning::DiscretizedTrajectory* trajectory, const size_t& i,
    const double& s_shift) {
  double dis = 0.0;
  if (trajectory->size() - 1 == i) {
    dis = std::fabs(trajectory->back().path_point().s() -
                    trajectory->at(i).path_point().s());
  } else {
    dis = std::fabs((trajectory->back().path_point().s() - s_shift) -
                    trajectory->at(i).path_point().s());
  }
  double dis_limited_v = 0.0;
  if (dis < teb_trajectory_near_end_) {
    dis_limited_v =
        std::max(dis / teb_trajectory_near_end_ * teb_max_speed_limit_,
                 FLAGS_teb_min_front_speed);
  } else {
    dis_limited_v = teb_max_speed_limit_;
  }
  return dis_limited_v;
}

double TEBTrajectoryPartition::GetFluObsToVehicleSpeed(
    const common::math::Vec2d& vehicle_point, const double& vehicle_heading) {
  Box2d ego_box(vehicle_point, vehicle_heading, ego_length_, ego_width_);
  Vec2d shift_vec{shift_distance_ * std::cos(vehicle_heading),
                  shift_distance_ * std::sin(vehicle_heading)};
  ego_box.Shift(shift_vec);

  double roi_obs_min_dist = kLargeDist;
  double near_max_speed = teb_max_speed_limit_ * kNearCoeff;
  double roi_obs_speed = near_max_speed;
  double roi_dist = kLargeDist;
  for (const auto& obstacle : frame_->open_space_roi_obstacles()) {
    common::math::Vec2d obs_point =
        Vec2d(obstacle->Perception().position().x(),
              obstacle->Perception().position().y());
    double car2obs_dis = vehicle_point.DistanceTo(obs_point);
    if (car2obs_dis > FLAGS_cal_speed_filter_too_far_obs) {
      continue;
    }
    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        FLAGS_enable_openspace_use_polygon_plan) {
      roi_dist = obstacle->PerceptionPolygon().DistanceTo(ego_box);
    } else {
      roi_dist = obstacle->PerceptionBoundingBox().DistanceTo(ego_box);
    }
    if (roi_dist < roi_obs_min_dist) {
      roi_obs_min_dist = roi_dist;
    }
    if (roi_obs_min_dist < kSafeBuffer) {
      return 0.0;
    }
  }
  roi_obs_min_dist = std::max((roi_obs_min_dist), 0.0);
  roi_obs_speed = std::min(
      (near_max_speed * roi_obs_min_dist / flu_up_dist_threshold_on_ego_),
      near_max_speed);

  double costmap_obs_min_dist = kLargeDist;
  double costmap_obs_speed = near_max_speed;
  double costmap_dist = kLargeDist;
  if (FLAGS_enable_use_costmap) {
    for (const auto polygon : frame_->costmap_polygons()) {
      costmap_dist = polygon.DistanceTo(ego_box);
      if (costmap_dist < costmap_obs_min_dist) {
        costmap_obs_min_dist = costmap_dist;
      }
      if (costmap_obs_min_dist < kSafeBuffer) {
        return 0.0;
      }
    }
    costmap_obs_min_dist = std::max((costmap_obs_min_dist), 0.0);
    costmap_obs_speed = std::min(
        (near_max_speed * costmap_obs_min_dist / flu_up_dist_threshold_on_ego_),
        near_max_speed);
  }

  return std::min(roi_obs_speed, costmap_obs_speed);
}

void TEBTrajectoryPartition::InterpolateCurrentTrajectory(
    const double step, DiscretizedTrajectory* interpolated_trajectory_ptr,
    DiscretizedTrajectory** current_trajectory_ptr) {
  DiscretizedTrajectory current_trajectory = **current_trajectory_ptr;
  CHECK_GT(current_trajectory.size(), 0U);
  interpolated_trajectory_ptr->clear();
  interpolated_trajectory_ptr->shrink_to_fit();
  size_t trajectory_intervals_num = current_trajectory.size() - 1;
  TrajectoryPoint tp;
  PathPoint* path_point = tp.mutable_path_point();
  for (size_t i = 0; i < trajectory_intervals_num; ++i) {
    const auto& start_trajectory_point = current_trajectory[i];
    const auto& end_trajectory_point = current_trajectory[i + 1];
    const auto& start_path_point = current_trajectory[i].path_point();
    const auto& end_path_point = current_trajectory[i + 1].path_point();
    const double dist = std::abs(end_path_point.s() - start_path_point.s());
    if (dist <= step) {
      interpolated_trajectory_ptr->emplace_back(current_trajectory[i]);
      continue;
    }
    double dx = (end_path_point.x() - start_path_point.x()) / dist;
    double dy = (end_path_point.y() - start_path_point.y()) / dist;
    double dtheta =
        (NormalizeAngle(end_path_point.theta() - start_path_point.theta())) /
        dist;
    double dkappa = (end_path_point.kappa() - start_path_point.kappa()) / dist;
    double ddkappa =
        (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist;
    double dddkappa =
        (end_path_point.ddkappa() - start_path_point.ddkappa()) / dist;
    double ds = (end_path_point.s() - start_path_point.s()) / dist;

    double dv = (end_trajectory_point.v() - start_trajectory_point.v()) / dist;
    double da = (end_trajectory_point.a() - start_trajectory_point.a()) / dist;
    double dt = (end_trajectory_point.relative_time() -
                 start_trajectory_point.relative_time()) /
                dist;
    double dsteer =
        (end_trajectory_point.steer() - start_trajectory_point.steer()) / dist;

    for (double d = 0; d < dist; d += step) {
      if (std::fabs((start_path_point.s() + ds * d) - end_path_point.s()) <=
          step * 0.5) {
        continue;
      }
      tp.set_v(start_trajectory_point.v() + dv * d);
      tp.set_a(start_trajectory_point.a() + da * d);
      tp.set_relative_time(start_trajectory_point.relative_time() +
                           dt * (static_cast<double>(d)));
      tp.set_steer(start_trajectory_point.steer() + dsteer * d);

      path_point->set_x(start_path_point.x() + dx * d);
      path_point->set_y(start_path_point.y() + dy * d);
      path_point->set_theta(
          NormalizeAngle(start_path_point.theta() + dtheta * d));
      path_point->set_kappa(start_path_point.kappa() + dkappa * d);
      path_point->set_dkappa(start_path_point.dkappa() + ddkappa * d);
      path_point->set_ddkappa(start_path_point.ddkappa() + dddkappa * d);
      path_point->set_s(start_path_point.s() + ds * d);

      interpolated_trajectory_ptr->emplace_back(tp);
    }
  }
  interpolated_trajectory_ptr->emplace_back(current_trajectory.back());
  *current_trajectory_ptr = interpolated_trajectory_ptr;
  current_trajectory.clear();
  current_trajectory.shrink_to_fit();
}

void TEBTrajectoryPartition::BsplineSmooth(const bool is_reverse,
                                           DiscretizedTrajectory* trajectory) {
  if (!FLAGS_enable_bspline_smooth) {
    ADEBUG << " FLAGS_enable_bspline_smooth false,return";
    return;
  }
  std::vector<BSPoint> control_points;
  size_t trajectory_size = trajectory->size();
  AINFO << "start_c_id_ " << start_c_id_ << " end_c_id_ " << end_c_id_;

  size_t step = end_c_id_ - start_c_id_ > (kBsplineOrder + 1) ? kSmoothStep : 1;
  for (size_t i = start_c_id_; i < end_c_id_;) {
    if (std::fabs(trajectory->at(i).path_point().kappa()) > kChangeKappa) {
      control_points.emplace_back(BSPoint{trajectory->at(i).path_point().x(),
                                          trajectory->at(i).path_point().y()});
      i += step;
    } else {
      ++i;
    }
  }
  control_points.emplace_back(
      BSPoint{trajectory->at(end_c_id_).path_point().x(),
              trajectory->at(end_c_id_).path_point().y()});

  AINFO << "control_points size" << control_points.size();

  if (control_points.size() > kBsplineOrder) {
    Bspline bsplineTrack(kBsplineOrder, quniform, control_points);
    auto res = bsplineTrack.creatBspline();
    AINFO << "res.size(); " << res.size();

    DiscretizedTrajectory temp;
    bool is_inserted = false;
    double start_time = trajectory->at(start_c_id_).relative_time();
    double start_s = trajectory->at(start_c_id_).path_point().s();
    double total_s = trajectory->at(end_c_id_).path_point().s() - start_s;
    double total_t = trajectory->at(end_c_id_).relative_time() - start_time;

    for (size_t i = 0; i < trajectory_size; ++i) {
      if (i < start_c_id_ || i >= end_c_id_) {
        temp.emplace_back(trajectory->at(i));
      } else {
        if (!is_inserted) {
          TrajectoryPoint tp = trajectory->at(i);
          double yaw = 0;
          for (size_t j = 0; j + 1 < res.size(); ++j) {
            tp.mutable_path_point()->set_x(res[j].x);
            tp.mutable_path_point()->set_y(res[j].y);
            tp.set_relative_time(start_time + total_t * j / res.size());
            tp.mutable_path_point()->set_s(start_s + total_s * j / res.size());
            double dy = res[j + 1].y - res[j].y;
            double dx = res[j + 1].x - res[j].x;
            yaw = is_reverse ? NormalizeAngle(std::atan2(dy, dx) + M_PI)
                             : std::atan2(dy, dx);
            double s = std::sqrt(dx * dx + dy * dy);
            ADEBUG << " ss = " << s << "yyaw " << yaw << " j " << j;
            tp.mutable_path_point()->set_theta(yaw);
            temp.emplace_back(tp);
          }
        }
        is_inserted = true;
      }
    }

    *trajectory = temp;
    openspace_common_->CalTheta(is_reverse, trajectory);
    CalCurve(is_reverse, trajectory);
  }
}

void TEBTrajectoryPartition::FillTrajectoryTime(
    const double& start_time, const double& end_time,
    DiscretizedTrajectory* trajectory) {
  if (trajectory->empty()) {
    return;
  }
  double start_s = (*trajectory)[0].path_point().s();
  double end_s = (*trajectory)[trajectory->size() - 1].path_point().s();

  bool trace_s_is_zero = std::fabs(end_s - start_s) < 1e-6 ? true : false;

  for (size_t i = 0; i < trajectory->size(); ++i) {
    if (!trace_s_is_zero) {
      double current_point_s = trajectory->at(i).path_point().s();
      double current_point_time = start_time + (end_time - start_time) *
                                                   (current_point_s - start_s) /
                                                   (end_s - start_s);
      TrajectoryPoint* trajectory_point = &(trajectory->at(i));
      trajectory_point->set_relative_time(current_point_time);
    } else {
      TrajectoryPoint* trajectory_point = &(trajectory->at(i));
      trajectory_point->set_relative_time(start_time);
    }
  }
}

void TEBTrajectoryPartition::PullOverRoutingEndExtendLength(
    const size_t& current_trajectory_index,
    const size_t& partitioned_trajectories_size, const bool& is_reverse,
    DiscretizedTrajectory* trajectory) {
  if (injector_->pullover_using_ && !injector_->pullover_finished) {
    if (partitioned_trajectories_size - 1 == current_trajectory_index) {
      injector_->pullover_end_trace_ = true;
      RoutingEndExtendLength(is_reverse, trajectory);
      const common::math::Vec2d adc_pos = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};
      size_t adc_index = trajectory->QueryNearestPoint(adc_pos);
      double adc2pullover_end_s =
          std::fabs(trajectory->back().path_point().s() -
                    trajectory->at(adc_index).path_point().s());
      bool heading_same =
          std::fabs(common::math::NormalizeAngle(
              injector_->vehicle_state()->heading() - pullover_end_heading_)) <
          kPullOverHeadingThr;
      if ((adc2pullover_end_s <= kRoutingEndExtendLength && heading_same) ||
          adc2pullover_end_s <= kPullOverReachedDisThr) {
        frame_->mutable_open_space_info()->set_destination_reached(true);
      } else {
        frame_->mutable_open_space_info()->set_destination_reached(false);
      }
    } else {
      injector_->pullover_end_trace_ = false;
    }
  }
  return;
}

void TEBTrajectoryPartition::RoutingEndExtendLength(
    const bool is_reverse, DiscretizedTrajectory* trajectory) {
  if (trajectory->size() < 2) {
    AERROR << "end traj size < 2, return.";
    return;
  }
  const auto& reference_line_info = frame_->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  common::math::Vec2d end_point_xy = {trajectory->back().path_point().x(),
                                      trajectory->back().path_point().y()};
  common::SLPoint end_point_sl;
  reference_line.XYToSL(end_point_xy, &end_point_sl);
  // AINFO << "end_point_sl.l: " << end_point_sl.l();

  auto ref_point = reference_line.GetNearestReferencePoint(end_point_sl.s());
  pullover_end_heading_ = ref_point.heading();

  double trace_dy = trajectory->back().path_point().y() -
                    trajectory->at(trajectory->size() - 2).path_point().y();
  double trace_dx = trajectory->back().path_point().x() -
                    trajectory->at(trajectory->size() - 2).path_point().x();
  bool heading_same =
      std::fabs(common::math::NormalizeAngle(std::atan2(trace_dy, trace_dx) -
                                             ref_point.heading())) < (M_PI_2);
  // AINFO << "heading_same: " << heading_same;

  double start = end_point_sl.s();
  double end = heading_same ? end_point_sl.s() + kRoutingEndExtendLength
                            : end_point_sl.s() - kRoutingEndExtendLength;
  double dir = heading_same ? 1.0 : -1.0;
  double step_s = dir * kStepDistance;

  double gear_dir = is_reverse ? -1.0 : 1.0;

  common::SLPoint sl_point_temp = end_point_sl;
  common::math::Vec2d xy_point_temp;
  TrajectoryPoint trajectory_point_temp;
  double relative_time_temp =
      trajectory->back().relative_time() -
      trajectory->at(trajectory->size() - 2).relative_time();
  // AINFO << "relative_time_temp: " << relative_time_temp;

  for (double i = start + step_s; (heading_same) ? (i < end) : (i > end);
       i += step_s) {
    sl_point_temp.set_s(i);
    sl_point_temp.set_l(end_point_sl.l());
    reference_line.SLToXY(sl_point_temp, &xy_point_temp);
    auto ref_point_temp = reference_line.GetNearestReferencePoint(i);
    trajectory_point_temp.mutable_path_point()->set_x(xy_point_temp.x());
    trajectory_point_temp.mutable_path_point()->set_y(xy_point_temp.y());
    trajectory_point_temp.mutable_path_point()->set_theta(
        ref_point_temp.heading());
    trajectory_point_temp.mutable_path_point()->set_kappa(
        ref_point_temp.kappa());

    trajectory_point_temp.mutable_path_point()->set_s(
        trajectory->back().path_point().s() + gear_dir * kStepDistance);
    trajectory_point_temp.set_relative_time(trajectory->back().relative_time() +
                                            relative_time_temp);
    trajectory->emplace_back(trajectory_point_temp);
  }
  return;
}

void TEBTrajectoryPartition::TraceInterpolatePolyfitSmooth(
    const bool& is_reverse, DiscretizedTrajectory* trajectory) {
  bool is_stop_trace = JudgeStopTrace(trajectory);
  AINFO << "is_stop_trace:" << is_stop_trace;
  if (!is_stop_trace) {
    auto open_space_info_ptr = frame_->mutable_open_space_info();
    auto current_trajectory = trajectory;
    if (nullptr == current_trajectory) {
      AERROR << "current_trajectory is empty";
      return;
    }
    if (current_trajectory->size() >= kPolyfitCoefficient + 1) {
      auto* polyfit_trajectory_ptr =
          open_space_info_ptr->mutable_interpolated_trajectory_result();
      if (!openspace_util_->PolyfitTrajectory(is_reverse, kInterpolatePitch_S,
                                              polyfit_trajectory_ptr,
                                              &current_trajectory)) {
        AERROR << "bspline Polyfit_Trace_Failed, use polyfit";
        if (!PolyfitTrajectory(is_reverse, kInterpolatePitch_S,
                               polyfit_trajectory_ptr, &current_trajectory)) {
          AERROR << "Polyfit_Trace_Failed, return.";
          return;
        }
      }
      AINFO << "Only Polyfit CurrentTrajectory_Size: "
            << current_trajectory->size();
      *trajectory = *current_trajectory;
      AINFO << "transmit after trajectory size " << trajectory->size();
    } else {
      // BsplineSmooth(is_reverse, trajectory);
      auto* interpolated_trajectory_ptr =
          open_space_info_ptr->mutable_interpolated_trajectory_result();
      InterpolateCurrentTrajectory(kInterpolatePitch_S,
                                   interpolated_trajectory_ptr,
                                   &current_trajectory);
      auto* polyfit_trajectory_ptr =
          open_space_info_ptr->mutable_interpolated_trajectory_result();
      if (!PolyfitTrajectory(is_reverse, kInterpolatePitch_S,
                             polyfit_trajectory_ptr, &current_trajectory)) {
        AERROR << "Polyfit_Trace_Failed";
        return;
      }
      AINFO << "Interpolate Polyfit CurrentTrajectory_Size: "
            << current_trajectory->size();
      *trajectory = *current_trajectory;
      AINFO << "transmit after trajectory size " << trajectory->size();
    }
    for (size_t i = 0; i < trajectory->size(); ++i) {
      ADEBUG << "Polyfit after"
             << " x=" << std::setprecision(10)
             << (*trajectory)[i].path_point().x()
             << " y=" << std::setprecision(10)
             << (*trajectory)[i].path_point().y();
    }
  }
  return;
}

bool TEBTrajectoryPartition::JudgeStopTrace(
    const DiscretizedTrajectory* trajectory) {
  if (trajectory->size() < 1U) {
    return true;
  }
  if (!(Vec2d(trajectory->front().path_point().x(),
              trajectory->front().path_point().y()) ==
        Vec2d(trajectory->back().path_point().x(),
              trajectory->back().path_point().y()))) {
    AERROR << "The starting point and ending point of the trajectory are "
              "inconsistent, not stop trajectory! return false";
    return false;
  }
  Vec2d first_point = Vec2d(trajectory->front().path_point().x(),
                            trajectory->front().path_point().y());
  size_t cnt = 0;
  for (size_t i = 0; i < trajectory->size(); ++i) {
    if (first_point == Vec2d(trajectory->at(i).path_point().x(),
                             trajectory->at(i).path_point().y())) {
      cnt++;
    }
    if (cnt < i) {
      return false;
    }
  }
  if (trajectory->size() == cnt) {
    AINFO << "cnt == traj_size, is stop trajectory!";
    return true;
  }
  return false;
}

void TEBTrajectoryPartition::CalCurve(const bool is_reverse,
                                      DiscretizedTrajectory* trajectory) {
  if (trajectory->empty()) {
    return;
  }
  size_t trajectory_size = trajectory->size();
  if (trajectory_size > 0 && trajectory_size < 3) {
    for (size_t i = 0; i < trajectory_size; ++i) {
      trajectory->at(i).mutable_path_point()->set_kappa(0.0);
    }
  } else {
    double origin_x = 0.0;
    double origin_y = 0.0;
    for (size_t i = 1; i < trajectory_size; ++i) {
      if (i + 1 < trajectory_size) {
        origin_x = trajectory->at(i).mutable_path_point()->x();
        origin_y = trajectory->at(i).mutable_path_point()->y();
        CirclePoint a = CirclePoint(
            trajectory->at(i - 1).mutable_path_point()->x() - origin_x,
            trajectory->at(i - 1).mutable_path_point()->y() - origin_y);

        CirclePoint b = CirclePoint(0.0, 0.0);

        CirclePoint c = CirclePoint(
            trajectory->at(i + 1).mutable_path_point()->x() - origin_x,
            trajectory->at(i + 1).mutable_path_point()->y() - origin_y);

        Circle circle = Circle(a, b, c);
        // AINFO << "circle.radius: " << circle.radius();
        double kappa = circle.radius() > 0.0 ? 1.0 / circle.radius() : 0.0;

        // direction
        Vec2d ab = Vec2d(trajectory->at(i).mutable_path_point()->x() -
                             trajectory->at(i - 1).mutable_path_point()->x(),
                         trajectory->at(i).mutable_path_point()->y() -
                             trajectory->at(i - 1).mutable_path_point()->y());
        Vec2d ac = Vec2d(trajectory->at(i + 1).mutable_path_point()->x() -
                             trajectory->at(i - 1).mutable_path_point()->x(),
                         trajectory->at(i + 1).mutable_path_point()->y() -
                             trajectory->at(i - 1).mutable_path_point()->y());
        bool left_circle = ab.CrossProd(ac) > 0;
        // AINFO << "left_circle: " << left_circle;
        kappa = left_circle ? kappa : -kappa;
        // AINFO << "kappa: " << kappa;
        kappa = is_reverse ? -kappa : kappa;

        kappa = std::min(std::max(kappa, -kMaxKappa), kMaxKappa);
        if (std::fabs(kappa) <= kEpsilon) {
          kappa = 0.0;
        }
        trajectory->at(i).mutable_path_point()->set_kappa(kappa);
      } else {
        trajectory->at(i).mutable_path_point()->set_kappa(
            trajectory->at(i - 1).path_point().kappa());
      }
    }
    trajectory->at(0).mutable_path_point()->set_kappa(
        trajectory->at(1).path_point().kappa());
  }
  return;
}

void TEBTrajectoryPartition::CalTheta(const bool is_reverse,
                                      DiscretizedTrajectory* trajectory) {
  AINFO << __func__ << ", is_reverse: " << is_reverse;
  size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    if (i + 1 < trajectory_size) {
      double dx = trajectory->at(i + 1).path_point().x() -
                  trajectory->at(i).path_point().x();
      double dy = trajectory->at(i + 1).path_point().y() -
                  trajectory->at(i).path_point().y();

      // this method already consider reverse condition, do not check is reverse
      double theta_yaw = is_reverse ? NormalizeAngle(std::atan2(dy, dx) + M_PI)
                                    : std::atan2(dy, dx);
      trajectory->at(i).mutable_path_point()->set_theta(theta_yaw);
    } else if (trajectory->size() > 1) {
      trajectory->at(i).mutable_path_point()->set_theta(
          trajectory->at(i - 1).path_point().theta());
    } else {
      trajectory->at(i).mutable_path_point()->set_theta(0.0);
    }
  }
  return;
}

}  // namespace planning
}  // namespace century
