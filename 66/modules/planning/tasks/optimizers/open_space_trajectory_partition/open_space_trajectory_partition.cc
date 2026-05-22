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
#include "modules/planning/tasks/optimizers/open_space_trajectory_partition/open_space_trajectory_partition.h"

#include <memory>
#include <queue>

#include "absl/strings/str_cat.h"

#include "cyber/time/clock.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/planning_context.h"

namespace century {
namespace planning {

namespace {
constexpr double kSmoothPercent = 0.4;
constexpr int kIdDistToEnd = 6;
constexpr double kBackSpeed = -0.6;
constexpr double kBackReachS = -0.25;
constexpr double kMinSpeed = 0.35;
constexpr double kReachS = 0.35;
constexpr double kEpsilon = 0.001;
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

// constructor
OpenSpaceTrajectoryPartition::OpenSpaceTrajectoryPartition(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : TrajectoryOptimizer(config, injector) {
  // configure
  open_space_trajectory_partition_config_ =
      config_.open_space_trajectory_partition_config();
  heading_search_range_ =
      open_space_trajectory_partition_config_.heading_search_range();
  heading_track_range_ =
      open_space_trajectory_partition_config_.heading_track_range();
  distance_search_range_ =
      open_space_trajectory_partition_config_.distance_search_range();
  heading_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.heading_offset_to_midpoint();
  lateral_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.lateral_offset_to_midpoint();
  longitudinal_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.longitudinal_offset_to_midpoint();
  vehicle_box_iou_threshold_to_midpoint_ =
      open_space_trajectory_partition_config_
          .vehicle_box_iou_threshold_to_midpoint();
  linear_velocity_threshold_on_ego_ = open_space_trajectory_partition_config_
                                          .linear_velocity_threshold_on_ego();

  // vehicle parameter
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  // vehicle length
  ego_length_ = vehicle_param_.length();
  // vehicle width
  ego_width_ = vehicle_param_.width();
  // offset between planning center and geometric center
  shift_distance_ = ego_length_ * 0.5 - vehicle_param_.back_edge_to_center();
  // vehicle base
  wheel_base_ = vehicle_param_.wheel_base();
}

void OpenSpaceTrajectoryPartition::Restart() {
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  current_gear_status->gear_switching_flag = false;
  current_gear_status->gear_shift_period_finished = true;
  current_gear_status->gear_shift_period_started = true;
  current_gear_status->gear_shift_period_time = 0.0;
  current_gear_status->gear_shift_start_time = 0.0;
  current_gear_status->gear_shift_position = canbus::Chassis::GEAR_DRIVE;
}

// process function
Status OpenSpaceTrajectoryPartition::Process() {
  ADEBUG << "OpenSpaceTrajectoryPartition Process";
  // open space information
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();

  // planned trajectory result
  const auto& stitched_trajectory_result =
      open_space_info.stitched_trajectory_result();

  // interpolate planned trajectory result
  auto* interpolated_trajectory_result_ptr =
      open_space_info_ptr->mutable_interpolated_trajectory_result();
  InterpolateTrajectory(stitched_trajectory_result,
                        interpolated_trajectory_result_ptr);

  // partite planned trajectory
  auto* partitioned_trajectories =
      open_space_info_ptr->mutable_partitioned_trajectories();
  PartitionTrajectory(*interpolated_trajectory_result_ptr,
                      partitioned_trajectories);

  // update vehicle state
  UpdateVehicleInfo();

  // open space information
  //const auto& open_space_status =
  //    injector_->planning_context()->planning_status().open_space();

  // first trajectory
  //auto current_partitioned_trajectory = &(partitioned_trajectories->at(0));
  // segment trajectories size
  size_t trajectories_size = partitioned_trajectories->size();
  ADEBUG << "partitioned trajectories size " << trajectories_size;

  // trajectory encoding
  std::vector<std::string> trajectories_encodings;    // first_point + last_point
  for (size_t i = 0; i < trajectories_size; ++i) {
    // segment trajectory i
    const auto& trajectory = partitioned_trajectories->at(i).first;
    std::string trajectory_encoding;
    // encode trajectory
    if (!EncodeTrajectory(trajectory, &trajectory_encoding)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Trajectory empty in trajectory partition");
    }
    AINFO << "encoding " << i << '\t' << trajectory_encoding << '\t'
          << "gear " << partitioned_trajectories->at(i).second;
    trajectories_encodings.emplace_back(std::move(trajectory_encoding));
  }

/*
  // plan succeed and position not init
  if (!open_space_status.position_init() &&
      frame_->open_space_info().open_space_provider_success()) {
    // open space context
    auto* open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();

    // chosen trajectory
    auto* chosen_partitioned_trajectory =
        open_space_info_ptr->mutable_chosen_partitioned_trajectory();

    // planned trajectory
    auto* mutable_trajectory =
        open_space_info_ptr->mutable_stitched_trajectory_result();

    // first trajectory
    auto trajectory = &(current_partitioned_trajectory->first);
    // first trajectory points size
    const int trajectory_size = trajectory->size();

    // current vehicle postiion
    const Vec2d adc_position = {injector_->vehicle_state()->x(),
                                injector_->vehicle_state()->y()};

    // nearest index
    int current_index = trajectory->QueryNearestPoint(adc_position);
    // check finished trajectory 0
    if (CheckFinishInitPosition(open_space_info.partitioned_trajectories(),
                                chosen_partitioned_trajectory)) {
      open_space_status->set_position_init(true);
      if (trajectories_size > 1) {
        // add trajectories_encodings[0] into partitioned_trajectories_index_history
        UpdateTrajHistory(trajectories_encodings[0]);
      }
    }

    // update current_index
    current_index =
        std::max(std::min(current_index + 1, trajectory_size - 1), 0);

    // update partitioned trajectories relative_time and s
    // update planned trajectory relative_time and s
    // get chosen_partitioned_trajectory
    AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(), 0,
                           current_index, mutable_trajectory,
                           chosen_partitioned_trajectory);
    return Status::OK();
  }
*/

  // Choose the one to follow based on the closest partitioned trajectory
  size_t current_trajectory_index = 0;
  size_t current_trajectory_point_index = 0;
  bool flag_change_to_next = false;

  // partitioned trajectories closest point index
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      closest_point_on_trajs;

  for (size_t i = 0; i < trajectories_size; ++i) {
    // segment trajectory i gear
    const auto& gear = partitioned_trajectories->at(i).second;
    // segment trajectory i
    const auto& trajectory = partitioned_trajectories->at(i).first;
    // trajectory point size
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0U);

    // check vehicle reached trajectory last point,
    // update current_trajectory_index, current_trajectory_point_index
    flag_change_to_next = CheckReachTrajectoryEnd(
        trajectory, gear, trajectories_size, i, &current_trajectory_index,
        &current_trajectory_point_index);
    ADEBUG << "trajectories " << i << '\t' << trajectories_encodings[i] << '\t'
           << "flag_change_to_next " << flag_change_to_next;
    if (flag_change_to_next &&
        !CheckTrajTraversed(trajectories_encodings[i])) {
      // add trajectories_encodings[i] into partitioned_trajectories_index_history$
      UpdateTrajHistory(trajectories_encodings[i]);
      break;
    }

    if (flag_change_to_next) {
      continue;
    }

    // Choose the closest point to track
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        closest_point;
    for (size_t j = 0; j < trajectory_size; ++j) {
      // trajectory point j
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      //
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      // distance
      const double distance = tracking_vector.Length();
      //
      const double tracking_direction = tracking_vector.Angle();
      // trajectory point j moving heading
      const double traj_point_moving_direction =
          gear == canbus::Chassis::GEAR_REVERSE
              ? NormalizeAngle(path_point_theta + M_PI)
              : path_point_theta;

      // the different between vehicle moving direction and to heading direction
      const double head_track_difference = std::abs(
          NormalizeAngle(tracking_direction - vehicle_moving_direction_));
      // the different between vehicle moving direction and trajectory point j direction
      const double heading_search_difference = std::abs(NormalizeAngle(
          traj_point_moving_direction - vehicle_moving_direction_));

      ADEBUG << "distance " << distance << '\t'
             << longitudinal_offset_to_midpoint_ * 2.0;
      ADEBUG << "heading_search_difference " << heading_search_difference << '\t'
             << heading_search_range_;
      ADEBUG << "heading_track_difference " << head_track_difference << '\t'
             << heading_track_range_;

      if (distance < longitudinal_offset_to_midpoint_ * 2.0 &&
          heading_search_difference < heading_search_range_ &&
          head_track_difference < heading_track_range_) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        // iou
        closest_point.emplace(j, iou_ratio);
      }
    }
    ADEBUG << "closest point size " << closest_point.size();
    if (!closest_point.empty()) {
      size_t closest_point_index = closest_point.top().first;
      double max_iou_ratio = closest_point.top().second;
      closest_point_on_trajs.emplace(std::make_pair(i, closest_point_index),
                                     max_iou_ratio);
    }
  }

  if (!flag_change_to_next) {                     // not in segment trajectory end
    bool use_fail_safe_search = false;
    if (closest_point_on_trajs.empty()) {
      use_fail_safe_search = true;
    } else {
      bool closest_and_not_repeated_traj_found = false;
      while (!closest_point_on_trajs.empty()) {
        // update current_trajectory_index
        current_trajectory_index = closest_point_on_trajs.top().first.first;
        // update current_trajectory_point_index
        current_trajectory_point_index =
            closest_point_on_trajs.top().first.second;
        if (CheckTrajTraversed(
                trajectories_encodings[current_trajectory_index])) {
          closest_point_on_trajs.pop();
        } else {
          closest_and_not_repeated_traj_found = true;
          break;
        }
      }
      if (!closest_and_not_repeated_traj_found) {
        use_fail_safe_search = true;
      }
    }

    // failed to find nearest trajectory and nearest point
    if (use_fail_safe_search) {
      if (!UseFailSafeSearch(*partitioned_trajectories, trajectories_encodings,
                             &current_trajectory_index,
                             &current_trajectory_point_index)) {
        const std::string msg =
            "Fail to find nearest trajectory point to follow";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
  }

  ADEBUG << "current trajectory index " << current_trajectory_index;
  ADEBUG << "current trajectory point index " << current_trajectory_point_index;

  // chosen trajectory
  auto* chosen_partitioned_trajectory =
      open_space_info_ptr->mutable_chosen_partitioned_trajectory();

  // FLAGS_use_gear_shift_trajectory = false
  if (FLAGS_use_gear_shift_trajectory) {
    // chosen trajectory
    auto trajectory = &(chosen_partitioned_trajectory->first);
    if (InsertGearShiftTrajectory(flag_change_to_next, current_trajectory_index,
                                  open_space_info.partitioned_trajectories(),
                                  chosen_partitioned_trajectory) &&
        chosen_partitioned_trajectory->first.size() != 0) {
      trajectory = &(chosen_partitioned_trajectory->first);
      ADEBUG << "After InsertGearShiftTrajectory [" << trajectory->size()
             << "]";
      return Status::OK();
    }
  }

  // planned trajectory
  auto* mutable_trajectory =
      open_space_info_ptr->mutable_stitched_trajectory_result();

  // update partitioned trajectories relative_time and s
  // update planned trajectory relative_time and s
  // get chosen_partitioned_trajectory
  AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(),
                         current_trajectory_index,
                         current_trajectory_point_index, mutable_trajectory,
                         chosen_partitioned_trajectory);

  return Status::OK();
}

// interpolate planned trajectory
void OpenSpaceTrajectoryPartition::InterpolateTrajectory(
    const DiscretizedTrajectory& stitched_trajectory_result,
    DiscretizedTrajectory* interpolated_trajectory) {
  // no interpolate
  if (FLAGS_use_iterative_anchoring_smoother) {
    *interpolated_trajectory = stitched_trajectory_result;
    return;
  }

  // clear interpolated_trajectory
  interpolated_trajectory->clear();
  // the number of interpolated points between adjacent trajectory points
  size_t interpolated_pieces_num =
      open_space_trajectory_partition_config_.interpolated_pieces_num();
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  CHECK_GT(stitched_trajectory_result.size(), 0U);
  CHECK_GT(interpolated_pieces_num, 0U);
  // planned trajectory max index
  size_t trajectory_to_be_partitioned_intervals_num =
      stitched_trajectory_result.size() - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    // time duration
    double relative_time_interval =
        (stitched_trajectory_result.at(i + 1).relative_time() -
         stitched_trajectory_result.at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    // save stitched_trajectory_result[i]
    interpolated_trajectory->push_back(stitched_trajectory_result.at(i));
    // interpolate stitched_trajectory_result[i] and stitched_trajectory_result[i+1]
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          stitched_trajectory_result.at(i).relative_time() +
          (static_cast<double>(j) + 1.0) * relative_time_interval;
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              stitched_trajectory_result.at(i),
              stitched_trajectory_result.at(i + 1), relative_time));
    }
  }
  interpolated_trajectory->push_back(stitched_trajectory_result.back());
}

// update vehicle state
void OpenSpaceTrajectoryPartition::UpdateVehicleInfo() {
  // vehicle state
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  // current vehicle heading
  ego_theta_ = vehicle_state.heading();
  // current vehicle position
  ego_x_ = vehicle_state.x();
  ego_y_ = vehicle_state.y();
  // current vehicle velocity
  ego_v_ = vehicle_state.linear_velocity();
  // current vehicle bounding box
  Box2d box({ego_x_, ego_y_}, ego_theta_, ego_length_, ego_width_);
  ego_box_ = std::move(box);
  Vec2d ego_shift_vec{shift_distance_ * std::cos(ego_theta_),
                      shift_distance_ * std::sin(ego_theta_)};
  ego_box_.Shift(ego_shift_vec);
  // current vehicle moving heading
  vehicle_moving_direction_ =
      vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(ego_theta_ + M_PI)
          : ego_theta_;
}

// encode trajectory first point and last point
bool OpenSpaceTrajectoryPartition::EncodeTrajectory(
    const DiscretizedTrajectory& trajectory, std::string* const encoding) {
  // check trajectory is empty
  if (trajectory.empty()) {
    AERROR << "Fail to encode trajectory because it is empty";
    return false;
  }

  // first point
  const auto& init_path_point = trajectory.front().path_point();
  // last point
  const auto& last_path_point = trajectory.back().path_point();

  const int init_point_x =
      static_cast<int>(init_path_point.x() * 1000.0);
  const int init_point_y =
      static_cast<int>(init_path_point.y() * 1000.0);
  const int init_point_heading =
      static_cast<int>(init_path_point.theta() * 10000.0);
  const int last_point_x =
      static_cast<int>(last_path_point.x() * 1000.0);
  const int last_point_y =
      static_cast<int>(last_path_point.y() * 1000.0);
  const int last_point_heading =
      static_cast<int>(last_path_point.theta() * 10000.0);

  *encoding = absl::StrCat(
      // init point
      init_point_x, "_", init_point_y, "_", init_point_heading, "/",
      // last point
      last_point_x, "_", last_point_y, "_", last_point_heading);
  return true;
}

// check trajectory_encoding_to_check trajectory finished
bool OpenSpaceTrajectoryPartition::CheckTrajTraversed(
    const std::string& trajectory_encoding_to_check) {
  // open space context
  const auto& open_space_status =
      injector_->planning_context()->planning_status().open_space();
  // finished segment trajectories size
  const int index_history_size =
      open_space_status.partitioned_trajectories_index_history_size();

  ADEBUG << "find encode " << trajectory_encoding_to_check;
  ADEBUG << "traversed trajectory size " << index_history_size;
  for (int i = 0; i < index_history_size; ++i)
    ADEBUG << open_space_status.partitioned_trajectories_index_history(i);

  for (int i = 0; i < index_history_size; i++) {
    // trajectory i encode
    const auto& index_history =
        open_space_status.partitioned_trajectories_index_history(i);
    // trajectory_encoding_to_check trajectory finished
    if (index_history == trajectory_encoding_to_check) {
      ADEBUG << "traversed trajectory, return true";
      return true;
    }
  }
  ADEBUG << "not traversed trajectory, return false";
  return false;
}

// add finished segment trajectory into partitioned_trajectories_index_history
void OpenSpaceTrajectoryPartition::UpdateTrajHistory(
    const std::string& chosen_trajectory_encoding) {
  // open space context
  auto* open_space_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_open_space();

  // traversed trajectories history list
  const auto& trajectory_history =
      injector_->planning_context()
          ->planning_status()
          .open_space()
          .partitioned_trajectories_index_history();
  // add finished trajectory into partitioned_trajectories_index_history
  if (trajectory_history.empty()) {
    open_space_status->add_partitioned_trajectories_index_history(
        chosen_trajectory_encoding);
    return;
  }
  if (*(trajectory_history.rbegin()) == chosen_trajectory_encoding) {
    return;
  }
  ADEBUG << "add trajectory " << chosen_trajectory_encoding;
  open_space_status->add_partitioned_trajectories_index_history(
      chosen_trajectory_encoding);
}

// trajectory segmentation
void OpenSpaceTrajectoryPartition::PartitionTrajectory(
    const DiscretizedTrajectory& raw_trajectory,
    std::vector<TrajGearPair>* partitioned_trajectories) {
  CHECK_NOTNULL(partitioned_trajectories);

  // raw_trajectory size
  size_t horizon = raw_trajectory.size();

  // add a trajectory
  partitioned_trajectories->clear();
  partitioned_trajectories->emplace_back();
  TrajGearPair* current_trajectory_gear = &(partitioned_trajectories->back());

  // trajectory
  auto* trajectory = &(current_trajectory_gear->first);
  // gear position
  auto* gear = &(current_trajectory_gear->second);

  // first trajectory point
  const auto& first_path_point = raw_trajectory.front().path_point();
  // second trajectory point
  const auto& second_path_point = raw_trajectory[1].path_point();
  // first trajectory point heading
  double heading_angle = first_path_point.theta();
  const Vec2d init_tracking_vector(
      second_path_point.x() - first_path_point.x(),
      second_path_point.y() - first_path_point.y());
  // first trajectory point trajectory heading
  double tracking_angle = init_tracking_vector.Angle();
  // first trajectory point gear position
  *gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
              (M_PI_2)
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;

  // init last_ps_vec
  Vec2d last_pos_vec(first_path_point.x(), first_path_point.y());
  double distance_s = 0.0;
  bool is_trajectory_last_point = false;

  for (size_t i = 0; i < horizon - 1; ++i) {
    const TrajectoryPoint& trajectory_point = raw_trajectory.at(i);
    const TrajectoryPoint& next_trajectory_point = raw_trajectory.at(i + 1);

    // Check gear change
    heading_angle = trajectory_point.path_point().theta();
    const Vec2d tracking_vector(next_trajectory_point.path_point().x() -
                                    trajectory_point.path_point().x(),
                                next_trajectory_point.path_point().y() -
                                    trajectory_point.path_point().y());
    tracking_angle = tracking_vector.Angle();
    // trajectory point i gear position
    auto cur_gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2)
            ? canbus::Chassis::GEAR_DRIVE
            : canbus::Chassis::GEAR_REVERSE;

    // gear change
    if (cur_gear != *gear) {
      is_trajectory_last_point = true;
      // insert trajectory_point into trajectory
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

    // insert trajectory_point into trajectory
    LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                        &last_pos_vec, &distance_s, trajectory);
  }
  is_trajectory_last_point = true;
  const TrajectoryPoint& last_trajectory_point = raw_trajectory.back();
  // insert last_trajectory_point into trajectory
  LoadTrajectoryPoint(last_trajectory_point, is_trajectory_last_point, *gear,
                      &last_pos_vec, &distance_s, trajectory);
}

// insert trajectory_point into current_trajectory
void OpenSpaceTrajectoryPartition::LoadTrajectoryPoint(
    const TrajectoryPoint& trajectory_point,
    const bool is_trajectory_last_point,
    const canbus::Chassis::GearPosition& gear, Vec2d* last_pos_vec,
    double* distance_s, DiscretizedTrajectory* current_trajectory) {
  // add a trajectory point
  current_trajectory->emplace_back();
  TrajectoryPoint* point = &(current_trajectory->back());
  // set point attribute
  point->CopyFrom(trajectory_point);
  // update s
  point->mutable_path_point()->set_s(*distance_s);
  Vec2d cur_pos_vec(trajectory_point.path_point().x(),
                    trajectory_point.path_point().y());
  // update distance_s
  *distance_s += (gear == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0) *
                 (cur_pos_vec.DistanceTo(*last_pos_vec));
  // update last_pos_vec
  *last_pos_vec = cur_pos_vec;
  // update kappa
  point->mutable_path_point()->set_kappa((is_trajectory_last_point ? -1 : 1) *
                                         std::tan(trajectory_point.steer()) /
                                         wheel_base_ / 2.0);
}

// check vehicle has reached trajectory last point
// update current_trajectory_index and current_trajectory_point_index
bool OpenSpaceTrajectoryPartition::CheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    const size_t trajectories_index, size_t* current_trajectory_index,
    size_t* current_trajectory_point_index) {
  // last trajectory point
  const TrajectoryPoint& trajectory_end_point = trajectory.back();
  // trajectory points size
  const size_t trajectory_size = trajectory.size();
  const PathPoint& path_end_point = trajectory_end_point.path_point();
  const double path_end_point_x = path_end_point.x();
  const double path_end_point_y = path_end_point.y();
  // distance to last trajectory point
  const Vec2d tracking_vector(ego_x_ - path_end_point_x,
                              ego_y_ - path_end_point_y);
  //last trajectory point heading
  const double path_end_point_theta = path_end_point.theta();
  //
  const double included_angle =
      NormalizeAngle(path_end_point_theta - tracking_vector.Angle());
  // distance to last trajectory point
  const double distance_to_trajs_end = tracking_vector.Length();
  // lateral offset
  const double lateral_offset =
      std::abs(distance_to_trajs_end * std::sin(included_angle));
  // longitudinal offset
  const double longitudinal_offset =
      std::abs(distance_to_trajs_end * std::cos(included_angle));
  // last trajectory point heading
  const double traj_end_point_moving_direction =
      gear == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(path_end_point_theta + M_PI)
          : path_end_point_theta;

  // moving direction error
  const double heading_search_to_trajs_end = std::abs(NormalizeAngle(
      traj_end_point_moving_direction - vehicle_moving_direction_));

  ADEBUG << "heading_search_to_trajs_end" << heading_search_to_trajs_end;

  ADEBUG << __func__ << '\t'
         << "lateral_offset " << lateral_offset << '\t'
         << lateral_offset_to_midpoint_ << '\t'
         << "longitudinal_offset " << longitudinal_offset << '\t'
         << longitudinal_offset_to_midpoint_ << '\t'
         << "ego_v " << ego_v_ << '\t' << vehicle_param_.max_abs_speed_when_stopped();

  // If close to the end point, start on the next trajectory
  if (lateral_offset < lateral_offset_to_midpoint_ &&
      longitudinal_offset < longitudinal_offset_to_midpoint_ &&
      std::abs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped()) {
    // update current_trajectory_index and current_trajectory_point_index
    if (trajectories_index + 1 >= trajectories_size) {
      *current_trajectory_index = trajectories_size - 1;
      *current_trajectory_point_index = trajectory_size - 1;
    } else {
      *current_trajectory_index = trajectories_index + 1;
      *current_trajectory_point_index = 0;
    }
    AINFO << "Reach the end of a trajectory, switching to next one";
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

bool OpenSpaceTrajectoryPartition::UseFailSafeSearch(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const std::vector<std::string>& trajectories_encodings,
    size_t* current_trajectory_index, size_t* current_trajectory_point_index) {
  AERROR << "Trajectory partition fail, using failsafe search";
  const size_t trajectories_size = partitioned_trajectories.size();
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      failsafe_closest_point_on_trajs;
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = partitioned_trajectories.at(i).first;
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
      *current_trajectory_index =
          failsafe_closest_point_on_trajs.top().first.first;
      *current_trajectory_point_index =
          failsafe_closest_point_on_trajs.top().first.second;
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

bool OpenSpaceTrajectoryPartition::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  const auto* last_frame = injector_->frame_history()->Latest();
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
        open_space_trajectory_partition_config_.gear_shift_period_duration()) {
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

void OpenSpaceTrajectoryPartition::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t =
      open_space_trajectory_partition_config_.gear_shift_max_t();
  const double gear_shift_unit_t =
      open_space_trajectory_partition_config_.gear_shift_unit_t();
  // TrajectoryPoint point;
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
  ADEBUG << "gear_switch_idle_time_trajectory"
         << gear_switch_idle_time_trajectory->first.size();
  gear_switch_idle_time_trajectory->second = gear_position;
}

// update partitioned_trajectories[current_trajectory_index] relative_time and s
// update unpartitioned_trajectory_result relative_time and s
// get current_partitioned_trajectory
void OpenSpaceTrajectoryPartition::AdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    DiscretizedTrajectory* unpartitioned_trajectory_result,
    TrajGearPair* current_partitioned_trajectory) {
  ///
  // segment trajectory size
  const size_t partitioned_trajectories_size = partitioned_trajectories.size();
  CHECK_GT(partitioned_trajectories_size, current_trajectory_index);

  // Reassign relative time and relative s to have the closest point as origin
  // point
  // segment trajectory current_trajectory_index
  *(current_partitioned_trajectory) =
      partitioned_trajectories.at(current_trajectory_index);
  // trajectory
  auto trajectory = &(current_partitioned_trajectory->first);
  // gear position
  const auto gear = &(current_partitioned_trajectory->second);
  // first reverse trajectory flag
  if (*gear == canbus::Chassis::GEAR_REVERSE && current_trajectory_index < 2) {
    frame_->mutable_open_space_info()->set_is_first_reverse_traj(true);
  } else {
    frame_->mutable_open_space_info()->set_is_first_reverse_traj(false);
  }
  // trajectory point size
  const size_t trajectory_size = trajectory->size();
  // trajectory point closest_trajectory_point_index
  size_t use_id = std::min(closest_trajectory_point_index, trajectory_size - 1);
  // trajectory point closest_trajectory_point_index relative_time
  double time_shift = trajectory->at(use_id).relative_time();
  // trajectory point closest_trajectory_point_index s
  double s_shift = trajectory->at(use_id).path_point().s();
  //double max_front_v = 0;
  //double smooth_v = 0;
  // update trajectory point attribute
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    // update relative_time
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    // update s
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);

/*
    // set v
    if (*gear != canbus::Chassis::GEAR_REVERSE) {    // gear forward
      // max forward velocity
      max_front_v = std::min(std::max(max_front_v, trajectory_point->v()),
                             FLAGS_teb_max_front_speed);
      if (i < kSmoothPercent * trajectory_size) {
        smooth_v = std::max(std::min(smooth_v, trajectory_point->v()),
                            FLAGS_teb_min_front_speed);
        smooth_v = std::min(smooth_v, FLAGS_teb_max_front_speed);
        trajectory_point->set_v(smooth_v);
      } else if (i + kIdDistToEnd < trajectory_size) {
        smooth_v = std::max(
            std::min(0.5 * (smooth_v + trajectory_point->v()), max_front_v),
            smooth_v);
        smooth_v = std::min(smooth_v, FLAGS_teb_max_front_speed);
        trajectory_point->set_v(smooth_v);
      } else {
        smooth_v = std::min(std::max(trajectory_point->v(), kMinSpeed),
                            FLAGS_teb_max_front_speed);
        trajectory_point->set_v(smooth_v);
      }
    }

    if (*gear == canbus::Chassis::GEAR_REVERSE) {
      trajectory_point->set_v(kBackSpeed);
    }
    if (i + 1 == trajectory_size) {
      trajectory_point->set_v(0);
    }
*/
  }
  // TODO(wentao liu): now use new method to use stitched_trajectory_result,
  // temp not update  stitched_trajectory_result s and t; need test

  // // Reassign relative t and s on stitched_trajectory_result for accurate
  // next
  // // frame stitching
  const size_t interpolated_pieces_num =
      open_space_trajectory_partition_config_.interpolated_pieces_num();
  // unpartitioned trajectory points size
  const size_t unpartitioned_trajectory_size =
      unpartitioned_trajectory_result->size();
  // current vehicle position index in unpartitioned_trajectory
  size_t index_estimate = 0;
  for (size_t i = 0; i < current_trajectory_index; ++i) {
    index_estimate += partitioned_trajectories.at(i).first.size();
  }
  index_estimate += closest_trajectory_point_index;
  index_estimate /= interpolated_pieces_num;
  if (index_estimate >= unpartitioned_trajectory_size) {
    index_estimate = unpartitioned_trajectory_size - 1;
  }
  // time offset
  time_shift =
      unpartitioned_trajectory_result->at(index_estimate).relative_time();
  // s offset
  s_shift =
      unpartitioned_trajectory_result->at(index_estimate).path_point().s();
  // update unpartitioned_trajectory_result attribute
  for (size_t i = 0; i < unpartitioned_trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point =
        &(unpartitioned_trajectory_result->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
///
}

// check finished segment trajectory
bool OpenSpaceTrajectoryPartition::CheckFinishInitPosition(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* current_partitioned_trajectory) {
  // vehicle position
  const Vec2d adc_position = {injector_->vehicle_state()->x(),
                              injector_->vehicle_state()->y()};
  // first trajectory
  *(current_partitioned_trajectory) = partitioned_trajectories.at(0);
  // trajectory
  auto trajectory = &(current_partitioned_trajectory->first);
  // nearest index
  size_t adc_index = trajectory->QueryNearestPoint(adc_position);

  // trajectory point adc_index relative_time
  double time_shift = trajectory->at(adc_index).relative_time();
  // trajectory point adc_index s
  double s_shift = trajectory->at(adc_index).path_point().s();

  // update trajectory point attribute
  const size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    // update relative time
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    // update s
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }

  // current vehicle state index
  TrajectoryPoint* start_point = &(trajectory->at(adc_index));
  // trajectory end point
  TrajectoryPoint* end_point = &(trajectory->at(trajectory_size - 1));
  // dist_to_end has a dispersion error
  double dist_to_end_s =
      end_point->path_point().s() - start_point->path_point().s();

  double dx = end_point->path_point().x() - ego_x_;
  double dy = end_point->path_point().y() - ego_x_;

  // distance to end
  double dist_to_end_position = std::hypot(dx, dy);

  TrajectoryPoint* point = &(trajectory->at(0));
  // trajectory total length
  double length = end_point->path_point().s() - point->path_point().s();
  ADEBUG << "dist_to_end_s = " << dist_to_end_s << " adc_speed "
        << ego_v_ << " total length " << length << " dist_to_end_position "
        << dist_to_end_position;
  return ((std::fabs(dist_to_end_s) < longitudinal_offset_to_midpoint_ ||
           dist_to_end_position < longitudinal_offset_to_midpoint_) &&
         std::fabs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped());
}

}  // namespace planning
}  // namespace century
