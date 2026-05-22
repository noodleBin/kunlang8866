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

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"

namespace century {
namespace planning {
class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
 public:
  // constructor
  OpenSpaceTrajectoryPartition(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  // destructor
  ~OpenSpaceTrajectoryPartition() = default;

  void Restart();

 private:

  // process function
  common::Status Process() override;

  /*
   * @brief interpolate planned trajectory
   * @param stitched_trajectory_result   planned trajectory
   * @param interpolated_trajectory  interpolated trajectory
   * */
  void InterpolateTrajectory(
      const DiscretizedTrajectory& stitched_trajectory_result,
      DiscretizedTrajectory* interpolated_trajectory);

  // update vehicle state
  void UpdateVehicleInfo();

  /*
   * @brief encode trajectory = first_point / last_point
   * @param trajectory   segment trajectory
   * @param encoding   encoding
   * */
  bool EncodeTrajectory(const DiscretizedTrajectory& trajectory,
                        std::string* const encoding);

  /*
   * @brief check whether the trajectory_encoding_to_check has been traversed
   * @param trajectory_encoding_to_check   trajectory encode
   * */
  bool CheckTrajTraversed(const std::string& trajectory_encoding_to_check);

  /*
   * @brief add traversed trajectory into history
   * @param chosen_trajectory_encoding    trajectory encode
   * */
  void UpdateTrajHistory(const std::string& chosen_trajectory_encoding);

  /*
   * @brief trajectory segmentation
   * @param trajectory  planned trajectory
   * @param partitioned_trajectories  partitioned trajectory
   * */
  void PartitionTrajectory(const DiscretizedTrajectory& trajectory,
                           std::vector<TrajGearPair>* partitioned_trajectories);

  /*
   * @brief insert trajectory_point into current_trajectory
   * @param trajectory_point              trajectory point
   * @param is_trajectory_last_point      last trajectory point flag
   * @param gear                          gear
   * @param last_pos_vec                  previous trajectory point
   * @param distance_s                    trajectory accumulate s
   * @param current_trajectory            trajectory
   * */
  void LoadTrajectoryPoint(const common::TrajectoryPoint& trajectory_point,
                           const bool is_trajectory_last_point,
                           const canbus::Chassis::GearPosition& gear,
                           common::math::Vec2d* last_pos_vec,
                           double* distance_s,
                           DiscretizedTrajectory* current_trajectory);

  /*
   * @brief check whether the vehicle has reached trajectory last point
   * @param trajectory    trajectory
   * @param gear    gear position
   * @param trajectories_size     segment trajectories size
   * @param trajectories_index                trajectory index
   * @param current_trajectory_index          current trajectory index
   * @param current_trajectory_point_index    current point index
   * */
  bool CheckReachTrajectoryEnd(const DiscretizedTrajectory& trajectory,
                               const canbus::Chassis::GearPosition& gear,
                               const size_t trajectories_size,
                               const size_t trajectories_index,
                               size_t* current_trajectory_index,
                               size_t* current_trajectory_point_index);

  bool UseFailSafeSearch(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const std::vector<std::string>& trajectories_encodings,
      size_t* current_trajectory_index, size_t* current_trajectory_point_index);

  bool InsertGearShiftTrajectory(
      const bool flag_change_to_next, const size_t current_trajectory_index,
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void GenerateGearShiftTrajectory(
      const canbus::Chassis::GearPosition& gear_position,
      TrajGearPair* gear_switch_idle_time_trajectory);

  /*
   * @brief get current_partitioned_trajectory and update partitioned_trajectories
   *        and unpartitioned_trajectory_result attribute
   * @param partitioned_trajectories           segment trajectories
   * @param current_trajectory_index           current segment trajectories index
   * @param closest_trajectory_point_index     current trajectory index
   * @param stitched_trajectory_result         planned trajectory
   * @param current_partitioned_trajectory     current segment trajectory
   * */
  void AdjustRelativeTimeAndS(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const size_t current_trajectory_index,
      const size_t closest_trajectory_point_index,
      DiscretizedTrajectory* stitched_trajectory_result,
      TrajGearPair* current_partitioned_trajectory);

  /*
   * @brief
   * @param partitioned_trajectories    segment trajectories
   * @param current_partitioned_trajectory    first trajectory
   * */
  bool CheckFinishInitPosition(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* current_partitioned_trajectory);

 private:
  // configure
  OpenSpaceTrajectoryPartitionConfig open_space_trajectory_partition_config_;
  double heading_search_range_ = 0.0;
  double heading_track_range_ = 0.0;
  double distance_search_range_ = 0.0;
  double heading_offset_to_midpoint_ = 0.0;
  // lateral error threshold
  double lateral_offset_to_midpoint_ = 0.0;
  // longitudinal error threshold
  double longitudinal_offset_to_midpoint_ = 0.0;
  double vehicle_box_iou_threshold_to_midpoint_ = 0.0;
  double linear_velocity_threshold_on_ego_ = 0.0;

  // vehicle parameter
  common::VehicleParam vehicle_param_;
  // vehicle length
  double ego_length_ = 0.0;
  // vehicle width
  double ego_width_ = 0.0;
  // offset between planning center and geometric center
  double shift_distance_ = 0.0;
  // wheel base
  double wheel_base_ = 0.0;

  // current vehicle heading
  double ego_theta_ = 0.0;
  // current vehicle position
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  // current vehicle velocity
  double ego_v_ = 0.0;
  // current vehicle bounding box
  common::math::Box2d ego_box_;
  // current vehicle moving direction
  double vehicle_moving_direction_ = 0.0;

  struct pair_comp_ {
    bool operator()(
        const std::pair<std::pair<size_t, size_t>, double>& left,
        const std::pair<std::pair<size_t, size_t>, double>& right) const {
      return left.second <= right.second;
    }
  };
  struct comp_ {
    bool operator()(const std::pair<size_t, double>& left,
                    const std::pair<size_t, double>& right) {
      return left.second <= right.second;
    }
  };
};
}  // namespace planning
}  // namespace century
