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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line.h"

namespace century {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static bool ComputeStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason,
      const std::shared_ptr<DependencyInjector>& injector);

  static void ComputeOpenSpaceStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason, bool is_poor_status = false);

  static void ComputeOpenspaceTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason);

  static void ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state,
      const common::TrajectoryPoint* matched_point,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory);

  static void ComputeReinitOpenSpaceStitchingTrajectory(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state,
      const common::TrajectoryPoint* matched_point,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory);

  static void ComputeFallbackStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory);
  static void ComputeFallbackReinitStitchingTrajectory(
      const common::VehicleState& vehicle_state,
      const common::TrajectoryPoint* matched_point,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory);
  static void ComputeFallbackPositionProjection(
      const double x, const double y,
      const PublishableTrajectory& prev_trajectory,
      std::pair<double, double>* const frenet_sd);

  static bool IsNeedReinitStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason,
      const std::shared_ptr<DependencyInjector>& injector);
  static bool IsNeedReinitOpenSpaceStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason);

 private:
  static bool ReplanForDiffTooLarge(
      const bool replan_by_offset, const double planning_cycle_time,
      const common::VehicleState& vehicle_state,
      common::TrajectoryPoint* const time_matched_point,
      const common::TrajectoryPoint& position_matched_point,
      std::vector<common::TrajectoryPoint>* const stitching_trajectory,
      std::string* const replan_reason);

  static void ComputePositionProjection(
      const double x, const double y,
      const common::TrajectoryPoint& matched_point,
      std::pair<double, double>* const frenet_sd);

  static void ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state,
      common::TrajectoryPoint* const reinit_point);
};

}  // namespace planning
}  // namespace century
