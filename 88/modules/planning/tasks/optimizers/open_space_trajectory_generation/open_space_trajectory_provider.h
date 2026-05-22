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
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_optimizer.h"
#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"
#include "modules/planning/tasks/task.h"

namespace century {
namespace planning {

// input data
struct OpenSpaceTrajectoryThreadData {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  century::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
  hdmap::Path nearby_path;
};

struct OpenSpaceTrajectoryThreadResult {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  century::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
};

class OpenSpaceTrajectoryProvider : public TrajectoryOptimizer {
 public:
  OpenSpaceTrajectoryProvider(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  ~OpenSpaceTrajectoryProvider();

  void Stop();
  void Restart();

 private:

  century::common::Status Process() override;

  common::Status Process(
      const century::planning::Frame* pre_frame,
      std::vector<common::TrajectoryPoint>& stitching_trajectory);

  // new a thread for planning
  void GenerateTrajectoryThread();

  /*
   * @brief check is arrived at destination
   * @param vehicle_state  vehicle state
   * @param end_pose   end pose (origin axis)
   * @param rotate_angle, translate_origin  origin pose
   */
  bool IsVehicleNearDestination(const common::VehicleState& vehicle_state,
                                const std::vector<double>& end_pose,
                                double rotate_angle,
                                const common::math::Vec2d& translate_origin);

  bool IsVehicleStopDueToFallBack(const bool is_on_fallback,
                                  const common::VehicleState& vehicle_state);

  /*
   * @brief generate stop trajectory
   * @param trajectory_data  planned trajectory
   */
  void GenerateStopTrajectory(DiscretizedTrajectory* const trajectory_data);

  /*
   * @brief load planned trajectory result
   * @param last_frame  last frame
   * @param trajectory_data  planned trajectory
   */
  void LoadResult(const Frame* last_frame,
                  DiscretizedTrajectory* const trajectory_data);

  /*
   * @brief use last frame result
   * @param last_frame  last frame
   * @param trajectory_data   planned trajectory
   */
  void ReuseLastFrameResult(const Frame* last_frame,
                            DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameDebug(const Frame* last_frame);

  /*
   * @brief compute the distance between planning_init_point and end_pose
   * @param planning_init_point    planning_init_point
   * @param end_pose           end pose
   * @param rotate_angle       origin heading
   * @param translate_origin   origin position
   */
  double GetDistanceToEndPose(
      const common::TrajectoryPoint& planning_init_point,
      const std::vector<double>& end_pose, double rotate_angle,
      const Vec2d& translate_origin);

 private:
  bool thread_init_flag_ = false;
  bool is_first_enter_openspace_ = false;
  common::math::Vec2d last_end_point_ = {0.0, 0.0};
  std::shared_ptr<DiscretizedTrajectory> last_plan_traj_ptr_;

  std::unique_ptr<OpenSpaceTrajectoryOptimizer>
      open_space_trajectory_optimizer_;

  size_t optimizer_thread_counter = 0;

  OpenSpaceTrajectoryThreadData thread_data_;
  std::atomic<bool> start_plan_task_{false};
  std::future<void> task_future_;
  std::atomic<bool> is_generation_thread_stop_{false};
  std::atomic<bool> trajectory_updated_{false};
  std::atomic<bool> data_ready_{false};
  std::atomic<bool> trajectory_error_{false};
  std::atomic<bool> trajectory_skipped_{false};
  std::mutex open_space_mutex_;
  int ready_count_ = 0;
};

}  // namespace planning
}  // namespace century
