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
#include "modules/planning/tasks/optimizers/teb_trajectory_generation/teb_trajectory_optimizer.h"
#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"
#include "modules/planning/tasks/task.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"

namespace century {
namespace planning {

struct TEBTrajectoryThreadData {
  century::canbus::Chassis::GearPosition last_gear;
  century::planning::DiscretizedTrajectory last_trace;
  std::vector<std::vector<common::math::Vec2d>> world_obstacles_vertices_vec;
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> start_pose;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  century::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
  std::vector<Obstacle> obstacles;
  std::vector<std::vector<common::math::Vec2d>> boundary_vertices_vec;
  StaticAreaPolygons costmap_obstacles;
};

class TEBTrajectoryProvider : public TrajectoryOptimizer {
 public:
  TEBTrajectoryProvider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

  ~TEBTrajectoryProvider();

  void Stop();

  void Restart();

 private:
  century::common::Status Process() override;

  century::common::Status ProcessNormal();

  century::common::Status ProcessThread();

  void UpdateScenario();

  void GenerateTrajectoryThread();

  bool IsVehicleNearDestination(const common::VehicleState& vehicle_state,
                                const std::vector<double>& end_pose,
                                double rotate_angle,
                                const common::math::Vec2d& translate_origin);

  bool IsVehicleStopDueToFallBack(const bool is_on_fallback,
                                  const common::VehicleState& vehicle_state);

  void GenerateStopTrajectory(DiscretizedTrajectory* const trajectory_data);

  void LoadResult(const Frame* last_frame,
                  DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameResult(const Frame* last_frame,
                            DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameDebug(const Frame* last_frame);

  double GetDistanceToEndPose(
      const common::TrajectoryPoint& planning_init_point,
      const std::vector<double>& end_pose, double rotate_angle,
      const Vec2d& translate_origin);
  void RecomputeStartPoint(const common::VehicleState& vehicle_state,
                           common::math::Vec2d* start_point,
                           double* adc_heading);

 private:
  bool thread_init_flag_ = false;
  bool is_first_enter_openspace_ = false;
  common::math::Vec2d last_end_point_ = {0.0, 0.0};

  std::unique_ptr<TEBTrajectoryOptimizer> teb_trajectory_optimizer_;

  size_t optimizer_thread_counter_ = 0;

  TEBTrajectoryThreadData thread_data_;
  std::atomic<bool> start_plan_task_{false};

  std::future<void> task_future_;
  std::atomic<bool> is_generation_thread_stop_{false};
  std::atomic<bool> trajectory_updated_{false};
  std::atomic<bool> data_ready_{false};
  std::atomic<bool> trajectory_error_{false};
  std::atomic<bool> trajectory_skipped_{false};
  std::mutex open_space_mutex_;
  int ready_count_ = 0;

  bool has_warm_start_ = false;
  century::planning::RescueStatus* rescue_status_;

  // teb
  std::vector<ObstaclePtr> teb_obstacles_;
  // reference points for teb
  ViaPointContainer via_points_;

  std::shared_ptr<OpenspaceCommon> openspace_common_ptr_ =
      std::make_shared<OpenspaceCommon>();

  bool last_rescue_crowded_ = false;
};

}  // namespace planning
}  // namespace century
