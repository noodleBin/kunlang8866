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

#include "Eigen/Eigen"

#ifdef ALIVE
#undef ALIVE
#endif

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/proto/open_space_task_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/open_space/teb/optimal_planner.h"
#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_problem.h"
#include "modules/planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

namespace century {
namespace planning {
class TEBTrajectoryOptimizer {
 public:
  explicit TEBTrajectoryOptimizer(const TEBTrajectoryOptimizerConfig& config);
  explicit TEBTrajectoryOptimizer(const TEBTrajectoryOptimizerConfig& config,
                                  century::planning::RescueStatus* rescue_status);

  virtual ~TEBTrajectoryOptimizer() = default;

  void ClearPlanner();

  void SetUseKappaContraint(bool use_kappa_contraint);

  common::Status Plan(
      const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const std::vector<double>& start_pose,
      const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
      double rotate_angle, const Vec2d& translate_origin,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      const std::vector<Obstacle>& obstacles,
      const std::vector<std::vector<Vec2d>>& boundary_vertices_vec,
      const StaticAreaPolygons& costmap_obstacles, bool use_map_bound,
      bool* start_collision_flag = nullptr);

  common::Status ProcessTebPlanWithKappa(
      const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
      const double& init_x, const double& init_y, const double& init_phi,
      const std::vector<double>& XYbounds, double rotate_angle,
      const Vec2d& translate_origin,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      const std::vector<Obstacle>& obstacles,
      const common::TrajectoryPoint& traj_point, HybridAStartResult& result,
      bool* start_collision_flag);

  common::Status UpdateTEB(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const std::vector<double>& start_pose,
      const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
      double rotate_angle, const Vec2d& translate_origin,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      const std::vector<Obstacle>& obstacles,
      const std::vector<std::vector<Vec2d>>& boundary_vertices_vec,
      const StaticAreaPolygons& costmap_obstacles);

  void GetStitchingTrajectory(
      std::vector<common::TrajectoryPoint>* stitching_trajectory) {
    stitching_trajectory->clear();
    *stitching_trajectory = stitching_trajectory_;
  }

  common::Status ProcessPartitionTrajectories(
      const std::vector<HybridAStartResult>& partition_trajectories,
      const size_t& size, const HybridAStartResult& result, double rotate_angle,
      const Vec2d& translate_origin, double& last_relative_time,
      Twist& robot_vel);

  void GetOptimizedTrajectory(DiscretizedTrajectory* optimized_trajectory) {
    optimized_trajectory->clear();
    *optimized_trajectory = optimized_trajectory_;
  }

  void UpdateDebugInfo(
      ::century::planning_internal::OpenSpaceDebug* open_space_debug);

  century::planning_internal::OpenSpaceDebug* mutable_open_space_debug() {
    return &open_space_debug_;
  }

 private:
  void ConvertObstacleToLine(
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec);

  void ConvertObstacleToCircle(const std::vector<Obstacle>& obstacles,
                               double rotate_angle,
                               const common::math::Vec2d& translate_origin);

  void ConvertObstacleToPolygon(const std::vector<Obstacle>& obstacles,
                                double rotate_angle,
                                const common::math::Vec2d& translate_origin);

  void ConvertStaticAreaToPolygon(const StaticAreaPolygons& costmap_obstacles,
                                  double rotate_angle,
                                  const common::math::Vec2d& translate_origin);

  void ConvertViaPoints();

  void ConvertToTebConfig();
  void ConvertToTebConfigTrajectory();
  void ConvertToTebConfigRobot();
  void ConvertToTebConfigGoalTolerance();
  void ConvertToTebConfigObstacles();
  void ConvertToTebConfigOptimization();
  void ConvertToTebConfigHomotopy();
  void ConvertToTebConfigRecovery();

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  void PathPointNormalizing(double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);

  void RecordDebugInfo(
      const HybridAStartResult& result,
      const common::TrajectoryPoint& trajectory_stitching_point,
      const common::math::Vec2d& translate_origin, const double rotate_angle,
      const std::vector<double>& start_pose,
      const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec);
  void RecordWarmStartInfo(const HybridAStartResult& result);

 private:
  TEBTrajectoryOptimizerConfig config_;
  std::vector<KappaContraintRatioConfig> kappa_contraint_configs_;
  bool use_kappa_contraint_ = false;

  century::planning::RescueStatus* rescue_status_;

  // WYQ
  std::unique_ptr<HybridAStar> warm_start_;
  std::unique_ptr<DistanceApproachProblem> distance_approach_;
  std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;

  std::unique_ptr<TebOptimalPlanner> teb_planner_;

  std::vector<common::TrajectoryPoint> stitching_trajectory_;
  DiscretizedTrajectory optimized_trajectory_;

  century::planning_internal::OpenSpaceDebug open_space_debug_;

  // teb
  TebConfig teb_config_;
  std::vector<ObstaclePtr> teb_obstacles_;
  // reference points for teb
  ViaPointContainer via_points_;
  bool teb_plan_without_smooth_ = true;
  std::vector<double> start_pose_;
  std::vector<double> end_pose_;
  // base plan for teb
  std::vector<PoseStamped> transformed_plan_;
  RobotFootprintModelPtr adc_model_;
  TebVisualizationPtr teb_visual_;
  std::vector<Twist> velocity_profile_;

  std::shared_ptr<OpenspaceCommon> openspace_common_ptr_ =
      std::make_shared<OpenspaceCommon>();
};
}  // namespace planning
}  // namespace century
