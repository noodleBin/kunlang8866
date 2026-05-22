/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
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
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star_bidirectional.h"
#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_problem.h"
#include "modules/planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

namespace century {
namespace planning {
class OpenSpaceTrajectoryOptimizer {
 public:
  OpenSpaceTrajectoryOptimizer(
      const OpenSpaceTrajectoryOptimizerConfig& config);

  virtual ~OpenSpaceTrajectoryOptimizer() = default;

  /*
   * @brief plan trajectory function
   * @param stitching_trajectory             stitching trajectory
   * @param end_pose                         end pose in origin axis
   * @param XYbounds                         ROI
   * @param rotate_angle, translate_origin   origin pose
   * @param obstacles_edges_num
   * @param obstacles_A
   * @param obstacles_b
   * @param obstacles_vertices_vec           obstacle list
   * @param time_latency                     calculate duration
   * @param use_pure_astar                   not use reed-shepp curve flag
   * @param nearby_path                      reference line
   */
  common::Status Plan(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
      double rotate_angle, const common::math::Vec2d& translate_origin,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec,
      double* time_latency, bool use_pure_astar = false,
      const hdmap::Path *nearby_path = nullptr);

  std::vector<double> GetPlannedEndPose() {
    return planned_end_pose_;
  }

  common::Status TrajectorySmoother(
      const HybridAStartResult& result,
      const common::TrajectoryPoint& trajectory_stitching_point,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<double>& XYbounds, Eigen::MatrixXd* const xWS,
      Eigen::MatrixXd* const uWS, Eigen::MatrixXd* const state_result_ds,
      Eigen::MatrixXd* const control_result_ds,
      Eigen::MatrixXd* const time_result_ds, Eigen::MatrixXd* const l_warm_up,
      Eigen::MatrixXd* const n_warm_up, Eigen::MatrixXd* const dual_l_result_ds,
      Eigen::MatrixXd* const dual_n_result_ds);

  void GetStitchingTrajectory(
      std::vector<common::TrajectoryPoint>* stitching_trajectory) {
    stitching_trajectory->clear();
    *stitching_trajectory = stitching_trajectory_;
  }

  void GetOptimizedTrajectory(DiscretizedTrajectory* optimized_trajectory) {
    optimized_trajectory->clear();
    *optimized_trajectory = optimized_trajectory_;
  }

  void RecordDebugInfo(
      const common::TrajectoryPoint& trajectory_stitching_point,
      const common::math::Vec2d& translate_origin, const double rotate_angle,
      const std::vector<double>& end_pose, const Eigen::MatrixXd& xWS,
      const Eigen::MatrixXd& uWs, const Eigen::MatrixXd& l_warm_up,
      const Eigen::MatrixXd& n_warm_up, const Eigen::MatrixXd& dual_l_result_ds,
      const Eigen::MatrixXd& dual_n_result_ds,
      const Eigen::MatrixXd& state_result_ds,
      const Eigen::MatrixXd& control_result_ds,
      const Eigen::MatrixXd& time_result_ds,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec);

  void UpdateDebugInfo(
      ::century::planning_internal::OpenSpaceDebug* open_space_debug);

  century::planning_internal::OpenSpaceDebug* mutable_open_space_debug() {
    return &open_space_debug_;
  }

 private:

  /*
   * @brief check whether the planning start point is close to the end_pose
   * @param planning_init_point    planning start point
   * @param end_pose               end pose
   * @param rotate_angle, translate_origin   origin pose
   * */
  bool IsInitPointNearDestination(
      const common::TrajectoryPoint& planning_init_point,
      const std::vector<double>& end_pose, double rotate_angle,
      const common::math::Vec2d& translate_origin);

  /*
   * @brief (x, y, phi)  world --> origin
   * @param rotate_angle, translate_origin   origin pose
   * @param (x, y, phi)  coordinate in world/origin axis
   */
  void PathPointNormalizing(double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);

  /*
   * @brief (x, y, phi)  origin --> world
   * @param rotate_angle, translate_origin   origin pose
   * @param (x, y, phi)  coordinate in world/origin axis
   */
  void PathPointDeNormalizing(double rotate_angle,
                              const common::math::Vec2d& translate_origin,
                              double* x, double* y, double* phi);

  /*
   * @brief get optimized_trajectory_
   * @param state_result_ds                state sequence
   * @param control_result_ds              control sequence
   * @param time_result_ds                 time sequence
   */
  void LoadTrajectory(const Eigen::MatrixXd& state_result_ds,
                      const Eigen::MatrixXd& control_result_ds,
                      const Eigen::MatrixXd& time_result_ds);

  // xWS = [result.x(), result.y(), result.phi(), result.v()]
  // uWS = [result.steer(), result.a()]
  void LoadHybridAstarResultInEigen(HybridAStartResult* result,
                                    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
                                    Eigen::MatrixXd* time_result_ds);

  void UseWarmStartAsResult(
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

  /*
   * @brief
   * @param xWS = [x, y, phi, v]
   * @param uWS = [steer, a]
   * @param XYbounds             ROI
   * @param obstacles_edges_num
   * @param obstacles_A
   * @param obstacles_b
   * @param obstacles_vertices_vec   obstacle list
   * @param last_time_u = [init_steer, init_a]
   * @param init_v = init_v
   * @param state_result_ds          return state sequence
   * @param control_result_ds        return control sequence
   * @param time_result_ds           return time sequence
   * @param l_warm_up
   * @param n_warm_up
   * @param dual_l_result_ds
   * @param dual_n_result_ds
   */
  bool GenerateDistanceApproachTraj(
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const std::vector<double>& XYbounds,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec,
      const Eigen::MatrixXd& last_time_u, const double init_v,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
      Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

  bool GenerateDecoupledTraj(
      const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec,
      Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
      Eigen::MatrixXd* time_result_dc);

  void LoadResult(const DiscretizedTrajectory& discretized_trajectory,
                  Eigen::MatrixXd* state_result_dc,
                  Eigen::MatrixXd* control_result_dc,
                  Eigen::MatrixXd* time_result_dc);

  void CombineTrajectories(
      const std::vector<Eigen::MatrixXd>& xWS_vec,
      const std::vector<Eigen::MatrixXd>& uWS_vec,
      const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& l_warm_up_vec,
      const std::vector<Eigen::MatrixXd>& n_warm_up_vec,
      const std::vector<Eigen::MatrixXd>& dual_l_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& dual_n_result_ds_vec,
      Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
      Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

 private:
  OpenSpaceTrajectoryOptimizerConfig config_;

  std::unique_ptr<OpenspaceCommon> openspace_common_;
  std::unique_ptr<HybridAStarBidirectional> warm_start_;
  std::unique_ptr<DistanceApproachProblem> distance_approach_;
  std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;

  std::vector<common::TrajectoryPoint> stitching_trajectory_;
  DiscretizedTrajectory optimized_trajectory_;

  century::planning_internal::OpenSpaceDebug open_space_debug_;

  std::vector<double> planned_end_pose_;

  double rotate_angle_ = 0.0;
  common::math::Vec2d translate_origin_;
};
}  // namespace planning
}  // namespace century
