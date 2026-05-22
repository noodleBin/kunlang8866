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
 * @file piecewise_jerk_path_optimizer.h
 **/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace century {
namespace planning {

class PiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  PiecewiseJerkPathOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  virtual ~PiecewiseJerkPathOptimizer() = default;

 private:
  common::Status Process(const SpeedData& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         const bool path_reusable,
                         PathData* const path_data) override;

  void GetAllCandidatePathData(
      const ReferenceLine& reference_line,
      const common::TrajectoryPoint& init_point,
      const PathData& final_path_data,
      std::vector<PathData>* const candidate_path_data);

  common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
      const common::TrajectoryPoint& traj_point);

  std::vector<common::PathPoint> ConvertPathPointRefFromFrontAxeToRearAxe(
      const PathData& path_data);

  /**
   * @brief
   *
   * @param init_state path start point
   * @param end_state path end point
   * @param path_reference_l_ref: a vector with default value 0.0
   * @param path_reference_size: length of learning model output
   * @param delta_s: path point spatial distance
   * @param is_valid_path_reference: whether using learning model output or not
   * @param lat_boundaries: path boundaries
   * @param ddl_bounds: constains
   * @param w: weighting scales
   * @param max_iter: optimization max interations
   * @param ptr_x: optimization result of x
   * @param ptr_dx: optimization result of dx
   * @param ptr_ddx: optimization result of ddx
   * @return true
   * @return false
   */
  bool OptimizePath(
      const ReferenceLine& reference_line, const PathBoundary& path_boundary,
      const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state,
      std::vector<double>* ptr_x, std::vector<double>* ptr_dx,
      std::vector<double>* ptr_ddx, PathData* const path_data);

  double ComputeEndStateL(const ReferenceLine& reference_line,
                          const PathBoundary& path_boundary,
                          const std::pair<std::array<double, 3>,
                                          std::array<double, 3>>& init_state);
  bool IsNeedBackWardTodiagonal();
  bool AdcInLane();

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;

  double GaussianWeighting(const double x, const double peak_weighting,
                           const double peak_weighting_x) const;
  bool IsSafeToLaneborrow(const bool is_turn_left) const;
  bool IsSafeToLeftborrow(const SLBoundary leftborrow_obs) const;
  bool SelfBorrowSafeCheck(const double request_end_state_l) const;
  bool SafeReturnCheck(const double request_end_state_l) const;
  bool HysteresisFilter(const double obstacle_distance,
                        const double safe_distance,
                        const double distance_buffer) const;
  double LaneChangeLateralTTC(const Obstacle& obstacle) const;
  void IsSCanExit(const ReferenceLineInfo& reference_line_info,
                  bool* left_neighbor_exitable, bool* right_neighbor_exitable,
                  const double s) const;
  double GetSelfPathEndStateL(
      const PathBoundary& path_boundary,
      const std::pair<std::array<double, 3>, std::array<double, 3>>&
          init_state);
  double GetSelfPathBlockS(const PathBoundary& path_boundary,
                           double curr_lane_right_width,
                           double curr_lane_left_width, double* center_l);
  void CheckBorrowInfo();
  bool CheckIsDiagonalRoad();
  void GetLaneBorrowEndState(const PathBoundary& path_boundary,
                             double* const end_state_l);

  void SaveLastSuccessPath(const PathBoundary& path_boundary,
                           const bool res_opt, const std::vector<double>& opt_l,
                           const std::vector<double>& opt_dl,
                           const std::vector<double>& opt_ddl);

  bool IsValidOptimizeCondition(const ReferenceLine& reference_line,
                                const PathBoundary& path_boundary,
                                const bool res_opt,
                                std::vector<double>* const opt_l,
                                std::vector<double>* const opt_dl,
                                std::vector<double>* const opt_ddl);

  void ReuseLastSuccessPathData(const ReferenceLine& reference_line,
                                const double path_delta_s,
                                const common::math::Vec2d& last_success_point,
                                const std::vector<double>& last_success_opt_l,
                                const std::vector<double>& last_success_opt_dl,
                                const std::vector<double>& last_success_opt_ddl,
                                std::vector<double>* const opt_l,
                                std::vector<double>* const opt_dl,
                                std::vector<double>* const opt_ddl);

  std::array<double, 5> GetPathOptimizeWeight(
      const std::pair<std::array<double, 3>, std::array<double, 3>>&
          init_state);

  double ComputePullOverEndStatel(const ReferenceLine& reference_line,
                                  const PathBoundary& path_boundary);
  double GetEndStateLForHasConvex(double request_end_l);
  bool IsValidPathReference(
      const ReferenceLine& reference_line, const PathBoundary& path_boundary,
      PiecewiseJerkPathProblem* const piecewise_jerk_problem,
      std::array<double, 3>* const end_state);

  void SetValidPathData(const ReferenceLine& reference_line,
                        const PathBoundary& path_boundary,
                        const std::vector<double>& opt_l,
                        const std::vector<double>& opt_dl,
                        const std::vector<double>& opt_ddl,
                        PathData* const path_data,
                        std::vector<PathData>* const candidate_path_data);

  void ComputeddlBounds(
      const PathBoundary& path_boundary,
      std::vector<std::pair<double, double>>* const ddl_bounds);

  double ComputeJerkBound(const std::pair<std::array<double, 3>,
                                          std::array<double, 3>>& init_state);

  void SetSmoothXRef(const PathBoundary& path_boundary,
                     std::vector<double>* const x_ref,
                     const double path_x_ref_pre_smooth_time);

  std::pair<double, double> GetMaxValidEndState(
      const std::vector<std::pair<double, double>>& end_state_set);

  void UpdateLoosenPathConstrainState(const bool has_path_ok);

  bool IsSpecialSceneForSelfBorrow();

  bool IsNeedWaitToBorrowStraightly();

  void UpdatePedestrainCautionState(const bool need_caution_pedestrain);

 private:
  std::vector<double> last_self_success_opt_l_;
  std::vector<double> last_self_success_opt_dl_;
  std::vector<double> last_self_success_opt_ddl_;
  common::math::Vec2d last_self_success_point_ = {0.0, 0.0};

  std::vector<double> last_lanechange_success_opt_l_;
  std::vector<double> last_lanechange_success_opt_dl_;
  std::vector<double> last_lanechange_success_opt_ddl_;
  common::math::Vec2d last_lanechange_success_point_ = {0.0, 0.0};

  double self_borrow_l_ = 0.0;
  double real_self_borrow_l_ = 0.0;
  bool is_can_exit_ = true;
  bool last_adc_is_in_merge_lane_ = false;
  bool has_left_neighbor_lane_ = false;
  bool has_right_neighbor_lane_ = false;
  bool accept_to_left_borrow_ = false;
  bool accept_to_right_borrow_ = false;
  double get_left_neighbor_lane_width_ = 0.0;
  double get_right_neighbor_lane_width_ = 0.0;
  double is_diagonal_road_ = false;
  bool need_to_use_narrow_center_l_ = false;
  double request_end_l_ = 0.0;
  int count_need_use_large_lateral_speed_ = 0;
};

}  // namespace planning
}  // namespace century
