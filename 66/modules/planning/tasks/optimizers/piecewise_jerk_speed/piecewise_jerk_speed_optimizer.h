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
 * @file piecewise_jerk_speed_optimizer.h
 **/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/optimizers/speed_optimizer.h"

namespace century {
namespace planning {

class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
 private:
  enum PassableAreaType {
    NONE_PASSABLE_AREA = 0,
    PASSABLE_AREA = 1,
    BELOW_PASSABLE_AREA = 2,
    ABOVE_PASSABLE_AREA = 3,
    MIDDLE_PASSABLE_AREA = 4,
    BOTH_SIDES_PASSABLE_AREA = 5
  };

 public:
  PiecewiseJerkSpeedOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  virtual ~PiecewiseJerkSpeedOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;
  void ReconstructSpeedLimitByKinematic(
      const double min_v_upper_bound, double init_speed, double init_acc,
      std::vector<std::pair<double, double>>* s_dot_bounds);
  void ExtractSpeedPoints(const std::vector<double>& s,
                          const std::vector<double>& ds,
                          const std::vector<double>& dds,
                          SpeedData* const speed_data);
  void DebugSpeedLimit(
      const std::vector<double>& x_ref,
      const std::vector<std::pair<double, double>>& s_dot_bounds);
  void GetSpeedBoundaryAndRefS(
      const PathData& path_data, const SpeedData& reference_speed_data,
      const std::array<double, 3U>& init_s,
      std::vector<std::pair<double, double>>* s_dot_bounds,
      std::vector<double>* penalty_dx, std::vector<double>* x_ref);
  void ReInitStartPointByAccel(
      const std::vector<std::pair<double, double>>& s_bounds,
      const double accel);
  bool GetSTBoundary(std::vector<std::pair<double, double>>* ptr_s_bounds);
  const Obstacle* GetCloseObstacle(double* close_key_point_s);
  bool KinematicSpeedOptimize(double s_0, const std::array<double, 3>& init_s,
                              SpeedData* const speed_data);
  bool SlowBreaking(double* min_yield_s);
  bool IsSameDirection(const Obstacle& obstacle, double adc_theta) const;
  double GetTargetAcc(const SpeedData& reference_speed_data,
                      const double init_v);
  void GetWightCoeffs(const double target_a, const bool use_ref_v,
                      std::array<double, 4U>* coeff_weight);
  void GetWights(const double target_a,
                 const std::array<double, 4U>& coeff_weight,
                 const std::array<double, 3U>& init_s,
                 std::array<double, 4U>* weights);
  common::Status SetProblemStatus(
      const std::array<double, 4U>& weights, const PathData& path_data,
      const std::array<double, 3U>& init_s, const SpeedLimit& speed_limit,
      const bool use_ref_v, const double total_length,
      SpeedData* const speed_data,
      PiecewiseJerkSpeedProblem* piecewise_jerk_problem);
  bool PreProcessForSpeedProblem(const SpeedData& reference_speed_data,
                                 const std::array<double, 3U>& init_s,
                                 const SpeedLimit& speed_limit,
                                 std::array<double, 4U>* ptr_weights);
  bool ExportSlowerBrakeSpeed(const std::array<double, 3U>& init_s,
                              SpeedData* const speed_data);
  bool MakeSlowerBrakeByParamAndPassableArea(
      const std::array<double, 3U>& init_s, SpeedData* const speed_data);
  bool MakeSlowerBrakeAfterQPFailureByPassableArea(
      const std::array<double, 3U>& init_s, SpeedData* const speed_data);
  PassableAreaType HasPassableArea(
      const double min_accel, const double max_accel,
      const double max_presight_time, const bool need_check_after_stop,
      std::array<double, 2U>* accel_bounds = nullptr) const;
  bool GetPassableStateAtAccel(const IndexedObstacles& indexed_obstacles,
                               const double max_presight_time, const double s,
                               const double curr_time, const double move_time,
                               const bool need_check_after_stop) const;
  PassableAreaType GetPassableState(
      const std::vector<std::pair<bool, double>>& passable_states_all_accel,
      std::array<double, 2U>* accel_bounds) const;
  bool IsNeedSlowerAccelerationByDkappa(const DiscretizedPath& path_points);
  common::Status CheckValidityOfSpeedData(SpeedData* const speed_data);
  bool UpdateDecel(double* min_stop_decel);
  void KinematicSlowerBreakingForApproachObs(
      double target_speed, double target_accel,
      const std::array<double, 3U>& init_s, SpeedData* const speed_data);
  double SetTargetAcc();
  double SetTargetAccAtIntersection();

 private:
  size_t num_of_knots_ = 0UL;
  double target_a_ = 0.0;
  int start_up_count_ = 0;
};

}  // namespace planning
}  // namespace century
