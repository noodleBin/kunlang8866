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

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace century {
namespace planning {

class RiskShiftDecider : public Decider {
 public:
  RiskShiftDecider(const TaskConfig& config,
                   const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  // 0. Reverse Shift Function
  void ReverseShiftAvoid(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);

  bool GetReverseRiskObstacle(ReferenceLineInfo* const reference_line_info);

  void UpdateReverseShiftState(ReferenceLineInfo* const reference_line_info);

  double OptimizeRiskProblem(ReferenceLineInfo* const reference_line_info);

  bool GetShiftLimitBounds(ReferenceLineInfo* const reference_line_info,
                           std::pair<double, double>* const shift_limit_bounds);

  double ComputeRiskShiftValue(ReferenceLineInfo* const reference_line_info);

  double ComputeRiskValue(ReferenceLineInfo* const reference_line_info,
                          const double check_l);

  double ComputeEfficiencyValue(ReferenceLineInfo* const reference_line_info,
                                const double check_l);

  bool CanAvoidStaticObstacle(ReferenceLineInfo* const reference_line_info,
                              const Obstacle* obstacle);

  bool CheckLateralIsSafe(ReferenceLineInfo* const reference_line_info,
                          const Obstacle* obstacle);

  bool CanIgnoreRearCar(ReferenceLineInfo* const reference_line_info,
                        const Obstacle* obstacle);

  double GetLateralSpeedBuffer(ReferenceLineInfo* const reference_line_info,
                               const Obstacle* obstacle);

  bool CheckCollisionSafe(ReferenceLineInfo* const reference_line_info,
                          const Obstacle* obstacle);

  void SaveRiskObstacleInfo(ReferenceLineInfo* const reference_line_info,
                            const Obstacle* obstacle,
                            bool* const is_left_reverse_obs,
                            bool* const is_right_reverse_obs);

  bool EnableShiftStaticObstacle(const Obstacle* obstacle);

  bool IsNeedConsiderSlowForwardObstacle(
      ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle);

  void UpdateReverseRiskObstacleInfo(
      ReferenceLineInfo* const reference_line_info, const Obstacle* obstacle,
      const bool is_left_reverse_obs, const bool is_right_reverse_obs);

  double GetLeftFrontReverseRisk(ReferenceLineInfo* const reference_line_info,
                                 const double check_l, const double delta_t);

  double GetRightFrontReverseRisk(ReferenceLineInfo* const reference_line_info,
                                  const double check_l, const double delta_t);

  double GetLeftRearForwardRisk(ReferenceLineInfo* const reference_line_info,
                                const double check_l, const double delta_t);

  double GetRightRearForwardRisk(ReferenceLineInfo* const reference_line_info,
                                 const double check_l, const double delta_t);

  double RiskAreaEstimate(const bool reverse_check, const double init_v,
                          const double obs_v, const double delta_l,
                          const double delta_s);

  bool DisableRiskShiftInPlayStreet();

  int GetReverseCheckTimeBaseOnDistance(
      ReferenceLineInfo* const reference_line_info, std::string id);

  // 1. Efficiency Borrow Function
  void EfficiencyShiftBypass(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info);

  bool CheckFrontSlowBlockObstacle(ReferenceLineInfo* const reference_line_info,
                                   std::vector<std::string>* const slow_obs);

  bool CheckIsNearJunction();

  bool IrrelevantObstacle(ReferenceLineInfo* const reference_line_info,
                          const Obstacle* obstacle);

  bool CheckIsCrossing(ReferenceLineInfo* const reference_line_info,
                       const Obstacle* obstacle);

  bool HasObsSideBySide(ReferenceLineInfo* const reference_line_info,
                        const Obstacle* obstacle);

  bool ObstacleIsNotSlow(ReferenceLineInfo* const reference_line_info,
                         const Obstacle* obstacle);

  bool ObstacleIsTooFar(ReferenceLineInfo* const reference_line_info,
                        const Obstacle* obstacle);

  bool FrontHasVehicle(ReferenceLineInfo* const reference_line_info,
                       const Obstacle* obstacle);

  bool NoNeedByPass(ReferenceLineInfo* const reference_line_info,
                    const Obstacle* obstacle);

  bool CheckObstacleType(const Obstacle* obstacle);

  void UpdateEfficiencyBypassState(bool has_slow_obs);

  void SaveSlowObstacleInfo(ReferenceLineInfo* const reference_line_info,
                            const std::vector<std::string> slow_obs);

 private:
  bool is_in_reverse_avoid_state_ = false;
  bool has_left_unsafe_sidebyside_obs_ = false;
  double lateral_step_ = 0.0;
  bool is_in_efficiency_bypass_state_ = false;
  bool has_pedestrian_obs_ = false;
  bool has_sidebyside_obs_ = false;
  std::pair<double, double> last_lateral_risk_shift_ = {0.0, 0.0};

  std::unordered_map<std::string, int> left_front_reverse_id_;
  std::unordered_map<std::string, int> right_front_reverse_id_;
  std::pair<std::string, double> left_front_reverse_nearest_id_ = {
      "", std::numeric_limits<double>::max()};
  std::pair<std::string, double> right_front_reverse_nearest_id_ = {
      "", std::numeric_limits<double>::max()};
  std::pair<std::string, double> left_rear_forward_nearest_id_ = {
      "", std::numeric_limits<double>::max()};
  std::pair<std::string, double> right_rear_forward_nearest_id_ = {
      "", std::numeric_limits<double>::max()};
};

}  // namespace planning
}  // namespace century
