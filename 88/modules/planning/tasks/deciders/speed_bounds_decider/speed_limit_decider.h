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
 *   @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_limit.h"

namespace century {
namespace planning {

class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                    ReferenceLineInfo* reference_line_info,
                    const PlanningContext* context);

  virtual ~SpeedLimitDecider() = default;

  virtual common::Status GetSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  virtual common::Status GetHighSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  virtual common::Status UpdateSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  virtual common::Status UpdateSpeedLimitsForDecision(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  virtual common::Status UpdateSpeedLimitsForStopDecision(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;

 private:
  FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;

  double GetFollowSpeedLimit(const SLBoundary& adc_sl_boundary,
                             const Obstacle* obstacle) const;
  double GetSpeedLimitFromReferenceLine(const double& reference_line_s) const;
  double GetSpeedLimitFromCentripetalAcc(const double& kappa) const;
  double CalcCentripetalAccSpeedLimit(
      const double& kappa, const double max_centric_acceleration_limit) const;
  double GetSpeedLimitFromObstacles(
      const IndexedList<std::string, Obstacle>& obstacles,
      const common::PathPoint& path_point,
      const common::FrenetFramePoint& frenet_point,
      const double speed_limit_from_reference_line) const;
  double GetSpeedLimitFromApproachingObstacles(
      const std::vector<std::string>& approaching_obs_ids,
      const common::PathPoint& path_point,
      const double speed_limit_from_reference_line) const;
  double GetSpeedLimitFromNudgeObstacles(
      const Obstacle& obstacle, const SLBoundary& path_point_sl,
      const double speed_limit_from_reference_line) const;
  double MakeSpeedLimitForDynamicPedestrian(const Obstacle& obstacle,
                                            const double path_point_heading,
                                            const double obs_path_l) const;
  double GetSpeedLimitFromPedestrian(const Obstacle& obstacle,
                                     const double path_point_l,
                                     const double path_point_heading,
                                     const double reference_line_s) const;
  double GetSpeedLimitFromBicycle(const Obstacle& obstacle,
                                  const double path_point_l,
                                  const double path_point_heading,
                                  const double reference_line_s) const;

  double GetSpeedLimitFromNudgeObstaclesForHighSpeed(
      const IndexedList<std::string, Obstacle>& obstacles,
      const SLBoundary& adc_sl_boundary_path_point,
      const bool adc_sl_boundary_result) const;

  double GetSpeedLimitFromPedestrianForHighSpeed(
      const IndexedList<std::string, Obstacle>& obstacles,
      const SLBoundary& adc_sl_boundary_path_point,
      const common::PathPoint path_point, bool adc_sl_boundary_result) const;
  void UpdateSpeedLimitsForStopYieldFollowDecision(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  void UpdateSpeedLimitsForSlowdownObstacle(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  void UpdateSpeedLimitsForCrossObstacle(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  void UpdateSpeedLimitsForReverseObstacle(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;
  double GetSpeedLimitFromYieldObstacle(const double path_s) const;
  double GetSpeedLimitFromStopObstacle(const double distance) const;
  double GetSpeedLimitFromFollowObstacle(const double distance,
                                         const double obs_speed) const;
  double GetNearstCrossDistance() const;
  void GetCloseStopYieldFollowObstacleId(
      const IndexedList<std::string, Obstacle>& obstacles,
      std::string* const close_obs_id_stop,
      std::string* const close_obs_id_yield,
      std::string* const close_obs_id_follow) const;
  double SetSpeedLimitForCautionObs(const Obstacle* ptr_obstacle,
                                    const SLBoundary& path_point_sl) const;
  bool  IsNeedReturnlane() const;

  //   double GetFollowLateralBuffer(const double obs_speed) const;

 private:
  const SpeedBoundsDeciderConfig& speed_bounds_config_;
  const SpeedLimitDeciderConfig& speed_limit_config_;
  ReferenceLineInfo* reference_line_info_;
  const PlanningContext* planning_context_;
  const bool need_accelerate_for_overtake_ = false;
  double accel_ratio_for_overtake_ = 1.0;
  static HysteresisInterval obstacles_interval_;
};

}  // namespace planning
}  // namespace century
