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

#include "Eigen/Dense"

#include "modules/common/configs/proto/vehicle_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"

namespace century {
namespace planning {
class TEBFallbackDecider : public Decider {
 public:
  TEBFallbackDecider(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

 private:
  century::common::Status Process(Frame* frame) override;

  void BuildPredictedEnvironment(const std::vector<const Obstacle*>& obstacles,
                                 std::vector<std::vector<common::math::Box2d>>&
                                     predicted_bounding_rectangles);

  bool IsCollisionFreeTrajectory(
      const TrajGearPair& trajectory_gear_pair,
      const std::vector<std::vector<common::math::Box2d>>&
          predicted_bounding_rectangles,
      size_t* current_idx, size_t* first_collision_idx);

  bool IsCollisionFreeTrajectory(const TrajGearPair& trajectory_gear_pair,
                                 const std::vector<const Obstacle*>& obstacles,
                                 const size_t* current_idx,
                                 size_t* first_collision_idx);

  bool IsADCWillCollision(const TrajGearPair& trajectory_gear_pair,
                          const std::vector<const Obstacle*>& obstacles,
                          const size_t* current_idx);

  bool IsADCWillCollisionUseDynamic(
      const TrajGearPair& trajectory_gear_pair,
      const std::vector<const Obstacle*>& obstacles, const size_t* current_idx);

  bool IsADCBackCollisionUseDynamic(
      const TrajGearPair& trajectory_gear_pair,
      const std::vector<const Obstacle*>& obstacles, const size_t* current_idx);

  bool IsADCWillCollisionWithDynamicVehicle(
      const TrajGearPair& trajectory_gear_pair,
      const std::vector<const Obstacle*>& obstacles, const size_t* current_idx);

  bool IsADCWillCollisionWithStaticVehicle(
      const std::vector<const Obstacle*>& obstacles, const size_t* current_idx,
      const TrajGearPair& trajectory_gear_pair);

  void ReverseTrajectory(const size_t* current_idx,
                         const TrajGearPair* trajectory_gear_pair);

  void CalFallBackIndexWithDelayTime(TrajGearPair* trajectory_gear_pair,
                                     size_t* current_idx);

  void ProcessBlockScenario(const std::vector<const Obstacle*>& obstacles,
                            size_t* current_idx,
                            TrajGearPair* trajectory_gear_pair);

  void ConvertCostmap();

  bool IsADCDeviationTrajectory(const TrajGearPair& trajectory_pb);

  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);

  bool CheckObstacleCollision(const double relative_s,
                              const common::math::Box2d& ego_box,
                              const std::vector<const Obstacle*>& obstacles);
  void DealBuffer();
  void CheckCollisionForFallback(const TrajGearPair& trajectory_gear_pair,
                                 const std::vector<const Obstacle*>& obstacles,
                                 const size_t* current_index,
                                 size_t* first_collision_index, bool* fallback);

  size_t block_count_ = 0;
  std::vector<common::math::Polygon2d> costmap_polygons_;
  century::planning::RescueStatus* rescue_status_;
  bool is_dynamic_collision_ = false;
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double ego_length_bare_ = 0.0;
  double ego_width_bare_ = 0.0;
  double shift_distance_ = 0.0;
  double distance_center_ = 0.0;
};

}  // namespace planning
}  // namespace century
