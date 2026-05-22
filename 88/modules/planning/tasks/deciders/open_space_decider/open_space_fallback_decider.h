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
#include "modules/planning/open_space/openspace_common/openspace_common.h"

namespace century {
namespace planning {
class OpenSpaceFallbackDecider : public Decider {
 public:
  OpenSpaceFallbackDecider(const TaskConfig& config,
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
                                 size_t* current_idx,
                                 size_t* first_collision_idx);

  bool IsADCDeviationTrajectory(const TrajGearPair& trajectory_pb);

  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);

  bool CheckObstacleCollision(const double relative_time,
                              const common::math::Box2d& ego_box,
                              const std::vector<const Obstacle*>& obstacles);

  void EmergencyStop(TrajGearPair& traj_info);

 private:
  OpenspaceCommon openspace_common_;
};

}  // namespace planning
}  // namespace century
