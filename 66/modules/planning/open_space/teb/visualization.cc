/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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

#include "modules/planning/open_space/teb/optimal_planner.h"
#include "modules/planning/open_space/teb/visualization.h"

namespace century {
namespace planning {

TebVisualization::TebVisualization() : initialized_(false) {}

TebVisualization::TebVisualization(const TebConfig& cfg) : initialized_(false) {
  Initialize(cfg);
}

void TebVisualization::Initialize(const TebConfig& cfg) {
  cfg_ = &cfg;
  initialized_ = true;
}

void TebVisualization::PublishGlobalPlan(
    const std::vector<PoseStamped>& global_plan) const {}

void TebVisualization::PublishLocalPlan(
    const std::vector<PoseStamped>& local_plan) const {}

void TebVisualization::PublishLocalPlanAndPoses(
    const TimedElasticBand& teb) const {}

void TebVisualization::PublishInfeasibleRobotPose(
    const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model) {
  // publishRobotFootprintModel(current_pose, robot_model,
  // "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
}

void TebVisualization::PublishObstacles(const ObstContainer& obstacles) const {}

void TebVisualization::PublishViaPoints(
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>>& via_points,
    const std::string& ns) const {}

void TebVisualization::PublishTebContainer(
    const TebOptPlannerContainer& teb_planner, const std::string& ns) {}

void TebVisualization::PublishFeedbackMessage(
    const std::vector<boost::shared_ptr<TebOptimalPlanner>>& teb_planners,
    unsigned int selected_trajectory_idx, const ObstContainer& obstacles) {}

void TebVisualization::PublishFeedbackMessage(
    const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles) {}

bool TebVisualization::PrintErrorWhenNotInitialized() const { return true; }

}  // namespace planning
}  // namespace century
