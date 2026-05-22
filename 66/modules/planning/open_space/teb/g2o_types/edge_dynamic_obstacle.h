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

/*
 * @file
 */

#pragma once

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/penalties.h"
#include "modules/planning/open_space/teb/g2o_types/teb_obstacles.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_timediff.h"
#include "modules/planning/open_space/teb/utils/robot_footprint_model.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic
 * (moving) obstacles.
 *
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and
 * minimizes: \n \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight
 * \f$. \n \e dist2obstacle denotes the minimum distance to the obstacle
 * trajectory (spatial and temporal). \n \e weight can be set using
 * setInformation(). \n \e penaltyBelow denotes the penalty function, see
 * PenaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and
 * @warning Experimental
 */
class EdgeDynamicObstacle
    : public BaseTebUnaryEdge<2, const TebObstacle*, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeDynamicObstacle() : t_(0) {}

  /**
   * @brief Construct edge and specify the time for its associated pose
   * (neccessary for computeError).
   * @param t_ Estimated time until current pose is reached
   */
  explicit EdgeDynamicObstacle(double t) : t_(t) {}

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = robot_model_->EstimateSpatioTemporalDistance(
        bandpt->pose(), _measurement, t_);

    _error[0] = PenaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist,
                                      cfg_->optim.penalty_epsilon);
    _error[1] = PenaltyBoundFromBelow(
        dist, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0);
  }

  /**
   * @brief Set TebObstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */
  void SetObstacle(const TebObstacle* obstacle) { _measurement = obstacle; }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void SetRobotModel(const BaseRobotFootprintModel* robot_model) {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void SetParameters(const TebConfig& cfg,
                     const BaseRobotFootprintModel* robot_model,
                     const TebObstacle* obstacle) {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

 protected:
  //!< Store pointer to robot_model
  const BaseRobotFootprintModel* robot_model_;
  //!< Estimated time until current pose is reached
  double t_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
