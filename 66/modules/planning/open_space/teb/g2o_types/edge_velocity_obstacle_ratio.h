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
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_timediff.h"
#include "modules/planning/open_space/teb/utils/robot_footprint_model.h"

namespace century {
namespace planning {

/**
 * @class EdgeVelocityObstacleRatio
 * @brief Edge defining the cost function for keeping a minimum distance from
 * obstacles.
 *
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point ) \cdot weight \f$. \n
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles,
 * TebOptimalPlanner::EdgeInflatedObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
class EdgeVelocityObstacleRatio
    : public BaseTebMultiEdge<2, const TebObstacle*> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeVelocityObstacleRatio() : robot_model_(nullptr) {
    // The three vertices are two poses and one time difference
    // Since we derive from a g2o::BaseMultiEdge, set the desired number of
    // vertices
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT =
        static_cast<const VertexTimeDiff*>(_vertices[2]);

    const Eigen::Vector2d deltaS =
        conf2->estimate().position() - conf1->estimate().position();

    double dist = deltaS.norm();
    const double angle_diff =
        g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0) {
      double radius = dist / (2 * sin(angle_diff / 2));
      // actual arg length!
      dist = fabs(angle_diff * radius);
    }
    CHECK_NE(deltaT->estimate(), 0.0);
    double vel = dist / deltaT->estimate();

    // consider direction
    vel *= FastSigmoid(100 * (deltaS.x() * cos(conf1->theta()) +
                              deltaS.y() * sin(conf1->theta())));

    const double omega = angle_diff / deltaT->estimate();

    double dist_to_obstacle =
        robot_model_->CalculateDistance(conf1->pose(), _measurement);

    double ratio;
    if (dist_to_obstacle < cfg_->obstacles.obstacle_proximity_lower_bound) {
      ratio = 0;
    } else if (dist_to_obstacle >
               cfg_->obstacles.obstacle_proximity_upper_bound) {
      ratio = 1;
    } else {
      ratio =
          (dist_to_obstacle - cfg_->obstacles.obstacle_proximity_lower_bound) /
          (cfg_->obstacles.obstacle_proximity_upper_bound -
           cfg_->obstacles.obstacle_proximity_lower_bound);
    }

    ratio *= cfg_->obstacles.obstacle_proximity_ratio_max_vel;

    const double max_vel_fwd = ratio * cfg_->robot.max_vel_x;
    const double max_omega = ratio * cfg_->robot.max_vel_theta;
    _error[0] = PenaltyBoundToInterval(vel, max_vel_fwd, 0);
    _error[1] = PenaltyBoundToInterval(omega, max_omega, 0);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function
   * @param obstacle 2D position vector containing the position of the obstacle
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

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century