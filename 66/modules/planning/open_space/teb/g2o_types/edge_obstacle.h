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
#include "modules/planning/open_space/teb/utils/robot_footprint_model.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class EdgeObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from
 * obstacles.
 *
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point ) \cdot weight \f$. \n
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see PenaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles,
 * TebOptimalPlanner::EdgeInflatedObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
class EdgeObstacle
    : public BaseTebUnaryEdge<1, const TebObstacle*, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeObstacle() { _measurement = NULL; }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = robot_model_->CalculateDistance(bandpt->pose(), _measurement);

    // Original obstacle cost.
    _error[0] = PenaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist,
                                      cfg_->optim.penalty_epsilon);

    if (cfg_->optim.obstacle_cost_exponent != 1.0 &&
        cfg_->obstacles.min_obstacle_dist > 0.0) {
      // Optional non-linear cost. Note the max cost (before weighting) is
      // the same as the straight line version and that all other costs are
      // below the straight line (for positive exponent), so it may be
      // necessary to increase weight_obstacle and/or the inflation_weight
      // when using larger exponents.

      // function have problem
      _error[0] = cfg_->obstacles.min_obstacle_dist *
                  std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist,
                           cfg_->optim.obstacle_cost_exponent);
    }
  }

#ifdef USE_ANALYTIC_JACOBI
#if 1
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus() {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    // Eigen::Vector2d deltaS = *_measurement - bandpt->position();
    Eigen::Vector2d deltaS = _measurement->GetCentroid() - bandpt->position();

    double angdiff = atan2(deltaS[1], deltaS[0]) - bandpt->theta();

    double dist_squared = deltaS.squaredNorm();
    CHECK_GT(dist_squared, 0.0);
    double dist = sqrt(dist_squared);

    double aux0 = sin(angdiff);
    double dev_left_border = PenaltyBoundFromBelowDerivative(
        dist * fabs(aux0), cfg_->obstacles.min_obstacle_dist,
        cfg_->optim.penalty_epsilon);

    if (dev_left_border == 0) {
      _jacobianOplusXi(0, 0) = 0;
      _jacobianOplusXi(0, 1) = 0;
      _jacobianOplusXi(0, 2) = 0;
      return;
    }

    double aux1 = -fabs(aux0) / dist;
    double dev_norm_x = deltaS[0] * aux1;
    double dev_norm_y = deltaS[1] * aux1;

    double aux2 = cos(angdiff) * g2o::sign(aux0);
    double aux3 = aux2 / dist_squared;
    double dev_proj_x = aux3 * deltaS[1] * dist;
    double dev_proj_y = -aux3 * deltaS[0] * dist;
    double dev_proj_angle = -aux2;

    _jacobianOplusXi(0, 0) = dev_left_border * (dev_norm_x + dev_proj_x);
    _jacobianOplusXi(0, 1) = dev_left_border * (dev_norm_y + dev_proj_y);
    _jacobianOplusXi(0, 2) = dev_left_border * dev_proj_angle;
  }
#endif
#endif

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setObstacle(const TebObstacle* obstacle) { _measurement = obstacle; }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model) {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg,
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

/**
 * @class EdgeInflatedObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from
 * inflated obstacles.
 *
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, min_obstacle_dist ) \cdot
 * weight_inflation \f$. \n Additional, a second penalty is provided with \n \f$
 * \min \textrm{penaltyBelow}( dist2point, inflation_dist ) \cdot
 * weight_inflation \f$. It is assumed that inflation_dist > min_obstacle_dist
 * and weight_inflation << weight_inflation. \e dist2point denotes the minimum
 * distance to the point obstacle. \n \e penaltyBelow denotes the penalty
 * function, see PenaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
class EdgeInflatedObstacle
    : public BaseTebUnaryEdge<2, const TebObstacle*, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeInflatedObstacle() { _measurement = NULL; }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = robot_model_->CalculateDistance(bandpt->pose(), _measurement);

    // Original "straight line" obstacle cost. The max possible value
    // before weighting is min_obstacle_dist
    _error[0] = PenaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist,
                                      cfg_->optim.penalty_epsilon);

    if (cfg_->optim.obstacle_cost_exponent != 1.0 &&
        cfg_->obstacles.min_obstacle_dist > 0.0) {
      // Optional non-linear cost. Note the max cost (before weighting) is
      // the same as the straight line version and that all other costs are
      // below the straight line (for positive exponent), so it may be
      // necessary to increase weight_obstacle and/or the inflation_weight
      // when using larger exponents.
      _error[0] = cfg_->obstacles.min_obstacle_dist *
                  std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist,
                           cfg_->optim.obstacle_cost_exponent);
    }

    // Additional linear inflation cost
    _error[1] =
        PenaltyBoundFromBelow(dist, cfg_->obstacles.inflation_dist, 0.0);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setObstacle(const TebObstacle* obstacle) { _measurement = obstacle; }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model) {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg,
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
