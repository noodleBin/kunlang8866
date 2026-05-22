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

#include <algorithm>
#include <iostream>

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/penalties.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_timediff.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class EdgeVelocity
 * @brief Edge defining the cost function for limiting the translational and
 * rotational velocity.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta
 * T_i \f$ and minimizes: \n \f$ \min \textrm{penaltyInterval}( [v,omega]^T )
 * \cdot weight \f$. \n \e v is calculated using the difference quotient and the
 * position parts of both poses. \n \e omega is calculated using the difference
 * quotient of both yaw angles followed by a normalization to [-pi, pi]. \n \e
 * weight can be set using setInformation(). \n \e penaltyInterval denotes the
 * penalty function, see PenaltyBoundToInterval(). \n The dimension of the error
 * / cost vector is 2: the first component represents the translational velocity
 * and the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocity : public BaseTebMultiEdge<2, double> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeVelocity() {
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

    //     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) +
    //     deltaS[1]*sin(conf1->theta())); // consider direction
    // consider direction
    vel *= FastSigmoid(100 * (deltaS.x() * cos(conf1->theta()) +
                              deltaS.y() * sin(conf1->theta())));

    const double omega = angle_diff / deltaT->estimate();

    _error[0] = PenaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards,
                                       cfg_->robot.max_vel_x,
                                       cfg_->optim.penalty_epsilon);
    _error[1] = PenaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,
                                       cfg_->optim.penalty_epsilon);
  }

#ifdef USE_ANALYTIC_JACOBI
#if 1
  // TODO(all) the hardcoded jacobian does not include the changing direction
  // (just the absolute value)
  //  Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT =
        static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist * deltaT->estimate();
    CHECK_NE(deltaT->estimate(), 0.0);
    double aux2 = 1 / deltaT->estimate();

    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;

    double dev_border_vel = PenaltyBoundToIntervalDerivative(
        vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,
        cfg_->optim.penalty_epsilon);
    double dev_border_omega = PenaltyBoundToIntervalDerivative(
        omega, cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    _jacobianOplus[0].resize(2, 3);  // conf1
    _jacobianOplus[1].resize(2, 3);  // conf2
    _jacobianOplus[2].resize(2, 1);  // deltaT

    //  if (aux1==0) aux1=1e-6;
    //  if (aux2==0) aux2=1e-6;

    if (dev_border_vel != 0) {
      CHECK_NE(aux1, 0.0);
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0, 0) = -deltaS[0] * aux3;             // vel x1
      _jacobianOplus[0](0, 1) = -deltaS[1] * aux3;             // vel y1
      _jacobianOplus[1](0, 0) = deltaS[0] * aux3;              // vel x2
      _jacobianOplus[1](0, 1) = deltaS[1] * aux3;              // vel y2
      _jacobianOplus[2](0, 0) = -vel * aux2 * dev_border_vel;  // vel deltaT
    } else {
      _jacobianOplus[0](0, 0) = 0;  // vel x1
      _jacobianOplus[0](0, 1) = 0;  // vel y1
      _jacobianOplus[1](0, 0) = 0;  // vel x2
      _jacobianOplus[1](0, 1) = 0;  // vel y2
      _jacobianOplus[2](0, 0) = 0;  // vel deltaT
    }

    if (dev_border_omega != 0) {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1, 0) = -omega * aux4;  // omega deltaT
      _jacobianOplus[0](1, 2) = -aux4;          // omega angle1
      _jacobianOplus[1](1, 2) = aux4;           // omega angle2
    } else {
      _jacobianOplus[2](1, 0) = 0;  // omega deltaT
      _jacobianOplus[0](1, 2) = 0;  // omega angle1
      _jacobianOplus[1](1, 2) = 0;  // omega angle2
    }

    _jacobianOplus[0](1, 0) = 0;  // omega x1
    _jacobianOplus[0](1, 1) = 0;  // omega y1
    _jacobianOplus[1](1, 0) = 0;  // omega x2
    _jacobianOplus[1](1, 1) = 0;  // omega y2
    _jacobianOplus[0](0, 2) = 0;  // vel angle1
    _jacobianOplus[1](0, 2) = 0;  // vel angle2
  }
#endif
#endif

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class EdgeVelocityHolonomic
 * @brief Edge defining the cost function for limiting the translational and
 * rotational velocity according to x,y and theta.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta
 * T_i \f$ and minimizes: \n \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T
 * ) \cdot weight \f$. \n \e vx denotes the translational velocity w.r.t. x-axis
 * (computed using finite differneces). \n \e vy denotes the translational
 * velocity w.r.t. y-axis (computed using finite differneces). \n \e omega is
 * calculated using the difference quotient of both yaw angles followed by a
 * normalization to [-pi, pi]. \n \e weight can be set using setInformation().
 * \n \e penaltyInterval denotes the penalty function, see
 * PenaltyBoundToInterval(). \n The dimension of the error / cost vector is 3:
 * the first component represents the translational velocity w.r.t. x-axis, the
 * second one w.r.t. the y-axis and the third one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeVelocityHolonomic() {
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
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    double cos_theta1 = std::cos(conf1->theta());
    double sin_theta1 = std::sin(conf1->theta());

    // transform conf2 into current robot frame conf1 (inverse 2d rotation
    // matrix)
    double r_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double r_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();

    CHECK_NE(deltaT->estimate(), 0.0);
    double vx = r_dx / deltaT->estimate();
    double vy = r_dy / deltaT->estimate();
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) /
                   deltaT->estimate();

    // ----------------New-----------------------------------------
    double max_vel_trans_remaining_y;
    double max_vel_trans_remaining_x;
    max_vel_trans_remaining_y = std::sqrt(std::max(
        0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vx * vx));
    max_vel_trans_remaining_x = std::sqrt(std::max(
        0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vy * vy));

    double max_vel_y =
        std::min(max_vel_trans_remaining_y, cfg_->robot.max_vel_y);
    double max_vel_x =
        std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x);
    double max_vel_x_backwards =
        std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x_backwards);

    _error[0] = PenaltyBoundToInterval(vx, -max_vel_x_backwards, max_vel_x,
                                       cfg_->optim.penalty_epsilon);
    // we do not apply the penalty epsilon here, since
    // the velocity could be close to zero
    _error[1] = PenaltyBoundToInterval(vy, max_vel_y, 0.0);
    _error[2] = PenaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,
                                       cfg_->optim.penalty_epsilon);
    // ----------------New-----------------------------------------

    // ----------------Old-----------------------------------------
    // _error[0] = penaltyBoundToInterval(vx, -cfg_->robot.max_vel_x_backwards,
    //                                    cfg_->robot.max_vel_x,
    //                                    cfg_->optim.penalty_epsilon);
    // // we do not apply the penalty epsilon here, since the velocity
    // // could be close to zero
    // _error[1] = penaltyBoundToInterval(vy, cfg_->robot.max_vel_y, 0.0);
    // _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,
    //                                    cfg_->optim.penalty_epsilon);
    // ----------------Old-----------------------------------------
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
