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

#include <cmath>

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/penalties.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class EdgeKinematicsDiffDrive
 * @brief Edge defining the cost function for satisfying the non-holonomic
 * kinematics of a differential drive mobile robot.
 *
 * The edge depends on two vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1} \f$ and
 * minimizes a geometric interpretation of the non-holonomic constraint:
 * 	- C. Rösmann et al.: Trajectory modification considering dynamic
 * constraints of autonomous robots, ROBOTIK, 2012.
 *
 * The \e weight can be set using setInformation(): Matrix element 1,1: (Choose
 * a very high value: ~1000). \n A second equation is implemented to penalize
 * backward motions (second element of the error /cost vector). \n The \e weight
 * can be set using setInformation(): Matrix element 2,2: (A value ~1 allows
 * backward driving, but penalizes it slighly). \n The dimLinearizeOplusension
 * of the error / cost vector is 2: the first component represents the
 * nonholonomic constraint cost, the second one backward-drive cost.
 * @see TebOptimalPlanner::AddEdgesKinematics, EdgeKinematicsCarlike
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeKinematicsDiffDrive
    : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeKinematicsDiffDrive() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    // non holonomic constraint
    _error[0] = fabs((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] -
                     (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

    // positive-drive-direction constraint
    Eigen::Vector2d angle_vec(cos(conf1->theta()), sin(conf1->theta()));
    _error[1] = PenaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);
    // epsilon=0, otherwise it pushes the first bandpoints away from start
  }

#ifdef USE_ANALYTIC_JACOBI
#if 1
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    double cos1 = cos(conf1->theta());
    double cos2 = cos(conf2->theta());
    double sin1 = sin(conf1->theta());
    double sin2 = sin(conf2->theta());
    double aux1 = sin1 + sin2;
    double aux2 = cos1 + cos2;

    double dd_error_1 = deltaS[0] * cos1;
    double dd_error_2 = deltaS[1] * sin1;
    double dd_dev =
        PenaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);

    double dev_nh_abs =
        g2o::sign((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] -
                  (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

    // conf1
    _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;   // nh x1
    _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;  // nh y1
    _jacobianOplusXi(1, 0) = -cos1 * dd_dev;      // drive-dir x1
    _jacobianOplusXi(1, 1) = -sin1 * dd_dev;      // drive-dir y1
    _jacobianOplusXi(0, 2) =
        (-dd_error_2 - dd_error_1) * dev_nh_abs;  // nh angle
    _jacobianOplusXi(1, 2) =
        (-sin1 * deltaS[0] + cos1 * deltaS[1]) * dd_dev;  // drive-dir angle1

    // conf2
    _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;  // nh x2
    _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;   // nh y2
    _jacobianOplusXj(1, 0) = cos1 * dd_dev;       // drive-dir x2
    _jacobianOplusXj(1, 1) = sin1 * dd_dev;       // drive-dir y2
    _jacobianOplusXj(0, 2) =
        (-sin2 * deltaS[1] - cos2 * deltaS[0]) * dev_nh_abs;  // nh angle
    _jacobianOplusXj(1, 2) = 0;  // drive-dir angle1
  }
#endif
#endif

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class EdgeKinematicsCarlike
 * @brief Edge defining the cost function for satisfying the non-holonomic
 * kinematics of a carlike mobile robot.
 *
 * The edge depends on two vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1} \f$ and
 * minimizes a geometric interpretation of the non-holonomic constraint:
 *  - C. Rösmann et al.: Trajectory modification considering dynamic constraints
 * of autonomous robots, ROBOTIK, 2012.
 *
 * The definition is identically to the one of the differential drive robot.
 * Additionally, this edge incorporates a minimum turning radius that is
 * required by carlike robots. The turning radius is defined by \f$ r=v/omega
 * \f$.
 *
 * The \e weight can be set using setInformation(): Matrix element 1,1: (Choose
 * a very high value: ~1000). \n The second equation enforces a minimum turning
 * radius. The \e weight can be set using setInformation(): Matrix element 2,2.
 * \n The dimension of the error / cost vector is 3: the first component
 * represents the nonholonomic constraint cost, the second one backward-drive
 * cost and the third one the minimum turning radius
 * @see TebOptimalPlanner::AddEdgesKinematics, EdgeKinematicsDiffDrive
 * @remarks Bounding the turning radius from below is not affected by the
 * penalty_epsilon parameter, the user might add an extra margin to the
 * min_turning_radius param.
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeKinematicsCarlike
    : public BaseTebBinaryEdge<3, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeKinematicsCarlike() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    // non holonomic constraint
    _error[0] = fabs((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] -
                     (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

    // limit minimum turning radius
    double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    // straight line motion
    if (angle_diff == 0) {
      _error[1] = 0;
    } else if (cfg_->trajectory.exact_arc_length) {
      // use exact computation of the radius
      _error[1] =
          PenaltyBoundFromBelow(fabs(deltaS.norm() / (2 * sin(angle_diff / 2))),
                                cfg_->robot.min_turning_radius, 0.0);
    } else {
      // origin
      _error[1] = PenaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff),
                                        cfg_->robot.min_turning_radius, 0.0);

      // lwt: new need test
      // double actual_radius = deltaS.norm() / fabs(angle_diff);
      // _error[1] = actual_radius < cfg_->robot.min_turning_radius
      //                 ? (cfg_->robot.min_turning_radius - actual_radius)
      //                 : 0;
    }

    // positive-drive-direction constraint
    _error[2] = 0.0;
    // Eigen::Vector2d angle_vec(cos(conf1->theta()), sin(conf1->theta()));
    // _error[2] = PenaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);
    // This edge is not affected by the epsilon parameter, the user might add an
    // exra margin to the min_turning_radius parameter.
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeKinematicsAckermann : public BaseTebUnaryEdge<1, double, VertexPose> {
 public:
  EdgeKinematicsAckermann() : BaseTebUnaryEdge<1, double, VertexPose>() {}
  EdgeKinematicsAckermann(double v, double delta)
      : BaseTebUnaryEdge<1, double, VertexPose>(), _v(v), _delta(delta) {}

  void computeError() {
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);

    // Get the pose of the robot
    const Eigen::Vector2d& pos = pose->position();
    const double& theta = pose->theta();

    // // Get the control input
    // const double& v = _measurement;

    // // Get the Ackermann steering angle
    // const double& delta = _measurement;

    const double& v = _v;
    const double& delta = _delta;

    // Compute the predicted pose of the robot
    Eigen::Vector2d pred_pos;
    pred_pos.x() = pos.x() + v * cos(theta) * _dt;
    pred_pos.y() = pos.y() + v * sin(theta) * _dt;
    const double pred_theta = theta + v / _L * tan(delta) * _dt;

    // Compute the error
    _error[0] = pred_theta - pred_pos.y() / _R;
  }

  void linearizeOplus() {
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);

    // Get the pose of the robot
    // const Eigen::Vector2d& pos = pose->position();
    const double& theta = pose->theta();

    // // Get the control input
    // const double& v = _measurement;

    // // Get the Ackermann steering angle
    // const double& delta = _measurement;

    // Get the control input
    const double& v = _v;
    const double& delta = _delta;

    // Compute the predicted pose of the robot
    // Eigen::Vector2d pred_pos;
    // pred_pos.x() = pos.x() + v * cos(theta) * _dt;
    // pred_pos.y() = pos.y() + v * sin(theta) * _dt;
    // const double pred_theta = theta + v / _L * tan(delta) * _dt;

    // Compute the Jacobian matrix
    _jacobianOplusXi(0, 0) = 0;
    _jacobianOplusXi(0, 1) = -1 / _R;
    _jacobianOplusXi(0, 2) = -v * sin(theta) * _dt / _R;
    _jacobianOplusXi(0, 3) = -v * sin(theta) * tan(delta) * _dt / _L / _R;
    _jacobianOplusXi(0, 4) = v * cos(theta) * _dt / _R;
    _jacobianOplusXi(0, 5) = v * cos(theta) * tan(delta) * _dt / _L / _R;
  }

 private:
  double _L = 1.5;  // Distance between the front and rear wheels
  double _R = 6.0;  // Turning radius
  double _dt = 0.1;
  double _v;      // Velocity
  double _delta;  // Ackermann steering angle

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
