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

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/misc.h>

#include "modules/planning/open_space/teb/utils/pose_se2.h"

namespace century {
namespace planning {

/**
 * @class VertexPose
 * @brief This class stores and wraps a SE2 pose (position and orientation) into
 * a vertex that can be optimized via g2o
 * @see PoseSE2
 * @see VertexTimeDiff
 */
class VertexPose : public g2o::BaseVertex<3, PoseSE2> {
 public:
  /**
   * @brief Default constructor
   * @param fixed if \c true, this vertex is considered fixed during
   * optimization [default: \c false]
   */
  explicit VertexPose(bool fixed = false) {
    setToOriginImpl();
    setFixed(fixed);
  }

  /**
   * @brief Construct pose using a given PoseSE2
   * @param pose PoseSE2 defining the pose [x, y, angle_rad]
   * @param fixed if \c true, this vertex is considered fixed during
   * optimization [default: \c false]
   */
  explicit VertexPose(const PoseSE2& pose, bool fixed = false) {
    _estimate = pose;
    setFixed(fixed);
  }

  /**
   * @brief Construct pose using a given 2D position vector and orientation
   * @param position Eigen::Vector2d containing x and y coordinates
   * @param theta yaw-angle
   * @param fixed if \c true, this vertex is considered fixed during
   * optimization [default: \c false]
   */
  VertexPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta,
             bool fixed = false) {
    _estimate.position() = position;
    _estimate.theta() = theta;
    setFixed(fixed);
  }

  /**
   * @brief Construct pose using single components x, y, and the yaw angle
   * @param x x-coordinate
   * @param y y-coordinate
   * @param theta yaw angle in rad
   * @param fixed if \c true, this vertex is considered fixed during
   * optimization [default: \c false]
   */
  VertexPose(double x, double y, double theta, bool fixed = false) {
    _estimate.x() = x;
    _estimate.y() = y;
    _estimate.theta() = theta;
    setFixed(fixed);
  }

  /**
   * @brief Destructs the VertexPose
   */
  ~VertexPose() {}

  /**
   * @brief Access the pose
   * @see estimate
   * @return reference to the PoseSE2 estimate
   */
  inline PoseSE2& pose() { return _estimate; }

  /**
   * @brief Access the pose (read-only)
   * @see estimate
   * @return const reference to the PoseSE2 estimate
   */
  inline const PoseSE2& pose() const { return _estimate; }

  /**
   * @brief Access the 2D position part
   * @see estimate
   * @return reference to the 2D position part
   */
  inline Eigen::Vector2d& position() { return _estimate.position(); }

  /**
   * @brief Access the 2D position part (read-only)
   * @see estimate
   * @return const reference to the 2D position part
   */
  inline const Eigen::Vector2d& position() const {
    return _estimate.position();
  }

  /**
   * @brief Access the x-coordinate the pose
   * @return reference to the x-coordinate
   */
  inline double& x() { return _estimate.x(); }

  /**
   * @brief Access the x-coordinate the pose (read-only)
   * @return const reference to the x-coordinate
   */
  inline const double& x() const { return _estimate.x(); }

  /**
   * @brief Access the y-coordinate the pose
   * @return reference to the y-coordinate
   */
  inline double& y() { return _estimate.y(); }

  /**
   * @brief Access the y-coordinate the pose (read-only)
   * @return const reference to the y-coordinate
   */
  inline const double& y() const { return _estimate.y(); }

  /**
   * @brief Access the orientation part (yaw angle) of the pose
   * @return reference to the yaw angle
   */
  inline double& theta() { return _estimate.theta(); }

  /**
   * @brief Access the orientation part (yaw angle) of the pose (read-only)
   * @return const reference to the yaw angle
   */
  inline const double& theta() const { return _estimate.theta(); }

  /**
   * @brief Set the underlying estimate (2D vector) to zero.
   */
  void setToOriginImpl() override { _estimate.SetZero(); }

  /**
   * @brief Define the update increment \f$ \mathbf{x}_{k+1} = \mathbf{x}_k +
   * \Delta \mathbf{x} \f$. A simple addition for the position. The angle is
   * first added to the previous estimated angle and afterwards normalized to
   * the interval \f$ [-\pi \pi] \f$
   * @param update increment that should be added to the previous esimate
   */
  void oplusImpl(const double* update) override { _estimate.Plus(update); }

  /**
   * @brief Read an estimate from an input stream.
   * First the x-coordinate followed by y and the yaw angle.
   * @param is input stream
   * @return always \c true
   */
  bool read(std::istream& is) override {
    is >> _estimate.x() >> _estimate.y() >> _estimate.theta();
    return true;
  }

  /**
   * @brief Write the estimate to an output stream
   * First the x-coordinate followed by y and the yaw angle.
   * @param os output stream
   * @return \c true if the export was successful, otherwise \c false
   */
  bool write(std::ostream& os) const override {
    os << _estimate.x() << " " << _estimate.y() << _estimate.theta();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
