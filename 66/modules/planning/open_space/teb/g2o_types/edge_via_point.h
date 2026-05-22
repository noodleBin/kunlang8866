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

#include <g2o/core/base_unary_edge.h>

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"

namespace century {
namespace planning {

/**
 * @class EdgeViaPoint
 * @brief Edge defining the cost function for pushing a configuration towards a
 * via point
 *
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min  dist2point \cdot weight \f$. \n
 * \e dist2point denotes the distance to the via point. \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgesViaPoints
 * @remarks Do not forget to call setTebConfig() and setViaPoint()
 */
class EdgeViaPoint
    : public BaseTebUnaryEdge<1, const Eigen::Vector2d*, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeViaPoint() { _measurement = NULL; }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    _error[0] = (bandpt->position() - *_measurement).norm();
  }

  /**
   * @brief Set pointer to associated via point for the underlying cost function
   * @param via_point 2D position vector containing the position of the via
   * point
   */
  void SetViaPoint(const Eigen::Vector2d* via_point) {
    _measurement = via_point;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param via_point 2D position vector containing the position of the via
   * point
   */
  void SetParameters(const TebConfig& cfg, const Eigen::Vector2d* via_point) {
    cfg_ = &cfg;
    _measurement = via_point;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
