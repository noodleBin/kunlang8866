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
#include "modules/planning/open_space/teb/g2o_types/penalties.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"

namespace century {
namespace planning {

/**
 * @class EdgePreferRotDir
 * @brief Edge defining the cost function for penalzing a specified turning
 * direction, in particular left resp. right turns
 *
 * The edge depends on two consecutive vertices \f$ \mathbf{s}_i,
 * \mathbf{s}_{i+1} \f$ and penalizes a given rotation direction based on the \e
 * weight and \e dir (\f$ dir \in \{-1,1\} \f$) \e dir should be +1 to prefer
 * left rotations and -1 to prefer right rotations  \n \e weight can be set
 * using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgePreferRotDir
 */
class EdgePreferRotDir
    : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgePreferRotDir() { _measurement = 1; }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);

    _error[0] = PenaltyBoundFromBelow(
        _measurement * g2o::normalize_theta(conf2->theta() - conf1->theta()), 0,
        0);
  }

  /**
   * @brief Specify the prefered direction of rotation
   * @param dir +1 to prefer the left side, -1 to prefer the right side
   */
  void SetRotDir(double dir) { _measurement = dir; }

  /** Prefer rotations to the right */
  void PreferRight() { _measurement = -1; }

  /** Prefer rotations to the right */
  void PreferLeft() { _measurement = 1; }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
