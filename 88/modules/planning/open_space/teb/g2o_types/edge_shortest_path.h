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

#include <cfloat>

#include <Eigen/Core>

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_pose.h"

namespace century {
namespace planning {

/**
 * @class EdgeShortestPath
 * @brief Edge defining the cost function for minimizing the Euclidean distance
 * between two consectuive poses.
 *
 * @see TebOptimalPlanner::AddEdgesShortestPath
 */
class EdgeShortestPath
    : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeShortestPath() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose *pose1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *pose2 = static_cast<const VertexPose *>(_vertices[1]);
    _error[0] = (pose2->position() - pose1->position()).norm();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
