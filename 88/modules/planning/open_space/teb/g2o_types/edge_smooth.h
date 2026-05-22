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
 * @class EdgeSmooth
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
 * backward driving, but penalizes it slighly). \n The dimension of the error /
 * cost vector is 2: the first component represents the nonholonomic constraint
 * cost, the second one backward-drive cost.
 * @see TebOptimalPlanner::AddEdgeSmooth
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeSmooth : public BaseTebMultiEdge<1, double> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeSmooth() { 
    this->resize(3); 
    }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* conf3 = static_cast<const VertexPose*>(_vertices[2]);

    Eigen::Vector2d pre = conf1->position();
    Eigen::Vector2d cur = conf2->position();

    Eigen::Vector2d next = conf3->position();

    // smooth constraint

    // cost : k * ((-4) * (pre - 2*cur + next))^2 , 4 *k can also be k1

    // ref: fem_pos_deviation_smooth
    // _error[0] =
    //     ((pre[0] - 2 * cur[0] + next[0]) * (pre[0] - 2 * cur[0] + next[0]) +
    //      (pre[1] - 2 * cur[1] + next[1]) * (pre[1] - 2 * cur[1] + next[1]));

    double theta_1 = atan2(cur[1] - pre[1], cur[0] - pre[0]);
    double theta_2 = atan2(next[1] - cur[1], next[0] - cur[0]);
    _error[0] = fabs(theta_2 - theta_1);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
