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

#include <float.h>

#include <Eigen/Core>

#include "modules/planning/open_space/teb/g2o_types/base_teb_edges.h"
#include "modules/planning/open_space/teb/g2o_types/penalties.h"
#include "modules/planning/open_space/teb/g2o_types/vertex_timediff.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class EdgeTimeOptimal
 * @brief Edge defining the cost function for minimizing transition time of the
 * trajectory.
 *
 * The edge depends on a single vertex \f$ \Delta T_i \f$ and minimizes: \n
 * \f$ \min \Delta T_i^2 \cdot scale \cdot weight \f$. \n
 * \e scale is determined using the penaltyEquality() function, since we
 * experiences good convergence speeds with it. \n \e weight can be set using
 * setInformation() (something around 1.0 seems to be fine). \n
 * @see TebOptimalPlanner::AddEdgesTimeOptimal
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeTimeOptimal : public BaseTebUnaryEdge<1, double, VertexTimeDiff> {
 public:
  /**
   * @brief Construct edge.
   */
  EdgeTimeOptimal() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    const VertexTimeDiff* timediff =
        static_cast<const VertexTimeDiff*>(_vertices[0]);

    _error[0] = timediff->dt();
  }

#ifdef USE_ANALYTIC_JACOBI
#if 1
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus() {
    _jacobianOplusXi(0, 0) = 1;
  }
#endif
#endif

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
