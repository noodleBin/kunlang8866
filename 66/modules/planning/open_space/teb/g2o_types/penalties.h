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

#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace century {
namespace planning {

/**
 * @brief Linear penalty function for bounding \c var to the interval \f$ -a <
 * var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not
 * satisfied
 */
inline double PenaltyBoundToInterval(const double& var, const double& a,
                                     const double& epsilon) {
  if (var < -a + epsilon) {
    return (-var - (a - epsilon));
  }
  if (var <= a - epsilon) {
    return 0.;
  } else {
    return (var - (a - epsilon));
  }
}

/**
 * @brief Linear penalty function for bounding \c var to the interval \f$ a <
 * var < b \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not
 * satisfied
 */
inline double PenaltyBoundToInterval(const double& var, const double& a,
                                     const double& b, const double& epsilon) {
  if (var < a + epsilon) {
    return (-var + (a + epsilon));
  }
  if (var <= b - epsilon) {
    return 0.;
  } else {
    return (var - (b - epsilon));
  }
}

/**
 * @brief Linear penalty function for bounding \c var from below: \f$ a < var
 * \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundFromBelowDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not
 * satisfied
 */
inline double PenaltyBoundFromBelow(const double& var, const double& a,
                                    const double& epsilon) {
  if (var >= a + epsilon) {
    return 0.;
  } else {
    return (-var + (a + epsilon));
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the
 * interval \f$ -a < var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double PenaltyBoundToIntervalDerivative(const double& var,
                                               const double& a,
                                               const double& epsilon) {
  if (var < -a + epsilon) {
    return -1;
  }
  if (var <= a - epsilon) {
    return 0.;
  } else {
    return 1;
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the
 * interval \f$ a < var < b \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double PenaltyBoundToIntervalDerivative(const double& var,
                                               const double& a, const double& b,
                                               const double& epsilon) {
  if (var < a + epsilon) {
    return -1;
  }
  if (var <= b - epsilon) {
    return 0.;
  } else {
    return 1;
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var from
 * below: \f$ a < var \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see PenaltyBoundFromBelow
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double PenaltyBoundFromBelowDerivative(const double& var,
                                              const double& a,
                                              const double& epsilon) {
  if (var >= a + epsilon) {
    return 0.;
  } else {
    return -1;
  }
}

}  // namespace planning
}  // namespace century
