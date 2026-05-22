/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file curve_math.cc
 **/

#include "modules/planning/math/curve_math.h"

#include <cmath>
#include <limits>

#include "modules/common/util/util.h"

namespace century {
namespace planning {

// kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
double CurveMath::ComputeCurvature(const double dx, const double d2x,
                                   const double dy, const double d2y) {
  if (common::util::IsFloatEqual(dx, 0.0) &&
      common::util::IsFloatEqual(dy, 0.0)) {
    AERROR << "the dx and dy is zero!";
    return std::numeric_limits<double>::max();
  }
  const double a = dx * d2y - dy * d2x;
  auto norm_square = dx * dx + dy * dy;
  auto norm = std::sqrt(norm_square);
  const double b = norm * norm_square;
  return a / b;
}

double CurveMath::ComputeCurvatureDerivative(const double dx, const double d2x,
                                             const double d3x, const double dy,
                                             const double d2y,
                                             const double d3y) {
  const double a = dx * d2y - dy * d2x;
  const double b = dx * d3y - dy * d3x;
  const double c = dx * d2x + dy * d2y;
  const double d = dx * dx + dy * dy;

  if (common::util::IsFloatEqual(d, 0.0)) {
    AERROR << "the d is zero!";
    return std::numeric_limits<double>::max();
  }
  return (b * d - 3.0 * a * c) / (d * d * d);
}

}  // namespace planning
}  // namespace century
