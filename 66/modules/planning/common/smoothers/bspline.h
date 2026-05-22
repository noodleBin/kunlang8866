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
 * @file
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "modules/planning/common/frame.h"

namespace century {
namespace planning {
enum Type {
  uniform = 0,  //
  quniform = 1  //
};
struct BSPoint {
  double x;
  double y;
};

class Bspline {
 public:
  Bspline(int _k, int type, const std::vector<BSPoint>& _p);
  ~Bspline();
  double BsplineBfunc(int i, int k, double uu);
  std::vector<BSPoint> creatBspline();

 public:
  int k_;                  // b spline degree
  int n_;                  // control point size -1
  std::vector<double> u_;  //
  double delta_u_ = 0.02;  //
  double uBegin_;
  double uEnd_;
  std::vector<BSPoint> p_;       // control point
  std::vector<BSPoint> pTrack_;  // smooth point
};

}  // namespace planning
}  // namespace century
