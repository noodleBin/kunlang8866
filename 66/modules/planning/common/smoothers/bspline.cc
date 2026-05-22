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

#include "modules/planning/common/smoothers/bspline.h"

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace {
constexpr double kEpsilon = 0.001;
constexpr int kInterNum = 5;
}  // namespace
Bspline::Bspline(int _k, int type, const std::vector<BSPoint>& _p) {
  CHECK_GT(_k, 0);
  k_ = _k;
  n_ = _p.size() - 1;
  int n = n_;
  int k = k_;
  if (k_ > n_ + 1 || _p.empty()) {
    AINFO << "Bspline init error!";
    return;
  }

  p_ = _p;
  CHECK_NE(n + 1, 0.0);
  delta_u_ = std::max(1.0 / (n + 1), kEpsilon);
  double u_tmp = 0.0;
  u_.push_back(u_tmp);

  if (type == uniform) {
    CHECK_NE(k + n, 0.0);
    double dis_u = 1.0 / (k + n);
    for (int i = 1; i < n + k + 1; ++i) {
      u_tmp += dis_u;
      u_.push_back(u_tmp);
    }
  } else if (type == quniform) {
    int j = 3;
    int devide_v = k + n - (j - 1) * 2;
    CHECK_NE(devide_v, 0);
    double dis_u = 1.0 / devide_v;
    for (int i = 1; i < j; ++i) {
      u_.push_back(u_tmp);
    }
    for (int i = j; i < n + k - j + 2; ++i) {
      u_tmp += dis_u;
      u_.push_back(u_tmp);
    }
    for (int i = n + k - j + 2; i < n + k + 1; ++i) {
      u_.push_back(u_tmp);
    }
  }
  uBegin_ = u_[k - 1];
  uEnd_ = u_[n + 1];
  ADEBUG << "uBegin_ " << uBegin_ << "uEnd_ " << uEnd_;
}

Bspline::~Bspline() {
  p_.clear();
  u_.clear();
  pTrack_.clear();
}

double Bspline::BsplineBfunc(int i, int k, double uu) {
  double Bfunc = 0.0;
  if (k == 1) {
    if (u_[i] <= uu && uu < u_[i + 1]) {
      Bfunc = 1.0;
    } else {
      Bfunc = 0.0;
    }
  } else if (k >= 2) {
    double A = 0.0;
    double B = 0.0;
    if (u_[i + k - 1] - u_[i] < kEpsilon) {
      A = 0.0;
    } else {
      A = (uu - u_[i]) /
          std::fmax(FLAGS_numerical_epsilon, (u_[i + k - 1] - u_[i]));
    }

    if (u_[i + k] - u_[i + 1] < kEpsilon) {
      B = 0.0;  //
    } else {
      B = (u_[i + k] - uu) /
          std::fmax(FLAGS_numerical_epsilon, (u_[i + k] - u_[i + 1]));
    }

    Bfunc = A * BsplineBfunc(i, k - 1, uu) + B * BsplineBfunc(i + 1, k - 1, uu);
  }

  return Bfunc;
}

std::vector<BSPoint> Bspline::creatBspline() {
  double uu = 0.0;
  // j is cycle num,no mean

  for (size_t j = 0; j <= p_.size() * kInterNum; ++j) {
    BSPoint Pu = {0.0, 0.0};
    if (uu >= 1) {
      break;
    }
    for (int i = 0; i < n_ + 1; ++i) {
      double xtmp = p_[i].x;
      double ytmp = p_[i].y;
      double BfuncTmp = BsplineBfunc(i, k_, uu);
      Pu.x += xtmp * BfuncTmp;
      Pu.y += ytmp * BfuncTmp;
    }
    pTrack_.push_back(Pu);
    uu += delta_u_ / kInterNum;
    ADEBUG << "uu " << uu;
  }
  ADEBUG << "pTrack_ size" << pTrack_.size();
  return pTrack_;
}

}  // namespace planning
}  // namespace century
