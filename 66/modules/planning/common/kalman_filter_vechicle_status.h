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
 * @file kalman_filter_vechicle_status.h
 **/

#pragma once

#include <array>
#include <string>

#include "modules/common/math/kalman_filter.h"

namespace century {
namespace planning {

class KalmanFilterForVehicleStatus {
 public:
  KalmanFilterForVehicleStatus();

  void EstimateWithMeasurement(double diff_time, double distance, double speed,
                               double acc, double pulse = 0.0);
  void EstimateWithoutMeasurement(double diff_time, double pulse = 0.0);

  void GetStateEstimate(std::array<double, 3UL> *ptr_estimate_state) {
    CHECK_NOTNULL(ptr_estimate_state);
    *ptr_estimate_state = estimate_state_;
  }

  // the meaning of each letter in debug string is below:
  // F is transition matrix
  // B is control matrix
  // H is observation matrix
  // Q is the covariance matrix of the transition noise
  // R is the covariance matrix of the observation noise
  // x is mean of current state belief distribution
  // P is the covariance matrix of current state belief distribution
  void GetDebugString(std::string *debug) {
    *debug = "Kalman Filter State:\n" + kf_.DebugString();
  }

 private:
  void UpdateControlMatrix();

  void UpdateTransformMatrix();

  void Predict(double diff_time, double pulse = 0.0);

  void Correct(double distance, double speed, double acc = 0.0);

 private:
  common::math::KalmanFilter<double, 3, 3, 1> kf_;
  double diff_time_ = 0.0;
  std::array<double, 3UL> estimate_state_;
};

}  // namespace planning
}  // namespace century
