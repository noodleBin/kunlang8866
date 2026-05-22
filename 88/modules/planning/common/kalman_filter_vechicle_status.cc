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
 * @file kalman_filter_vechicle_status.cc
 **/

#include "modules/planning/common/kalman_filter_vechicle_status.h"

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

namespace {
constexpr double KDefaultUnitTime = 0.1;
constexpr double kInitDistanceVariance = 0.01;
constexpr double kInitSpeedVariance = 0.01;
constexpr double kInitAccelerationVariance = 0.01;
}  // namespace

KalmanFilterForVehicleStatus::KalmanFilterForVehicleStatus() {
  Eigen::Matrix<double, 3, 1> init_state;
  Eigen::Matrix<double, 3, 3> cov_init_state;
  Eigen::Matrix<double, 3, 3> transform_matrix;
  Eigen::Matrix<double, 3, 3> cov_transform_noise;
  Eigen::Matrix<double, 3, 3> observation_matrix;
  Eigen::Matrix<double, 3, 3> cov_observation_noise;
  Eigen::Matrix<double, 3, 1> control_matrix;
  init_state.setZero();
  // Initial state belief covariance
  cov_init_state.setZero();
  cov_init_state(0, 0) = kInitDistanceVariance;
  cov_init_state(1, 1) = kInitSpeedVariance;
  cov_init_state(2, 2) = kInitAccelerationVariance;
  // Transition matrix
  transform_matrix.setIdentity();
  transform_matrix(0, 1) = KDefaultUnitTime;
  transform_matrix(0, 2) = 0.5 * KDefaultUnitTime * KDefaultUnitTime;
  transform_matrix(1, 2) = KDefaultUnitTime;
  // Covariance of the state transition noise
  cov_transform_noise.setZero();
  cov_transform_noise(0, 0) = FLAGS_transform_distance_variance;
  cov_transform_noise(1, 1) = FLAGS_transform_speed_variance;
  cov_transform_noise(2, 2) = FLAGS_transform_acceleration_variance;
  // Observation matrix
  observation_matrix.setIdentity();
  // Covariance of observation noise
  cov_observation_noise.setIdentity();
  cov_observation_noise(0, 0) = FLAGS_observation_distance_variance;
  cov_observation_noise(1, 1) = FLAGS_observation_speed_variance;
  cov_observation_noise(2, 2) = FLAGS_observation_acceleration_variance;
  // Control matrix designed by constant Jerk
  control_matrix(0, 0) =
      KDefaultUnitTime * KDefaultUnitTime * KDefaultUnitTime / 6.0;
  control_matrix(1, 0) = KDefaultUnitTime * KDefaultUnitTime * 0.5;
  control_matrix(2, 0) = KDefaultUnitTime;

  // set kalman filter
  kf_.SetStateEstimate(init_state, cov_init_state);
  kf_.SetTransitionMatrix(transform_matrix);
  kf_.SetTransitionNoise(cov_transform_noise);
  kf_.SetObservationMatrix(observation_matrix);
  kf_.SetObservationNoise(cov_observation_noise);
  kf_.SetControlMatrix(control_matrix);
}

void KalmanFilterForVehicleStatus::EstimateWithMeasurement(
    double diff_time, double distance, double speed, double acc, double pulse) {
  Predict(diff_time, pulse);
  Correct(distance, speed, acc);
  auto estimate = kf_.GetStateEstimate();
  estimate_state_ = {estimate(0), estimate(1), estimate(2)};
}

void KalmanFilterForVehicleStatus::EstimateWithoutMeasurement(double diff_time,
                                                              double pulse) {
  Predict(diff_time, pulse);
  auto estimate = kf_.GetStateEstimate();
  estimate_state_ = {estimate(0), estimate(1), estimate(2)};
}

void KalmanFilterForVehicleStatus::Predict(double diff_time, double pulse) {
  Eigen::Matrix<double, 1, 1> u;
  u(0, 0) = pulse;
  if (diff_time_ != diff_time) {
    diff_time_ = diff_time;
    // Update transition matrix
    UpdateTransformMatrix();
    // Update control matrix
    UpdateControlMatrix();
  }
  kf_.Predict(u);
}

void KalmanFilterForVehicleStatus::Correct(double distance, double speed,
                                           double acc) {
  Eigen::Matrix<double, 3, 1> measurement;
  measurement(0, 0) = distance;
  measurement(1, 0) = speed;
  measurement(2, 0) = acc;
  kf_.Correct(measurement);
}

void KalmanFilterForVehicleStatus::UpdateControlMatrix() {
  // Update control matrix
  // Control matrix designed by constant Jerk
  Eigen::Matrix<double, 3, 1> control_matrix;
  control_matrix(0, 0) = diff_time_ * diff_time_ * diff_time_ / 6.0;
  control_matrix(1, 0) = diff_time_ * diff_time_ * 0.5;
  control_matrix(2, 0) = diff_time_;
  kf_.SetControlMatrix(control_matrix);
}

void KalmanFilterForVehicleStatus::UpdateTransformMatrix() {
  // Update transition matrix according to diff_time
  Eigen::Matrix<double, 3, 3> transform_matrix;
  transform_matrix.setIdentity();
  transform_matrix(0, 1) = diff_time_;
  transform_matrix(0, 2) = 0.5 * diff_time_ * diff_time_;
  transform_matrix(1, 2) = diff_time_;
  kf_.SetTransitionMatrix(transform_matrix);
}

}  // namespace planning
}  // namespace century
