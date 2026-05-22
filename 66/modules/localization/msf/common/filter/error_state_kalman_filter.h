/******************************************************************************
 * Copyright 2023 The Move-X Authors. All Rights Reserved.
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
#pragma once
#include "kalman_filter.h"
#include "third_party/manif/include/manif/manif.h"
#include "third_party/mmath/linear_algebra.h"
#include "third_party/mmath/se3.h"
#include "third_party/mmath/so3.h"

#include "cyber/cyber.h"

namespace century {
namespace loc {

namespace {
// indices:
constexpr int K_DIM_STATE = 15;
constexpr int K_INDEX_ERROR_POS = 0;
constexpr int K_INDEX_ERROR_VEL = 3;
constexpr int K_INDEX_ERROR_ORI = 6;
constexpr int K_INDEX_ERROR_BIAS_ACCEL = 9;
constexpr int K_INDEX_ERROR_BIAS_GYRO = 12;
constexpr int K_DIM_PROCESS_NOISE = 18;
constexpr int K_INDEX_NOISE_ACCEL_PRE = 0;
constexpr int K_INDEX_NOISE_ACCEL = 3;
constexpr int K_INDEX_NOISE_GYRO_PRE = 6;
constexpr int K_INDEX_NOISE_GYRO = 9;
constexpr int K_INDEX_NOISE_BIAS_ACCEL = 12;
constexpr int K_INDEX_NOISE_BIAS_GYRO = 15;
constexpr int K_SIZE_POSITION = 3;

// dimensions:
constexpr int K_DIM_MEASUREMENT_POSE = 6;
constexpr int K_DIM_MEASUREMENT_POSE_NOISE = 6;
constexpr int K_DIM_MEASUREMENT_VEL = 3;
constexpr int K_DIM_MEASUREMENT_VEL_NOISE = 3;

Eigen::Vector3d translation_base_imu = {-1.48, -0.85, 0.55};
}  // namespace

class ErrorStateKalmanFilter : public KalmanFilter {
 public:
  explicit ErrorStateKalmanFilter(const YAML::Node& node);

  /**
   * @brief  init filter
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  void Init(const Eigen::Matrix4d& init_pose, const Eigen::Vector3d& vel,
            const ImuData& imu_data);

  /**
   * @brief  get odometry estimation
   * @param  pose, init pose
   * @param  vel, init vel
   * @return void
   */
  void GetOdometry(Eigen::Matrix4d& pose, Eigen::Vector3d& linear_vel,
                   Eigen::Vector3d& linear_acc, Eigen::Vector3d& ang_vel);

  double GetLatestTimeStamp();

  /**
   * @brief  get covariance estimation
   * @param  cov, covariance output
   * @return void
   */
  void GetCovariance(Cov& cov);

  /**
   * @brief get the deviation of the pose
   * @param Deviation, output deviation
   * @return void
   */
  void GetDeviation(Deviation* devi);

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  void UpdateObservabilityAnalysis(const double& time,
                                   const MeasurementType& measurement_type);

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  bool SaveObservabilityAnalysis(const MeasurementType& measurement_type);

  bool AddNewNode(const ImuData& imu_data);

  bool AddMeasurement(const Measurement& measurement);

  Eigen::Vector3d GetLinearAcceleration();

  Eigen::Vector3d GetAngularVelocity();

  Eigen::Vector3d GetLinearVelocity();

 private:
  // state:
  using VectorX = Eigen::Matrix<double, K_DIM_STATE, 1>;
  using MatrixP = Eigen::Matrix<double, K_DIM_STATE, K_DIM_STATE>;
  using VectorN = Eigen::Matrix<double, K_DIM_PROCESS_NOISE, 1>;

  // process equation:
  using MatrixF = Eigen::Matrix<double, K_DIM_STATE, K_DIM_STATE>;
  using MatrixB = Eigen::Matrix<double, K_DIM_STATE, K_DIM_PROCESS_NOISE>;
  using MatrixQ =
      Eigen::Matrix<double, K_DIM_PROCESS_NOISE, K_DIM_PROCESS_NOISE>;

  // measurement equation:
  using MatrixGPose =
      Eigen::Matrix<double, K_DIM_MEASUREMENT_POSE, K_DIM_STATE>;
  using MatrixCPose = Eigen::Matrix<double, K_DIM_MEASUREMENT_POSE,
                                    K_DIM_MEASUREMENT_POSE_NOISE>;
  using MatrixRPose = Eigen::Matrix<double, K_DIM_MEASUREMENT_POSE_NOISE,
                                    K_DIM_MEASUREMENT_POSE_NOISE>;

  using MatrixGVel = Eigen::Matrix<double, K_DIM_MEASUREMENT_VEL, K_DIM_STATE>;
  using MatrixCVel =
      Eigen::Matrix<double, K_DIM_MEASUREMENT_VEL, K_DIM_MEASUREMENT_VEL_NOISE>;
  using MatrixRVel = Eigen::Matrix<double, K_DIM_MEASUREMENT_VEL_NOISE,
                                   K_DIM_MEASUREMENT_VEL_NOISE>;

  // measurement:
  using VectorYPose = Eigen::Matrix<double, K_DIM_MEASUREMENT_POSE, 1>;
  using VectorYVel = Eigen::Matrix<double, K_DIM_MEASUREMENT_VEL, 1>;

  // Kalman gain:
  using MatrixKPose =
      Eigen::Matrix<double, K_DIM_STATE, K_DIM_MEASUREMENT_POSE>;

  // state observality matrix:
  using MatrixQPose =
      Eigen::Matrix<double, K_DIM_STATE * K_DIM_MEASUREMENT_POSE, K_DIM_STATE>;

  using MatrixQVel =
      Eigen::Matrix<double, K_DIM_STATE * K_DIM_MEASUREMENT_VEL, K_DIM_STATE>;

  struct Odometry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d accl_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  };

  struct ErrorState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VectorX X;
    VectorX X_store;
    MatrixP P;
  };

  struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // double stacked state
    ErrorState state;

    Odometry odometry;

    // input used to move to next capture, fix to 6(omega and acc)
    ImuData input;

    // all measurement
    std::vector<KalmanFilter::Measurement> measurements;
  };
  using StateNodeBuffer = std::deque<StateNode>;

  bool Predict(StateNodeBuffer::iterator node_prev,
               StateNodeBuffer::iterator node_curr);

  bool UpdateNodeRange(StateNodeBuffer::iterator node_start,
                       StateNodeBuffer::iterator node_end);

  bool Update(StateNodeBuffer::iterator& node_curr);

  bool ApplyMeasurement(StateNodeBuffer::iterator& node_curr,
                        Measurement measurement);

  bool ApplyMeasurementPose(StateNodeBuffer::iterator& node_curr,
                            const Eigen::Matrix4d& T_nb, Eigen::VectorXd& Y,
                            Eigen::MatrixXd& G, Eigen::MatrixXd& K);

  bool ApplyMeasurementVel(StateNodeBuffer::iterator& node_curr,
                           const Eigen::Vector3d& v_b, Eigen::VectorXd& Y,
                           Eigen::MatrixXd& G, Eigen::MatrixXd& K);

  bool ApplyMeasurementENUVel(StateNodeBuffer::iterator& node_curr,
                           const Eigen::Vector3d& v_enu, Eigen::VectorXd& Y,
                           Eigen::MatrixXd& G, Eigen::MatrixXd& K);

  /**
   * @brief  get unbiased angular velocity in body frame
   * @param  angular_vel, angular velocity measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased angular velocity in body frame
   */
  Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d& angular_vel,
                                        const Eigen::Matrix3d& R);
  /**
   * @brief  get unbiased linear acceleration in navigation frame
   * @param  linear_acc, linear acceleration measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased linear acceleration in navigation frame
   */
  Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d& linear_acc,
                                       const Eigen::Matrix3d& R);

  void EliminateError(StateNodeBuffer::iterator& node_curr);

  void DropHistory();

  bool FindClosestStateNode(const double timestamp,
                            StateNodeBuffer::iterator& node_iter);

  /**
   * @brief  is covariance stable
   * @param  INDEX_OFSET, state index offset
   * @param  THRESH, covariance threshold, defaults to 1.0e-5
   * @return void
   */
  bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

  /**
   * @brief  reset filter state
   * @param  void
   * @return void
   */

  void ResetState(StateNodeBuffer::iterator& node_curr);

  void ResetStoreState(StateNodeBuffer::iterator& node_curr);
  /**
   * @brief  reset filter covariance
   * @param  void
   * @return void
   */
  void ResetCovariance(void);

  /**
   * @brief  get Q analysis for pose measurement
   * @param  void
   * @return void
   */
  void GetQPose(Eigen::MatrixXd& Q, Eigen::VectorXd& Y);

 private:
  // state:
  VectorX X_ = VectorX::Zero();
  // store the X_'s state in correction process
  VectorX X_store_ = VectorX::Zero();
  VectorN N_ = VectorN::Zero();
  MatrixP P_ = MatrixP::Zero();
  // process & measurement equations:
  MatrixF F_ = MatrixF::Zero();
  MatrixB B_ = MatrixB::Zero();
  MatrixQ Q_ = MatrixQ::Zero();

  MatrixGPose GPose_ = MatrixGPose::Zero();
  MatrixCPose CPose_ = MatrixCPose::Zero();
  MatrixRPose RPose_ = MatrixRPose::Zero();
  MatrixQPose QPose_ = MatrixQPose::Zero();

  MatrixGVel GVel_ = MatrixGVel::Zero();
  MatrixCVel CVel_ = MatrixCVel::Zero();
  MatrixRVel RVel_ = MatrixRVel::Zero();
  MatrixQVel QVel_ = MatrixQVel::Zero();

  // measurement:
  VectorYPose YPose_;
  VectorYVel YVel_;

  StateNodeBuffer state_node_buffer_;
  std::mutex state_node_mutex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace loc
}  // namespace century
