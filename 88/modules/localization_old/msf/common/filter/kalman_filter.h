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

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>

#include "third_party/mmath/se3.h"
#include "third_party/mmath/so3.h"

namespace century {
namespace loc {

struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0;
  Eigen::Vector3d linear_acc;
  Eigen::Vector3d ang_vel;
};
using ImuDataPtr = std::shared_ptr<ImuData>;

// it is the vehicle center pose parsed from the ins.  
struct InsLocData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0;
  mmath::SE3 map_veh_pose;
  Eigen::Vector3d linear_vel;
  Eigen::Matrix<double, 9, 1> noise_vec;
};
using InsLocDataPtr = std::shared_ptr<InsLocData>;

struct LidarLocData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0;
  double fitness_score = 0.0;
  bool is_converged = false;
  mmath::SE3 lidar_loc_pose;
  // Eigen::Matrix<double, 6, 6> cov;
};

using LidarLocDataPtr = std::shared_ptr<LidarLocData>;

struct ChassisData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0;
  double bridge_1_left_wheel_angle = 0.0;
  double bridge_1_right_wheel_angle = 0.0;
  double bridge_4_left_wheel_angle = 0.0;
  double bridge_4_right_wheel_angle = 0.0;
  double wheel_speed_0 = 0.0;
  double wheel_speed_1 = 0.0;
  double wheel_speed_2 = 0.0;
  double wheel_speed_3 = 0.0;
};

using ChassisDataPtr = std::shared_ptr<ChassisData>;

class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /** @brief enum for observation type */
  enum MeasurementType {
    POSE = 0,
    POSE_VEL,
    BODY_VEL,
    ZERO_ENU_VEL,
    ENU_VEL,
    POSI,
    POSI_VEL
  };

  /** @brief Kalman filter measurement data */
  struct Measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // timestamp:
    double time{0.0};

    MeasurementType measurement_type;

    // a. pose observation, lidar/visual frontend:
    Eigen::Matrix4d T_nb = Eigen::Matrix4d::Zero();
    // b. body frame velocity observation, odometer:
    Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
    // c. body frame angular velocity, needed by motion constraint:
    Eigen::Vector3d w_b = Eigen::Vector3d::Zero();
    // d. magnetometer:
    Eigen::Vector3d B_b = Eigen::Vector3d::Zero();
    // e. ENU velocity observation, odometer::
    Eigen::Vector3d v_enu = Eigen::Vector3d::Zero();
  };

  /** @brief Kalman filter process covariance data */
  struct Cov {
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } pos;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } vel;
    // here quaternion is used for orientation representation:
    struct {
      double w{1.0};
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } ori;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } gyro_bias;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } accel_bias;
  };

  struct Deviation {
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } pos;

    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } ori;
  };

  /**
   * @brief  init filter
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  virtual void Init(const Eigen::Matrix4d &init_pose,
                    const Eigen::Vector3d &vel, const ImuData &imu_data) = 0;

  // /**
  //  * @brief  update state & covariance estimation, Kalman prediction
  //  * @param  imu_data, input IMU measurements
  //  * @return true if success false otherwise
  //  */
  // virtual bool Update(const ImuData &imu_data) = 0;

  // /**
  //  * @brief  correct state & covariance estimation, Kalman correction
  //  * @param  measurement_type, input measurement type
  //  * @param  measurement, input measurement
  //  * @return void
  //  */
  // virtual bool Correct(const MeasurementType &measurement_type,
  //                      const Measurement &measurement) = 0;

  /**
   * @brief  get filter time
   * @return filter time as double
   */
  double GetTime(void) const { return time_; }

  /**
   * @brief  get odometry estimation
   * @param  pose, output pose
   * @param  vel, output vel
   * @return void
   */
  virtual void GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &linear_vel,
                           Eigen::Vector3d &linear_acc,
                           Eigen::Vector3d &ang_vel) = 0;  

  /**
   * @brief  get covariance estimation
   * @param  cov, output covariance
   * @return void
   */
  virtual void GetCovariance(Cov &cov) = 0;  // NOLINT

  /**
   * @brief get the deviation of the pose
   * @param Deviation, output deviation
   * @return void
   */
  virtual void GetDeviation(Deviation *devi) = 0;

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual void UpdateObservabilityAnalysis(
      const double &time, const MeasurementType &measurement_type) = 0;

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual bool SaveObservabilityAnalysis(
      const MeasurementType &measurement_type) = 0;

 protected:
  KalmanFilter() {}

  static void AnalyzeQ(const int &DIM_STATE, const double &time,
                       const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
                       std::vector<std::vector<double>> &data);  // NOLINT

  static void WriteAsCSV(const int &DIM_STATE,
                         const std::vector<std::vector<double>> &data,
                         const std::string &filename);

  // time:
  double time_{0.0};

  // data buff:
  std::deque<ImuData, Eigen::aligned_allocator<ImuData>> imu_data_buff_;

  // earth constants:
  Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  // observability analysis:
  struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<double> time;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> Q;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> Y;
  } observability_;

  // hyper-params:
  // a. earth constants:
  struct {
    double gravity_magnitude{0.0};
    double rotation_speed{0.0};
    double latitude{0.0};
    double longitude{0.0};
  } earth_;
  // b. prior state covariance, process & measurement noise:
  struct {
    struct {
      double posi{0.0};
      double vel{0.0};
      double ori{0.0};
      double epslion{0.0};
      double delta{0.0};
    } prior;
    struct {
      double gyro{0.0};
      double accel{0.0};
      double bias_accel{0.0};
      double bias_gyro{0.0};
    } process;
    struct {
      struct {
        double posi{0.0};
        double ori{0.0};
      } pose;
      double posi{0.0};
      double vel{0.0};
      double ori{0.0};
    } measurement;
  } cov_;
};
}  // namespace loc
}  // namespace century
