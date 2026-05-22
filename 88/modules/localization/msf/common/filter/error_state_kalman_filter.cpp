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
#include "error_state_kalman_filter.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <ostream>

#include "glog/logging.h"

namespace century {
namespace loc {
ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node& node) {
  // a. earth constants:
  earth_.gravity_magnitude = node["earth"]["gravity_magnitude"].as<double>();
  earth_.latitude = node["earth"]["latitude"].as<double>();

  // b. prior state covariance:
  cov_.prior.posi = node["covariance"]["prior"]["pos"].as<double>();
  cov_.prior.vel = node["covariance"]["prior"]["vel"].as<double>();
  cov_.prior.ori = node["covariance"]["prior"]["ori"].as<double>();
  cov_.prior.epslion = node["covariance"]["prior"]["epsilon"].as<double>();
  cov_.prior.delta = node["covariance"]["prior"]["delta"].as<double>();

  // c. process noise:
  cov_.process.accel = node["covariance"]["process"]["accel"].as<double>();
  cov_.process.gyro = node["covariance"]["process"]["gyro"].as<double>();
  cov_.process.bias_accel =
      node["covariance"]["process"]["bias_accel"].as<double>();
  cov_.process.bias_gyro =
      node["covariance"]["process"]["bias_gyro"].as<double>();

  // d. measurement noise:
  cov_.measurement.pose.posi =
      node["covariance"]["measurement"]["pose"]["pos"].as<double>();
  cov_.measurement.pose.ori =
      node["covariance"]["measurement"]["pose"]["ori"].as<double>();
  cov_.measurement.vel = node["covariance"]["measurement"]["vel"].as<double>();

  // prompt:
  AINFO << "Error-State Kalman Filter params:";
  AINFO << "gravity magnitude: " << earth_.gravity_magnitude
        << "\n latitude: " << earth_.latitude
        << "\n prior cov. pos.: " << cov_.prior.posi
        << "\n prior cov. vel.: " << cov_.prior.vel
        << "\n prior cov. ori: " << cov_.prior.ori
        << "\n prior cov. epsilon.: " << cov_.prior.epslion
        << "\n prior cov. delta.: " << cov_.prior.delta
        << "\n process noise gyro.: " << cov_.process.gyro
        << "\n process noise accel.: " << cov_.process.accel
        << "\n measurement noise pose.: "
        << " pos: " << cov_.measurement.pose.posi
        << ", ori.: " << cov_.measurement.pose.ori;

  // init filter:
  // a. earth constants:
  g_ = Eigen::Vector3d(0.0, 0.0, earth_.gravity_magnitude);
  // b. prior state & covariance:
  // ResetState();
  ResetCovariance();
  N_.segment<3>(K_INDEX_NOISE_ACCEL_PRE) =
      Eigen::Vector3d::Constant(cov_.process.accel);
  N_.segment<3>(K_INDEX_NOISE_ACCEL) =
      Eigen::Vector3d::Constant(cov_.process.accel);
  N_.segment<3>(K_INDEX_NOISE_GYRO_PRE) =
      Eigen::Vector3d::Constant(cov_.process.gyro);
  N_.segment<3>(K_INDEX_NOISE_GYRO) =
      Eigen::Vector3d::Constant(cov_.process.gyro);
  N_.segment<3>(K_INDEX_NOISE_BIAS_ACCEL) =
      Eigen::Vector3d::Constant(cov_.process.bias_accel);
  N_.segment<3>(K_INDEX_NOISE_BIAS_GYRO) =
      Eigen::Vector3d::Constant(cov_.process.bias_gyro);
  AINFO << "-------------N_------------- \n"
        << std::fixed << std::scientific << N_.transpose();
  // c. process noise:
  Q_.block<3, 3>(K_INDEX_NOISE_ACCEL_PRE, K_INDEX_NOISE_ACCEL_PRE) =
      cov_.process.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(K_INDEX_NOISE_ACCEL, K_INDEX_NOISE_ACCEL) =
      cov_.process.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(K_INDEX_NOISE_GYRO_PRE, K_INDEX_NOISE_GYRO_PRE) =
      cov_.process.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(K_INDEX_NOISE_GYRO, K_INDEX_NOISE_GYRO) =
      cov_.process.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(K_INDEX_NOISE_BIAS_ACCEL, K_INDEX_NOISE_BIAS_ACCEL) =
      cov_.process.bias_accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(K_INDEX_NOISE_BIAS_GYRO, K_INDEX_NOISE_BIAS_GYRO) =
      cov_.process.bias_gyro * Eigen::Matrix3d::Identity();
  AINFO << "-------------Q_------------- \n"
        << std::fixed << std::scientific << Q_;
  // d. measurement noise:
  RPose_.block<3, 3>(0, 0) =
      cov_.measurement.pose.posi * Eigen::Matrix3d::Identity();
  RPose_.block<3, 3>(3, 3) =
      cov_.measurement.pose.ori * Eigen::Matrix3d::Identity();
  AINFO << "------------RPose_----------- \n"
        << std::fixed << std::scientific << RPose_;

  RVel_.block<3, 3>(0, 0) = cov_.measurement.vel * Eigen::Matrix3d::Identity();
  AINFO << "------------RVel_------------ \n"
        << std::fixed << std::scientific << RVel_;

  // e. process equation:
  F_.block<3, 3>(K_INDEX_ERROR_POS, K_INDEX_ERROR_VEL) =
      Eigen::Matrix3d::Identity();
  F_.block<3, 3>(K_INDEX_ERROR_ORI, K_INDEX_ERROR_GYRO) =
      -Eigen::Matrix3d::Identity();

  B_.block<3, 3>(K_INDEX_ERROR_ORI, K_INDEX_NOISE_GYRO) =
      Eigen::Matrix3d::Identity();
  B_.block<3, 3>(K_INDEX_ERROR_ACCEL, K_INDEX_NOISE_BIAS_ACCEL) =
      Eigen::Matrix3d::Identity();
  B_.block<3, 3>(K_INDEX_ERROR_GYRO, K_INDEX_NOISE_BIAS_GYRO) =
      Eigen::Matrix3d::Identity();

  // f. measurement equation:
  GPose_.block<3, 3>(0, K_INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, K_INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  // init soms:
  QPose_.block<K_DIM_MEASUREMENT_POSE, K_DIM_STATE>(0, 0) = GPose_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ErrorStateKalmanFilter::Init(const Eigen::Matrix4d& init_pose,
                                  const Eigen::Vector3d& vel,
                                  const ImuData& imu_data) {
  // init odometry:
  // Eigen::Matrix3d C_nb = init_pose.block<3, 3>(0, 0);
  // a. init C_nb using IMU estimation:
  Eigen::Affine3d imu_pose = Eigen::Affine3d(init_pose);

  std::lock_guard<std::mutex> lock(state_node_mutex_);
  StateNode state_node;
  state_node.state.P = P_;
  state_node.state.X = X_;
  state_node.state.X_store = X_store_;
  state_node.odometry.pose = imu_pose.matrix();
  // b. convert flu velocity into navigation frame:
  state_node.odometry.vel = /*C_nb * */vel;
  state_node.input = imu_data;
  state_node_buffer_.emplace_front(state_node);

  AINFO
      << "\nKalman Filter Inited at " << std::fixed << imu_data.timestamp
      << "\nInit Position(VEHICLE): \n"
      << init_pose.block<3, 1>(0, 3).transpose() << "\nInit Position(IMU): \n"
      << state_node_buffer_.front().odometry.pose.block<3, 1>(0, 3).transpose();
}
bool ErrorStateKalmanFilter::AddNewNode(const ImuData& imu_data) {
  // AINFO << "ESKF Add New Node--> timestamp: " << std::fixed
  //       << imu_data.timestamp;
  std::lock_guard<std::mutex> lock(state_node_mutex_);
  auto last_state_node = state_node_buffer_.front();
  if (imu_data.timestamp > last_state_node.input.timestamp) {
    StateNode state_node;
    state_node.input = imu_data;
    state_node_buffer_.emplace_front(state_node);

    auto node_prev = std::next(state_node_buffer_.begin());
    auto node_curr = state_node_buffer_.begin();
    Predict(node_prev, node_curr);
    DropHistory();
    // std::for_each(state_node_buffer_.begin(), state_node_buffer_.end(),
    //               [](const StateNode &node) {
    //                 AINFO << "State_node_buffer pose: " << std::fixed
    //                       << node.input.timestamp << "-->"
    //                       << node.odometry.pose.block<3, 1>(0,
    //                       3).transpose();
    //               });
    return true;
  }
  return false;
}

bool ErrorStateKalmanFilter::AddMeasurement(const Measurement& measurement) {
  ADEBUG << "ESKF Correct measurement timestamp: " << std::fixed
        << measurement.time;
  ADEBUG << "ESKF Correct measurement type: " << measurement.measurement_type;
  {
    StateNodeBuffer::iterator state_node_closest;
    if (!FindClosestStateNode(measurement.time, state_node_closest)) {
      AWARN << "No state node found for measurement timestamp: " << std::fixed
            << measurement.time;
      AWARN << "The latest state node timestamp: " << std::fixed
            << state_node_buffer_.front().input.timestamp;

      return false;
    }
    AINFO << "found closest state node, timestamp: " << std::fixed
          << state_node_closest->input.timestamp;
    state_node_closest->measurements.emplace_back(measurement);
    Update(state_node_closest);
    UpdateNodeRange(state_node_closest, state_node_buffer_.begin());

    ADEBUG << "State_node_buffer pose: \n";
    std::for_each(
        state_node_buffer_.begin(), state_node_buffer_.end(),
        [](const StateNode& node) {
          ADEBUG
              << std::fixed << node.input.timestamp << "-->"
              << std::setprecision(8)
              << Eigen::Affine3d(node.odometry.pose)
                         .rotation()
                         .eulerAngles(0, 1, 2)
                         .transpose() * mmath::kRadToDeg
              << "||"
              << Eigen::Affine3d(node.odometry.pose).translation().transpose();
        });
  }

  return true;
}

bool ErrorStateKalmanFilter::Predict(StateNodeBuffer::iterator node_prev,
                                     StateNodeBuffer::iterator node_curr) {
  auto accl_0 = node_prev->input.linear_acc;
  auto gyro_0 = node_prev->input.ang_vel;
  auto accl_1 = node_curr->input.linear_acc;
  auto gyro_1 = node_curr->input.ang_vel;
  auto dt = node_curr->input.timestamp - node_prev->input.timestamp;

  ADEBUG << "Predict: " << std::fixed << node_prev->input.timestamp << "-->"
        << node_curr->input.timestamp;
  Eigen::Quaterniond quat(node_prev->odometry.pose.block<3, 3>(0, 0));
  quat.normalize();
  Eigen::Vector3d un_accl_0 = quat * (accl_0 - node_prev->odometry.accl_bias);
  Eigen::Vector3d un_gyro =
      0.5 * (gyro_0 + gyro_1) - node_prev->odometry.gyro_bias;
  Eigen::Quaterniond result_quat =
      quat * Eigen::Quaterniond(1, un_gyro(0) * dt / 2, un_gyro(1) * dt / 2,
                                un_gyro(2) * dt / 2);
  result_quat.normalize();
  Eigen::Vector3d un_accl_1 =
      result_quat * (accl_1 - node_prev->odometry.accl_bias);
  Eigen::Vector3d un_accl = 0.5 * (un_accl_0 + un_accl_1);

  Eigen::Vector3d trans(node_prev->odometry.pose.block<3, 1>(0, 3));
  Eigen::Vector3d result_trans =
      trans + node_prev->odometry.vel * dt + 0.5 * un_accl * dt * dt;
  node_curr->odometry.pose.block<3, 3>(0, 0) = result_quat.toRotationMatrix();
  node_curr->odometry.pose.block<3, 1>(0, 3) = result_trans;
  node_curr->odometry.vel = node_prev->odometry.vel + un_accl * dt;
  //---------------------------
  Eigen::Vector3d w_x = 0.5 * (gyro_0 + gyro_1) - node_prev->odometry.gyro_bias;
  Eigen::Vector3d a_0_x = accl_0 - node_prev->odometry.accl_bias;
  Eigen::Vector3d a_1_x = accl_1 - node_prev->odometry.accl_bias;
  Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

  R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
  R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1),
      a_0_x(0), 0;
  R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1),
      a_1_x(0), 0;

  MatrixF F = MatrixF::Zero();
  F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
  F.block<3, 3>(0, 6) = -0.25 * quat.toRotationMatrix() * R_a_0_x * dt * dt +
                        -0.25 * result_quat.toRotationMatrix() * R_a_1_x *
                            (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt *
                            dt;
  F.block<3, 3>(0, 9) =
      -0.25 * (quat.toRotationMatrix() + result_quat.toRotationMatrix()) * dt *
      dt;
  F.block<3, 3>(0, 12) =
      0.25 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt * dt;

  F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(3, 6) = -0.5 * quat.toRotationMatrix() * R_a_0_x * dt +
                        -0.5 * result_quat.toRotationMatrix() * R_a_1_x *
                            (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
  F.block<3, 3>(3, 9) =
      -0.5 * (quat.toRotationMatrix() + result_quat.toRotationMatrix()) * dt;
  F.block<3, 3>(3, 12) =
      0.5 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt;

  F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - R_w_x * dt;
  F.block<3, 3>(6, 12) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt;

  F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

  MatrixB B = MatrixB::Zero();
  B.block<3, 3>(0, 0) = -0.25 * quat.toRotationMatrix() * dt * dt;
  B.block<3, 3>(0, 3) =
      0.25 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt * dt;
  B.block<3, 3>(0, 6) = -0.25 * result_quat.toRotationMatrix() * dt * dt;
  B.block<3, 3>(0, 9) =
      0.25 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt * dt;

  B.block<3, 3>(3, 0) = -0.5 * quat.toRotationMatrix() * dt;
  B.block<3, 3>(3, 3) =
      0.25 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt;
  B.block<3, 3>(3, 6) = -0.5 * result_quat.toRotationMatrix() * dt;
  B.block<3, 3>(3, 9) =
      0.25 * result_quat.toRotationMatrix() * R_a_1_x * dt * dt;

  B.block<3, 3>(6, 3) = -0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
  B.block<3, 3>(6, 9) = -0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;

  B.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dt;

  B.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dt;

  node_curr->state.X = F * node_prev->state.X;
  node_curr->state.P =
      F * node_prev->state.P * F.transpose() + B * Q_ * B.transpose();
  return true;
}

bool ErrorStateKalmanFilter::UpdateNodeRange(
    StateNodeBuffer::iterator node_start, StateNodeBuffer::iterator node_end) {
  if (node_start == node_end) {
    return true;
  }

  auto node_prev = node_start;
  auto node_curr = node_start;
  for (--node_curr; node_prev != node_end; --node_prev, --node_curr) {
    if (Predict(node_prev, node_curr)) {
      Update(node_curr);
    } else {
      AWARN << "Update capture failed";
      return false;
    }
  }

  return true;
}

bool ErrorStateKalmanFilter::Update(StateNodeBuffer::iterator& node_curr) {
  ADEBUG << "Update: " << std::fixed << node_curr->input.timestamp;

  if (!node_curr->measurements.empty()) {
    for (const auto& measurement : node_curr->measurements) {
      ApplyMeasurement(node_curr, measurement);
    }
    EliminateError(node_curr);
    ResetState(node_curr);
  }
  return true;
}

bool ErrorStateKalmanFilter::ApplyMeasurement(
    StateNodeBuffer::iterator& node_curr, Measurement measurement) {
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;

  switch (measurement.measurement_type) {
    case MeasurementType::POSE:
      ApplyMeasurementPose(node_curr, measurement.T_nb, Y, G, K);
      break;
    case MeasurementType::BODY_VEL:
      ApplyMeasurementVel(node_curr, measurement.v_b, Y, G, K);
      break;
    case MeasurementType::ENU_VEL:
      ApplyMeasurementENUVel(node_curr, measurement.v_enu, Y, G, K);
      break;
    default:
      AERROR << "not support this type.";
      return false;  // TODO(ZXF)
  }

  // perform Kalman correct:
  auto pre_state_x = node_curr->state.X;
  auto pre_state_p = node_curr->state.P;
  node_curr->state.P = (MatrixP::Identity() - K * G) * node_curr->state.P;
  node_curr->state.X = node_curr->state.X + K * (Y - G * node_curr->state.X);
  ADEBUG << "-------------X ------------- \n"
        << std::fixed << std::setprecision(8) << pre_state_x.transpose();
  ADEBUG << "---------updated X---------- \n"
        << std::fixed << std::setprecision(8) << node_curr->state.X.transpose();
  ADEBUG << "-------------P-------------- \n"
        << std::fixed << std::setprecision(8) << pre_state_p;
  ADEBUG << "---------updated P---------- \n"
        << std::fixed << std::setprecision(8) << node_curr->state.P;
  ADEBUG << "-------------K-------------- \n"
        << std::fixed << std::setprecision(8) << K;

  X_store_ = node_curr->state.X;
  return true;
}

bool ErrorStateKalmanFilter::ApplyMeasurementPose(
    StateNodeBuffer::iterator& node_curr, const Eigen::Matrix4d& T_nb,
    Eigen::VectorXd& Y, Eigen::MatrixXd& G, Eigen::MatrixXd& K) {
  // set measurement:
  MatrixGPose GPose;
  MatrixCPose CPose;
  GPose.setZero();
  GPose.block<3, 3>(0, K_INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
  GPose.block<3, 3>(3, K_INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
  G = GPose;
  CPose.setZero();
  CPose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  auto T_n_imu = Eigen::Affine3d(T_nb);
  // set measurement equation:
  // Eigen::Vector3d delta_p =
  //     node_curr->odometry.pose.block<3, 1>(0, 3) - T_n_imu.block<3, 1>(0, 3);
  // Eigen::Matrix3d delta_R = T_n_imu.block<3, 3>(0, 0).transpose() *
  //                               node_curr->odometry.pose.block<3, 3>(0, 0) -
  //                           Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_p =
      node_curr->odometry.pose.block<3, 1>(0, 3) - T_n_imu.translation();
  Eigen::Matrix3d delta_R = T_n_imu.rotation().transpose() *
                                node_curr->odometry.pose.block<3, 3>(0, 0) -
                            Eigen::Matrix3d::Identity();
  Eigen::Vector3d dtheta = mmath::vee(delta_R);
  VectorYPose YPose;
  YPose.block<3, 1>(0, 0) = delta_p;
  YPose.block<3, 1>(3, 0) = dtheta;
  Y = YPose;
  // set Kalman gain:
  K = node_curr->state.P * G.transpose() *
      (G * node_curr->state.P * G.transpose() +
       CPose * RPose_ * CPose.transpose())
          .inverse();
  return true;
}

bool ErrorStateKalmanFilter::ApplyMeasurementVel(
    StateNodeBuffer::iterator& node_curr, const Eigen::Vector3d& v_b,
    Eigen::VectorXd& Y, Eigen::MatrixXd& G, Eigen::MatrixXd& K) {
  // set measurement:
  MatrixGVel GVel;
  MatrixCVel CVel;
  GVel.setZero();
  GVel.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  G = GVel;
  CVel.setZero();
  CVel.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  // set measurement equation:

  AINFO << "vel: "
        << (node_curr->odometry.pose.block<3, 3>(0, 0).transpose() *
            node_curr->odometry.vel)
               .transpose();
  AINFO << "v_b: " << v_b.transpose();

  Eigen::Vector3d v_bb_obs = node_curr->odometry.vel -
                             node_curr->odometry.pose.block<3, 3>(0, 0) * v_b;
  AINFO << "v_bb_obs: " << v_bb_obs.transpose();

  VectorYVel YVel;
  YVel = v_bb_obs;
  Y = YVel;

  // set Kalman gain:
  K = node_curr->state.P * G.transpose() *
      (G * node_curr->state.P * G.transpose() + CVel * RVel_ * CVel.transpose())
          .inverse();
  return true;
}

bool ErrorStateKalmanFilter::ApplyMeasurementENUVel(
    StateNodeBuffer::iterator& node_curr, const Eigen::Vector3d& v_enu,
    Eigen::VectorXd& Y, Eigen::MatrixXd& G, Eigen::MatrixXd& K) {
  // set measurement:
  MatrixGVel GVel;
  MatrixCVel CVel;
  GVel.setZero();
  GVel.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  G = GVel;
  CVel.setZero();
  CVel.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  // set measurement equation:

  // AINFO << "vel: "
  //       << (node_curr->odometry.pose.block<3, 3>(0, 0).transpose() *
  //           node_curr->odometry.vel)
  //              .transpose();
  // AINFO << "v_b: " << v_b.transpose();

  Eigen::Vector3d v_bb_obs = node_curr->odometry.vel - v_enu;
  AINFO << "v_bb_obs: " << v_bb_obs.transpose() << "-->" << v_enu.transpose();

  VectorYVel YVel;
  YVel = v_bb_obs;
  Y = YVel;

  // set Kalman gain:
  K = node_curr->state.P * G.transpose() *
      (G * node_curr->state.P * G.transpose() + CVel * RVel_ * CVel.transpose())
          .inverse();
  return true;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(Eigen::Matrix4d& pose,
                                         Eigen::Vector3d& linear_vel,
                                         Eigen::Vector3d& linear_acc,
                                         Eigen::Vector3d& ang_vel) {
  std::lock_guard<std::mutex> lock(state_node_mutex_);
  StateNodeBuffer::iterator node_begin = std::next(state_node_buffer_.begin());
  Eigen::Matrix4d pose_updated = node_begin->odometry.pose;
  Eigen::Vector3d vel_updated = node_begin->odometry.vel;
  Eigen::VectorXd X = node_begin->state.X;
  linear_acc = node_begin->input.linear_acc;
  ang_vel = node_begin->input.ang_vel;

  // eliminate error:
  // a. position:
  pose_updated.block<3, 1>(0, 3) =
      pose_updated.block<3, 1>(0, 3) - X.block<3, 1>(K_INDEX_ERROR_POS, 0);
  // b. velocity:
  vel_updated = vel_updated - X.block<3, 1>(K_INDEX_ERROR_VEL, 0);
  // c. orientation:
  Eigen::Matrix3d C_nn =
      mmath::SO3::exp(X.block<3, 1>(K_INDEX_ERROR_ORI, 0)).getRotationMatrix();
  pose_updated.block<3, 3>(0, 0) = C_nn * pose_updated.block<3, 3>(0, 0);

  // finally:
  pose = pose_updated;
  linear_vel = vel_updated;
}

double ErrorStateKalmanFilter::GetLatestTimeStamp() {
  std::lock_guard<std::mutex> lock(state_node_mutex_);
  auto node_begin_next = std::next(state_node_buffer_.begin());
  return node_begin_next->input.timestamp;
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov& cov) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // a. delta position:
  cov.pos.x = P_(K_INDEX_ERROR_POS + OFFSET_X, K_INDEX_ERROR_POS + OFFSET_X);
  cov.pos.y = P_(K_INDEX_ERROR_POS + OFFSET_Y, K_INDEX_ERROR_POS + OFFSET_Y);
  cov.pos.z = P_(K_INDEX_ERROR_POS + OFFSET_Z, K_INDEX_ERROR_POS + OFFSET_Z);

  // b. delta velocity:
  cov.vel.x = P_(K_INDEX_ERROR_VEL + OFFSET_X, K_INDEX_ERROR_VEL + OFFSET_X);
  cov.vel.y = P_(K_INDEX_ERROR_VEL + OFFSET_Y, K_INDEX_ERROR_VEL + OFFSET_Y);
  cov.vel.z = P_(K_INDEX_ERROR_VEL + OFFSET_Z, K_INDEX_ERROR_VEL + OFFSET_Z);

  // c. delta orientation:
  cov.ori.x = P_(K_INDEX_ERROR_ORI + OFFSET_X, K_INDEX_ERROR_ORI + OFFSET_X);
  cov.ori.y = P_(K_INDEX_ERROR_ORI + OFFSET_Y, K_INDEX_ERROR_ORI + OFFSET_Y);
  cov.ori.z = P_(K_INDEX_ERROR_ORI + OFFSET_Z, K_INDEX_ERROR_ORI + OFFSET_Z);

  // d. gyro. bias:
  cov.gyro_bias.x =
      P_(K_INDEX_ERROR_GYRO + OFFSET_X, K_INDEX_ERROR_GYRO + OFFSET_X);
  cov.gyro_bias.y =
      P_(K_INDEX_ERROR_GYRO + OFFSET_Y, K_INDEX_ERROR_GYRO + OFFSET_Y);
  cov.gyro_bias.z =
      P_(K_INDEX_ERROR_GYRO + OFFSET_Z, K_INDEX_ERROR_GYRO + OFFSET_Z);

  // e. accel bias:
  cov.accel_bias.x =
      P_(K_INDEX_ERROR_ACCEL + OFFSET_X, K_INDEX_ERROR_ACCEL + OFFSET_X);
  cov.accel_bias.y =
      P_(K_INDEX_ERROR_ACCEL + OFFSET_Y, K_INDEX_ERROR_ACCEL + OFFSET_Y);
  cov.accel_bias.z =
      P_(K_INDEX_ERROR_ACCEL + OFFSET_Z, K_INDEX_ERROR_ACCEL + OFFSET_Z);
}

/**
 * @brief get the deviation of the pose
 * @param Deviation, output deviation
 * @return void
 */
void ErrorStateKalmanFilter::GetDeviation(Deviation* devi) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // position deviation
  devi->pos.x = X_store_(K_INDEX_ERROR_POS + OFFSET_X, 0);
  devi->pos.y = X_store_(K_INDEX_ERROR_POS + OFFSET_Y, 0);
  devi->pos.z = X_store_(K_INDEX_ERROR_POS + OFFSET_Z, 0);

  // orientaion deviation
  devi->ori.x = X_store_(K_INDEX_ERROR_ORI + OFFSET_X, 0);
  devi->ori.y = X_store_(K_INDEX_ERROR_ORI + OFFSET_Y, 0);
  devi->ori.z = X_store_(K_INDEX_ERROR_ORI + OFFSET_Z, 0);
}

Eigen::Vector3d ErrorStateKalmanFilter::GetLinearAcceleration() {
  return state_node_buffer_.front().input.linear_acc;
}

Eigen::Vector3d ErrorStateKalmanFilter::GetAngularVelocity() {
  return state_node_buffer_.front().input.ang_vel;
}

Eigen::Vector3d ErrorStateKalmanFilter::GetLinearVelocity() {
  return state_node_buffer_.front().odometry.vel;
}
/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d& angular_vel, const Eigen::Matrix3d& R) {
  return angular_vel - state_node_buffer_.front().odometry.gyro_bias;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedLinearAcc(
    const Eigen::Vector3d& linear_acc, const Eigen::Matrix3d& R) {
  return R * (linear_acc - state_node_buffer_.front().odometry.accl_bias) - g_;
}

/**
 * @brief  eliminate error
 * @param  void
 * @return void
 */

void ErrorStateKalmanFilter::EliminateError(
    StateNodeBuffer::iterator& node_curr) {
  // correct state estimation using the state of ESKF
  // a. position:  b. velocity: c. orientation:
  node_curr->odometry.pose.block<3, 1>(0, 3) -=
      node_curr->state.X.block<3, 1>(K_INDEX_ERROR_POS, 0);
  node_curr->odometry.vel -=
      node_curr->state.X.block<3, 1>(K_INDEX_ERROR_VEL, 0);
  Eigen::Matrix3d dtheta_cross = mmath::crossMatrix<double>(
      node_curr->state.X.block<3, 1>(K_INDEX_ERROR_ORI, 0));
  node_curr->odometry.pose.block<3, 3>(0, 0) =
      node_curr->odometry.pose.block<3, 3>(0, 0) *
      (Eigen::Matrix3d::Identity() - dtheta_cross);
  Eigen::Quaterniond q_tmp(node_curr->odometry.pose.block<3, 3>(0, 0));
  q_tmp.normalize();
  node_curr->odometry.pose.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();
  node_curr->odometry.gyro_bias -=
      node_curr->state.X.block<3, 1>(K_INDEX_ERROR_GYRO, 0);
  node_curr->odometry.accl_bias -=
      node_curr->state.X.block<3, 1>(K_INDEX_ERROR_ACCEL, 0);
  // d. gyro bias:
  if (IsCovStable(K_INDEX_ERROR_GYRO)) {
    node_curr->odometry.gyro_bias +=
        node_curr->state.X.block<3, 1>(K_INDEX_ERROR_GYRO, 0);
  }
  // e. accel bias:
  if (IsCovStable(K_INDEX_ERROR_ACCEL)) {
    node_curr->odometry.accl_bias +=
        node_curr->state.X.block<3, 1>(K_INDEX_ERROR_ACCEL, 0);
  }
}

void ErrorStateKalmanFilter::DropHistory() {
  while (state_node_buffer_.front().input.timestamp -
             state_node_buffer_.back().input.timestamp >
         1.500) {
    state_node_buffer_.pop_back();
  }
}

bool ErrorStateKalmanFilter::FindClosestStateNode(
    const double timestamp, StateNodeBuffer::iterator& node_iter) {
  StateNodeBuffer::iterator state_node_iter =
      std::find_if(state_node_buffer_.begin(), state_node_buffer_.end(),
                   [timestamp](const StateNode& state_node) {
                     return timestamp > state_node.input.timestamp;
                   });
  if (state_node_iter == state_node_buffer_.end()) {
    AWARN << "No state node found!";
    return false;
  }

  if (state_node_iter == state_node_buffer_.begin()) {
    AWARN << "The measurement timestamp is exceed!";
    return false;
  }

  auto state_node_iter_prev = std::prev(state_node_iter);
  node_iter = state_node_iter_prev->input.timestamp - timestamp <
                      timestamp - state_node_iter->input.timestamp
                  ? state_node_iter_prev
                  : state_node_iter;
  return true;
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < K_SIZE_POSITION; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}

void ErrorStateKalmanFilter::ResetState(StateNodeBuffer::iterator& node_curr) {
  node_curr->state.X_store = node_curr->state.X;
  node_curr->state.X = VectorX::Zero();
}
void ErrorStateKalmanFilter::ResetStoreState(
    StateNodeBuffer::iterator& node_curr) {
  node_curr->state.X_store = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetCovariance(void) {
  P_ = MatrixP::Zero();

  P_.block<3, 3>(K_INDEX_ERROR_POS, K_INDEX_ERROR_POS) =
      cov_.prior.posi * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(K_INDEX_ERROR_VEL, K_INDEX_ERROR_VEL) =
      cov_.prior.vel * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(K_INDEX_ERROR_ORI, K_INDEX_ERROR_ORI) =
      cov_.prior.ori * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(K_INDEX_ERROR_GYRO, K_INDEX_ERROR_GYRO) =
      cov_.prior.epslion * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(K_INDEX_ERROR_ACCEL, K_INDEX_ERROR_ACCEL) =
      cov_.prior.delta * Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::GetQPose(Eigen::MatrixXd& Q, Eigen::VectorXd& Y) {
  // build observability matrix for position measurement:
  Y = Eigen::VectorXd::Zero(K_DIM_STATE * K_DIM_MEASUREMENT_POSE);
  Y.block<K_DIM_MEASUREMENT_POSE, 1>(0, 0) = YPose_;
  for (int i = 1; i < K_DIM_STATE; ++i) {
    QPose_.block<K_DIM_MEASUREMENT_POSE, K_DIM_STATE>(
        i * K_DIM_MEASUREMENT_POSE, 0) =
        (QPose_.block<K_DIM_MEASUREMENT_POSE, K_DIM_STATE>(
             (i - 1) * K_DIM_MEASUREMENT_POSE, 0) *
         F_);

    Y.block<K_DIM_MEASUREMENT_POSE, 1>(i * K_DIM_MEASUREMENT_POSE, 0) = YPose_;
  }

  Q = QPose_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double& time, const MeasurementType& measurement_type) {
  // get Q:
  Eigen::MatrixXd Q;
  Eigen::VectorXd Y;
  switch (measurement_type) {
    case MeasurementType::POSE:
      GetQPose(Q, Y);
      break;
    default:
      break;
  }

  observability_.time.emplace_back(time);
  observability_.Q.emplace_back(Q);
  observability_.Y.emplace_back(Y);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType& measurement_type) {
  // get fusion strategy:
  std::string type;
  switch (measurement_type) {
    case MeasurementType::POSE:
      type = std::string("pose");
      break;
    case MeasurementType::POSE_VEL:
      type = std::string("pose_velocity");
      break;
    case MeasurementType::POSI:
      type = std::string("position");
      break;
    case MeasurementType::POSI_VEL:
      type = std::string("position_velocity");
      break;
    default:
      return false;
      break;
  }

  // build Q_so:
  const int N = observability_.Q.at(0).rows();

  std::vector<std::vector<double>> q_data, q_so_data;

  Eigen::MatrixXd Qso(observability_.Q.size() * N, K_DIM_STATE);
  Eigen::VectorXd Yso(observability_.Y.size() * N);

  for (size_t i = 0; i < observability_.Q.size(); ++i) {
    const double& time = observability_.time.at(i);

    const Eigen::MatrixXd& Q = observability_.Q.at(i);
    const Eigen::VectorXd& Y = observability_.Y.at(i);

    Qso.block(i * N, 0, N, K_DIM_STATE) = Q;
    Yso.block(i * N, 0, N, 1) = Y;

    KalmanFilter::AnalyzeQ(K_DIM_STATE, time, Q, Y, q_data);

    if (0 < i && (0 == i % 10)) {
      KalmanFilter::AnalyzeQ(K_DIM_STATE, observability_.time.at(i - 5),
                             Qso.block((i - 10), 0, 10 * N, K_DIM_STATE),
                             Yso.block((i - 10), 0, 10 * N, 1), q_so_data);
    }
  }

  std::string q_data_csv = "data/observability/" + type + ".csv";
  std::string q_so_data_csv = "data/observability/" + type + "_som.csv";

  KalmanFilter::WriteAsCSV(K_DIM_STATE, q_data, q_data_csv);
  KalmanFilter::WriteAsCSV(K_DIM_STATE, q_so_data, q_so_data_csv);

  return true;
}
}  // namespace loc
}  // namespace century
