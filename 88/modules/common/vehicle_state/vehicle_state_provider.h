/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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
 * @file vehicle_state.h
 *
 * @brief Declaration of the class VehicleStateProvider.
 */
#pragma once

#include <memory>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/localization.pb.h"

/**
 * @namespace century::common
 * @brief century::common
 */
namespace century {
namespace common {

/**
 * @class VehicleStateProvider
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleStateProvider {
 public:
  /**
   * @brief Constructor by information of localization and chassis.
   * @param localization Localization information of the vehicle.
   * @param chassis Chassis information of the vehicle.
   */
  Status Update(const localization::LocalizationEstimate& localization,
                const canbus::Chassis& chassis);
  Status Update(const localization::LocalizationEstimate& localization,
                const canbus::Chassis& chassis,
                const bool is_reverse_drive_routing);

  /**
   * @brief Update VehicleStateProvider instance by protobuf files.
   * @param localization_file the localization protobuf file.
   * @param chassis_file The chassis protobuf file
   */
  void Update(const std::string& localization_file,
              const std::string& chassis_file);

  void UpdateAcceleration(const double acceleration) {
    vehicle_state_.set_linear_acceleration(acceleration);
  }

  double timestamp() const;

  const localization::Pose& pose() const;
  const localization::Pose& original_pose() const;

  /**
   * @brief Default destructor.
   */
  virtual ~VehicleStateProvider() = default;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  /**
   * @brief Get the kappa of vehicle position.
   *  the positive or negative sign is decided by the vehicle heading vector
   *  along the path
   * @return The kappa of vehicle position.
   */
  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  /**
   * @brief Get the vehicle's gear position.
   * @return The vehicle's gear position.
   */
  double gear() const;

  /**
   * @brief Calculate the lowpass filtered value of input.
   * @param ts The runtime.
   * @param cutoff_freq The cutoff frequncy.
   * @param ts digital_filter object.
   */
  void SetDigitalFilter(double ts, double cutoff_freq, DigitalFilter *digital_filter);

  /**
   * @brief Get the vehicle's steering angle.
   * @return double
   */
  double steering_percentage() const;

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);
  void set_is_backward_trajectory(const double is_backward_trajectory);

  void set_waiting_angle_flag(const double waiting_angle_flag);

  bool waiting_angle_flag()  const;

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  math::Vec2d EstimateFuturePosition(const double t) const;

  /**
   * @brief Compute the position of center of mass(COM) of the vehicle,
   *        given the distance from rear wheels to the center of mass.
   * @param rear_to_com_distance Distance from rear wheels to
   *        the vehicle's center of mass.
   * @return The position of the vehicle's center of mass.
   */
  math::Vec2d ComputeCOMPosition(const double rear_to_com_distance) const;

  const VehicleState& vehicle_state() const;

 private:
  bool ConstructExceptLinearVelocity(
      const localization::LocalizationEstimate& localization);
  bool ConstructExceptLinearVelocity(
      const localization::LocalizationEstimate& localization,
      const bool is_reverse_drive_routing);

  common::VehicleState vehicle_state_;
  localization::LocalizationEstimate original_localization_;
  double m_VehicleSpeedZ1_ = 0.0;
  double m_Acceleration_;
  bool m_waiting_angle_flag_b = false;
  bool is_backward_trajectory_=false;
  DigitalFilter digital_filter_acceleration;

};

}  // namespace common
}  // namespace century
