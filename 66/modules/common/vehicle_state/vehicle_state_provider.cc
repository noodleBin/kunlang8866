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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include <algorithm>
#include <cmath>

#include "Eigen/Core"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace common {
using century::common::VehicleConfigHelper;
namespace {
constexpr double kMaxKappa = 0.1;
constexpr double kEpsilon = 1e-6;
constexpr double kRatio = 1.0;
constexpr double kts = 0.02;
constexpr double kcutoffFreq = 10.0;
}  // namespace
Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis) {
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    std::string msg = absl::StrCat(
        "Fail to update because ConstructExceptLinearVelocity error.",
        "localization:\n", localization.DebugString());
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }
  if (localization.has_measurement_time()) {
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_timestamp_sec()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().timestamp_sec());
  }

  if (chassis.has_gear_location()) {
    vehicle_state_.set_gear(chassis.gear_location());
  } else {
    vehicle_state_.set_gear(canbus::Chassis::GEAR_NONE);
  }

  if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(chassis.speed_mps());
    if (!FLAGS_reverse_heading_vehicle_state &&
        vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
      vehicle_state_.set_linear_velocity(-chassis.speed_mps());
    }
    // double t_time_d = 0.02;
    // double t_Acceleration_d =
    //     (chassis.speed_mps() - m_VehicleSpeedZ1_) / t_time_d;

    // SetDigitalFilter(t_time_d, 10, &digital_filter_acceleration);

    // m_Acceleration_ = digital_filter_acceleration.Filter(t_Acceleration_d);

    // m_VehicleSpeedZ1_ = chassis.speed_mps();
  } else {
  }


  if(chassis.has_steering_percentage()) {
    vehicle_state_.set_steering_percentage(chassis.steering_percentage());
 }

// if (chassis.has_waiting_steering_flag()) {
//   vehicle_state_.set_waiting_steering_flag(chassis.waiting_steering_flag());
// }
// if (chassis.has_waiting_steering_flag()) {
//   vehicle_state_.set_waiting_steering_flag(chassis.waiting_steering_flag());
// }

static constexpr double kEpsilon = 1e-6;
if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
  vehicle_state_.set_kappa(0.0);
} else {
  vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                           vehicle_state_.linear_velocity());
  // the angular velocity of imu may not be accurate, which may result in a
  // larger curvature calculated during replanning.
  const auto &veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double kappa =
      std::tan(vehicle_state_.steering_percentage() * 0.01 *
               veh_param.max_steer_angle() / veh_param.steer_ratio()) /
      (veh_param.wheel_base() * 0.5);
    if (kappa < 0.0) {
      kappa = std::max(kappa, -kMaxKappa);
    } else {
      kappa = std::min(kappa, kMaxKappa);
    }
  if (FLAGS_enable_use_minimum_turning_radius_to_get_kappa) {
    double max_kappa_use_min_radius =
        1.0 / std::max(veh_param.min_turn_radius(), kEpsilon);
    double max_kappa_use_steering_percentage =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        (veh_param.wheel_base() * 0.5);
    ADEBUG << "max_kappa_use_min_radius = " << max_kappa_use_min_radius
           << "          max_kappa_use_steering_percentage="
           << max_kappa_use_steering_percentage;
    double ratio = kRatio;
    if (max_kappa_use_steering_percentage > max_kappa_use_min_radius) {
      ratio = max_kappa_use_min_radius /
              std::max(max_kappa_use_steering_percentage, kMaxKappa);
    }
    kappa = kappa * ratio;
  }

  if (FLAGS_enable_use_steer) {
    vehicle_state_.set_kappa(kappa);
  }
}

vehicle_state_.set_driving_mode(chassis.driving_mode());

return Status::OK();
}

// for planning if need to backward planner.
Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis, const bool is_reverse_drive_routing) {
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization, is_reverse_drive_routing)) {
    std::string msg = absl::StrCat(
        "Fail to update because ConstructExceptLinearVelocity error.",
        "localization:\n", localization.DebugString());
    AERROR << msg;
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }
  if (localization.has_measurement_time()) {
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_timestamp_sec()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().timestamp_sec());
  }

  if (chassis.has_gear_location()) {
    vehicle_state_.set_gear(chassis.gear_location());
  } else {
    vehicle_state_.set_gear(canbus::Chassis::GEAR_NONE);
  }

  // TODO(zongxingguo): maby openspace need  negative speed.
  if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(std::fabs(chassis.speed_mps()));
    if (vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
      if(chassis.speed_mps() > kEpsilon){
        AERROR<<" chass gear reverse ,but speed large zero.";
      }
      vehicle_state_.set_linear_velocity(std::fabs(chassis.speed_mps()));
    }
  } else {
    AERROR<<"no get speed from chassis.";
  }

  if (chassis.has_steering_percentage()) {
    vehicle_state_.set_steering_percentage(chassis.steering_percentage());
  }

  // if (chassis.has_waiting_steering_flag()) {
  //   vehicle_state_.set_waiting_steering_flag(chassis.waiting_steering_flag());
  // }
  // if (chassis.has_waiting_steering_flag()) {
  //   vehicle_state_.set_waiting_steering_flag(chassis.waiting_steering_flag());
  // }

  static constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
    // the angular velocity of imu may not be accurate, which may result in a
    // larger curvature calculated during replanning.
    const auto &veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    double kappa =
        std::tan(vehicle_state_.steering_percentage() * 0.01 *
                 veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        (veh_param.wheel_base() * 0.5);
    if (kappa < 0.0) {
      kappa = std::max(kappa, -kMaxKappa);
    } else {
      kappa = std::min(kappa, kMaxKappa);
    }
    if (FLAGS_enable_use_minimum_turning_radius_to_get_kappa) {
      double max_kappa_use_min_radius =
          1.0 / std::max(veh_param.min_turn_radius(), kEpsilon);
      double max_kappa_use_steering_percentage =
          std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
          (veh_param.wheel_base() * 0.5);
      ADEBUG << "max_kappa_use_min_radius = " << max_kappa_use_min_radius
             << "          max_kappa_use_steering_percentage="
             << max_kappa_use_steering_percentage;
      double ratio = kRatio;
      if (max_kappa_use_steering_percentage > max_kappa_use_min_radius) {
        ratio = max_kappa_use_min_radius /
                std::max(max_kappa_use_steering_percentage, kMaxKappa);
      }
      kappa = kappa * ratio;
    }

    if (FLAGS_enable_use_steer) {
      vehicle_state_.set_kappa(kappa);
    }
  }

  vehicle_state_.set_driving_mode(chassis.driving_mode());


  return Status::OK();
}

// for planning, we need to change localization heading when need use backword
// planner.
bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate &localization,
    const bool is_reverse_drive_routing) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  vehicle_state_.mutable_pose()->CopyFrom(localization.pose());
  if (localization.pose().has_position()) {
    vehicle_state_.set_x(localization.pose().position().x());
    vehicle_state_.set_y(localization.pose().position().y());
    vehicle_state_.set_z(localization.pose().position().z());
  }

  const auto &orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    AINFO << "localization.pose().heading() = "
          << localization.pose().heading();
    AINFO << "update pose heading = " << localization.pose().heading() + M_PI;
    vehicle_state_.set_heading(localization.pose().heading());
    if (is_reverse_drive_routing) {
      vehicle_state_.set_heading(century::common::math::NormalizeAngle(
          (localization.pose().heading() + M_PI)));
    }

  } else {
    vehicle_state_.set_heading(
        math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz()));
  }

  if (FLAGS_enable_map_reference_unify) {
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity_vrf().z());

    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration_vrf().x());
  } else {
    if (!localization.pose().has_angular_velocity()) {
      AERROR << "localization.pose() has no angular velocity.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity().z());

    if (!localization.pose().has_linear_acceleration()) {
      AERROR << "localization.pose() has no linear acceleration.";
      return false;
    }

    double t_pose_acceleration_d = 0.0;
    t_pose_acceleration_d =
        std::sqrt(localization.pose().linear_acceleration().x() *
                      localization.pose().linear_acceleration().x() +
                  localization.pose().linear_acceleration().y() *
                      localization.pose().linear_acceleration().y());
    SetDigitalFilter(kts, kcutoffFreq, &digital_filter_acceleration);

    t_pose_acceleration_d =
        digital_filter_acceleration.Filter(t_pose_acceleration_d);
    vehicle_state_.set_linear_acceleration(t_pose_acceleration_d);
  }

  if (localization.pose().has_euler_angles()) {
    vehicle_state_.set_roll(localization.pose().euler_angles().y());
    vehicle_state_.set_pitch(localization.pose().euler_angles().x());
    vehicle_state_.set_yaw(localization.pose().euler_angles().z());
  } else {
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  return true;
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate &localization) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  vehicle_state_.mutable_pose()->CopyFrom(localization.pose());
  if (localization.pose().has_position()) {
    vehicle_state_.set_x(localization.pose().position().x());
    vehicle_state_.set_y(localization.pose().position().y());
    vehicle_state_.set_z(localization.pose().position().z());
  }

  const auto &orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    vehicle_state_.set_heading(localization.pose().heading());
  } else {
    vehicle_state_.set_heading(
        math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz()));
  }

  if (FLAGS_enable_map_reference_unify) {
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity_vrf().z());

    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration_vrf().x());
  } else {
    if (!localization.pose().has_angular_velocity()) {
      AERROR << "localization.pose() has no angular velocity.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity().z());

    if (!localization.pose().has_linear_acceleration()) {
      AERROR << "localization.pose() has no linear acceleration.";
      return false;
    }

    double t_pose_acceleration_d = 0.0;
    t_pose_acceleration_d =
        std::sqrt(localization.pose().linear_acceleration().x() *
                      localization.pose().linear_acceleration().x() +
                  localization.pose().linear_acceleration().y() *
                      localization.pose().linear_acceleration().y());
    SetDigitalFilter(kts, kcutoffFreq, &digital_filter_acceleration);

    t_pose_acceleration_d =
        digital_filter_acceleration.Filter(t_pose_acceleration_d);
    vehicle_state_.set_linear_acceleration(t_pose_acceleration_d);
  }

  if (localization.pose().has_euler_angles()) {
    vehicle_state_.set_roll(localization.pose().euler_angles().y());
    vehicle_state_.set_pitch(localization.pose().euler_angles().x());
    vehicle_state_.set_yaw(localization.pose().euler_angles().z());
  } else {
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  if (FLAGS_reverse_heading_control &&
      canbus::Chassis::GEAR_REVERSE == vehicle_state_.gear()&&is_backward_trajectory_) {
    vehicle_state_.set_heading(
        common::math::NormalizeAngle(vehicle_state_.heading() + M_PI));
    vehicle_state_.set_angular_velocity(vehicle_state_.angular_velocity());
    vehicle_state_.set_linear_acceleration(
        -vehicle_state_.linear_acceleration());
    // vehicle_state_.set_roll(
    //     common::math::NormalizeAngle(vehicle_state_.roll() + M_PI));
    // vehicle_state_.set_pitch(
    //     common::math::NormalizeAngle(vehicle_state_.pitch() + M_PI));
    vehicle_state_.set_yaw(
        common::math::NormalizeAngle(vehicle_state_.yaw() + M_PI));
  }
  return true;
}

void VehicleStateProvider::SetDigitalFilter(double ts, double cutoff_freq,
                                            DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}

double VehicleStateProvider::x() const { return vehicle_state_.x(); }

double VehicleStateProvider::y() const { return vehicle_state_.y(); }

double VehicleStateProvider::z() const { return vehicle_state_.z(); }

double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa(); }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

double VehicleStateProvider::gear() const { return vehicle_state_.gear(); }

double VehicleStateProvider::steering_percentage() const {
  return vehicle_state_.steering_percentage();
}

bool VehicleStateProvider::waiting_angle_flag() const {
  return vehicle_state_.waiting_steering_flag();
}

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

const localization::Pose &VehicleStateProvider::pose() const {
  return vehicle_state_.pose();
}

const localization::Pose &VehicleStateProvider::original_pose() const {
  return original_localization_.pose();
}

void VehicleStateProvider::set_is_backward_trajectory(const double is_backward_trajectory) {
  is_backward_trajectory_=is_backward_trajectory;
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

void VehicleStateProvider::set_waiting_angle_flag(
    const double waiting_angle_flag) {
  vehicle_state_.set_waiting_steering_flag(waiting_angle_flag);
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /
                      vehicle_state_.angular_velocity();
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                            vehicle_state_.z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + vehicle_state_.x(),
                     vec_distance[1] + vehicle_state_.y());
}

math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  if ((FLAGS_state_transform_to_com_reverse &&
       vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) ||
      (FLAGS_state_transform_to_com_drive &&
       vehicle_state_.gear() == canbus::Chassis::GEAR_DRIVE)) {
    v << 0.0, rear_to_com_distance, 0.0;
  } else {
    v << 0.0, 0.0, 0.0;
  }
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    // Update the COM position with rotation
    com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  }
  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace common
}  // namespace century
