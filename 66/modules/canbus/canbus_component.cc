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

#include "modules/canbus/canbus_component.h"

#include "cyber/time/time.h"
// #include "cyber/timedelay/timedelay.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

using century::common::ErrorCode;
using century::control::ControlCommand;
using century::control::VoiceBroadcastScenario;
using century::cyber::Time;
using century::drivers::canbus::CanClientFactory;
using century::guardian::GuardianCommand;

namespace century {
namespace canbus {
namespace {
constexpr double kHundred = 100.0;
constexpr double kMaxWheelAngle = 33.0;
constexpr double kMaxMotorTorque = 1700;
constexpr double kMaxBrakePressure = 14.0;

constexpr double kMinBrake = 1.0;
constexpr int64_t kThousand = 1000;
constexpr int64_t kDelayThreshold = 100;
constexpr int64_t kControlDelayThreshold = 1500;  // ms
constexpr double kEpsilon = 1e-6;
constexpr double kMaxSpeedCompensationThreshold = 1.2;
constexpr double kMinSpeedCompensationThreshold = 0.8;
constexpr int8_t kDefaultVoiceNum = 15;
// The following three sets of data are audio file location, voice broadcast
// priority, and single voice broadcast time
const std::map<century::control::VoiceBroadcastScenario,
               std::tuple<int, int, double>>
    kVoiceBroadcastScenarioMap = {
        {VoiceBroadcastScenario::VOICE_BROADCAST_NOT_REQUIRED, {0, 0, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_DEPART, {5, 12, 6.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BRAKE, {9, 11, 9.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_TURN_LEFT, {13, 9, 10.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_TURN_RIGHT, {11, 10, 10.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_OBSTACLE_AHEAD, {1, 17, 7.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_ARRIVE_STATION, {3, 13, 9.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_LOADING_REQUEST, {8, 14, 9.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_UNLOADING_REQUEST,
         {7, 15, 9.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_PASS_REQUEST, {6, 16, 7.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_POWER_ALARM, {4, 3, 10.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_PRESSURE_REQUEST,
         {10, 2, 10.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_OBSTACLE_STOP, {2, 6, 6.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_OBSTACLE_COLLISION,
         {12, 1, 9.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_VEHICLE_STOP, {14, 4, 6.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_PEDESTRIAN_STOP, {15, 5, 6.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_REVERSE_DRIVE, {16, 7, 7.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_CROSS_DIAGNAL, {17, 8, 8.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC, {18, 18, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_02, {19, 19, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_03, {20, 20, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_04, {21, 21, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_05, {22, 22, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_06, {23, 23, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_07, {24, 24, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_08, {25, 25, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_09, {26, 26, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_BACKGROUND_MUSIC_10, {27, 27, 0.0}},
        {VoiceBroadcastScenario::VOICE_BROADCAST_DANGER_STAY_AWAY, {28, 1, 6.0}}
        };
}  // namespace

century::ad::msg::st_ad_ipc_msg_vehicle_can
    CanbusComponent::vehicle_can_rx_data_{};

std::string CanbusComponent::Name() const { return FLAGS_canbus_module_name; }

CanbusComponent::CanbusComponent()
    : last_timestamp_(0),
      monitor_logger_buffer_(
          century::common::monitor::MonitorMessageItem::CANBUS) {}

CanbusComponent::~CanbusComponent() {
  is_stop_ = true;
  if (FLAGS_enable_canbus_thread) {
    task_future_.get();
  }
}
bool CanbusComponent::Init() {
  if (!GetProtoConfig(&canbus_conf_)) {
    AERROR << "Unable to load canbus conf file: " << ConfigFilePath();
    return false;
  }

  last_timestamp_ = 0;

  AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic;
  control_cmd_reader_config.pending_queue_size =
      FLAGS_control_cmd_pending_queue_size;

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      control_cmd_reader_config,
      [this](const std::shared_ptr<ControlCommand> &cmd) {
        ADEBUG << "Received control data: run canbus callback.";
        OnControlCommand(*cmd);
      });

  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);

  if (CanbusConf::XAGV70E == canbus_conf_.vehicle_model()) {
    AINFO << "vehicle_model XAGV70E";
    socket_rx_handle_ = std::make_shared<century::sockcanrx::SocketCanRecv>(
        VehicleCanType::XAGV70E);
    socket_tx_handle_ = std::make_shared<century::sockcantx::SocketCanTran>(
        VehicleCanType::XAGV70E);
  } else if (CanbusConf::KL850R0 == canbus_conf_.vehicle_model()) {
    AINFO << "vehicle_model KL850R0";
    socket_rx_handle_ = std::make_shared<century::sockcanrx::SocketCanRecv>(
        VehicleCanType::KL850R0);
    socket_tx_handle_ = std::make_shared<century::sockcantx::SocketCanTran>(
        VehicleCanType::KL850R0);
  } else {
    // default use XAGV70E
    AINFO << "vehicle_model default use XAGV70E";
    socket_rx_handle_ = std::make_shared<century::sockcanrx::SocketCanRecv>(
        VehicleCanType::XAGV70E);
    socket_tx_handle_ = std::make_shared<century::sockcantx::SocketCanTran>(
        VehicleCanType::XAGV70E);
  }

  is_stop_ = false;
  if (FLAGS_enable_canbus_thread) {
    task_future_ = cyber::Async(&CanbusComponent::GenerateThread, this);
  }

  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

void CanbusComponent::PublishChassis() {
  Chassis chassis;
  // vehicle velocity, unit:m/s
  {
    std::lock_guard<std::mutex> lock(can_rx_mutex_);
    SetBaseChassisInfo(chassis);
    
    if (VCUFeedbackChargeStatus::CHARGE_NOT ==
        vehicle_can_rx_data_.b2v_st1_chrg_status) {
      chassis.set_charge_status(Chassis::CHARGE_NOT);
    } else if (VCUFeedbackChargeStatus::CHARGING_UP ==
               vehicle_can_rx_data_.b2v_st1_chrg_status) {
      chassis.set_charge_status(Chassis::CHARGING_UP);
    } else if (VCUFeedbackChargeStatus::CHARGE_FINISHED ==
               vehicle_can_rx_data_.b2v_st1_chrg_status) {
      chassis.set_charge_status(Chassis::CHARGE_FINISHED);
    } else if (VCUFeedbackChargeStatus::CHARGE_ERROR ==
               vehicle_can_rx_data_.b2v_st1_chrg_status) {
      chassis.set_charge_status(Chassis::CHARGE_ERROR);
    } else {
      chassis.set_charge_status(Chassis::CHARGE_INVALID);
    }

    if (VCUFeedbackFaultLevel::FAULT_NORMAL ==
        vehicle_can_rx_data_.b2v_fault_level) {
      chassis.set_b2v_fault_level(Chassis::FAULT_NORMAL);
    } else if (VCUFeedbackFaultLevel::FAULT_LIGHT ==
               vehicle_can_rx_data_.b2v_fault_level) {
      chassis.set_b2v_fault_level(Chassis::FAULT_LIGHT);
    } else if (VCUFeedbackFaultLevel::FAULT_MORE_SEVERE ==
               vehicle_can_rx_data_.b2v_fault_level) {
      chassis.set_b2v_fault_level(Chassis::FAULT_MORE_SEVERE);
    } else if (VCUFeedbackFaultLevel::FAULT_MOST_SEVERE ==
               vehicle_can_rx_data_.b2v_fault_level) {
      chassis.set_b2v_fault_level(Chassis::FAULT_MOST_SEVERE);
    } else {
      chassis.set_b2v_fault_level(Chassis::FAULT_INVALID);
    }

    if (VCUFeedbackFaultLevel::FAULT_NORMAL ==
        vehicle_can_rx_data_.whole_err_level) {
      chassis.set_whole_err_level(Chassis::FAULT_NORMAL);
    } else if (VCUFeedbackFaultLevel::FAULT_LIGHT ==
               vehicle_can_rx_data_.whole_err_level) {
      chassis.set_whole_err_level(Chassis::FAULT_LIGHT);
    } else if (VCUFeedbackFaultLevel::FAULT_MORE_SEVERE ==
               vehicle_can_rx_data_.whole_err_level) {
      chassis.set_whole_err_level(Chassis::FAULT_MORE_SEVERE);
    } else if (VCUFeedbackFaultLevel::FAULT_MOST_SEVERE ==
               vehicle_can_rx_data_.whole_err_level) {
      chassis.set_whole_err_level(Chassis::FAULT_MOST_SEVERE);
    } else {
      chassis.set_whole_err_level(Chassis::FAULT_INVALID);
    }

    if (VCUFeedbackGearPosition::GEAR_PARKING ==
        vehicle_can_rx_data_.vcu1_gear) {
      chassis.set_gear_location(Chassis::GEAR_PARKING);
    } else if (VCUFeedbackGearPosition::GEAR_REVERSE ==
               vehicle_can_rx_data_.vcu1_gear) {
      chassis.set_gear_location(Chassis::GEAR_REVERSE);
    } else if (VCUFeedbackGearPosition::GEAR_NEUTRAL ==
               vehicle_can_rx_data_.vcu1_gear) {
      chassis.set_gear_location(Chassis::GEAR_NEUTRAL);
    } else if (VCUFeedbackGearPosition::GEAR_DRIVE ==
               vehicle_can_rx_data_.vcu1_gear) {
      chassis.set_gear_location(Chassis::GEAR_DRIVE);
    } else {
      chassis.set_gear_location(Chassis::GEAR_INVALID);
    }

    if (CanbusConf::XAGV70E == canbus_conf_.vehicle_model()) {
      // VCU feedback steering mode
      if (VCUFeedbackSteeringMode::PARKING_PARALLEL ==
          vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_PARALLEL);
      } else if (VCUFeedbackSteeringMode::PARKING_FOREARD_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_FOREARD_DIAGONAL);
      } else if (VCUFeedbackSteeringMode::PARKING_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_DIAGONAL);
      } else if (VCUFeedbackSteeringMode::PARKING_REVERSE_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_REVERSE_DIAGONAL);
      } else {
        chassis.set_current_steering_mode(Chassis::PARKING_INVALID);
      }

      // VCU feedback working mode
      if (VCUFeedbackWorkingMode::WORKINGMODE_NAVIGATION ==
          vehicle_can_rx_data_.current_working_mode) {
        if (vehicle_can_rx_data_.tele_mode) {
          chassis.set_driving_mode(Chassis::COMPLETE_REMOTE);
        } else {
          chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
        }
      } else if (VCUFeedbackWorkingMode::WORKINGMODE_MANUAL ==
                vehicle_can_rx_data_.current_working_mode) {
        chassis.set_driving_mode(Chassis::COMPLETE_MANUAL);
      } else if (VCUFeedbackWorkingMode::WORKINGMODE_MEDIAN ==
                vehicle_can_rx_data_.current_working_mode) {
        chassis.set_driving_mode(Chassis::COMPLETE_MEDIAN);
      } else {
        chassis.set_driving_mode(Chassis::COMPLETE_MAINTENANCE);
      }
    } else if (CanbusConf::KL850R0 == canbus_conf_.vehicle_model()) {
      // VCU feedback steering mode
      if (VCUFeedbackSteeringMode_KL::KL_PARALLEL ==
          vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_PARALLEL);
      } else if (VCUFeedbackSteeringMode_KL::KL_FOREARD_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_FOREARD_DIAGONAL);
      } else if (VCUFeedbackSteeringMode_KL::KL_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_DIAGONAL);
      } else if (VCUFeedbackSteeringMode_KL::KL_REVERSE_DIAGONAL ==
                 vehicle_can_rx_data_.current_steering_mode) {
        chassis.set_current_steering_mode(Chassis::PARKING_REVERSE_DIAGONAL);
      } else {
        chassis.set_current_steering_mode(Chassis::PARKING_INVALID);
      }

      // VCU feedback working mode
      if (VCUFeedbackWorkingMode_KL::KL_NAVIGATION ==
          vehicle_can_rx_data_.current_working_mode) {
          chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
      } else if (VCUFeedbackWorkingMode_KL::KL_REMOTE ==
                vehicle_can_rx_data_.current_working_mode) {
        chassis.set_driving_mode(Chassis::COMPLETE_REMOTE);
      } else if (VCUFeedbackWorkingMode_KL::KL_MANUAL ==
                vehicle_can_rx_data_.current_working_mode) {
        chassis.set_driving_mode(Chassis::COMPLETE_MANUAL);
      } else {
        chassis.set_driving_mode(Chassis::COMPLETE_MEDIAN);
      }
    } else {
      chassis.set_current_steering_mode(Chassis::PARKING_INVALID);
      chassis.set_driving_mode(Chassis::COMPLETE_MAINTENANCE);
    }

    if (VCUFeedbackWalkingMode::WALKINGMODE_VELOCITY ==
        vehicle_can_rx_data_.current_walking_mode) {
      chassis.set_current_walking_mode(Chassis::WALKINGMODE_VELOCITY);
    } else if (VCUFeedbackWalkingMode::WALKINGMODE_TORQUE ==
               vehicle_can_rx_data_.current_walking_mode) {
      chassis.set_current_walking_mode(Chassis::WALKINGMODE_TORQUE);
    } else if (VCUFeedbackWalkingMode::WALKINGMODE_POSITION ==
               vehicle_can_rx_data_.current_walking_mode) {
      chassis.set_current_walking_mode(Chassis::WALKINGMODE_POSITION);
    } else {
      chassis.set_current_walking_mode(Chassis::WALKINGMODE_INVALID);
    }

    chassis.set_tire_pressure_11(vehicle_can_rx_data_.tire_pressure_11);
    chassis.set_tire_pressure_14(vehicle_can_rx_data_.tire_pressure_14);
    chassis.set_tire_pressure_21(vehicle_can_rx_data_.tire_pressure_21);
    chassis.set_tire_pressure_24(vehicle_can_rx_data_.tire_pressure_24);
    chassis.set_tire_pressure_31(vehicle_can_rx_data_.tire_pressure_31);
    chassis.set_tire_pressure_34(vehicle_can_rx_data_.tire_pressure_34);
    chassis.set_tire_pressure_41(vehicle_can_rx_data_.tire_pressure_41);
    chassis.set_tire_pressure_44(vehicle_can_rx_data_.tire_pressure_44);

    chassis.set_battery_soc(vehicle_can_rx_data_.b2v_soc);
    chassis.set_position_mode_feedback(
        vehicle_can_rx_data_.position_mode_feedback);
    chassis.set_total_weight(vehicle_can_rx_data_.total_weight);
  }
  chassis.set_stop_command_flag(stop_command_flag_);
  if (control_init_flags_) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    chassis.mutable_control_header()->CopyFrom(latest_control_.header());
    chassis.set_control_target_speed(latest_control_.target_torque_2axis());
    chassis.set_control_target_brake(latest_control_.guide1_brake());
    chassis.set_control_target_steering(
        latest_control_.target_steering_angle_1axis());
    chassis.set_control_target_gear_location(latest_control_.gear_location());
  }

  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(chassis);
  ADEBUG << chassis.ShortDebugString();
}

void CanbusComponent::SetBaseChassisInfo(Chassis& chassis) {
  chassis.set_bridge_1_left_wheel_angle(
      vehicle_can_rx_data_.bridge_1_left_wheel_angle);
  chassis.set_bridge_1_right_wheel_angle(
      vehicle_can_rx_data_.bridge_1_right_wheel_angle);
  chassis.set_bridge_4_left_wheel_angle(
      vehicle_can_rx_data_.bridge_4_left_wheel_angle);
  chassis.set_bridge_4_right_wheel_angle(
      vehicle_can_rx_data_.bridge_4_right_wheel_angle);
  chassis.set_steering_percentage(
      (vehicle_can_rx_data_.bridge_1_left_wheel_angle +
       vehicle_can_rx_data_.bridge_1_right_wheel_angle) *
      kHundred / (2 * kMaxWheelAngle));
  chassis.set_steering_percentage_rear(
      (vehicle_can_rx_data_.bridge_4_left_wheel_angle +
       vehicle_can_rx_data_.bridge_4_right_wheel_angle) *
      kHundred / (2 * kMaxWheelAngle));
  chassis.set_wheel_speed_0(vehicle_can_rx_data_.wheel_speed_0 /
                            (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_wheel_speed_1(vehicle_can_rx_data_.wheel_speed_1 /
                            (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_wheel_speed_2(vehicle_can_rx_data_.wheel_speed_2 /
                            (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_wheel_speed_3(vehicle_can_rx_data_.wheel_speed_3 /
                            (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_speed_mps((vehicle_can_rx_data_.front_drive_wheel_speed +
                         vehicle_can_rx_data_.back_drive_wheel_speed) /
                        (2 * (velocity_compensation_coefficient_ + kEpsilon)));
  chassis.set_speed_orign((vehicle_can_rx_data_.front_drive_wheel_speed +
                           vehicle_can_rx_data_.back_drive_wheel_speed) /
                          2);
  chassis.set_front_drive_wheel_speed(
      vehicle_can_rx_data_.front_drive_wheel_speed /
      (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_back_drive_wheel_speed(
      vehicle_can_rx_data_.back_drive_wheel_speed /
      (velocity_compensation_coefficient_ + kEpsilon));
  chassis.set_front_motor_torque(vehicle_can_rx_data_.front_motor_torque);

  chassis.set_throttle_percentage(vehicle_can_rx_data_.front_motor_torque *
                                  kHundred / kMaxMotorTorque);
  chassis.set_back_motor_torque(vehicle_can_rx_data_.back_motor_torque);
  chassis.set_throttle_percentage_rear(vehicle_can_rx_data_.back_motor_torque *
                                       kHundred / kMaxMotorTorque);
  chassis.set_y2_12_brake_proportional_electromagnet(
      vehicle_can_rx_data_.y2_12_brake_proportional_electromagnet);
  chassis.set_y3_34_brake_proportional_electromagnet(
      vehicle_can_rx_data_.y3_34_brake_proportional_electromagnet);
  chassis.set_front_drive_service_brake_pressure(
      vehicle_can_rx_data_.front_drive_service_brake_pressure);
  chassis.set_brake_percentage(
      vehicle_can_rx_data_.front_drive_service_brake_pressure * kHundred /
      kMaxBrakePressure);
  chassis.set_back_drive_service_brake_pressure(
      vehicle_can_rx_data_.back_drive_service_brake_pressure);
  chassis.set_brake_percentage_rear(
      vehicle_can_rx_data_.back_drive_service_brake_pressure * kHundred /
      kMaxBrakePressure);
}

bool CanbusComponent::Proc() {
  static int64_t last_time = Time::Now().ToMicrosecond();
  int64_t now_time = Time::Now().ToMicrosecond();
  if (((now_time - last_time) / kThousand) > 25) {
    AINFO << "The difference between two adjacent timestamps of canbus is :"
          << ((now_time - last_time) / kThousand) << " ms.";
  }
  last_time = now_time;
  stop_command_flag_ = false;
  if (FLAGS_enable_monitor_control && !FLAGS_enable_canbus_debug_flag) {
    int64_t current_timestamp = Time::Now().ToMicrosecond();
    int64_t time_diff_ms = (current_timestamp - last_timestamp_) / kThousand;
    if (time_diff_ms > kControlDelayThreshold) {
      stop_command_flag_ = true;
      AERROR << "The timestamp is too long between "
             << "two adjacent control message : " << time_diff_ms << " ms.";
    }
  }
  if (FLAGS_enable_canbus_debug_flag) {
    CanbusDebugFunction();
  } else if (stop_command_flag_) {
    StopCommand();
  }
  {
    std::lock_guard<std::mutex> lock(can_rx_mutex_);
    vehicle_can_rx_data_ = socket_rx_handle_->GetCanRxStatus();
  }
  PublishChassis();

  return true;
}

void CanbusComponent::OnControlCommand(const ControlCommand &control_command) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  latest_control_.CopyFrom(control_command);
  control_init_flags_ = true;
  int64_t current_timestamp = Time::Now().ToMicrosecond();
  int64_t time_diff_ms = (current_timestamp - last_timestamp_) / kThousand;
  last_timestamp_ = current_timestamp;

  // if command coming too soon, just ignore it.
  if (time_diff_ms < FLAGS_min_cmd_interval) {
    AINFO << "Control command comes too soon. Ignore.\n Required "
             "FLAGS_min_cmd_interval["
          << FLAGS_min_cmd_interval << "]ms, actual time interval["
          << time_diff_ms << "]ms.";
    return;
  }
  if (stop_command_flag_ && control_command.guide1_brake() < kMinBrake) {
    AINFO << "An anomaly has been detected and a stop command has been issued";
    return;
  }
  if (FLAGS_enable_canbus_debug_flag) {
    AINFO << "canbus debug ,return. ";
    return;
  }

  // When the delay exceeds 100 ms
  if (time_diff_ms > kDelayThreshold) {
    ADEBUG << "The timestamp difference between "
           << "two adjacent control message : " << time_diff_ms << " ms.";
  }

  if (control_command.has_velocity_compensation_coefficient() &&
      !control_command.velocity_compensation_coefficient()) {
    velocity_compensation_coefficient_ = common::math::Clamp(
        control_command.velocity_compensation_coefficient(),
        kMinSpeedCompensationThreshold, kMaxSpeedCompensationThreshold);
  }
  vehicle_can_command_.guide1_brake = control_command.guide1_brake();
  vehicle_can_command_.guide1_target_distance = 0;
  if (CanbusConf::XAGV70E == canbus_conf_.vehicle_model()) {
    vehicle_can_command_.guide1_xingzou_mode = 0;  // xugong speed mode is 0
    vehicle_can_command_.guide1_compny = 2;
    vehicle_can_command_.guide1_navigation_standby_mode = 0;
  } else {
    vehicle_can_command_.guide1_xingzou_mode = 2;  // xian speed mode is 2
    vehicle_can_command_.guide1_compny = 1;
    vehicle_can_command_.guide1_navigation_standby_mode = 1;
  }
  vehicle_can_command_.guide1_zhuanxiang_mode =
      static_cast<uint8_t>(control_command.guide1_zhuanxiang_mode());
  vehicle_can_command_.guide1_gear =
      static_cast<uint8_t>(control_command.gear_location());
  if (VoiceBroadcastScenario::VOICE_BROADCAST_DANGER_STAY_AWAY ==
      control_command.voice_broadcast_scenario()) {
    vehicle_can_command_.guide1_double_flash = 1;
    AINFO << "vehicle_can_command_.guide1_double_flash = 1";
  } else {
    vehicle_can_command_.guide1_double_flash =
        control_command.guide1_double_flash();
    AINFO << "vehicle_can_command_.guide1_double_flash = "
             "control_command.guide1_double_flash()";
  }
  vehicle_can_command_.guide1_left_light = control_command.guide1_left_light();
  vehicle_can_command_.guide1_right_light =
      control_command.guide1_right_light();
  vehicle_can_command_.guide1_horn = control_command.guide1_horn();
  vehicle_can_command_.guide1_estop = control_command.guide1_estop();
  vehicle_can_command_.guide1_driving_light =
      control_command.guide1_driving_light();
  vehicle_can_command_.guide1_clearance_light =
      control_command.guide1_clearance_light();
  vehicle_can_command_.guide1_HVC = 0;
  vehicle_can_command_.guide1_reset = control_command.guide1_reset();
  vehicle_can_command_.guide1_lowpower_mode = 0;
  vehicle_can_command_.guide1_target_torque_2_axis =
      control_command.target_torque_2axis() *
      velocity_compensation_coefficient_;
  vehicle_can_command_.guide1_target_steering_angle_1axis =
      control_command.target_steering_angle_1axis();
  vehicle_can_command_.guide1_target_torque_3_axis =
      control_command.target_torque_3axis() *
      velocity_compensation_coefficient_;
  vehicle_can_command_.guide1_target_steering_angle_4_axis =
      control_command.target_steering_angle_4axis();

  vehicle_can_command_.guide1_voice_num = control_command.guide1_voice_num();
  vehicle_can_command_.guide1_voice_volue =
      control_command.guide1_voice_volue();

  vehicle_can_command_.guide1_f_low_beam_light =
      control_command.guide1_f_low_beam_light();
  vehicle_can_command_.guide1_f_lhigh_beam_light =
      control_command.guide1_f_lhigh_beam_light();
  vehicle_can_command_.guide1_r_low_beam_light =
      control_command.guide1_r_low_beam_light();
  vehicle_can_command_.guide1_r_high_beam_light =
      control_command.guide1_r_high_beam_light();
  vehicle_can_command_.guide1_fog_light = control_command.guide1_fog_light();
  vehicle_can_command_.guide1_unload_lamp =
      control_command.guide1_unload_lamp();
  vehicle_can_command_.guide1_loading_lamp =
      control_command.guide1_loading_lamp();
  vehicle_can_command_.guide1_loaded_lamp =
      control_command.guide1_loaded_lamp();
  if (control_command.aeb_enable()) {
    vehicle_can_command_.AEB_Enable = (uint8_t)1;
  } else {
    vehicle_can_command_.AEB_Enable = (uint8_t)0;
  }

  static double start_time = Time::Now().ToSecond();
  static double current_voice_scenario_report_time = 0.0;
  static int current_voice_scenario_priority = 0;
  static int current_voice_scenario_num = 0;

  double current_time = Time::Now().ToSecond();

  auto ref_voice_scenario = control_command.voice_broadcast_scenario();
  int ref_voice_scenario_priority = 0;
  int ref_voice_scenario_num = 0;

  double ref_voice_scenario_report_time = 0.0;

  auto it = kVoiceBroadcastScenarioMap.find(ref_voice_scenario);
  if (it != kVoiceBroadcastScenarioMap.end()) {
    std::tie(ref_voice_scenario_num, ref_voice_scenario_priority,
             ref_voice_scenario_report_time) =
        kVoiceBroadcastScenarioMap.at(ref_voice_scenario);
  }

  if (ref_voice_scenario_num != current_voice_scenario_num &&
      ref_voice_scenario_priority) {
    if (ref_voice_scenario_priority < current_voice_scenario_priority ||
        (ref_voice_scenario_priority > current_voice_scenario_priority &&
         (current_time - start_time) >= current_voice_scenario_report_time)) {
      start_time = Time::Now().ToSecond();
      current_voice_scenario_report_time = ref_voice_scenario_report_time;
      current_voice_scenario_priority = ref_voice_scenario_priority;
      current_voice_scenario_num = ref_voice_scenario_num;
    }
  } else if (it == kVoiceBroadcastScenarioMap.end()) {
    current_voice_scenario_report_time = 0.0;
    current_voice_scenario_priority = 0;
    current_voice_scenario_num = 0;

    vehicle_can_command_.guide1_voice_num = current_voice_scenario_num;
  }

  if ((current_time - start_time) < current_voice_scenario_report_time) {
    vehicle_can_command_.guide1_voice_num = current_voice_scenario_num;
  } else {
    vehicle_can_command_.guide1_voice_num = current_voice_scenario_num;

    current_voice_scenario_report_time = 0.0;
    current_voice_scenario_priority = 0;
    current_voice_scenario_num = 0;
  }
  AINFO << " current_voice_scenario_num: " << current_voice_scenario_num
        << " ref_voice_scenario_num: " << ref_voice_scenario_num
        << " current_voice_scenario_report_time: "
        << current_voice_scenario_report_time
        << " current_voice_scenario_priority: "
        << current_voice_scenario_priority
        << " ref_voice_scenario_priority: " << ref_voice_scenario_priority;
  vehicle_can_command_.guide1_voice_volue =
      canbus_conf_.debug_guide1_voice_volue();
  // if (VCUFeedbackWorkingMode::WORKINGMODE_NAVIGATION !=
  //         vehicle_can_rx_data_.current_working_mode ||
  //     vehicle_can_rx_data_.yuankong_mode) {
  //   current_voice_scenario_report_time = 0.0;
  //   current_voice_scenario_priority = 0;
  //   current_voice_scenario_num = 0;
  //   vehicle_can_command_.guide1_voice_num = current_voice_scenario_num;
  // }
  vehicle_can_command_.ts = current_timestamp;
  
  if (CanbusConf::KL850R0 == canbus_conf_.vehicle_model()) {
    vehicle_can_command_.guide1_voice_num = kDefaultVoiceNum;
  }

  socket_tx_handle_->SetVehicleCanCmdInfo(vehicle_can_command_);
}

void CanbusComponent::StopCommand() {
  int64_t current_timestamp = Time::Now().ToMicrosecond();

  vehicle_can_command_.guide1_brake = 30;
  vehicle_can_command_.guide1_target_distance = 0;
  vehicle_can_command_.guide1_xingzou_mode = 0;
  vehicle_can_command_.guide1_zhuanxiang_mode = 0;
  vehicle_can_command_.guide1_compny = 2;
  vehicle_can_command_.guide1_gear = 0;
  vehicle_can_command_.guide1_double_flash = 1;
  vehicle_can_command_.guide1_left_light = 0;
  vehicle_can_command_.guide1_right_light = 0;
  vehicle_can_command_.guide1_horn = 0;
  vehicle_can_command_.guide1_estop = 0;
  vehicle_can_command_.guide1_driving_light = 0;
  vehicle_can_command_.guide1_clearance_light = 0;
  vehicle_can_command_.guide1_navigation_standby_mode = 0;
  vehicle_can_command_.guide1_voice_num = 0;
  vehicle_can_command_.guide1_voice_volue = 0;
  vehicle_can_command_.guide1_HVC = 0;
  vehicle_can_command_.guide1_reset = 0;
  vehicle_can_command_.guide1_lowpower_mode = 0;
  vehicle_can_command_.guide1_target_torque_2_axis = 0;
  vehicle_can_command_.guide1_target_steering_angle_1axis = 0;
  vehicle_can_command_.guide1_target_torque_3_axis = 0;
  vehicle_can_command_.guide1_target_steering_angle_4_axis = 0;

  vehicle_can_command_.guide1_f_low_beam_light = 0;
  vehicle_can_command_.guide1_f_lhigh_beam_light = 0;
  vehicle_can_command_.guide1_r_low_beam_light = 0;
  vehicle_can_command_.guide1_r_high_beam_light = 0;
  vehicle_can_command_.guide1_fog_light = 0;
  vehicle_can_command_.guide1_unload_lamp = 0;
  vehicle_can_command_.guide1_loading_lamp = 0;
  vehicle_can_command_.guide1_loaded_lamp = 0;

  vehicle_can_command_.ts = current_timestamp;

  socket_tx_handle_->SetVehicleCanCmdInfo(vehicle_can_command_);
}

void CanbusComponent::CanbusDebugFunction() {
  int64_t current_timestamp = Time::Now().ToMicrosecond();

  vehicle_can_command_.guide1_brake = canbus_conf_.debug_guide1_brake();
  vehicle_can_command_.guide1_target_distance =
      canbus_conf_.debug_guide1_target_distance();
  vehicle_can_command_.guide1_xingzou_mode = 0;
  vehicle_can_command_.guide1_zhuanxiang_mode =
      static_cast<uint8_t>(canbus_conf_.debug_guide1_zhuanxiang_mode());
  vehicle_can_command_.guide1_compny = 2;
  vehicle_can_command_.guide1_gear = canbus_conf_.debug_guide1_gear();
  vehicle_can_command_.guide1_double_flash =
      canbus_conf_.debug_guide1_double_flash();
  vehicle_can_command_.guide1_left_light =
      canbus_conf_.debug_guide1_left_light();
  vehicle_can_command_.guide1_right_light =
      canbus_conf_.debug_guide1_right_light();
  vehicle_can_command_.guide1_horn = canbus_conf_.debug_guide1_horn();
  vehicle_can_command_.guide1_estop = canbus_conf_.debug_guide1_estop();
  vehicle_can_command_.guide1_driving_light =
      canbus_conf_.debug_guide1_driving_light();
  vehicle_can_command_.guide1_clearance_light =
      canbus_conf_.debug_guide1_clearance_light();
  vehicle_can_command_.guide1_navigation_standby_mode =
      canbus_conf_.debug_guide1_navigation_standby_mode();
  vehicle_can_command_.guide1_HVC = 0;
  vehicle_can_command_.guide1_reset = canbus_conf_.debug_guide1_reset();
  vehicle_can_command_.guide1_lowpower_mode =
      canbus_conf_.debug_guide1_lowpower_mode();
  vehicle_can_command_.guide1_target_torque_2_axis =
      canbus_conf_.debug_guide1_target_torque_2_axis();
  vehicle_can_command_.guide1_target_steering_angle_1axis =
      canbus_conf_.debug_guide1_target_steering_angle_1axis();
  vehicle_can_command_.guide1_target_torque_3_axis =
      canbus_conf_.debug_guide1_target_torque_3_axis();
  vehicle_can_command_.guide1_target_steering_angle_4_axis =
      canbus_conf_.debug_guide1_target_steering_angle_4_axis();

  vehicle_can_command_.guide1_voice_num = canbus_conf_.debug_guide1_voice_num();
  vehicle_can_command_.guide1_voice_volue =
      canbus_conf_.debug_guide1_voice_volue();

  vehicle_can_command_.guide1_f_low_beam_light =
      canbus_conf_.debug_guide1_f_low_beam_light();
  vehicle_can_command_.guide1_f_lhigh_beam_light =
      canbus_conf_.debug_guide1_f_lhigh_beam_light();
  vehicle_can_command_.guide1_r_low_beam_light =
      canbus_conf_.debug_guide1_r_low_beam_light();
  vehicle_can_command_.guide1_r_high_beam_light =
      canbus_conf_.debug_guide1_r_high_beam_light();
  vehicle_can_command_.guide1_fog_light = canbus_conf_.debug_guide1_fog_light();
  vehicle_can_command_.guide1_unload_lamp =
      canbus_conf_.debug_guide1_unload_lamp();
  vehicle_can_command_.guide1_loading_lamp =
      canbus_conf_.debug_guide1_loading_lamp();
  vehicle_can_command_.guide1_loaded_lamp =
      canbus_conf_.debug_guide1_loaded_lamp();
  vehicle_can_command_.AEB_Enable = canbus_conf_.debug_aeb_enable();

  vehicle_can_command_.ts = current_timestamp;

  socket_tx_handle_->SetVehicleCanCmdInfo(vehicle_can_command_);
}

void CanbusComponent::GenerateThread() {
  while (!is_stop_) {
    static constexpr int32_t kSleepTime = 10;  // milliseconds
    if (socket_rx_handle_->VechCanRxStatusActive()) {
      socket_rx_handle_->VechCanRxCheck();
    }
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
  }
}

}  // namespace canbus
}  // namespace century
