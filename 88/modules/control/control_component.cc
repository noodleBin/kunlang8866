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
#include "modules/control/control_component.h"

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"
#define DECEL_SOFT 0.5
#define SYS_TIME 0.02
namespace century {
namespace control {

using century::canbus::Chassis;
using century::common::ErrorCode;
using century::common::Status;
using century::common::VehicleSignal;
using century::common::VehicleStateProvider;
using century::cyber::Clock;
using century::fas_aeb_backend::FasAebInfo;
using century::localization::LocalizationEstimate;
using century::monitor::MonitoredData;
using century::planning::ADCTrajectory;
using century::planning::DisplayType;
using century::planning::HonkingScenario;

namespace {
constexpr double kWheelAngleErrorThreshold = 5.0;
constexpr double kVelocityAlmostZero = 0.12;
constexpr double kTurnSignalExitThreshold = 1.5;
constexpr double kTurnSignalEnterThreshold = 3.0;
constexpr uint32_t kLightValidFlag = 1;
constexpr int kEarliestLightOffTime = 6;
constexpr int kLatestLightOnTime = 18;
constexpr int kMaxReachedDestinationNum = 50;
constexpr double kHardbrake = 250;
constexpr uint32_t kManualInterventionBrake = 250;
const std::map<HonkingScenario, std::tuple<uint8_t, double, double>>
    kHonkingMap = {{HonkingScenario::NO_HONKING, {0, 0, 0}},
                   {HonkingScenario::JUNCTION, {2, 0.4, 0.4}},
                   {HonkingScenario::DRIVEOFF, {1, 0.4, 0.4}},
                   {HonkingScenario::REACHE_DESTINATION, {2, 0.4, 0.4}},
                   {HonkingScenario::TUNING, {2, 0.4, 0.4}},
                   {HonkingScenario::PEDESTRAN, {2, 0.4, 0.4}}};
}  // namespace
ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}
/// @brief 1.construct listeners of chassis, planning, location and pad_msg
/// ///////////////////////////////
///////////2.print log ///////////////////////////////
/// @return
bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  AINFO << "Conf file: " << ConfigFilePath() << " is loaded.";

  // initial controller agent when not using control submodules
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  if (!FLAGS_use_control_submodules &&
      !controller_agent_.Init(injector_, &control_conf_).ok()) {
    // set controller
    ADEBUG << "original control";
    monitor_logger_buffer_.ERROR("Control init controller failed! Stopping...");
    return false;
  }

  cyber::ReaderConfig system_monitor_aarch_reader_config;

#if defined __x86_64__
  system_monitor_aarch_reader_config.channel_name =
      "/century/monitor/monitor_data_x86";
#else
  system_monitor_aarch_reader_config.channel_name =
      "/century/monitor/monitor_data_aarch";
#endif

  // system_monitor_aarch_reader_config.channel_name =
  //     FLAGS_system_monitor_aarch_topic;
  system_monitor_aarch_reader_config.pending_queue_size =
      FLAGS_monitor_pending_queue_size;

  monitor_aarch_reader_ = node_->CreateReader<MonitoredData>(
      system_monitor_aarch_reader_config,
      [this](const std::shared_ptr<MonitoredData> &msg) {
        ADEBUG << "Received aarch monitor data: run control callback.";
        OnCenturyMonitorAarch(msg);
      });

  cyber::ReaderConfig AEB_Enable_reader_config;
  AEB_Enable_reader_config.channel_name = FLAGS_fas_aeb_info_topic;
  AEB_Enable_reader_config.pending_queue_size = FLAGS_aeb_enable_size;

  AEB_Enable_reader =
      node_->CreateReader<FasAebInfo>(AEB_Enable_reader_config, nullptr);
  ACHECK(AEB_Enable_reader != nullptr);

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  cyber::ReaderConfig AEB_reader_config;
  AEB_reader_config.channel_name = FLAGS_planning_aeb_topic;
  AEB_reader_config.pending_queue_size = FLAGS_planning_aeb_pending_queue_size;
  AEB_reader_ =
      node_->CreateReader<planning::AebResult>(AEB_reader_config, nullptr);
  // ACHECK(AEB_reader_ != nullptr);

  cyber::ReaderConfig emergency_stop_config;
  emergency_stop_config.channel_name = FLAGS_emergency_stop_topic;
  emergency_stop_config.pending_queue_size =
      FLAGS_emergency_stop_pending_queue_size;
  emergency_stop_reader_ = node_->CreateReader<century::mcloud::EmergencyStop>(
      emergency_stop_config, nullptr);
  ACHECK(emergency_stop_reader_ != nullptr);

  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    ACHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    ACHECK(local_view_writer_ != nullptr);
  }

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  return true;
}

void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

void ControlComponent::OnAEBEnable(
    const std::shared_ptr<FasAebInfo> &AEBEnable) {
  ADEBUG << "Received AEBEnable data: run AEBEnable callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_aeb_enable_.CopyFrom(*AEBEnable);
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  ADEBUG << "trajectory->trajectory_point().empty(): "
         << trajectory->trajectory_point().empty();
  ADEBUG << "trajectory->trajectory_point().size(): "
         << trajectory->trajectory_point().size();
  ADEBUG << "gear: " << trajectory->gear();

  latest_trajectory_.CopyFrom(*trajectory);

  // AINFO << "latest_trajectory_:  " << latest_trajectory_;
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

void ControlComponent::OnAEB(const std::shared_ptr<planning::AebResult> &aeb) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_aeb_.CopyFrom(*aeb);
}

void ControlComponent::OnEmergencyStop(
    const std::shared_ptr<century::mcloud::EmergencyStop>& emergency_stop) {
  // check is new emergency stop
  // bool is_new_emergency_stop = false;
  // if (latest_emergency_stop_.has_header()) {
  //   if (emergency_stop->has_header() &&
  //       latest_emergency_stop_.header().timestamp_sec() !=
  //           emergency_stop->header().timestamp_sec() &&
  //       latest_emergency_stop_.header().sequence_num() !=
  //           emergency_stop->header().sequence_num() &&
  //       emergency_stop->emergency_stop()) {
  //     is_new_emergency_stop = true;
  //   }
  // } else {
  //   if (emergency_stop->emergency_stop()) {
  //     is_new_emergency_stop = true;
  //   }
  // }

  // if (is_new_emergency_stop) {
  //   m_AEB_enable_b = true;
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   latest_emergency_stop_.CopyFrom(*emergency_stop);
  // }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_emergency_stop_.CopyFrom(*emergency_stop);
}

void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

void ControlComponent::OnCenturyMonitorAarch(
    const std::shared_ptr<century::monitor::MonitoredData> &msg) {
  int state = 0;
  if (FLAGS_enable_check_monitor_input) {
    for (const auto &data : msg->fault_data()) {
      state = std::min(std::max(state, data.level()),
                       static_cast<int>(MonitorState::EMERGENCY_LEVEL_4));
    }

    if (state >= MonitorState::EMERGENCY_LEVEL_3) {
      AINFO << "state >= MonitorState::EMERGENCY_LEVEL_3 , monitor_aarch has "
               "request ="
            << state;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    monitor_aarch_state_ = static_cast<MonitorState>(state);
    monitor_aarch_receive_time_ = Clock::NowInSeconds();
  }
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);
  // check data
  AINFO << "input status =: " << status;
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        century::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    control_command->set_control_state(ControlState::CHASSIS_ERROR);
    estop_reason_ = status.error_message();
  } else {
    Status status_ts = CheckTimestamp(local_view_);
    AINFO << "time stamp status =: " << status_ts;
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      status = status_ts;
      control_command->set_control_state(ControlState::GUARD);
      if (local_view_.chassis().driving_mode() !=
          century::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            century::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
        control_command->set_control_state(ControlState::CHASSIS_ERROR);
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          century::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  // check estop
  // estop_ = control_conf_.enable_persistent_estop()
  //              ? estop_ || local_view_.trajectory().estop().is_estop()
  //              : local_view_.trajectory().estop().is_estop();

  estop_ = estop_ || local_view_.trajectory().estop().is_estop();

  ADEBUG << "estop = : " << estop_;
  if (local_view_.trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.trajectory().estop().reason();
  }

  if (local_view_.trajectory().trajectory_point().empty()) {
    control_command->set_control_state(ControlState::TRAJECTORY_LOSSED);
  }

  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
    if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      control_command->set_control_state(ControlState::TRAJECTORY_ERROR);
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }
  AINFO << "estop_reason_: " << estop_reason_;
  if (!estop_) {
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();

    debug->mutable_localization_header()->CopyFrom(
        local_view_.localization().header());

    debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());

    debug->mutable_trajectory_header()->CopyFrom(
        local_view_.trajectory().header());

    debug->mutable_aeb_header()->CopyFrom(local_view_.aeb_msg().header());

    if (local_view_.trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_.trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_sequence_num()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
    // controller agent
    Status status_compute = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), &local_view_.aeb_msg(), control_command);

    AINFO << "status_compute" << status_compute;

    if (!status_compute.ok()) {
      AERROR << "Control main function failed";
      control_command->set_control_state(ControlState::FEEDBACK_CONTROL);
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }
  // if planning set estop, then no control process triggered
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->CopyFrom(last_control_command_);
    control_command->set_target_torque_2axis(0);
    control_command->set_target_torque_3axis(0);
    control_command->set_guide1_brake(control_conf_.soft_estop_brake());
    if (estop_reason_.find("estop from planning") != std::string::npos) {
      m_AEB_enable_b = true;
    }
    // control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
  // check signal
  if (local_view_.trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_.trajectory().decision().vehicle_signal());
  }
  if (VehicleSignal::TURN_LEFT == control_command->signal().turn_signal()) {
    control_command->set_guide1_left_light(kLightValidFlag);
  } else if (VehicleSignal::TURN_RIGHT ==
             control_command->signal().turn_signal()) {
    control_command->set_guide1_right_light(kLightValidFlag);
  }
  // control_command->set_guide1_estop(estop_);
  return status;
}
bool ControlComponent::Proc() {
  AERROR << "Proc--------";
  const auto start_time = Clock::Now();
  estop_ = false;
  estop_reason_.clear();

  ControlCommand control_command;
  control_command.set_aeb_enable(false);
  AEB_Enable_reader->Observe();
  const auto &AEB_enable_msg = AEB_Enable_reader->GetLatestObserved();
  if (AEB_enable_msg == nullptr) {
    AERROR << "AEB_enable msg is not ready!";
    control_command.set_aeb_enable(true);
    control_cmd_writer_->Write(control_command);
    return false;
  }
  OnAEBEnable(AEB_enable_msg);

  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    control_cmd_writer_->Write(control_command);
    return false;
  }
  OnChassis(chassis_msg);

  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  ADEBUG << "trajectory_msg_FLAG: " << (trajectory_msg == nullptr);
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
    control_cmd_writer_->Write(control_command);
    return false;
  }
  OnPlanning(trajectory_msg);

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    control_cmd_writer_->Write(control_command);
    return false;
  }
  OnLocalization(localization_msg);

  AINFO << "before read AEB";
  AEB_reader_->Observe();
  const auto &aeb_msg = AEB_reader_->GetLatestObserved();
  if (aeb_msg == nullptr) {
    AERROR << "AEB msg is not ready!";
    control_cmd_writer_->Write(control_command);
    return false;
  }
  OnAEB(aeb_msg);

  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  emergency_stop_reader_->Observe();
  const auto& emergency_stop_msg = emergency_stop_reader_->GetLatestObserved();
  if (emergency_stop_msg != nullptr) {
    OnEmergencyStop(emergency_stop_msg);
  }

  {
    // TODO(SHU): to avoid redundent copy
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    local_view_.mutable_aeb_msg()->CopyFrom(latest_aeb_);
    local_view_.mutable_aeb_enable()->CopyFrom(latest_aeb_enable_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }
  if (century::planning::AebWarningLevel::WARNING_LEVEL_HIGH ==
      local_view_.aeb_msg().warning_level()) {
    m_AEB_enable_b = true;
  }
  // use control submodules
  if (FLAGS_use_control_submodules) {
    local_view_.mutable_header()->set_lidar_timestamp(
        local_view_.trajectory().header().lidar_timestamp());
    local_view_.mutable_header()->set_camera_timestamp(
        local_view_.trajectory().header().camera_timestamp());
    local_view_.mutable_header()->set_radar_timestamp(
        local_view_.trajectory().header().radar_timestamp());
    common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

    const auto end_time = Clock::Now();

    // measure latency
    static century::common::LatencyRecorder latency_recorder(
        FLAGS_control_local_view_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);

    local_view_writer_->Write(local_view_);
    return true;
  }

  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
  }

  if (control_conf_.is_control_test_mode() &&
      control_conf_.control_test_duration() > 0 &&
      (start_time - init_time_).ToSecond() >
          control_conf_.control_test_duration()) {
    AERROR << "Control finished testing. exit";
    return false;
  }

  control_command.set_monitor_aarch_state(monitor_aarch_state_);
  Status status = ProduceControlCommand(&control_command);
  AINFO << "ProduceControlCommand status: " << status;
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  // static double t_speedcmd = local_view_.chassis().speed_mps();
  // if (!status.ok()) {
  //   // t_speedcmd-= DECEL_SOFT * SYS_TIME;
  //   // control_command.set_guide1_brake(control_conf_.soft_estop_brake());
  //   // control_command.set_target_torque_2axis(t_speedcmd);
  //   // control_command.set_target_torque_3axis(t_speedcmd);
  //   // control_command.set_target_torque_2axis(0.0);
  //   // control_command.set_target_torque_3axis(0.0);
  // } else {
  //   t_speedcmd = local_view_.chassis().speed_mps();
  // }
  if (!local_view_.aeb_enable().fas_aeb_switch()) {
    m_AEB_enable_b = false;
  }
  control_command.set_aeb_enable(m_AEB_enable_b);
  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  // forward estop reason among following control frames.
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }
  ADEBUG << "set control command";
  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  common::util::FillHeader(node_->Name(), &control_command);

  ADEBUG << control_command.ShortDebugString();
  if (control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";

  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > control_conf_.control_period() * 1e3);
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  // measure latency
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static century::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  double desired_wheel_angle = control_command.target_steering_angle_1axis();

  SignalCommand(desired_wheel_angle, &control_command);
  HonkingCommand(&control_command);
  VoiceBroadcastCommand(&control_command);
  ManualCliclBrakeCommand(&control_command);
  VehicleFaultRecovery(&control_command);
  last_control_command_ = control_command;
  control_cmd_writer_->Write(control_command);
  return true;
}

Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }
  injector_->vehicle_state()->set_is_backward_trajectory(
      local_view->trajectory().is_backward_trajectory());
  injector_->vehicle_state()->Update(local_view->localization(),
                                     local_view->chassis());

  return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }

  if (FLAGS_enable_check_monitor_input) {
    if (current_timestamp - monitor_aarch_receive_time_ >
        control_conf_.max_planning_miss_num() *
            control_conf_.trajectory_period()) {
      AERROR << "Century monitor_aarch msg lost for " << std::setprecision(6)
             << current_timestamp - monitor_aarch_receive_time_ << "s";
      monitor_logger_buffer_.ERROR("Monitor_aarch msg lost");
      return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                    "Century monitor_aarch msg timeout");
    }
  }
  return Status::OK();
}

void ControlComponent::SignalCommand(const double &desired_wheel_angle,
                                     ControlCommand *const control_command) {
  //  it has dead_zone
  if (std::fabs(desired_wheel_angle) < kTurnSignalExitThreshold) {
    turn_signal_by_wheelangle_ = VehicleSignal::TURN_NONE;
  } else if (desired_wheel_angle > kTurnSignalEnterThreshold) {
    turn_signal_by_wheelangle_ = VehicleSignal::TURN_LEFT;
    if (local_view_.trajectory().is_backward_trajectory()) {
      turn_signal_by_wheelangle_ = VehicleSignal::TURN_RIGHT;
    }
  } else if (desired_wheel_angle < -kTurnSignalEnterThreshold) {
    turn_signal_by_wheelangle_ = VehicleSignal::TURN_RIGHT;
    if (local_view_.trajectory().is_backward_trajectory()) {
      turn_signal_by_wheelangle_ = VehicleSignal::TURN_LEFT;
    }
  }

  // astar signal should process in planning
  if (control_command->signal().turn_signal() == VehicleSignal::TURN_NONE) {
    control_command->mutable_signal()->set_turn_signal(
        turn_signal_by_wheelangle_);
  }
  if (VehicleSignal::TURN_LEFT == control_command->signal().turn_signal()) {
    control_command->set_guide1_left_light(kLightValidFlag);
  } else if (VehicleSignal::TURN_RIGHT ==
             control_command->signal().turn_signal()) {
    control_command->set_guide1_right_light(kLightValidFlag);
  }
  // Turn on running lights and profile lights according to the time

  auto ret = Clock::Now().ToVector();
  if (!ret.empty()) {
    int hour = ret[3];
    ADEBUG << "hour: " << hour;
    if (hour < kEarliestLightOffTime || hour >= kLatestLightOnTime) {
      control_command->set_guide1_driving_light(kLightValidFlag);

      control_command->set_guide1_clearance_light(kLightValidFlag);
      control_command->set_guide1_f_low_beam_light(kLightValidFlag);
      control_command->set_guide1_r_low_beam_light(kLightValidFlag);
    }
  }
}

void ControlComponent::HonkingCommand(ControlCommand *const control_command) {
  static int honking_remained_number = 0;
  static int honking_num = 0;
  static double honking_start_time = 0;
  static double single_honk_time = control_conf_.single_honk_time();
  static double single_honk_between_time =
      control_conf_.single_honk_between_time();
  double current_time = Clock::NowInSeconds();
  static bool enable_honking_count = false;
  static bool last_has_honking = false;

  if (!honking_remained_number &&
      (current_time - honking_start_time) >
          (single_honk_time + single_honk_between_time)) {
    if (local_view_.trajectory().has_honking_status() &&
        local_view_.trajectory().honking_status().need_honking() &&
        !last_has_honking) {
      enable_honking_count = true;
    }

    if (enable_honking_count && local_view_.trajectory().has_honking_status() &&
        local_view_.trajectory().honking_status().need_honking()) {
      honking_num > kMaxReachedDestinationNum ? kMaxReachedDestinationNum
                                              : honking_num++;
    } else {
      honking_num = 0;
      enable_honking_count = false;
    }
    if (kMaxReachedDestinationNum == honking_num) {
      if (kHonkingMap.find(
              local_view_.trajectory().honking_status().scenario()) !=
          kHonkingMap.end()) {
        std::tie(honking_remained_number, single_honk_time,
                 single_honk_between_time) =
            kHonkingMap.at(
                local_view_.trajectory().honking_status().scenario());
      } else {
        honking_remained_number = control_conf_.max_honking_number();
        single_honk_time = control_conf_.single_honk_time();
        single_honk_between_time = control_conf_.single_honk_between_time();
      }
    }
  } else {
    honking_num = 0;
    enable_honking_count = false;
    last_has_honking = false;
  }
  bool enable_honking = false;
  if ((current_time - honking_start_time) < single_honk_time) {
    enable_honking = true;
  } else if ((current_time - honking_start_time) <
             (single_honk_time + single_honk_between_time)) {
    enable_honking = false;
  } else if (honking_remained_number) {
    honking_remained_number--;
    honking_start_time = Clock::NowInSeconds();
    enable_honking = true;
  }
  AERROR << "enable_honking: " << enable_honking << " honking_status.scenario: "
         << local_view_.trajectory().honking_status().scenario()
         << " need_honking: "
         << local_view_.trajectory().honking_status().need_honking()
         << " single_honk_time: " << single_honk_time
         << " single_honk_between_time: " << single_honk_between_time
         << " last_has_honking" << last_has_honking
         << " honking_num: " << honking_num;

  control_command->set_guide1_horn(enable_honking);
}

void ControlComponent::VoiceBroadcastCommand(
    ControlCommand *const control_command) {
  if (!local_view_.trajectory().has_display_type()) {
    control_command->set_voice_broadcast_scenario(VOICE_BROADCAST_NOT_REQUIRED);
    return;
  }
  switch (local_view_.trajectory().display_type()) {
    case DisplayType::DEFAULT_DIS: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_NOT_REQUIRED);
      break;
    }
    case DisplayType::DRIVEOFF_DIS: {
      control_command->set_voice_broadcast_scenario(VOICE_BROADCAST_DEPART);
      break;
    }
    case DisplayType::VEHICLE_STOPED: {
      control_command->set_voice_broadcast_scenario(VOICE_BROADCAST_BRAKE);
      break;
    }
    case DisplayType::DIAGNAL_LEFT:
    case DisplayType::TURN_LEFT: {
      control_command->set_voice_broadcast_scenario(VOICE_BROADCAST_TURN_LEFT);
      break;
    }
    case DisplayType::DIAGNAL_RIGHT:
    case DisplayType::TURN_RIGHT: {
      control_command->set_voice_broadcast_scenario(VOICE_BROADCAST_TURN_RIGHT);
      break;
    }
    case DisplayType::OBSTACAL_FRONT: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_OBSTACLE_AHEAD);
      break;
    }
    case DisplayType::STOP_FOR_OBSTACLE: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_OBSTACLE_STOP);
      break;
    }
    case DisplayType::REACHED_DESTINATION: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_ARRIVE_STATION);
      break;
    }
    case DisplayType::WAITING_LOAD: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_LOADING_REQUEST);
      break;
    }
    case DisplayType::WAITING_UNLOAD: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_UNLOADING_REQUEST);
      break;
    }
    case DisplayType::REQUIER_PASS: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_PASS_REQUEST);
      break;
    }
    case DisplayType::COLLISION: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_OBSTACLE_COLLISION);
      break;
    }
    case DisplayType::LOW_SOC_WARNING: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_POWER_ALARM);
      break;
    }
    case DisplayType::LOW_TIRE_PRESSURE:
    case DisplayType::HIGH_TIRE_PRESSURE: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_PRESSURE_REQUEST);
      break;
    }
    case DisplayType::STOP_FOR_VEHICLE: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_VEHICLE_STOP);
      break;
    }
    case DisplayType::STOP_FOR_PEDESTRIAN: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_PEDESTRIAN_STOP);
      break;
    }
    case DisplayType::REVERCE_DRIVE: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_REVERSE_DRIVE);
      break;
    }
    case DisplayType::CROSS_DIAGNAL: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_CROSS_DIAGNAL);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_02: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_02);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_03: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_03);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_04: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_04);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_05: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_05);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_06: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_06);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_07: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_07);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_08: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_08);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_09: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_09);
      break;
    }
    case DisplayType::BACKGROUND_MUSIC_10: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_BACKGROUND_MUSIC_10);
      break;
    }
    default: {
      control_command->set_voice_broadcast_scenario(
          VOICE_BROADCAST_NOT_REQUIRED);
      break;
    }
  }
  // AERROR << "control_command->oice_broadcast_scenario: "
  //        << control_command->voice_broadcast_scenario();
}
void ControlComponent::ManualCliclBrakeCommand(
      ControlCommand *const control_command) {
  if (latest_emergency_stop_.emergency_stop()) {
    control_command->set_target_torque_2axis(0.0);
    control_command->set_target_torque_3axis(0.0);
    control_command->set_gear_location(control_command->gear_location());
    control_command->set_guide1_brake(kManualInterventionBrake);
    control_command->set_manual_click_brake_enable(true);
  } else {
    control_command->set_target_torque_2axis(control_command->target_torque_2axis());
    control_command->set_target_torque_3axis(control_command->target_torque_3axis());
    control_command->set_gear_location(control_command->gear_location());
    control_command->set_guide1_brake(control_command->guide1_brake());
    control_command->set_manual_click_brake_enable(false);
  }
}
void ControlComponent::VehicleFaultRecovery(
      ControlCommand *const control_command) {
  unsigned int fault_reset = 0;
  fault_reset = 
      canbus::Chassis::FAULT_MOST_SEVERE ==
      local_view_.chassis().whole_err_level() ? 1 : 0;
  control_command->set_guide1_reset(fault_reset);
}

}  // namespace control
}  // namespace century
