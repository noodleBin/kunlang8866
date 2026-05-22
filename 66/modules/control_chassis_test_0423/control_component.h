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

#pragma once

#include <memory>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_aeb.pb.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/mcloud/proto/emergency_stop.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/submodules/preprocessor_submodule.h"

/**
 * @namespace century::control
 * @brief century::control
 */
namespace century {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class ControlComponent final : public century::cyber::TimerComponent {
  friend class ControlTestBase;

 public:
  ControlComponent();
  bool Init() override;

  bool Proc() override;

 private:
  // Upon receiving pad message
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  void OnChassis(const std::shared_ptr<century::canbus::Chassis> &chassis);

  void OnAEBEnable(const std::shared_ptr<century::fas_aeb_backend::FasAebInfo> &FasAebInfo);

  void OnPlanning(
      const std::shared_ptr<century::planning::ADCTrajectory> &trajectory);
  void OnAEB(const std::shared_ptr<planning::AebResult> &aeb);

  void OnEmergencyStop(
      const std::shared_ptr<century::mcloud::EmergencyStop>& emergency_stop);

  void OnLocalization(
      const std::shared_ptr<century::localization::LocalizationEstimate>
          &localization);
  void OnCenturyMonitorAarch(
      const std::shared_ptr<century::monitor::MonitoredData> &msg);
  // Upon receiving monitor message
  void OnMonitor(
      const century::common::monitor::MonitorMessage &monitor_message);
  void SignalCommand(const double &desired_wheel_angle,
                     ControlCommand *const control_command);
  void HonkingCommand(ControlCommand *const control_command);
  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput(LocalView *local_view);
  common::Status CheckTimestamp(const LocalView &local_view);
  common::Status CheckPad();
  void VoiceBroadcastCommand(ControlCommand *const control_command);
  void ManualCliclBrakeCommand(ControlCommand *const control_command);
  void VehicleFaultRecovery(ControlCommand *const control_command);
  void ChassisTestMode(ControlCommand *const control_command);
  bool PoseSave(const century::localization::LocalizationEstimate localization);

 private:
  century::cyber::Time init_time_;

  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  fas_aeb_backend::FasAebInfo latest_aeb_enable_;
  planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;
  planning::AebResult latest_aeb_;
  common::Header latest_replan_trajectory_header_;
  century::mcloud::EmergencyStop latest_emergency_stop_;

  ControllerAgent controller_agent_;

  bool estop_ = false;
  std::string estop_reason_;
  bool pad_received_ = false;
  bool m_AEB_enable_b = false;
  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;
  double monitor_aarch_receive_time_ = 0.0;
  MonitorState monitor_aarch_state_ = MonitorState::DEFAULT_MONITOR;

  ControlConf control_conf_;

  std::mutex mutex_;
  std::shared_ptr<cyber::Reader<century::monitor::MonitoredData>>
      monitor_aarch_reader_;

  std::shared_ptr<cyber::Reader<century::fas_aeb_backend::FasAebInfo>> AEB_Enable_reader;    
  std::shared_ptr<cyber::Reader<century::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<century::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<century::planning::ADCTrajectory>>
      trajectory_reader_;
  std::shared_ptr<cyber::Reader<century::planning::AebResult>> AEB_reader_;
  std::shared_ptr<cyber::Reader<century::mcloud::EmergencyStop>> emergency_stop_reader_;
      
  ControlCommand last_control_command_;
  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
  // when using control submodules
  std::shared_ptr<cyber::Writer<LocalView>> local_view_writer_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  LocalView local_view_;

  std::shared_ptr<DependencyInjector> injector_;
  common::VehicleSignal::TurnSignal turn_signal_by_wheelangle_ =
      century::common::VehicleSignal::TURN_NONE;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace century
