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
 * @file
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "third_party/dbc/socket_can_vehicleinforx.h"
#include "third_party/dbc/socket_can_vehicleinfotx.h"

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/guardian/proto/guardian.pb.h"

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"
#include "modules/canbus/century_ad_ipc.h"
#include "modules/canbus/century_ad_ipc_msg.h"
#include "modules/canbus/century_ad_ipc_util.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace century::canbus
 * @brief century::canbus
 */
namespace century {
namespace canbus {

enum VCUFeedbackChargeStatus {
  CHARGE_NOT = 0,
  CHARGING_UP = 1,
  CHARGE_FINISHED = 2,
  CHARGE_ERROR = 3,

};
enum VCUFeedbackFaultLevel {
  FAULT_NORMAL = 0,
  FAULT_LIGHT = 1,
  FAULT_MORE_SEVERE = 2,
  FAULT_MOST_SEVERE = 3,

};
enum VCUFeedbackGearPosition {
  GEAR_PARKING = 0,
  GEAR_REVERSE = 1,
  GEAR_NEUTRAL = 2,
  GEAR_DRIVE = 3,
};
enum VCUFeedbackSteeringMode {
  PARKING_PARALLEL = 0,
  PARKING_FOREARD_DIAGONAL = 1,
  PARKING_DIAGONAL = 2,
  PARKING_REVERSE_DIAGONAL = 4,
};

enum VCUFeedbackSteeringMode_KL {
  KL_REVERSE_DIAGONAL = 2,
  KL_FOREARD_DIAGONAL = 3,
  KL_DIAGONAL = 4,
  KL_PARALLEL = 5,
};

enum VCUFeedbackWorkingMode {
  WORKINGMODE_MANUAL = 0,
  WORKINGMODE_NAVIGATION = 1,
  WORKINGMODE_MEDIAN = 2,
  WORKINGMODE_MAINTENANCE = 3,
};

enum VCUFeedbackWorkingMode_KL {
  KL_MEDIAN = 0,
  KL_MANUAL = 1,
  KL_NAVIGATION = 2,
  KL_REMOTE = 3,
};

enum VCUFeedbackWalkingMode {
  WALKINGMODE_VELOCITY = 0,
  WALKINGMODE_TORQUE = 1,
  WALKINGMODE_POSITION = 2,
};
/**
 * @class Canbus
 *
 * @brief canbus module main class.
 * It processes the control data to send protocol messages to can card.
 */
class CanbusComponent final : public century::cyber::TimerComponent {
 public:
  CanbusComponent();
  virtual ~CanbusComponent();
  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const;

 private:
  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

  /**
   * @brief module on_time function
   */
  bool Proc() override;

  void PublishChassis();
  void OnControlCommand(
      const century::control::ControlCommand &control_command);
  void StopCommand();
  void CanbusDebugFunction();

  void GenerateThread();
  void SetBaseChassisInfo(Chassis& chassis);

  CanbusConf canbus_conf_;
  std::shared_ptr<cyber::Reader<century::control::ControlCommand>>
      control_command_reader_;
  int64_t last_timestamp_ = 0;
  bool stop_command_flag_ = false;
  bool canbus_rx_flag_ = false;
  bool control_init_flags_ = false;
  double velocity_compensation_coefficient_ = 1.0;
  std::atomic<bool> is_stop_{false};
  std::future<void> task_future_;
  std::mutex can_rx_mutex_;
  std::mutex control_mutex_;
  ::century::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  std::shared_ptr<cyber::Writer<Chassis>> chassis_writer_;
  control::ControlCommand latest_control_;

 private:
  century::ad::msg::st_ad_ipc_msg_vehicle_can_command vehicle_can_command_;

 private:
  static century::ad::msg::st_ad_ipc_msg_vehicle_can vehicle_can_rx_data_;

  std::shared_ptr<century::sockcanrx::SocketCanRecv> socket_rx_handle_;
  std::shared_ptr<century::sockcantx::SocketCanTran> socket_tx_handle_;
};

CYBER_REGISTER_COMPONENT(CanbusComponent)

}  // namespace canbus
}  // namespace century
