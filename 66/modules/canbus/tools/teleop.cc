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

#include <termios.h>

#include <cstdio>
#include <iostream>
#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
// #include "cyber/threadlib/threadlib.h"
#include <thread>

#include "cyber/time/time.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/message_util.h"

// gflags
DEFINE_double(throttle_inc_delta, 2.0,
              "throttle pedal command delta percentage.");
DEFINE_double(brake_inc_delta, 2.0, "brake pedal delta percentage");
DEFINE_double(steer_inc_delta, 2.0, "steer delta percentage");
// TODO(ALL) : switch the acceleration cmd or pedal cmd
// default : use pedal cmd
DEFINE_bool(
    use_acceleration, false,
    "switch to use acceleration instead of throttle pedal and brake pedal");

namespace {

using century::canbus::Chassis;
using century::common::VehicleSignal;
using century::control::ControlCommand;
using century::control::PadMessage;
using century::cyber::CreateNode;
using century::cyber::Reader;
using century::cyber::Time;
using century::cyber::Writer;

const uint32_t KEYCODE_O = 0x4F;  // '0'

const uint32_t KEYCODE_UP1 = 0x57;  // 'w'
const uint32_t KEYCODE_UP2 = 0x77;  // 'w'
const uint32_t KEYCODE_DN1 = 0x53;  // 'S'
const uint32_t KEYCODE_DN2 = 0x73;  // 's'
const uint32_t KEYCODE_LF1 = 0x41;  // 'A'
const uint32_t KEYCODE_LF2 = 0x61;  // 'a'
const uint32_t KEYCODE_RT1 = 0x44;  // 'D'
const uint32_t KEYCODE_RT2 = 0x64;  // 'd'

const uint32_t KEYCODE_PKBK = 0x50;  // hand brake or parking brake

// set throttle, gear, and brake
const uint32_t KEYCODE_SETT1 = 0x54;  // 'T'
const uint32_t KEYCODE_SETT2 = 0x74;  // 't'
const uint32_t KEYCODE_SETG1 = 0x47;  // 'G'
const uint32_t KEYCODE_SETG2 = 0x67;  // 'g'
const uint32_t KEYCODE_SETB1 = 0x42;  // 'B'
const uint32_t KEYCODE_SETB2 = 0x62;  // 'b'
const uint32_t KEYCODE_ZERO = 0x30;   // '0'

const uint32_t KEYCODE_SETQ1 = 0x51;  // 'Q'
const uint32_t KEYCODE_SETQ2 = 0x71;  // 'q'

// change action
const uint32_t KEYCODE_MODE = 0x6D;  // 'm'

// emergency stop
const uint32_t KEYCODE_ESTOP = 0x45;  // 'E'

// help
const uint32_t KEYCODE_HELP = 0x68;   // 'h'
const uint32_t KEYCODE_HELP2 = 0x48;  // 'H'

class Teleop {
 public:
  Teleop() {
    ResetControlCommand();
    node_ = CreateNode("teleop");
  }
  static void PrintKeycode() {
    system("clear");
    printf("=====================    KEYBOARD MAP   ===================\n");
    printf("HELP:               [%c]     |\n", KEYCODE_HELP);
    printf("Set Action      :   [%c]+Num\n", KEYCODE_MODE);
    printf("                     0 RESET ACTION\n");
    printf("                     1 START ACTION\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Set Gear:           [%c]+Num\n", KEYCODE_SETG1);
    printf("                     0 GEAR_NEUTRAL\n");
    printf("                     1 GEAR_DRIVE\n");
    printf("                     2 GEAR_REVERSE\n");
    printf("                     3 GEAR_PARKING\n");
    printf("                     4 GEAR_LOW\n");
    printf("                     5 GEAR_INVALID\n");
    printf("                     6 GEAR_NONE\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Throttle/Speed up:  [%c]     |  Set Throttle:       [%c]+Num\n",
           KEYCODE_UP1, KEYCODE_SETT1);
    printf("Brake/Speed down:   [%c]     |  Set Brake:          [%c]+Num\n",
           KEYCODE_DN1, KEYCODE_SETB1);
    printf("Steer LEFT:         [%c]     |  Steer RIGHT:        [%c]\n",
           KEYCODE_LF1, KEYCODE_RT1);
    printf("Parking Brake:     [%c]     |  Emergency Stop      [%c]\n",
           KEYCODE_PKBK, KEYCODE_ESTOP);
    printf("\n-----------------------------------------------------------\n");
    printf("Exit: Ctrl + C, then press enter to normal terminal\n");
    printf("===========================================================\n");
  }

  void KeyboardLoopThreadFunc() { return; }

  ControlCommand &control_command() { return control_command_; }

  Chassis::GearPosition GetGear(int32_t gear) {
    switch (gear) {
      case 0:
        return Chassis::GEAR_NEUTRAL;
      case 1:
        return Chassis::GEAR_DRIVE;
      case 2:
        return Chassis::GEAR_REVERSE;
      case 3:
        return Chassis::GEAR_PARKING;
      case 4:
        return Chassis::GEAR_LOW;
      case 5:
        return Chassis::GEAR_INVALID;
      case 6:
        return Chassis::GEAR_NONE;
      default:
        return Chassis::GEAR_INVALID;
    }
  }

  void GetPadMessage(PadMessage *pad_msg, int32_t int_action) {
    century::control::DrivingAction action =
        century::control::DrivingAction::RESET;
    switch (int_action) {
      case 0:
        action = century::control::DrivingAction::RESET;
        AINFO << "SET Action RESET";
        break;
      case 1:
        action = century::control::DrivingAction::START;
        AINFO << "SET Action START";
        break;
      default:
        AINFO << "unknown action: " << int_action << " use default RESET";
        break;
    }
    pad_msg->set_action(action);
    return;
  }

  double GetCommand(double val, double inc) {
    val += inc;
    if (val > 100.0) {
      val = 100.0;
    } else if (val < -100.0) {
      val = -100.0;
    }
    return val;
  }

  void Send() {
    century::common::util::FillHeader("control", &control_command_);
    control_command_writer_->Write(control_command_);
    ADEBUG << "Control Command send OK:" << control_command_.ShortDebugString();
  }

  void ResetControlCommand() {
    control_command_.Clear();
    control_command_.set_throttle(0.0);
    control_command_.set_brake(0.0);
    control_command_.set_steering_rate(0.0);
    control_command_.set_steering_target(0.0);
    control_command_.set_parking_brake(false);
    control_command_.set_speed(0.0);
    control_command_.set_acceleration(0.0);
    control_command_.set_reset_model(false);
    control_command_.set_engine_on_off(false);
    control_command_.set_driving_mode(Chassis::COMPLETE_MANUAL);
    control_command_.set_gear_location(Chassis::GEAR_INVALID);
    control_command_.mutable_signal()->set_turn_signal(
        VehicleSignal::TURN_NONE);
  }

  void OnChassis(const Chassis &chassis) { Send(); }

  int32_t Start() {
    if (is_running_) {
      AERROR << "Already running.";
      return -1;
    }
    is_running_ = true;
    chassis_reader_ = node_->CreateReader<Chassis>(
        FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
          OnChassis(*chassis);
        });
    control_command_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    keyboard_thread_.reset(
        new std::thread([this]() { KeyboardLoopThreadFunc(); }));
    if (keyboard_thread_ == nullptr) {
      AERROR << "Unable to create can client receiver thread.";
      return -1;
    }
    return 0;
  }

  void Stop() {
    if (is_running_) {
      is_running_ = false;
      if (keyboard_thread_ != nullptr && keyboard_thread_->joinable()) {
        keyboard_thread_->joinable();
        keyboard_thread_.reset();
        AINFO << "Teleop keyboard stopped [ok].";
      }
    }
  }

  bool IsRunning() const { return is_running_; }

 private:
  std::unique_ptr<std::thread> keyboard_thread_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Writer<ControlCommand>> control_command_writer_;
  ControlCommand control_command_;
  bool is_running_ = false;
  std::shared_ptr<century::cyber::Node> node_;
};

}  // namespace

int main(int32_t argc, char **argv) {
  century::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  Teleop teleop;

  if (teleop.Start() != 0) {
    AERROR << "Teleop start failed.";
    return -1;
  }
  Teleop::PrintKeycode();
  century::cyber::WaitForShutdown();
  teleop.Stop();
  AINFO << "Teleop exit done.";
  return 0;
}
