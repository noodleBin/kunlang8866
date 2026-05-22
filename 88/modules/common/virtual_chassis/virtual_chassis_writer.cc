/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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
#include "modules/canbus/proto/chassis.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/service_discovery/topology_manager.h"

using century::cyber::Rate;
using century::cyber::Time;

constexpr const char* file_path =
    "/century/modules/common/virtual_chassis/conf/1_chassis.pb.txt";
constexpr const char* node_name = "virtual_chassis";

const uint16_t kTimeKeep = 45;
const double kDoubleBrake = 22.0;
const double kDoublethrottle = 0.0;
const double kDoubleFreq = 50.0;
const double kDoubleSpeed = 0.0;
int main(int argc, char* argv[]) {
  century::cyber::Init(argv[0]);

  auto talker_node = century::cyber::CreateNode(node_name);

  auto chassis_writer_ = talker_node->CreateWriter<century::canbus::Chassis>(
      "/century/canbus/chassis");
  century::canbus::Chassis msg;

  if (!century::cyber::common::GetProtoFromFile(file_path, &msg)) {
    return 0;
  }
  uint64_t seq = 0;

  msg.set_brake_percentage(kDoubleBrake);
  msg.set_throttle_percentage(kDoublethrottle);
  msg.set_gear_location(
      century::canbus::Chassis_GearPosition::Chassis_GearPosition_GEAR_PARKING);
  msg.set_speed_mps(kDoubleSpeed);

  Rate rate(kDoubleFreq);
  auto start_time = century::cyber::Time::Now().ToSecond();
  while (century::cyber::OK()) {
    auto time = century::cyber::Time::Now().ToSecond();

    if (time - start_time > kTimeKeep) {
      break;
    }

    msg.set_steering_timestamp(time);
    msg.mutable_header()->set_timestamp_sec(time);
    msg.mutable_header()->set_sequence_num(seq++);
    chassis_writer_->Write(msg);
    rate.Sleep();
  }
  return 0;
}
