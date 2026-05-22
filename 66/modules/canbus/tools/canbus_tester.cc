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

#include <thread>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"

#include "gflags/gflags.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"

using century::control::ControlCommand;
using century::cyber::Rate;
using century::cyber::Reader;
using century::cyber::Writer;

int main(int32_t argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;

  // init cyber framework
  century::cyber::Init("testing_canbus_tester");
  std::shared_ptr<century::cyber::Node> node_(
      century::cyber::CreateNode("canbus_tester"));
  std::shared_ptr<Writer<ControlCommand>> control_command_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);

  ControlCommand control_cmd;
  if (!century::cyber::common::GetProtoFromFile(FLAGS_canbus_test_file,
                                               &control_cmd)) {
    AERROR << "failed to load file: " << FLAGS_canbus_test_file;
    return -1;
  }

  Rate rate(1.0);  // frequency

  while (century::cyber::OK()) {
    // pub.publish(msg);
    control_command_writer_->Write(control_cmd);
    rate.Sleep();
  }

  return 0;
}
