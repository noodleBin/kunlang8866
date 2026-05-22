/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/planning/proto/planning_aeb.pb.h"

using century::common::util::FillHeader;
using century::planning::AebResult;
using century::planning::AebWarningLevel;
using century::planning::AEBDebug;

class AebResultSender {
 public:
  AebResultSender() : node_(century::cyber::CreateNode("aeb_result_sender_" + std::to_string(getpid()))) {
    aeb_result_writer_ = node_->CreateWriter<AebResult>("/century/planning/planning_aeb");
    std::cout << "AebResultSender initialized with node: aeb_result_sender_" << getpid() << std::endl;
  }

  void SendAebResult(AebWarningLevel warning_level, bool ready_to_enable_aeb) {
    auto aeb_result = std::make_shared<AebResult>();

    FillHeader("aeb_result_sender", aeb_result.get());

    aeb_result->set_warning_level(warning_level);

    aeb_result->set_ready_to_enable_aeb(ready_to_enable_aeb);

    AEBDebug* debug = aeb_result->mutable_aeb_debug();
    debug->set_time(century::cyber::Time::Now().ToSecond());

    for (int i = 0; i < 10; i++) {
      aeb_result_writer_->Write(aeb_result);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    AINFO << "AebResult sent: warning_level=" << static_cast<int>(warning_level)
          << ", ready_to_enable_aeb=" << ready_to_enable_aeb;
  }

  void SendScenarios() {
    SendAebResult(AebWarningLevel::WARNING_LEVEL_NONE, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    SendAebResult(AebWarningLevel::WARNING_LEVEL_LOW, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    SendAebResult(AebWarningLevel::WARNING_LEVEL_MEDIUM, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    SendAebResult(AebWarningLevel::WARNING_LEVEL_HIGH, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

 private:
  std::unique_ptr<century::cyber::Node> node_;
  std::shared_ptr<century::cyber::Writer<AebResult>> aeb_result_writer_;
};

int main(int argc, char** argv) {
  if (!century::cyber::Init("aeb_result_sender_demo")) {
    return -1;
  }

  AebResultSender sender;

  sender.SendScenarios();

  std::this_thread::sleep_for(std::chrono::seconds(10));

  return 0;
}
