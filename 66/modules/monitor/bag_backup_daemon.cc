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

#include <array>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/proto/record_control.pb.h"

namespace {

constexpr const int KIntBufferSize = 256;

constexpr const char* kStartCmd =
    "nohup bash /century/bag_backup.sh >/dev/null 2>&1 </dev/null &";
constexpr const char* kStatusCmd =
    "ps -ef | grep bag_backup.sh | grep -v grep | wc -l";

bool ExecCommand(const std::string& cmd, std::string& output) {
  output.clear();
  if (cmd.empty()) {
    return false;
  }

  std::array<char, KIntBufferSize> buffer{};
  std::string full_cmd = cmd + " 2>&1";

  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(full_cmd.c_str(), "r"),
                                                pclose);
  if (!pipe) {
    return false;
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    output.append(buffer.data());
  }

  int status = pclose(pipe.release());
  if (-1 == status) {
    return false;
  }
  return true;
}

int QueryRecordStatus(std::string* error) {
  std::string result;
  bool ret = ExecCommand(kStatusCmd, result);
  if (!ret) {
    if (error) {
      *error = "ExecCommand failed";
    }
    return 0;
  }

  if (!result.empty()) {
    result.erase(result.find_last_not_of(" \n\r\t") + 1);
  }

  try {
    return std::stoi(result);
  } catch (const std::exception& e) {
    if (error) {
      *error = e.what();
    }
    return 0;
  }
}

void PublishStatus(
    const std::shared_ptr<century::cyber::Writer<century::dreamview::RecordStatus>>&
        writer,
    int status, const std::string& error, uint64_t timestamp_ms) {
  if (!writer) {
    return;
  }
  century::dreamview::RecordStatus response;
  response.set_status(status);
  if (!error.empty()) {
    response.set_error(error);
  }
  response.set_timestamp_ms(timestamp_ms);
  writer->Write(response);
}

void MonitorUntilStopped(
    const std::shared_ptr<century::cyber::Writer<century::dreamview::RecordStatus>>&
        writer,
    uint64_t timestamp_ms) {
  static std::atomic<bool> monitoring{false};
  bool expected = false;
  if (!monitoring.compare_exchange_strong(expected, true)) {
    return;
  }

  std::thread([writer, timestamp_ms]() {
    int last_status = -1;
    while (true) {
      std::string error;
      int status = QueryRecordStatus(&error);
      if (status != last_status) {
        PublishStatus(writer, status, error, timestamp_ms);
        last_status = status;
      }
      if (0 == status) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    monitoring.store(false);
  }).detach();
}

}  // namespace

int main(int argc, char** argv) {
  century::cyber::Init(argv[0]);
  auto node = century::cyber::CreateNode("bag_backup_daemon");
  auto status_writer =
      node->CreateWriter<century::dreamview::RecordStatus>(
          FLAGS_record_status_topic);

  auto reader = node->CreateReader<century::dreamview::RecordCommand>(
      FLAGS_record_cmd_topic,
      [status_writer](
          const std::shared_ptr<century::dreamview::RecordCommand>& msg) {
        if (!msg) {
          return;
        }
        uint64_t timestamp_ms =
            msg->has_timestamp_ms() ? msg->timestamp_ms() : 0;
        if (century::dreamview::RecordCommand::START_RECORD_CURRENT ==
            msg->type()) {
          std::string output;
          bool ret = ExecCommand(kStartCmd, output);
          std::string error;
          if (!ret) {
            error = output.empty() ? "start command failed" : output;
          }
          int status = QueryRecordStatus(&error);
          PublishStatus(status_writer, status, error, timestamp_ms);
          if (status > 0) {
            MonitorUntilStopped(status_writer, timestamp_ms);
          }
          return;
        }
        if (century::dreamview::RecordCommand::RECORD_STATUS ==
            msg->type()) {
          std::string error;
          int status = QueryRecordStatus(&error);
          PublishStatus(status_writer, status, error, timestamp_ms);
          return;
        }
      });

  (void)reader;
  century::cyber::WaitForShutdown();
  return 0;
}
