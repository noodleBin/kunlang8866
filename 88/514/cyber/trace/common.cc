/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
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

#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#include "cyber/common/global_data.h"
#include "cyber/trace/common.h"
#include "cyber/trace/trace_enums.h"

namespace century {
namespace trace {

namespace {
constexpr uint64_t kSecToNsec = 1000000000LL;
constexpr const char* kCmdLinePath = "/proc/self/cmdline";
}  // namespace

uint64_t Timespec2Nanoseconds(const timespec& ts) {
  return ts.tv_sec * kSecToNsec + ts.tv_nsec;
}

bool PathExists(const char* path) {
  struct stat buffer;
  return (stat(path, &buffer) == 0);
}

std::string GetProcessName() {
  if (cyber::common::GlobalData::Instance()) {
    return cyber::common::GlobalData::Instance()->ProcessGroup();
  }

  std::string processName;
  std::ifstream cmdlineFile(kCmdLinePath);
  if (cmdlineFile.is_open()) {
    std::getline(cmdlineFile, processName, '\0');
    size_t lastSlashIndex = processName.find_last_of('/');
    if (lastSlashIndex != std::string::npos) {
      processName = processName.substr(lastSlashIndex + 1);
    }
    cmdlineFile.close();
  }
  return processName;
}

}  // namespace trace
}  // namespace century
