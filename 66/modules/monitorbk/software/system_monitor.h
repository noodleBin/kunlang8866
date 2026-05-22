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
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/monitor/proto/system_status.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/timer/timer.h"
#include "modules/monitor/common/recurrent_runner.h"

namespace century {
namespace monitor {

struct CpuUsageInfo {
  std::string name;
  uint32_t user;
  uint32_t nice;
  uint32_t system;
  uint32_t idle;
  uint32_t lowait;
  uint32_t irq;
  uint32_t softirq;
};

struct AdVersionStatus {
  std::atomic_bool init = {false};
  std::string monitor_version;
  std::string version;
  std::string map_file;
  std::string base_map;
  std::string sim_map;
  std::string routing_map;
};

class SystemMonitor {
 public:
  SystemMonitor();
  void RunOnce(const double current_time);

  std::shared_ptr<SystemMonitorData> GetSystemMonitorData() const {
    return monitor_data_addition_;
  }

 private:
  void UpdateSystemStatus();
  void CheckAndUpdateSystemStatus();

  double GetDiskInfo() const;
  double GetMemoryInfo() const;
  double GetCpuUsage(const CpuUsageInfo* old, const CpuUsageInfo* now) const;
  double GetCpuInfo() const;
  void GetAdVersion();

  void GetMonitedAdditionData();

 private:
  void MonitorConfig();
  void GetConfigVersion();
  int ReadFileMd5Sum(const std::string& file, std::string* content);

 private:
  std::shared_ptr<SystemMonitorData> monitor_data_addition_ = nullptr;
  std::shared_ptr<SystemData> system_data_ = nullptr;
  AdVersionStatus ad_version_status_;
  uint16_t cpu_processor_number_;

  mutable CpuUsageInfo old_cpu_usage_;
};

}  // namespace monitor
}  // namespace century
