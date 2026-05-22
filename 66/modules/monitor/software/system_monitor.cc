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

#include "modules/monitor/software/system_monitor.h"

#include <limits.h>
#include <sys/statfs.h>
#include <unistd.h>

#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/process.hpp>

#include "gflags/gflags.h"
#include "third_party/safec/include/safe_mem_lib.h"
#include "third_party/safec/include/safe_str_lib.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/common_def.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/monitor/common/monitor_gflags.h"
#include "modules/monitor/common/monitor_manager.h"

#ifndef RETURN_VAL_MSG_IF
#define RETURN_VAL_MSG_IF(condition, val, msg) \
  if (condition) {                             \
    AERROR << msg;                             \
    return (val);                              \
  }
#endif

DEFINE_string(system_monitor_name, "SystemMonitor",
              "Name of the system monitor.");
DEFINE_double(system_monitor_interval, 0.25,
              "system status checking interval in seconds.");

namespace century {
namespace monitor {
namespace {

namespace bp = boost::process;

constexpr uint16_t kByteToGbShift = 30;
constexpr uint16_t kMaxStringLength = 256;
constexpr double kDoubleEpsilon = 1.0e-8;
constexpr uint8_t kMonitorFeq = 2;
constexpr const char* kBaseMap = "base_map.bin";
constexpr const char* kRoutingMap = "routing_map.bin";
constexpr const char* kSimMap = "sim_map.bin";

#if defined __aarch64__
const int DOMAIN_TYPE = common::DomainType::AARCH64;
constexpr const char* platform = "aarch64";
#else
const int DOMAIN_TYPE = common::DomainType::X86;
constexpr const char* platform = "x86";
#endif
// static CpuUsageInfo old_cpu_occupy;
// static CpuUsageInfo cpu_occupy;
}  // namespace

static double CalculateDiskUsage(const struct statfs& disk_info) {
  const auto block_size = disk_info.f_bsize;
  const auto total_blocks = disk_info.f_blocks;
  const auto free_blocks = disk_info.f_bfree;
  const auto avail_blocks = disk_info.f_bavail;

  const auto total_size = (block_size * total_blocks) >> kByteToGbShift;
  const auto free_size = (free_blocks * block_size) >> kByteToGbShift;
  const auto avail_size = (avail_blocks * block_size) >> kByteToGbShift;

  AINFO << std::fixed << std::setprecision(2)
        << "Disk total size = " << total_size << " GB, "
        << "free size = " << free_size << " GB, "
        << "available size = " << avail_size << " GB";

  return (total_size == 0)
             ? 1.0
             : static_cast<double>(total_size - free_size) / total_size;
}

SystemMonitor::SystemMonitor() {
  std::string node_name = "SystemMonitorNode";

  cpu_processor_number_ = std::thread::hardware_concurrency();
  monitor_data_addition_ = std::make_shared<SystemMonitorData>();
  system_data_ = std::make_shared<SystemData>();
}

void SystemMonitor::UpdateSystemStatus() {
  const auto cpu_usage = GetCpuInfo();
  const auto disk_usage = GetDiskInfo();
  const auto mem_usage = GetMemoryInfo();

  system_data_->set_cpu_usage(cpu_usage);
  system_data_->set_disk_usage(disk_usage);
  system_data_->set_mem_usage(mem_usage);
}

void SystemMonitor::CheckAndUpdateSystemStatus() {
  static uint8_t kCheckFrequency = 0;
  RETURN_IF(++kCheckFrequency < kMonitorFeq)
  kCheckFrequency = 0;
  UpdateSystemStatus();
  GetMonitedAdditionData();
}

void SystemMonitor::MonitorConfig() {
  static uint8_t kCheckFrequency = 0;
  constexpr uint8_t kMaxMonitorSeconds = 5 * kMonitorFeq;
  RETURN_IF(++kCheckFrequency < kMaxMonitorSeconds)
  kCheckFrequency = 0;
  GetConfigVersion();
}

double SystemMonitor::GetDiskInfo() const {
  std::string exec_str(kMaxStringLength, '\0');
  ssize_t num = readlink("/proc/self/exe", const_cast<char*>(exec_str.data()),
                         kMaxStringLength);
  RETURN_VAL_MSG_IF(num == -1, -1, "readlink error")

  boost::filesystem::path exe_path(exec_str);
  const auto is_path_exist = boost::filesystem::exists(exe_path);
  RETURN_VAL_MSG_IF((!is_path_exist), -1, "wrong process path")

  const auto process_path = exe_path.parent_path().string();
  struct statfs disk_info;
  RETURN_VAL_MSG_IF(statfs(process_path.c_str(), &disk_info) != 0, -1,
                    "statfs error")

  const auto disk_usage = CalculateDiskUsage(disk_info);
  return disk_usage;
}

double SystemMonitor::GetMemoryInfo() const {
  std::ifstream meminfo_file("/proc/meminfo");
  RETURN_VAL_MSG_IF(!meminfo_file.is_open(), -1,
                    " GetMemoryInfo() error! File not exist")

  constexpr int kFileLines = 5;
  std::array<int, kFileLines> mem_info = {-1};
  std::string line;

  for (int i = 0; i < kFileLines && std::getline(meminfo_file, line); ++i) {
    std::istringstream iss(line);
    std::string name;
    int value;
    if (!(iss >> name >> value)) {
      AERROR << "GetMemoryInfo error! Invalid format!";
      return -1;
    }
    mem_info[i] = value;
  }

  meminfo_file.close();

  int mem_left = mem_info[1] + mem_info[3] + mem_info[4];
  AINFO << "Total memory size = " << mem_info[0]
        << " KB, used memory size = " << mem_info[0] - mem_left
        << " KB, free memory size = " << mem_info[1]
        << " KB, buffer memory size = " << mem_info[3]
        << " KB, cached memory size = " << mem_info[4] << " KB";

  return (mem_info[0] == 0)
             ? 1.0
             : static_cast<double>(mem_info[0] - mem_left) / mem_info[0];
}

double SystemMonitor::GetCpuUsage(const CpuUsageInfo* last,
                                  const CpuUsageInfo* now) const {
  auto sum = [](const auto info) {
    return info->user + info->nice + info->system + info->idle + info->lowait +
           info->irq + info->softirq;
  };

  double total_diff = sum(now) - sum(last);
  double idle_diff = now->idle - last->idle;

  return (total_diff < kDoubleEpsilon) ? 0.0
                                       : (total_diff - idle_diff) / total_diff;
}

double SystemMonitor::GetCpuInfo() const {
  const std::string filename = "/proc/stat";
  std::ifstream file_stream(filename);
  RETURN_VAL_MSG_IF(
      !file_stream.is_open(), -1.0,
      std::string("GetCpuInfo() error! File not exist: ").append(filename))

  std::string line;
  if (!std::getline(file_stream, line) || line.substr(0, 3) != "cpu") {
    AERROR << "GetCpuInfo error! Fail to read cpu information!";
    return -1.0;
  }

  auto read_cpu_usage = [](const std::string& line) {
    std::istringstream iss(line);
    CpuUsageInfo info;
    iss >> info.name >> info.user >> info.nice >> info.system >> info.idle >>
        info.lowait >> info.irq >> info.softirq;
    return info;
  };

  CpuUsageInfo cur_cpu_usage = read_cpu_usage(line);

  auto cpu_use = GetCpuUsage(&old_cpu_usage_, &cur_cpu_usage);
  old_cpu_usage_ = cur_cpu_usage;

  AINFO << "cpu usage = " << cpu_use << "%, the number of cpu processor is "
        << cpu_processor_number_;
  return cpu_use;
}

void SystemMonitor::GetMonitedAdditionData() {
  auto* system_data = monitor_data_addition_->mutable_system_data();
  system_data->CopyFrom(*system_data_);

  auto* ota_package_data = monitor_data_addition_->mutable_ota_package_data();
  ota_package_data->set_software_version(ad_version_status_.version);
  ota_package_data->set_monitor_software_version(
      ad_version_status_.monitor_version);
  ota_package_data->set_map_file(ad_version_status_.map_file);
  ota_package_data->set_base_map(ad_version_status_.base_map);
  ota_package_data->set_sim_map(ad_version_status_.sim_map);
  ota_package_data->set_routing_map(ad_version_status_.routing_map);
}

void SystemMonitor::RunOnce(const double current_time) {
  if (!ad_version_status_.init.load()) {
    GetAdVersion();
    GetConfigVersion();
  }
  MonitorConfig();
  CheckAndUpdateSystemStatus();
}

int SystemMonitor::ReadFileMd5Sum(const std::string& file,
                                  std::string* content) {
  try {
    std::string command = "md5sum " + file;
    bp::ipstream pipe;
    bp::child child_process(command, bp::std_out > pipe);

    std::string line;
    if (std::getline(pipe, line) && !line.empty()) {
      auto pos = line.find(' ');
      if (pos != std::string::npos) {
        line.erase(pos);
      }
      *content = line;
    } else {
      content->clear();
      AERROR << "Error read md5sum failed. " << std::endl;
    }

    child_process.wait();
  } catch (const std::exception& e) {
    AERROR << "Error executing md5sum command: " << e.what() << std::endl;
    return errno;
  }

  return 0;
}

void SystemMonitor::GetConfigVersion() {
  gflags::ReadFromFlagsFile(FLAGS_common_config_path, "system_monitor", true);
  ad_version_status_.map_file = FLAGS_map_dir;

  auto get_map_path = [](const auto& path) {
    return FLAGS_map_dir + '/' + path;
  };

  auto get_map_context = [&](const auto& map_name, auto& map_context) {
    auto map_path = get_map_path(map_name);

    if (!century::cyber::common::PathExists(map_path)) {
      ad_version_status_.base_map =
          std::string(platform) + " " + map_path + " not exist";
      return false;
    }

    std::string context;
    ReadFileMd5Sum(map_path.c_str(), &context);
    if (context.empty()) {
      return false;
    }

    map_context = context;

    return true;
  };

  auto read_flag = get_map_context(kBaseMap, ad_version_status_.base_map);
  RETURN_IF(!read_flag)
  read_flag = get_map_context(kRoutingMap, ad_version_status_.routing_map);
  RETURN_IF(!read_flag)
  read_flag = get_map_context(kSimMap, ad_version_status_.sim_map);
}

void SystemMonitor::GetAdVersion() {
  auto read_file_content = [](const auto& file, auto& content) {
    try {
      std::ifstream input_file(file);
      if (!input_file.is_open()) {
        AERROR << "Error open file failed: " << file;
        return;
      }

      content.clear();
      std::string line;
      if (std::getline(input_file, line)) {
        content = std::move(line);
      }
      input_file.close();
    } catch (const std::exception& e) {
      AERROR << "Error read file failed: " << e.what() << ", file: " << file;
    }
  };

  std::string ad_version_file = FLAGS_adplan_version_file;
#ifdef __aarch64__
  ad_version_file = FLAGS_adsens_version_file;
#endif
  read_file_content(ad_version_file, ad_version_status_.version);
  // TODO(zheqiang.wu): MD5sum currently using software version files
  ReadFileMd5Sum(ad_version_file, &ad_version_status_.monitor_version);

  ad_version_status_.init.store(true);

  AINFO << "ad_version_file:" << ad_version_file
        << ", version:" << ad_version_status_.version
        << ", monitor_version:" << ad_version_status_.monitor_version;
}
}  // namespace monitor
}  // namespace century
