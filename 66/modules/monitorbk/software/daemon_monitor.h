/******************************************************************************
 * Copyright 2022 The century Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/dreamview/proto/hmi_mode.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/monitor/proto/monitor_switch_config.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "cyber/base/thread_safe_queue.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/timer/timer.h"
#include "modules/common/configs/common_def.h"
#include "modules/monitor/common/monitor_reader_factory.h"
#include "modules/monitor/common/monitor_typedef.h"
#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/software/system_monitor.h"

namespace century {
namespace monitor {

template <typename MsgType>
using ReaderPtr = std::shared_ptr<cyber::Reader<MsgType>>;

class DaemonMonitor : public RecurrentRunner {
 public:
  DaemonMonitor();
  void RunOnce(const double current_time) override;

 private:
  bool LoadMonitorConfig();
  void InitializeTopicMonitorConfig();
  void InitializeChannelInfo();
  void InitializeMonitorData();

  bool IsDomainMatching(const std::string& channel_name) const;

  void CheckModuleChannels();
  void GetSystemMonitorData();
  void GetRunningStatus();
  void GetSystemStatusFaultData();
  void DaemonMonitorStatus();

  void CollectFaultData();
  void PublishMonitoredData();

  bool GetDiskWriteStatus();
  bool GetLogWriteStatus();
  bool GetCyberRecorderWriteStatus();
  int GetModifiedFilesWithin(const std::string& path, int times,
                             const std::string& exclude = "") const;
  void CheckDiskWritableStatus();

  MonitorErrorCode CheckNetwork();
  bool HostReachable(const std::string& address) const;
  bool CheckLocalNetwork(const std::string& local_ip) const;
  void CreateCheckNetworkTask();
  //   void GetSuitableDiffTimeThreshold(const std::string& channel);

  void CheckPredictionStatus();

 private:
  void InitializeChannelWarnings();
  void InitializeChannelErrors();
  void InitializeFrameTime();

  void BootSwitchConfigInit();
  void CreateBootSwitchService() noexcept;
  void BootSwitch(int32_t type);
  void AdRecovery(int32_t type);

 private:
  std::unordered_map<std::string, MonitorConfig> monitor_configs_;
  std::vector<std::string> required_process_;
  std::shared_ptr<cyber::Node> daemon_monitor_node_;
  ReaderPtr<canbus::Chassis> chassis_reader_;
  ReaderPtr<control::ControlCommand> control_reader_;
  ReaderPtr<planning::ADCTrajectory> planning_reader_;
  ReaderPtr<monitor::MonitoredData> monitor_reader_;

  std::shared_ptr<cyber::Timer> time_task_;
  std::shared_ptr<cyber::Writer<monitor::MonitoredData>> monitor_writer_;
  std::shared_ptr<monitor::MonitoredData> monitored_data_;
  std::unordered_map<std::string, MonitorErrorCode> channel_to_warning_;
  std::unordered_map<std::string, MonitorErrorCode> channel_to_error_;

  ErrorInfoQueuePtr error_info_queue_;
  std::unordered_map<std::string, bool> channel_to_switch_flag_;
  std::unordered_map<
      int, const google::protobuf::RepeatedPtrField<monitor::ConfigInfo>*>
      switch_config_map_;
  std::unordered_map<std::string, std::atomic<ChannelFreq<double, int>>>
      channel_freqs_;
  monitor::Platform switch_config_;

  std::atomic_bool planning_in_teb_ = {false};
  std::atomic_bool auto_driving_status_ = {false};
  std::atomic<double> cur_time_second_ = {0.0};  // Not an exact real-time
  double last_check_disk_time_ = {0.0};
  uint16_t suitable_diff_time_threshold_ = {0};

  std::unique_ptr<monitor::SystemMonitor> system_monitor_;
  std::shared_ptr<monitor::SystemMonitorData> system_monitor_data_ = nullptr;

 private:
  bool LoadErrorCodeInfos() noexcept;

 private:
  //   new field
  std::unordered_map<MonitorErrorCode, ErrorInfo> error_code_infos_;
  std::shared_ptr<DaemonReaderManager> daemon_reader_manager_;
};

}  // namespace monitor
}  // namespace century
