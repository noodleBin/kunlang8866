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

#include "modules/monitor/software/daemon_monitor.h"

#include <limits.h>
#include <sys/statfs.h>
#include <unistd.h>

#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>

#include "boost/filesystem.hpp"
#include "gflags/gflags.h"
#include "third_party/tinyxml/tinystr.h"
#include "third_party/tinyxml/tinyxml.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/timedelay/timedelay.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/util.h"
#include "modules/monitor/common/monitor_gflags.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(daemon_monitor_name, "DaemonMonitor",
              "Name of the daemon monitor.");

DEFINE_double(daemon_monitor_interval, 0.25,
              "Daemon status checking interval in seconds.");

#define IF_DOMAIN_MATCH(channel_name)                           \
  if ((DOMAIN_TYPE == monitor_configs_[channel_name].domain) || \
      (true == monitor_configs_[channel_name].cross_domain))

// #define ERR_CODE_MSG(err_code, code) [err_code] = {code, (#err_code) + 2}
namespace century {
namespace monitor {
namespace {
#if defined __aarch64__
const int DOMAIN_TYPE = common::DomainType::AARCH64;
constexpr const char* kServiceName = "/century/monitor/monitor_service_orin";
#else
const int DOMAIN_TYPE = common::DomainType::X86;
constexpr const char* kServiceName = "/century/monitor/monitor_service_x86";
#endif

// constexpr uint8_t kMaxCheckSkips = 10;
constexpr uint16_t kMaxCacheFrames = 2;
// constexpr uint16_t kCubtekRadarNormalStatus = 1;
// constexpr uint16_t kUdasUltrasonicNormalStatus = 1;
constexpr uint16_t kDiskCheckInterval = 60;
// constexpr uint16_t kDiffTimeThreshold = 2;
constexpr uint16_t kMaxStringLength = 256;

// constexpr double kMinimumTime = 1.0e-6;
// constexpr double kDoubleEpsilon = 1.0e-8;
// constexpr double kDoubleMaximumDiffTime = 1.0e2;
constexpr double kDoubleDiskCheckFreq = 60.0;
// constexpr double kSecondsToNanoseconds = 1e9;
// constexpr double kMinFrequencyEpsilon = 1.0e-6;
constexpr double kMaxFrequencyEpsilon = 65535.0;
// constexpr double kTransmissionLatencyDuration = 5.0;
// constexpr double kLocRsDiffTime = 1.0;
// constexpr double kLocKillDiffTime = 5.0;
// constexpr double kPerceptionDiffCurTime = 1.0;
// constexpr double kModuleDiffCurTime = 1.0;
// constexpr double kPerceptionDiffPredictionTime = 1.0;
// constexpr double kLastKillTimeDiffNowTime = 1.0;
// constexpr double kTwoSeconds = 2.0;
// constexpr double kRsSdkHangingCheckPeriod = 11.0;
// constexpr double kPoorAccuracyDuration = 30.0;

// constexpr const uint32_t kDoubleCheckNetworkFreq = 4 * 1000;

constexpr const char* kLogPath = "/century/data/log/";
constexpr const char* kBagPath = "/century/data/bag/";
constexpr const char* kWriteTestFilePath = "/century/data/log/test.txt";
constexpr const char* kErrorCodeConfigPath =
    "/century/modules/monitor/config/monitor_error_codes.yml";
constexpr const char* kMonitorConfigPath =
    "/century/modules/monitor/config/monitor_config.xml";

template <typename EMType>
inline void SetErrorMessage(EMType* error_info, const ErrorInfo& info) {
  error_info->set_code(info.code);
  error_info->set_code_msg(info.message);
}

std::unique_ptr<std::mutex> mutex_error_code = std::make_unique<std::mutex>();

inline void operator>>(const YAML::Node& node, ErrorInfo& error_info) {
  error_info.name = node["name"].as<std::string>();
  error_info.value = node["value"].as<int>();
  error_info.code = node["code"].as<std::string>();
  error_info.message = node["message"].as<std::string>();
}

struct TopicMonitorSwitch {
  bool enable;
  module_type module;
  std::string topic;
};

const std::vector<TopicMonitorSwitch> kTopicMonitorSwitchList = {
    {FLAGS_chassis_topic_monitor_switch, module_type::E_CHASSIS,
     FLAGS_chassis_topic},
    {FLAGS_control_topic_monitor_switch, module_type::E_CONTROL,
     FLAGS_control_command_topic},
    {FLAGS_perceptionObstacle_topic_monitor_switch, module_type::E_PERCEPTION,
     FLAGS_perception_obstacle_topic},
    {FLAGS_trafficLigth_topic_monitor_switch, module_type::E_TRAFFIC_LIGHT,
     FLAGS_traffic_light_detection_topic},
    {FLAGS_planning_topic_monitor_switch, module_type::E_PLANNING,
     FLAGS_planning_trajectory_topic},
    {FLAGS_prediction_topic_monitor_switch, module_type::E_PREDICTION,
     FLAGS_prediction_topic},
    {FLAGS_localization_topic_monitor_switch, module_type::E_LOCALIZATION,
     FLAGS_localization_topic},
    {FLAGS_loc_topic_monitor_switch, module_type::E_LOCALIZATION,
    FLAGS_loc_topic},
    {FLAGS_mems_topic_monitor_switch, module_type::E_MEMS,
     FLAGS_tracker_mems_topic},
    {FLAGS_radar_topic_monitor_switch, module_type::E_RADAR,
     FLAGS_tracker_radar_topic},
    {FLAGS_rs_topic_monitor_switch, module_type::E_RS, FLAGS_tracker_rs_topic},
    {FLAGS_ultrasonic_topic_monitor_switch, module_type::E_ULTRASONIC,
     FLAGS_tracker_ultrasonic_topic},
    {FLAGS_camera_topic_monitor_switch, module_type::E_CAMERA,
     FLAGS_tracker_camera_topic},
    {FLAGS_cubtek_radar_eol_switch, module_type::E_RADAR_CUBTEK,
     FLAGS_cubtek_radar_eol_topic},
    {FLAGS_mcloud_topic_monitor_switch, module_type::E_MCLOUD,
     FLAGS_mcloud_info_topic},
    {FLAGS_front_12mm_status_topic_monitor_switch, module_type::E_CAMERA,
     FLAGS_front_12mm_status_topic},
    {FLAGS_front_3mm_status_topic_monitor_switch, module_type::E_CAMERA,
     FLAGS_front_3mm_status_topic},
    {FLAGS_back_3mm_status_topic_monitor_switch, module_type::E_CAMERA,
     FLAGS_back_3mm_status_topic},
    {FLAGS_mems_status_topic_monitor_switch, module_type::E_MEMS,
     FLAGS_mems_status_topic},
    {FLAGS_udas_ultrasonic_eol_monitor_switch, module_type::E_UDAS_ULTRASONIC,
     FLAGS_udas_ultrasonic_eol_topic},
    {FLAGS_imu_raw_topic_monitor_switch, module_type::E_IMU_RAW,
     FLAGS_imu_raw_topic}};

}  // namespace

class MonitorConfigLoader {
 public:
  /**
   * @brief Load monitor config from xml file
   * @param file_path file path of monitor config
   * @return true if load config successfully
   */
  bool LoadConfig(const std::string& file_path);

  /**
   * @brief Get all monitor config
   * @return monitor config map
   */
  const std::unordered_map<std::string, MonitorConfig>& GetMonitorConfigs()
      const {
    return monitor_configs_;
  }

  /**
   * @brief Get required processes
   * @return required process list
   */
  const std::vector<std::string>& GetRequiredProcesses() const {
    return required_process_;
  }

 private:
  /**
   * @brief Parse process configs
   * @param xml_root root element of xml
   * @return true if parse successfully
   */
  bool ParseProcessConfigs(const TiXmlElement* xml_root);

  /**
   * @brief Parse running processes
   * @param xml_root root element of xml
   * @return true if parse successfully
   */
  bool ParseRunningProcesses(const TiXmlElement* xml_root);

  /**
   * @brief Get text of element
   * @param parent parent element
   * @param element_name element name
   * @return text of element
   */
  std::string GetElementText(const TiXmlElement* parent,
                             const std::string& element_name);

  /**
   * @brief Get float of element
   * @param parent parent element
   * @param element_name element name
   * @return float of element
   */
  float GetElementFloat(const TiXmlElement* parent,
                        const std::string& element_name);

  /**
   * @brief Get domain type of element
   * @param parent parent element
   * @param element_name element name
   * @return domain type of element
   */
  common::DomainType GetElementDomain(const TiXmlElement* parent,
                                      const std::string& element_name);

  /**
   * @brief Get boolean of element
   * @param parent parent element
   * @param element_name element name
   * @return boolean of element
   */
  bool GetElementBoolean(const TiXmlElement* parent,
                         const std::string& element_name);

  /**
   * @brief monitor config map
   */
  std::unordered_map<std::string, MonitorConfig> monitor_configs_;

  /**
   * @brief required process list
   */
  std::vector<std::string> required_process_;
};

bool MonitorConfigLoader::LoadConfig(const std::string& file_path) {
  TiXmlDocument doc;
  if (!doc.LoadFile(file_path.c_str()) || doc.Error()) {
    ADEBUG << file_path << " open error";
    return false;
  }

  const TiXmlElement* xml_root = doc.FirstChildElement("config");
  if (!xml_root) {
    AERROR << "xml has no config root";
    return false;
  }

  if (!ParseProcessConfigs(xml_root)) {
    AERROR << "Failed to parse process configurations";
    return false;
  }

  if (!ParseRunningProcesses(xml_root)) {
    AERROR << "Failed to parse running processes";
    return false;
  }

  return true;
}

bool MonitorConfigLoader::ParseProcessConfigs(const TiXmlElement* xml_root) {
  const TiXmlElement* xml_config = xml_root->FirstChildElement("process");
  if (!xml_config) {
    AERROR << "xml has no process";
    return false;
  }

  for (; xml_config != nullptr;
       xml_config = xml_config->NextSiblingElement("process")) {
    MonitorConfig config;
    if (xml_config->Attribute("name")) {
      config.process_name = xml_config->Attribute("name");
    }
    config.channel = GetElementText(xml_config, "channel");
    config.threshold_freq = GetElementFloat(xml_config, "freq_threshold");
    config.expected_freq = GetElementFloat(xml_config, "freq_expected");
    config.domain = GetElementDomain(xml_config, "domain_type");
    config.cross_domain = GetElementBoolean(xml_config, "cross_domain");

    monitor_configs_.emplace(config.channel, config);
  }

  AINFO << "monitor_config.xml process size: " << monitor_configs_.size();
  return true;
}

bool MonitorConfigLoader::ParseRunningProcesses(const TiXmlElement* xml_root) {
  const TiXmlElement* xml_process =
      xml_root->FirstChildElement("running_process");
  if (!xml_process) return false;

  for (const TiXmlElement* xml_name =
           xml_process->FirstChildElement("process_name");
       xml_name != nullptr;
       xml_name = xml_name->NextSiblingElement("process_name")) {
    required_process_.emplace_back(xml_name->GetText());
  }

  return !required_process_.empty();
}

std::string MonitorConfigLoader::GetElementText(
    const TiXmlElement* parent, const std::string& element_name) {
  const TiXmlElement* element = parent->FirstChildElement(element_name.c_str());
  return element ? element->GetText() : "";
}

float MonitorConfigLoader::GetElementFloat(const TiXmlElement* parent,
                                           const std::string& element_name) {
  std::string text = GetElementText(parent, element_name);
  return text.empty() ? 0.0f : std::stof(text);
}

common::DomainType MonitorConfigLoader::GetElementDomain(
    const TiXmlElement* parent, const std::string& element_name) {
  std::string domain_str = GetElementText(parent, element_name);
  return (domain_str == "aarch64") ? common::DomainType::AARCH64
                                   : common::DomainType::X86;
}

bool MonitorConfigLoader::GetElementBoolean(const TiXmlElement* parent,
                                            const std::string& element_name) {
  std::string text = GetElementText(parent, element_name);
  return (text == "true");
}

bool DaemonMonitor::LoadErrorCodeInfos() noexcept {
  try {
    auto param_root_ = YAML::LoadFile(kErrorCodeConfigPath);
    const auto error_size = param_root_["errors"].size();
    for (auto i = 0u; i < error_size; i++) {
      param_root_["errors"][i] >>
          error_code_infos_[static_cast<MonitorErrorCode>(i)];
    }
    AINFO << "error code size: " << error_code_infos_.size() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }

  return true;
}

bool DaemonMonitor::LoadMonitorConfig() {
  MonitorConfigLoader monitor_config_loader;
  if (!monitor_config_loader.LoadConfig(kMonitorConfigPath)) {
    AERROR << "Failed to load monitor config";
    return false;
  }

  monitor_configs_ = monitor_config_loader.GetMonitorConfigs();
  required_process_ = monitor_config_loader.GetRequiredProcesses();
  return true;
}

void DaemonMonitor::CreateBootSwitchService() noexcept {

}

void DaemonMonitor::InitializeChannelWarnings() {
  const std::vector<std::pair<std::string, MonitorErrorCode>> warning_channels =
      {{FLAGS_localization_topic, E_LOW_FREQ_LOCALIZATION},
       {FLAGS_loc_topic, E_LOW_FREQ_LOC},
       {FLAGS_perception_obstacle_topic, E_LOW_FREQ_PERCEPTION},
       {FLAGS_planning_trajectory_topic, E_LOW_FREQ_PLANNING},
       {FLAGS_control_command_topic, E_LOW_FREQ_CONTROL},
       {FLAGS_prediction_topic, E_LOW_FREQ_PREDICTION},
       {FLAGS_traffic_light_detection_topic, E_LOW_FREQ_TRAFFICLIGHT},
       {FLAGS_chassis_topic, E_LOW_FREQ_CHASSIS},
       {FLAGS_tracker_mems_topic, E_LOW_FREQ_MEMS},
       {FLAGS_tracker_radar_topic, E_LOW_FREQ_RADAR},
       {FLAGS_tracker_rs_topic, E_LOW_FREQ_RS},
       {FLAGS_tracker_ultrasonic_topic, E_LOW_FREQ_ULTRASONIC},
       {FLAGS_tracker_camera_topic, E_LOW_FREQ_CAMERA},
       {FLAGS_cubtek_radar_eol_topic, E_LOW_FREQ_RADAR_CUBTK_EOL},
       {FLAGS_mcloud_info_topic, E_LOW_FREQ_MCLOUD},
       {FLAGS_front_12mm_status_topic, E_LOW_FREQ_FRONT_12MM_STATUS},
       {FLAGS_front_3mm_status_topic, E_LOW_FREQ_FRONT_3MM_STATUS},
       {FLAGS_back_3mm_status_topic, E_LOW_FREQ_BACK_3MM_STATUS},
       {FLAGS_mems_status_topic, E_LOW_FREQ_MEMS_STATUS},
       {FLAGS_udas_ultrasonic_eol_topic, E_LOW_FREQ_UDAS_ULTRASONIC_EOL},
       {FLAGS_imu_raw_topic, E_LOW_FREQ_IMU_RAW}};

  for (const auto& channel : warning_channels) {
    channel_to_warning_[channel.first] = channel.second;
  }
}

void DaemonMonitor::InitializeChannelErrors() {
  const std::vector<std::pair<std::string, MonitorErrorCode>> error_channels = {
      {FLAGS_localization_topic, E_NO_FREQ_LOCALIZATION},
      {FLAGS_loc_topic, E_NO_FREQ_LOC},
      {FLAGS_perception_obstacle_topic, E_NO_FREQ_PERCEPTION},
      {FLAGS_planning_trajectory_topic, E_NO_FREQ_PLANNING},
      {FLAGS_control_command_topic, E_NO_FREQ_CONTROL},
      {FLAGS_prediction_topic, E_NO_FREQ_PREDICTION},
      {FLAGS_traffic_light_detection_topic, E_NO_FREQ_TRAFFICLIGHT},
      {FLAGS_chassis_topic, E_NO_FREQ_CHASSIS},
      {FLAGS_tracker_mems_topic, E_NO_FREQ_MEMS},
      {FLAGS_tracker_radar_topic, E_NO_FREQ_RADAR},
      {FLAGS_tracker_rs_topic, E_NO_FREQ_RS},
      {FLAGS_tracker_ultrasonic_topic, E_NO_FREQ_ULTRASONIC},
      {FLAGS_tracker_camera_topic, E_NO_FREQ_CAMERA},
      {FLAGS_cubtek_radar_eol_topic, E_NO_FREQ_RADAR_CUBTK_EOL},
      {FLAGS_mcloud_info_topic, E_NO_FREQ_MCLOUD},
      {FLAGS_front_12mm_status_topic, E_NO_FREQ_FRONT_12MM_STATUS},
      {FLAGS_front_3mm_status_topic, E_NO_FREQ_FRONT_3MM_STATUS},
      {FLAGS_back_3mm_status_topic, E_NO_FREQ_BACK_3MM_STATUS},
      {FLAGS_mems_status_topic, E_NO_FREQ_MEMS_STATUS},
      {FLAGS_udas_ultrasonic_eol_topic, E_NO_FREQ_UDAS_ULTRASONIC_EOL},
      {FLAGS_imu_raw_topic, E_NO_FREQ_IMU_RAW}};

  for (const auto& channel : error_channels) {
    channel_to_error_[channel.first] = channel.second;
  }
}

void DaemonMonitor::InitializeFrameTime() {
  const std::vector<std::string> channels = {
      FLAGS_localization_topic,
      FLAGS_loc_topic,
      FLAGS_perception_obstacle_topic,
      FLAGS_planning_trajectory_topic,
      FLAGS_control_command_topic,
      FLAGS_prediction_topic,
      FLAGS_traffic_light_detection_topic,
      FLAGS_chassis_topic,
      FLAGS_tracker_mems_topic,
      FLAGS_tracker_radar_topic,
      FLAGS_tracker_rs_topic,
      FLAGS_tracker_ultrasonic_topic,
      FLAGS_tracker_camera_topic,
      FLAGS_cubtek_radar_eol_topic,
      FLAGS_mcloud_info_topic,
      FLAGS_front_12mm_status_topic,
      FLAGS_front_3mm_status_topic,
      FLAGS_back_3mm_status_topic,
      FLAGS_mems_status_topic,
      FLAGS_udas_ultrasonic_eol_topic,
      FLAGS_imu_raw_topic};

  // auto initialize_time_for_channels = [this](
  //                                      const auto& channels,
  //                                      const auto& time) {
  //   for (const auto& channel : channels) {
  //     frame_time_last_[channel] = time;
  //     frame_time_current_[channel] = time;
  //   }
  // };

  // initialize_time_for_channels(channels, century::cyber::Time::MonoTime());
}

void DaemonMonitor::InitializeChannelInfo() {
  InitializeChannelWarnings();
  InitializeChannelErrors();
  InitializeFrameTime();
}

void DaemonMonitor::InitializeMonitorData() {
  auto instance = DaemonMonitorData::Instance();
  instance->error_code_infos_ = error_code_infos_;
  instance->monitor_configs_ = monitor_configs_;
  instance->channel_to_warning_ = channel_to_warning_;
  instance->channel_to_error_ = channel_to_error_;
  // instance->channel_to_switch_flag_ = channel_to_switch_flag_;

  error_info_queue_ = std::make_shared<ErrorInfoQueue>();
  instance->error_info_queue_ = error_info_queue_;
}

DaemonMonitor::DaemonMonitor()
    : RecurrentRunner(FLAGS_daemon_monitor_name,
                      FLAGS_daemon_monitor_interval) {
  bool load_xml_success = LoadMonitorConfig();
  AINFO_IF(!load_xml_success) << "DaemonMonitor load xml failed";
  auto load_json_success = LoadErrorCodeInfos();
  AINFO_IF(!load_json_success) << "DaemonMonitor load json failed";

  InitializeChannelInfo();
  InitializeTopicMonitorConfig();
  InitializeMonitorData();

  std::string node_name = "DaemonMonitorNode";
#ifdef __aarch64__
  node_name += "aarch64";
#else
  node_name += "x86";
#endif
  daemon_monitor_node_ = century::cyber::CreateNode(node_name);
  daemon_reader_manager_ =
      std::make_shared<DaemonReaderManager>(daemon_monitor_node_);

  CreateCheckNetworkTask();

  // CreateChannelMonitorReader();
  daemon_reader_manager_->CreateReader();

  system_monitor_ = std::make_unique<monitor::SystemMonitor>();

  auto channel_name = FLAGS_system_monitor_x86_topic;
  if (DOMAIN_TYPE == common::DomainType::AARCH64) {
    channel_name = FLAGS_system_monitor_aarch_topic;
  }
  AINFO << "monitor channel name:" << channel_name;
  monitor_writer_ =
      daemon_monitor_node_->CreateWriter<century::monitor::MonitoredData>(
          channel_name);
  if (DOMAIN_TYPE == common::DomainType::X86) {
    monitor_reader_ =
        daemon_monitor_node_->CreateReader<century::monitor::MonitoredData>(
            FLAGS_system_monitor_aarch_topic);
  }
  monitored_data_ = std::make_shared<century::monitor::MonitoredData>();
}

inline bool DaemonMonitor::IsDomainMatching(
    const std::string& channel_name) const {
  const auto& config = monitor_configs_.at(channel_name);
  return (config.domain == DOMAIN_TYPE) || config.cross_domain;
}

void DaemonMonitor::InitializeTopicMonitorConfig() {
  auto init_monitor_config = [this](const auto& topic_monitor) {
    if (IsDomainMatching(topic_monitor.topic)) {
      DaemonMonitorData::Instance()->channel_freqs_.emplace(
          topic_monitor.topic, ChannelFreq<double, int>());
    }
    monitor_configs_[topic_monitor.topic].module = topic_monitor.module;
    DaemonMonitorData::Instance()->channel_to_switch_flag_.emplace(
        topic_monitor.topic, topic_monitor.enable);
  };

  for (const auto& topic_monitor_switch : kTopicMonitorSwitchList) {
    init_monitor_config(topic_monitor_switch);
  }
}

void DaemonMonitor::CreateCheckNetworkTask() {
  auto check_network_handler = [this]() {
    auto error_code = CheckNetwork();
    if (E_OK == error_code) {
      return;
    }
    ErrorMessage info;
    info.module = module_type::E_SYSTEM;
    SetErrorMessage(&info, error_code_infos_[error_code]);
    info.level = E_WARNING;

#ifdef __aarch64__
    info.domain = common::DomainType::AARCH64;
#else
    info.domain = common::DomainType::X86;
#endif

    error_info_queue_->Enqueue(info);
  };
  time_task_ = std::make_shared<cyber::Timer>(FLAGS_check_network_freq,
                                              check_network_handler, false);
  time_task_->Start();
}

void DaemonMonitor::BootSwitch(int32_t type) {
  AINFO << "switch type : " << type;
  if (switch_config_map_.find(type) == switch_config_map_.end()) {
    return;
  }

#ifdef __x86_64__
  system("bash /home/century/x86_start.sh");
#else
  system("bash /home/nvidia/arm_start.sh");
#endif

  const auto& configs = *switch_config_map_[type];

  for (const auto& config_command : configs) {
    std::string name = config_command.name();
    KillProcessByName(name);
  }
}

void DaemonMonitor::AdRecovery(int32_t type) {
  double time_stamp = cyber::Time().Now().ToSecond();
  AINFO << "ad recovery begin time : " << time_stamp;
  system("bash /century/scripts/ad_recovery.sh &");
}

void DaemonMonitor::CheckModuleChannels() {
  // This segment will be triggered per second, (1/s)
  // double timestamp = century::cyber::Clock::NowInSeconds();
  // ErrorMessage info;
  // info.duration_second = 0;
  // info.timestamp = timestamp;
  // for (auto& channel_freq : DaemonMonitorData::Instance()->channel_freqs_) {
  //   auto freq = channel_freq.second.load();
  //   auto channel = channel_freq.first;
  //   info.level = E_EMERGENCY_STOP;
  //   info.domain = monitor_configs_[channel].domain;
  //   info.module = monitor_configs_[channel].module;
  //   if (channel == FLAGS_imu_raw_topic) {
  //     info.level = E_WARNING;
  //   }
  //   if (std::abs(freq.GetKey()) < kMinimumTime) {
  //     SetErrorMessage(&info,
  //                     error_code_infos_[channel_to_error_[channel_freq.first]]);
  //     if (channel_to_switch_flag_[channel]) {
  //       error_info_queue_->Enqueue(info);
  //     }
  //     AERROR << channel_freq.first << " not enabled";
  //     continue;
  //   }
  //   GetSuitableDiffTimeThreshold(channel);
  //   double msg_frame_time = frame_time_current_[channel].ToSecond();
  //   auto time_diff = std::abs(timestamp - msg_frame_time);
  //   if (time_diff < suitable_diff_time_threshold_) {
  //     AINFO << channel_freq.first
  //           << " : normal current time = " << std::to_string(timestamp)
  //           << ", channel time = " << std::to_string(freq.GetKey())
  //           << ", msg_frame time = " << std::to_string(msg_frame_time)
  //           << ", channel cnt = " << std::to_string(freq.GetValue())
  //           << ", time diff = " << time_diff << ", suitable_diff ="
  //           << std::to_string(suitable_diff_time_threshold_);
  //   } else {
  //     SetErrorMessage(&info,
  //                     error_code_infos_[channel_to_error_[channel_freq.first]]);
  //     if (channel_to_switch_flag_[channel]) {
  //       error_info_queue_->Enqueue(info);
  //     }
  //     AERROR << channel_freq.first
  //            << " : abnormal current time = " << std::to_string(timestamp)
  //            << ", channel time = " << std::to_string(freq.GetKey())
  //            << ", msg_frame time = " << std::to_string(msg_frame_time)
  //            << ", channel cnt = " << std::to_string(freq.GetValue())
  //            << ", time diff = " << time_diff << ", suitable_diff ="
  //            << std::to_string(suitable_diff_time_threshold_);
  //   }
  // }
}

void DaemonMonitor::GetSystemMonitorData() {
  auto* system_monitor_data = monitored_data_->mutable_system_monitor_data();
  system_monitor_data->CopyFrom(*system_monitor_data_);
}

// void DaemonMonitor::GetRunningStatus() {
//   auto* running_status = monitored_data_->mutable_running_status();
//   running_status->set_auto_driving_status(auto_driving_status_.load());

//   if (nullptr != planning_reader_) {
//     planning_reader_->Observe();
//     auto trajectory = planning_reader_->GetLatestObserved();
//     if (nullptr != trajectory && trajectory->has_trajectory_scenario()) {
//       running_status->set_trajectory_scenario(
//           trajectory->trajectory_scenario());
//     }
//   }
// }

void DaemonMonitor::GetSystemStatusFaultData() {
  auto& system_data = system_monitor_data_->system_data();
  double cpu_usage = system_data.cpu_usage();
  double disk_usage = system_data.disk_usage();
  double mem_usage = system_data.mem_usage();

  if (cpu_usage > FLAGS_cputhreshold) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_OUT_OF_CPU]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }
  if (disk_usage > FLAGS_diskthreshold) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_OUT_OF_DISK]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }
  if (mem_usage > FLAGS_memthreshold) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_OUT_OF_MEM]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }
  if (disk_usage > FLAGS_diskwarningthreshold &&
      disk_usage < FLAGS_diskthreshold) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_LOW_DISK]);
    fault_info->set_level(E_INFO);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }
}

void DaemonMonitor::CheckPredictionStatus() {}

void DaemonMonitor::DaemonMonitorStatus() {
  // 1. check some process  running status
  if (DOMAIN_TYPE == common::DomainType::AARCH64) {
    CheckPredictionStatus();
    // CheckCenterPointStatus();
    // CheckRsSdkHanging();
  }
  // 2. added ErrorMessage :check channel enable status
  // CheckModuleChannels();
  // 2.1 added ErrorMessage : some runtime checks
  daemon_reader_manager_->CheckTransmissionLatency();

  // 3.construct fault_data
  CollectFaultData();
  GetSystemStatusFaultData();
  // 3.1 do some special checks and add fault_data
  // if (DOMAIN_TYPE == common::DomainType::X86) {
  //   // CheckMapConfigInfo();
  //   CheckDiskWritableStatus();
  // }

  CheckDiskWritableStatus();
  // 4.construct system_monitor_data
  GetSystemMonitorData();

  // 5.construct running_status
  // if (DOMAIN_TYPE == common::DomainType::X86) {
  //   GetRunningStatus();
  // }
  // 6.pub monited_data
  PublishMonitoredData();
}

void DaemonMonitor::PublishMonitoredData() {
  if (FLAGS_turn_on_monitor) {
    common::util::FillHeader(daemon_monitor_node_->Name(),
                             monitored_data_.get());
    monitored_data_->set_domain_type(DOMAIN_TYPE);
    monitor_writer_->Write(monitored_data_);
  }
}

void DaemonMonitor::CollectFaultData() {
  monitored_data_->clear_fault_data();
  const auto queue_size = error_info_queue_->Size();
  for (auto i = 0u; i < queue_size; ++i) {
    ErrorMessage info;
    error_info_queue_->Dequeue(&info);
    auto fault_info = monitored_data_->add_fault_data();
    fault_info->set_code(info.error);
    fault_info->set_code_msg(info.msg);
    fault_info->set_level(info.level);
    fault_info->set_domain_type(info.domain);
    fault_info->set_module_type(info.module);
  }
}

void DaemonMonitor::RunOnce(const double current_time) {
  cur_time_second_.store(cyber::Time().Now().ToSecond());
  DaemonMonitorData::Instance()->cur_time_second_.store(
      cur_time_second_.load());
  // monitor cpu、mem、disk
  system_monitor_->RunOnce(current_time);
  system_monitor_data_ = system_monitor_->GetSystemMonitorData();

  DaemonMonitorStatus();
}

int DaemonMonitor::GetModifiedFilesWithin(const std::string& path, int times,
                                          const std::string& exclude) const {
  int count = 0;

  boost::filesystem::path dir_path(path);
  if (!boost::filesystem::is_directory(dir_path)) {
    AERROR << dir_path << " is not a directory";
    return count;
  }

  auto now = floor(cyber::Clock::NowInSeconds());
  for (const auto& path : boost::filesystem::directory_iterator(dir_path)) {
    if (!boost::filesystem::is_regular_file(path.status())) {
      continue;
    }
    if (!exclude.empty() && path.path().string() == exclude) {
      continue;
    }
    auto last_modified = boost::filesystem::last_write_time(path.path());
    auto diff = now - last_modified;
    if (diff > times) {
      continue;
    }
    ++count;
  }
  return count;
}

bool DaemonMonitor::GetCyberRecorderWriteStatus() {
  std::string process_name = R"("cyber_recorder record")";
  std::string cmd = "ps aux | grep " + process_name + " | grep -v grep";
  int result = system(cmd.c_str());
  if (0 != result) {
    AERROR << "cyber_recorder not run";
    return false;
  }

  int count = GetModifiedFilesWithin(kBagPath, kDiskCheckInterval);
  if (0 == count) {
    AERROR << "cyber_recorder write error";
    return false;
  }
  return true;
}

bool DaemonMonitor::GetLogWriteStatus() {
  int count =
      GetModifiedFilesWithin(kLogPath, kDiskCheckInterval, kWriteTestFilePath);
  if (0 == count) {
    AERROR << "log write error";
    return false;
  }
  return true;
}

bool DaemonMonitor::GetDiskWriteStatus() {
  std::ofstream outputFile(kWriteTestFilePath);
  if (outputFile.fail()) {
    AERROR << "Error opening file";
    return false;
  }
  outputFile << "test wirte disk!";
  if (outputFile.fail()) {
    AERROR << "Error writing to file";
    return false;
  }
  outputFile.close();
  return true;
}

void DaemonMonitor::CheckDiskWritableStatus() {
  auto now = floor(cyber::Clock::NowInSeconds());
  if (last_check_disk_time_ < kMinimumTime) {
    last_check_disk_time_ = now;
  }

  if (now - last_check_disk_time_ < kDoubleDiskCheckFreq) {
    return;
  }

  last_check_disk_time_ = now;

  bool disk_status = GetDiskWriteStatus();
  bool cyber_recorder_status = GetCyberRecorderWriteStatus();
  bool log_status = GetLogWriteStatus();

  if (!cyber_recorder_status) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info,
                    error_code_infos_[E_CYBER_RECORDER_WRITE_STATUS]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }

  if (!log_status) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_LOG_WRITE_STATUS]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }

  if (!disk_status) {
    auto fault_info = monitored_data_->add_fault_data();
    SetErrorMessage(fault_info, error_code_infos_[E_DISK_WRITE_STATUS]);
    fault_info->set_level(E_WARNING);
    fault_info->set_domain_type(DOMAIN_TYPE);
    fault_info->set_module_type(module_type::E_SYSTEM);
  }
}

bool DaemonMonitor::CheckLocalNetwork(const std::string& local_ip) const {
  std::stringstream ss;
  ss << "ifconfig | grep 'inet ' | awk '{print $2}' | grep '" << local_ip
     << "'";

  int ret = system(ss.str().c_str());
  if (0 != ret) {
    AERROR << "local network down";
    return false;
  }
  return true;
}

bool DaemonMonitor::HostReachable(const std::string& address) const {
  std::stringstream ss;
  ss << "ping -c 1 -W 1 " << address << " 2>&1";

  int ret = system(ss.str().c_str());
  if (0 != ret) {
    AERROR << "address " << address << " not reach";
    return false;
  }

  return true;
}

MonitorErrorCode DaemonMonitor::CheckNetwork() {
  if (!CheckLocalNetwork(FLAGS_local_address)) {
    AERROR << "local network error";
    return E_NETDOWN;
  }

  if (!HostReachable(FLAGS_outside_address)) {
    AERROR << "Error reaching external network";
    return E_NETNOTREACH;
  }

  if (!HostReachable(FLAGS_tbox_address)) {
    AERROR << "Error reaching box";
    return E_NETNOTREACH;
  }

  if (!HostReachable(FLAGS_other_machine_address)) {
    AERROR << "Error reaching other machine";
    return E_NETNOTREACH;
  }
  return E_OK;
}

// void DaemonMonitor::GetSuitableDiffTimeThreshold(const std::string& channel)
// {
//   if (planning_in_teb_ && channel == FLAGS_planning_trajectory_topic) {
//     double freq = FLAGS_teb_planning_required_frq * FLAGS_lowfreqthreshold;
//     suitable_diff_time_threshold_ = std::abs(freq) < kMinFrequencyEpsilon
//                                         ? (1 /
//                                         FLAGS_teb_planning_required_frq) : (1
//                                         / freq);
//     return;
//   }
//   if (monitor_configs_[channel].threshold_freq > kMinFrequencyEpsilon) {
//     auto reciprocal_data = 1 / monitor_configs_[channel].threshold_freq;
//     auto twice_reciprocal_data = reciprocal_data * 2;
//     if (twice_reciprocal_data > kDiffTimeThreshold) {
//       suitable_diff_time_threshold_ =
//           static_cast<uint16_t>(twice_reciprocal_data);
//       return;
//     }
//   }
//   suitable_diff_time_threshold_ = kDiffTimeThreshold;
// }

}  // namespace monitor
}  // namespace century
