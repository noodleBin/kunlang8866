/******************************************************************************
 * Copyright 2024 The century Authors. All Rights Reserved.
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

#include "cyber/base/thread_safe_queue.h"
#include "modules/common/configs/common_def.h"

namespace century {
namespace monitor {
using ::century::common::module_type;

const int kAlignmentSize = 16;

enum ErrorLevel {
  E_INFO = 1,
  E_WARNING,
  E_PULL_OVER,
  E_EMERGENCY_STOP,
};

enum MonitorErrorCode : int32_t {
  E_OK = 0,                                 ///< success
  E_NOT_INIT,                               ///< not initialize
  E_OUT_OF_MEM,                             ///< insufficient memory resources
  E_OUT_OF_CPU,                             ///< insufficient CPU resources
  E_LOW_DISK,                               ///< low disk resources
  E_OUT_OF_DISK,                            ///< insufficient memory resources
  E_DIFF_OF_MAP,                            ///< use map diff
  E_LOW_FREQ_LOCALIZATION,                  ///< low localization freq
  E_NO_FREQ_LOCALIZATION,                   ///< no localization freq
  E_LOCALIZATION_STATUS_ABNORMAL,           ///< localization status error
  E_LOCALIZATION_DELAY,                     ///< localization delay status
  E_LOW_FREQ_LOC,                  ///< low localization freq
  E_NO_FREQ_LOC,                   ///< no localization freq
  E_LOC_STATUS_ABNORMAL,           ///< localization status error
  E_LOC_DELAY,                     ///< localization delay status
  E_LOC_DIFF_MAX,
  E_LOC_INIT_IMU_WAITING,                   ///< IMU Waiting...
  E_LOC_INIT_GPS_WAITING,                   ///< GPS Waiting...
  E_LOC_INIT_ODOMETRY_WAITING,              ///< ODOMETRY Waiting...
  E_LOC_INIT_LIDAR_WAITING,                 ///< LIDAR Waiting...
  E_LOC_INIT_KEY_NOT_FOUND,                 ///< KEY_NOT_FOUND
  E_LOW_FREQ_PERCEPTION,                    ///< low perception freq
  E_NO_FREQ_PERCEPTION,                     ///< no perception freq
  E_LOW_FREQ_PLANNING,                      ///< low planning freq
  E_NO_FREQ_PLANNING,                       ///< no planning freq
  E_LOW_FREQ_CONTROL,                       ///< low control freq
  E_NO_FREQ_CONTROL,                        ///< no control freq
  E_CONTROL_CALIBRATION_ANOMALY,            ///< vehicle_calibration_anomaly
  E_CONTROL_BRAKE_ABNORMAL_INPUT,           ///< ABNORMAL_INPUT
  E_CONTROL_BRAKE_DEVIATION_TRAJECTORY,     ///< DEVIATION_TRAJECTORY
  E_CONTROL_BRAKE_FALLBACK_TRAJECTORY,      ///< FALLBACK_TRAJECTORY
  E_CONTROL_BRAKE_LOCATION_POOR,            ///< LOCATION_POOR
  E_LOW_FREQ_PREDICTION,                    ///< low prediction freq
  E_NO_FREQ_PREDICTION,                     ///< no prediction freq
  E_LOW_FREQ_TRAFFICLIGHT,                  ///< low trafficlight freq
  E_NO_FREQ_TRAFFICLIGHT,                   ///< no trafficlight freq
  E_LOW_FREQ_CHASSIS,                       ///< low chassis freq
  E_NO_FREQ_CHASSIS,                        ///< no chassis freq
  E_LOW_FREQ_MEMS,                          ///< low mems freq
  E_NO_FREQ_MEMS,                           ///< no mems freq
  E_LOW_FREQ_RADAR,                         ///< low radar freq
  E_NO_FREQ_RADAR,                          ///< no radar freq
  E_LOW_FREQ_RS,                            ///< low rs freq
  E_NO_FREQ_RS,                             ///< no rs freq
  E_RS_SDK_STATUS_ZERO,                     ///< rs_sdk_status==0 more than 3
  E_LOW_FREQ_ULTRASONIC,                    ///< low ultrasonic freq
  E_NO_FREQ_ULTRASONIC,                     ///< no ultrasonic freq
  E_LOW_FREQ_CAMERA,                        ///< low camera freq
  E_NO_FREQ_CAMERA,                         ///< no camera freq
  E_LOW_FREQ_RADAR_CUBTK_EOL,               ///< low cubtek_radar eol freq
  E_NO_FREQ_RADAR_CUBTK_EOL,                ///< no cubtek_radar eol freq
  E_RADAR_CUBTK_EOL_LEFT_STATUS,            ///< cubtek_radar left status
  E_RADAR_CUBTK_EOL_RIGHT_STATUS,           ///< cubtek_radar right status
  E_RADAR_CUBTK_EOL_REAR_STATUS,            ///< cubtek_radar rear status
  E_LOW_FREQ_MCLOUD,                        ///< low mcloud freq
  E_NO_FREQ_MCLOUD,                         ///< no mcloud freq
  E_MCLOUD_STATUS_ERROR,                    ///< mcloud status error
  E_MCLOUD_BOXSTATUS_OPEN,                  ///< mcloud boxstatus opened
  E_NO_FREQ_MONITOR_X86,                    ///< no moitor_x86 freq
  E_NO_FREQ_MONITOR_AARCH,                  ///< no moitor_aarch freq
  E_LOW_FREQ_FRONT_12MM_STATUS,             ///< low front 12mm status freq
  E_NO_FREQ_FRONT_12MM_STATUS,              ///< no front 12mm status freq
  E_LOW_FREQ_FRONT_3MM_STATUS,              ///< low front 3mm status freq
  E_NO_FREQ_FRONT_3MM_STATUS,               ///< no front 3mm status freq
  E_LOW_FREQ_BACK_3MM_STATUS,               ///< low back 3mm status freq
  E_NO_FREQ_BACK_3MM_STATUS,                ///< no back 3mm status freq
  E_DISK_WRITE_STATUS,                      ///< disk write error
  E_LOG_WRITE_STATUS,                       ///< log write error
  E_CYBER_RECORDER_WRITE_STATUS,            ///< cyber_recorder write error
  E_LOW_FREQ_MEMS_STATUS,                   ///< low mems status freq
  E_NO_FREQ_MEMS_STATUS,                    ///< no mems status freq
  E_MEMS_FRONT_LEFT_RIGHT_STATUS_ABNORMAL,  ///< mems front, left, right status
                                            ///< abnormal
  E_MEMS_LEFT_RIGHT_STATUS_ABNORMAL,   ///< mems left, right status abnormal
  E_MEMS_FRONT_LEFT_STATUS_ABNORMAL,   ///< mems front, left status abnormal
  E_MEMS_LEFT_STATUS_ABNORMAL,         ///< mems left status abnormal
  E_MEMS_FRONT_RIGHT_STATUS_ABNORMAL,  ///< mems front, right status abnormal
  E_MEMS_RIGHT_STATUS_ABNORMAL,        ///< mems right status abnormal
  E_MEMS_FRONT_STATUS_ABNORMAL,        ///< mems front status abnormal

  E_LOW_FREQ_UDAS_ULTRASONIC_EOL,         ///< low udas ultrasonic eol freq
  E_NO_FREQ_UDAS_ULTRASONIC_EOL,          ///< no udas ultrasonic eol freq
  E_UDAS_ULTRASONIC_RIGHT_STATUS,         ///< udas ultrasonic right status
  E_UDAS_ULTRASONIC_MIDDLE_RIGHT_STATUS,  ///< udas ultrasonic middle right
                                          ///< status
  E_UDAS_ULTRASONIC_MIDDLE_LEFT_STATUS,   ///< udas ultrasonic middle left
                                          ///< status
  E_UDAS_ULTRASONIC_LEFT_STATUS,          ///< udas ultrasonic left status

  E_LOW_FREQ_IMU_RAW,  ///< low imu raw
  E_NO_FREQ_IMU_RAW,   ///< no imu raw

  E_EXCEEDING_TIME_DIFF,  /// exceeding the maximum time diff
  E_NETNOTREACH,          ///< net not reachable
  E_NETDOWN,              ///< no network hardware

  E_END,  ///< place holder
};

struct alignas(kAlignmentSize) ErrorMessage {
  void set_code(const std::string& code) noexcept { error = code; }
  void set_code_msg(const std::string& message) noexcept { msg = message; }

  module_type module{module_type::E_SYSTEM};
  ErrorLevel level{ErrorLevel::E_INFO};
  common::DomainType domain{common::DomainType::X86};
  double duration_second{0.0};
  uint64_t timestamp{0ull};
  std::string error;
  std::string msg;
};

struct alignas(kAlignmentSize) MonitorConfig {
  bool cross_domain{false};
  common::DomainType domain{common::DomainType::X86};
  module_type module{module_type::E_SYSTEM};
  float threshold_freq{0.0f};
  float expected_freq{0.0f};
  std::string process_name;
  std::string channel;
};

struct alignas(kAlignmentSize) PredictionStatusCheck {
  int kill_flag{0};
  double kill_prediction_timestamp{0.0};
};

template <typename T, typename R>
class ChannelFreq {
 public:
  ChannelFreq() noexcept : key(T(0)), value(R(0)) {}
  ChannelFreq(T k, R v) noexcept : key(k), value(v) {}
  T GetKey() const { return key; }
  R GetValue() const { return value; }
  void SetKey(T k) { key = k; }
  void SetValue(R v) { value = v; }

 private:
  T key;
  R value;
};

struct ErrorInfo {
  int value;
  std::string name;
  std::string code;
  std::string message;
};

using ErrorInfoQueue = cyber::base::ThreadSafeQueue<ErrorMessage>;
using ErrorInfoQueuePtr = std::shared_ptr<ErrorInfoQueue>;

struct ReaderParam {
  std::string topic_name;
  ErrorInfoQueuePtr error_info_queue;
  std::shared_ptr<ChannelFreq<double, int>> channel_freqs;
  module_type module_type_name;
  ErrorLevel log_level_ultra_low_frequency;
  ErrorLevel log_level_low_frequency;

  bool channel_to_switch_flag{false};
};

struct DaemonMonitorData {
  std::atomic<double> cur_time_second_{0.0};
  std::unordered_map<MonitorErrorCode, ErrorInfo> error_code_infos_;
  std::unordered_map<std::string, MonitorConfig> monitor_configs_;
  std::unordered_map<std::string, MonitorErrorCode> channel_to_warning_;
  std::unordered_map<std::string, MonitorErrorCode> channel_to_error_;
  std::unordered_map<std::string, bool> channel_to_switch_flag_;

  std::unordered_map<std::string, std::atomic<ChannelFreq<double, int>>>
      channel_freqs_;
  ErrorInfoQueuePtr error_info_queue_;

  DECLARE_SINGLETON(DaemonMonitorData)
};

DaemonMonitorData::DaemonMonitorData() {}

}  // namespace monitor
}  // namespace century