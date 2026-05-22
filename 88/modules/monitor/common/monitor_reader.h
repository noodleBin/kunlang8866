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

#include <unordered_map>

#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "cyber/timedelay/timedelay.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/monitor/common/monitor_gflags.h"
#include "modules/monitor/common/monitor_typedef.h"

namespace century {
namespace monitor {

namespace {
constexpr double kSecondsToNanoseconds = 1e9;
constexpr double kMinimumTime = 1.0e-6;
constexpr uint8_t kMaxCheckSkips = 10;
constexpr double kPoorAccuracyDuration = 30.0;
constexpr double kDoubleEpsilon = 1.0e-8;
constexpr double kMsgDelayThreshold = 1.0;
constexpr double kTransmissionLatencyDuration = 5.0;
constexpr double kMinFrequencyEpsilon = 1.0e-6;
constexpr uint16_t kDiffTimeThreshold = 2;

}  // namespace

template <typename MsgType>
using ReaderPtr = std::shared_ptr<cyber::Reader<MsgType>>;

class BaseReader {
 public:
  virtual ~BaseReader() = default;
  virtual void CreateReader() noexcept = 0;
  virtual void CheckModuleChannels(const double timestamp) noexcept = 0;
};

template <typename MsgType>
class CyberReader : public BaseReader {
 public:
  using MsgTypePtr = std::shared_ptr<MsgType>;
  using ReceivedCallback = std::function<void()>;

  CyberReader(const std::string& channel_name,
              const std::shared_ptr<cyber::Node>& node)
      : channel_name_(channel_name), node_(node) {
    domain_ =
        DaemonMonitorData::Instance()->monitor_configs_[channel_name].domain;
    channel_to_switch_flag_ =
        DaemonMonitorData::Instance()->channel_to_switch_flag_[channel_name];
    error_info_queue_ = DaemonMonitorData::Instance()->error_info_queue_;
    frame_time_current_ = century::cyber::Time::MonoTime();
    frame_time_last_ = century::cyber::Time::MonoTime();
  }
  virtual ~CyberReader() = default;

  virtual void CreateReader() noexcept override {
    if (!channel_name_.empty()) {
      reader_ = node_->CreateReader<MsgType>(
          channel_name_,
          std::bind(&CyberReader::OnReceivedMsg, this, std::placeholders::_1));
    }
  }

  void CheckModuleChannels(const double timestamp) noexcept override {
    // This segment will be triggered per second, (1/s)
    // auto timestamp = century::cyber::Clock::NowInSeconds();
    ErrorMessage info;
    info.duration_second = 0;
    info.timestamp = timestamp;

    auto channel_freq =
        DaemonMonitorData::Instance()->channel_freqs_[channel_name_].load();
    // auto channel = channel_freq.GetKey();
    info.level = E_EMERGENCY_STOP;
    info.domain =
        DaemonMonitorData::Instance()->monitor_configs_[channel_name_].domain;
    info.module =
        DaemonMonitorData::Instance()->monitor_configs_[channel_name_].module;
    if (FLAGS_imu_raw_topic == channel_name_) {
      info.level = E_WARNING;
    }

    if (std::abs(channel_freq.GetKey()) < kMinimumTime) {
      SetErrorMessage(
          &info,
          DaemonMonitorData::Instance()
              ->error_code_infos_[DaemonMonitorData::Instance()
                                      ->channel_to_error_[channel_name_]]);

      AERROR << "channel_name_: " << channel_name_ << " channel_to_switch_flag_ : " << channel_to_switch_flag_
      << " error msg : " << DaemonMonitorData::Instance()
      ->error_code_infos_[DaemonMonitorData::Instance()
                              ->channel_to_error_[channel_name_]].message;
      if (channel_to_switch_flag_) {
        error_info_queue_->Enqueue(info);
      }
      AERROR << channel_name_ << " not enabled";
      return;
    }

    const auto suitable_diff_time_threshold = GetSuitableDiffTimeThreshold();
    const auto msg_frame_time = frame_time_current_.ToSecond();
    const auto time_diff = std::abs(timestamp - msg_frame_time);

    if (time_diff < suitable_diff_time_threshold) {
      AINFO << channel_name_
            << " : normal current time = " << std::to_string(timestamp)
            << ", channel time = " << std::to_string(channel_freq.GetKey())
            << ", msg_frame time = " << std::to_string(msg_frame_time)
            << ", channel cnt = " << std::to_string(channel_freq.GetValue())
            << ", time diff = " << time_diff << ", suitable_diff ="
            << std::to_string(suitable_diff_time_threshold);
    } else {
      SetErrorMessage(
          &info,
          DaemonMonitorData::Instance()
              ->error_code_infos_[DaemonMonitorData::Instance()
                                      ->channel_to_error_[channel_name_]]);
      if (channel_to_switch_flag_) {
        error_info_queue_->Enqueue(info);
      }
      AERROR << channel_name_
             << " : abnormal current time = " << std::to_string(timestamp)
             << ", channel time = " << std::to_string(channel_freq.GetKey())
             << ", msg_frame time = " << std::to_string(msg_frame_time)
             << ", channel cnt = " << std::to_string(channel_freq.GetValue())
             << ", time diff = " << time_diff << ", suitable_diff ="
             << std::to_string(suitable_diff_time_threshold);
    }
  }

  ReaderPtr<MsgType> GetReader() noexcept {
    return reader_;
  }
 protected:
  template <typename EMType>
  void SetErrorMessage(EMType* error_info, const ErrorInfo& info) {
    error_info->set_code(info.code);
    error_info->set_code_msg(info.message);
  }

  void AddedErrorInfo(const std::string& channel_name,
                      const MonitorErrorCode error_code, const ErrorLevel level,
                      const uint64_t timestamp) noexcept {
    if (!channel_to_switch_flag_) {
      return;
    }

    ErrorMessage info;
    info.module =
        DaemonMonitorData::Instance()->monitor_configs_[channel_name].module;
    info.duration_second = 0;
    info.timestamp = timestamp;
    info.level = level;
    info.domain = domain_;
    SetErrorMessage(
        &info, DaemonMonitorData::Instance()->error_code_infos_[error_code]);
    error_info_queue_->Enqueue(info);
  }

  virtual void MsgFreqCheck(const std::string& channel_name,
                            const module_type& module_type_name,
                            double* threshold_freq,
                            double* expected_freq) noexcept {
    constexpr double kMaxFrequencyEpsilon = 65535.0;

    if (module_type::E_PLANNING == module_type_name) {
      AERROR << "type error, ADCTrajectory is not acceptable channel_name= " << channel_name;
      *threshold_freq = kMaxFrequencyEpsilon;
      *expected_freq = kMaxFrequencyEpsilon;
    } else {
      *expected_freq = DaemonMonitorData::Instance()
                           ->monitor_configs_[channel_name]
                           .threshold_freq;
      *threshold_freq = FLAGS_lowfreqthreshold * (*expected_freq);
    }
    return;
  }

  void CheckMessageFrequency(const MsgTypePtr& msg,
                             const std::string& channel_name,
                             const module_type& module_type_name,
                             const ErrorLevel& log_level_ultra_low_frequency,
                             const ErrorLevel& log_level_low_frequency) {
    double time_stamp = msg->header().timestamp_sec();
    auto freq =
        DaemonMonitorData::Instance()->channel_freqs_[channel_name].load();
    frame_time_current_ = century::cyber::Time(time_stamp);
    auto duration_time =
        (frame_time_current_ - frame_time_last_).ToNanosecond();
    double threshold_freq = 0.0;
    double expected_freq = 0.0;
    MsgFreqCheck(channel_name, module_type_name, &threshold_freq,
                 &expected_freq);

    if (duration_time < kSecondsToNanoseconds) {
      freq.SetValue(freq.GetValue() + 1);
      DaemonMonitorData::Instance()->channel_freqs_[channel_name].store(freq);
    } else {
      duration_time =
          duration_time > kMinimumTime ? duration_time : kMinimumTime;
      double current_freq =
          kSecondsToNanoseconds * freq.GetValue() / duration_time;

      ErrorMessage info;
      info.module = module_type_name;
      info.duration_second = 0;
      info.timestamp = time_stamp;
      info.domain = domain_;
      if (current_freq < threshold_freq) {
        info.level = log_level_ultra_low_frequency;
        auto channel_error_info =
            DaemonMonitorData::Instance()
                ->error_code_infos_[DaemonMonitorData::Instance()
                                        ->channel_to_error_[channel_name]];
        SetErrorMessage(&info, channel_error_info);
        if (FLAGS_treat_low_freq_as_error && channel_to_switch_flag_) {
          error_info_queue_->Enqueue(info);
        }
      } else if (current_freq < expected_freq &&
                 current_freq >= threshold_freq) {
        info.level = log_level_low_frequency;
        auto channel_warning_info =
            DaemonMonitorData::Instance()
                ->error_code_infos_[DaemonMonitorData::Instance()
                                        ->channel_to_warning_[channel_name]];
        SetErrorMessage(&info, channel_warning_info);
        if (channel_to_switch_flag_) {
          error_info_queue_->Enqueue(info);
        }
        AINFO << "channel name:" << channel_name << ", freq: " << current_freq;
      }

      freq.SetKey(floor(time_stamp));
      freq.SetValue(1);
      DaemonMonitorData::Instance()->channel_freqs_[channel_name].store(freq);
      frame_time_last_ = frame_time_current_;
    }
  }

  virtual uint16_t GetSuitableDiffTimeThreshold() noexcept {
    uint16_t suitable_diff_time_threshold = kDiffTimeThreshold;
    const auto freq_threshold = DaemonMonitorData::Instance()
                                    ->monitor_configs_[channel_name_]
                                    .threshold_freq;
    if (freq_threshold > kMinFrequencyEpsilon) {
      auto reciprocal_data = 1 / freq_threshold;
      auto twice_reciprocal_data = reciprocal_data * 2;
      if (twice_reciprocal_data > kDiffTimeThreshold) {
        suitable_diff_time_threshold =
            static_cast<uint16_t>(twice_reciprocal_data);
        return suitable_diff_time_threshold;
      }
    }

    return suitable_diff_time_threshold;
  }

 protected:
  std::string channel_name_;
  std::shared_ptr<cyber::Node> node_;
  ReaderPtr<MsgType> reader_;
  ErrorInfoQueuePtr error_info_queue_;
  common::DomainType domain_;
  bool channel_to_switch_flag_;
  cyber::Time frame_time_current_;
  cyber::Time frame_time_last_;

  virtual void OnReceivedMsg(const std::shared_ptr<MsgType>& msg) noexcept {}
};

}  // namespace monitor
}  // namespace century
