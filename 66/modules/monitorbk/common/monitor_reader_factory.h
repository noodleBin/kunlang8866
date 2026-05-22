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

#include "modules/common/util/util.h"
#include "modules/monitor/common/monitor_reader.h"
#include "modules/monitor/common/monitor_utils.h"

namespace century {
namespace monitor {

// constexpr double kMinimumTime = 1.0e-6;

template <typename ReaderType>
std::shared_ptr<BaseReader> CreateReaderByTopic(
    const std::string& topic, std::shared_ptr<cyber::Node> node) {
  std::shared_ptr<BaseReader> reader =
      std::make_shared<ReaderType>(topic, node);
  reader->CreateReader();
  return reader;
}

template <typename T, typename ReaderType>
std::shared_ptr<BaseReader> CreateReaderByTopic(
    const std::shared_ptr<T> reader_ptr, const std::string& topic,
    std::shared_ptr<cyber::Node> node) {
  std::shared_ptr<BaseReader> reader =
      std::make_shared<ReaderType>(reader_ptr, topic, node);
  reader->CreateReader();
  return reader;
}

class TransmissionInterface {
 public:
  double GetLatency() const { return transmission_latency_start_.load(); }

  void JudgeTransmissionLatency(const double timestamp_sec) {
    double cur_timestamp = cyber::Time::Now().ToSecond();
    double time_diff = std::fabs(timestamp_sec - cur_timestamp);
    if (time_diff < FLAGS_transmission_latency_diff) {
      transmission_latency_start_.store(0.0);  // normal
    } else if (transmission_latency_start_.load() < kMinimumTime) {
      transmission_latency_start_.store(cur_timestamp);
      AWARN << "transmission_latency_start_ time_diff "
            << std::to_string(time_diff);
    }
  }

  void CheckTransmissionLatency(const ErrorInfoQueuePtr& safe_queue,
                                const ErrorInfo& error_info,
                                double cur_system_time) const {
    if (transmission_latency_start_.load() <= kMinimumTime) {
      return;
    }

    if (cur_system_time - transmission_latency_start_.load() >
        kTransmissionLatencyDuration) {
      AWARN << "transmission_latency_start_ "
            << std::to_string(transmission_latency_start_.load());

      if (!FLAGS_transmission_latency_switch) {
        return;
      }
      ErrorMessage info;
      info.module = module_type::E_SYSTEM;
      info.level = E_PULL_OVER;
      info.duration_second = 0;
      info.timestamp = static_cast<uint64_t>(cur_system_time);

#if defined __aarch64__
      info.domain = common::DomainType::AARCH64;
#else
      info.domain = common::DomainType::X86;
#endif

      info.set_code(error_info.code);
      info.set_code_msg(error_info.message);

      safe_queue->Enqueue(info);
    }
  }

 private:
  std::atomic<double> transmission_latency_start_ = {0.0};
};

class ChassisReader : public CyberReader<canbus::Chassis> {
 public:
  using MsgType = canbus::Chassis;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;

  ~ChassisReader() = default;

  void CheckTransmissionLatency() noexcept {
    transmission_.CheckTransmissionLatency(
        error_info_queue_,
        DaemonMonitorData::Instance()->error_code_infos_[E_EXCEEDING_TIME_DIFF],
        DaemonMonitorData::Instance()->cur_time_second_.load());
  }

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    static uint8_t times = 0;
    if (kMaxCheckSkips == ++times) {
      transmission_.JudgeTransmissionLatency(msg->header().timestamp_sec());
      times = 0;
    }

    CheckMessageFrequency(msg, channel_name_, module_type::E_CHASSIS,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }

 private:
  TransmissionInterface transmission_;
};

/**
 * @brief Control command reader
 *
 */
class ControlCommandReader : public CyberReader<control::ControlCommand> {
 public:
  using MsgType = control::ControlCommand;
  using MsgPtr = std::shared_ptr<MsgType>;

  using CyberReader<MsgType>::CyberReader;

 private:
  void HandleControlCommand(const MsgPtr& msg) noexcept {
    // No need to check every time.
    // static uint32_t skip_time = 0;
    // if (++skip_time < kMaxCheckSkips) {
    //   return;
    // }
    // skip_time = 0;  // start check
    // const auto timestamp = msg->header().timestamp_sec();
    // if (msg->has_vehicle_calibration_anomaly() &&
    //     msg->vehicle_calibration_anomaly()) {
    //   AddedErrorInfo(channel_name_, E_CONTROL_CALIBRATION_ANOMALY,
    //   E_PULL_OVER,
    //                  timestamp);
    // }

    // if (msg->has_control_brake_scenario() &&
    //     control::CONTROL_BRAKE_NORMAL != msg->control_brake_scenario()) {
    //   MonitorErrorCode error_code = E_OK;
    //   switch (msg->control_brake_scenario()) {
    //     case control::CONTROL_BRAKE_ABNORMAL_INPUT:
    //       error_code = E_CONTROL_BRAKE_ABNORMAL_INPUT;
    //       break;
    //     case control::CONTROL_BRAKE_DEVIATION_TRAJECTORY:
    //       error_code = E_CONTROL_BRAKE_DEVIATION_TRAJECTORY;
    //       break;
    //     case control::CONTROL_BRAKE_FALLBACK_TRAJECTORY:
    //       error_code = E_CONTROL_BRAKE_FALLBACK_TRAJECTORY;
    //       break;
    //     case control::CONTROL_BRAKE_LOCATION_POOR:
    //       error_code = E_CONTROL_BRAKE_LOCATION_POOR;
    //       break;
    //     default:
    //       break;
    //   }
    //   if (E_OK != error_code) {
    //     AddedErrorInfo(channel_name_, error_code, E_PULL_OVER, timestamp);
    //   }
    // }
  }

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    HandleControlCommand(msg);
    CheckMessageFrequency(msg, channel_name_, module_type::E_CONTROL,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }
};

class PerceptionObstaclesReader
    : public CyberReader<perception::PerceptionObstacles> {
 public:
  using MsgType = perception::PerceptionObstacles;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;

  ~PerceptionObstaclesReader() = default;

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    TIMEDELAY_TRACE(DELAYTRACE_MONITOR, "monitor_fusion",
                    msg->header().sequence_num())
    CheckMessageFrequency(msg, channel_name_, module_type::E_PERCEPTION,
                          E_EMERGENCY_STOP, E_EMERGENCY_STOP);
  }
};

class TrafficLightDetectionReader
    : public CyberReader<perception::TrafficLightDetection> {
 public:
  using CyberReader<perception::TrafficLightDetection>::CyberReader;
};

class TransmissionObstaclesReader
    : public CyberReader<perception::TransmissionObstacles> {
 public:
  using MsgType = perception::TransmissionObstacles;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;

  ~TransmissionObstaclesReader() = default;

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    CheckMessageFrequency(msg, channel_name_, module_type::E_MEMS,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }
};

class ADCTrajectoryReader : public CyberReader<planning::ADCTrajectory> {
 public:
  using MsgType = planning::ADCTrajectory;
  using MsgPtr = typename CyberReader<MsgType>::MsgTypePtr;
  using CyberReader<MsgType>::CyberReader;

  ~ADCTrajectoryReader() = default;

 private:
  void CheckPlanningError(const MsgPtr& msg) noexcept {
    auto status = msg->header().status().error_code();
    if (common::ErrorCode::PLANNING_ERROR_NEED_RESTART != status) {
      planning_abnormal_timestamp_ = 0.0;
      return;
    }

    const auto timestamp = cyber::Clock::NowInSeconds();
    if (planning_abnormal_timestamp_ < kMinimumTime) {
      planning_abnormal_timestamp_ = timestamp;
      return;
    }

    if (std::abs(timestamp - planning_abnormal_timestamp_) >
        FLAGS_kill_planning_time_diff) {
      AINFO << "kill planning process,last PLANNING_ERROR_NEED_RESTART time:"
            << planning_abnormal_timestamp_ << ",now time=" << timestamp;
      planning_abnormal_timestamp_ = 0.0;
      KillProcessByName("planning.dag");
    }
  }

 protected:
  void MsgFreqCheck(const std::string& channel_name,
                    const module_type& module_type_name, double* threshold_freq,
                    double* expected_freq) noexcept override {
    if (module_type::E_PLANNING == module_type_name) {
      if (nullptr == received_msg_) {
        return;
      }
      planning::ADCTrajectory::TrajectoryScenario trajectory_scenario =
          received_msg_->trajectory_scenario();
      planning_in_teb_ =
          (trajectory_scenario != planning::ADCTrajectory::LANEFOLLOW);
      *expected_freq = planning_in_teb_ ? FLAGS_teb_planning_required_frq
                                        : DaemonMonitorData::Instance()
                                              ->monitor_configs_[channel_name]
                                              .threshold_freq;

      *threshold_freq = FLAGS_lowfreqthreshold * (*expected_freq);
    } else {
      AERROR << "type error, can not convert ADCTrajectory";
      constexpr double kMaxFrequencyEpsilon = 65535.0;

      *threshold_freq = kMaxFrequencyEpsilon;
      *expected_freq = kMaxFrequencyEpsilon;
    }
    return;
  }

  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    received_msg_ = msg;
    CheckPlanningError(msg);
    CheckMessageFrequency(msg, channel_name_, module_type::E_PLANNING,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }

  uint16_t GetSuitableDiffTimeThreshold() noexcept override {
    uint16_t suitable_diff_time_threshold = kDiffTimeThreshold;
    if (planning_in_teb_ && FLAGS_planning_trajectory_topic == channel_name_) {
      double freq = FLAGS_teb_planning_required_frq * FLAGS_lowfreqthreshold;
      suitable_diff_time_threshold = std::abs(freq) < kMinFrequencyEpsilon
                                         ? (1 / FLAGS_teb_planning_required_frq)
                                         : (1 / freq);
      return suitable_diff_time_threshold;
    }

    return suitable_diff_time_threshold;
  }

 private:
  bool planning_in_teb_ = {false};
  double planning_abnormal_timestamp_ = {0.0};

  MsgPtr received_msg_ = nullptr;
};

class PredictionObstaclesReader
    : public CyberReader<prediction::PredictionObstacles> {
 public:
  using MsgType = prediction::PredictionObstacles;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    TIMEDELAY_TRACE(DELAYTRACE_MONITOR, "monitor_prediction",
                    msg->header().sequence_num())
    CheckMessageFrequency(msg, channel_name_, module_type::E_PREDICTION,
                          E_EMERGENCY_STOP, E_EMERGENCY_STOP);
  }
};

class TrackerRsReader : public CyberReader<perception::TransmissionObstacles> {
 public:
  using MsgType = perception::TransmissionObstacles;
  using MsgPtr = std::shared_ptr<MsgType>;

  TrackerRsReader(const std::string& topic_name,
                  const std::shared_ptr<cyber::Node>& node,
                  double& abnormal_status_timestamp,
                  double& kill_process_timestamp,
                  double& localization_time_stamp)
      : CyberReader<MsgType>(topic_name, node),
        rs_abnormal_status_timestamp_(abnormal_status_timestamp),
        kill_process_timestamp_(kill_process_timestamp),
        localization_time_stamp_(localization_time_stamp) {}

 protected:
  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    CheckRsStatus(msg);
    if (nullptr != msg && 0 != msg->rs_sdk_status()) {
      CheckMessageFrequency(msg, channel_name_, module_type::E_RS,
                            E_EMERGENCY_STOP, E_PULL_OVER);
    }
  }

 private:
  void CheckRsStatus(const MsgPtr& msg) noexcept {
    int rs_sdk_status = msg->rs_sdk_status();
    if (0 == rs_sdk_status) {
      double timestamp = century::cyber::Clock::NowInSeconds();
      if (std::abs(timestamp - localization_time_stamp_) > kMsgDelayThreshold) {
        return;
      }

      if (++rs_sdk_status_zero_count_ >= FLAGS_rs_sdk_status_zero_count) {
        AddedErrorInfo(channel_name_, E_RS_SDK_STATUS_ZERO, E_PULL_OVER,
                       timestamp);
        AWARN << "rs_sdk_status_zero_count:" << rs_sdk_status_zero_count_;
      }

      if (kill_process_timestamp_ > 0.0) {
        if (timestamp - kill_process_timestamp_ >
            FLAGS_kill_process_time_diff) {
          AINFO << "kill ros sdk process,last kill process time:"
                << kill_process_timestamp_
                << ",now time=" << std::to_string(timestamp);
          //  KillProcessCmd("rs_sdk_demo");
          kill_process_timestamp_ = timestamp;
        }
        return;
      }

      if (rs_zero_status_timestamp_ < kDoubleEpsilon) {
        rs_zero_status_timestamp_ = timestamp;
      }

      if (timestamp - rs_zero_status_timestamp_ >
          FLAGS_kill_ros_sdk_time_diff) {
        AINFO << "kill ros sdk process,zero status  time:"
              << rs_zero_status_timestamp_
              << ",now time=" << std::to_string(timestamp);
        // KillProcessCmd("rs_sdk_demo");
        kill_process_timestamp_ = timestamp;
      }
    } else {
      kill_process_timestamp_ = 0.0;
      rs_zero_status_timestamp_ = 0.0;
      rs_sdk_status_zero_count_ = 0;
    }
  }

 private:
  double& rs_abnormal_status_timestamp_;
  double& kill_process_timestamp_;
  double& localization_time_stamp_;

  double rs_zero_status_timestamp_{0.0};
  uint32_t rs_sdk_status_zero_count_{0};
};

class LocalizationEstimateReader
    : public CyberReader<localization::LocalizationEstimate> {
 public:
  using MsgType = localization::LocalizationEstimate;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;
  using LocalStatus = localization::LocalizationEstimate;

  ~LocalizationEstimateReader() = default;

  void CheckTransmissionLatency() noexcept {
    transmission_.CheckTransmissionLatency(
        error_info_queue_,
        DaemonMonitorData::Instance()->error_code_infos_[E_EXCEEDING_TIME_DIFF],
        DaemonMonitorData::Instance()->cur_time_second_.load());
  }

 protected:
  void CreateReader() noexcept override {
    CyberReader<MsgType>::CreateReader();
    tracker_rs_reader_ = std::make_shared<TrackerRsReader>(
        FLAGS_tracker_rs_topic, node_, localization_abnormal_status_timestamp_,
        kill_process_timestamp_, localization_time_stamp_);
  }

  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    static uint8_t times = 0;

    // No need to check every time.
    if (kMaxCheckSkips == ++times) {
      transmission_.JudgeTransmissionLatency(msg->header().timestamp_sec());
      times = 0;
    }

    HandleLocalizationMsg(msg);
    CheckLocalizationStatus(msg);
    CheckMessageFrequency(msg, channel_name_, module_type::E_LOCALIZATION,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }

 private:
  void CheckLocalizationStatus(const MsgPtr& msg) noexcept {
    constexpr double kLastKillTimeDiffNowTime = 1.0;

    const auto pose_y = msg->pose().position().y();
    const auto loc_status = msg->status_type();

    // Early return if localization is normal
    if (pose_y >= kDoubleEpsilon && LocalStatus::LOST != loc_status) {
      localization_abnormal_status_timestamp_ = 0.0;
      return;
    }

    const double timestamp = cyber::Clock::NowInSeconds();
    if (abs(timestamp - kill_process_timestamp_) < kLastKillTimeDiffNowTime) {
      return;
    }

    if (localization_abnormal_status_timestamp_ < kDoubleEpsilon) {
      localization_abnormal_status_timestamp_ = timestamp;
    }

    if (!(timestamp - localization_abnormal_status_timestamp_ >
          FLAGS_loc_abnormal_status_time)) {
      return;
    }
    const std::string abnormal_msg =
        msg->status_type() == LocalStatus::LOST ? "localization lost" : "y = 0";

    AddedErrorInfo(channel_name_, E_LOCALIZATION_STATUS_ABNORMAL,
                   E_EMERGENCY_STOP, timestamp);
    AINFO << "Kill location process, reason: " << abnormal_msg << ", time: "
          << std::to_string(localization_abnormal_status_timestamp_)
          << ", now time: " << std::to_string(timestamp);

    // KillProcessByName("rs_sdk_demo");
    kill_process_timestamp_ = timestamp;
  }

  void HandleLocalizationMsg(const MsgPtr& msg) noexcept {
    static double poor_accuracy_start_time = 0.0;
    const double time_stamp = msg->header().timestamp_sec();
    localization_time_stamp_ = cyber::Clock::NowInSeconds();
    const int localization_status = msg->status_type();

    if (LocalStatus::NORMAL == localization_status) {
      poor_accuracy_start_time = 0.0;
      return;
    }

    ErrorLevel level = (LocalStatus::POOR_ACCURACY == localization_status)
                           ? E_INFO
                           : E_EMERGENCY_STOP;

    if (auto_driving_status_.load() &&
        LocalStatus::POOR_ACCURACY == localization_status) {
      if (common::util::IsFloatEqual(poor_accuracy_start_time, 0.0)) {
        poor_accuracy_start_time =
            DaemonMonitorData::Instance()->cur_time_second_;
      }
      if (DaemonMonitorData::Instance()->cur_time_second_ -
              poor_accuracy_start_time >=
          kPoorAccuracyDuration) {
        level = E_EMERGENCY_STOP;
      }
    } else {
      poor_accuracy_start_time = 0.0;
    }

    AddedErrorInfo(channel_name_, E_LOCALIZATION_STATUS_ABNORMAL, level,
                   time_stamp);

    AERROR << "Abnormal localization status - RS: " << localization_status;
  }

 private:
  double localization_time_stamp_ = {0.0};
  std::atomic_bool auto_driving_status_ = {false};
  double localization_abnormal_status_timestamp_ = {0.0};
  double kill_process_timestamp_ = {0.0};

  std::shared_ptr<TrackerRsReader> tracker_rs_reader_;
  TransmissionInterface transmission_;
};

class LocReader : public CyberReader<localization::LocalizationEstimate> {
 public:
  using MsgType = localization::LocalizationEstimate;
  using MsgPtr = std::shared_ptr<MsgType>;
  using CyberReader<MsgType>::CyberReader;
  using LocalStatus = localization::LocalizationEstimate;

  LocReader(const std::shared_ptr<LocalizationEstimateReader>& other_loc,
            const std::string& channel_name,
            const std::shared_ptr<cyber::Node>& node)
      : CyberReader<localization::LocalizationEstimate>(channel_name, node),
        localization_reader_(other_loc) {
    if (!localization_reader_) {
      AERROR << "LocalizationEstimateReader is null!";
    }
  }

  ~LocReader() = default;

  void CheckTransmissionLatency() noexcept {
    transmission_.CheckTransmissionLatency(
        error_info_queue_,
        DaemonMonitorData::Instance()->error_code_infos_[E_EXCEEDING_TIME_DIFF],
        DaemonMonitorData::Instance()->cur_time_second_.load());
  }

 protected:
  void CreateReader() noexcept override {
    CyberReader<MsgType>::CreateReader();

    // localization_reader_ =
    // std::make_shared<LocalizationEstimateReader>(FLAGS_localization_topic,
    // node_);

    tracker_rs_reader_ = std::make_shared<TrackerRsReader>(
        FLAGS_tracker_rs_topic, node_, localization_abnormal_status_timestamp_,
        kill_process_timestamp_, localization_time_stamp_);
  }

  void OnReceivedMsg(const MsgPtr& msg) noexcept override {
    static uint8_t times = 0;

    // No need to check every time.
    if (kMaxCheckSkips == ++times) {
      transmission_.JudgeTransmissionLatency(msg->header().timestamp_sec());
      times = 0;
    }

    HandleLocalizationMsg(msg);
    CheckLocalizationStatus(msg);
    CheckMessageFrequency(msg, channel_name_, module_type::E_LOCALIZATION,
                          E_EMERGENCY_STOP, E_PULL_OVER);
  }

 private:
  void CheckLocalizationStatus(const MsgPtr& msg) noexcept {
    constexpr double kLastKillTimeDiffNowTime = 1.0;

    const auto pose_y = msg->pose().position().y();
    const auto loc_status = msg->status_type();

    // Early return if localization is normal
    if (pose_y >= kDoubleEpsilon && LocalStatus::LOST != loc_status) {
      localization_abnormal_status_timestamp_ = 0.0;
      return;
    }

    const double timestamp = cyber::Clock::NowInSeconds();
    if (abs(timestamp - kill_process_timestamp_) < kLastKillTimeDiffNowTime) {
      return;
    }

    if (localization_abnormal_status_timestamp_ < kDoubleEpsilon) {
      localization_abnormal_status_timestamp_ = timestamp;
    }

    if (!(timestamp - localization_abnormal_status_timestamp_ >
          FLAGS_loc_abnormal_status_time)) {
      return;
    }
    AERROR << "Check CheckLocalizationStatus";
    const std::string abnormal_msg =
        msg->status_type() == LocalStatus::LOST ? "localization lost" : "y = 0";

    AddedErrorInfo(channel_name_, E_LOC_STATUS_ABNORMAL, E_EMERGENCY_STOP,
                   timestamp);
    AINFO << "Kill location process, reason: " << abnormal_msg << ", time: "
          << std::to_string(localization_abnormal_status_timestamp_)
          << ", now time: " << std::to_string(timestamp);

    // KillProcessByName("rs_sdk_demo");
    kill_process_timestamp_ = timestamp;
  }

  bool IsDistanceExceedThreshold(double msg_x, double msg_y, double other_msg_x,
                                 double other_msg_y, double speed) {
    double diff_x = abs(msg_x - other_msg_x);
    double diff_y = abs(msg_y - other_msg_y);
    double distance = std::hypot(msg_x - other_msg_x, msg_y - other_msg_y);
    if (diff_x > FLAGS_localization_diff_horizontal ||
        diff_y > FLAGS_localization_diff_horizontal ||
        distance > FLAGS_localization_diff_site) {
      return false;
    }
    return true;
  }

  void HandleLocalizationMsg(const MsgPtr& msg) noexcept {
    static double poor_accuracy_start_time = 0.0;
    const double time_stamp = msg->header().timestamp_sec();
    localization_time_stamp_ = cyber::Clock::NowInSeconds();
    const int localization_status = msg->status_type();

    localization_reader_->GetReader()->Observe();
    auto other_msg = localization_reader_->GetReader()->GetLatestObserved();

    if (other_msg) {
      auto other_msg_x = other_msg->pose().position().x();
      auto other_msg_y = other_msg->pose().position().y();

      auto msg_x = msg->pose().position().x();
      auto msg_y = msg->pose().position().y();

      double speed = FLAGS_planning_speed_limit * 100;
      if (!IsDistanceExceedThreshold(msg_x, msg_y, other_msg_x, other_msg_y,
                                     speed)) {
        AERROR << "two localization diff more than max";
        AddedErrorInfo(channel_name_, E_LOC_DIFF_MAX, E_EMERGENCY_STOP,
                       time_stamp);
      }
    }

    if (LocalStatus::NORMAL == localization_status) {
      poor_accuracy_start_time = 0.0;
      return;
    }

    ErrorLevel level = (LocalStatus::POOR_ACCURACY == localization_status)
                           ? E_INFO
                           : E_EMERGENCY_STOP;

    if (auto_driving_status_.load() &&
        LocalStatus::POOR_ACCURACY == localization_status) {
      if (common::util::IsFloatEqual(poor_accuracy_start_time, 0.0)) {
        poor_accuracy_start_time =
            DaemonMonitorData::Instance()->cur_time_second_;
      }
      if (DaemonMonitorData::Instance()->cur_time_second_ -
              poor_accuracy_start_time >=
          kPoorAccuracyDuration) {
        level = E_EMERGENCY_STOP;
      }
    } else {
      poor_accuracy_start_time = 0.0;
    }

    AddedErrorInfo(channel_name_, E_LOC_STATUS_ABNORMAL, level, time_stamp);

    AERROR << "Abnormal localization status - RS: " << localization_status;
  }

 private:
  double localization_time_stamp_ = {0.0};
  std::atomic_bool auto_driving_status_ = {false};
  double localization_abnormal_status_timestamp_ = {0.0};
  double kill_process_timestamp_ = {0.0};

  std::shared_ptr<TrackerRsReader> tracker_rs_reader_;
  std::shared_ptr<LocalizationEstimateReader> localization_reader_;

  TransmissionInterface transmission_;
};

// class ImuRawReader : public CyberReader<century::modules::drivers::imu::Imu>
// {
//  public:
//   using MsgType = century::modules::drivers::imu::Imu;
//   using MsgPtr = std::shared_ptr<MsgType>;
//   using CyberReader<MsgType>::CyberReader;

//  protected:
//   void OnReceivedMsg(const MsgPtr& msg) noexcept override {
//     CheckMessageFrequency(msg, channel_name_, module_type::E_IMU_RAW,
//     E_WARNING,
//                           E_WARNING);
//   }
// };

// class MemsStatusReader : public CyberReader<drivers::zvision::MemsStatus> {
//  public:
//   using MsgType = drivers::zvision::MemsStatus;
//   using MsgPtr = std::shared_ptr<MsgType>;
//   using CyberReader<MsgType>::CyberReader;

//  protected:
//   void OnReceivedMsg(const MsgPtr& msg) noexcept override {
//     CheckError(msg);
//     CheckMessageFrequency(msg, channel_name_, module_type::E_MEMS, E_WARNING,
//                           E_WARNING);
//   }

//  private:
//   void CheckError(const MsgPtr& msg) noexcept {
//     double time_stamp = msg->header().timestamp_sec();
//     int32_t mems_status = msg->status();
//     const int32_t mems_normal_status = 7;

//     if (mems_status == mems_normal_status) {
//       return;
//     }

//     auto error_code = static_cast<MonitorErrorCode>(
//         E_MEMS_FRONT_LEFT_RIGHT_STATUS_ABNORMAL + mems_status);
//     AddedErrorInfo(channel_name_, error_code, E_WARNING, time_stamp);
//   }
// };

class DaemonReaderManager {
 public:
  DaemonReaderManager(const std::shared_ptr<cyber::Node>& daemon_monitor_node)
      : daemon_monitor_node_(daemon_monitor_node) {}

  ~DaemonReaderManager() = default;

  void CheckTransmissionLatency() noexcept {
    auto localization_ptr =
        std::dynamic_pointer_cast<LocalizationEstimateReader>(
            localization_reader_);
    if (localization_ptr) {
      localization_ptr->CheckTransmissionLatency();
    }

    auto loc_ptr =
        std::dynamic_pointer_cast<LocalizationEstimateReader>(loc_reader_);
    if (loc_ptr) {
      loc_ptr->CheckTransmissionLatency();
    }

    auto chassis_ptr =
        std::dynamic_pointer_cast<ChassisReader>(chassis_reader_);
    if (chassis_ptr) {
      chassis_ptr->CheckTransmissionLatency();
    }
    AERROR << "Run CheckTransmissionLatency";
    double timestamp = century::cyber::Clock::NowInSeconds();
    for (const auto& reader : all_readers_) {
      reader->CheckModuleChannels(timestamp);
    }
  }

  void CreateReader() noexcept {
    // all_readers_ = {

    all_readers_.emplace_back(CreateReaderByTopic<ChassisReader>(
        FLAGS_chassis_topic, daemon_monitor_node_));
    all_readers_.emplace_back(CreateReaderByTopic<PerceptionObstaclesReader>(
        FLAGS_perception_obstacle_topic, daemon_monitor_node_));
    localization_reader_ = CreateReaderByTopic<LocalizationEstimateReader>(
        FLAGS_localization_topic, daemon_monitor_node_);

    all_readers_.emplace_back(localization_reader_);
    all_readers_.emplace_back(CreateReaderByTopic<ControlCommandReader>(
        FLAGS_control_command_topic, daemon_monitor_node_));
    all_readers_.emplace_back(CreateReaderByTopic<PredictionObstaclesReader>(
        FLAGS_prediction_topic, daemon_monitor_node_));
    all_readers_.emplace_back(CreateReaderByTopic<ADCTrajectoryReader>(
        FLAGS_planning_trajectory_topic, daemon_monitor_node_));
    if (FLAGS_loc_topic != FLAGS_localization_topic) {
      loc_reader_ = CreateReaderByTopic<LocalizationEstimateReader, LocReader>(
          std::dynamic_pointer_cast<LocalizationEstimateReader>(
              localization_reader_),
          FLAGS_loc_topic, daemon_monitor_node_);
      all_readers_.emplace_back(loc_reader_);
    }

    // imu_raw_reader_ = std::make_shared<ImuRawReader>(FLAGS_imu_raw_topic,
    //                                                  daemon_monitor_node_);
    // imu_raw_reader_->CreateReader();

    // mems_status_reader_ = std::make_shared<MemsStatusReader>(
    //     FLAGS_mems_status_topic, daemon_monitor_node_);
    // mems_status_reader_->CreateReader();

    transmission_obstacles_reader_ =
        std::make_shared<TransmissionObstaclesReader>(FLAGS_tracker_mems_topic,
                                                      daemon_monitor_node_);
    transmission_obstacles_reader_->CreateReader();
  }

 private:
  std::shared_ptr<cyber::Node> daemon_monitor_node_;

  std::shared_ptr<BaseReader> chassis_reader_;
  std::shared_ptr<BaseReader> perception_obstacle_reader_;
  std::shared_ptr<BaseReader> localization_reader_;
  std::shared_ptr<BaseReader> control_command_reader_;
  std::shared_ptr<BaseReader> prediction_obstacle_reader_;
  std::shared_ptr<BaseReader> planning_reader_;
  std::shared_ptr<BaseReader> loc_reader_;
  // std::shared_ptr<BaseReader> imu_raw_reader_;
  // std::shared_ptr<BaseReader> mems_status_reader_;
  std::shared_ptr<BaseReader> transmission_obstacles_reader_;

  std::vector<std::shared_ptr<BaseReader>> all_readers_;
};

}  // namespace monitor
}  // namespace century
