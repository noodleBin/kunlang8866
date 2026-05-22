/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
#include "modules/camera_monitor/camera_monitor_component.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace century {
namespace camera_monitor {

CameraMonitorComponent::CameraMonitorComponent() = default;

CameraMonitorComponent::~CameraMonitorComponent() {
  AINFO << "CameraMonitorComponent::~CameraMonitorComponent()";

  if (timer_) {
    timer_->Stop();
    timer_.reset();
  }

  front_image_reader_.reset();
  front_compressed_image_reader_.reset();

  rear_image_reader_.reset();
  rear_compressed_image_reader_.reset();

  left_front_image_reader_.reset();
  left_front_compressed_image_reader_.reset();

  left_rear_image_reader_.reset();
  left_rear_compressed_image_reader_.reset();

  right_front_image_reader_.reset();
  right_front_compressed_image_reader_.reset();

  right_rear_image_reader_.reset();
  right_rear_compressed_image_reader_.reset();
}

bool CameraMonitorComponent::Init() {
  front_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/front/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::FRONT, msg);
      });

  rear_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/rear/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::REAR, msg);
      });

  left_front_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/left_front/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::LEFT_FRONT, msg);
      });

  left_rear_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/left_rear/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::LEFT_REAR, msg);
      });

  right_front_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/right_front/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::RIGHT_FRONT, msg);
      });

  right_rear_image_reader_ = node_->CreateReader<century::drivers::Image>(
      "/century/sensor/camera/right_rear/image",
      [this](const std::shared_ptr<century::drivers::Image>& msg) {
        HandleImage(CameraIndex::RIGHT_REAR, msg);
      });

/*
front_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/front/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::FRONT, msg);
      });

rear_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/rear/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::REAR, msg);
      });

left_front_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/left_front/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::LEFT_FRONT, msg);
      });

left_rear_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/left_rear/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::LEFT_REAR, msg);
      });

right_front_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/right_front/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::RIGHT_FRONT, msg);
      });

right_rear_compressed_image_reader_ =
  node_->CreateReader<century::drivers::CompressedImage>(
      "/century/sensor/camera/right_rear/image/compressed",
      [this](
          const std::shared_ptr<century::drivers::CompressedImage>& msg) {
        HandleCompressedImage(CameraIndex::RIGHT_REAR, msg);
      });
*/

  timer_ = std::make_unique<cyber::Timer>(
      kDisplayStaticsMs, [this]() { this->OnTimer(); }, false);

  timer_->Start();

  AINFO << "CameraMonitorComponent init success";
  return true;
}

void CameraMonitorComponent::HandleImage(
    CameraIndex camera_index,
    const std::shared_ptr<century::drivers::Image>& msg) {
  if (!msg) {
    return;
  }

  const double receive_time_sec = century::cyber::Time::Now().ToSecond();
  const double exposure_time_sec = msg->header().timestamp_sec();
  const double send_time_sec = msg->measurement_time();

  GetStatistics(camera_index, StreamIndex::IMAGE)
      .Update(CameraName(camera_index), StreamName(StreamIndex::IMAGE),
              exposure_time_sec, send_time_sec, receive_time_sec);
}

void CameraMonitorComponent::HandleCompressedImage(
    CameraIndex camera_index,
    const std::shared_ptr<century::drivers::CompressedImage>& msg) {
}

CameraMonitorComponent::DelayStatistics& CameraMonitorComponent::GetStatistics(
    CameraIndex camera_index, StreamIndex stream_index) {
  return statistics_[static_cast<std::size_t>(camera_index)]
                    [static_cast<std::size_t>(stream_index)];
}

void CameraMonitorComponent::DelayStatistics::Update(const char* camera_name,
                                                     const char* stream_name,
                                                     double exposure_time_sec,
                                                     double send_time_sec,
                                                     double receive_time_sec) {
  std::lock_guard<std::mutex> lock(mutex_);

  ++total_packets_;

  const bool invalid_time =
      !std::isfinite(exposure_time_sec) || !std::isfinite(send_time_sec) ||
      !std::isfinite(receive_time_sec) || exposure_time_sec <= 0.0 ||
      send_time_sec <= 0.0 || receive_time_sec <= 0.0 ||
      send_time_sec < exposure_time_sec || receive_time_sec < send_time_sec;

  if (invalid_time) {
    ++invalid_packets_;

    AINFO << "[camera_delay_invalid]"
          << " camera=" << camera_name << ", stream=" << stream_name
          << ", exposure_time_sec=" << exposure_time_sec
          << ", send_time_sec=" << send_time_sec
          << ", receive_time_sec=" << receive_time_sec;

    return;
  }

  const double exposure_to_send_delay_ms =
      (send_time_sec - exposure_time_sec) * 1000.0;

  const double transport_delay_ms = (receive_time_sec - send_time_sec) * 1000.0;

  const double end_to_end_delay_ms =
      (receive_time_sec - exposure_time_sec) * 1000.0;

  if (end_to_end_delay_ms > kEndToEndDelayWarnThresholdMs) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);

    oss << "[camera_delay_warn]"
        << " camera=" << camera_name << ", stream=" << stream_name
        << ", exposure_time_sec=" << exposure_time_sec
        << ", send_time_sec=" << send_time_sec
        << ", receive_time_sec=" << receive_time_sec << std::setprecision(3)
        << ", exposure_to_send_delay_ms=" << exposure_to_send_delay_ms
        << ", transport_delay_ms=" << transport_delay_ms
        << ", end_to_end_delay_ms=" << end_to_end_delay_ms
        << ", threshold_ms=" << kEndToEndDelayWarnThresholdMs;

    AINFO << oss.str();
  }

  exposure_to_send_sum_ms_ += exposure_to_send_delay_ms;
  transport_sum_ms_ += transport_delay_ms;
  end_to_end_sum_ms_ += end_to_end_delay_ms;

  exposure_to_send_max_ms_ =
      std::max(exposure_to_send_max_ms_, exposure_to_send_delay_ms);

  transport_max_ms_ = std::max(transport_max_ms_, transport_delay_ms);

  end_to_end_max_ms_ = std::max(end_to_end_max_ms_, end_to_end_delay_ms);

  if (transport_delay_ms < 5.0) {
    ++transport_less_5ms_packets_;
  } else if (transport_delay_ms < 10.0) {
    ++transport_5_10ms_packets_;
  } else if (transport_delay_ms < 15.0) {
    ++transport_10_15ms_packets_;
  } else if (transport_delay_ms < 20.0) {
    ++transport_15_20ms_packets_;
  } else {
    ++transport_greater_equal_20ms_packets_;
  }
}

CameraMonitorComponent::DelaySnapshot
CameraMonitorComponent::DelayStatistics::SnapshotAndReset() {
  std::lock_guard<std::mutex> lock(mutex_);

  DelaySnapshot snapshot;

  snapshot.total_packets = total_packets_;
  snapshot.invalid_packets = invalid_packets_;

  snapshot.transport_less_5ms_packets = transport_less_5ms_packets_;
  snapshot.transport_5_10ms_packets = transport_5_10ms_packets_;
  snapshot.transport_10_15ms_packets = transport_10_15ms_packets_;
  snapshot.transport_15_20ms_packets = transport_15_20ms_packets_;
  snapshot.transport_greater_equal_20ms_packets =
      transport_greater_equal_20ms_packets_;

  snapshot.exposure_to_send_sum_ms = exposure_to_send_sum_ms_;
  snapshot.transport_sum_ms = transport_sum_ms_;
  snapshot.end_to_end_sum_ms = end_to_end_sum_ms_;

  snapshot.exposure_to_send_max_ms = exposure_to_send_max_ms_;
  snapshot.transport_max_ms = transport_max_ms_;
  snapshot.end_to_end_max_ms = end_to_end_max_ms_;

  ResetLocked();

  return snapshot;
}

void CameraMonitorComponent::DelayStatistics::ResetLocked() {
  total_packets_ = 0;
  invalid_packets_ = 0;

  transport_less_5ms_packets_ = 0;
  transport_5_10ms_packets_ = 0;
  transport_10_15ms_packets_ = 0;
  transport_15_20ms_packets_ = 0;
  transport_greater_equal_20ms_packets_ = 0;

  exposure_to_send_sum_ms_ = 0.0;
  transport_sum_ms_ = 0.0;
  end_to_end_sum_ms_ = 0.0;

  exposure_to_send_max_ms_ = 0.0;
  transport_max_ms_ = 0.0;
  end_to_end_max_ms_ = 0.0;
}

void CameraMonitorComponent::OnTimer() {
  AINFO << "================ Camera Delay Statistics, interval: "
        << kDisplayStaticsMs / 1000 << "s ================";

  for (std::size_t camera = 0;
       camera < static_cast<std::size_t>(CameraIndex::COUNT); ++camera) {
    for (std::size_t stream = 0;
         stream < static_cast<std::size_t>(StreamIndex::COUNT); ++stream) {
      const auto camera_index = static_cast<CameraIndex>(camera);
      const auto stream_index = static_cast<StreamIndex>(stream);

      DelaySnapshot snapshot =
          GetStatistics(camera_index, stream_index).SnapshotAndReset();

      const uint64_t valid_packets =
          snapshot.total_packets >= snapshot.invalid_packets
              ? snapshot.total_packets - snapshot.invalid_packets
              : 0;

      const double exposure_to_send_avg_ms =
          valid_packets > 0 ? snapshot.exposure_to_send_sum_ms /
                                  static_cast<double>(valid_packets)
                            : 0.0;

      const double transport_avg_ms =
          valid_packets > 0
              ? snapshot.transport_sum_ms / static_cast<double>(valid_packets)
              : 0.0;

      const double end_to_end_avg_ms =
          valid_packets > 0
              ? snapshot.end_to_end_sum_ms / static_cast<double>(valid_packets)
              : 0.0;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3);

      oss << "[camera=" << CameraName(camera_index)
          << "][stream=" << StreamName(stream_index)
          << "] total=" << snapshot.total_packets << ", valid=" << valid_packets
          << ", invalid=" << snapshot.invalid_packets
          << ", transport_bucket_ms={"
          << "<5:" << snapshot.transport_less_5ms_packets
          << ", 5~10:" << snapshot.transport_5_10ms_packets
          << ", 10~15:" << snapshot.transport_10_15ms_packets
          << ", 15~20:" << snapshot.transport_15_20ms_packets
          << ", >=20:" << snapshot.transport_greater_equal_20ms_packets << "}"
          << ", exposure_to_send_avg_ms=" << exposure_to_send_avg_ms
          << ", exposure_to_send_max_ms=" << snapshot.exposure_to_send_max_ms
          << ", transport_avg_ms=" << transport_avg_ms
          << ", transport_max_ms=" << snapshot.transport_max_ms
          << ", end_to_end_avg_ms=" << end_to_end_avg_ms
          << ", end_to_end_max_ms=" << snapshot.end_to_end_max_ms;

      AINFO << oss.str();
    }
  }

  AINFO << "==============================================================";
}

const char* CameraMonitorComponent::CameraName(CameraIndex camera_index) {
  switch (camera_index) {
    case CameraIndex::FRONT:
      return "front";
    case CameraIndex::REAR:
      return "rear";
    case CameraIndex::LEFT_FRONT:
      return "left_front";
    case CameraIndex::LEFT_REAR:
      return "left_rear";
    case CameraIndex::RIGHT_FRONT:
      return "right_front";
    case CameraIndex::RIGHT_REAR:
      return "right_rear";
    default:
      return "unknown";
  }
}

const char* CameraMonitorComponent::StreamName(StreamIndex stream_index) {
  switch (stream_index) {
    case StreamIndex::IMAGE:
      return "image";
    default:
      return "unknown";
  }
}

}  // namespace camera_monitor
}  // namespace century

