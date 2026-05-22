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

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "modules/drivers/proto/sensor_image.pb.h"

#include "cyber/cyber.h"

namespace century {
namespace camera_monitor {

using century::cyber::Component;

class CameraMonitorComponent final : public Component<> {
 public:
  CameraMonitorComponent();
  ~CameraMonitorComponent() override;

  bool Init() override;

 private:
  static constexpr uint32_t kDisplayStaticsMs = 30 * 1000U;
  static constexpr double kEndToEndDelayWarnThresholdMs = 60.0;

  enum class CameraIndex : std::size_t {
    FRONT = 0,
    REAR,
    LEFT_FRONT,
    LEFT_REAR,
    RIGHT_FRONT,
    RIGHT_REAR,
    COUNT
  };

  // enum class StreamIndex : std::size_t { IMAGE = 0, COMPRESSED_IMAGE, COUNT };
  enum class StreamIndex : std::size_t { IMAGE = 0, COUNT };

  struct DelaySnapshot {
    uint64_t total_packets = 0;
    uint64_t invalid_packets = 0;

    uint64_t transport_less_5ms_packets = 0;
    uint64_t transport_5_10ms_packets = 0;
    uint64_t transport_10_15ms_packets = 0;
    uint64_t transport_15_20ms_packets = 0;
    uint64_t transport_greater_equal_20ms_packets = 0;

    double exposure_to_send_sum_ms = 0.0;
    double transport_sum_ms = 0.0;
    double end_to_end_sum_ms = 0.0;

    double exposure_to_send_max_ms = 0.0;
    double transport_max_ms = 0.0;
    double end_to_end_max_ms = 0.0;
  };

  class DelayStatistics {
   public:
    void Update(const char* camera_name, const char* stream_name,
                double exposure_time_sec, double send_time_sec,
                double receive_time_sec);

    DelaySnapshot SnapshotAndReset();

   private:
    void ResetLocked();

    std::mutex mutex_;

    uint64_t total_packets_ = 0;
    uint64_t invalid_packets_ = 0;

    uint64_t transport_less_5ms_packets_ = 0;
    uint64_t transport_5_10ms_packets_ = 0;
    uint64_t transport_10_15ms_packets_ = 0;
    uint64_t transport_15_20ms_packets_ = 0;
    uint64_t transport_greater_equal_20ms_packets_ = 0;

    double exposure_to_send_sum_ms_ = 0.0;
    double transport_sum_ms_ = 0.0;
    double end_to_end_sum_ms_ = 0.0;

    double exposure_to_send_max_ms_ = 0.0;
    double transport_max_ms_ = 0.0;
    double end_to_end_max_ms_ = 0.0;
  };

  using StreamStatistics =
      std::array<DelayStatistics, static_cast<std::size_t>(StreamIndex::COUNT)>;

  using CameraStatistics =
      std::array<StreamStatistics,
                 static_cast<std::size_t>(CameraIndex::COUNT)>;

  void OnTimer();

  void HandleImage(CameraIndex camera_index,
                   const std::shared_ptr<century::drivers::Image>& msg);

  void HandleCompressedImage(
      CameraIndex camera_index,
      const std::shared_ptr<century::drivers::CompressedImage>& msg);

  DelayStatistics& GetStatistics(CameraIndex camera_index,
                                 StreamIndex stream_index);

  static const char* CameraName(CameraIndex camera_index);
  static const char* StreamName(StreamIndex stream_index);

 private:
  CameraStatistics statistics_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>> front_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      front_compressed_image_reader_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>> rear_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      rear_compressed_image_reader_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>>
      left_front_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      left_front_compressed_image_reader_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>>
      left_rear_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      left_rear_compressed_image_reader_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>>
      right_front_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      right_front_compressed_image_reader_;

  std::shared_ptr<cyber::Reader<century::drivers::Image>>
      right_rear_image_reader_;
  std::shared_ptr<cyber::Reader<century::drivers::CompressedImage>>
      right_rear_compressed_image_reader_;

  std::unique_ptr<cyber::Timer> timer_;
};

CYBER_REGISTER_COMPONENT(CameraMonitorComponent)

}  // namespace camera_monitor
}  // namespace century
