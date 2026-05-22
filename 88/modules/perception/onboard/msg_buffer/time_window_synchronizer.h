/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "modules/drivers/proto/sensor_image.pb.h"

namespace century {
namespace perception {
namespace onboard {

constexpr int kCameraCount = 6;

struct CameraInputData {
  std::shared_ptr<drivers::CompressedImage> compressed_image;
  std::shared_ptr<drivers::Image> raw_image;
};

class TimeWindowSynchronizer {
 public:
  TimeWindowSynchronizer() = default;

  static constexpr double kSlowWaitLogMs = 20.0;

  struct WindowState {
    std::array<std::optional<CameraInputData>, kCameraCount> slots;
    std::array<std::optional<double>, kCameraCount> raw_timestamps;
    std::array<std::optional<double>, kCameraCount> arrival_wall_times;
    std::array<std::optional<int64_t>, kCameraCount> aligned_timestamps;
    std::array<int, kCameraCount> arrival_order{{-1, -1, -1, -1, -1, -1}};
    int filled_count = 0;
    std::chrono::steady_clock::time_point first_arrival_tp;
  };

  int64_t AlignToHundredMs(int64_t milliseconds);
  int64_t AlignTimestamp(double timestamp);
  double TimestampMsToSeconds(int64_t timestamp_ms) const;
  std::string FormatWindowTimestamp(int64_t timestamp_ms) const;

  bool Query(double lidar_timestamp,
             std::array<CameraInputData, kCameraCount>& result);

  void BuildFallbackInput(
      double lidar_timestamp,
      std::array<CameraInputData, kCameraCount>& result,
      std::array<bool, kCameraCount>& available_mask);

  void PushCompressed(int cam_id,
                      const std::shared_ptr<drivers::CompressedImage>& image);
  void PushRaw(int cam_id, const std::shared_ptr<drivers::Image>& image);
  void Push(int cam_id, double timestamp, const CameraInputData& input);

 private:
  bool HasCompleteWindowLocked(int64_t target_window) const;
  double ElapsedMs(const std::chrono::steady_clock::time_point& start) const;
  double CurrentUnixSeconds() const;
  std::string SnapshotLocked() const;
  std::string WindowSummaryLocked(int64_t target_window) const;
  double ArrivalOffsetMsLocked(
      int cam_id,
      const std::array<std::optional<CameraInputData>, kCameraCount>& slots,
      const std::array<std::optional<double>, kCameraCount>& raw_timestamps,
      const std::array<int, kCameraCount>& arrival_order) const;
  std::string FormatWindowSnapshotLocked(
      int64_t timestamp, int filled_count, double fill_elapsed_ms,
      const std::array<std::optional<CameraInputData>, kCameraCount>& slots,
      const std::array<std::optional<double>, kCameraCount>& raw_timestamps,
      const std::array<std::optional<double>, kCameraCount>& arrival_wall_times,
      const std::array<std::optional<int64_t>, kCameraCount>&
          aligned_timestamps,
      const std::array<int, kCameraCount>& arrival_order) const;
  void CopyWindowLocked(
      int64_t target_window,
      std::array<CameraInputData, kCameraCount>& result) const;
  void CopyPartialWindowLocked(
      int64_t target_window,
      std::array<CameraInputData, kCameraCount>& result,
      std::array<bool, kCameraCount>& available_mask) const;
  void EraseWindowLocked(int64_t target_window);
  void CleanupOldWindowsLocked(int64_t newest_window);

  std::mutex mutex_;
  std::map<int64_t, WindowState> windows_;
  std::condition_variable condition_variable_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace century
