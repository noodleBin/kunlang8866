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

#include "modules/perception/onboard/msg_buffer/time_window_synchronizer.h"

#include <iomanip>
#include <sstream>

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace onboard {

int64_t TimeWindowSynchronizer::AlignToHundredMs(int64_t milliseconds) {
  return ((milliseconds + 50) / 100) * 100;
}

int64_t TimeWindowSynchronizer::AlignTimestamp(double timestamp) {
  int64_t timestamp_ms = static_cast<int64_t>(timestamp * 1000.0 + 0.5);
  return AlignToHundredMs(timestamp_ms);
}

double TimeWindowSynchronizer::TimestampMsToSeconds(
    int64_t timestamp_ms) const {
  return static_cast<double>(timestamp_ms) / 1000.0;
}

std::string TimeWindowSynchronizer::FormatWindowTimestamp(
    int64_t timestamp_ms) const {
  if (0 > timestamp_ms) {
    return "unset";
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << TimestampMsToSeconds(timestamp_ms);
  return oss.str();
}

bool TimeWindowSynchronizer::Query(
    double lidar_timestamp, std::array<CameraInputData, kCameraCount>& result) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto wait_start = std::chrono::steady_clock::now();
  const int64_t target_window = AlignTimestamp(lidar_timestamp);

  AINFO << std::fixed << std::setprecision(6)
        << "[CameraSyncQuery] lidar_ts=" << lidar_timestamp
        << " target_window=" << FormatWindowTimestamp(target_window)
        << " | target_state=" << WindowSummaryLocked(target_window);

  if (HasCompleteWindowLocked(target_window)) {
    double wait_ms = ElapsedMs(wait_start);
    if (wait_ms >= kSlowWaitLogMs) {
      AINFO << std::fixed << std::setprecision(3)
            << "[CameraSync] Target window consumed after " << wait_ms
            << " ms wait | target_window="
            << FormatWindowTimestamp(target_window)
            << " | target_state=" << WindowSummaryLocked(target_window);
    }
    CopyWindowLocked(target_window, result);
    EraseWindowLocked(target_window);
    return true;
  }

  if (condition_variable_.wait_for(
          lock, std::chrono::milliseconds(30), [this, target_window]() {
            return HasCompleteWindowLocked(target_window);
          })) {
    double wait_ms = ElapsedMs(wait_start);
    if (wait_ms >= kSlowWaitLogMs) {
      AINFO << std::fixed << std::setprecision(3)
            << "[CameraSync] Slow synchronized window ready after " << wait_ms
            << " ms | target_window=" << FormatWindowTimestamp(target_window)
            << " | target_state=" << WindowSummaryLocked(target_window);
    }
    CopyWindowLocked(target_window, result);
    EraseWindowLocked(target_window);
    return true;
  }

  AERROR << std::fixed << std::setprecision(3)
         << "[CameraSync] Timeout waiting for camera data after "
         << ElapsedMs(wait_start)
         << " ms | target_window=" << FormatWindowTimestamp(target_window)
         << " | target_state=" << WindowSummaryLocked(target_window);
  return false;
}

void TimeWindowSynchronizer::BuildFallbackInput(
    double lidar_timestamp, std::array<CameraInputData, kCameraCount>& result,
    std::array<bool, kCameraCount>& available_mask) {
  std::unique_lock<std::mutex> lock(mutex_);
  const int64_t target_window = AlignTimestamp(lidar_timestamp);
  result.fill(CameraInputData());
  available_mask.fill(false);

  auto it = windows_.find(target_window);
  if (windows_.end() == it) {
    AWARN << std::fixed << std::setprecision(6)
          << "[CameraSyncFallback] No camera data found for target_window="
          << FormatWindowTimestamp(target_window)
          << ", fallback to all-black images.";
    return;
  }

  CopyPartialWindowLocked(target_window, result, available_mask);
  AWARN << std::fixed << std::setprecision(6)
        << "[CameraSyncFallback] Use partial camera window for target_window="
        << FormatWindowTimestamp(target_window) << " | target_state="
        << WindowSummaryLocked(target_window);
  EraseWindowLocked(target_window);
}

void TimeWindowSynchronizer::PushCompressed(
    int cam_id, const std::shared_ptr<drivers::CompressedImage>& image) {
  CameraInputData input;
  input.compressed_image = image;
  Push(cam_id, image->header().timestamp_sec(), input);
}

void TimeWindowSynchronizer::PushRaw(
    int cam_id, const std::shared_ptr<drivers::Image>& image) {
  CameraInputData input;
  input.raw_image = image;
  Push(cam_id, image->header().timestamp_sec(), input);
}

void TimeWindowSynchronizer::Push(int cam_id, double timestamp,
                                  const CameraInputData& input) {
  int64_t aligned_timestamp = AlignTimestamp(timestamp);
  std::unique_lock<std::mutex> lock(mutex_);

  WindowState& window = windows_[aligned_timestamp];
  if (!window.slots[cam_id].has_value()) {
    if (0 == window.filled_count) {
      window.first_arrival_tp = std::chrono::steady_clock::now();
    }
    window.slots[cam_id] = input;
    window.raw_timestamps[cam_id] = timestamp;
    window.arrival_wall_times[cam_id] = CurrentUnixSeconds();
    window.aligned_timestamps[cam_id] = aligned_timestamp;
    window.arrival_order[cam_id] = window.filled_count;
    window.filled_count++;
    AINFO << std::fixed << std::setprecision(6)
          << "[CameraSyncArrival] cam=" << cam_id
          << " recv_wall=" << window.arrival_wall_times[cam_id].value_or(-1.0)
          << " raw_ts=" << timestamp
          << " aligned_ts=" << FormatWindowTimestamp(aligned_timestamp)
          << " current_window=" << FormatWindowTimestamp(aligned_timestamp)
          << " filled=" << window.filled_count << "/" << kCameraCount;
  } else {
    AWARN << std::fixed << std::setprecision(6)
          << "[CameraSync] Duplicate camera " << cam_id
          << " for window=" << FormatWindowTimestamp(aligned_timestamp)
          << " incoming_raw_ts=" << timestamp
          << " stored_raw_ts=" << window.raw_timestamps[cam_id].value_or(-1.0)
          << " | target_state=" << WindowSummaryLocked(aligned_timestamp);
  }

  if (window.filled_count > kCameraCount) {
    AERROR << "ERROR OVERFLOW DATA";
  }

  if (kCameraCount == window.filled_count) {
    double fill_ms = ElapsedMs(window.first_arrival_tp);
    if (fill_ms >= kSlowWaitLogMs) {
      AINFO << std::fixed << std::setprecision(3)
            << "[CameraSync] Window filled in " << fill_ms
            << " ms after camera " << cam_id << " arrived"
            << " | target_window=" << FormatWindowTimestamp(aligned_timestamp)
            << " | target_state=" << WindowSummaryLocked(aligned_timestamp);
    }
    condition_variable_.notify_one();
  }

  CleanupOldWindowsLocked(aligned_timestamp);
}

bool TimeWindowSynchronizer::HasCompleteWindowLocked(
    int64_t target_window) const {
  auto it = windows_.find(target_window);
  return windows_.end() != it && kCameraCount == it->second.filled_count;
}

double TimeWindowSynchronizer::ElapsedMs(
    const std::chrono::steady_clock::time_point& start) const {
  if (std::chrono::steady_clock::time_point() == start) {
    return 0.0;
  }
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

double TimeWindowSynchronizer::CurrentUnixSeconds() const {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string TimeWindowSynchronizer::SnapshotLocked() const {
  std::ostringstream oss;
  bool first = true;
  for (const auto& entry : windows_) {
    if (!first) {
      oss << " ";
    }
    first = false;
    const auto& window = entry.second;
    oss << "window{"
        << FormatWindowSnapshotLocked(
               entry.first, window.filled_count,
               ElapsedMs(window.first_arrival_tp), window.slots,
               window.raw_timestamps, window.arrival_wall_times,
               window.aligned_timestamps, window.arrival_order)
        << "}";
  }
  if (first) {
    oss << "windows=empty";
  }
  return oss.str();
}

std::string TimeWindowSynchronizer::WindowSummaryLocked(
    int64_t target_window) const {
  auto it = windows_.find(target_window);
  if (windows_.end() == it) {
    return "missing";
  }
  const auto& window = it->second;
  return FormatWindowSnapshotLocked(
      target_window, window.filled_count, ElapsedMs(window.first_arrival_tp),
      window.slots, window.raw_timestamps, window.arrival_wall_times,
      window.aligned_timestamps, window.arrival_order);
}

double TimeWindowSynchronizer::ArrivalOffsetMsLocked(
    int cam_id,
    const std::array<std::optional<CameraInputData>, kCameraCount>& slots,
    const std::array<std::optional<double>, kCameraCount>& raw_timestamps,
    const std::array<int, kCameraCount>& arrival_order) const {
  if (!slots[cam_id].has_value()) {
    return -1.0;
  }

  double first_raw_ts = -1.0;
  for (int i = 0; i < kCameraCount; ++i) {
    if (!slots[i].has_value()) {
      continue;
    }
    if (0 == arrival_order[i]) {
      first_raw_ts = raw_timestamps[i].value_or(-1.0);
      break;
    }
  }

  if (0.0 > first_raw_ts) {
    return -1.0;
  }

  return (raw_timestamps[cam_id].value_or(first_raw_ts) - first_raw_ts) *
         1000.0;
}

std::string TimeWindowSynchronizer::FormatWindowSnapshotLocked(
    int64_t timestamp, int filled_count, double fill_elapsed_ms,
    const std::array<std::optional<CameraInputData>, kCameraCount>& slots,
    const std::array<std::optional<double>, kCameraCount>& raw_timestamps,
    const std::array<std::optional<double>, kCameraCount>& arrival_wall_times,
    const std::array<std::optional<int64_t>, kCameraCount>& aligned_timestamps,
    const std::array<int, kCameraCount>& arrival_order) const {
  std::ostringstream oss;
  oss << "window=" << FormatWindowTimestamp(timestamp)
      << " filled=" << filled_count << "/" << kCameraCount;
  if (0 <= timestamp) {
    oss << " fill_elapsed_ms=" << std::fixed << std::setprecision(3)
        << fill_elapsed_ms;
  }
  oss << " cams={";
  for (int i = 0; i < kCameraCount; ++i) {
    if (0 != i) {
      oss << ", ";
    }
    oss << i << ":";
    if (!slots[i].has_value()) {
      oss << "missing";
      continue;
    }
    oss << std::fixed << std::setprecision(6)
        << raw_timestamps[i].value_or(-1.0) << "->"
        << FormatWindowTimestamp(aligned_timestamps[i].value_or(-1)) << "#"
        << arrival_order[i] << "@" << std::fixed << std::setprecision(3)
        << ArrivalOffsetMsLocked(i, slots, raw_timestamps, arrival_order)
        << "ms"
        << " wall=" << std::fixed << std::setprecision(6)
        << arrival_wall_times[i].value_or(-1.0);
  }
  oss << "}";
  return oss.str();
}

void TimeWindowSynchronizer::CopyWindowLocked(
    int64_t target_window,
    std::array<CameraInputData, kCameraCount>& result) const {
  const auto& window = windows_.at(target_window);
  for (int i = 0; i < kCameraCount; ++i) {
    result[i] = window.slots[i].value();
  }
}

void TimeWindowSynchronizer::CopyPartialWindowLocked(
    int64_t target_window, std::array<CameraInputData, kCameraCount>& result,
    std::array<bool, kCameraCount>& available_mask) const {
  const auto& window = windows_.at(target_window);
  for (int i = 0; i < kCameraCount; ++i) {
    if (!window.slots[i].has_value()) {
      available_mask[i] = false;
      continue;
    }
    result[i] = window.slots[i].value();
    available_mask[i] = true;
  }
}

void TimeWindowSynchronizer::EraseWindowLocked(int64_t target_window) {
  windows_.erase(target_window);
}

void TimeWindowSynchronizer::CleanupOldWindowsLocked(int64_t newest_window) {
  constexpr int64_t kKeepWindowSpanMs = 300;
  for (auto it = windows_.begin(); it != windows_.end();) {
    if (it->first < newest_window - kKeepWindowSpanMs) {
      AINFO << std::fixed << std::setprecision(3)
            << "[CameraSyncCleanup] drop_window="
            << FormatWindowTimestamp(it->first) << " | state="
            << FormatWindowSnapshotLocked(
                   it->first, it->second.filled_count,
                   ElapsedMs(it->second.first_arrival_tp), it->second.slots,
                   it->second.raw_timestamps, it->second.arrival_wall_times,
                   it->second.aligned_timestamps, it->second.arrival_order);
      it = windows_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
