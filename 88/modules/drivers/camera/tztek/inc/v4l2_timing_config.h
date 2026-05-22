#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace century {
namespace drivers {
namespace camera {

constexpr uint32_t kDefaultInitDeviceDelayMs = 1000;
constexpr uint32_t kDefaultClearBufferTimeoutMs = 100;
constexpr uint32_t kDefaultFirstCaptureDelayMs = 5;
constexpr uint32_t kDefaultStartCaptureRetryDelayMs = 20;
constexpr uint32_t kDefaultPollMinTimeoutMs = 50;
constexpr uint32_t kDefaultPollMaxTimeoutMs = 500;
constexpr uint32_t kDefaultGrabFailureRetryDelayMs = 5;

constexpr uint32_t kMinInitDeviceDelayMs = 0;
constexpr uint32_t kMaxInitDeviceDelayMs = 5000;
constexpr uint32_t kMinClearBufferTimeoutMs = 1;
constexpr uint32_t kMaxClearBufferTimeoutMs = 1000;
constexpr uint32_t kMinFirstCaptureDelayMs = 0;
constexpr uint32_t kMaxFirstCaptureDelayMs = 1000;
constexpr uint32_t kMinStartCaptureRetryDelayMs = 0;
constexpr uint32_t kMaxStartCaptureRetryDelayMs = 1000;
constexpr uint32_t kMinPollMinTimeoutMs = 1;
constexpr uint32_t kMaxPollMinTimeoutMs = 1000;
constexpr uint32_t kMinPollMaxTimeoutMs = 1;
constexpr uint32_t kMaxPollMaxTimeoutMs = 5000;
constexpr uint32_t kMinGrabFailureRetryDelayMs = 1;
constexpr uint32_t kMaxGrabFailureRetryDelayMs = 1000;

struct V4l2TimingOptions {
  uint32_t init_device_delay_ms = kDefaultInitDeviceDelayMs;
  uint32_t clear_buffer_timeout_ms = kDefaultClearBufferTimeoutMs;
  uint32_t first_capture_delay_ms = kDefaultFirstCaptureDelayMs;
  uint32_t start_capture_retry_delay_ms = kDefaultStartCaptureRetryDelayMs;
  uint32_t poll_min_timeout_ms = kDefaultPollMinTimeoutMs;
  uint32_t poll_max_timeout_ms = kDefaultPollMaxTimeoutMs;
  uint32_t grab_failure_retry_delay_ms = kDefaultGrabFailureRetryDelayMs;
};

using V4l2TimingWarnFn = std::function<void(const std::string&)>;

V4l2TimingOptions ResolveV4l2TimingOptions(
    const V4l2TimingOptions& configured_options,
    const V4l2TimingWarnFn& warn_fn = V4l2TimingWarnFn());

}  // namespace camera
}  // namespace drivers
}  // namespace century
