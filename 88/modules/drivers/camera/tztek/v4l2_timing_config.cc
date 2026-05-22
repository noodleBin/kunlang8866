#include "v4l2_timing_config.h"

#include <sstream>

namespace century {
namespace drivers {
namespace camera {
namespace {

uint32_t ResolveRange(const char* name, uint32_t value, uint32_t min_value,
                      uint32_t max_value, uint32_t default_value,
                      const V4l2TimingWarnFn& warn_fn) {
  if (value >= min_value && value <= max_value) {
    return value;
  }
  if (warn_fn) {
    std::ostringstream oss;
    oss << "invalid " << name << ": " << value << ", valid range ["
        << min_value << ", " << max_value << "], fallback to "
        << default_value;
    warn_fn(oss.str());
  }
  return default_value;
}

}  // namespace

V4l2TimingOptions ResolveV4l2TimingOptions(
    const V4l2TimingOptions& configured_options,
    const V4l2TimingWarnFn& warn_fn) {
  V4l2TimingOptions resolved;
  resolved.init_device_delay_ms = ResolveRange(
      "v4l2_timing_config.init_device_delay_ms",
      configured_options.init_device_delay_ms, kMinInitDeviceDelayMs,
      kMaxInitDeviceDelayMs, kDefaultInitDeviceDelayMs, warn_fn);
  resolved.clear_buffer_timeout_ms = ResolveRange(
      "v4l2_timing_config.clear_buffer_timeout_ms",
      configured_options.clear_buffer_timeout_ms, kMinClearBufferTimeoutMs,
      kMaxClearBufferTimeoutMs, kDefaultClearBufferTimeoutMs, warn_fn);
  resolved.first_capture_delay_ms = ResolveRange(
      "v4l2_timing_config.first_capture_delay_ms",
      configured_options.first_capture_delay_ms, kMinFirstCaptureDelayMs,
      kMaxFirstCaptureDelayMs, kDefaultFirstCaptureDelayMs, warn_fn);
  resolved.start_capture_retry_delay_ms = ResolveRange(
      "v4l2_timing_config.start_capture_retry_delay_ms",
      configured_options.start_capture_retry_delay_ms,
      kMinStartCaptureRetryDelayMs, kMaxStartCaptureRetryDelayMs,
      kDefaultStartCaptureRetryDelayMs, warn_fn);
  resolved.poll_min_timeout_ms = ResolveRange(
      "v4l2_timing_config.poll_min_timeout_ms",
      configured_options.poll_min_timeout_ms, kMinPollMinTimeoutMs,
      kMaxPollMinTimeoutMs, kDefaultPollMinTimeoutMs, warn_fn);
  resolved.poll_max_timeout_ms = ResolveRange(
      "v4l2_timing_config.poll_max_timeout_ms",
      configured_options.poll_max_timeout_ms, kMinPollMaxTimeoutMs,
      kMaxPollMaxTimeoutMs, kDefaultPollMaxTimeoutMs, warn_fn);
  resolved.grab_failure_retry_delay_ms = ResolveRange(
      "v4l2_timing_config.grab_failure_retry_delay_ms",
      configured_options.grab_failure_retry_delay_ms,
      kMinGrabFailureRetryDelayMs, kMaxGrabFailureRetryDelayMs,
      kDefaultGrabFailureRetryDelayMs, warn_fn);

  if (resolved.poll_min_timeout_ms > resolved.poll_max_timeout_ms) {
    if (warn_fn) {
      std::ostringstream oss;
      oss << "invalid v4l2_timing_config poll timeout range: min "
          << resolved.poll_min_timeout_ms << " > max "
          << resolved.poll_max_timeout_ms << ", fallback to default "
          << kDefaultPollMinTimeoutMs << "/" << kDefaultPollMaxTimeoutMs;
      warn_fn(oss.str());
    }
    resolved.poll_min_timeout_ms = kDefaultPollMinTimeoutMs;
    resolved.poll_max_timeout_ms = kDefaultPollMaxTimeoutMs;
  }

  return resolved;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
