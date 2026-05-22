#include "v4l2_timing_config.h"

#include <cstdint>
#include <string>
#include <vector>

namespace camera = century::drivers::camera;

int ExpectEqual(uint32_t actual, uint32_t expected) {
  return actual == expected ? 0 : 1;
}

int main() {
  int failures = 0;

  camera::V4l2TimingOptions defaults;
  failures += ExpectEqual(defaults.init_device_delay_ms,
                          camera::kDefaultInitDeviceDelayMs);
  failures += ExpectEqual(defaults.clear_buffer_timeout_ms,
                          camera::kDefaultClearBufferTimeoutMs);
  failures += ExpectEqual(defaults.first_capture_delay_ms,
                          camera::kDefaultFirstCaptureDelayMs);
  failures += ExpectEqual(defaults.start_capture_retry_delay_ms,
                          camera::kDefaultStartCaptureRetryDelayMs);
  failures += ExpectEqual(defaults.poll_min_timeout_ms,
                          camera::kDefaultPollMinTimeoutMs);
  failures += ExpectEqual(defaults.poll_max_timeout_ms,
                          camera::kDefaultPollMaxTimeoutMs);
  failures += ExpectEqual(defaults.grab_failure_retry_delay_ms,
                          camera::kDefaultGrabFailureRetryDelayMs);

  std::vector<std::string> warnings;
  auto warn = [&warnings](const std::string& message) {
    warnings.push_back(message);
  };

  camera::V4l2TimingOptions valid;
  valid.init_device_delay_ms = 0;
  valid.clear_buffer_timeout_ms = 1;
  valid.first_capture_delay_ms = 1000;
  valid.start_capture_retry_delay_ms = 1000;
  valid.poll_min_timeout_ms = 1;
  valid.poll_max_timeout_ms = 5000;
  valid.grab_failure_retry_delay_ms = 1;
  camera::V4l2TimingOptions resolved =
      camera::ResolveV4l2TimingOptions(valid, warn);
  failures += ExpectEqual(resolved.init_device_delay_ms, 0);
  failures += ExpectEqual(resolved.clear_buffer_timeout_ms, 1);
  failures += ExpectEqual(resolved.first_capture_delay_ms, 1000);
  failures += ExpectEqual(resolved.start_capture_retry_delay_ms, 1000);
  failures += ExpectEqual(resolved.poll_min_timeout_ms, 1);
  failures += ExpectEqual(resolved.poll_max_timeout_ms, 5000);
  failures += ExpectEqual(resolved.grab_failure_retry_delay_ms, 1);
  failures += warnings.empty() ? 0 : 1;

  camera::V4l2TimingOptions invalid;
  invalid.init_device_delay_ms = 5001;
  invalid.clear_buffer_timeout_ms = 0;
  invalid.first_capture_delay_ms = 1001;
  invalid.start_capture_retry_delay_ms = 1001;
  invalid.poll_min_timeout_ms = 0;
  invalid.poll_max_timeout_ms = 5001;
  invalid.grab_failure_retry_delay_ms = 0;
  warnings.clear();
  resolved = camera::ResolveV4l2TimingOptions(invalid, warn);
  failures += ExpectEqual(resolved.init_device_delay_ms,
                          camera::kDefaultInitDeviceDelayMs);
  failures += ExpectEqual(resolved.clear_buffer_timeout_ms,
                          camera::kDefaultClearBufferTimeoutMs);
  failures += ExpectEqual(resolved.first_capture_delay_ms,
                          camera::kDefaultFirstCaptureDelayMs);
  failures += ExpectEqual(resolved.start_capture_retry_delay_ms,
                          camera::kDefaultStartCaptureRetryDelayMs);
  failures += ExpectEqual(resolved.poll_min_timeout_ms,
                          camera::kDefaultPollMinTimeoutMs);
  failures += ExpectEqual(resolved.poll_max_timeout_ms,
                          camera::kDefaultPollMaxTimeoutMs);
  failures += ExpectEqual(resolved.grab_failure_retry_delay_ms,
                          camera::kDefaultGrabFailureRetryDelayMs);
  failures += warnings.size() == 7 ? 0 : 1;

  camera::V4l2TimingOptions inverted_poll;
  inverted_poll.poll_min_timeout_ms = 600;
  inverted_poll.poll_max_timeout_ms = 100;
  warnings.clear();
  resolved = camera::ResolveV4l2TimingOptions(inverted_poll, warn);
  failures += ExpectEqual(resolved.poll_min_timeout_ms,
                          camera::kDefaultPollMinTimeoutMs);
  failures += ExpectEqual(resolved.poll_max_timeout_ms,
                          camera::kDefaultPollMaxTimeoutMs);
  failures += warnings.size() == 1 ? 0 : 1;

  return failures == 0 ? 0 : 1;
}
