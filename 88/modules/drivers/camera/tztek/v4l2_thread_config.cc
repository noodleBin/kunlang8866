#include "v4l2_thread_config.h"

#include <sstream>

namespace century {
namespace drivers {
namespace camera {
namespace {

void Warn(const V4l2ThreadWarnFn& warn_fn, const std::string& message) {
  if (warn_fn) {
    warn_fn(message);
  }
}

int ResolvePriority(int value, const V4l2ThreadWarnFn& warn_fn) {
  if (value >= kMinRealtimePriority && value <= kMaxRealtimePriority) {
    return value;
  }
  std::ostringstream oss;
  oss << "invalid v4l2_thread_config.realtime_priority: " << value
      << ", valid range [" << kMinRealtimePriority << ", "
      << kMaxRealtimePriority << "], fallback to "
      << kDefaultRealtimePriority;
  Warn(warn_fn, oss.str());
  return kDefaultRealtimePriority;
}

uint32_t ResolveCoreCount(uint32_t value, const V4l2ThreadWarnFn& warn_fn) {
  if (value >= kMinCpuAffinityCoreCount && value <= kMaxCpuAffinityCoreCount) {
    return value;
  }
  std::ostringstream oss;
  oss << "invalid v4l2_thread_config.cpu_affinity_core_count: " << value
      << ", valid range [" << kMinCpuAffinityCoreCount << ", "
      << kMaxCpuAffinityCoreCount << "], fallback to "
      << kDefaultCpuAffinityCoreCount;
  Warn(warn_fn, oss.str());
  return kDefaultCpuAffinityCoreCount;
}

}  // namespace

V4l2ThreadOptions ResolveV4l2ThreadOptions(
    const V4l2ThreadOptions& configured_options,
    const V4l2ThreadWarnFn& warn_fn) {
  V4l2ThreadOptions resolved;
  resolved.enable_realtime_scheduling =
      configured_options.enable_realtime_scheduling;
  resolved.realtime_priority =
      ResolvePriority(configured_options.realtime_priority, warn_fn);
  resolved.enable_cpu_affinity = configured_options.enable_cpu_affinity;
  resolved.cpu_affinity_core_count =
      ResolveCoreCount(configured_options.cpu_affinity_core_count, warn_fn);

  for (const auto& item : configured_options.channel_cpu_affinities) {
    const uint32_t channel = item.first;
    const uint32_t core = item.second;
    if (channel > kMaxCpuAffinityChannel) {
      std::ostringstream oss;
      oss << "invalid v4l2_thread_config.channel_cpu_affinities.channel: "
          << channel << ", max " << kMaxCpuAffinityChannel
          << ", ignore this entry";
      Warn(warn_fn, oss.str());
      continue;
    }
    if (core > kMaxCpuAffinityCore) {
      std::ostringstream oss;
      oss << "invalid v4l2_thread_config.channel_cpu_affinities.core: "
          << core << ", max " << kMaxCpuAffinityCore
          << ", ignore this entry";
      Warn(warn_fn, oss.str());
      continue;
    }
    resolved.channel_cpu_affinities[channel] = core;
  }
  return resolved;
}

uint32_t ResolveCpuAffinityCore(const V4l2ThreadOptions& options,
                                uint32_t channel) {
  const auto iter = options.channel_cpu_affinities.find(channel);
  if (iter != options.channel_cpu_affinities.end()) {
    return iter->second;
  }
  const uint32_t core_count =
      options.cpu_affinity_core_count >= kMinCpuAffinityCoreCount
          ? options.cpu_affinity_core_count
          : kDefaultCpuAffinityCoreCount;
  return channel % core_count;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
