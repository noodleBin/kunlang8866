#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace century {
namespace drivers {
namespace camera {

constexpr int kDefaultRealtimePriority = 90;
constexpr int kMinRealtimePriority = 1;
constexpr int kMaxRealtimePriority = 99;
constexpr uint32_t kDefaultCpuAffinityCoreCount = 6;
constexpr uint32_t kMinCpuAffinityCoreCount = 1;
constexpr uint32_t kMaxCpuAffinityCoreCount = 256;
constexpr uint32_t kMaxCpuAffinityChannel = 255;
constexpr uint32_t kMaxCpuAffinityCore = 255;

struct V4l2ThreadOptions {
  bool enable_realtime_scheduling = true;
  int realtime_priority = kDefaultRealtimePriority;
  bool enable_cpu_affinity = true;
  uint32_t cpu_affinity_core_count = kDefaultCpuAffinityCoreCount;
  std::map<uint32_t, uint32_t> channel_cpu_affinities;
};

using V4l2ThreadWarnFn = std::function<void(const std::string&)>;

V4l2ThreadOptions ResolveV4l2ThreadOptions(
    const V4l2ThreadOptions& configured_options,
    const V4l2ThreadWarnFn& warn_fn = V4l2ThreadWarnFn());

uint32_t ResolveCpuAffinityCore(const V4l2ThreadOptions& options,
                                uint32_t channel);

}  // namespace camera
}  // namespace drivers
}  // namespace century
