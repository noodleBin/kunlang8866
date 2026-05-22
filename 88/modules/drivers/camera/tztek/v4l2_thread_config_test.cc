#include "v4l2_thread_config.h"

#include <cstdint>
#include <string>
#include <vector>

namespace camera = century::drivers::camera;

int ExpectBool(bool actual, bool expected) {
  return actual == expected ? 0 : 1;
}

int ExpectInt(int actual, int expected) {
  return actual == expected ? 0 : 1;
}

int ExpectUInt(uint32_t actual, uint32_t expected) {
  return actual == expected ? 0 : 1;
}

int main() {
  int failures = 0;

  camera::V4l2ThreadOptions defaults;
  failures += ExpectBool(defaults.enable_realtime_scheduling, true);
  failures += ExpectInt(defaults.realtime_priority,
                        camera::kDefaultRealtimePriority);
  failures += ExpectBool(defaults.enable_cpu_affinity, true);
  failures += ExpectUInt(defaults.cpu_affinity_core_count,
                         camera::kDefaultCpuAffinityCoreCount);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(defaults, 0), 0);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(defaults, 5), 5);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(defaults, 6), 0);
  camera::V4l2ThreadOptions unvalidated_zero_core_count;
  unvalidated_zero_core_count.cpu_affinity_core_count = 0;
  failures += ExpectUInt(
      camera::ResolveCpuAffinityCore(unvalidated_zero_core_count, 7), 1);

  std::vector<std::string> warnings;
  auto warn = [&warnings](const std::string& message) {
    warnings.push_back(message);
  };

  camera::V4l2ThreadOptions valid;
  valid.enable_realtime_scheduling = false;
  valid.realtime_priority = 1;
  valid.enable_cpu_affinity = false;
  valid.cpu_affinity_core_count = 256;
  valid.channel_cpu_affinities[0] = 2;
  valid.channel_cpu_affinities[7] = 3;
  camera::V4l2ThreadOptions resolved =
      camera::ResolveV4l2ThreadOptions(valid, warn);
  failures += ExpectBool(resolved.enable_realtime_scheduling, false);
  failures += ExpectInt(resolved.realtime_priority, 1);
  failures += ExpectBool(resolved.enable_cpu_affinity, false);
  failures += ExpectUInt(resolved.cpu_affinity_core_count, 256);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(resolved, 0), 2);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(resolved, 7), 3);
  failures += warnings.empty() ? 0 : 1;

  camera::V4l2ThreadOptions invalid;
  invalid.realtime_priority = 0;
  invalid.cpu_affinity_core_count = 0;
  invalid.channel_cpu_affinities[256] = 1;
  invalid.channel_cpu_affinities[1] = 256;
  warnings.clear();
  resolved = camera::ResolveV4l2ThreadOptions(invalid, warn);
  failures += ExpectInt(resolved.realtime_priority,
                        camera::kDefaultRealtimePriority);
  failures += ExpectUInt(resolved.cpu_affinity_core_count,
                         camera::kDefaultCpuAffinityCoreCount);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(resolved, 1), 1);
  failures += warnings.size() == 4 ? 0 : 1;

  camera::V4l2ThreadOptions duplicate;
  duplicate.channel_cpu_affinities[3] = 4;
  duplicate.channel_cpu_affinities[3] = 5;
  resolved = camera::ResolveV4l2ThreadOptions(duplicate);
  failures += ExpectUInt(camera::ResolveCpuAffinityCore(resolved, 3), 5);

  return failures == 0 ? 0 : 1;
}
