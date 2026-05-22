#include "platform_config.h"

#include <algorithm>
#include <cctype>

namespace century {
namespace drivers {
namespace camera {
namespace {

bool IsBlank(const std::string& value) {
  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
}

}  // namespace

std::string ResolveHbJ5devPath(const std::string& platform_horizon_path,
                               const std::string& legacy_hb_j5dev_path) {
  if (!platform_horizon_path.empty()) {
    return platform_horizon_path;
  }
  return legacy_hb_j5dev_path;
}

std::vector<std::string> DefaultCameraPowerValuePaths() {
  return {
      "/sys/class/tz_gpio/camera_0_1-power/value",
      "/sys/class/tz_gpio/camera_2_3-power/value",
      "/sys/class/tz_gpio/camera_4_5-power/value",
      "/sys/class/tz_gpio/camera_6_7-power/value",
  };
}

std::vector<std::string> ResolveCameraPowerValuePaths(
    const std::vector<std::string>& configured_paths,
    const PlatformConfigWarnFn& warn_fn) {
  const std::vector<std::string> defaults = DefaultCameraPowerValuePaths();
  if (configured_paths.empty()) {
    return defaults;
  }

  std::vector<std::string> resolved;
  resolved.reserve(configured_paths.size());
  for (const auto& path : configured_paths) {
    if (!IsBlank(path)) {
      resolved.push_back(path);
    }
  }

  if (resolved.size() == defaults.size()) {
    return resolved;
  }

  if (warn_fn) {
    warn_fn("invalid camera_power_gpio.power_value_paths count: " +
            std::to_string(resolved.size()) + ", expected " +
            std::to_string(defaults.size()) + ", fallback to defaults");
  }
  return defaults;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
