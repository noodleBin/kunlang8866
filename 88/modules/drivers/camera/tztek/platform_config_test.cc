#include <cassert>
#include <string>
#include <vector>

#include "platform_config.h"

using century::drivers::camera::DefaultCameraPowerValuePaths;
using century::drivers::camera::ResolveHbJ5devPath;
using century::drivers::camera::ResolveCameraPowerValuePaths;

int main() {
  assert(ResolveHbJ5devPath("/new/path.json", "/old/path.json") == "/new/path.json");
  assert(ResolveHbJ5devPath("", "/old/path.json") == "/old/path.json");
  assert(ResolveHbJ5devPath("/new/path.json", "") == "/new/path.json");
  assert(ResolveHbJ5devPath("", "") == "");

  const std::vector<std::string> defaults = DefaultCameraPowerValuePaths();
  assert(defaults.size() == 4U);
  assert(defaults[0] == "/sys/class/tz_gpio/camera_0_1-power/value");
  assert(defaults[1] == "/sys/class/tz_gpio/camera_2_3-power/value");
  assert(defaults[2] == "/sys/class/tz_gpio/camera_4_5-power/value");
  assert(defaults[3] == "/sys/class/tz_gpio/camera_6_7-power/value");

  assert(ResolveCameraPowerValuePaths({}) == defaults);

  const std::vector<std::string> configured = ResolveCameraPowerValuePaths({
      "/tmp/camera_power0",
      "/tmp/camera_power1",
      "/tmp/camera_power2",
      "/tmp/camera_power3",
  });
  assert(configured.size() == 4U);
  assert(configured[0] == "/tmp/camera_power0");
  assert(configured[3] == "/tmp/camera_power3");

  std::vector<std::string> warnings;
  const auto warn = [&warnings](const std::string& message) {
    warnings.push_back(message);
  };
  assert(ResolveCameraPowerValuePaths({"", "  "}, warn) == defaults);
  assert(warnings.size() == 1U);
  warnings.clear();
  assert(ResolveCameraPowerValuePaths({"/tmp/only_one"}, warn) == defaults);
  assert(warnings.size() == 1U);
  return 0;
}
