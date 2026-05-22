#include "enabled_camera_mask.h"

#include <cstdint>
#include <string>
#include <vector>

namespace camera = century::drivers::camera;

int ExpectUInt(uint32_t actual, uint32_t expected) {
  return actual == expected ? 0 : 1;
}

int ExpectVector(const std::vector<int>& actual,
                 const std::vector<int>& expected) {
  return actual == expected ? 0 : 1;
}

int main() {
  int failures = 0;
  std::vector<std::string> warnings;
  const auto warn = [&warnings](const std::string& message) {
    warnings.push_back(message);
  };

  failures +=
      ExpectUInt(camera::ResolveEnabledCameraMask(
                     camera::kDefaultEnabledCameraMask, warn),
                 camera::kDefaultEnabledCameraMask);
  failures += ExpectVector(camera::ExpandEnabledCameraChannels(
                               camera::kDefaultEnabledCameraMask),
                           {0, 1, 2, 3, 4, 5});
  failures += ExpectVector(camera::ExpandEnabledCameraChannels(0x03), {0, 1});
  failures +=
      ExpectVector(camera::ExpandEnabledCameraChannels(0xA5), {0, 2, 5, 7});
  failures += warnings.empty() ? 0 : 1;

  failures += ExpectUInt(camera::ResolveEnabledCameraMask(0, warn),
                         camera::kDefaultEnabledCameraMask);
  failures += ExpectUInt(camera::ResolveEnabledCameraMask(256, warn),
                         camera::kDefaultEnabledCameraMask);
  failures += warnings.size() == 2U ? 0 : 1;

  return failures == 0 ? 0 : 1;
}
