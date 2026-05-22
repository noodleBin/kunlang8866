#include "enabled_camera_mask.h"

#include <sstream>

namespace century {
namespace drivers {
namespace camera {

uint32_t ResolveEnabledCameraMask(
    uint32_t configured_mask,
    const EnabledCameraMaskWarnFn& warn_fn) {
  if (configured_mask >= kMinEnabledCameraMask &&
      configured_mask <= kMaxEnabledCameraMask) {
    return configured_mask;
  }

  if (warn_fn) {
    std::ostringstream oss;
    oss << "invalid enabled_camera_mask: " << configured_mask
        << ", valid range [" << kMinEnabledCameraMask << ", "
        << kMaxEnabledCameraMask << "], fallback to "
        << kDefaultEnabledCameraMask;
    warn_fn(oss.str());
  }
  return kDefaultEnabledCameraMask;
}

std::vector<int> ExpandEnabledCameraChannels(uint32_t enabled_mask) {
  std::vector<int> channel_indices;
  int bit_position = 0;
  while (enabled_mask != 0U) {
    if ((enabled_mask & 1U) != 0U) {
      channel_indices.push_back(bit_position);
    }
    enabled_mask >>= 1U;
    ++bit_position;
  }
  return channel_indices;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
