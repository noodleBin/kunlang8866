#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace century {
namespace drivers {
namespace camera {

constexpr uint32_t kDefaultEnabledCameraMask = 0x3F;
constexpr uint32_t kMinEnabledCameraMask = 1;
constexpr uint32_t kMaxEnabledCameraMask = 0xFF;

using EnabledCameraMaskWarnFn = std::function<void(const std::string&)>;

uint32_t ResolveEnabledCameraMask(
    uint32_t configured_mask,
    const EnabledCameraMaskWarnFn& warn_fn = EnabledCameraMaskWarnFn());

std::vector<int> ExpandEnabledCameraChannels(uint32_t enabled_mask);

}  // namespace camera
}  // namespace drivers
}  // namespace century
