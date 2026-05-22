#pragma once

#include <cstdint>

namespace century {
namespace drivers {
namespace camera {

constexpr int kDefaultV4l2BufferCount = 2;
constexpr int kMinV4l2BufferCount = 2;
constexpr int kMaxV4l2BufferCount = 4;

int ResolveV4l2BufferCount(uint32_t configured_count);

}  // namespace camera
}  // namespace drivers
}  // namespace century
