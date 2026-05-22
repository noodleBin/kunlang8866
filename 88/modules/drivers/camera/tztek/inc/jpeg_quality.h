#pragma once

#include <cstdint>

namespace century {
namespace drivers {
namespace camera {

constexpr int kDefaultJpegQuality = 90;

int ResolveJpegQuality(uint32_t configured_quality);

}  // namespace camera
}  // namespace drivers
}  // namespace century
