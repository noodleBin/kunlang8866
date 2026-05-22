#pragma once

#include <cstdint>

namespace century {
namespace drivers {
namespace camera {

int BytesPerPixelForV4l2PixelFormat(uint32_t pixfmt);
int CalculatePackedImageStride(int width, uint32_t pixfmt);

}  // namespace camera
}  // namespace drivers
}  // namespace century
