#include "pixel_format_utils.h"

#include <linux/videodev2.h>

namespace century {
namespace drivers {
namespace camera {

int BytesPerPixelForV4l2PixelFormat(uint32_t pixfmt) {
  switch (pixfmt) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_VYUY:
    case V4L2_PIX_FMT_SRGGB12:
    default:
      return 2;
  }
}

int CalculatePackedImageStride(int width, uint32_t pixfmt) {
  if (width <= 0) {
    return 0;
  }
  return width * BytesPerPixelForV4l2PixelFormat(pixfmt);
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
