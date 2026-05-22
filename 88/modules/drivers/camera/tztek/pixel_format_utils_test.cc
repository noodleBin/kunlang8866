#include "pixel_format_utils.h"

#include <linux/videodev2.h>

namespace camera = century::drivers::camera;

int ExpectInt(int actual, int expected) {
  return actual == expected ? 0 : 1;
}

int main() {
  int failures = 0;

  failures +=
      ExpectInt(camera::BytesPerPixelForV4l2PixelFormat(V4L2_PIX_FMT_YUYV), 2);
  failures +=
      ExpectInt(camera::BytesPerPixelForV4l2PixelFormat(V4L2_PIX_FMT_UYVY), 2);
  failures +=
      ExpectInt(camera::BytesPerPixelForV4l2PixelFormat(V4L2_PIX_FMT_YVYU), 2);
  failures +=
      ExpectInt(camera::BytesPerPixelForV4l2PixelFormat(V4L2_PIX_FMT_VYUY), 2);
  failures += ExpectInt(
      camera::BytesPerPixelForV4l2PixelFormat(V4L2_PIX_FMT_SRGGB12), 2);
  failures += ExpectInt(camera::BytesPerPixelForV4l2PixelFormat(0), 2);

  failures +=
      ExpectInt(camera::CalculatePackedImageStride(1920, V4L2_PIX_FMT_YUYV),
                3840);
  failures +=
      ExpectInt(camera::CalculatePackedImageStride(1920, V4L2_PIX_FMT_UYVY),
                3840);
  failures += ExpectInt(
      camera::CalculatePackedImageStride(1920, V4L2_PIX_FMT_SRGGB12), 3840);
  failures +=
      ExpectInt(camera::CalculatePackedImageStride(0, V4L2_PIX_FMT_YUYV), 0);
  failures +=
      ExpectInt(camera::CalculatePackedImageStride(-1, V4L2_PIX_FMT_YUYV), 0);

  return failures == 0 ? 0 : 1;
}
