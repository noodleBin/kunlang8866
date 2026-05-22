#include "v4l2_buffer_config.h"

using century::drivers::camera::ResolveV4l2BufferCount;

int main() {
  if (ResolveV4l2BufferCount(2) != 2) {
    return 1;
  }
  if (ResolveV4l2BufferCount(3) != 3) {
    return 1;
  }
  if (ResolveV4l2BufferCount(4) != 4) {
    return 1;
  }
  if (ResolveV4l2BufferCount(0) != 2) {
    return 1;
  }
  if (ResolveV4l2BufferCount(1) != 2) {
    return 1;
  }
  if (ResolveV4l2BufferCount(5) != 2) {
    return 1;
  }
  return 0;
}
