#include "v4l2_buffer_config.h"

namespace century {
namespace drivers {
namespace camera {

int ResolveV4l2BufferCount(uint32_t configured_count) {
  return (configured_count >= static_cast<uint32_t>(kMinV4l2BufferCount) &&
          configured_count <= static_cast<uint32_t>(kMaxV4l2BufferCount))
             ? static_cast<int>(configured_count)
             : kDefaultV4l2BufferCount;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
