#include "jpeg_quality.h"

namespace century {
namespace drivers {
namespace camera {

int ResolveJpegQuality(uint32_t configured_quality) {
  return (configured_quality >= 1U && configured_quality <= 100U)
             ? static_cast<int>(configured_quality)
             : kDefaultJpegQuality;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
