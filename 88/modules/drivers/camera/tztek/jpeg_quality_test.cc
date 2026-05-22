#include <cassert>
#include "jpeg_quality.h"

using century::drivers::camera::ResolveJpegQuality;

int main() {
  assert(ResolveJpegQuality(90) == 90);
  assert(ResolveJpegQuality(1) == 1);
  assert(ResolveJpegQuality(100) == 100);
  assert(ResolveJpegQuality(0) == 90);
  assert(ResolveJpegQuality(101) == 90);
  return 0;
}
