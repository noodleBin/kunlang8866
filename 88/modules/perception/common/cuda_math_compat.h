#pragma once

#include <cmath>
#include <math.h>

// CUDA 12.x's libcudacxx expects these C math symbols to be visible in the
// global namespace when thrust/cuda::std pulls in <cmath>.
using std::fpclassify;
using std::isgreater;
using std::isgreaterequal;
using std::isless;
using std::islessequal;
using std::islessgreater;
using std::isnormal;
using std::isunordered;
