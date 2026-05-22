/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lidar_preprocess.h"

namespace bevfusion {
namespace lidar {

__global__ void ConvertPointsToHalfKernel(const float* points, half* out_half,
                                          int num_points) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  int input_idx = idx * 4;
  int output_idx = idx * 4;

  float x = points[input_idx];
  float y = points[input_idx + 1];
  float z = points[input_idx + 2];
  float intensity = points[input_idx + 3];

  out_half[output_idx] = __float2half_rn(x);
  out_half[output_idx + 1] = __float2half_rn(y);
  out_half[output_idx + 2] = __float2half_rn(z);
  out_half[output_idx + 3] = __float2half_rn(intensity);
}

__global__ void ConvertPointsToHalfKernelVectorized(const float* points,
                                                    half* out_half,
                                                    int num_points) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  int input_idx = idx * 4;

  float x = points[input_idx];
  float y = points[input_idx + 1];
  float z = points[input_idx + 2];
  float intensity = points[input_idx + 3];

  half2* out_half2 = reinterpret_cast<half2*>(out_half + idx * 4);

  out_half2[0] = __floats2half2_rn(x, y);
  out_half2[1] = __floats2half2_rn(z, intensity);
}

void LaunchConvertPointsToHalf(const float* points, half* out_half,
                               int num_points, cudaStream_t stream) {
  const int threads = 256;
  const int blocks = (num_points + threads - 1) / threads;

  ConvertPointsToHalfKernelVectorized<<<blocks, threads, 0, stream>>>(
      points, out_half, num_points);
}

}  // namespace lidar
}  // namespace bevfusion
