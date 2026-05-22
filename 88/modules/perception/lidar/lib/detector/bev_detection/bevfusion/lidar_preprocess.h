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

#ifndef BEVFUSION_LIDAR_PREPROCESS_CUH
#define BEVFUSION_LIDAR_PREPROCESS_CUH

#include <cuda_fp16.h>
#include <cuda_runtime.h>

namespace bevfusion {
namespace lidar {

// Convert PointF data to half and reorder it for BEVFusion.
__global__ void ConvertPointsToHalfKernel(const float* points, half* out_half,
                                          int num_points);

// Launch the point conversion kernel on the provided stream.
void LaunchConvertPointsToHalf(const float* points, half* out_half,
                               int num_points, cudaStream_t stream);

// Vectorized variant using half2 stores.
__global__ void ConvertPointsToHalfKernelVectorized(const float* points,
                                                    half* out_half,
                                                    int num_points);

}  // namespace lidar
}  // namespace bevfusion

#endif  // BEVFUSION_LIDAR_PREPROCESS_CUH
