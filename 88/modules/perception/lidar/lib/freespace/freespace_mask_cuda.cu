/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lidar/lib/freespace/freespace_mask_cuda.h"

#include <cmath>
#include <cstdint>

#include <cuda_runtime.h>

namespace century {
namespace perception {
namespace lidar {

namespace {
constexpr double kPi = 3.14159265358979323846;

struct DeviceRayCastInput {
  const uint8_t* occupied = nullptr;
  const uint8_t* map_allowed = nullptr;
  int width = 0;
  int height = 0;
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double resolution = 0.0;
  double self_x_min = 0.0;
  double self_x_max = 0.0;
  double self_y_min = 0.0;
  double self_y_max = 0.0;
  double source_x = 0.0;
  double source_y = 0.0;
  double max_range = 0.0;
  int ray_count = 0;
};

__device__ int ToGridIndex(const int gx, const int gy, const int width) {
  return gy * width + gx;
}

__device__ bool IsInsideVehicle(const DeviceRayCastInput input,
                                const double x, const double y) {
  return x >= input.self_x_min && x <= input.self_x_max &&
         y >= input.self_y_min && y <= input.self_y_max;
}

__device__ void CellCenter(const DeviceRayCastInput input, const int gx,
                           const int gy, double* x, double* y) {
  *x = input.x_min + (static_cast<double>(gx) + 0.5) * input.resolution;
  *y = input.y_min + (static_cast<double>(gy) + 0.5) * input.resolution;
}

__device__ bool UpdateAxis(const double origin, const double direction,
                           const double min_value, const double max_value,
                           double* near_t, double* far_t) {
  constexpr double kDirectionEpsilon = 1e-12;
  if (fabs(direction) < kDirectionEpsilon) {
    return origin >= min_value && origin <= max_value;
  }

  double axis_enter = (min_value - origin) / direction;
  double axis_exit = (max_value - origin) / direction;
  if (axis_enter > axis_exit) {
    const double tmp = axis_enter;
    axis_enter = axis_exit;
    axis_exit = tmp;
  }
  *near_t = fmax(*near_t, axis_enter);
  *far_t = fmin(*far_t, axis_exit);
  return *near_t <= *far_t;
}

__device__ bool ComputeRayGridRange(const DeviceRayCastInput input,
                                    const double dir_x, const double dir_y,
                                    double* t_enter, double* t_exit) {
  double near_t = 0.0;
  double far_t = INFINITY;
  if (!UpdateAxis(input.source_x, dir_x, input.x_min, input.x_max, &near_t,
                  &far_t) ||
      !UpdateAxis(input.source_y, dir_y, input.y_min, input.y_max, &near_t,
                  &far_t) ||
      far_t < 0.0) {
    return false;
  }
  *t_enter = near_t;
  *t_exit = far_t;
  return true;
}

__global__ void CastFreespaceRaysKernel(DeviceRayCastInput input,
                                        double* ray_ranges,
                                        uint8_t* ray_valid,
                                        uint8_t* ray_hit_occupied,
                                        double* ray_hit_occ_x,
                                        double* ray_hit_occ_y) {
  const int ray_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (ray_id >= input.ray_count) {
    return;
  }

  const double angular_step = 2.0 * kPi / static_cast<double>(input.ray_count);
  const double theta = -kPi + static_cast<double>(ray_id) * angular_step;
  double sin_theta = 0.0;
  double cos_theta = 0.0;
  sincos(theta, &sin_theta, &cos_theta);
  double t_enter = 0.0;
  double t_exit = 0.0;
  if (!ComputeRayGridRange(input, cos_theta, sin_theta, &t_enter, &t_exit)) {
    return;
  }

  constexpr double kRayEntryEpsilonScale = 1e-3;
  double travel_t = fmax(0.0, t_enter);
  if (t_enter > 0.0) {
    travel_t += input.resolution * kRayEntryEpsilonScale;
  }
  double x = input.source_x + travel_t * cos_theta;
  double y = input.source_y + travel_t * sin_theta;
  x = fmin(input.x_max - 1e-9, fmax(input.x_min, x));
  y = fmin(input.y_max - 1e-9, fmax(input.y_min, y));

  int gx = static_cast<int>(floor((x - input.x_min) / input.resolution));
  int gy = static_cast<int>(floor((y - input.y_min) / input.resolution));
  if (gx < 0 || gx >= input.width || gy < 0 || gy >= input.height) {
    return;
  }

  const int step_x = (0.0 < cos_theta) - (cos_theta < 0.0);
  const int step_y = (0.0 < sin_theta) - (sin_theta < 0.0);
  const double next_boundary_x =
      0 < step_x ? input.x_min + static_cast<double>(gx + 1) * input.resolution
                 : input.x_min + static_cast<double>(gx) * input.resolution;
  const double next_boundary_y =
      0 < step_y ? input.y_min + static_cast<double>(gy + 1) * input.resolution
                 : input.y_min + static_cast<double>(gy) * input.resolution;
  const double t_delta_x =
      0 == step_x ? INFINITY : input.resolution / fabs(cos_theta);
  const double t_delta_y =
      0 == step_y ? INFINITY : input.resolution / fabs(sin_theta);
  double t_max_x =
      0 == step_x ? INFINITY : (next_boundary_x - input.source_x) / cos_theta;
  double t_max_y =
      0 == step_y ? INFINITY : (next_boundary_y - input.source_y) / sin_theta;

  bool exited_vehicle = false;
  double last_free_r = -1.0;
  while (travel_t <= t_exit && gx >= 0 && gx < input.width && gy >= 0 &&
         gy < input.height) {
    double cell_center_x = 0.0;
    double cell_center_y = 0.0;
    CellCenter(input, gx, gy, &cell_center_x, &cell_center_y);
    const int idx = ToGridIndex(gx, gy, input.width);
    if (!IsInsideVehicle(input, cell_center_x, cell_center_y)) {
      exited_vehicle = true;
      if (0 == input.map_allowed[idx] || 0 != input.occupied[idx]) {
        if (0 != input.occupied[idx]) {
          ray_hit_occupied[ray_id] = 1;
          ray_hit_occ_x[ray_id] = cell_center_x;
          ray_hit_occ_y[ray_id] = cell_center_y;
        }
        break;
      }
    }

    const double cell_exit_t = fmin(fmin(t_max_x, t_max_y), t_exit);
    if (exited_vehicle) {
      last_free_r = cell_exit_t;
    }

    if (t_max_x < t_max_y) {
      gx += step_x;
      travel_t = t_max_x;
      t_max_x += t_delta_x;
    } else if (t_max_y < t_max_x) {
      gy += step_y;
      travel_t = t_max_y;
      t_max_y += t_delta_y;
    } else {
      gx += step_x;
      gy += step_y;
      travel_t = t_max_x;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    }
  }

  if (last_free_r > 0.0) {
    ray_ranges[ray_id] = fmin(last_free_r, input.max_range);
    ray_valid[ray_id] = 1;
  }
}

bool CheckCuda(cudaError_t error) {
  return error == cudaSuccess;
}

}  // namespace

bool CastFreespaceRaysCuda(const FreespaceCudaRayCastInput& input,
                           double* ray_ranges,
                           uint8_t* ray_valid,
                           uint8_t* ray_hit_occupied,
                           double* ray_hit_occ_x,
                           double* ray_hit_occ_y) {
  if (nullptr == input.occupied || nullptr == input.map_allowed ||
      nullptr == ray_ranges || nullptr == ray_valid ||
      nullptr == ray_hit_occupied || nullptr == ray_hit_occ_x ||
      nullptr == ray_hit_occ_y || input.width <= 0 || input.height <= 0 ||
      input.ray_count <= 0) {
    return false;
  }

  const size_t grid_bytes =
      static_cast<size_t>(input.width) * static_cast<size_t>(input.height) *
      sizeof(uint8_t);
  const size_t ray_double_bytes =
      static_cast<size_t>(input.ray_count) * sizeof(double);
  const size_t ray_byte_bytes =
      static_cast<size_t>(input.ray_count) * sizeof(uint8_t);

  uint8_t* d_occupied = nullptr;
  uint8_t* d_map_allowed = nullptr;
  double* d_ray_ranges = nullptr;
  uint8_t* d_ray_valid = nullptr;
  uint8_t* d_ray_hit_occupied = nullptr;
  double* d_ray_hit_occ_x = nullptr;
  double* d_ray_hit_occ_y = nullptr;

  bool ok = CheckCuda(cudaMalloc(&d_occupied, grid_bytes)) &&
            CheckCuda(cudaMalloc(&d_map_allowed, grid_bytes)) &&
            CheckCuda(cudaMalloc(&d_ray_ranges, ray_double_bytes)) &&
            CheckCuda(cudaMalloc(&d_ray_valid, ray_byte_bytes)) &&
            CheckCuda(cudaMalloc(&d_ray_hit_occupied, ray_byte_bytes)) &&
            CheckCuda(cudaMalloc(&d_ray_hit_occ_x, ray_double_bytes)) &&
            CheckCuda(cudaMalloc(&d_ray_hit_occ_y, ray_double_bytes));
  if (!ok) {
    cudaFree(d_occupied);
    cudaFree(d_map_allowed);
    cudaFree(d_ray_ranges);
    cudaFree(d_ray_valid);
    cudaFree(d_ray_hit_occupied);
    cudaFree(d_ray_hit_occ_x);
    cudaFree(d_ray_hit_occ_y);
    return false;
  }

  ok = CheckCuda(cudaMemcpy(d_occupied, input.occupied, grid_bytes,
                            cudaMemcpyHostToDevice)) &&
       CheckCuda(cudaMemcpy(d_map_allowed, input.map_allowed, grid_bytes,
                            cudaMemcpyHostToDevice)) &&
       CheckCuda(cudaMemset(d_ray_ranges, 0, ray_double_bytes)) &&
       CheckCuda(cudaMemset(d_ray_valid, 0, ray_byte_bytes)) &&
       CheckCuda(cudaMemset(d_ray_hit_occupied, 0, ray_byte_bytes)) &&
       CheckCuda(cudaMemset(d_ray_hit_occ_x, 0, ray_double_bytes)) &&
       CheckCuda(cudaMemset(d_ray_hit_occ_y, 0, ray_double_bytes));
  if (!ok) {
    cudaFree(d_occupied);
    cudaFree(d_map_allowed);
    cudaFree(d_ray_ranges);
    cudaFree(d_ray_valid);
    cudaFree(d_ray_hit_occupied);
    cudaFree(d_ray_hit_occ_x);
    cudaFree(d_ray_hit_occ_y);
    return false;
  }

  DeviceRayCastInput device_input;
  device_input.occupied = d_occupied;
  device_input.map_allowed = d_map_allowed;
  device_input.width = input.width;
  device_input.height = input.height;
  device_input.x_min = input.x_min;
  device_input.x_max = input.x_max;
  device_input.y_min = input.y_min;
  device_input.y_max = input.y_max;
  device_input.resolution = input.resolution;
  device_input.self_x_min = input.self_x_min;
  device_input.self_x_max = input.self_x_max;
  device_input.self_y_min = input.self_y_min;
  device_input.self_y_max = input.self_y_max;
  device_input.source_x = input.source_x;
  device_input.source_y = input.source_y;
  device_input.max_range = input.max_range;
  device_input.ray_count = input.ray_count;

  constexpr int kThreads = 128;
  const int blocks = (input.ray_count + kThreads - 1) / kThreads;
  CastFreespaceRaysKernel<<<blocks, kThreads>>>(
      device_input, d_ray_ranges, d_ray_valid, d_ray_hit_occupied,
      d_ray_hit_occ_x, d_ray_hit_occ_y);
  ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize()) &&
       CheckCuda(cudaMemcpy(ray_ranges, d_ray_ranges, ray_double_bytes,
                            cudaMemcpyDeviceToHost)) &&
       CheckCuda(cudaMemcpy(ray_valid, d_ray_valid, ray_byte_bytes,
                            cudaMemcpyDeviceToHost)) &&
       CheckCuda(cudaMemcpy(ray_hit_occupied, d_ray_hit_occupied,
                            ray_byte_bytes, cudaMemcpyDeviceToHost)) &&
       CheckCuda(cudaMemcpy(ray_hit_occ_x, d_ray_hit_occ_x, ray_double_bytes,
                            cudaMemcpyDeviceToHost)) &&
       CheckCuda(cudaMemcpy(ray_hit_occ_y, d_ray_hit_occ_y, ray_double_bytes,
                            cudaMemcpyDeviceToHost));

  cudaFree(d_occupied);
  cudaFree(d_map_allowed);
  cudaFree(d_ray_ranges);
  cudaFree(d_ray_valid);
  cudaFree(d_ray_hit_occupied);
  cudaFree(d_ray_hit_occ_x);
  cudaFree(d_ray_hit_occ_y);
  return ok;
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
