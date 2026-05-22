/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
#include <cuda_fp16.h>

#include "modules/perception/common/cuda_math_compat.h"
#include <thrust/sort.h>

#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-geometry.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/launch.cuh"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensor.hpp"

namespace bevfusion {
namespace camera {

struct GeometryParameterExtra : public GeometryParameter {
  unsigned int D;
  nvtype::Float3 dx;
  nvtype::Float3 bx;
  nvtype::Int3 nx;
};

static __forceinline__ __device__ float dot(const float4& T, const float3& p) {
  return T.x * p.x + T.y * p.y + T.z * p.z;
}

static __forceinline__ __device__ float project(const float4& T, const float3& p) {
  return T.x * p.x + T.y * p.y + T.z * p.z + T.w;
}

static __forceinline__ __device__ float3 inverse_project(const float4* T, const float3& p) {
  float3 r;
  r.x = p.x - T[0].w;
  r.y = p.y - T[1].w;
  r.z = p.z - T[2].w;
  return make_float3(dot(T[0], r), dot(T[1], r), dot(T[2], r));
}

static __global__ void arange_kernel(unsigned int num, int32_t* p) {
  int idx = cuda_linear_index;
  if (idx < num) {
    p[idx] = idx;
  }
}

static __global__ void interval_starts_kernel(
    unsigned int num, unsigned int remain, unsigned int total,
    const int32_t* ranks, const int32_t* indices,
    int32_t* interval_starts, int32_t* interval_starts_size) {
  int idx = cuda_linear_index;
  if (idx >= num) {
    return;
  }

  unsigned int i = remain + 1 + idx;
  if (ranks[i] != ranks[i - 1]) {
    unsigned int offset = atomicAdd(interval_starts_size, 1);
    interval_starts[offset] = idx + 1;
  }
}

static __global__ void collect_starts_kernel(
    unsigned int num, unsigned int remain, unsigned int numel_geometry,
    const int32_t* indices, const int32_t* interval_starts,
    const int32_t* geometry, int3* intervals) {
  int i = cuda_linear_index;
  if (i >= num) {
    return;
  }

  int3 val;
  val.x = interval_starts[i] + remain;
  val.y = (i < num - 1) ? interval_starts[i + 1] + remain : numel_geometry - 1;
  val.z = geometry[indices[val.x]];
  intervals[i] = val;
}

static void __host__ matrix_inverse_4x4(const float* m, float* inv) {
  double det = m[0] * (m[5] * m[10] - m[9] * m[6]) - m[1] * (m[4] * m[10] - m[6] * m[8]) + m[2] * (m[4] * m[9] - m[5] * m[8]);
  double invdet = 1.0 / det;
  inv[0] = (m[5] * m[10] - m[9] * m[6]) * invdet;
  inv[1] = (m[2] * m[9] - m[1] * m[10]) * invdet;
  inv[2] = (m[1] * m[6] - m[2] * m[5]) * invdet;
  inv[3] = m[3];
  inv[4] = (m[6] * m[8] - m[4] * m[10]) * invdet;
  inv[5] = (m[0] * m[10] - m[2] * m[8]) * invdet;
  inv[6] = (m[4] * m[2] - m[0] * m[6]) * invdet;
  inv[7] = m[7];
  inv[8] = (m[4] * m[9] - m[8] * m[5]) * invdet;
  inv[9] = (m[8] * m[1] - m[0] * m[9]) * invdet;
  inv[10] = (m[0] * m[5] - m[4] * m[1]) * invdet;
  inv[11] = m[11];
  inv[12] = m[12];
  inv[13] = m[13];
  inv[14] = m[14];
  inv[15] = m[15];
}

static __global__ void create_frustum_kernel(unsigned int feat_width, unsigned int feat_height, unsigned int D,
                                             unsigned int image_width, unsigned int image_height, float w_interval,
                                             float h_interval, nvtype::Float3 dbound, float3* frustum) {
  int ix = cuda_2d_x;
  int iy = cuda_2d_y;
  int id = blockIdx.z;
  if (ix >= feat_width || iy >= feat_height) {
    return;
  }

  unsigned int offset = (id * feat_height + iy) * feat_width + ix;
  frustum[offset] = make_float3(ix * w_interval, iy * h_interval, dbound.x + id * dbound.z);
}

static __global__ void compute_geometry_kernel(
    unsigned int numel_frustum, const float3* frustum,
    const float4* camera2lidar, const float4* camera_intrins_inv,
    const float4* img_aug_matrix_inv,
    nvtype::Float3 bx, nvtype::Float3 dx, nvtype::Int3 nx,
    unsigned int* keep_count, int* ranks,
    nvtype::Int3 geometry_dim, unsigned int num_camera,
    int* geometry_out) {

  int tid = cuda_linear_index;
  if (tid >= numel_frustum) {
    return;
  }

  float3 point = frustum[tid];

  for (int icam = 0; icam < num_camera; ++icam) {
    float3 projed = inverse_project(img_aug_matrix_inv, point);
    projed.x *= projed.z;
    projed.y *= projed.z;

    projed = make_float3(
        dot(camera_intrins_inv[4 * icam + 0], projed),
        dot(camera_intrins_inv[4 * icam + 1], projed),
        dot(camera_intrins_inv[4 * icam + 2], projed));

    projed = make_float3(
        project(camera2lidar[4 * icam + 0], projed),
        project(camera2lidar[4 * icam + 1], projed),
        project(camera2lidar[4 * icam + 2], projed));

    int pid = icam * numel_frustum + tid;

    int3 c;
    c.x = int((projed.x - (bx.x - dx.x * 0.5f)) / dx.x);
    c.y = int((projed.y - (bx.y - dx.y * 0.5f)) / dx.y);
    c.z = int((projed.z - (bx.z - dx.z * 0.5f)) / dx.z);

    bool kept =
        c.x >= 0 && c.x < nx.x &&
        c.y >= 0 && c.y < nx.y &&
        c.z >= 0 && c.z < nx.z;

    if (!kept) {
      ranks[pid] = 0;
    } else {
      atomicAdd(keep_count, 1);
      ranks[pid] = (c.x * nx.y + c.y) * nx.z + c.z;
      geometry_out[pid] =
          (c.z * geometry_dim.z * geometry_dim.y + c.x) *
          geometry_dim.x + c.y;
    }
  }
}

class GeometryImplement : public Geometry {
 public:
  ~GeometryImplement() override {
    if (counter_host_) {
      checkRuntime(cudaFreeHost(counter_host_));
    }
    if (keep_count_) {
      checkRuntime(cudaFree(keep_count_));
    }
    if (frustum_) {
      checkRuntime(cudaFree(frustum_));
    }
    if (geometry_) {
      checkRuntime(cudaFree(geometry_));
    }
    if (ranks_) {
      checkRuntime(cudaFree(ranks_));
    }
    if (indices_) {
      checkRuntime(cudaFree(indices_));
    }
    if (interval_starts_) {
      checkRuntime(cudaFree(interval_starts_));
    }
    if (interval_starts_size_) {
      checkRuntime(cudaFree(interval_starts_size_));
    }
    if (intervals_) {
      checkRuntime(cudaFree(intervals_));
    }
    if (camera2lidar_) {
      checkRuntime(cudaFree(camera2lidar_));
    }
    if (camera_intrinsics_inverse_) {
      checkRuntime(cudaFree(camera_intrinsics_inverse_));
    }
    if (img_aug_matrix_inverse_) {
      checkRuntime(cudaFree(img_aug_matrix_inverse_));
    }
    if (camera_intrinsics_inverse_host_) {
      checkRuntime(cudaFreeHost(camera_intrinsics_inverse_host_));
    }
    if (img_aug_matrix_inverse_host_) {
      checkRuntime(cudaFreeHost(img_aug_matrix_inverse_host_));
    }
    if (cub_tmp_) {
      checkRuntime(cudaFree(cub_tmp_));
    }
  }

  bool init(GeometryParameter param) {
    static_cast<GeometryParameter&>(param_) = param;

    param_.D = (unsigned int)((param_.dbound.y - param_.dbound.x) / param_.dbound.z);
    param_.bx = {param_.xbound.x + param_.xbound.z * 0.5f,
                 param_.ybound.x + param_.ybound.z * 0.5f,
                 param_.zbound.x + param_.zbound.z * 0.5f};
    param_.dx = {param_.xbound.z, param_.ybound.z, param_.zbound.z};
    param_.nx = {
        int((param_.xbound.y - param_.xbound.x) / param_.xbound.z),
        int((param_.ybound.y - param_.ybound.x) / param_.ybound.z),
        int((param_.zbound.y - param_.zbound.x) / param_.zbound.z)};

    cudaStream_t stream = nullptr;
    float w_interval = (param_.image_width - 1.0f) /
                       (param_.feat_width - 1.0f);
    float h_interval = (param_.image_height - 1.0f) /
                       (param_.feat_height - 1.0f);

    numel_frustum_ = param_.feat_width * param_.feat_height * param_.D;
    numel_geometry_ = numel_frustum_ * param_.num_camera;

    checkRuntime(cudaMallocHost(&counter_host_, sizeof(unsigned int)));
    checkRuntime(cudaMalloc(&keep_count_, sizeof(unsigned int)));
    checkRuntime(cudaMalloc(&frustum_, numel_frustum_ * sizeof(float3)));
    checkRuntime(cudaMalloc(&geometry_, numel_geometry_ * sizeof(int)));
    checkRuntime(cudaMalloc(&ranks_, numel_geometry_ * sizeof(int)));
    checkRuntime(cudaMalloc(&indices_, numel_geometry_ * sizeof(int)));
    checkRuntime(cudaMalloc(&interval_starts_, numel_geometry_ * sizeof(int)));
    checkRuntime(cudaMalloc(&interval_starts_size_, sizeof(int)));
    checkRuntime(cudaMalloc(&intervals_, numel_geometry_ * sizeof(int3)));

    bytes_of_matrix_ = param_.num_camera * 16 * sizeof(float);
    checkRuntime(cudaMalloc(&camera2lidar_, bytes_of_matrix_));
    checkRuntime(cudaMalloc(&camera_intrinsics_inverse_, bytes_of_matrix_));
    checkRuntime(cudaMalloc(&img_aug_matrix_inverse_, bytes_of_matrix_));
    checkRuntime(cudaMallocHost(&camera_intrinsics_inverse_host_, bytes_of_matrix_));
    checkRuntime(cudaMallocHost(&img_aug_matrix_inverse_host_, bytes_of_matrix_));
    cuda_2d_launch(create_frustum_kernel, stream, param_.feat_width, param_.feat_height, param_.D, param_.image_width,
                param_.image_height, w_interval, h_interval, param_.dbound, frustum_);
    return true;
  }

  void update(const float* camera2lidar,
              const float* camera_intrinsics,
              const float* img_aug_matrix,
              void* stream) override {

    cudaStream_t s = static_cast<cudaStream_t>(stream);
    for (unsigned int icamera = 0; icamera < param_.num_camera; ++icamera) {
      unsigned int offset = icamera * 16;
      matrix_inverse_4x4(camera_intrinsics + offset,
                         camera_intrinsics_inverse_host_ + offset);
      matrix_inverse_4x4(img_aug_matrix + offset,
                         img_aug_matrix_inverse_host_ + offset);
    }

    checkRuntime(cudaMemcpyAsync(camera2lidar_, camera2lidar, bytes_of_matrix_, cudaMemcpyHostToDevice, s));
    checkRuntime(cudaMemcpyAsync(camera_intrinsics_inverse_, camera_intrinsics_inverse_host_, bytes_of_matrix_, cudaMemcpyHostToDevice, s));
    checkRuntime(cudaMemcpyAsync(img_aug_matrix_inverse_, img_aug_matrix_inverse_host_, bytes_of_matrix_, cudaMemcpyHostToDevice, s));
    checkRuntime(cudaMemsetAsync(keep_count_, 0, sizeof(unsigned int), s));

    cuda_linear_launch(compute_geometry_kernel, s, numel_frustum_,
                       frustum_, (float4*)camera2lidar_,
                       (float4*)camera_intrinsics_inverse_,
                       (float4*)img_aug_matrix_inverse_,
                       param_.bx, param_.dx, param_.nx,
                       keep_count_, ranks_, param_.geometry_dim,
                       param_.num_camera, geometry_);

    checkRuntime(cudaMemcpyAsync(counter_host_, keep_count_, sizeof(unsigned int),
                                 cudaMemcpyDeviceToHost, s));

    cuda_linear_launch(arange_kernel, s, numel_geometry_, indices_);

    size_t tmp_bytes = 0;
    cub::DeviceRadixSort::SortPairs(
        nullptr, tmp_bytes,
        ranks_, ranks_,
        indices_, indices_,
        numel_geometry_, 0, 32, s);

    if (cub_tmp_bytes_ < tmp_bytes) {
      if (cub_tmp_) {
        checkRuntime(cudaFree(cub_tmp_));
      }
      checkRuntime(cudaMalloc(&cub_tmp_, tmp_bytes));
      cub_tmp_bytes_ = tmp_bytes;
    }

    cub::DeviceRadixSort::SortPairs(
        cub_tmp_, cub_tmp_bytes_,
        ranks_, ranks_,
        indices_, indices_,
        numel_geometry_, 0, 32, s);

    checkRuntime(cudaStreamSynchronize(s));

    unsigned int remain = numel_geometry_ - *counter_host_;
    unsigned int threads = *counter_host_ - 1;

    checkRuntime(cudaMemsetAsync(interval_starts_size_, 0, sizeof(int), s));
    checkRuntime(cudaMemsetAsync(interval_starts_, 0, sizeof(int), s));

    cuda_linear_launch(interval_starts_kernel, s,
                       threads, remain, numel_geometry_,
                       ranks_, indices_,
                       interval_starts_ + 1, interval_starts_size_);

    checkRuntime(cudaMemcpyAsync(counter_host_, interval_starts_size_,
                                 sizeof(unsigned int), cudaMemcpyDeviceToHost, s));
    checkRuntime(cudaStreamSynchronize(s));

    n_intervals_ = *counter_host_ + 1;

    cub::DeviceRadixSort::SortKeys(
        cub_tmp_, cub_tmp_bytes_,
        interval_starts_, interval_starts_,
        n_intervals_, 0, 32, s);

    cuda_linear_launch(collect_starts_kernel, s,
                       n_intervals_, remain, numel_geometry_,
                       indices_, interval_starts_,
                       geometry_, intervals_);
  }

  void free_excess_memory() {
    if (counter_host_) {
      checkRuntime(cudaFreeHost(counter_host_));
      counter_host_ = nullptr;
    }
    if (keep_count_) {
      checkRuntime(cudaFree(keep_count_));
      keep_count_ = nullptr;
    }
    if (frustum_) {
      checkRuntime(cudaFree(frustum_));
      frustum_ = nullptr;
    }
    if (geometry_) {
      checkRuntime(cudaFree(geometry_));
      geometry_ = nullptr;
    }
    if (ranks_) {
      checkRuntime(cudaFree(ranks_));
      ranks_ = nullptr;
    }
    if (indices_) {
      checkRuntime(cudaFree(indices_));
      indices_ = nullptr;
    }
    if (interval_starts_) {
      checkRuntime(cudaFree(interval_starts_));
      interval_starts_ = nullptr;
    }
    if (interval_starts_size_) {
      checkRuntime(cudaFree(interval_starts_size_));
      interval_starts_size_ = nullptr;
    }
    if (intervals_) {
      checkRuntime(cudaFree(intervals_));
      intervals_ = nullptr;
    }
    if (camera2lidar_) {
      checkRuntime(cudaFree(camera2lidar_));
      camera2lidar_ = nullptr;
    }
    if (camera_intrinsics_inverse_) {
      checkRuntime(cudaFree(camera_intrinsics_inverse_));
      camera_intrinsics_inverse_ = nullptr;
    }
    if (img_aug_matrix_inverse_) {
      checkRuntime(cudaFree(img_aug_matrix_inverse_));
      img_aug_matrix_inverse_ = nullptr;
    }
    if (camera_intrinsics_inverse_host_) {
      checkRuntime(cudaFreeHost(camera_intrinsics_inverse_host_));
      camera_intrinsics_inverse_host_ = nullptr;
    }
    if (img_aug_matrix_inverse_host_) {
      checkRuntime(cudaFreeHost(img_aug_matrix_inverse_host_));
      img_aug_matrix_inverse_host_ = nullptr;
    }
    if (cub_tmp_) {
      checkRuntime(cudaFree(cub_tmp_));
      cub_tmp_ = nullptr;
      cub_tmp_bytes_ = 0;
    }
}

  unsigned int num_intervals() override {
    return n_intervals_;
  }
  unsigned int num_indices() override {
    return numel_geometry_;
  }
  nvtype::Int3* intervals() override {
    return (nvtype::Int3*)intervals_;
  }
  unsigned int* indices() override {
    return (unsigned int*)indices_;
  }

 private:
  size_t bytes_of_matrix_ = 0;
  void* cub_tmp_ = nullptr;
  size_t cub_tmp_bytes_ = 0;

  float* camera2lidar_ = nullptr;
  float* camera_intrinsics_inverse_ = nullptr;
  float* img_aug_matrix_inverse_ = nullptr;
  float* camera_intrinsics_inverse_host_ = nullptr;
  float* img_aug_matrix_inverse_host_ = nullptr;

  float3* frustum_ = nullptr;
  int* geometry_ = nullptr;
  int* ranks_ = nullptr;
  int* indices_ = nullptr;
  int* interval_starts_ = nullptr;
  int* interval_starts_size_ = nullptr;
  int3* intervals_ = nullptr;

  unsigned int* keep_count_ = nullptr;
  unsigned int* counter_host_ = nullptr;

  unsigned int numel_frustum_ = 0;
  unsigned int numel_geometry_ = 0;
  unsigned int n_intervals_ = 0;

  GeometryParameterExtra param_;
};

std::shared_ptr<Geometry> create_geometry(GeometryParameter param) {
  auto inst = std::make_shared<GeometryImplement>();
  if (!inst->init(param)) {
    inst.reset();
  }
  return inst;
}

};  // namespace camera
};  // namespace bevfusion
