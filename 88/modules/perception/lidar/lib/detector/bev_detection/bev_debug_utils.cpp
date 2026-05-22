/****************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/lidar/lib/detector/bev_detection/bev_debug_utils.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"

namespace century {
namespace perception {
namespace lidar {

namespace {

float HalfToFloat(const nvtype::half& value) {
  return __half2float(*reinterpret_cast<const __half*>(&value));
}

struct HostTensor {
  std::vector<int64_t> shape;
  std::vector<char> data;
};

bool LoadHostTensor(const std::string& path, size_t element_size,
                    HostTensor* tensor) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    AERROR << "Failed to open tensor file: " << path;
    return false;
  }

  int64_t num_dims = 0;
  ifs.read(reinterpret_cast<char*>(&num_dims), sizeof(int64_t));
  tensor->shape.resize(num_dims);
  size_t total = 1;
  for (int64_t i = 0; i < num_dims; ++i) {
    ifs.read(reinterpret_cast<char*>(&tensor->shape[i]), sizeof(int64_t));
    total *= tensor->shape[i];
  }
  tensor->data.resize(total * element_size);
  ifs.read(tensor->data.data(), tensor->data.size());
  return true;
}

float NormalizeAngleDiff(float diff) {
  constexpr float kPi = 3.14159265358979323846f;
  while (diff > kPi) {
    diff -= 2.0f * kPi;
  }
  while (diff < -kPi) {
    diff += 2.0f * kPi;
  }
  return diff;
}

}  // namespace

BevDebugUtils::BevDebugUtils() {
}

BevDebugUtils::~BevDebugUtils() {
}

void BevDebugUtils::SetOutputDir(const std::string& dir) {
  output_dir_ = dir;
  struct stat st;
  if (stat(dir.c_str(), &st) != 0) {
    (void)system(("mkdir -p " + dir).c_str());
  }
}

void BevDebugUtils::SetEnabled(bool enabled) {
  enabled_ = enabled;
}

template <typename T>
void BevDebugUtils::SaveTensor(const void* data, const std::vector<T>& shape,
                               const std::string& filename, size_t element_size,
                               cudaStream_t stream) {
  if (!enabled_) {
    return;
  }

  std::string filepath = output_dir_ + filename;

  auto slash = filepath.find_last_of('/');
  if (slash != std::string::npos) {
    std::string parent = filepath.substr(0, slash);
    struct stat st;
    if (stat(parent.c_str(), &st) != 0) {
      (void)system(("mkdir -p " + parent).c_str());
    }
  }

  size_t total_bytes = element_size;
  for (auto s : shape) {
    total_bytes *= s;
  }

  void* host_data = nullptr;
  checkRuntime(cudaMallocHost(&host_data, total_bytes));

  checkRuntime(cudaMemcpyAsync(host_data, data, total_bytes,
                               cudaMemcpyDeviceToHost, stream));
  checkRuntime(cudaStreamSynchronize(stream));

  std::ofstream ofs(filepath, std::ios::binary);
  if (ofs.is_open()) {
    int64_t num_dims = shape.size();
    ofs.write(reinterpret_cast<const char*>(&num_dims), sizeof(int64_t));
    for (auto s : shape) {
      int64_t dim = s;
      ofs.write(reinterpret_cast<const char*>(&dim), sizeof(int64_t));
    }
    ofs.write(reinterpret_cast<const char*>(host_data), total_bytes);
    ofs.close();

    std::ostringstream shape_oss;
    for (size_t i = 0; i < shape.size(); ++i) {
      if (i > 0) {
        shape_oss << ", ";
      }
      shape_oss << shape[i];
    }
    AINFO << "Saved tensor to " << filepath << " | Shape: [" << shape_oss.str()
          << "] | Dtype: float16 | Bytes: " << total_bytes;
  } else {
    AERROR << "Failed to open file for writing: " << filepath;
  }

  checkRuntime(cudaFreeHost(host_data));
}

nvtype::half* BevDebugUtils::LoadTensor(const std::string& filepath,
                                        std::vector<int64_t>& shape,
                                        cudaStream_t stream) {
  std::string full_path = output_dir_ + filepath;
  std::ifstream ifs(full_path, std::ios::binary);
  if (!ifs.is_open()) {
    AERROR << "Failed to open file: " << full_path;
    return nullptr;
  }

  int64_t num_dims;
  ifs.read(reinterpret_cast<char*>(&num_dims), sizeof(int64_t));

  shape.resize(num_dims);
  for (int64_t i = 0; i < num_dims; ++i) {
    ifs.read(reinterpret_cast<char*>(&shape[i]), sizeof(int64_t));
  }

  size_t total_elements = 1;
  for (auto s : shape) {
    total_elements *= s;
  }
  size_t total_bytes = total_elements * sizeof(nvtype::half);

  void* host_buffer = nullptr;
  checkRuntime(cudaMallocHost(&host_buffer, total_bytes));

  ifs.read(reinterpret_cast<char*>(host_buffer), total_bytes);
  ifs.close();

  nvtype::half* device_ptr = nullptr;
  checkRuntime(cudaMalloc(&device_ptr, total_bytes));
  checkRuntime(cudaMemcpyAsync(device_ptr, host_buffer, total_bytes,
                               cudaMemcpyHostToDevice, stream));
  checkRuntime(cudaStreamSynchronize(stream));

  checkRuntime(cudaFreeHost(host_buffer));

  std::ostringstream shape_oss;
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      shape_oss << ", ";
    }
    shape_oss << shape[i];
  }
  AINFO << "Loaded tensor from " << full_path << " | Shape: ["
        << shape_oss.str() << "] | Bytes: " << total_bytes;

  return device_ptr;
}

float* BevDebugUtils::LoadTensorFloat(const std::string& filepath,
                                      std::vector<int64_t>& shape,
                                      cudaStream_t stream) {
  std::string full_path = output_dir_ + filepath;
  std::ifstream ifs(full_path, std::ios::binary);
  if (!ifs.is_open()) {
    AERROR << "Failed to open file: " << full_path;
    return nullptr;
  }

  int64_t num_dims;
  ifs.read(reinterpret_cast<char*>(&num_dims), sizeof(int64_t));

  shape.resize(num_dims);
  for (int64_t i = 0; i < num_dims; ++i) {
    ifs.read(reinterpret_cast<char*>(&shape[i]), sizeof(int64_t));
  }

  size_t total_elements = 1;
  for (auto s : shape) {
    total_elements *= s;
  }
  size_t total_bytes = total_elements * sizeof(float);

  void* host_buffer = nullptr;
  checkRuntime(cudaMallocHost(&host_buffer, total_bytes));
  ifs.read(reinterpret_cast<char*>(host_buffer), total_bytes);
  ifs.close();

  float* device_ptr = nullptr;
  checkRuntime(cudaMalloc(&device_ptr, total_bytes));
  checkRuntime(cudaMemcpyAsync(device_ptr, host_buffer, total_bytes,
                               cudaMemcpyHostToDevice, stream));
  checkRuntime(cudaStreamSynchronize(stream));
  checkRuntime(cudaFreeHost(host_buffer));

  std::ostringstream shape_oss;
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      shape_oss << ", ";
    }
    shape_oss << shape[i];
  }
  AINFO << "Loaded fp32 tensor from " << full_path << " | Shape: ["
        << shape_oss.str() << "] | Bytes: " << total_bytes;

  return device_ptr;
}

void BevDebugUtils::CompareWithPython(const void* gpu_data,
                                      const std::string& py_filename,
                                      const std::string& tag,
                                      size_t element_size,
                                      cudaStream_t stream) {
  if (!enabled_) {
    return;
  }

  std::string full_path = output_dir_ + "py/" + py_filename;
  std::ifstream ifs(full_path, std::ios::binary);
  if (!ifs.is_open()) {
    AERROR << "[Compare " << tag << "] Failed to open: " << full_path;
    return;
  }

  int64_t num_dims;
  ifs.read(reinterpret_cast<char*>(&num_dims), sizeof(int64_t));
  std::vector<int64_t> shape(num_dims);
  for (int64_t i = 0; i < num_dims; ++i) {
    ifs.read(reinterpret_cast<char*>(&shape[i]), sizeof(int64_t));
  }

  size_t total_elements = 1;
  for (auto s : shape) {
    total_elements *= s;
  }
  size_t py_bytes = total_elements * element_size;

  std::vector<char> py_host(py_bytes);
  ifs.read(py_host.data(), py_bytes);
  ifs.close();

  std::vector<char> cpp_host(py_bytes);
  checkRuntime(cudaStreamSynchronize(stream));
  checkRuntime(
      cudaMemcpy(cpp_host.data(), gpu_data, py_bytes, cudaMemcpyDeviceToHost));

  double max_abs_err = 0.0;
  double sum_abs_err = 0.0;
  double sum_sq_err = 0.0;
  double max_rel_err = 0.0;
  int nan_count = 0;
  int inf_count = 0;

  if (element_size == 2) {
    const nvtype::half* py_ptr =
        reinterpret_cast<const nvtype::half*>(py_host.data());
    const nvtype::half* cpp_ptr =
        reinterpret_cast<const nvtype::half*>(cpp_host.data());
    for (size_t i = 0; i < total_elements; ++i) {
      float pv = HalfToFloat(py_ptr[i]);
      float cv = HalfToFloat(cpp_ptr[i]);
      if (std::isnan(cv)) {
        ++nan_count;
        continue;
      }
      if (std::isinf(cv)) {
        ++inf_count;
        continue;
      }
      float diff = std::abs(pv - cv);
      sum_abs_err += diff;
      sum_sq_err += diff * diff;
      if (diff > max_abs_err) {
        max_abs_err = diff;
      }
      float denom = std::max(std::abs(pv), 1e-6f);
      float rel = diff / denom;
      if (rel > max_rel_err) {
        max_rel_err = rel;
      }
    }
  } else {
    const float* py_ptr = reinterpret_cast<const float*>(py_host.data());
    const float* cpp_ptr = reinterpret_cast<const float*>(cpp_host.data());
    for (size_t i = 0; i < total_elements; ++i) {
      float diff = std::abs(py_ptr[i] - cpp_ptr[i]);
      sum_abs_err += diff;
      sum_sq_err += diff * diff;
      if (diff > max_abs_err) {
        max_abs_err = diff;
      }
      float denom = std::max(std::abs(py_ptr[i]), 1e-6f);
      float rel = diff / denom;
      if (rel > max_rel_err) {
        max_rel_err = rel;
      }
    }
  }

  double mean_abs_err = sum_abs_err / total_elements;
  double rmse = std::sqrt(sum_sq_err / total_elements);

  std::ostringstream shape_oss;
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      shape_oss << ",";
    }
    shape_oss << shape[i];
  }
  AINFO << "\n===== [Compare " << tag << "] =====";
  AINFO << "  Python file: " << py_filename;
  AINFO << "  Shape: [" << shape_oss.str() << "]  Elements: " << total_elements;
  AINFO << std::fixed << std::setprecision(6)
        << "  Max Abs Error : " << max_abs_err;
  AINFO << std::fixed << std::setprecision(6)
        << "  Mean Abs Error: " << mean_abs_err;
  AINFO << std::fixed << std::setprecision(6) << "  RMSE          : " << rmse;
  AINFO << std::fixed << std::setprecision(6)
        << "  Max Rel Error : " << max_rel_err;
  if (nan_count > 0 || inf_count > 0) {
    AINFO << "  NaN: " << nan_count << "  Inf: " << inf_count;
  }
  AINFO << "============================\n";
}

namespace {

bool ReadHalfFile(const std::string& path, std::vector<nvtype::half>* out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    AERROR << "Failed to open: " << path;
    return false;
  }
  int64_t nd = 0;
  ifs.read(reinterpret_cast<char*>(&nd), sizeof(int64_t));
  size_t total = 1;
  for (int64_t i = 0; i < nd; ++i) {
    int64_t d = 0;
    ifs.read(reinterpret_cast<char*>(&d), sizeof(int64_t));
    total *= static_cast<size_t>(d);
  }
  out->resize(total);
  ifs.read(reinterpret_cast<char*>(out->data()), total * sizeof(nvtype::half));
  return true;
}

std::vector<nvtype::half> CopyDeviceHalf(const void* src, size_t n,
                                         cudaStream_t stream) {
  std::vector<nvtype::half> host(n);
  checkRuntime(cudaMemcpyAsync(host.data(), src, n * sizeof(nvtype::half),
                               cudaMemcpyDeviceToHost, stream));
  return host;
}

}  // namespace

void BevDebugUtils::CompareHeadByQueryMatch(
    const void* cpp_reg, const void* cpp_height, const void* cpp_dim,
    const void* cpp_rot, const void* cpp_vel, const void* cpp_score,
    cudaStream_t stream) {
  if (!enabled_) {
    return;
  }

  const std::string pydir = output_dir_ + "py/";
  std::vector<nvtype::half> py_reg, py_height, py_dim, py_rot, py_vel, py_score;
  if (!ReadHalfFile(pydir + "08_head_reg.bin", &py_reg) ||
      !ReadHalfFile(pydir + "08_head_height.bin", &py_height) ||
      !ReadHalfFile(pydir + "08_head_dim.bin", &py_dim) ||
      !ReadHalfFile(pydir + "08_head_rot.bin", &py_rot) ||
      !ReadHalfFile(pydir + "08_head_vel.bin", &py_vel) ||
      !ReadHalfFile(pydir + "08_head_score.bin", &py_score)) {
    AERROR << "[CompareHeadByQueryMatch] missing python head dumps.";
    return;
  }

  constexpr int kN = 200;
  if (py_reg.size() != 2 * kN || py_height.size() != 1 * kN ||
      py_dim.size() != 3 * kN || py_rot.size() != 2 * kN ||
      py_vel.size() != 2 * kN || py_score.size() != 15 * kN) {
    AERROR << "[CompareHeadByQueryMatch] unexpected shape in python dumps.";
    return;
  }

  checkRuntime(cudaStreamSynchronize(stream));
  auto c_reg = CopyDeviceHalf(cpp_reg, 2 * kN, stream);
  auto c_height = CopyDeviceHalf(cpp_height, 1 * kN, stream);
  auto c_dim = CopyDeviceHalf(cpp_dim, 3 * kN, stream);
  auto c_rot = CopyDeviceHalf(cpp_rot, 2 * kN, stream);
  auto c_vel = CopyDeviceHalf(cpp_vel, 2 * kN, stream);
  auto c_score = CopyDeviceHalf(cpp_score, 15 * kN, stream);
  checkRuntime(cudaStreamSynchronize(stream));

  std::vector<int> match(kN, -1);
  std::vector<float> dists(kN, 0.f);
  float sum_d = 0.f, max_d = 0.f;
  for (int i = 0; i < kN; ++i) {
    float px = HalfToFloat(py_reg[0 * kN + i]);
    float py = HalfToFloat(py_reg[1 * kN + i]);
    int best = -1;
    float best_d2 = 1e18f;
    for (int j = 0; j < kN; ++j) {
      float cx = HalfToFloat(c_reg[0 * kN + j]);
      float cy = HalfToFloat(c_reg[1 * kN + j]);
      float dx = px - cx;
      float dy = py - cy;
      float d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best = j;
      }
    }
    match[i] = best;
    dists[i] = std::sqrt(best_d2);
    sum_d += dists[i];
    if (dists[i] > max_d) {
      max_d = dists[i];
    }
  }
  std::vector<float> sorted_d = dists;
  std::sort(sorted_d.begin(), sorted_d.end());
  float median_d = sorted_d[kN / 2];
  std::vector<int> seen(kN, 0);
  int uniq = 0;
  for (int i = 0; i < kN; ++i) {
    if (!seen[match[i]]) {
      seen[match[i]] = 1;
      ++uniq;
    }
  }

  auto stat = [&](const char* name, int ch,
                  const std::vector<nvtype::half>& py_data,
                  const std::vector<nvtype::half>& cpp_data) {
    double sum_abs = 0.0, sum_sq = 0.0, max_abs = 0.0;
    const int total = ch * kN;
    for (int c = 0; c < ch; ++c) {
      for (int i = 0; i < kN; ++i) {
        int j = match[i];
        float pv = HalfToFloat(py_data[c * kN + i]);
        float cv = HalfToFloat(cpp_data[c * kN + j]);
        float diff = std::abs(pv - cv);
        sum_abs += diff;
        sum_sq += diff * diff;
        if (diff > max_abs) {
          max_abs = diff;
        }
      }
    }
    double mean = sum_abs / total;
    double rmse = std::sqrt(sum_sq / total);
    std::ostringstream line;
    line << "  " << std::left << std::setw(6) << std::setfill(' ') << name
         << "  mean=" << std::fixed << std::setprecision(6) << mean
         << "  max=" << max_abs << "  rmse=" << rmse;
    AINFO << line.str();
  };

  AINFO << "\n===== [Compare Head by Query-Match] =====";
  AINFO << std::fixed << std::setprecision(4)
        << "  Match dist (BEV px): mean=" << sum_d / kN << "  max=" << max_d
        << "  median=" << median_d;
  AINFO << "  Unique cpp queries matched: " << uniq << " / " << kN;
  stat("reg", 2, py_reg, c_reg);
  stat("height", 1, py_height, c_height);
  stat("dim", 3, py_dim, c_dim);
  stat("rot", 2, py_rot, c_rot);
  stat("vel", 2, py_vel, c_vel);
  stat("score", 15, py_score, c_score);
  AINFO << "==========================================\n";
}

void BevDebugUtils::SaveCameraImages(const unsigned char** images,
                                     int num_cameras, int width, int height,
                                     const std::string& filename) {
  if (!enabled_) {
    return;
  }

  std::string filepath = output_dir_ + filename;

  std::ofstream ofs(filepath, std::ios::binary);
  if (ofs.is_open()) {
    int64_t shape[4] = {num_cameras, height, width, 3};
    ofs.write(reinterpret_cast<const char*>(&shape[0]), 4 * sizeof(int64_t));

    size_t bytes_per_image = width * height * 3;
    for (int i = 0; i < num_cameras; ++i) {
      ofs.write(reinterpret_cast<const char*>(images[i]), bytes_per_image);
    }
    ofs.close();
    AINFO << "Saved camera images to " << filepath;
  }
}

void BevDebugUtils::SaveLidarPoints(const nvtype::half* points, int num_points,
                                    int num_features,
                                    const std::string& filename,
                                    cudaStream_t stream) {
  if (!enabled_) {
    return;
  }

  std::vector<int64_t> shape = {num_points, num_features};
  SaveTensor(points, shape, filename, sizeof(nvtype::half), stream);
}

bool BevDebugUtils::LoadPythonVoxels(void** voxel_features, void** voxel_coords,
                                     int64_t& num_voxels, int64_t& voxel_dim,
                                     cudaStream_t stream) {
  std::string feat_path = output_dir_ + "py/py_01_voxel_features.bin";
  std::string coord_path = output_dir_ + "py/py_02_voxel_coords.bin";

  std::ifstream feat_file(feat_path, std::ios::binary);
  std::ifstream coord_file(coord_path, std::ios::binary);

  if (!feat_file.is_open() || !coord_file.is_open()) {
    return false;
  }

  int64_t feat_num_dims;
  feat_file.read(reinterpret_cast<char*>(&feat_num_dims), sizeof(int64_t));
  std::vector<int64_t> feat_shape(feat_num_dims);
  for (int i = 0; i < feat_num_dims; ++i) {
    feat_file.read(reinterpret_cast<char*>(&feat_shape[i]), sizeof(int64_t));
  }

  size_t feat_elements = feat_shape[0] * feat_shape[1];
  size_t feat_bytes = feat_elements * sizeof(nvtype::half);
  std::vector<nvtype::half> feat_data(feat_elements);
  feat_file.read(reinterpret_cast<char*>(feat_data.data()), feat_bytes);
  feat_file.close();

  int64_t coord_num_dims;
  coord_file.read(reinterpret_cast<char*>(&coord_num_dims), sizeof(int64_t));
  std::vector<int64_t> coord_shape(coord_num_dims);
  for (int i = 0; i < coord_num_dims; ++i) {
    coord_file.read(reinterpret_cast<char*>(&coord_shape[i]), sizeof(int64_t));
  }

  size_t coord_elements = coord_shape[0] * coord_shape[1];
  size_t coord_bytes = coord_elements * sizeof(int32_t);
  std::vector<int32_t> coord_data(coord_elements);
  coord_file.read(reinterpret_cast<char*>(coord_data.data()), coord_bytes);
  coord_file.close();

  AERROR << "Loaded Python voxels: " << feat_shape[0] << " voxels, "
         << feat_shape[1] << " dims";

  if (*voxel_features != nullptr) {
    cudaFree(*voxel_features);
  }
  if (*voxel_coords != nullptr) {
    cudaFree(*voxel_coords);
  }

  cudaMalloc(voxel_features, feat_bytes);
  cudaMemcpyAsync(*voxel_features, feat_data.data(), feat_bytes,
                  cudaMemcpyHostToDevice, stream);

  cudaMalloc(voxel_coords, coord_bytes);
  cudaMemcpyAsync(*voxel_coords, coord_data.data(), coord_bytes,
                  cudaMemcpyHostToDevice, stream);
  cudaStreamSynchronize(stream);

  num_voxels = feat_shape[0];
  voxel_dim = feat_shape[1];

  AERROR << "Python voxels loaded to device memory";
  return true;
}

void BevDebugUtils::CompareFinalBBoxes(
    const std::vector<bevfusion::head::transbbox::BoundingBox>& cpp_boxes) {
  HostTensor bbox_tensor;
  HostTensor score_tensor;
  HostTensor label_tensor;
  if (!LoadHostTensor(output_dir_ + "py/09_pred_bboxes.bin", sizeof(float),
                      &bbox_tensor) ||
      !LoadHostTensor(output_dir_ + "py/09_pred_scores.bin", sizeof(float),
                      &score_tensor) ||
      !LoadHostTensor(output_dir_ + "py/09_pred_labels.bin", sizeof(int32_t),
                      &label_tensor)) {
    return;
  }

  const float* py_bbox =
      reinterpret_cast<const float*>(bbox_tensor.data.data());
  const float* py_score =
      reinterpret_cast<const float*>(score_tensor.data.data());
  const int32_t* py_label =
      reinterpret_cast<const int32_t*>(label_tensor.data.data());

  size_t py_count = bbox_tensor.shape.empty() ? 0 : bbox_tensor.shape[0];
  size_t py_dim = bbox_tensor.shape.size() > 1 ? bbox_tensor.shape[1] : 0;

  // Match each C++ output box to its nearest Python box of the same label.
  // Gate by a small position radius so unmatched detections (likely caused by
  // score-threshold boundary noise) don't pollute the stats.
  constexpr float kMatchRadius = 1.0f;  // meters
  std::vector<bool> py_matched(py_count, false);

  double sum_center = 0.0;
  double sum_size = 0.0;
  double sum_yaw = 0.0;
  double sum_vel = 0.0;
  double sum_score = 0.0;
  int match_count = 0;
  int unmatched_count = 0;

  struct MatchedPair {
    int cpp_idx;
    int py_idx;
    float dist;
  };
  std::vector<MatchedPair> pairs;
  pairs.reserve(cpp_boxes.size());

  for (size_t j = 0; j < cpp_boxes.size(); ++j) {
    const auto& box = cpp_boxes[j];
    int best = -1;
    float best_dist = 1e9f;
    for (size_t i = 0; i < py_count; ++i) {
      if (py_matched[i] || py_label[i] != box.id) {
        continue;
      }
      float dx = box.position.x - py_bbox[i * py_dim + 0];
      float dy = box.position.y - py_bbox[i * py_dim + 1];
      float dz = box.position.z - py_bbox[i * py_dim + 2];
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < best_dist) {
        best_dist = dist;
        best = static_cast<int>(i);
      }
    }
    if (best < 0 || best_dist > kMatchRadius) {
      ++unmatched_count;
      continue;
    }

    py_matched[best] = true;
    sum_center += best_dist;
    if (py_dim >= 6) {
      sum_size += std::abs(box.size.w - py_bbox[best * py_dim + 3]) +
                  std::abs(box.size.l - py_bbox[best * py_dim + 4]) +
                  std::abs(box.size.h - py_bbox[best * py_dim + 5]);
    }
    if (py_dim >= 7) {
      sum_yaw += std::abs(
          NormalizeAngleDiff(box.z_rotation - py_bbox[best * py_dim + 6]));
    }
    if (py_dim >= 9) {
      sum_vel += std::abs(box.velocity.vx - py_bbox[best * py_dim + 7]) +
                 std::abs(box.velocity.vy - py_bbox[best * py_dim + 8]);
    }
    sum_score += std::abs(box.score - py_score[best]);
    ++match_count;
    pairs.push_back({static_cast<int>(j), best, best_dist});
  }

  AINFO << "\n===== [Compare Final BBoxes] =====";
  AINFO << "  Python boxes   : " << py_count;
  AINFO << "  C++ boxes      : " << cpp_boxes.size();
  AINFO << "  Matched        : " << match_count << "  (radius=" << kMatchRadius
        << "m, same-label)";
  AINFO << "  C++ unmatched  : " << unmatched_count;
  if (match_count > 0) {
    AINFO << std::fixed << std::setprecision(6)
          << "  Mean center L2 : " << sum_center / match_count;
    AINFO << std::fixed << std::setprecision(6)
          << "  Mean size L1   : " << sum_size / match_count;
    AINFO << std::fixed << std::setprecision(6)
          << "  Mean yaw abs   : " << sum_yaw / match_count;
    AINFO << std::fixed << std::setprecision(6)
          << "  Mean vel L1    : " << sum_vel / match_count;
    AINFO << std::fixed << std::setprecision(6)
          << "  Mean score abs : " << sum_score / match_count;
    AINFO << "  --- per-matched-pair ---";
    for (const auto& p : pairs) {
      const auto& c = cpp_boxes[p.cpp_idx];
      const float* b = py_bbox + p.py_idx * py_dim;
      float pw = py_dim >= 6 ? b[3] : 0.f;
      float pl = py_dim >= 6 ? b[4] : 0.f;
      float ph = py_dim >= 6 ? b[5] : 0.f;
      float pyaw = py_dim >= 7 ? b[6] : 0.f;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << "  label=" << c.id
          << " dist=" << p.dist << "  size cpp=(" << c.size.w << "," << c.size.l
          << "," << c.size.h << ") py=(" << pw << "," << pl << "," << ph << ")"
          << "  size_L1="
          << (std::abs(c.size.w - pw) + std::abs(c.size.l - pl) +
              std::abs(c.size.h - ph))
          << "  yaw cpp=" << c.z_rotation << " py=" << pyaw
          << "  score cpp=" << c.score << " py=" << py_score[p.py_idx];
      AINFO << oss.str();
    }
  }
  AINFO << "===============================\n";
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
