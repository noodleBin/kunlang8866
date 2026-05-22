/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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

#include <thrust/functional.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/sort.h>
#include <thrust/transform.h>

#include <memory>
#include <vector>

#include "modules/perception/inference/tensorrt/plugins/kernels.h"
#include "modules/perception/inference/tensorrt/plugins/rcnn_proposal_plugin.h"

namespace century {
namespace perception {
namespace inference {

namespace {

constexpr int kBoxDim = 4;
constexpr int kImInfoDim = 6;
constexpr int kNumSliceAxes = 4;
constexpr float kDefaultValue = -1.0f;

template <typename T>
class DeviceMemory {
 public:
  explicit DeviceMemory(size_t size) : size_(size) {
    BASE_CUDA_CHECK(
        cudaMalloc(reinterpret_cast<void **>(&ptr_), size_ * sizeof(T)));
  }

  ~DeviceMemory() {
    if (nullptr != ptr_) {
      cudaFree(ptr_);
    }
  }

  DeviceMemory(const DeviceMemory&) = delete;
  DeviceMemory& operator=(const DeviceMemory&) = delete;

  DeviceMemory(DeviceMemory&& other) noexcept
      : ptr_(other.ptr_), size_(other.size_) {
    other.ptr_ = nullptr;
    other.size_ = 0;
  }

  DeviceMemory& operator=(DeviceMemory&& other) noexcept {
    if (&other != this) {
      if (nullptr != ptr_) {
        cudaFree(ptr_);
      }
      ptr_ = other.ptr_;
      size_ = other.size_;
      other.ptr_ = nullptr;
      other.size_ = 0;
    }
    return *this;
  }

  T* get() const { return ptr_; }
  size_t size() const { return size_; }

 private:
  T* ptr_ = nullptr;
  size_t size_;
};

template <typename T>
class HostMemory {
 public:
  explicit HostMemory(size_t size) : size_(size) {
    ptr_ = new T[size_]();
  }

  ~HostMemory() {
    if (nullptr != ptr_) {
      delete[] ptr_;
    }
  }

  HostMemory(const HostMemory&) = delete;
  HostMemory& operator=(const HostMemory&) = delete;

  HostMemory(HostMemory&& other) noexcept : ptr_(other.ptr_), size_(other.size_) {
    other.ptr_ = nullptr;
    other.size_ = 0;
  }

  HostMemory& operator=(HostMemory&& other) noexcept {
    if (&other != this) {
      if (nullptr != ptr_) {
        delete[] ptr_;
      }
      ptr_ = other.ptr_;
      size_ = other.size_;
      other.ptr_ = nullptr;
      other.size_ = 0;
    }
    return *this;
  }

  T* get() const { return ptr_; }
  size_t size() const { return size_; }

 private:
  T* ptr_ = nullptr;
  size_t size_;
};

}  // namespace

__global__ void get_rois_nums_kernel(const int nthreads, const float* rois,
                                      int* batch_rois_nums) {
  const int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    const int batch_id = static_cast<int>(rois[index * 5]);
    if (0 <= batch_id) {
      atomicAdd(&batch_rois_nums[batch_id], 1);
    }
  }
}

__global__ void transpose_bbox_pred_kernel(const int nthreads,
                                            const float* bbox_pred,
                                            const int box_len,
                                            const int num_class,
                                            float* out_bbox_pred) {
  const int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    const int roi_id = index / num_class / box_len;
    const int class_id = (index / box_len) % num_class;
    const int feature_id = index % box_len;

    const int in_index =
        roi_id * box_len * num_class + feature_id * num_class + class_id;
    out_bbox_pred[index] = bbox_pred[in_index];
  }
}

__global__ void get_max_score_kernel(const int nthreads, const float* bbox_pred,
                                      const float* scores, const int num_class,
                                      const float threshold_objectness,
                                      const float* class_thresholds,
                                      float* out_bbox_pred, float* out_scores,
                                      float* out_all_probs, int* filter_count) {
  const int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (nthreads <= index) {
    return;
  }

  const int box_id = index;
  if ((1.0f - scores[box_id * (num_class + 1)]) < threshold_objectness) {
    return;
  }

  float score_max = -FLT_MAX;
  int cls_max = -1;
  for (int c = 0; c < num_class; ++c) {
    const float score =
        scores[box_id * (num_class + 1) + c + 1] -
        class_thresholds[c];
    if (score > score_max) {
      score_max = score;
      cls_max = c;
    }
  }

  if (0 > score_max) {
    return;
  }

  const int counter = atomicAdd(filter_count, 1);
  const int box_cls_id = box_id * (num_class + 1) + cls_max + 1;
  for (int i = 0; i < kBoxDim; ++i) {
    out_bbox_pred[counter * kBoxDim + i] = bbox_pred[box_cls_id * kBoxDim + i];
  }
  out_scores[counter] = scores[box_cls_id];
  for (int i = 0; i < num_class + 1; ++i) {
    out_all_probs[counter * (num_class + 1) + i] =
        scores[box_id * (num_class + 1) + i];
  }
}

int RCNNProposalPlugin::enqueue(int batchSize, const void* const* inputs,
                                 void** outputs, void* workspace,
                                 cudaStream_t stream) {
  const float* cls_score_softmax =
      reinterpret_cast<const float*>(inputs[0]);
  const float* bbox_pred =
      reinterpret_cast<const float*>(inputs[1]);
  const float* rois = reinterpret_cast<const float*>(inputs[2]);
  const float* im_info =
      reinterpret_cast<const float*>(inputs[3]);
  float* result_boxes = reinterpret_cast<float*>(outputs[0]);

  const int bbox_pred_size = num_rois_ * kBoxDim * kBoxDim;
  const int output_size = batchSize * top_n_ * out_channel_;

  InitializeResultBoxes(result_boxes, output_size, stream);

  float origin_height, origin_width;
  CopyAndExtractImInfo(im_info, batchSize, stream, &origin_height,
                        &origin_width);

  DeviceMemory<float> dev_thresholds(num_class_);
  CopyClassThresholdsToDevice(dev_thresholds.get(), stream);

  DeviceMemory<float> dev_bbox_mean(kBoxDim);
  DeviceMemory<float> dev_bbox_std(kBoxDim);
  DeviceMemory<float> norm_bbox_pred(bbox_pred_size);
  NormalizeBboxPred(bbox_pred, bbox_pred_size, dev_bbox_mean.get(),
                     dev_bbox_std.get(), norm_bbox_pred.get(), stream);

  DeviceMemory<int> dev_slice_axis(kNumSliceAxes);
  DeviceMemory<float> sliced_rois(num_rois_ * kBoxDim);
  SliceRois(rois, dev_slice_axis.get(), sliced_rois.get(), stream);

  DeviceMemory<float> decoded_bbox_pred(bbox_pred_size);
  DecodeBboxes(sliced_rois.get(), norm_bbox_pred.get(), decoded_bbox_pred.get(),
               stream);

  if (refine_out_of_map_bbox_) {
    ClipBoxes(decoded_bbox_pred.get(), bbox_pred_size, origin_height,
              origin_width, stream);
  }

  HostMemory<int> batch_rois_nums(batchSize);
  CountRoisPerBatch(rois, batch_rois_nums.get(), batchSize, stream);

  ProcessBatches(cls_score_softmax, decoded_bbox_pred.get(),
                  batch_rois_nums.get(), batchSize, result_boxes, stream);

  return 0;
}

void RCNNProposalPlugin::InitializeResultBoxes(float* result_boxes,
                                                int output_size,
                                                cudaStream_t stream) {
  HostMemory<float> init_result_boxes(output_size);
  std::fill_n(init_result_boxes.get(), output_size, kDefaultValue);
  BASE_CUDA_CHECK(cudaMemcpyAsync(
      result_boxes, init_result_boxes.get(), output_size * sizeof(float),
      cudaMemcpyHostToDevice, stream));
}

void RCNNProposalPlugin::CopyAndExtractImInfo(const float* im_info,
                                               int batch_size,
                                               cudaStream_t stream,
                                               float* origin_height,
                                               float* origin_width) {
  HostMemory<float> host_im_info(batch_size * kImInfoDim);
  BASE_CUDA_CHECK(cudaMemcpyAsync(host_im_info.get(), im_info,
                                   batch_size * kImInfoDim * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream));
  *origin_height = host_im_info.get()[0];
  *origin_width = host_im_info.get()[1];
}

void RCNNProposalPlugin::CopyClassThresholdsToDevice(float* dev_thresholds,
                                                      cudaStream_t stream) {
  HostMemory<float> host_thresholds(num_class_);
  for (int i = 0; i < num_class_; ++i) {
    host_thresholds.get()[i] = thresholds_[i];
  }
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_thresholds, host_thresholds.get(),
                                   num_class_ * sizeof(float),
                                   cudaMemcpyHostToDevice, stream));
}

void RCNNProposalPlugin::NormalizeBboxPred(const float* bbox_pred,
                                            int bbox_pred_size,
                                            float* dev_bbox_mean,
                                            float* dev_bbox_std,
                                            float* norm_bbox_pred,
                                            cudaStream_t stream) {
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_bbox_mean, bbox_mean_, kBoxDim * sizeof(float),
                                   cudaMemcpyHostToDevice, stream));
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_bbox_std, bbox_std_, kBoxDim * sizeof(float),
                                   cudaMemcpyHostToDevice, stream));
  BASE_CUDA_CHECK(cudaMemcpyAsync(norm_bbox_pred, bbox_pred,
                                   bbox_pred_size * sizeof(float),
                                   cudaMemcpyDeviceToDevice, stream));

  const int nthreads = bbox_pred_size;
  const int block_size = DIVUP(nthreads, thread_size_);
  repeatedly_mul_cuda(block_size, thread_size_, 0, stream, nthreads,
                      norm_bbox_pred, norm_bbox_pred, dev_bbox_std, kBoxDim);
  repeatedly_add_cuda(block_size, thread_size_, 0, stream, nthreads,
                      norm_bbox_pred, norm_bbox_pred, dev_bbox_mean, kBoxDim);
}

void RCNNProposalPlugin::SliceRois(const float* rois, int* dev_slice_axis,
                                    float* sliced_rois, cudaStream_t stream) {
  const int slice_axis[kNumSliceAxes] = {1, 2, 3, 4};
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_slice_axis, slice_axis,
                                   kNumSliceAxes * sizeof(int),
                                   cudaMemcpyHostToDevice, stream));

  const int nthreads = num_rois_ * kBoxDim;
  const int block_size = DIVUP(nthreads, thread_size_);
  slice2d_cuda(block_size, thread_size_, 0, stream, nthreads, rois,
               sliced_rois, dev_slice_axis, kBoxDim,
               5);
}

void RCNNProposalPlugin::DecodeBboxes(const float* sliced_rois,
                                       const float* norm_bbox_pred,
                                       float* decoded_bbox_pred,
                                       cudaStream_t stream) {
  const int bbox_pred_size = num_rois_ * kBoxDim;
  BASE_CUDA_CHECK(cudaMemsetAsync(decoded_bbox_pred, 0,
                                   bbox_pred_size * sizeof(float), stream));

  const int nthreads = num_rois_ * kBoxDim;
  const int block_size = DIVUP(nthreads, thread_size_);
  bbox_transform_inv_cuda(block_size, thread_size_, 0, stream, nthreads,
                          sliced_rois, norm_bbox_pred, num_rois_, kBoxDim,
                          decoded_bbox_pred);
}

void RCNNProposalPlugin::ClipBoxes(float* decoded_bbox_pred, int bbox_pred_size,
                                    float origin_height, float origin_width,
                                    cudaStream_t stream) {
  const int nthreads = bbox_pred_size;
  const int block_size = DIVUP(nthreads, thread_size_);
  clip_boxes_cuda(block_size, thread_size_, 0, stream, nthreads,
                  decoded_bbox_pred, origin_height, origin_width);
}

void RCNNProposalPlugin::CountRoisPerBatch(const float* rois,
                                            int* batch_rois_nums,
                                            int batch_size,
                                            cudaStream_t stream) {
  DeviceMemory<int> dev_batch_rois_nums(batch_size);
  BASE_CUDA_CHECK(cudaMemsetAsync(dev_batch_rois_nums.get(), 0,
                                   batch_size * sizeof(int), stream));

  const int nthreads = num_rois_;
  const int block_size = DIVUP(nthreads, thread_size_);
  get_rois_nums_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads, rois, dev_batch_rois_nums.get());

  BASE_CUDA_CHECK(cudaMemcpyAsync(batch_rois_nums, dev_batch_rois_nums.get(),
                                   batch_size * sizeof(int),
                                   cudaMemcpyDeviceToHost, stream));
}

void RCNNProposalPlugin::ProcessBatches(const float* cls_score_softmax,
                                         const float* decoded_bbox_pred,
                                         const int* batch_rois_nums,
                                         int batch_size, float* result_boxes,
                                         cudaStream_t stream) {
  DeviceMemory<float> max_bbox(max_candidate_n_ * kBoxDim);
  DeviceMemory<float> max_score(max_candidate_n_);
  DeviceMemory<float> max_all_probs(max_candidate_n_ * (num_class_ + 1));
  DeviceMemory<int> max_filtered_count(1);

  DeviceMemory<float> filtered_bbox(max_candidate_n_ * kBoxDim);
  DeviceMemory<float> filtered_score(max_candidate_n_);
  DeviceMemory<float> filtered_all_probs(max_candidate_n_ * (num_class_ + 1));
  DeviceMemory<int> filtered_count(1);

  DeviceMemory<int> sorted_indexes(max_candidate_n_);

  DeviceMemory<float> pre_nms_bbox(max_candidate_n_ * kBoxDim);
  DeviceMemory<float> pre_nms_score(max_candidate_n_);
  DeviceMemory<float> pre_nms_all_probs(max_candidate_n_ * (num_class_ + 1));

  acc_box_num_ = 0;
  for (int batch_id = 0; batch_id < batch_size; ++batch_id) {
    const int cur_ptr = batch_id * 300;
    ProcessSingleBatch(batch_id, cur_ptr, cls_score_softmax,
                       decoded_bbox_pred, batch_rois_nums, max_bbox.get(),
                       max_score.get(), max_all_probs.get(),
                       max_filtered_count.get(), filtered_bbox.get(),
                       filtered_score.get(), filtered_all_probs.get(),
                       filtered_count.get(), sorted_indexes.get(),
                       pre_nms_bbox.get(), pre_nms_score.get(),
                       pre_nms_all_probs.get(), result_boxes, stream);
  }
}

void RCNNProposalPlugin::ProcessSingleBatch(
    int batch_id, int cur_ptr, const float* cls_score_softmax,
    const float* decoded_bbox_pred, const int* batch_rois_nums,
    float* max_bbox, float* max_score, float* max_all_probs,
    int* max_filtered_count, float* filtered_bbox, float* filtered_score,
    float* filtered_all_probs, int* filtered_count, int* sorted_indexes,
    float* pre_nms_bbox, float* pre_nms_score, float* pre_nms_all_probs,
    float* result_boxes, cudaStream_t stream) {
  ResetBuffers(max_bbox, max_score, max_all_probs, max_filtered_count, stream);

  const int nthreads = batch_rois_nums[batch_id];
  const int block_size = DIVUP(nthreads, thread_size_);
  get_max_score_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads,
      decoded_bbox_pred + size_t(cur_ptr * (num_class_ + 1) * kBoxDim),
      cls_score_softmax + size_t(cur_ptr * (num_class_ + 1)), num_class_,
      threshold_objectness_, thresholds_.data(), max_bbox, max_score,
      max_all_probs, max_filtered_count);

  int host_max_filtered_count = 0;
  BASE_CUDA_CHECK(cudaMemcpyAsync(&host_max_filtered_count, max_filtered_count,
                                   sizeof(int), cudaMemcpyDeviceToHost,
                                   stream));
  if (0 == host_max_filtered_count) {
    return;
  }

  ResetBuffers(filtered_bbox, filtered_score, filtered_all_probs, filtered_count,
               stream);

  const int filter_threads = host_max_filtered_count;
  const int filter_block_size = DIVUP(filter_threads, thread_size_);
  filter_boxes_cuda(filter_block_size, thread_size_, 0, stream,
                    filter_threads, max_bbox, max_score, max_all_probs,
                    host_max_filtered_count, 1, 1,
                    num_class_ + 1, 0, 0, min_size_mode_,
                    min_size_h_, min_size_w_, 0.0f, filtered_bbox,
                    filtered_score, filtered_all_probs, filtered_count);

  int host_filtered_count = 0;
  BASE_CUDA_CHECK(cudaMemcpyAsync(&host_filtered_count, filtered_count,
                                   sizeof(int), cudaMemcpyDeviceToHost,
                                   stream));
  if (0 == host_filtered_count) {
    return;
  }

  thrust::sequence(thrust::device, sorted_indexes,
                   sorted_indexes + host_filtered_count);
  thrust::sort_by_key(thrust::device, filtered_score,
                      filtered_score + size_t(host_filtered_count),
                      sorted_indexes, thrust::greater<float>());

  ResetBuffers(pre_nms_bbox, pre_nms_score, pre_nms_all_probs, nullptr, stream);

  const int keep_threads = std::min(max_candidate_n_, host_filtered_count);
  const int keep_block_size = DIVUP(keep_threads, thread_size_);
  keep_topN_boxes_cuda(keep_block_size, thread_size_, 0, stream,
                       keep_threads, filtered_bbox, filtered_score,
                       filtered_all_probs, sorted_indexes, filtered_count,
                       rpn_proposal_output_score_, max_candidate_n_,
                       num_class_ + 1, max_candidate_n_, pre_nms_bbox,
                       pre_nms_score, pre_nms_all_probs);

  const int cur_filter_count = std::min(host_filtered_count, max_candidate_n_);
  NmsForward(rpn_proposal_output_score_, cur_filter_count, kBoxDim,
             overlap_ratio_, max_candidate_n_, top_n_, batch_id,
             num_class_ + 1, pre_nms_bbox, pre_nms_score,
             pre_nms_all_probs,
             result_boxes + size_t(acc_box_num_ * out_channel_),
             &acc_box_num_, stream);
}

void RCNNProposalPlugin::ResetBuffers(float* bbox, float* score,
                                       float* all_probs, int* count,
                                       cudaStream_t stream) {
  if (nullptr != bbox) {
    BASE_CUDA_CHECK(
        cudaMemsetAsync(bbox, 0, max_candidate_n_ * kBoxDim * sizeof(float),
                        stream));
  }
  if (nullptr != score) {
    BASE_CUDA_CHECK(cudaMemsetAsync(score, 0,
                                     max_candidate_n_ * sizeof(float), stream));
  }
  if (nullptr != all_probs) {
    BASE_CUDA_CHECK(cudaMemsetAsync(
        all_probs, 0,
        max_candidate_n_ * (num_class_ + 1) * sizeof(float), stream));
  }
  if (nullptr != count) {
    BASE_CUDA_CHECK(cudaMemsetAsync(count, 0, sizeof(int), stream));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace century
