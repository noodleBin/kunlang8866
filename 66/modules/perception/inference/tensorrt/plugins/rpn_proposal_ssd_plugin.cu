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

#include <thrust/sort.h>

#include "modules/perception/inference/tensorrt/plugins/kernels.h"
#include "modules/perception/inference/tensorrt/plugins/rpn_proposal_ssd_plugin.h"

namespace century {
namespace perception {
namespace inference {

// Constants for bounding box and score dimensions
constexpr int kBboxCoordCount = 4;  // x_min, y_min, x_max, y_max
constexpr int kClassScoreCount = 2;  // background, foreground
constexpr float kAnchorCenterOffsetFactor = 0.5f;  // For center to min/max conversion
constexpr int kRoiOutputCount = 5;  // batch_id, x_min, y_min, x_max, y_max
constexpr int kImInfoCount = 6;  // origin_height, origin_width, scale, ...
constexpr float kInitRoisValue = -1.0f;  // Initial value for output ROIs

// TODO(chenjiahao): add heat_map_b as anchor_offset
// output anchors dims: [H, W, num_anchor_per_point, 4]
__global__ void generate_anchors_kernel(const int height, const int width,
                                        const float anchor_stride,
                                        const int num_anchor_per_point,
                                        const float *anchor_heights,
                                        const float *anchor_widths,
                                        float *anchors) {
  int thread_index = threadIdx.x + blockIdx.x * blockDim.x;
  const int num_anchor = height * width * num_anchor_per_point;
  if (thread_index >= num_anchor) {
    return;
  }

  float anchor_offset = 0;
  int position_index = thread_index / num_anchor_per_point;
  int anchor_index = thread_index % num_anchor_per_point;
  int width_index = position_index % width;
  int height_index = position_index / width;

  float center_x = width_index * anchor_stride + anchor_offset;
  float center_y = height_index * anchor_stride + anchor_offset;

  float x_min = center_x - kAnchorCenterOffsetFactor * (anchor_widths[anchor_index] - 1);
  float y_min = center_y - kAnchorCenterOffsetFactor * (anchor_heights[anchor_index] - 1);
  float x_max = center_x + kAnchorCenterOffsetFactor * (anchor_widths[anchor_index] - 1);
  float y_max = center_y + kAnchorCenterOffsetFactor * (anchor_heights[anchor_index] - 1);

  anchors[thread_index * kBboxCoordCount] = x_min;
  anchors[thread_index * kBboxCoordCount + 1] = y_min;
  anchors[thread_index * kBboxCoordCount + 2] = x_max;
  anchors[thread_index * kBboxCoordCount + 3] = y_max;
}

// in_boxes dims: [N, num_box_per_point * 4, H, W],
// out_boxes dims: [N, H * W * num_box_per_point， 4]
template <typename Dtype>
__global__ void reshape_boxes_kernel(const int nthreads, const Dtype *in_boxes,
                                     const int height, const int width,
                                     const int num_box_per_point,
                                     Dtype *out_boxes) {
  int thread_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (thread_index < nthreads) {
    int num_point = height * width;

    int batch_id = thread_index / num_point / num_box_per_point / kBboxCoordCount;
    int feature_id = thread_index % kBboxCoordCount;
    int box_index = (thread_index / kBboxCoordCount) % num_box_per_point;
    int point_index = (thread_index / num_box_per_point / kBboxCoordCount) % num_point;

    int in_index =
        ((batch_id * num_box_per_point + box_index) * kBboxCoordCount + feature_id) *
            num_point +
        point_index;
    out_boxes[thread_index] = in_boxes[in_index];
  }
}

// in_scores dims: [N, 2 * num_box_per_point, H, W],
// out_scores dims: [N, H * W * num_box_per_point, 2]
template <typename Dtype>
__global__ void reshape_scores_kernel(const int nthreads,
                                      const Dtype *in_scores, const int height,
                                      const int width,
                                      const int num_box_per_point,
                                      Dtype *out_scores) {
  int thread_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (thread_index < nthreads) {
    int num_point = height * width;

    int batch_id = thread_index / num_point / num_box_per_point / kClassScoreCount;
    int class_id = thread_index % kClassScoreCount;
    int box_index = (thread_index / kClassScoreCount) % num_box_per_point;
    int point_index = (thread_index / num_box_per_point / kClassScoreCount) % num_point;

    int in_index =
        ((batch_id * kClassScoreCount + class_id) * num_box_per_point + box_index) *
            num_point +
        point_index;
    out_scores[thread_index] = in_scores[in_index];
  }
}

void RPNProposalSSDPlugin::InitializeOutputBuffers(float *out_rois,
                                                   int out_rois_size,
                                                   cudaStream_t stream) {
  float *init_out_rois = new float[out_rois_size]();
  std::fill_n(init_out_rois, out_rois_size, kInitRoisValue);
  BASE_CUDA_CHECK(cudaMemcpyAsync(out_rois, init_out_rois,
                                  out_rois_size * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  delete[] init_out_rois;
}

void RPNProposalSSDPlugin::ReshapeBboxPred(const float *rpn_bbox_pred,
                                           int batch_size,
                                           float *temp_rpn_bbox_pred,
                                           cudaStream_t stream) {
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batch_size * num_anchor * kBboxCoordCount;
  int nthreads = rpn_bbox_pred_size;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  reshape_boxes_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads, rpn_bbox_pred, height_, width_, num_anchor_per_point_,
      temp_rpn_bbox_pred);

  float *dev_bbox_mean = nullptr;
  float *dev_bbox_std = nullptr;
  BASE_CUDA_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_bbox_mean), kBboxCoordCount * sizeof(float)));
  BASE_CUDA_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_bbox_std), kBboxCoordCount * sizeof(float)));
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_bbox_mean, bbox_mean_, kBboxCoordCount * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_bbox_std, bbox_std_, kBboxCoordCount * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  repeatedly_mul_cuda(block_size, thread_size_, 0, stream, nthreads,
                      temp_rpn_bbox_pred, temp_rpn_bbox_pred, dev_bbox_std, kBboxCoordCount);
  repeatedly_add_cuda(block_size, thread_size_, 0, stream, nthreads,
                      temp_rpn_bbox_pred, temp_rpn_bbox_pred, dev_bbox_mean, kBboxCoordCount);
  BASE_CUDA_CHECK(cudaFree(dev_bbox_mean));
  BASE_CUDA_CHECK(cudaFree(dev_bbox_std));
}

void RPNProposalSSDPlugin::GenerateAnchors(float *anchors,
                                           cudaStream_t stream) {
  float *dev_anchor_heights = nullptr;
  float *dev_anchor_widths = nullptr;
  int anchors_size = height_ * width_ * num_anchor_per_point_ * kBboxCoordCount;

  BASE_CUDA_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_anchor_heights),
                 num_anchor_per_point_ * sizeof(float)));
  BASE_CUDA_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_anchor_widths),
                 num_anchor_per_point_ * sizeof(float)));
  BASE_CUDA_CHECK(cudaMemsetAsync(
      dev_anchor_heights, 0, num_anchor_per_point_ * sizeof(float), stream));
  BASE_CUDA_CHECK(cudaMemsetAsync(
      dev_anchor_widths, 0, num_anchor_per_point_ * sizeof(float), stream));
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_anchor_heights, anchor_heights_,
                                  num_anchor_per_point_ * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));
  BASE_CUDA_CHECK(cudaMemcpyAsync(dev_anchor_widths, anchor_widths_,
                                  num_anchor_per_point_ * sizeof(float),
                                  cudaMemcpyHostToDevice, stream));

  int block_size = (anchors_size - 1) / thread_size_ + 1;
  generate_anchors_kernel<<<block_size, thread_size_, 0, stream>>>(
      height_, width_, heat_map_a_, num_anchor_per_point_, dev_anchor_heights,
      dev_anchor_widths, anchors);

  BASE_CUDA_CHECK(cudaFree(dev_anchor_heights));
  BASE_CUDA_CHECK(cudaFree(dev_anchor_widths));
}

void RPNProposalSSDPlugin::DecodeBboxes(const float *anchors,
                                        const float *temp_rpn_bbox_pred,
                                        int batch_size, float *proposals,
                                        cudaStream_t stream) {
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batch_size * num_anchor * kBboxCoordCount;
  BASE_CUDA_CHECK(cudaMemsetAsync(proposals, 0,
                                  rpn_bbox_pred_size * sizeof(float), stream));
  int nthreads = batch_size * num_anchor;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  bbox_transform_inv_cuda(block_size, thread_size_, 0, stream, nthreads,
                          anchors, temp_rpn_bbox_pred, num_anchor, 1,
                          proposals);
}

void RPNProposalSSDPlugin::ClipBboxes(float *proposals, int rpn_bbox_pred_size,
                                      float origin_height, float origin_width,
                                      cudaStream_t stream) {
  int nthreads = rpn_bbox_pred_size;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  clip_boxes_cuda(block_size, thread_size_, 0, stream, nthreads, proposals,
                  origin_height, origin_width);
}

void RPNProposalSSDPlugin::FilterBboxes(const float *proposals,
                                        const float *temp_scores, int batch_size,
                                        float *filtered_proposals,
                                        float *filtered_scores,
                                        int *filtered_count,
                                        cudaStream_t stream) {
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batch_size * num_anchor * kBboxCoordCount;
  BASE_CUDA_CHECK(cudaMemsetAsync(filtered_proposals, 0,
                                  rpn_bbox_pred_size * sizeof(float), stream));
  BASE_CUDA_CHECK(cudaMemsetAsync(
      filtered_scores, 0, batch_size * num_anchor * sizeof(float), stream));
  BASE_CUDA_CHECK(
      cudaMemsetAsync(filtered_count, 0, batch_size * sizeof(int), stream));
  int nthreads = batch_size * num_anchor;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  // TODO(chenjiahao): filter area
  filter_boxes_cuda(block_size, thread_size_, 0, stream, nthreads, proposals,
                    temp_scores, nullptr, num_anchor, 1, kClassScoreCount, 0, 0, 1,
                    min_size_mode_, min_size_h_, min_size_w_,
                    threshold_objectness_, filtered_proposals, filtered_scores,
                    nullptr, filtered_count);
}

void RPNProposalSSDPlugin::SortAndKeepTopN(
    const float *filtered_proposals, float *filtered_scores, int batch_size,
    int num_anchor, const int *filtered_count, int *host_filtered_count,
    float *pre_nms_proposals, int *sorted_indexes, cudaStream_t stream) {
  BASE_CUDA_CHECK(cudaMemcpyAsync(host_filtered_count, filtered_count,
                                  batch_size * sizeof(int),
                                  cudaMemcpyDeviceToHost, stream));

  for (int i = 0; batch_size > i; ++i) {
    thrust::sequence(thrust::device, sorted_indexes + i * num_anchor,
                     sorted_indexes + i * num_anchor + host_filtered_count[i]);
    thrust::sort_by_key(
        thrust::device, filtered_scores + size_t(i * num_anchor),
        filtered_scores + size_t(i * num_anchor + host_filtered_count[i]),
        sorted_indexes + i * num_anchor, thrust::greater<float>());
  }

  BASE_CUDA_CHECK(cudaMemsetAsync(
      pre_nms_proposals, 0, batch_size * max_candidate_n_ * kBboxCoordCount * sizeof(float),
      stream));
  int nthreads = batch_size * max_candidate_n_;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  keep_topN_boxes_cuda(block_size, thread_size_, 0, stream, nthreads,
                       filtered_proposals, nullptr, nullptr, sorted_indexes,
                       filtered_count, false, num_anchor, 0, max_candidate_n_,
                       pre_nms_proposals, nullptr, nullptr);
}

void RPNProposalSSDPlugin::ApplyNmsAndOutput(int batch_size,
                                             const int *host_filtered_count,
                                             float *pre_nms_proposals,
                                             float *out_rois,
                                             cudaStream_t stream) {
  int acc_box_num = 0;
  for (int i = 0; batch_size > i; ++i) {
    int cur_filter_count = std::min(host_filtered_count[i], max_candidate_n_);
    NmsForward(false, cur_filter_count, kBboxCoordCount, overlap_ratio_, max_candidate_n_,
               top_n_, i, 0, pre_nms_proposals + size_t(i * max_candidate_n_ * kBboxCoordCount),
               nullptr, nullptr, out_rois + size_t(acc_box_num * kRoiOutputCount),
               &acc_box_num, stream);
  }
  out_rois_num_ = acc_box_num;
}

void RPNProposalSSDPlugin::AllocateAndReshapeScores(
    const float *rpn_cls_prob_reshape, int batch_size, float **temp_scores,
    cudaStream_t stream) {
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int scores_size = batch_size * num_anchor * kClassScoreCount;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(temp_scores),
                             scores_size * sizeof(float)));
  int nthreads = scores_size;
  int block_size = (nthreads - 1) / thread_size_ + 1;
  reshape_scores_kernel<<<block_size, thread_size_, 0, stream>>>(
      nthreads, rpn_cls_prob_reshape, height_, width_, num_anchor_per_point_,
      *temp_scores);
}

void RPNProposalSSDPlugin::AllocateFilteredBuffers(
    int batch_size, float **filtered_proposals, float **filtered_scores,
    int **filtered_count, cudaStream_t stream) {
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batch_size * num_anchor * kBboxCoordCount;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(filtered_proposals),
                             rpn_bbox_pred_size * sizeof(float)));
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(filtered_scores),
                             batch_size * num_anchor * sizeof(float)));
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(filtered_count),
                             batch_size * sizeof(int)));
}

int RPNProposalSSDPlugin::enqueue(int batchSize, const void *const *inputs,
                                  void **outputs, void *workspace,
                                  cudaStream_t stream) {
  // dimsNCHW: [N, 2 * num_anchor_per_point, H, W]
  const float *rpn_cls_prob_reshape =
      reinterpret_cast<const float *>(inputs[0]);
  // dimsNCHW: [N, num_anchor_per_point * 4, H, W]
  const float *rpn_bbox_pred = reinterpret_cast<const float *>(inputs[1]);
  // dims: [N, 6, 1, 1]
  const float *im_info = reinterpret_cast<const float *>(inputs[2]);
  float *out_rois = reinterpret_cast<float *>(outputs[0]);

  float *host_im_info = new float[batchSize * kImInfoCount]();
  BASE_CUDA_CHECK(cudaMemcpyAsync(host_im_info, im_info,
                                  batchSize * kImInfoCount * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream));

  const int origin_height = static_cast<int>(host_im_info[0]);
  const int origin_width = static_cast<int>(host_im_info[1]);
  int num_anchor = height_ * width_ * num_anchor_per_point_;
  int rpn_bbox_pred_size = batchSize * num_anchor * kBboxCoordCount;
  int anchors_size = num_anchor * kBboxCoordCount;
  int out_rois_size = batchSize * top_n_ * kRoiOutputCount;

  // Using thrust::fill might cause crash
  InitializeOutputBuffers(out_rois, out_rois_size, stream);

  // reshape to [N, num_anchor, 4]
  float *temp_rpn_bbox_pred = nullptr;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&temp_rpn_bbox_pred),
                             rpn_bbox_pred_size * sizeof(float)));
  ReshapeBboxPred(rpn_bbox_pred, batchSize, temp_rpn_bbox_pred, stream);

  // generate anchors
  float *anchors = nullptr;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&anchors),
                             anchors_size * sizeof(float)));
  BASE_CUDA_CHECK(
      cudaMemsetAsync(anchors, 0, anchors_size * sizeof(float), stream));
  GenerateAnchors(anchors, stream);

  // decode bbox
  float *proposals = nullptr;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&proposals),
                             rpn_bbox_pred_size * sizeof(float)));
  BASE_CUDA_CHECK(cudaMemsetAsync(proposals, 0,
                                  rpn_bbox_pred_size * sizeof(float), stream));
  DecodeBboxes(anchors, temp_rpn_bbox_pred, batchSize, proposals, stream);

  // clip boxes, i.e. refine proposals which are out of map
  if (refine_out_of_map_bbox_) {
    ClipBboxes(proposals, rpn_bbox_pred_size, origin_height, origin_width,
               stream);
  }

  // reshape scores to [N, num_anchor, 2]
  float *temp_scores = nullptr;
  AllocateAndReshapeScores(rpn_cls_prob_reshape, batchSize, &temp_scores, stream);

  // filter boxes according to min_size_mode and threshold_objectness
  float *filtered_proposals = nullptr;
  float *filtered_scores = nullptr;
  int *filtered_count = nullptr;
  AllocateFilteredBuffers(batchSize, &filtered_proposals, &filtered_scores,
                          &filtered_count, stream);
  FilterBboxes(proposals, temp_scores, batchSize, filtered_proposals,
               filtered_scores, filtered_count, stream);

  int *host_filtered_count = new int[batchSize]();

  // descending sort proposals by score, keep max N candidates, NMS
  float *pre_nms_proposals = nullptr;
  int *sorted_indexes = nullptr;
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&sorted_indexes),
                             batchSize * num_anchor * sizeof(int)));
  BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&pre_nms_proposals),
                             batchSize * max_candidate_n_ * kBboxCoordCount * sizeof(float)));
  SortAndKeepTopN(filtered_proposals, filtered_scores, batchSize, num_anchor,
                  filtered_count, host_filtered_count, pre_nms_proposals,
                  sorted_indexes, stream);

  // Nms, keep top N proposals and output final proposals
  // output dims: [num_roi, 5] (axis-1: batch_id, x_min, y_min, x_max, y_max)
  ApplyNmsAndOutput(batchSize, host_filtered_count, pre_nms_proposals, out_rois,
                    stream);

  // Free cuda memory
  BASE_CUDA_CHECK(cudaFree(temp_rpn_bbox_pred));
  BASE_CUDA_CHECK(cudaFree(anchors));
  BASE_CUDA_CHECK(cudaFree(proposals));
  BASE_CUDA_CHECK(cudaFree(temp_scores));
  BASE_CUDA_CHECK(cudaFree(filtered_proposals));
  BASE_CUDA_CHECK(cudaFree(filtered_scores));
  BASE_CUDA_CHECK(cudaFree(filtered_count));
  BASE_CUDA_CHECK(cudaFree(sorted_indexes));
  BASE_CUDA_CHECK(cudaFree(pre_nms_proposals));

  // Free host memory
  delete[] host_im_info;
  delete[] host_filtered_count;

  return 0;
}

}  // namespace inference
}  // namespace perception
}  // namespace century
