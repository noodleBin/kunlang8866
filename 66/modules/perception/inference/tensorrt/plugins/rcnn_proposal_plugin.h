#pragma once

#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace century {
namespace perception {
namespace inference {

class RCNNProposalPlugin : public nvinfer1::IPluginV2 {
 public:
  RCNNProposalPlugin(
      const BBoxRegParameter& bbox_reg_param,
      const DetectionOutputSSDParameter& detection_output_ssd_param,
      nvinfer1::Dims* in_dims) {
    num_rois_ = in_dims[2].d[0];

    for (int i = 0; i < 4; ++i) {
      bbox_mean_[i] = bbox_reg_param.bbox_mean(i);
      bbox_std_[i] = bbox_reg_param.bbox_std(i);
    }

    min_size_mode_ =
        static_cast<int>(detection_output_ssd_param.min_size_mode());
    min_size_h_ = detection_output_ssd_param.min_size_h();
    min_size_w_ = detection_output_ssd_param.min_size_w();

    num_class_ = detection_output_ssd_param.num_class();
    refine_out_of_map_bbox_ =
        detection_output_ssd_param.refine_out_of_map_bbox();
    regress_agnostic_ = detection_output_ssd_param.regress_agnostic();
    rpn_proposal_output_score_ =
        detection_output_ssd_param.rpn_proposal_output_score();

    threshold_objectness_ = detection_output_ssd_param.threshold_objectness();
    for (int i = 0; i < num_class_; ++i) {
      thresholds_.push_back(detection_output_ssd_param.threshold(i));
    }

    NMSSSDParameter nms_param = detection_output_ssd_param.nms_param();
    max_candidate_n_ = nms_param.max_candidate_n(0);
    overlap_ratio_ = nms_param.overlap_ratio(0);
    top_n_ = nms_param.top_n(0);

    out_channel_ = rpn_proposal_output_score_ ? 9 : 5;
  }

  virtual ~RCNNProposalPlugin() override {}

  int initialize() noexcept override { return 0; }
  void terminate() noexcept override {}
  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims* inputs,
                                     int nbInputDims) noexcept override {
    // TODO: complete input dims assertion
    // TODO: batch size is hard coded to 1 here
    return nvinfer1::Dims4(top_n_ * 1, out_channel_, 1, 1);
  }

  void configurePlugin(const nvinfer1::Dims* inputDims, int nbInputs,
                       const nvinfer1::Dims* outputDims, int nbOutputs,
                       int maxBatchSize) noexcept {}

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override {
    return 0;
  }

  int enqueue(int batchSize, const void* const* inputs, void* const* outputs,
              void* workspace, cudaStream_t stream) noexcept override;

  int enqueue(int batchSize, const void* const* inputs, void** outputs,
              void* workspace, cudaStream_t stream);
  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void* buffer) const noexcept override {
    char *d = reinterpret_cast<char*>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  const int thread_size_ = 512;
  bool refine_out_of_map_bbox_ = true;
  bool regress_agnostic_ = false;
  bool rpn_proposal_output_score_ = true;

  float bbox_mean_[4];
  float bbox_std_[4];
  float min_size_h_;
  float min_size_w_;
  float threshold_objectness_;
  float overlap_ratio_;
  int num_class_;
  int num_rois_;
  int max_candidate_n_;
  int min_size_mode_;
  int top_n_;
  int out_channel_;
  int acc_box_num_;

  std::vector<float> thresholds_;

  void InitializeResultBoxes(float* result_boxes, int output_size,
                             cudaStream_t stream);
  void CopyAndExtractImInfo(const float* im_info, int batch_size,
                            cudaStream_t stream, float* origin_height,
                            float* origin_width);
  void CopyClassThresholdsToDevice(float* dev_thresholds, cudaStream_t stream);
  void NormalizeBboxPred(const float* bbox_pred, int bbox_pred_size,
                         float* dev_bbox_mean, float* dev_bbox_std,
                         float* norm_bbox_pred, cudaStream_t stream);
  void SliceRois(const float* rois, int* dev_slice_axis, float* sliced_rois,
                 cudaStream_t stream);
  void DecodeBboxes(const float* sliced_rois, const float* norm_bbox_pred,
                    float* decoded_bbox_pred, cudaStream_t stream);
  void ClipBoxes(float* decoded_bbox_pred, int bbox_pred_size,
                 float origin_height, float origin_width, cudaStream_t stream);
  void CountRoisPerBatch(const float* rois, int* batch_rois_nums,
                         int batch_size, cudaStream_t stream);
  void ProcessBatches(const float* cls_score_softmax,
                      const float* decoded_bbox_pred,
                      const int* batch_rois_nums, int batch_size,
                      float* result_boxes, cudaStream_t stream);
  void ProcessSingleBatch(
      int batch_id, int cur_ptr, const float* cls_score_softmax,
      const float* decoded_bbox_pred, const int* batch_rois_nums,
      float* max_bbox, float* max_score, float* max_all_probs,
      int* max_filtered_count, float* filtered_bbox, float* filtered_score,
      float* filtered_all_probs, int* filtered_count, int* sorted_indexes,
      float* pre_nms_bbox, float* pre_nms_score, float* pre_nms_all_probs,
      float* result_boxes, cudaStream_t stream);
  void ResetBuffers(float* bbox, float* score, float* all_probs, int* count,
                    cudaStream_t stream);
};

}  // namespace inference
}  // namespace perception
}  // namespace century
