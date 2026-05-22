#pragma once

#include <algorithm>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace century {
namespace perception {
namespace inference {

class RPNProposalSSDPlugin : public nvinfer1::IPluginV2 {
 public:
  RPNProposalSSDPlugin(
      const BBoxRegParameter &bbox_reg_param,
      const DetectionOutputSSDParameter &detection_output_ssd_param,
      nvinfer1::Dims *in_dims) {
    height_ = in_dims[0].d[1];
    width_ = in_dims[0].d[2];

    for (int i = 0; i < 4; ++i) {
      bbox_mean_[i] = bbox_reg_param.bbox_mean(i);
      bbox_std_[i] = bbox_reg_param.bbox_std(i);
    }

    GenAnchorParameter gen_anchor_param =
        detection_output_ssd_param.gen_anchor_param();
    num_anchor_per_point_ = std::min(gen_anchor_param.anchor_width().size(),
                                     gen_anchor_param.anchor_height().size());
    anchor_heights_ = new float[num_anchor_per_point_]();
    anchor_widths_ = new float[num_anchor_per_point_]();
    for (int i = 0; i < num_anchor_per_point_; ++i) {
      anchor_heights_[i] = gen_anchor_param.anchor_height(i);
      anchor_widths_[i] = gen_anchor_param.anchor_width(i);
    }

    heat_map_a_ = detection_output_ssd_param.heat_map_a();

    min_size_mode_ =
        static_cast<int>(detection_output_ssd_param.min_size_mode());
    min_size_h_ = detection_output_ssd_param.min_size_h();
    min_size_w_ = detection_output_ssd_param.min_size_w();

    threshold_objectness_ = detection_output_ssd_param.threshold_objectness();
    refine_out_of_map_bbox_ =
        detection_output_ssd_param.refine_out_of_map_bbox();

    NMSSSDParameter nms_param = detection_output_ssd_param.nms_param();
    max_candidate_n_ = nms_param.max_candidate_n(0);
    overlap_ratio_ = nms_param.overlap_ratio(0);
    top_n_ = nms_param.top_n(0);
  }

  virtual ~RPNProposalSSDPlugin() override {}

  int initialize() noexcept override { return 0; }
  void terminate() noexcept override {}
  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    // TODO: complete inputs dims assertion
    // TODO: batch size is hard coded to 1 here
    return nvinfer1::Dims4(top_n_ * 1, 5, 1, 1);
  }

  void configurePlugin(const nvinfer1::Dims *inputDims, int nbInputs,
                       const nvinfer1::Dims *outputDims, int nbOutputs,
                       int maxBatchSize) noexcept {}

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override {
    return 0;
  }

  int enqueue(int batchSize, const void *const *inputs, void *const *outputs,
              void *workspace, cudaStream_t stream) noexcept override;
  int enqueue(int batchSize, const void *const *inputs, void **outputs,
              void *workspace, cudaStream_t stream);
  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  void InitializeOutputBuffers(float *out_rois, int out_rois_size,
                               cudaStream_t stream);
  void ReshapeBboxPred(const float *rpn_bbox_pred, int batch_size,
                       float *temp_rpn_bbox_pred, cudaStream_t stream);
  void GenerateAnchors(float *anchors, cudaStream_t stream);
  void DecodeBboxes(const float *anchors, const float *temp_rpn_bbox_pred,
                    int batch_size, float *proposals, cudaStream_t stream);
  void ClipBboxes(float *proposals, int rpn_bbox_pred_size, float origin_height,
                  float origin_width, cudaStream_t stream);
  void ReshapeScores(const float *rpn_cls_prob_reshape, int batch_size,
                     float *temp_scores, cudaStream_t stream);
  void FilterBboxes(const float *proposals, const float *temp_scores,
                    int batch_size, float *filtered_proposals,
                    float *filtered_scores, int *filtered_count,
                    cudaStream_t stream);
  void SortAndKeepTopN(const float *filtered_proposals, float *filtered_scores,
                       int batch_size, int num_anchor,
                       const int *filtered_count, int *host_filtered_count,
                       float *pre_nms_proposals, int *sorted_indexes,
                       cudaStream_t stream);
  void ApplyNmsAndOutput(int batch_size, const int *host_filtered_count,
                         float *pre_nms_proposals, float *out_rois,
                         cudaStream_t stream);
  void AllocateAndReshapeScores(const float *rpn_cls_prob_reshape,
                                int batch_size, float **temp_scores,
                                cudaStream_t stream);
  void AllocateFilteredBuffers(int batch_size, float **filtered_proposals,
                               float **filtered_scores, int **filtered_count,
                               cudaStream_t stream);

  const int thread_size_ = 512;
  bool refine_out_of_map_bbox_ = true;
  float bbox_mean_[4];
  float bbox_std_[4];
  float heat_map_a_;
  float min_size_h_ = 0.0f;
  float min_size_w_ = 0.0f;
  float threshold_objectness_ = 0.2f;
  float overlap_ratio_ = 0.7f;
  int height_;
  int width_;
  int max_candidate_n_ = 3000;
  int min_size_mode_ = 0;
  int num_anchor_per_point_;
  int top_n_ = 300;
  int out_rois_num_;

  float *anchor_heights_;
  float *anchor_widths_;
};

}  // namespace inference
}  // namespace perception
}  // namespace century
