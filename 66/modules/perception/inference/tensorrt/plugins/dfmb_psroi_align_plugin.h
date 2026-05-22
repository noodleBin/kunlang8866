#pragma once

#include "cyber/common/log.h"
#include "modules/perception/inference/tensorrt/rt_common.h"

namespace century {
namespace perception {
namespace inference {

class DFMBPSROIAlignPlugin : public nvinfer1::IPluginV2 {
 public:
  DFMBPSROIAlignPlugin(
      const DFMBPSROIAlignParameter &dfmb_psroi_align_parameter,
      nvinfer1::Dims *in_dims, int nbInputs) {
    heat_map_a_ = dfmb_psroi_align_parameter.heat_map_a();
    output_channel_ = dfmb_psroi_align_parameter.output_dim();
    group_height_ = dfmb_psroi_align_parameter.group_height();
    group_width_ = dfmb_psroi_align_parameter.group_width();
    pooled_height_ = dfmb_psroi_align_parameter.pooled_height();
    pooled_width_ = dfmb_psroi_align_parameter.pooled_width();
    pad_ratio_ = dfmb_psroi_align_parameter.pad_ratio();
    sample_per_part_ = dfmb_psroi_align_parameter.sample_per_part();

    trans_std_ = dfmb_psroi_align_parameter.trans_std();
    part_height_ = dfmb_psroi_align_parameter.part_height();
    part_width_ = dfmb_psroi_align_parameter.part_width();
    heat_map_b_ = dfmb_psroi_align_parameter.heat_map_b();
    no_trans_ = (nbInputs < 3);
    num_classes_ = no_trans_ ? 1 : in_dims[2].d[1];

    CHECK_GT(heat_map_a_, 0);
    CHECK_GE(heat_map_b_, 0);
    CHECK_GE(pad_ratio_, 0);
    CHECK_GT(output_channel_, 0);
    CHECK_GT(sample_per_part_, 0);
    CHECK_GT(group_height_, 0);
    CHECK_GT(group_width_, 0);
    CHECK_GT(pooled_height_, 0);
    CHECK_GT(pooled_width_, 0);
    CHECK_GE(part_height_, 0);
    CHECK_GE(part_width_, 0);

    channels_ = in_dims[0].d[0];
    height_ = in_dims[0].d[1];
    width_ = in_dims[0].d[2];
    output_dims_ = nvinfer1::Dims4(in_dims[1].d[0], output_channel_,
                                   pooled_height_, pooled_width_);
    output_size_ =
        in_dims[1].d[0] * output_channel_ * pooled_height_ * pooled_width_;

    CHECK_EQ(channels_, output_channel_ * group_height_ * group_width_);
    CHECK_EQ(in_dims[1].d[1], 5);
    if (!no_trans_) {
      CHECK_EQ(in_dims[2].d[1] % 2, 0);
      int num_classes = in_dims[2].d[1] / 2;
      CHECK_EQ(output_channel_ % num_classes, 0);
      CHECK_EQ(part_height_, in_dims[2].d[2]);
      CHECK_EQ(part_width_, in_dims[2].d[3]);
    }
  }

  virtual ~DFMBPSROIAlignPlugin() override {}

  int initialize() noexcept override { return 0; }
  void terminate() noexcept override {}
  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    // TODO: complete input dims assertion
    return output_dims_;
  }

  void configurePlugin(const nvinfer1::Dims *inputDims, int nbInputs,
                       const nvinfer1::Dims *outputDims, int nbOutputs,
                       int maxBatchSize) noexcept {}

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override { return 0; }

  int enqueue(int batchSize, const void *const *inputs, void *const *outputs,
              void *workspace, cudaStream_t stream) noexcept override;
  int enqueue(int batchSize, const void *const *inputs,
                                    void **outputs, void *workspace,
                                    cudaStream_t stream);
  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    // TODO: 实现序列化逻辑
  }

 private:
  const int thread_size_ = 512;
  float heat_map_a_;
  float heat_map_b_;
  float pad_ratio_;

  int output_channel_;
  bool no_trans_;
  float trans_std_;
  int sample_per_part_;
  int group_height_;
  int group_width_;
  int pooled_height_;
  int pooled_width_;
  int part_height_;
  int part_width_;
  int num_classes_;

  int channels_;
  int height_;
  int width_;
  int output_size_;

  nvinfer1::Dims output_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace century
