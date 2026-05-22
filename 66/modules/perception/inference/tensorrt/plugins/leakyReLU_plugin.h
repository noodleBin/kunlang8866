#pragma once

#include <algorithm>
#include <vector>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace century {
namespace perception {
namespace inference {

class ReLUPlugin : public nvinfer1::IPluginV2 {
 public:
  ReLUPlugin(const ReLUParameter &param, const nvinfer1::Dims &in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }
    negative_slope_ = param.negative_slope();
  }

  ReLUPlugin() {}

  ~ReLUPlugin() override {}

  int initialize() noexcept override { return 0; }

  void terminate() noexcept override {}

  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    return inputs[0];
  }

  void configurePlugin(const nvinfer1::Dims *inputDims, int nbInputs,
                       const nvinfer1::Dims *outputDims, int nbOutputs,
                       int maxBatchSize) noexcept {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override { return 0; }

  int enqueue(int batchSize, const void *const *inputs, void *const *outputs,
              void *workspace, cudaStream_t stream) noexcept override;
  int enqueue(int batchSize, const void *const *inputs,
                          void **outputs, void *workspace, cudaStream_t stream);

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    // TODO: 实现序列化逻辑
  }

 private:
  float negative_slope_;
  nvinfer1::Dims input_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace century
