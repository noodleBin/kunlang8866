#pragma once

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace century {
namespace perception {
namespace inference {

class SoftmaxPlugin : public nvinfer1::IPluginV2 {
 public:
  SoftmaxPlugin(const SoftmaxParameter &param, nvinfer1::Dims in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }
    axis_ = param.axis() - 1;
    CHECK_GE(axis_, 0);
    CHECK_LE(axis_ + 1, input_dims_.nbDims);

    inner_num_ = 1;
    for (int i = axis_ + 1; i < input_dims_.nbDims; i++) {
      inner_num_ *= input_dims_.d[i];
    }
    outer_num_ = 1;
    for (int i = 0; i < axis_; i++) {
      outer_num_ *= input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }

  SoftmaxPlugin() {}

  ~SoftmaxPlugin() override {
    cudnnDestroyTensorDescriptor(input_desc_);
    cudnnDestroyTensorDescriptor(output_desc_);
  }

  int initialize() noexcept override {
    cudnnCreate(&cudnn_);
    cublasCreate(&cublas_);
    return 0;
  }

  void terminate() noexcept override {
    cublasDestroy(cublas_);
    cudnnDestroy(cudnn_);
  }

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


  int enqueue(int batch_size, const void *const *inputs,
                            void **outputs, void *workspace,
                            cudaStream_t stream);
  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {}

 private:
  cudnnHandle_t cudnn_;
  cublasHandle_t cublas_;
  nvinfer1::Dims input_dims_;
  int axis_;
  int inner_num_;
  int outer_num_;
  cudnnTensorDescriptor_t input_desc_;
  cudnnTensorDescriptor_t output_desc_;
};

}  // namespace inference
}  // namespace perception
}  // namespace century
