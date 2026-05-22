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

#include <numeric>

#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-vtransform.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/launch.cuh"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensorrt.hpp"

// #include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"


namespace bevfusion {
namespace camera {

class VTransformImplement : public VTransform {
 public:
  const char* BindingInput  = "feat_in";
  const char* BindingOutput = "feat_out";

  virtual ~VTransformImplement() {
    if (output_feature_) checkRuntime(cudaFree(output_feature_));
  }

  bool init(const std::string& model) {
    engine_ = TensorRT::load(model);
    if (engine_ == nullptr) return false;

    output_dims_ = engine_->static_dims(BindingOutput);
    int32_t volumn = std::accumulate(output_dims_.begin(), output_dims_.end(), 1, std::multiplies<int32_t>());
    checkRuntime(cudaMalloc(&output_feature_, volumn * sizeof(nvtype::half)));
    return true;
  }

  virtual void print() override { engine_->print("Camerea VTransform"); }

  virtual nvtype::half* forward(const nvtype::half* camera_bev, void* stream = nullptr) override {
    engine_->forward({
      {BindingInput, camera_bev},
      {BindingOutput, output_feature_}
    }, static_cast<cudaStream_t>(stream));
    return output_feature_;
  }

  virtual std::vector<int> feat_shape() override { return output_dims_; }

 private:
  std::shared_ptr<TensorRT::Engine> engine_;
  nvtype::half* output_feature_ = nullptr;
  std::vector<int> output_dims_;
};

std::shared_ptr<VTransform> create_vtransform(const std::string& model) {
  std::shared_ptr<VTransformImplement> instance(new VTransformImplement());
  if (!instance->init(model)) {
    instance.reset();
  }
  return instance;
}

};  // namespace camera
};  // namespace bevfusion