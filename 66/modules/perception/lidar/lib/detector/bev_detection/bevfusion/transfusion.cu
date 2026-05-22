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

#include <algorithm>
#include <numeric>
#include <cyber/common/log.h> 

#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"


#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/launch.cuh"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensorrt.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/transfusion.hpp"
// #include "transfusion.hpp"

namespace bevfusion {
namespace fuser {

class TransfusionImplement : public Transfusion {
 public:
  // const char* BindingCamera = "camera";
  const char* BindingLidar  = "lidar";
  const char* BindingOutput = "middle";

  virtual ~TransfusionImplement() {
    if (output_) checkRuntime(cudaFree(output_));
  }

  virtual bool init(const std::string& model) {
    AERROR << "1 ************model: " << model;
    engine_ = TensorRT::load(model);
    AERROR << "2 ************model: engine_:  " << engine_;

    if (engine_ == nullptr) return false;

    AERROR << "3 ************model: engine_:  " << engine_;
    if (engine_->has_dynamic_dim()) {
      printf("Dynamic shapes are not supported.\n");
      AERROR << "3.1 ************engine_->dtype(BindingOutput):  " << static_cast<int>(engine_->dtype(BindingOutput));
      return false;
    }

    AERROR << "4 ************engine_->dtype(BindingOutput):  ";
    AERROR << "4.1 ************engine_->dtype(BindingOutput):  ";
    AERROR << "*****************ibinding: " << BindingOutput;


    AERROR << "4 ************engine_->dtype(BindingOutput):  " << static_cast<int>(engine_->dtype(BindingOutput));


    auto shape = engine_->static_dims(BindingOutput);
    Asserts(engine_->dtype(BindingOutput) == TensorRT::DType::HALF, "Invalid binding data type.");

    AERROR << "5 ************model: engine_:  " << engine_;


    size_t volumn = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int>());
    checkRuntime(cudaMalloc(&output_, volumn * sizeof(half)));

    AERROR << "************TransfusionImplement Create success: " << model;
    return true;
  }

  virtual void print() override { engine_->print("Transfusion"); }

  virtual nvtype::half* forward(const nvtype::half* camera_bev, const nvtype::half* lidar_bev, void* stream) override {
    cudaStream_t _stream = static_cast<cudaStream_t>(stream);
    engine_->forward({
      // {BindingCamera, camera_bev},
      {BindingLidar, lidar_bev},
      {BindingOutput, output_}}, _stream);
    return output_;
  }

 private:
  std::shared_ptr<TensorRT::Engine> engine_;
  nvtype::half* output_ = nullptr;
  std::vector<std::vector<int>> bindshape_;
};

std::shared_ptr<Transfusion> create_transfusion(const std::string& param) {
  std::shared_ptr<TransfusionImplement> instance(new TransfusionImplement());
  if (!instance->init(param)) {
    instance.reset();
  }
  return instance;
}

};  // namespace fuser
};  // namespace bevfusion