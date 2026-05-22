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

#include "lidar-scn.hpp"

#include <fstream>
#include <iostream>

#include "cyber/common/log.h"
#include "modules/perception/lidar/lib/detector/dnn_common/spconv/include/spconv/engine.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/spconv/parser/onnx-parser.hpp"

namespace bevfusion {
namespace lidar {

class SCNImplement : public SCN {
 public:
  bool init(const SCNParameter& param) {
    this->param_ = param;
    voxelization_ = create_voxelization(param_.voxelization);
    if (voxelization_ == nullptr) return false;

    native_scn_ = spconv::load_engine_from_onnx(
        param_.model, static_cast<spconv::Precision>(param_.precision));
    return native_scn_ != nullptr;
  }

  virtual const nvtype::half* forward(const nvtype::half* points,
                                      unsigned int num_points,
                                      void* stream) override {
    voxelization_->forward(points, num_points, stream, param_.order);

    native_scn_->input(0)->features().reference(
        (void*)voxelization_->features(),
        std::vector<int64_t>{voxelization_->num_voxels(),
                             voxelization_->voxel_dim()},
        spconv::DataType::Float16);
    native_scn_->input(0)->indices().reference(
        (void*)voxelization_->indices(),
        std::vector<int64_t>{voxelization_->num_voxels(),
                             voxelization_->indices_dim()},
        spconv::DataType::Int32);
    native_scn_->input(0)->set_grid_size(voxelization_->grid_size());
    native_scn_->forward(stream);
    return native_scn_->output(0)->features().ptr<nvtype::half>();
  }

  virtual std::vector<int64_t> shape() override {
    return native_scn_->output(0)->features().shape;
  }

 private:
  SCNParameter param_;
  std::shared_ptr<Voxelization> voxelization_;
  std::shared_ptr<spconv::Engine> native_scn_;
};

std::shared_ptr<SCN> create_scn(const SCNParameter& param) {
  std::shared_ptr<SCNImplement> instance(new SCNImplement());
  if (!instance->init(param)) {
    instance.reset();
  }
  return instance;
}

};  // namespace lidar
};  // namespace bevfusion