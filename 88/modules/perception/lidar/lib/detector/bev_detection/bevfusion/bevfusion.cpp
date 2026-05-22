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

#include "bevfusion.hpp"

#include <iomanip>
#include <numeric>

#include "cyber/common/log.h"
#include "modules/perception/lidar/lib/detector/bev_detection/bev_debug_utils.h"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/launch.cuh"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensor.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/timer.hpp"

namespace bevfusion {

class CoreImplement : public Core {
 public:
  virtual ~CoreImplement() = default;

  bool init(const CoreParameter& param) {
    if (param.use_camera) {
      camera_backbone_ = camera::create_backbone(param.camera_model);
      if (nullptr == camera_backbone_) {
        AERROR << "Failed to create camera backbone.";
        return false;
      }

      camera_bevpool_ = camera::create_bevpool(camera_backbone_->camera_shape(),
                                               param.geometry.geometry_dim.x,
                                               param.geometry.geometry_dim.y);
      if (nullptr == camera_bevpool_) {
        AERROR << "Failed to create camera bevpool.";
        return false;
      }

      camera_vtransform_ = camera::create_vtransform(param.camera_vtransform);
      if (nullptr == camera_vtransform_) {
        AERROR << "Failed to create camera vtransform.";
        return false;
      }

      normalizer_ = camera::create_normalization(param.normalize);
      if (nullptr == normalizer_) {
        AERROR << "Failed to create normalizer.";
        return false;
      }

      camera_depth_ = camera::create_depth(param.normalize.output_width,
                                           param.normalize.output_height,
                                           param.normalize.num_camera);
      if (nullptr == camera_depth_) {
        AERROR << "Failed to create depth.";
        return false;
      }

      camera_geometry_ = camera::create_geometry(param.geometry);
      if (nullptr == camera_geometry_) {
        AERROR << "Failed to create geometry.";
        return false;
      }
    }

    transfusion_ =
        fuser::create_transfusion(param.transfusion, param.use_camera);
    if (nullptr == transfusion_) {
      AERROR << "Failed to create transfusion.";
      return false;
    }

    transbbox_ = head::transbbox::create_transbbox(param.transbbox);
    if (nullptr == transbbox_) {
      AERROR << "Failed to create head transbbox.";
      return false;
    }

    lidar_scn_ = lidar::create_scn(param.lidar_scn);
    if (nullptr == lidar_scn_) {
      AERROR << "Failed to create lidar scn.";
      return false;
    }

    capacity_points_ = 300000;
    param_ = param;

    return true;
  }

  std::vector<head::transbbox::BoundingBox> forward_only(
      const void* camera_images, const nvtype::half* lidar_points_device,
      int num_points, void* stream, bool do_normalization) {
    int cappoints = static_cast<int>(capacity_points_);
    if (num_points > cappoints) {
      AWARN << "Points exceed capacity " << cappoints << ", cropping.";
    }

    num_points = std::min(cappoints, num_points);

    const nvtype::half* lidar_feature =
        this->lidar_scn_->forward(lidar_points_device, num_points, stream);

    if (param_.use_camera) {
      nvtype::half* normed_images = (nvtype::half*)camera_images;
      if (do_normalization) {
        normed_images = (nvtype::half*)this->normalizer_->forward(
            (const unsigned char**)(camera_images), stream);
      }
      const nvtype::half* depth = this->camera_depth_->forward(
          lidar_points_device, num_points,
          param_.lidar_scn.voxelization.num_feature, stream);

      this->camera_backbone_->forward(normed_images, depth, stream);
      const nvtype::half* camera_bev = this->camera_bevpool_->forward(
          this->camera_backbone_->feature(), this->camera_backbone_->depth(),
          this->camera_geometry_->indices(),
          this->camera_geometry_->intervals(),
          this->camera_geometry_->num_intervals(), stream);

      const nvtype::half* camera_bevfeat =
          camera_vtransform_->forward(camera_bev, stream);
      const nvtype::half* fusion_feature =
          this->transfusion_->forward(camera_bevfeat, lidar_feature, stream);
      return this->transbbox_->forward(fusion_feature,
                                       param_.transbbox.confidence_threshold,
                                       stream, param_.transbbox.sorted_bboxes);
    }

    const nvtype::half* fusion_feature =
        this->transfusion_->forward(nullptr, lidar_feature, stream);
    return this->transbbox_->forward(fusion_feature,
                                     param_.transbbox.confidence_threshold,
                                     stream, param_.transbbox.sorted_bboxes);
  }

  std::vector<head::transbbox::BoundingBox> forward_timer(
      const void* camera_images, const nvtype::half* lidar_points_device,
      int num_points, void* stream, bool do_normalization,
      float preprocess_time_ms) {
    int cappoints = static_cast<int>(capacity_points_);
    if (num_points > cappoints) {
      AWARN << "Points exceed capacity " << cappoints << ", cropping.";
    }

    num_points = std::min(cappoints, num_points);

    std::vector<float> times;
    cudaStream_t _stream = static_cast<cudaStream_t>(stream);
    AINFO << "==================BEVFusion===================";
    AINFO << "Preprocess: " << std::fixed << std::setprecision(3)
          << preprocess_time_ms << " ms";

    if (param_.use_camera) {
      nvtype::half* normed_images = (nvtype::half*)camera_images;
      if (do_normalization) {
        timer_.start(_stream);
        normed_images = (nvtype::half*)this->normalizer_->forward(
            (const unsigned char**)(camera_images), stream);
        timer_.stop("[NoSt] ImageNrom");
      }

      timer_.start(_stream);
      const nvtype::half* lidar_feature =
          this->lidar_scn_->forward(lidar_points_device, num_points, stream);
      times.emplace_back(timer_.stop("Lidar Backbone"));

      timer_.start(_stream);
      const nvtype::half* depth = this->camera_depth_->forward(
          lidar_points_device, num_points,
          param_.lidar_scn.voxelization.num_feature, stream);
      times.emplace_back(timer_.stop("Camera Depth"));

      timer_.start(_stream);
      this->camera_backbone_->forward(normed_images, depth, stream);
      times.emplace_back(timer_.stop("Camera Backbone"));

      timer_.start(_stream);
      const nvtype::half* camera_bev = this->camera_bevpool_->forward(
          this->camera_backbone_->feature(), this->camera_backbone_->depth(),
          this->camera_geometry_->indices(),
          this->camera_geometry_->intervals(),
          this->camera_geometry_->num_intervals(), stream);
      times.emplace_back(timer_.stop("Camera Bevpool"));

      timer_.start(_stream);
      const nvtype::half* camera_bevfeat =
          camera_vtransform_->forward(camera_bev, stream);
      times.emplace_back(timer_.stop("VTransform"));

      timer_.start(_stream);
      const nvtype::half* fusion_feature =
          this->transfusion_->forward(camera_bevfeat, lidar_feature, stream);
      times.emplace_back(timer_.stop("Transfusion"));

      timer_.start(_stream);
      auto output = this->transbbox_->forward(
          fusion_feature, param_.transbbox.confidence_threshold, stream,
          param_.transbbox.sorted_bboxes);
      times.emplace_back(timer_.stop("Head BoundingBox"));

      float trt_time =
          std::accumulate(times.begin(), times.end(), 0.0f, std::plus<float>{});
      AINFO << "TRT Total: " << std::fixed << std::setprecision(3) << trt_time
            << " ms";
      AINFO << "Total: " << std::fixed << std::setprecision(3)
            << preprocess_time_ms + trt_time << " ms";
      AINFO << "=============================================";

      return output;
    } else {
      timer_.start(_stream);
      const nvtype::half* lidar_feature =
          this->lidar_scn_->forward(lidar_points_device, num_points, stream);
      times.emplace_back(timer_.stop("Lidar Backbone"));

      timer_.start(_stream);
      const nvtype::half* fusion_feature =
          this->transfusion_->forward(nullptr, lidar_feature, stream);
      times.emplace_back(timer_.stop("Transfusion"));

      timer_.start(_stream);
      auto output = this->transbbox_->forward(
          fusion_feature, param_.transbbox.confidence_threshold, stream,
          param_.transbbox.sorted_bboxes);
      times.emplace_back(timer_.stop("Head BoundingBox"));

      float trt_time =
          std::accumulate(times.begin(), times.end(), 0.0f, std::plus<float>{});
      AINFO << "TRT Total: " << std::fixed << std::setprecision(3) << trt_time
            << " ms";
      AINFO << "Total: " << std::fixed << std::setprecision(3)
            << preprocess_time_ms + trt_time << " ms";

      return output;
    }
  }

  virtual std::vector<head::transbbox::BoundingBox> forward(
      const unsigned char** camera_images,
      const nvtype::half* lidar_points_device, int num_points, void* stream,
      float preprocess_time_ms) override {
    if (enable_debug_) {
      return this->forward_debug(camera_images, lidar_points_device, num_points,
                                 stream, true);
    } else if (enable_timer_) {
      return this->forward_timer(camera_images, lidar_points_device, num_points,
                                 stream, true, preprocess_time_ms);
    } else {
      return this->forward_only(camera_images, lidar_points_device, num_points,
                                stream, true);
    }
  }

  virtual std::vector<head::transbbox::BoundingBox> forward_no_normalize(
      const nvtype::half* camera_normed_images_device,
      const nvtype::half* lidar_points_device, int num_points, void* stream,
      float preprocess_time_ms) override {
    if (enable_debug_) {
      return this->forward_debug(camera_normed_images_device,
                                 lidar_points_device, num_points, stream,
                                 false);
    } else if (enable_timer_) {
      return this->forward_timer(camera_normed_images_device,
                                 lidar_points_device, num_points, stream, false,
                                 preprocess_time_ms);
    } else {
      return this->forward_only(camera_normed_images_device,
                                lidar_points_device, num_points, stream, false);
    }
  }

  virtual void set_timer(bool enable) override { enable_timer_ = enable; }
  virtual void set_debug(bool enable) override {
    enable_debug_ = enable;
    transbbox_->set_debug(enable);
  }

  std::vector<head::transbbox::BoundingBox> forward_debug(
      const void* camera_images, const nvtype::half* lidar_points_device,
      int num_points, void* stream, bool do_normalization) {
    using century::perception::lidar::BevDebugUtils;
    BevDebugUtils dbg;
    dbg.SetOutputDir("/century/data/bevfusion_debug/");
    dbg.SetEnabled(true);

    int cappoints = static_cast<int>(capacity_points_);
    num_points = std::min(cappoints, num_points);
    cudaStream_t _stream = static_cast<cudaStream_t>(stream);

    dbg.CompareWithPython(lidar_points_device, "00_input_points.bin",
                          "Input Points", sizeof(nvtype::half), _stream);

    const nvtype::half* lidar_feature =
        this->lidar_scn_->forward(lidar_points_device, num_points, stream);
    dbg.CompareWithPython(lidar_feature, "01_lidar_feature.bin",
                          "Lidar Feature [1,256,120,160]", sizeof(nvtype::half),
                          _stream);

    if (!param_.use_camera) {
      const nvtype::half* fusion_feature =
          this->transfusion_->forward(nullptr, lidar_feature, stream);
      return this->transbbox_->forward(fusion_feature,
                                       param_.transbbox.confidence_threshold,
                                       stream, param_.transbbox.sorted_bboxes);
    }

    nvtype::half* normed_images = (nvtype::half*)camera_images;
    if (do_normalization) {
      normed_images = (nvtype::half*)this->normalizer_->forward(
          (const unsigned char**)(camera_images), stream);
    }

    const nvtype::half* depth = this->camera_depth_->forward(
        lidar_points_device, num_points,
        param_.lidar_scn.voxelization.num_feature, stream);

    this->camera_backbone_->forward(normed_images, depth, stream);

    dbg.CompareWithPython(
        this->camera_backbone_->feature(), "05_camera_feature.bin",
        "Camera Feature [6,48,80,80]", sizeof(nvtype::half), _stream);
    dbg.CompareWithPython(this->camera_backbone_->depth(),
                          "05_depth_weights.bin", "Depth Weights [6,118,48,80]",
                          sizeof(nvtype::half), _stream);

    const nvtype::half* camera_bev = this->camera_bevpool_->forward(
        this->camera_backbone_->feature(), this->camera_backbone_->depth(),
        this->camera_geometry_->indices(), this->camera_geometry_->intervals(),
        this->camera_geometry_->num_intervals(), stream);
    dbg.CompareWithPython(camera_bev, "06_camera_bev.bin",
                          "Camera BEV [1,80,240,320]", sizeof(nvtype::half),
                          _stream);

    const nvtype::half* camera_bevfeat =
        camera_vtransform_->forward(camera_bev, stream);
    dbg.CompareWithPython(camera_bevfeat, "07_camera_vtransform.bin",
                          "Camera VTransform [1,80,120,160]",
                          sizeof(nvtype::half), _stream);

    const nvtype::half* fusion_feature =
        this->transfusion_->forward(camera_bevfeat, lidar_feature, stream);
    dbg.CompareWithPython(fusion_feature, "08_fused_feature.bin",
                          "Fused Feature [1,512,120,160]", sizeof(nvtype::half),
                          _stream);

    return this->transbbox_->forward(fusion_feature,
                                     param_.transbbox.confidence_threshold,
                                     stream, param_.transbbox.sorted_bboxes);
  }

  virtual void print() override {
    transfusion_->print();
    transbbox_->print();
  }

  virtual void update(const float* camera2lidar, const float* camera_intrinsics,
                      const float* lidar2image, const float* img_aug_matrix,
                      void* stream) override {
    if (!param_.use_camera) {
      return;
    }
    camera_depth_->update(img_aug_matrix, lidar2image, stream);
    camera_geometry_->update(camera2lidar, camera_intrinsics, img_aug_matrix,
                             stream);
  }

  virtual void free_excess_memory() override {
    if (!param_.use_camera) {
      return;
    }
    camera_geometry_->free_excess_memory();
  }

 private:
  CoreParameter param_;
  nv::EventTimer timer_;
  size_t capacity_points_ = 0;

  std::shared_ptr<camera::Normalization> normalizer_;
  std::shared_ptr<camera::Backbone> camera_backbone_;
  std::shared_ptr<camera::BEVPool> camera_bevpool_;
  std::shared_ptr<camera::VTransform> camera_vtransform_;
  std::shared_ptr<camera::Depth> camera_depth_;
  std::shared_ptr<camera::Geometry> camera_geometry_;
  std::shared_ptr<lidar::SCN> lidar_scn_;
  std::shared_ptr<fuser::Transfusion> transfusion_;
  std::shared_ptr<head::transbbox::TransBBox> transbbox_;
  float confidence_threshold_ = 0;
  bool enable_timer_ = false;
  bool enable_debug_ = false;
};

std::shared_ptr<Core> create_core(const CoreParameter& param) {
  std::shared_ptr<CoreImplement> instance(new CoreImplement());
  if (!instance->init(param)) {
    instance.reset();
  }
  return instance;
}

}  // namespace bevfusion
