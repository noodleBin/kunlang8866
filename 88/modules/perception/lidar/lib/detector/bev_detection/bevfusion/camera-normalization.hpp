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

#ifndef __CAMERA_NORMALIZATION_HPP__
#define __CAMERA_NORMALIZATION_HPP__

#include <initializer_list>
#include <memory>

// #include "common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"

namespace bevfusion {
namespace camera {

enum class NormType : int { Nothing = 0, MeanStd = 1, AlphaBeta = 2 };
enum class ChannelType : int { Nothing = 0, Invert = 1 };
enum class Interpolation : int { Nearest = 0, Bilinear = 1 };

struct NormMethod {
  float mean[3];
  float std[3];
  float alpha, beta;
  NormType type = NormType::Nothing;
  ChannelType channel_type = ChannelType::Nothing;

  // out = (x * alpha - mean) / std + beta
  static NormMethod mean_std(const float mean[3], const float std[3], float alpha = 1 / 255.0f, float beta = 0.0f,
                             ChannelType channel_type = ChannelType::Nothing);

  // out = x * alpha + beta
  static NormMethod alpha_beta(float alpha, float beta = 0, ChannelType channel_type = ChannelType::Nothing);

  // None
  static NormMethod None();
};

struct NormalizationParameter {
  int image_width;
  int image_height;
  int num_camera;
  int output_width;
  int output_height;
  float resize_lim;
  Interpolation interpolation;
  NormMethod method;
};

class Normalization {
 public:
  virtual ~Normalization() = default;
  // Here you should provide num_camera 3-channel images with the same dimensions as the width and
  // height provided at create time. The images pointer should be the host address. The pipeline is
  // as follows:
  // Step1: load pixel for interpolation.
  // Step2: execute ChannelType transformation, RGB->BGR etc.
  // Step3: execute Normalize transformation, MeanStd or AlphaBeta.
  // Step4: store as a planar memory.
  virtual nvtype::half* forward(const unsigned char** images, void* stream) = 0;
};

std::shared_ptr<Normalization> create_normalization(const NormalizationParameter& param);

};  // namespace camera
};  // namespace bevfusion

#endif  // __CAMERA_NORMALIZATION_HPP__