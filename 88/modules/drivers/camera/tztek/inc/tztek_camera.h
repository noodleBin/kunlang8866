/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <vector>

#include "camera_sys.h"
#include "hb_vin_interface.h"
#include "mgr_camera.h"
#include "vin_log.h"

#include "modules/drivers/proto/sensor_image.pb.h"

#include "cyber/cyber.h"

namespace century {
namespace drivers {
namespace camera {
using century::cyber::Reader;
using century::cyber::Writer;
using century::drivers::Image;
using century::drivers::camera::config::Config;
class TztekCamera {
 public:
  TztekCamera();
  ~TztekCamera();
  bool Init(std::shared_ptr<Config> camera_config);

 private:
  century::cyber::proto::QosProfile CreateQosProfile();
  std::shared_ptr<Writer<Image>> CreateImageWriter(const std::string channel);
  std::shared_ptr<Writer<CompressedImage>> CreateCompressedImageWriter(
      const std::string channel);
  int ResetPower(const std::vector<std::string>& power_value_paths);

  std::unique_ptr<century::cyber::Node> node_;
  std::vector<std::unique_ptr<CameraManager>> cameras_;
  bool output_raw_ = false;
};
}  // namespace camera
}  // namespace drivers
}  // namespace century
