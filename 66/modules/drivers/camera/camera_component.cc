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

#include "modules/drivers/camera/camera_component.h"

namespace century {
namespace drivers {
namespace camera {

bool CameraComponent::Init() {
  camera_config_ = std::make_shared<Config>();
  if (!century::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "UsbCam config: " << camera_config_->DebugString();

  tztek_camera_device_.reset(new TztekCamera());

  return tztek_camera_device_->Init(camera_config_);
}

bool CameraComponent::Proc() {

  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace century
