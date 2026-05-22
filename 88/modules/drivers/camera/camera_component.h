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

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/camera/tztek/inc/tztek_camera.h"

namespace century {
namespace drivers {
namespace camera {

using century::drivers::camera::config::Config;

class CameraComponent : public century::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  std::unique_ptr<TztekCamera> tztek_camera_device_;
  std::shared_ptr<Config> camera_config_;
};

CYBER_REGISTER_COMPONENT(CameraComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace century
