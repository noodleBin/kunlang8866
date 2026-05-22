/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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

#include <memory>
#include <vector>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/mcloud/cloud/cloud_mqtt.h"
#include "modules/mcloud/network/client.hpp"
#include "modules/mcloud/cloud/cloud.h"
// #include "client.hpp"
// #include "cloud.h"

/**
 * @namespace century::mcloud
 * @brief century::mcloud
 */
namespace century {
namespace mcloud {

class MCloud : public century::cyber::TimerComponent {
 public:
  ~MCloud() override;
  bool Init() override;
  bool Proc() override;

 private:
  TcpClient client_;
  Cloud cloud_;
  CloudMqtt cloud_mqtt_;
};

CYBER_REGISTER_COMPONENT(MCloud)

}  // namespace mcloud
}  // namespace century
