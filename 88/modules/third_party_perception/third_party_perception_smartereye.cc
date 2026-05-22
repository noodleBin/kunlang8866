/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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
#include "modules/third_party_perception/third_party_perception_smartereye.h"

#include <memory>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/tools/conversion_smartereye.h"

namespace century {
namespace third_party_perception {

using century::perception::PerceptionObstacles;

ThirdPartyPerceptionSmartereye::ThirdPartyPerceptionSmartereye(
          century::cyber::Node* const node) :ThirdPartyPerception(node) {
  smartereye_obstacles_reader_ =
      node_->CreateReader<century::drivers::SmartereyeObstacles>(
      FLAGS_smartereye_obstacles_topic,
      [this](const std::shared_ptr<
      century::drivers::SmartereyeObstacles> &message) {
        OnSmartereye(*message.get());
      });
  smartereye_lanemark_reader_ =
      node_->CreateReader<century::drivers::SmartereyeLanemark>(
      FLAGS_smartereye_lanemark_topic,
      [this](const std::shared_ptr<
      century::drivers::SmartereyeLanemark> &message) {
        OnSmartereyeLanemark(*message.get());
      });
}

void ThirdPartyPerceptionSmartereye::OnSmartereye(
              const century::drivers::SmartereyeObstacles& message) {
  ADEBUG << "Received smartereye data: run smartereye callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  eye_obstacles_ = conversion_smartereye::SmartereyeToPerceptionObstacles(
      message, smartereye_lanemark_, localization_, chassis_);
}

void ThirdPartyPerceptionSmartereye::OnSmartereyeLanemark(
              const century::drivers::SmartereyeLanemark& message) {
  ADEBUG << "Received smartereye data: run smartereye callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  smartereye_lanemark_.CopyFrom(message);
}

bool ThirdPartyPerceptionSmartereye::Process(
            PerceptionObstacles* const response) {
  ADEBUG << "Timer is triggered: publish PerceptionObstacles";
  CHECK_NOTNULL(response);

  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);

  *response = eye_obstacles_;

  common::util::FillHeader(FLAGS_third_party_perception_node_name, response);

  eye_obstacles_.Clear();
  return true;
}

}  // namespace third_party_perception
}  // namespace century
