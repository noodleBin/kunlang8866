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

/**
 * @file
 */

#pragma once

#include <memory>

#include "modules/drivers/proto/smartereye.pb.h"
#include "modules/third_party_perception/third_party_perception_base.h"

/**
 * @namespace century::third_party_perception
 * @brief century::third_party_perception
 */
namespace century {
namespace third_party_perception {

class ThirdPartyPerceptionSmartereye : public ThirdPartyPerception {
 public:
  explicit ThirdPartyPerceptionSmartereye(century::cyber::Node* const node);
  ThirdPartyPerceptionSmartereye() = default;
  ~ThirdPartyPerceptionSmartereye() = default;
  // Upon receiving smartereye data
  void OnSmartereye(const century::drivers::SmartereyeObstacles& message);
  void OnSmartereyeLanemark(const century::drivers::SmartereyeLanemark&);
  bool Process(
      century::perception::PerceptionObstacles* const response) override;

 private:
  century::perception::PerceptionObstacles eye_obstacles_;
  century::drivers::SmartereyeLanemark smartereye_lanemark_;
  std::shared_ptr<century::cyber::Reader<century::drivers::SmartereyeObstacles>>
      smartereye_obstacles_reader_ = nullptr;
  std::shared_ptr<century::cyber::Reader<century::drivers::SmartereyeLanemark>>
      smartereye_lanemark_reader_ = nullptr;
};

}  // namespace third_party_perception
}  // namespace century
