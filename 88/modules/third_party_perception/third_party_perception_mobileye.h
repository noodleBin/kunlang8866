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

#include "modules/third_party_perception/third_party_perception_base.h"

#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"

/**
 * @namespace century::third_party_perception
 * @brief century::third_party_perception
 */
namespace century {
namespace third_party_perception {

class ThirdPartyPerceptionMobileye : public ThirdPartyPerception {
 public:
  explicit ThirdPartyPerceptionMobileye(century::cyber::Node* const node);
  ThirdPartyPerceptionMobileye() = default;
  ~ThirdPartyPerceptionMobileye() = default;
  // Upon receiving mobileye data
  void OnMobileye(const century::drivers::Mobileye& message);
  // Upon receiving conti radar data
  void OnContiRadar(const century::drivers::ContiRadar& message);
  // Upon receiving esr radar data
  void OnDelphiESR(const century::drivers::DelphiESR& message);

  bool Process(
      century::perception::PerceptionObstacles* const response) override;

 private:
  std::shared_ptr<century::cyber::Reader<century::drivers::Mobileye>>
      mobileye_reader_ = nullptr;
  std::shared_ptr<century::cyber::Reader<century::drivers::DelphiESR>>
      delphi_esr_reader_ = nullptr;
  std::shared_ptr<century::cyber::Reader<century::drivers::ContiRadar>>
      conti_radar_reader_ = nullptr;
  century::perception::PerceptionObstacles radar_obstacles_;
  century::perception::PerceptionObstacles eye_obstacles_;
};

}  // namespace third_party_perception
}  // namespace century
