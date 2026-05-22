/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/guardian/proto/guardian_conf.pb.h"
#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace century::guardian
 * @brief century::guardian
 */
namespace century {
namespace guardian {

class GuardianComponent : public century::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  century::guardian::GuardianConf guardian_conf_;
  century::canbus::Chassis chassis_;
  century::monitor::SystemStatus system_status_;
  century::control::ControlCommand control_cmd_;
  century::guardian::GuardianCommand guardian_cmd_;

  double last_status_received_s_{};

  std::shared_ptr<century::cyber::Reader<century::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<century::cyber::Reader<century::control::ControlCommand>>
      control_cmd_reader_;
  std::shared_ptr<century::cyber::Reader<century::monitor::SystemStatus>>
      system_status_reader_;
  std::shared_ptr<century::cyber::Writer<century::guardian::GuardianCommand>>
      guardian_writer_;

  std::mutex mutex_;
};

CYBER_REGISTER_COMPONENT(GuardianComponent)

}  // namespace guardian
}  // namespace century
