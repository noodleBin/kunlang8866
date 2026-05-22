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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"

#include "modules/planning/proto/planning.pb.h"
#include "modules/led_monitor/controller/led_controller.h"
#include "modules/led_monitor/proto/led_monitor_config.pb.h"

namespace century {
namespace led_monitor {

using century::cyber::Component;

class LedMonitorComponent final : public Component<planning::ADCTrajectory> {
 public:
  LedMonitorComponent();
  ~LedMonitorComponent();;
  bool Init() override;
  bool Proc(const std::shared_ptr<planning::ADCTrajectory>& msg) override;

 private: 
  // Time interval, in milliseconds, between last receive planning display type.
  static constexpr double kRcvPlanningDisPlayTypeIntervalMs = 30 * 1000;
  void OnTimer();

 private:
  LedMonitorConfig led_monitor_config_;
  std::unique_ptr<LedController> led_controller_ = nullptr;
  std::mutex mutex_;
  std::unique_ptr<cyber::Timer> timer_;
};

CYBER_REGISTER_COMPONENT(LedMonitorComponent)

}  // namespace led_monitor
}  // namespace century
