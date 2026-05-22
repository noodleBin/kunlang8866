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

#include "modules/led_monitor/led_monitor_component.h"

namespace century {
namespace led_monitor {

LedMonitorComponent::LedMonitorComponent() {}

LedMonitorComponent::~LedMonitorComponent() {
  AINFO << "LedMonitorComponent::~LedMonitorComponent()";  
  if (timer_) {
    timer_->Stop();
  }
 
  if (led_controller_) {
    led_controller_->Stop();
  }
}

bool LedMonitorComponent::Init() {
  ACHECK(cyber::ComponentBase::GetProtoConfig(&led_monitor_config_))
      << "Unable to load led monitor conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "config file: " << cyber::ComponentBase::ConfigFilePath()
      << " is loaded.";

  AINFO << "Led monitor config: " << led_monitor_config_.DebugString();
  
  if (!led_monitor_config_.has_is_front()) {
    AERROR << "Led monitor config error: is_front not set.";
    return false;
  }
  
  if (!led_monitor_config_.has_ip()) {
    AERROR << "Led monitor config error: ip not set.";
    return false;
  }

  if (!led_monitor_config_.has_post_process()) {
    AERROR << "Led monitor config error: post_process not set.";
    return false;
  }

  // todo: check if topic_config is set
  // if (!led_monitor_config_.has_topic_config()) {
  //   AERROR << "Led monitor config error: topic_config not set.";
  //   return false;
  // }

  led_controller_ = std::make_unique<LedController>(led_monitor_config_.ip());
  
  if (!led_controller_->Init(led_monitor_config_)) {
    AERROR << "Failed to initialize LedController.";
    return false;
  }

  timer_.reset(new cyber::Timer(
    kRcvPlanningDisPlayTypeIntervalMs, [this]() { this->OnTimer(); }, false));
  timer_->Start();

  led_controller_->PlayProgram(0);

  return true;
}

void LedMonitorComponent::OnTimer() {
  led_controller_->PlayProgram(0);
}

bool LedMonitorComponent::Proc(const std::shared_ptr<planning::ADCTrajectory>& msg) {
  if (msg->has_display_type()) {
    timer_->Stop();
    timer_->Start();
    AINFO << "Received display type: " << msg->display_type();
    led_controller_->Process(msg);
  }
  
  return true;
}
}  // namespace led_monitor
}  // namespace century
