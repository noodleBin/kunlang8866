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

#include "cyber/component/timer_component.h"

#include "cyber/common/trace/callback_trace.h"
#include "cyber/timer/timer.h"

namespace century {
namespace cyber {

TimerComponent::TimerComponent() {}

TimerComponent::~TimerComponent() {}

bool TimerComponent::Process() {
  if (is_shutdown_.load()) {
    return true;
  }
  const std::string trace_name =
      node_ == nullptr ? "standalone_timer_component" : node_->Name();
  if (!common::CallbackTraceShouldTrace("timer_component", trace_name)) {
    return Proc();
  }
  const auto callback_trace_detail = std::string("module=") + trace_name;
  common::CallbackTrace trace(
      "timer_component", trace_name.c_str(), callback_trace_detail.c_str());
  return Proc();
}

bool TimerComponent::Initialize(const TimerComponentConfig& config) {
  if (!config.has_name() || !config.has_interval()) {
    AERROR << "Missing required field in config file.";
    return false;
  }
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  if (!Init()) {
    return false;
  }

  std::shared_ptr<TimerComponent> self =
      std::dynamic_pointer_cast<TimerComponent>(shared_from_this());
  std::string callback_trace_detail;
  if (common::CallbackTraceEnabled()) {
    callback_trace_detail = std::string("module=") + config.name();
  }
  auto func = [self, callback_trace_detail]() {
    const std::string trace_name =
        self->node_ == nullptr ? "standalone_timer_component"
                               : self->node_->Name();
    if (!common::CallbackTraceShouldTrace("timer_component", trace_name)) {
      self->Proc();
      return;
    }
    common::CallbackTrace trace(
        "timer_component", trace_name.c_str(), callback_trace_detail.c_str());
    self->Proc();
  };
  timer_.reset(new Timer(config.interval(), func, false));
  timer_->Start();
  return true;
}

void TimerComponent::Clear() { timer_.reset(); }

uint64_t TimerComponent::GetInterval() const { return interval_; }

}  // namespace cyber
}  // namespace century
