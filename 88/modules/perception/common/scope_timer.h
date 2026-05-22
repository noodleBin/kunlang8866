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

#include <sys/time.h>

#include <string>

namespace century {
namespace perception {
namespace common {

class SimpleTimer {
 public:
  SimpleTimer();

  void Start();
  double End(const std::string& msg);

 private:
  struct timeval start_time_;
};

class FunctionScopeTimer {
 public:
  explicit FunctionScopeTimer(std::string name);
  ~FunctionScopeTimer();

 private:
  std::string name_;
  double start_unix_;
};

double GetCurrentUnixTimestamp();

}  // namespace common
}  // namespace perception
}  // namespace century

#define SIMPLE_PERF_BLOCK_START_ID(timer_id)                         \
  century::perception::common::SimpleTimer _simple_timer_##timer_id; \
  _simple_timer_##timer_id.Start()

#define SIMPLE_PERF_BLOCK_END_ID(timer_id, msg) \
  _simple_timer_##timer_id.End(msg)

#define FUNCTION_SCOPE_TIMER(name) \
  century::perception::common::FunctionScopeTimer _function_scope_timer(name)
