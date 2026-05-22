/******************************************************************************
 * Copyright 2025 The century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/common/timer_util.h"
#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace common {

using century::cyber::Time;

void Timer::Start() { start_time_ = Time::Now(); }

int64_t Timer::End(const std::string& msg) {
  end_time_ = Time::Now();
  int64_t elapsed_time = (end_time_ - start_time_).ToNanosecond() / 1e6;
  AINFO << "TIMER " << msg << " elapsed_time: " << elapsed_time << " ms";

  // start new timer.
  start_time_ = end_time_;
  return elapsed_time;
}
}  // namespace common
}  // namespace perception
}  // namespace century