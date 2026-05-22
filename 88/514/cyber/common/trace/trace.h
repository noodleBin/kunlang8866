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

#ifndef CYBER_COMMON_TRACE_H_
#define CYBER_COMMON_TRACE_H_

#include <time.h>

#include <chrono>
#include <cstdint>

namespace century {
namespace cyber {
namespace common {

class Trace {
 public:
  Trace();
  virtual ~Trace();

  Trace(const Trace&) = delete;
  Trace& operator=(const Trace&) = delete;

 protected:
  void Finish();

  virtual bool Enabled() const = 0;
  virtual void OnTraceEnd(uint64_t elapsed_us, uint64_t thread_cpu_us) = 0;

 private:
  std::chrono::steady_clock::time_point start_time_;
  timespec thread_cpu_start_time_ = {0, 0};
  bool has_thread_cpu_start_time_ = false;
  bool finished_ = false;
};

}  // namespace common
}  // namespace cyber
}  // namespace century

#endif  // CYBER_COMMON_TRACE_H_
