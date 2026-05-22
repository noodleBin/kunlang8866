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

#ifndef CYBER_COMMON_CALLBACK_TRACE_H_
#define CYBER_COMMON_CALLBACK_TRACE_H_

#include <cstdint>
#include <string>

#include "cyber/common/trace/trace.h"

namespace century {
namespace cyber {
namespace common {

class CallbackTrace : public Trace {
 public:
  CallbackTrace(const std::string& type, const std::string& name,
                int64_t threshold_ms = -1);
  CallbackTrace(const std::string& type, const std::string& name,
                const std::string& detail, int64_t threshold_ms = -1);
  ~CallbackTrace() override;

  CallbackTrace(const CallbackTrace&) = delete;
  CallbackTrace& operator=(const CallbackTrace&) = delete;

 protected:
  bool Enabled() const override;
  void OnTraceEnd(uint64_t elapsed_us, uint64_t thread_cpu_us) override;

 private:
  std::string type_;
  std::string name_;
  std::string detail_;
  int64_t threshold_ms_ = -1;
};

std::string DemangleTypeName(const char* type_name);
bool CallbackTraceEnabled();

}  // namespace common
}  // namespace cyber
}  // namespace century

#endif  // CYBER_COMMON_CALLBACK_TRACE_H_
