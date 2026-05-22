/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
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

#include <sys/syscall.h>
#include <libgen.h>
#include <string>
#include <queue>
#include <fstream>
#include <type_traits>
#include <experimental/filesystem>

#include "cyber/trace/common.h"
#include "cyber/trace/writer.h"

#define TRACE_BEGIN(TASK_TYPE, TASK_ID, TRACE_CONFIG)    \
  century::trace::Trace<TRACE_CONFIG>::Instance()->Record( \
      TASK_TYPE, century::trace::TimestampType::BEGIN, TASK_ID);

#define TRACE_END(TASK_TYPE, TASK_ID, TRACE_CONFIG)      \
  century::trace::Trace<TRACE_CONFIG>::Instance()->Record( \
      TASK_TYPE, century::trace::TimestampType::END, TASK_ID);

namespace century {
namespace trace {

template <typename T>
class Trace {
 public:
  Trace(const Trace&) = delete;
  Trace& operator=(const Trace&) = delete;
  static Trace* Instance() {
    if (nullptr == instance_) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (nullptr == instance_) {
        instance_ = new Trace();
        atexit(Destroy);
      }
    }
    return instance_;
  }

  void Record(const TaskType& task_type, const TimestampType& time_type,
              uint64_t task_id) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    thread_local pid_t tid = syscall(SYS_gettid);
    thread_local timespec ts;
    thread_local TraceData td(Timespec2Nanoseconds(ts), tid, time_type,
                              task_type, task_id);
    clock_gettime(CLOCK_MONOTONIC, &ts);
    td.timestamp = Timespec2Nanoseconds(ts);
    td.time_type = time_type;
    td.task_type = task_type;
    td.task_id = task_id;
    writer_.Record(td);
  }

 private:
  static Trace* instance_;
  static std::mutex mtx_;
  std::atomic_bool stop_{true};
  Writer<T, TraceData> writer_;

  Trace() {
    stop_.store(false, std::memory_order_relaxed);
    writer_.Init();
  }

  ~Trace() {
    stop_.store(true, std::memory_order_release);
    writer_.Stop();
  }

  static void Destroy() {
    if (nullptr != instance_) {
      delete instance_;
      instance_ = nullptr;
    }
  }
};

template <typename T>
Trace<T>* Trace<T>::instance_ = nullptr;
template <typename T>
std::mutex Trace<T>::mtx_;

}  // namespace trace
}  // namespace century
