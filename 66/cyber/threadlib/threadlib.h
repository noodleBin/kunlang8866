/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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
#include <thread>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include "cyber/threadlib/threadlib_conf_data.h"
#include "cyber/scheduler/common/pin_thread.h"

namespace century {
namespace cyber {

class Thread {
 public:
  Thread() {}

  struct ThreadAttribute {
    std::string cpuset = "";
    std::string policy = "SCHED_OTHER";
    uint32_t prio = 0;
    ThreadAttribute() {}
    ThreadAttribute(const std::string& sched_cpuset,
                    const std::string& sched_policy, uint32_t sched_prio)
        : cpuset(sched_cpuset), policy(sched_policy), prio(sched_prio) {}
  };

  // use SFINAE to gurantee all threads can be constructed even if the
  // thread name is invalid.
  template <
      typename T, typename Func, typename... Args,
      std::enable_if_t<std::is_convertible<T, std::string>::value>* = nullptr>
  Thread(T&& name, Func&& f, Args&&... args)
      : thread_name_(std::forward<T>(name)),
        thread_(
            [this, f = std::forward<Func>(f),
             args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
              tid_.store(syscall(SYS_gettid), std::memory_order_release);
              ThreadFunction(f, args);
            }) {
    pthread_setname_np(thread_.native_handle(), thread_name_.c_str());
    SetThreadAttribute();
  }

  template <
      typename T, typename Func, typename... Args,
      std::enable_if_t<!std::is_convertible<T, std::string>::value>* = nullptr>
  Thread(T&& name, Func&& f, Args&&... args)
      : thread_([this, f = std::forward<Func>(f),
                 args = std::make_tuple(std::forward<Args>(args)...)]() {
          tid_ = syscall(SYS_gettid);
          ThreadFunction(f, args);
        }) {}

  Thread(Thread&& other) noexcept { *this = std::move(other); };

  Thread& operator=(const Thread&) = delete;

  Thread& operator=(Thread&& other) noexcept {
    if (this != &other) {
      thread_name_ = std::move(other.thread_name_);
      thread_ = std::move(other.thread_);
      thread_attr_ = std::move(thread_attr_);
    }
    return *this;
  };

  void Join() { thread_.join(); }

  void Detach() { thread_.detach(); }

  void Swap(Thread& other) noexcept {
    thread_.swap(other.thread_);
    thread_name_.swap(other.thread_name_);
  }

  bool Joinable() const noexcept { return thread_.joinable(); }

  std::thread::id GetId() const noexcept { return thread_.get_id(); }

  std::thread::native_handle_type NativeHandle() {
    return thread_.native_handle();
  }

  unsigned int HardwareConcurrency() noexcept {
    return thread_.hardware_concurrency();
  }

  std::string GetName() const noexcept { return thread_name_; }

  ThreadAttribute GetThreadAttribute() const noexcept { return thread_attr_; }

  ~Thread() {}

 private:
  std::string thread_name_;
  std::thread thread_;
  ThreadAttribute thread_attr_;
  std::atomic<pid_t> tid_{-1};

  template <typename Func, typename Tuple>
  void ThreadFunction(Func&& f, Tuple&& args) {
    ThreadFunction(std::forward<Func>(f), std::forward<Tuple>(args),
                   std::make_index_sequence<
                       std::tuple_size<std::decay_t<Tuple>>::value>{});
  }

  template <typename Func, typename Tuple, std::size_t... Indices,
            std::enable_if_t<!std::is_member_function_pointer<
                std::decay_t<Func>>::value>* = nullptr>
  void ThreadFunction(Func&& f, Tuple&& args, std::index_sequence<Indices...>) {
    std::forward<Func>(f)(std::get<Indices>(std::forward<Tuple>(args))...);
  }

  template <typename Func, typename Tuple, std::size_t... Indices,
            std::enable_if_t<std::is_member_function_pointer<
                std::decay_t<Func>>::value>* = nullptr>
  void ThreadFunction(Func&& f, Tuple&& args, std::index_sequence<Indices...>) {
    ThreadFunctionExpand(std::forward<Func>(f),
                         std::get<Indices>(std::forward<Tuple>(args))...);
  }

  template <typename Func, typename ObjType, typename... Args,
            std::enable_if_t<std::is_pointer<std::decay_t<ObjType>>::value>* =
                nullptr>
  void ThreadFunctionExpand(Func&& f, ObjType&& obj, Args&&... args) {
    (std::forward<ObjType>(obj)->*std::forward<Func>(f))(
        std::forward<Args>(args)...);
  }

  template <typename Func, typename ObjType, typename... Args,
            std::enable_if_t<!std::is_pointer<std::decay_t<ObjType>>::value>* =
                nullptr>
  void ThreadFunctionExpand(Func&& f, ObjType&& obj, Args&&... args) {
    (std::forward<ObjType>(obj).*
     std::forward<Func>(f))(std::forward<Args>(args)...);
  }

  /**
   * Do not overload SetUserThreadAttr(cpu_set, policy, priority),
   * as developers are not allowed to configure thread attribute by themselves.
   * Only stystem team can configure scheduling strategy.
   */
  void SetThreadAttribute() {
    auto inner_thr_confs_ =
        ThreadlibConfData::Instance()->GetUserThreadConfig();
    if (inner_thr_confs_.find(thread_name_) != inner_thr_confs_.end()) {
      auto& thread_conf = inner_thr_confs_[thread_name_];
      thread_attr_.cpuset = thread_conf.cpuset();
      thread_attr_.policy = thread_conf.policy();
      thread_attr_.prio = thread_conf.prio();
      std::vector<int> cpus;
      scheduler::ParseCpuset(thread_attr_.cpuset, &cpus);
      scheduler::SetSchedAffinity(&thread_, cpus, "range");
    }
    // there is a possiblity that "thread_" hasn't been executed but already
    // entered this section, which may cause setting thread configuarion with
    // tid=-1. Use atomic variable to solve this problem.
    int max_retry = 5;
    while (tid_.load(std::memory_order_acquire) == -1 && --max_retry) {
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    scheduler::SetSchedPolicy(&thread_, thread_attr_.policy, thread_attr_.prio,
                              tid_);
  }
};
}  // namespace cyber
}  // namespace century
