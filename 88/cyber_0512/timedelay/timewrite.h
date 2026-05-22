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

#include <libgen.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
// #include <chrono>
#include <cstdlib>
// #include <experimental/filesystem>
// #include <fstream>
// #include <iostream>
// #include <queue>
#include <string>
// #include <thread>
// #include <type_traits>
// #include <unordered_map>
// #include <utility>
// #include <vector>
// #include "lwrb.h"
// #include "concurrent_queue/concurrent_queue.h"
#include "cyber/common/global_data.h"
#include "cyber/threadlib/threadlib.h"
#include "cyber/trace/common.h"
#include "third_party/lwrb/include/lwrb.h"
#include "third_party/safec/include/safec.h"

#define DELAYTRACE_TOSTRING(x) _DELAYTRACE_TOSTRING(x)
#define _DELAYTRACE_TOSTRING(x) #x
#define _TIMEDELAY_TRACE(tag, ...) \
  century::timedelay::Timedelay::Instance()->Record##tag(__VA_ARGS__);

namespace century {
namespace timedelay {

namespace fs = std::experimental::filesystem;

std::string TimeDateString() {
  struct timeval tm;
  struct tm cur_tm;
  char buff[64];
  std::string timeString;
  gettimeofday(&tm, NULL);
  localtime_r(&tm.tv_sec, &cur_tm);
  snprintf_s(buff, sizeof(buff), "%04d%02d%02d%02d%02d%02d.%06ld",
             cur_tm.tm_year + 1900, cur_tm.tm_mon + 1, cur_tm.tm_mday,
             cur_tm.tm_hour, cur_tm.tm_min, cur_tm.tm_sec, tm.tv_usec);
  timeString = buff;
  return timeString;
}

class Timewrite {
 public:
  enum writetype {
    WRITETYPE_DEFAULT = 0,
    WRITETYPE_LIDAR,
    WRITETYPE_FUSION,
    WRITETYPE_PREDICTION,
    WRITETYPE_PLANNING,
    WRITETYPE_CONTROLL,
    WRITETYPE_CANBUS,
    WRITETYPE_MONITOR,
    WRITETYPE_LASTFLAGBIT
  };

  bool Init() {
    char* enable_trace_env = std::getenv(kEnableTimeDelayEnv_);
    if (enable_trace_env && (!strcmp(enable_trace_env, "true"))) {
      // enable_write_ = true;
    } else {
      enable_write_ = false;
      return false;
    }
    fs::path dir = fs::path(file_name_prefix_).parent_path();
    if (!PathExists(dir.c_str())) {
      std::cout << "cannot find directory" << dir << std::endl;
      return false;
    }
    enable_write_ = true;
    stop_.store(false, std::memory_order_relaxed);
    dump_thread_ = cyber::Thread(kDumpThreadName_, [this]() { WriteTrace(); });
    // std::thread mythread([this]() { WriteTrace(); });
    // mythread.detach();
    return true;
  }

  void Record(writetype type, const char* module, const char* data, int size) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    if (nullptr == lwrbinst_[type]) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (nullptr == lwrbinst_[type]) {
        lwrbinst_[type] = new lwrb_instance;
        if (nullptr == lwrbinst_[type]) {
          return;
        }
        lwrb_init(&lwrbinst_[type]->lwrb_instance_, lwrbinst_[type]->lwrb_buff,
                  sizeof(lwrbinst_[type]->lwrb_buff));
        lwrbinst_[type]->valid = true;
        lwrbinst_[type]->modulename = module;
      }
    }
    while (lwrbinst_[type]->pool_occupied_.test_and_set(
        std::memory_order_acquire)) {
    }
    lwrb_write(&lwrbinst_[type]->lwrb_instance_, data, size);
    lwrbinst_[type]->pool_occupied_.clear(std::memory_order_release);
  }

  void Stop() {
    if (!enable_write_) {
      return;
    }
    stop_.store(true, std::memory_order_relaxed);
    if (dump_thread_.Joinable()) {
      dump_thread_.Join();
    }
  }

 private:
  void WriteTrace() {
    char buff[2048];
    int fullsize;
    bool writeflag;
    while (!stop_.load(std::memory_order_acquire)) {
      writeflag = false;
      for (int i = 0; i < WRITETYPE_LASTFLAGBIT; ++i) {
        if (nullptr == lwrbinst_[i]) {
          continue;
        }

        if (!lwrbinst_[i]->writer_.is_open()) {
          file_name_ = file_name_prefix_ + lwrbinst_[i]->modulename + "_" +
                       TimeDateString() + "_" + std::to_string(getpid()) +
                       ".log";
          lwrbinst_[i]->writer_ = std::ofstream(file_name_, std::ios::out);
          lwrbinst_[i]->file_bytes_size_ = 0;
          if (!lwrbinst_[i]->writer_.is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            lwrb_reset(&lwrbinst_[i]->lwrb_instance_);
            continue;
          }
        }

        fullsize = lwrb_read(&lwrbinst_[i]->lwrb_instance_, buff, sizeof(buff));
        if (fullsize > 0) {
          lwrbinst_[i]->writer_.write(buff, fullsize);
          lwrbinst_[i]->writer_.flush();
          lwrbinst_[i]->file_bytes_size_ += fullsize;
          writeflag = true;
        } else {
          continue;
        }

        if (lwrbinst_[i]->file_bytes_size_ >= file_bytes_max_size_) {
          lwrbinst_[i]->writer_.close();
        }
      }

      if (!writeflag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    for (int i = 0; i < WRITETYPE_LASTFLAGBIT; ++i) {
      if (nullptr == lwrbinst_[i]) {
        continue;
      }
      if (!lwrbinst_[i]->writer_.is_open()) {
        lwrbinst_[i]->writer_.close();
      }
    }
  }

  bool PathExists(const char* path) {
    struct stat buffer;
    return (0 == stat(path, &buffer));
  }

 private:
  static const int lwrb_buff_size_ = 1024 * 100;
  static const int file_bytes_max_size_ = 20 * 1024 * 1024;
  struct lwrb_instance {
    lwrb lwrb_instance_{0};
    char lwrb_buff[lwrb_buff_size_]{0};
    std::ofstream writer_{0};
    bool valid{false};
    std::atomic_flag pool_occupied_{ATOMIC_FLAG_INIT};
    std::string modulename{0};
    uint32_t file_bytes_size_{0};
  };
  lwrb_instance* lwrbinst_[WRITETYPE_LASTFLAGBIT] = {nullptr};

  std::atomic_bool stop_{true};
  std::string file_name_;
  // std::string
  // file_name_prefix_="/home/liangzhou/workspace/code/Sample/demo1/log/delay_";
  std::string file_name_prefix_ = "/century/data/log/delay_";
  std::string kDumpThreadName_ = "timedelay";
  cyber::Thread dump_thread_;
  static constexpr const char* kEnableTimeDelayEnv_ = "ENABLE_TIMEDELAY";
  bool enable_write_ = false;
  std::mutex mtx_;
};
}  // namespace timedelay
}  // namespace century
