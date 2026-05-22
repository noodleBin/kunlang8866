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

#include "cyber/common/log.h"
#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/util.h"
#include "cyber/threadlib/threadlib_conf_data.h"

namespace century {
namespace cyber {
ThreadlibConfData* ThreadlibConfData::instance_ = nullptr;
std::mutex ThreadlibConfData::mtx_;

ThreadlibConfData* ThreadlibConfData::Instance() {
  if (nullptr == instance_) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (nullptr == instance_) {
      instance_ = new ThreadlibConfData();
    }
  }
  return instance_;
}

ThreadlibConfData::ThreadlibConfData() {}

ThreadlibConfData::~ThreadlibConfData() {
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}

void ThreadlibConfData::SetProcessGroup(const std::string& process_group) {
  process_group_ = process_group;
}

const std::string& ThreadlibConfData::ProcessGroup() const {
  return process_group_;
}

void ThreadlibConfData::ParseThreadlibConf() {
  std::string conf("conf/threadlib/");
  conf.append(ProcessGroup()).append(".conf");
  inner_thr_confs_.clear();
  auto cfg_file = common::GetAbsolutePath(common::WorkRoot(), conf);
  proto::ThreadlibConfig cfg;
  if (common::PathExists(cfg_file) &&
      common::GetProtoFromFile(cfg_file, &cfg)) {
    for (auto& thr : cfg.threads()) {
      inner_thr_confs_[thr.name()] = thr;
    }
  } else {
    AWARN << "cannot parse threadlib config: " << cfg_file;
  }
}

const proto::ThreadlibConfig& ThreadlibConfData::Config() const {
  return config_;
}

const std::unordered_map<std::string, proto::UserThread>&
ThreadlibConfData::GetUserThreadConfig() const {
  return inner_thr_confs_;
}

}  // namespace cyber
}  // namespace century
