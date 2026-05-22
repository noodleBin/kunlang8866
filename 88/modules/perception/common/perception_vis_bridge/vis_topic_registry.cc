/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *****************************************************************************/

#include "modules/perception/common/perception_vis_bridge/vis_topic_registry.h"

#include <cstdlib>
#include <utility>

namespace century {
namespace perception {
namespace {

constexpr char kTopicsEnv[] = "PERCEPTION_VIS_TOPICS";

bool IsKeyEnabled(const std::string& key) {
  const char* env = std::getenv(kTopicsEnv);
  if (nullptr == env || '\0' == env[0]) {
    return true;
  }
  const std::string value(env);
  size_t start = 0;
  while (start < value.size()) {
    size_t end = value.find(',', start);
    if (std::string::npos == end) {
      end = value.size();
    }
    if (value.substr(start, end - start) == key) {
      return true;
    }
    start = end + 1;
  }
  return false;
}

}  // namespace

VisTopicRegistry& VisTopicRegistry::Instance() {
  static VisTopicRegistry instance;
  return instance;
}

void VisTopicRegistry::Register(VisTopicSpec spec) {
  specs_.emplace_back(std::move(spec));
}

std::vector<VisTopicSpec> VisTopicRegistry::EnabledTopics() const {
  std::vector<VisTopicSpec> enabled;
  enabled.reserve(specs_.size());
  for (const auto& spec : specs_) {
    if (IsKeyEnabled(spec.key)) {
      enabled.emplace_back(spec);
    }
  }
  return enabled;
}

VisTopicAutoRegister::VisTopicAutoRegister(std::string key, std::string topic,
                                           std::string msg_type) {
  VisTopicRegistry::Instance().Register(
      {std::move(key), std::move(topic), std::move(msg_type)});
}

}  // namespace perception
}  // namespace century
