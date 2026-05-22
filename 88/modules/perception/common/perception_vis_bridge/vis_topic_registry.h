/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace century {
namespace perception {

struct VisTopicSpec {
  std::string key;
  std::string topic;
  std::string msg_type;
};

class VisTopicRegistry {
 public:
  static VisTopicRegistry& Instance();

  void Register(VisTopicSpec spec);
  std::vector<VisTopicSpec> EnabledTopics() const;
  const std::vector<VisTopicSpec>& AllTopics() const { return specs_; }

 private:
  VisTopicRegistry() = default;
  VisTopicRegistry(const VisTopicRegistry&) = delete;
  VisTopicRegistry& operator=(const VisTopicRegistry&) = delete;

  std::vector<VisTopicSpec> specs_;
};

class VisTopicAutoRegister {
 public:
  VisTopicAutoRegister(std::string key, std::string topic,
                       std::string msg_type);
};

}  // namespace perception
}  // namespace century

#define REGISTER_VIS_TOPIC(key_id, topic_str, msg_type_str)             \
  static ::century::perception::VisTopicAutoRegister                    \
      kPerceptionVisAutoReg_##key_id(#key_id, (topic_str), (msg_type_str))
