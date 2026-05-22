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

#include <stdint.h>

#include <memory>

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "modules/perception/common/perception_vis_bridge/vis_sink.h"
#include "modules/perception/common/perception_vis_bridge/vis_topic_registry.h"

namespace century {
namespace perception {

struct BridgeStats {
  uint64_t recv_count = 0;
  uint64_t null_count = 0;
  uint64_t oversize_count = 0;
  uint64_t write_fail_count = 0;
  uint64_t write_count = 0;
};

class RawVisBridge {
 public:
  RawVisBridge(cyber::Node* node, VisTopicSpec spec,
               std::unique_ptr<VisSink> sink);

  const BridgeStats& stats() const { return stats_; }
  const VisTopicSpec& spec() const { return spec_; }

 private:
  void OnRawMessage(
      const std::shared_ptr<cyber::message::RawMessage>& raw_msg);

  VisTopicSpec spec_;
  std::unique_ptr<VisSink> sink_;
  std::shared_ptr<cyber::Reader<cyber::message::RawMessage>> reader_;
  BridgeStats stats_;
  uint32_t next_sequence_num_ = 0;
};

}  // namespace perception
}  // namespace century
