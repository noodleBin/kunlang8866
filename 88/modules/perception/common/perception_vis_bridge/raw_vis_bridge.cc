/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *****************************************************************************/

#include "modules/perception/common/perception_vis_bridge/raw_vis_bridge.h"

#include <utility>

#include "modules/perception/common/perception_vis_bridge/perception_vis_shm.h"
#include "modules/perception/common/perception_vis_bridge/vis_sink.h"

namespace century {
namespace perception {
namespace {

constexpr uint64_t kVerboseStartupLogCount = 3;
constexpr uint64_t kPeriodicLogInterval = 50;

bool ShouldLog(const uint64_t count) {
  return count <= kVerboseStartupLogCount ||
         0 == (count % kPeriodicLogInterval);
}

}  // namespace

RawVisBridge::RawVisBridge(cyber::Node* node, VisTopicSpec spec,
                           std::unique_ptr<VisSink> sink)
    : spec_(std::move(spec)), sink_(std::move(sink)) {
  reader_ = node->CreateReader<cyber::message::RawMessage>(
      spec_.topic,
      [this](const std::shared_ptr<cyber::message::RawMessage>& raw_msg) {
        OnRawMessage(raw_msg);
      });
  AINFO << "Created raw perception vis bridge for topic: " << spec_.topic
        << ", msg_type: " << spec_.msg_type;
}

void RawVisBridge::OnRawMessage(
    const std::shared_ptr<cyber::message::RawMessage>& raw_msg) {
  if (nullptr == raw_msg) {
    ++stats_.null_count;
    if (ShouldLog(stats_.null_count)) {
      AWARN << "[perception_vis_bridge] topic=" << spec_.topic
            << " reason=null_message";
    }
    return;
  }

  // AERROR << "Received raw message for topic: " << spec_.topic
  //       << ", size: " << raw_msg->message.size();

  ++stats_.recv_count;
  const auto& payload = raw_msg->message;

  if (payload.size() > kPerceptionVisPayloadMaxBytes) {
    ++stats_.oversize_count;
    if (ShouldLog(stats_.oversize_count)) {
      AWARN << "[perception_vis_bridge] topic=" << spec_.topic
            << " reason=payload_too_large bytes=" << payload.size();
    }
    return;
  }

  if (nullptr == sink_) {
    return;
  }

  const auto* data = reinterpret_cast<const uint8_t*>(payload.data());
  const double now_sec = cyber::Time::Now().ToSecond();
  ++next_sequence_num_;
  VisMessageMeta meta;
  meta.topic = spec_.topic;
  meta.msg_type = spec_.msg_type;
  meta.sequence_num = next_sequence_num_;
  meta.header_timestamp_sec = now_sec;
  meta.measurement_time_sec = now_sec;
  if (!sink_->Write(meta, data, payload.size())) {
    ++stats_.write_fail_count;
    if (ShouldLog(stats_.write_fail_count)) {
      AWARN << "[perception_vis_bridge] topic=" << spec_.topic
            << " reason=sink_write_failed";
    }
    return;
  }

  ++stats_.write_count;
  if (ShouldLog(stats_.write_count)) {
    AINFO << "[perception_vis_bridge] topic=" << spec_.topic
          << " msg_type=" << spec_.msg_type
          << " recv_count=" << stats_.recv_count
          << " write_count=" << stats_.write_count
          << " payload_size=" << payload.size();
  }
}

}  // namespace perception
}  // namespace century
