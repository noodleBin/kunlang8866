/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *****************************************************************************/

#include "modules/perception/common/perception_vis_bridge/vis_sink.h"

#include <cstring>

namespace century {
namespace perception {
namespace {

template <size_t N>
void CopyString(const std::string& src, char (&dst)[N]) {
  static_assert(N > 0, "destination buffer must not be empty");
  std::memset(dst, 0, N);
  std::strncpy(dst, src.c_str(), N - 1);
}

}  // namespace

ShmVisSink::ShmVisSink(const std::string& topic)
    : queue_(std::make_unique<
             cyber::transport::shm_queue<PerceptionVisRawShm>>(topic)),
      buffer_(std::make_unique<PerceptionVisRawShm>()) {}

bool ShmVisSink::Write(const VisMessageMeta& meta, const uint8_t* data,
                       size_t size) {
  if (nullptr == queue_ || nullptr == buffer_) {
    return false;
  }
  auto& shm_message = *buffer_;
  shm_message = PerceptionVisRawShm{};
  CopyString(meta.topic, shm_message.topic);
  CopyString(meta.msg_type, shm_message.msg_type);
  shm_message.sequence_num = meta.sequence_num;
  shm_message.header_timestamp_sec = meta.header_timestamp_sec;
  shm_message.measurement_time_sec = meta.measurement_time_sec;
  shm_message.payload_size = static_cast<uint32_t>(size);
  if (size > 0 && nullptr != data) {
    std::memcpy(shm_message.payload, data, size);
  }
  return queue_->Write(&shm_message);
}

}  // namespace perception
}  // namespace century
