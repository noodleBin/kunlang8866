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

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <string>

#include "cyber/transport/shm/shm_queue.h"
#include "modules/perception/common/perception_vis_bridge/perception_vis_shm.h"

namespace century {
namespace perception {

struct VisMessageMeta {
  std::string topic;
  std::string msg_type;
  uint32_t sequence_num = 0;
  double header_timestamp_sec = 0.0;
  double measurement_time_sec = 0.0;
};

class VisSink {
 public:
  virtual ~VisSink() = default;
  virtual bool Write(const VisMessageMeta& meta, const uint8_t* data,
                     size_t size) = 0;
};

class ShmVisSink : public VisSink {
 public:
  explicit ShmVisSink(const std::string& topic);
  bool Write(const VisMessageMeta& meta, const uint8_t* data,
             size_t size) override;

 private:
  std::unique_ptr<cyber::transport::shm_queue<PerceptionVisRawShm>> queue_;
  std::unique_ptr<PerceptionVisRawShm> buffer_;
};

class NullVisSink : public VisSink {
 public:
  bool Write(const VisMessageMeta& /*meta*/, const uint8_t* /*data*/,
             size_t /*size*/) override {
    return true;
  }
};

}  // namespace perception
}  // namespace century
