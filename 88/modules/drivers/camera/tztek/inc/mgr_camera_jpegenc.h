/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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
#include <atomic>
#include <cstdint>
#include <deque>
#include <linux/videodev2.h>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "camera_type.h"
#include "jpeg_encode_api.h"
#include "vin_log.h"

#include "cyber/cyber.h"
#include "modules/drivers/proto/sensor_image.pb.h"

namespace century {
namespace drivers {
namespace camera {
using century::cyber::Reader;
using century::cyber::Writer;
using century::drivers::Image;
// using century::drivers::camera::config::Config;
class CameraJpegEncoder {
 public:
  CameraJpegEncoder(
      int channel, int width, int height, int quality,
      std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
      const std::string frame_id);
  ~CameraJpegEncoder();
  virtual bool Init();
  virtual bool Release();
  bool SaveI420(int input_channel, struct timespec frame_time, int width,
                int height, unsigned char *data, int data_len);
  friend void CAMERA_CALL JpegEncCallback(unsigned char *data, int data_len,
                                          void *user_data);

 protected:
  bool CreateHandle();
  bool DestroyHandle();

 private:
  int channel_;
  int width_;
  int height_;
  int quality_;
  std::string frame_id_;
  std::atomic<bool> is_initialized_{false};
  std::atomic<bool> is_releasing_{false};
  std::atomic<uint32_t> dropped_frame_count_{0};
  std::atomic<uint32_t> input_fail_count_{0};
  void *encoder_handle_ = nullptr;
  std::mutex input_mutex_;
  std::mutex queue_mutex_;
  std::vector<uint8_t> yuv_buffer_;
  std::deque<struct timespec> timestamps_;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;
};
}  // namespace camera
}  // namespace drivers
}  // namespace century
