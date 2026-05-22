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

#include "mgr_camera_jpegenc.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <sys/stat.h>

// #define SAVE_PIC
namespace century {
namespace drivers {
namespace camera {
namespace {
constexpr size_t kMaxPendingTimestamp = 64;
constexpr int kJpegMaxQuality = 100;
constexpr int kJpegMinQuality = 1;

double NormalizeTo100ms(const timespec& ts) {
  constexpr int64_t kNsPerSec = 1'000'000'000LL;
  constexpr int64_t kNsPerMs  = 1'000'000LL;
  // 
  int64_t ms_in_sec = ts.tv_nsec / kNsPerMs;  // [0, 999]
  int64_t ms_100 = (ms_in_sec / 100) * 100;

  int64_t new_ns =
      static_cast<int64_t>(ts.tv_sec) * kNsPerSec +
      ms_100 * kNsPerMs +
      (ts.tv_nsec - ms_in_sec * kNsPerMs);

  return static_cast<double>(new_ns) / kNsPerSec;
}
}  // namespace

void CAMERA_CALL JpegEncCallback(unsigned char *data, int data_len,
                                 void *user_data) {
  auto *encoder = static_cast<CameraJpegEncoder *>(user_data);
  if (encoder == nullptr || data == nullptr || data_len <= 0) {
    return;
  }
  if (encoder->is_releasing_.load() || !encoder->is_initialized_.load()) {
    return;
  }
  if (encoder->compressed_image_writer_ == nullptr) {
    return;
  }

  struct timespec frame_timespec;
  {
    std::lock_guard<std::mutex> lock(encoder->queue_mutex_);
    if (encoder->timestamps_.empty()) {
      AWARN << "jpeg callback timestamp queue empty, chan = " << encoder->channel_;
      return;
    }

    frame_timespec = encoder->timestamps_.front();
    encoder->timestamps_.pop_front();
  }

  // auto header_time = NormalizeTo100ms(frame_timespec);
  static std::atomic<uint64_t> sequence_num = {0};
  auto header_time = (double)frame_timespec.tv_sec + (double)frame_timespec.tv_nsec / 1e9;
  auto measurement_time = century::cyber::Time::Now().ToSecond();
  // auto delay_time = measurement_time - header_time;
  // AINFO << "compressed image send time delay: " << delay_time << " s.";
  // if (delay_time >= 0.1) {
  //   AWARN << "compressed image send time delay: " << delay_time << " s.";
  // }

  auto image = std::make_shared<CompressedImage>();
  image->mutable_header()->set_frame_id(encoder->frame_id_);
  image->mutable_header()->set_timestamp_sec(header_time);
  image->mutable_header()->set_sequence_num(
    static_cast<unsigned int>(sequence_num.fetch_add(1)));
  image->set_format("jpeg");
  image->set_measurement_time(measurement_time);
  image->set_data(reinterpret_cast<const char *>(data), data_len);
  encoder->compressed_image_writer_->Write(image);

#ifdef SAVE_PIC
  static std::map<int, time_t> last_save_time;
  static std::mutex save_mutex;
  const time_t current_time = frame_timespec.tv_sec;
  bool should_save = false;
  {
    std::lock_guard<std::mutex> lock(save_mutex);
    auto it = last_save_time.find(encoder->channel_);
    if (it == last_save_time.end() || it->second != current_time) {
      last_save_time[encoder->channel_] = current_time;
      should_save = true;
    }
  }

  if (should_save) {
    const char *save_dir = "/century/data/pics";
    struct stat st;
    if (stat(save_dir, &st) != 0) {
      if (mkdir(save_dir, 0755) != 0 && errno != EEXIST) {
        AWARN << "Failed to create directory: " << save_dir
              << ", errno: " << errno;
      }
    }

    char timestamp_str[32];
    struct tm tm_info;
    localtime_r(&current_time, &tm_info);
    strftime(timestamp_str, sizeof(timestamp_str), "%Y%m%d_%H%M%S", &tm_info);

    char jpeg_filename[256];
    snprintf(jpeg_filename, sizeof(jpeg_filename), "%s/%s_chan%d_%dx%d.jpeg",
             save_dir, timestamp_str, encoder->channel_, encoder->width_,
             encoder->height_);
    FILE *jpeg_fp = fopen(jpeg_filename, "wb");
    if (jpeg_fp) {
      fwrite(data, 1, data_len, jpeg_fp);
      fclose(jpeg_fp);
      AINFO << "Saved jpeg image: " << jpeg_filename << " (" << data_len
            << " bytes)";
    } else {
      AERROR << "Failed to save jpeg image: " << jpeg_filename;
    }
  }
#endif
}

CameraJpegEncoder::CameraJpegEncoder(
    int channel, int width, int height, int quality,
    std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
    const std::string frame_id) {
  channel_ = channel;
  width_ = width;
  height_ = height;
  quality_ = std::max(kJpegMinQuality, std::min(kJpegMaxQuality, quality));
  frame_id_ = frame_id;
  compressed_image_writer_ = compressed_image_writer;
}

CameraJpegEncoder::~CameraJpegEncoder() { Release(); }

bool CameraJpegEncoder::CreateHandle() {
  JPEGENC_PARA para;
  memset(&para, 0, sizeof(para));
  para.pixfmt = 0;
  para.userbuf = 1;
  // For TZFD mode, vendor encoder is most stable with internal default sizing.
  para.bufsize = 0;
  para.width = width_;
  para.height = height_;
  para.quality = quality_;
  para.mode = 2;
  para.gpuid = 0;
  encoder_handle_ = JPEGENC_CreateHandle(&para);
  if (encoder_handle_ == nullptr) {
    AERROR << "JPEGENC_CreateHandle failed,chan[" << channel_ << "]";
    return false;
  }
  if (JPEGENC_SetDataCallBack(encoder_handle_, JpegEncCallback, this) < 0) {
    AERROR << "JPEGENC_SetDataCallBack failed,chan[" << channel_ << "]";
    JPEGENC_DestroyHandle(encoder_handle_);
    encoder_handle_ = nullptr;
    return false;
  }

  return true;
}

bool CameraJpegEncoder::Init() {
  if (is_initialized_.load()) {
    return true;
  }
  if (compressed_image_writer_ == nullptr) {
    AERROR << "compressed image writer is null, chan[" << channel_ << "]";
    return false;
  }
  if (width_ <= 0 || height_ <= 0) {
    AERROR << "invalid jpeg encoder size, chan[" << channel_ << "], w="
           << width_ << ", h=" << height_;
    return false;
  }
  const int yuv_size = width_ * height_ * 3 / 2;
  if (yuv_size <= 0) {
    AERROR << "invalid yuv buffer size, chan[" << channel_ << "]";
    return false;
  }
  is_releasing_.store(false);
  dropped_frame_count_.store(0);
  input_fail_count_.store(0);
  yuv_buffer_.resize(static_cast<size_t>(yuv_size));
  if (yuv_buffer_.empty()) {
    AERROR << "allocate yuv buffer failed, chan[" << channel_ << "]";
    return false;
  }

  if (!CreateHandle()) {
    yuv_buffer_.clear();
    return false;
  }
  is_initialized_.store(true);
  return true;
}

bool CameraJpegEncoder::Release() {
  if (!is_initialized_.load() && encoder_handle_ == nullptr && yuv_buffer_.empty()) {
    return true;
  }
  is_releasing_.store(true);
  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    DestroyHandle();
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    timestamps_.clear();
  }
  is_initialized_.store(false);
  is_releasing_.store(false);
  return true;
}

bool CameraJpegEncoder::SaveI420(int input_channel, struct timespec frame_time,
                                 int width,
                                 int height, unsigned char *data,
                                 int data_len) {
  if (!is_initialized_.load() || is_releasing_.load()) {
    return false;
  }
  if (data == nullptr || data_len <= 0 || width <= 0 || height <= 0) {
    AWARN << "invalid save input, chan[" << channel_ << "], w=" << width
          << ", h=" << height << ", size=" << data_len;
    return false;
  }
  if (input_channel != channel_) {
    AWARN << "channel mismatch, expected[" << channel_ << "], got[" << input_channel
          << "]";
  }
  if (width != width_ || height != height_) {
    AERROR << "input size mismatch, expected(" << width_ << "x" << height_
           << "), got(" << width << "x" << height << ")";
    return false;
  }
  const int required_size = width * height * 3 / 2;
  if (required_size <= 0) {
    AERROR << "invalid required yuv size, required=" << required_size;
    return false;
  }

  if (data_len < required_size) {
    AERROR << "input I420 size too small, expected at least "
           << required_size << ", got " << data_len;
    return false;
  }

  auto rollback_timestamp = [this]() {
    std::lock_guard<std::mutex> qlock(queue_mutex_);
    if (!timestamps_.empty()) {
      timestamps_.pop_back();
    }
  };

  std::lock_guard<std::mutex> input_lock(input_mutex_);
  if (is_releasing_.load() || !is_initialized_.load() || encoder_handle_ == nullptr) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (timestamps_.size() >= kMaxPendingTimestamp) {
      const uint32_t drop_count = dropped_frame_count_.fetch_add(1) + 1;
      if (drop_count == 1 || drop_count % 100 == 0) {
        AWARN << "jpeg queue full, drop current frame, chan[" << channel_
              << "], dropped=" << drop_count;
      }
      return false;
    }
    timestamps_.push_back(frame_time);
  }

  if (required_size > static_cast<int>(yuv_buffer_.size())) {
    AERROR << "insufficient yuv buffer, required=" << required_size
           << ", allocated=" << yuv_buffer_.size();
    rollback_timestamp();
    return false;
  }

  memcpy(yuv_buffer_.data(), data, static_cast<size_t>(required_size));

  if (JPEGENC_InputData(encoder_handle_, yuv_buffer_.data(), required_size) < 0) {
    const uint32_t fail_count = input_fail_count_.fetch_add(1) + 1;
    if (fail_count == 1 || fail_count % 50 == 0) {
      AERROR << "JPEGENC_InputData failed, chan[" << channel_
             << "], fail_count=" << fail_count;
    }
    rollback_timestamp();
    return false;
  }
  input_fail_count_.store(0);
  return true;
}

bool CameraJpegEncoder::DestroyHandle() {
  if (encoder_handle_) {
    JPEGENC_SetDataCallBack(encoder_handle_, nullptr, nullptr);
    JPEGENC_DestroyHandle(encoder_handle_);
    encoder_handle_ = nullptr;
  }
  yuv_buffer_.clear();
  yuv_buffer_.shrink_to_fit();
  return true;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century
