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

#ifndef _H_V4L2CAM_H
#define _H_V4L2CAM_H
#include <semaphore.h>
#include <string.h>

#include <atomic>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <linux/videodev2.h>

#include "NvBufSurface.h"
#include "camera_type.h"
#include "v4l2_buffer_config.h"
#include "v4l2_thread_config.h"
#include "v4l2_timing_config.h"

#include "cyber/cyber.h"

#ifndef CAMERA_LATENCY_PROBE_ENABLED
#define CAMERA_LATENCY_PROBE_ENABLED 1
#endif

struct NvBuffer {
  unsigned char *start;
  unsigned int size;
  int dmabuff_fd;
};

struct NvColorFormat {
  unsigned int v4l2_pixfmt;
  NvBufSurfaceColorFormat nvbuf_color;
};

#define TIMESTAMP_MODE_HARD 0
#define TIMESTAMP_MODE_HARD_ONCE 1
#define TIMESTAMP_MODE_SOFT 2

using V4l2DataCallback = void (*)(int channel, struct timespec frame_time,
                                  int width, int height, unsigned char *data,
                                  int data_len, void *user_data);

class V4l2Camera {
 public:
  V4l2Camera(int pipe_id, int video_index, int width, int height, int fps,
           int format, int buffer_count,
           const century::drivers::camera::V4l2TimingOptions& timing_options,
           const century::drivers::camera::V4l2ThreadOptions& thread_options);
  ~V4l2Camera();
  int Init();
  int Release();
  int StartAcquire();
  int StopAcquire();
  void GrabRoutine();
  void SetDataCallback(V4l2DataCallback callback, void *user_data);
  int GetFps() { return fps_; }
  int GetWidth() const { return static_cast<int>(width_); }
  int GetHeight() const { return static_cast<int>(height_); }
  uint32_t GetPixelFormat() const { return v4l2_pixel_format_; }
  int ClearBuffer();

 protected:
  int GrabImage();
  int StartCapture();
  int StopCapture();
  void RtcpuToRealtime(timeval rtcpu_time, timespec *real_time);
  bool InitDevice();
  bool ReleaseDevice();
  bool ClearBufferInternal();
  bool InitDma();
  bool SetVideoFormat();
  bool SetStreamFps();
  bool CheckCapabilities();
  int Xioctl(int fd, unsigned long request, void *arg);
  int WaitForBufferReady(int timeout_ms);
  int QueueDmabuf(uint32_t index);
  int GetOffset();
  void RecoverStream();

 private:
  std::atomic<bool> is_running_{false};
  int fault_;
  int channel_;
  int fps_;
  int error_code_;
  long skipped_frame_count_;
  uint32_t frame_count_;
  int32_t video_fd_;
  uint32_t height_;
  uint32_t width_;
  std::string device_name_;
  std::deque<timespec> timestamp_queue_;
  sem_t trigger_semaphore_;
  std::mutex mutex_;
  std::unique_ptr<std::thread> grab_thread_ = nullptr;
  V4l2DataCallback data_callback_ = nullptr;
  void *user_data_ = nullptr;
  bool is_first_capture_;
  bool is_streaming_{false};
  bool is_stream_param_unsupported_{false};
  uint32_t consecutive_grab_fail_count_{0};
  uint32_t last_sequence_{0};
  int last_dq_errno_{0};
  uint64_t last_frame_timestamp_;
  long nsec_offset_;
  uint32_t v4l2_pixel_format_{V4L2_PIX_FMT_YUYV};
  int buffer_count_{century::drivers::camera::kDefaultV4l2BufferCount};
  century::drivers::camera::V4l2TimingOptions timing_options_;
  century::drivers::camera::V4l2ThreadOptions thread_options_;

  struct timeval last_v4l2_timestamp_;
  int config_format_;
  NvBuffer *nv_buffers_ = nullptr;
  NvBufSurface *surface_ = nullptr;
};
#endif
