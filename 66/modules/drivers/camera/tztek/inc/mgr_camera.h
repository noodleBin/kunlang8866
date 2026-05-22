/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camera_type.h"
#include "jpeg_encode_api.h"
#include "libyuv.h"
#include "mgr_camera_jpegenc.h"
#include "v4l2cam.hpp"
#include "vin_log.h"

#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "cyber/cyber.h"

namespace century {
namespace drivers {
namespace camera {
using century::cyber::Reader;
using century::cyber::Writer;
using century::drivers::Image;
using century::drivers::camera::config::Config;

class CCameraMgr {
 public:
  CCameraMgr(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight,
             int dwFps, int format, std::shared_ptr<Writer<Image>> image_writer,
             std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
             const std::string frame_id, bool output_raw);
  ~CCameraMgr();
  bool Init();
  void Start();

  static void callbackImage(int nChan, struct timespec stTime, int nWidth,
                            int nHeight, unsigned char *pData, int nDatalen,
                            void *pUserData);

  friend void CAMERA_CALL JpegEncCallBack(unsigned char *data, int datalen,
                                          void *userdata);

  long m_llTimestamp;

 private:
  bool creatHandle();    
  bool initHandle();
  void setCamPublish();

 private:
  std::mutex mtx;         
  std::unique_ptr<CV4l2Cam> m_pCameraBase = nullptr;
  uint32_t m_dwHeight;
  uint32_t m_dwWidth;
  int m_nChan;
  int m_nFps;
  int m_dwFormat;
  int m_dwVideoIndex;
  long m_llFrame;
  long m_oldllTimestamp{0};
  std::string frame_id_;
  std::shared_ptr<Image> image_ = nullptr;
  std::unique_ptr<CMgrCameraJpegEnc> m_pCamerajpeg = nullptr;
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;
  bool output_raw_ = false;
};
}  // namespace camera
}  // namespace drivers
}  // namespace century