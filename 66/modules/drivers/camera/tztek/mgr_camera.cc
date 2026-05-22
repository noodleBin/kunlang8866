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

#include "mgr_camera.h"

#define SKIP_NUM 1

namespace century {
namespace drivers {
namespace camera {

CCameraMgr::CCameraMgr(
    int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps,
    int format, std::shared_ptr<Writer<Image>> image_writer,
    std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
    const std::string frame_id, bool output_raw) {
  m_nChan = dwPipeId;
  m_dwVideoIndex = dwVideoIndex;
  m_dwWidth = dwWidth;
  m_dwHeight = dwHeight;
  m_nFps = dwFps;
  m_dwFormat = format;
  m_llTimestamp = 0;
  m_llFrame = 0;
  frame_id_ = frame_id;
  image_.reset(new Image);
  image_->mutable_header()->set_frame_id(frame_id_);
  image_->set_width(dwWidth);
  image_->set_height(dwHeight);
  image_->mutable_data()->reserve(dwWidth * dwHeight);
  image_->set_encoding("yuyv");
  image_->set_step(2 * dwWidth);
  image_writer_ = image_writer;
  compressed_image_writer_ = compressed_image_writer;
  output_raw_ = output_raw;
}

CCameraMgr::~CCameraMgr() {}

bool CCameraMgr::Init() {
  if (creatHandle()) {
    AINFO << "camera manager channel " << m_nChan << " creatHandle success.";
    if (initHandle()) {
      AINFO << "camera manager channel " << m_nChan << " initHandle success.";
      AINFO << "camera manager init channel " << m_nChan << " success.";
      return true;
    } else {
      AERROR << "camera manager channel " << m_nChan << " initHandle failed.";
    }
  } else {
    AERROR << "camera manager channel " << m_nChan << " creatHandle failed.";
  }  
  AERROR << "camera manager init channel " << m_nChan << " failed.";
  return false;
}

void CCameraMgr::Start() { m_pCameraBase->StartAcquire(); }

bool CCameraMgr::creatHandle() {
  m_pCameraBase.reset(new CV4l2Cam(m_nChan, m_dwVideoIndex, m_dwWidth,
                                   m_dwHeight, m_nFps, m_dwFormat));
  if (m_pCameraBase == nullptr) {
    AERROR << "creatHandle failed, /dev/video" << m_nChan;
    return false;
  }

  m_pCamerajpeg.reset(new CMgrCameraJpegEnc(
      m_nChan, m_dwWidth, m_dwHeight, 90, compressed_image_writer_, frame_id_));
  if (m_pCamerajpeg == nullptr) {
    AERROR << "creatHandle failed, /dev/video" << m_nChan;
    return false;
  }
  return true;
}

bool CCameraMgr::initHandle() {
  int ret = 0;
  ret = m_pCameraBase->Init();
  if (ret < 0) {
    return false;
  }
  setCamPublish();
  return m_pCamerajpeg->Init();
}

void CCameraMgr::setCamPublish() {
  m_pCameraBase->SetDataCallback(&CCameraMgr::callbackImage, this);
}

void CCameraMgr::callbackImage(int nChan, struct timespec stTime, int nWidth,
                               int nHeight, unsigned char *pData, int nDatalen,
                               void *pUserData) {
  CCameraMgr *pCamMgr = (CCameraMgr *)pUserData;
  if (pCamMgr == NULL) {
    return;
  }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  struct timespec stv;
  stv.tv_sec = tv.tv_sec;
  stv.tv_nsec = tv.tv_usec * 1000;
  // long llTimestamp = stTime.tv_sec * 1000 + stTime.tv_nsec / 1000000;

  // long llsysDiff =
  //     stv.tv_sec * 1000 + stv.tv_nsec / 1000000 - pCamMgr->m_oldllTimestamp;
  pCamMgr->m_oldllTimestamp = stv.tv_sec * 1000 + stv.tv_nsec / 1000000;

  // long dwDiff = llTimestamp - pCamMgr->m_llTimestamp;
  pCamMgr->m_llTimestamp = stTime.tv_sec * 1000 + stTime.tv_nsec / 1000000;
  // long diff_ms = stv.tv_sec * 1000 + stv.tv_nsec / 1000000 - llTimestamp;
  // debug_info(
  //     "[chan:%d]: frametime:%ld.%09ld,systime:%ld.%09ld,delay:%ldms "
  //     "interval:%ldms,llsysDiff:%ldms\n",
  //     nChan, stTime.tv_sec, stTime.tv_nsec, stv.tv_sec, stv.tv_nsec, diff_ms,
  //     dwDiff, llsysDiff);
  pCamMgr->m_llFrame++;

  if (pCamMgr->output_raw_) {
    auto measurement_time =
        (double)stTime.tv_sec + (double)stTime.tv_nsec / 1e9;
    pCamMgr->image_->mutable_header()->set_timestamp_sec(
        century::cyber::Time::Now().ToSecond());
    pCamMgr->image_->set_measurement_time(measurement_time);
    pCamMgr->image_->set_data(pData, nDatalen);
    pCamMgr->image_writer_->Write(pCamMgr->image_);
  }

  pCamMgr->m_pCamerajpeg->Save(nChan, stTime, nWidth, nHeight, pData, nDatalen);

  return;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century