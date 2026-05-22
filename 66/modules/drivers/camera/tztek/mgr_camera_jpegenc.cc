#include "mgr_camera_jpegenc.h"

#include "libyuv.h"

namespace century {
namespace drivers {
namespace camera {
void CAMERA_CALL JpegEncCallBack(unsigned char *data, int datalen,
                                 void *userdata) {
  CMgrCameraJpegEnc *pOper = (CMgrCameraJpegEnc *)userdata;
  if (pOper == NULL) {
    return;
  }

  struct timespec timespec_;
  {
    std::lock_guard<std::mutex> lock(pOper->m_mutex);
    if (pOper->m_stTimes.empty()) {
      return;
    }

    timespec_ = pOper->m_stTimes.front();
    pOper->m_stTimes.pop();
  }

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  // struct timespec stv;
  // stv.tv_sec = tv.tv_sec;
  // stv.tv_nsec = tv.tv_usec * 1000;
  // long llTimestamp = timespec_.tv_sec * 1000 + timespec_.tv_nsec / 1000000;
  // long diff_ms = stv.tv_sec * 1000 + stv.tv_nsec / 1000000 - llTimestamp;
  // AINFO << "jpeg diff_ms: " << diff_ms;

  auto measurement_time =
      (double)timespec_.tv_sec + (double)timespec_.tv_nsec / 1e9;
  // pOper->compressed_image_->mutable_header()->set_timestamp_sec(
  //     century::cyber::Time::Now().ToSecond());
  pOper->compressed_image_->mutable_header()->set_timestamp_sec(
      measurement_time);
  pOper->compressed_image_->set_data(data, datalen);
  // pOper->compressed_image_->set_measurement_time(measurement_time);
  pOper->compressed_image_->set_measurement_time(
      century::cyber::Time::Now().ToSecond());
  pOper->compressed_image_writer_->Write(pOper->compressed_image_);
}

CMgrCameraJpegEnc::CMgrCameraJpegEnc(
    int dwChan, int dwWidth, int dwHeight, int dwQuality,
    std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
    const std::string frame_id) {
  m_dwChan = dwChan;
  m_dwWidth = dwWidth;
  m_dwHeight = dwHeight;
  m_dwQuality = dwQuality;
  compressed_image_writer_ = compressed_image_writer;
  compressed_image_.reset(new CompressedImage);
  compressed_image_->mutable_header()->set_frame_id(frame_id);
  compressed_image_->mutable_data()->reserve(dwWidth * dwHeight);
  compressed_image_->set_format("jpeg");
}

CMgrCameraJpegEnc::~CMgrCameraJpegEnc() { Release(); }

bool CMgrCameraJpegEnc::createHandle() {
  JPEGENC_PARA para;
  memset(&para, 0, sizeof(para));
  para.pixfmt = 0;
  para.userbuf = 1;
  para.bufsize = 0;
  para.width = m_dwWidth;
  para.height = m_dwHeight;
  para.quality = m_dwQuality;
  para.mode = 2;
  para.gpuid = 0;
  m_pEncHandle = JPEGENC_CreateHandle(&para);
  if (m_pEncHandle == nullptr) {
    AERROR << "JPEGENC_CreateHandle failed,chan[" << m_dwChan << "]";
    return false;
  }
  JPEGENC_SetDataCallBack(m_pEncHandle, JpegEncCallBack, this);

  return true;
}

bool CMgrCameraJpegEnc::Init() {
  m_pYuv = new unsigned char[m_dwWidth * m_dwHeight * 3 / 2];
  if (!m_pYuv) {
    AERROR << "calloc  yuv buffer failed.";
    return false;
  }

  return createHandle();
}

bool CMgrCameraJpegEnc::Release() {
  destroyHandle();
  return true;
}

bool CMgrCameraJpegEnc::Save(int nChan, struct timespec stTime, int nWidth,
                             int nHeight, unsigned char *pData, int nDatalen) {
  if (pData == nullptr) {
    return false;
  }

  unsigned char *y_plane = m_pYuv;
  unsigned char *u_plane = y_plane + nWidth * nHeight;
  unsigned char *v_plane = u_plane + nWidth * nHeight / 4;
  libyuv::YUY2ToI420(pData, 2 * nWidth, y_plane, nWidth, u_plane, nWidth / 2,
                     v_plane, nWidth / 2, nWidth, nHeight);

  if (JPEGENC_InputData(m_pEncHandle, (unsigned char *)m_pYuv,
                        nWidth * nHeight * 3 / 2) < 0) {
    AERROR << "JPEGENC_InputData failed.";
    return false;
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_stTimes.push(stTime);
  return true;
}

bool CMgrCameraJpegEnc::destroyHandle() {
  if (m_pEncHandle) {
    JPEGENC_DestroyHandle(m_pEncHandle);
    m_pEncHandle = nullptr;
  }
  if (m_pYuv) {
    delete[] m_pYuv;
    m_pYuv = nullptr;
  }
  return true;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century