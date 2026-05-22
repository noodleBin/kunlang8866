#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_decoder_jeston.h"

#include <cassert>
#include <cstring>

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace camera {

int NvJpegDecoder::globalFrameCount_ = 0;

NvJpegDecoder::NvJpegDecoder(std::string name, int device_id, int width,
                             int height, int channel, bool use_undistortion)
    : name_(name),
      device_id_(device_id),
      width_(width),
      height_(height),
      channel_(channel),
      use_undistortion_(use_undistortion) {
  cudaSetDevice(device_id_);
  cudaStreamCreate(&stream_);

  // Create CUDA event for frame completion signaling
  cudaEventCreateWithFlags(&frame_ready_event_, cudaEventDisableTiming);

  nvJpegDecoderImpl_.reset(::NvJPEGDecoder::createJPEGDecoder(name_.c_str()));

  cudaMalloc(&d_rgb_, width_ * height_ * 3);
  cudaMallocHost(&h_rgb_, width_ * height_ * 3);

  cudaMalloc(&d_yuv_y_, width_ * height_);
  cudaMalloc(&d_yuv_u_, (width_ / 2) * (height_ / 2));
  cudaMalloc(&d_yuv_v_, (width_ / 2) * (height_ / 2));

  // Allocate temporary YUV buffer for raw decoding (max size: YUV422)
  d_yuv_temp_size_ = width_ * height_ * 2;
  cudaMalloc(&d_yuv_temp_, d_yuv_temp_size_);

  // Allocate YUYV buffer (width * height * 2 bytes)
  d_yuyv_temp_size_ = width_ * height_ * 2;
  cudaMalloc(&d_yuyv_temp_, d_yuyv_temp_size_);

  if (use_undistortion_) {
    cudaMalloc(&d_dst_, width_ * height_ * 3);
  }
}

NvJpegDecoder::~NvJpegDecoder() {
  cudaSetDevice(device_id_);

  if (nv_buffer_) {
    nv_buffer_->deallocateMemory();
    delete nv_buffer_;
    nv_buffer_ = nullptr;
  }

  if (h_rgb_) {
    cudaFreeHost(h_rgb_);
  }
  if (d_rgb_) {
    cudaFree(d_rgb_);
  }
  if (d_yuv_y_) {
    cudaFree(d_yuv_y_);
  }
  if (d_yuv_u_) {
    cudaFree(d_yuv_u_);
  }
  if (d_yuv_v_) {
    cudaFree(d_yuv_v_);
  }
  if (d_yuv_temp_) {
    cudaFree(d_yuv_temp_);
  }
  if (d_yuyv_temp_) {
    cudaFree(d_yuyv_temp_);
  }
  if (d_dst_) {
    cudaFree(d_dst_);
  }
  if (d_mapx_) {
    cudaFree(d_mapx_);
  }
  if (d_mapy_) {
    cudaFree(d_mapy_);
  }
  if (frame_ready_event_) {
    cudaEventDestroy(frame_ready_event_);
  }

  cudaStreamDestroy(stream_);
}

bool NvJpegDecoder::ProcessYuv420(const uint8_t* raw_data, size_t data_size,
                                  int width, int height) {
  size_t expected_size = width * height * 3 / 2;
  if (data_size != expected_size) {
    AERROR << "YUV420 data size mismatch. Expected: " << expected_size
           << ", Got: " << data_size;
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  cudaError_t err = cudaMemcpyAsync(d_yuv_temp_, raw_data, data_size,
                                    cudaMemcpyHostToDevice, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to upload YUV420 data to GPU: "
           << cudaGetErrorString(err);
    return false;
  }

  size_t y_size = width * height;
  size_t uv_size = (width / 2) * (height / 2);

  cudaMemcpyAsync(d_yuv_y_, d_yuv_temp_, y_size, cudaMemcpyDeviceToDevice,
                  work_stream);
  cudaMemcpyAsync(d_yuv_u_, d_yuv_temp_ + y_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);
  cudaMemcpyAsync(d_yuv_v_, d_yuv_temp_ + y_size + uv_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);

  yuv420_to_rgb_cuda(reinterpret_cast<unsigned char*>(d_yuv_y_),
                     reinterpret_cast<unsigned char*>(d_yuv_u_),
                     reinterpret_cast<unsigned char*>(d_yuv_v_), width, height,
                     reinterpret_cast<unsigned char*>(d_rgb_), work_stream);

  if (use_undistortion_) {
    fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::ProcessYuyv(const uint8_t* raw_data, size_t data_size,
                                int width, int height) {
  size_t expected_size = width * height * 2;
  if (data_size != expected_size) {
    AERROR << "YUYV data size mismatch. Expected: " << expected_size
           << ", Got: " << data_size;
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  cudaError_t err = cudaMemcpyAsync(d_yuyv_temp_, raw_data, data_size,
                                    cudaMemcpyHostToDevice, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to upload YUYV data to GPU: " << cudaGetErrorString(err);
    return false;
  }

  yuyv_to_rgb_cuda(reinterpret_cast<unsigned char*>(d_yuyv_temp_), width,
                   height, reinterpret_cast<unsigned char*>(d_rgb_),
                   work_stream);

  if (use_undistortion_) {
    fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::DecodeRaw(const uint8_t* raw_data, size_t data_size,
                              int width, int height,
                              const std::string& encoding) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (width != width_ || height != height_) {
    AERROR << "DecodeRaw: Image dimensions mismatch. Expected: " << width_
           << "x" << height_ << ", Got: " << width << "x" << height;
    return false;
  }

  if (nullptr == raw_data || 0 == data_size) {
    AERROR << "DecodeRaw: Invalid input data";
    return false;
  }

  bool success = false;

  if ("yuv420" == encoding || "i420" == encoding || "I420" == encoding) {
    success = ProcessYuv420(raw_data, data_size, width, height);
  } else if ("yuv422" == encoding || "UYVY" == encoding || "uyvy" == encoding ||
             "yuyv" == encoding || "YUYV" == encoding) {
    success = ProcessYuyv(raw_data, data_size, width, height);
  } else {
    AERROR << "DecodeRaw: Unsupported encoding: " << encoding;
    return false;
  }

  if (success) {
    ++cam_frame_count_;
    ++globalFrameCount_;
  }

  return success;
}

bool NvJpegDecoder::Decode(const uint8_t* jpeg_data, size_t jpeg_size,
                           int* out_width, int* out_height) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (nullptr == jpeg_data || jpeg_size < 10) {
    return false;
  }

  size_t clean_size = 0;
  for (size_t i = 0; i < jpeg_size - 1; ++i) {
    if (0xFF == jpeg_data[i] && 0xD9 == jpeg_data[i + 1]) {
      clean_size = i + 2;
      break;
    }
  }
  if (0 == clean_size) {
    return false;
  }

  uint32_t pixfmt;
  uint32_t w, h;
  int ret = nvJpegDecoderImpl_->decodeToBuffer(
      &nv_buffer_, const_cast<uint8_t*>(jpeg_data),
      static_cast<uint32_t>(clean_size), &pixfmt, &w, &h);

  if (ret < 0) {
    nvJpegDecoderImpl_.reset();
    nvJpegDecoderImpl_.reset(::NvJPEGDecoder::createJPEGDecoder(name_.c_str()));
    return false;
  }

  *out_width = static_cast<int>(w);
  *out_height = static_cast<int>(h);

  assert(*out_width == width_);
  assert(*out_height == height_);

  if (!ConvertYuv420ToRgb(w, h)) {
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  if (use_undistortion_) {
    fisheye_undistort_resize_cuda(d_rgb_, static_cast<int>(w),
                                  static_cast<int>(h), d_mapx_, d_mapy_, d_dst_,
                                  width_, height_, work_stream);
  }

  // Record completion event
  cudaEventRecord(frame_ready_event_, work_stream);

  ++cam_frame_count_;
  ++globalFrameCount_;

  // Some bug in jetpack 5.1, "When images of the same resolution are
  // input consecutively, the output may remain the same"
  nvJpegDecoderImpl_.reset();
  nvJpegDecoderImpl_.reset(::NvJPEGDecoder::createJPEGDecoder(name_.c_str()));

  return true;
}

bool NvJpegDecoder::FillBlackImage() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;
  size_t bytes = static_cast<size_t>(width_) * height_ * channel_;

  cudaError_t err = cudaMemsetAsync(d_rgb_, 0, bytes, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to fill black RGB image: " << cudaGetErrorString(err);
    return false;
  }

  if (use_undistortion_) {
    err = cudaMemsetAsync(d_dst_, 0, bytes, work_stream);
    if (err != cudaSuccess) {
      AERROR << "Failed to fill black undistorted image: "
             << cudaGetErrorString(err);
      return false;
    }
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::ConvertYuv420ToRgb(uint32_t width, uint32_t height) {
  if (!nv_buffer_) {
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  uint32_t pitch_y = nv_buffer_->planes[0].fmt.stride;
  cudaMemcpy2DAsync(d_yuv_y_, width, nv_buffer_->planes[0].data, pitch_y, width,
                    height, cudaMemcpyHostToDevice, work_stream);

  if (nv_buffer_->n_planes >= 2) {
    uint32_t pitch_u = nv_buffer_->planes[1].fmt.stride;
    cudaMemcpy2DAsync(d_yuv_u_, width / 2, nv_buffer_->planes[1].data, pitch_u,
                      width / 2, height / 2, cudaMemcpyHostToDevice,
                      work_stream);
  }

  if (nv_buffer_->n_planes >= 3) {
    uint32_t pitch_v = nv_buffer_->planes[2].fmt.stride;
    cudaMemcpy2DAsync(d_yuv_v_, width / 2, nv_buffer_->planes[2].data, pitch_v,
                      width / 2, height / 2, cudaMemcpyHostToDevice,
                      work_stream);
  }

  yuv420_to_rgb_cuda(reinterpret_cast<unsigned char*>(d_yuv_y_),
                     reinterpret_cast<unsigned char*>(d_yuv_u_),
                     reinterpret_cast<unsigned char*>(d_yuv_v_), width, height,
                     reinterpret_cast<unsigned char*>(d_rgb_), work_stream);

  return true;
}

uint8_t* NvJpegDecoder::GetDeviceBuffer() const {
  return use_undistortion_ ? d_dst_ : d_rgb_;
}

uint8_t* NvJpegDecoder::GetHostBuffer() {
  uint8_t* src_ptr = use_undistortion_ ? d_dst_ : d_rgb_;
  cudaError_t err = cudaMemcpyAsync(h_rgb_, src_ptr, width_ * height_ * 3,
                                    cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);
  if (err != cudaSuccess) {
    AERROR << "cudaMemcpy to host failed: " << cudaGetErrorString(err);
  }
  return h_rgb_;
}

bool NvJpegDecoder::SetDMap(std::vector<float>& dmapx,
                            std::vector<float>& dmapy) {
  cudaSetDevice(device_id_);
  size_t size = width_ * height_ * sizeof(float);

  if (!d_mapx_) {
    cudaMalloc(&d_mapx_, size);
  }
  if (!d_mapy_) {
    cudaMalloc(&d_mapy_, size);
  }

  cudaMemcpyAsync(d_mapx_, dmapx.data(), size, cudaMemcpyHostToDevice, stream_);
  cudaMemcpyAsync(d_mapy_, dmapy.data(), size, cudaMemcpyHostToDevice, stream_);
  cudaStreamSynchronize(stream_);

  return true;
}

void NvJpegDecoder::SetExternalStream(cudaStream_t external_stream) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // Store the external stream for low-priority decode work.
  external_stream_ = external_stream;
  AINFO << "Set external (low-priority) stream for camera decoder: " << name_;
}

void NvJpegDecoder::ClearExternalStream() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  external_stream_ = nullptr;
}

cudaStream_t NvJpegDecoder::GetStream() const {
  return external_stream_ ? external_stream_ : stream_;
}

void NvJpegDecoder::SaveDecodedData(const std::string& filename) {
  if (!nv_buffer_) {
    return;
  }

  uint32_t width = nv_buffer_->planes[0].fmt.width;
  uint32_t height = nv_buffer_->planes[0].fmt.height;
  uint32_t num_planes = nv_buffer_->n_planes;

  FILE* fp = fopen(filename.c_str(), "wb");
  if (!fp) {
    return;
  }

  fwrite(&width, sizeof(uint32_t), 1, fp);
  fwrite(&height, sizeof(uint32_t), 1, fp);
  fwrite(&num_planes, sizeof(uint32_t), 1, fp);

  for (uint32_t i = 0; i < num_planes; ++i) {
    uint32_t pitch = nv_buffer_->planes[i].fmt.stride;
    fwrite(&pitch, sizeof(uint32_t), 1, fp);
  }

  for (uint32_t plane = 0; plane < num_planes; ++plane) {
    uint32_t plane_pitch = nv_buffer_->planes[plane].fmt.stride;
    uint32_t plane_height = (plane == 0) ? height : height / 2;
    size_t plane_size = plane_pitch * plane_height;

    fwrite(nv_buffer_->planes[plane].data, 1, plane_size, fp);
  }

  fclose(fp);
}

}  // namespace camera
}  // namespace perception
}  // namespace century
