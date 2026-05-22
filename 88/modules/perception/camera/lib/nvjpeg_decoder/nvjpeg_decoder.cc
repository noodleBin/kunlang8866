#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_decoder.h"

#include <cstring>

#include "cyber/common/log.h"
#include "modules/perception/camera/lib/nvjpeg_decoder/fisheye_cuda.h"

namespace century {
namespace perception {
namespace camera {

NvJpegDecoder::NvJpegDecoder(std::string name, int device_id, int width,
                             int height, int channel, bool use_undistortion) {
  name_ = name;
  cudaSetDevice(device_id);
  nvjpegCreateSimple(&handle_);
  nvjpegJpegStateCreate(handle_, &state_);
  cudaStreamCreate(&stream_);
  cudaEventCreateWithFlags(&frame_ready_event_, cudaEventDisableTiming);
  width_ = width;
  height_ = height;
  channel_ = channel;
  use_undistortion_ = use_undistortion;

  size_t rgb_size = width_ * height_ * channel_;
  size_t yuv_size = width_ * height_ * 3 / 2;  // YUV420 = 1.5 bytes/pixel

  if (cudaMallocHost(&h_rgb_, rgb_size) != cudaSuccess) {
    AERROR << "cudaMallocHost failed: "
           << cudaGetErrorString(cudaGetLastError());
  }
  if (cudaMalloc(&d_rgb_, rgb_size) != cudaSuccess) {
    AERROR << "cudaMalloc failed: " << cudaGetErrorString(cudaGetLastError());
  }

  // Allocate YUV420 buffers
  cudaMalloc(&d_yuv_y_, width_ * height_);
  cudaMalloc(&d_yuv_u_, (width_ / 2) * (height_ / 2));
  cudaMalloc(&d_yuv_v_, (width_ / 2) * (height_ / 2));
  cudaMalloc(&d_yuv_temp_, yuv_size);

  // Allocate YUYV buffer (width * height * 2 bytes)
  cudaMalloc(&d_yuyv_temp_, width_ * height_ * 2);

  if (use_undistortion_) {
    cudaMalloc(&d_dst_, rgb_size);
  }
}

NvJpegDecoder::~NvJpegDecoder() {
  nvjpegJpegStateDestroy(state_);
  nvjpegDestroy(handle_);
  cudaStreamDestroy(stream_);
  cudaEventDestroy(frame_ready_event_);

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
  if (use_undistortion_) {
    cudaFree(d_dst_);
  }
}

bool NvJpegDecoder::SetDMap(std::vector<float>& dmapx,
                            std::vector<float>& dmapy) {
  cudaMalloc(&d_mapx_, width_ * height_ * sizeof(float));
  cudaMalloc(&d_mapy_, width_ * height_ * sizeof(float));
  cudaMemcpy(d_mapx_, dmapx.data(), width_ * height_ * sizeof(float),
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_mapy_, dmapy.data(), width_ * height_ * sizeof(float),
             cudaMemcpyHostToDevice);
  return true;
}

uint8_t* NvJpegDecoder::GetHostBuffer() const {
  uint8_t* src = use_undistortion_ ? d_dst_ : d_rgb_;
  cudaMemcpyAsync(h_rgb_, src, width_ * height_ * channel_,
                  cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);
  return h_rgb_;
}

uint8_t* NvJpegDecoder::GetDeviceBuffer() const {
  return use_undistortion_ ? d_dst_ : d_rgb_;
}

cudaStream_t NvJpegDecoder::GetStream() const {
  return external_stream_ ? external_stream_ : stream_;
}

bool NvJpegDecoder::Decode(const uint8_t* jpeg_data, size_t jpeg_size,
                           int* width, int* height) {
  std::lock_guard<std::mutex> lock(mutex_);

  *height = height_;
  *width = width_;

  memset(&output_, 0, sizeof(output_));
  output_.channel[0] = d_rgb_;
  output_.pitch[0] = (*width) * 3;

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  nvjpegStatus_t ret = nvjpegDecode(handle_, state_, jpeg_data, jpeg_size,
                                    NVJPEG_OUTPUT_RGBI, &output_, work_stream);

  if (use_undistortion_) {
    fisheye_undistort_resize_cuda(d_rgb_, *width, *height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return ret == NVJPEG_STATUS_SUCCESS;
}

bool NvJpegDecoder::ProcessYuv420(const uint8_t* raw_data, size_t data_size,
                                  int width, int height) {
  size_t expected_size = width * height * 3 / 2;  // YUV420 = 1.5 bytes/pixel
  if (data_size != expected_size) {
    AERROR << "YUV420 data size mismatch. Expected: " << expected_size
           << ", Got: " << data_size;
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  // Upload YUV420 data to GPU
  cudaError_t err = cudaMemcpyAsync(d_yuv_temp_, raw_data, data_size,
                                    cudaMemcpyHostToDevice, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to upload YUV420 data to GPU: "
           << cudaGetErrorString(err);
    return false;
  }

  // Split YUV420 planes
  size_t y_size = width * height;
  size_t uv_size = (width / 2) * (height / 2);

  // Y plane (full resolution)
  cudaMemcpyAsync(d_yuv_y_, d_yuv_temp_, y_size, cudaMemcpyDeviceToDevice,
                  work_stream);
  // U plane (quarter resolution)
  cudaMemcpyAsync(d_yuv_u_, d_yuv_temp_ + y_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);
  // V plane (quarter resolution)
  cudaMemcpyAsync(d_yuv_v_, d_yuv_temp_ + y_size + uv_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);

  // YUV420 -> RGB conversion on GPU
  yuv420_to_rgb_cuda(d_yuv_y_, d_yuv_u_, d_yuv_v_, width, height, d_rgb_,
                     work_stream);

  // Apply undistortion if enabled
  if (use_undistortion_) {
    fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  // Record completion event
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

  yuyv_to_rgb_cuda(d_yuyv_temp_, width, height, d_rgb_, work_stream);

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
  std::lock_guard<std::mutex> lock(mutex_);

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
  } else if ("yuyv" == encoding || "YUYV" == encoding || "yuv422" == encoding ||
             "UYVY" == encoding || "uyvy" == encoding) {
    success = ProcessYuyv(raw_data, data_size, width, height);
  } else if ("rgb8" == encoding || "RGB8" == encoding) {
    // Use the external low-priority stream when available.
    cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

    cudaMemcpyAsync(d_rgb_, raw_data, width * height * 3,
                    cudaMemcpyHostToDevice, work_stream);
    if (use_undistortion_) {
      fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                    d_dst_, width_, height_, work_stream);
    }
    cudaEventRecord(frame_ready_event_, work_stream);
    success = true;
  } else {
    AERROR << "DecodeRaw: Unsupported encoding: " << encoding;
    return false;
  }

  return success;
}

bool NvJpegDecoder::FillBlackImage() {
  std::lock_guard<std::mutex> lock(mutex_);
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;
  size_t rgb_size = width_ * height_ * channel_;

  cudaError_t err = cudaMemsetAsync(d_rgb_, 0, rgb_size, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to fill black RGB image: " << cudaGetErrorString(err);
    return false;
  }

  if (use_undistortion_) {
    err = cudaMemsetAsync(d_dst_, 0, rgb_size, work_stream);
    if (err != cudaSuccess) {
      AERROR << "Failed to fill black undistorted image: "
             << cudaGetErrorString(err);
      return false;
    }
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

void NvJpegDecoder::SetExternalStream(cudaStream_t external_stream) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Store the external stream for low-priority decode work.
  external_stream_ = external_stream;
  AINFO << "Set external (low-priority) stream for camera decoder: " << name_;
}

void NvJpegDecoder::ClearExternalStream() {
  std::lock_guard<std::mutex> lock(mutex_);
  external_stream_ = nullptr;
}

}  // namespace camera
}  // namespace perception
}  // namespace century
