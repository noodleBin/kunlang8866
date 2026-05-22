#pragma once
#include <stdint.h>

#include <cuda_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

void fisheye_undistort_resize_cuda(const unsigned char* input, int in_w,
                                   int in_h, const float* mapx,
                                   const float* mapy, unsigned char* output,
                                   int out_w, int out_h, cudaStream_t& stream);

void yuv422_to_rgb_cuda(const unsigned char* yuv_data, int width, int height,
                        unsigned char* rgb_data, cudaStream_t stream);

void yuyv_to_rgb_cuda(const unsigned char* yuyv_data, int width, int height,
                      unsigned char* rgb_output, cudaStream_t stream);

#ifdef __cplusplus
}
#endif
