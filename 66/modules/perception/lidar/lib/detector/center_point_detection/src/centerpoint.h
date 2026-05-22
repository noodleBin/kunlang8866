
/**
** @filename: centerpoint.h
**/
#pragma once

#ifndef MODULE_NAME
#define MODULE_NAME "perception"
#else
#undef MODULE_NAME
#define MODULE_NAME "perception"
#endif

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <assert.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "NvOnnxConfig.h"
#include "cuda_runtime.h"
#include "common.h"
#include "lidar_scn.h"
#include "postprocess.h"
#include "tensorrt.h"
#include "timer.h"

#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensor.hpp"

#include "modules/perception/lidar/lib/detector/dnn_common/spconv/include/spconv/engine.hpp"
#include "modules/perception/common/perception_gflags.h"

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {

typedef struct float11 {
  float val[11];
} float11;

typedef struct float9 { float val[9]; } float9;

struct Box {
  float x;
  float y;
  float z;
  float w;
  float l;
  float h;
  float r;
  float score;
  int label;
  std::string class_name;
};

class CenterPointVoxel {
 public:
  CenterPointVoxel(CenterPointDetectionConfig& centerpoint_config);

  ~CenterPointVoxel();

  std::vector<Bndbox>& getbboxes();
  // tools
  void get_info(void);

  // model inference
  int doinfer(float* lidar_arr, int lidar_num, cudaStream_t& stream);

  const nvtype::half* scn_forward(const nvtype::half* lidar_points, int num_points, cudaStream_t& stream);

 private:
  constexpr static float s_normalize_intensity_value_ = 255.0f;

  cudaStream_t stream_ = NULL;
  float* d_points_ = nullptr;

  //////////////////////////////////////////////////
  // engine params
  //////////////////////////////////////////////////
  Params params_;
  bool verbose_ = false;

  SCNParameter lidar_scn_param_;
  nvtype::half* lidar_points_device_ = nullptr;
  nvtype::half* lidar_points_host_ = nullptr;
  size_t capacity_points_ = 0;
  size_t bytes_capacity_points_ = 0;

  std::shared_ptr<SCN> lidar_scn_;
  std::shared_ptr<TensorRT::Engine> trt_;
  std::shared_ptr<PostProcessCuda> post_;

  unsigned int* h_detections_num_;
  float* d_detections_;
  float* d_detections_reshape_;  // add d_detections_reshape_

  std::vector<half*> d_reg_;
  std::vector<half*> d_height_;
  std::vector<half*> d_dim_;
  std::vector<half*> d_rot_;
  std::vector<half*> d_vel_;
  std::vector<half*> d_hm_;
  // half* d_reg_[NUM_TASKS];
  // half* d_height_[NUM_TASKS];
  // half* d_dim_[NUM_TASKS];
  // half* d_rot_[NUM_TASKS];
  // half* d_vel_[NUM_TASKS];
  // half* d_hm_[NUM_TASKS];

  int reg_n_;
  int reg_c_;
  int reg_h_;
  int reg_w_;
  int height_c_;
  int dim_c_;
  int rot_c_;
  int vel_c_;
  int hm_c_[NUM_TASKS];

  unsigned int* d_voxel_indices;
  std::vector<int> sparse_shape;

  std::vector<float9> detections_;
  unsigned int h_mask_size_;
  uint64_t* h_mask_ = nullptr;

  std::vector<Bndbox> nms_pred_;
};

}  // namespace centerpoint
}  // namespace lidar
}  // namespace perception
}  // namespace century
