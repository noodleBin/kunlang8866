#pragma once

#include <cuda.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>
#include "modules/perception/lidar/lib/detector/dnn_common/proto/centerpoint_detection_config.pb.h"

const unsigned int MAX_DET_NUM = 1000;  // nms_pre_max_size = 1000;
const unsigned int DET_CHANNEL = 9;
const unsigned int MAX_POINTS_NUM = 300000;
const unsigned int NUM_TASKS = 1;

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {

#define checkCudaErrors(op)                                                                  \
  {                                                                                          \
    auto status = ((op));                                                                    \
    if (status != 0) {                                                                       \
      std::cout << "Cuda failure: " << cudaGetErrorString(status) << " in file " << __FILE__ \
                << ":" << __LINE__ << " error status: " << status << std::endl;              \
      abort();                                                                               \
    }                                                                                        \
  }

class Params {
 public:
  // must be const value
  // unsigned int task_num_stride[NUM_TASKS] = {0, 1, 3, 6, 8, 13};
  std::vector<int> task_num_stride = {0, 1, 3, 6, 8, 13};
  int num_classes = 15;

  float out_size_factor = 8.0;
  float voxel_size[2] = { 0.05, 0.05, };
  float pc_range[2] = { -70.4, -40, };
  float post_center_range[6] = {-70.4, -40, -3, 70.4, 40, 5, };
  float nms_iou_threshold = 0.2;
  int nms_pre_max_size = MAX_DET_NUM;
  int nms_post_max_size = 83;

  // suggest be value
  int max_points_per_voxel = 5;
  int max_points = 300000;
  int max_voxels = 130000;

  // dynamic value follow the model config
  float score_threshold = 0.4;
  std::vector<float> score_thresholds = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 
                                         0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
  std::vector<float> nms_iou_thresholds = {
    0.01, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.20, 0.20, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.20, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.20, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01
  };
  float min_x_range = -70.4;
  float max_x_range = 70.4;
  float min_y_range = -40;
  float max_y_range = 40;
  float min_z_range = -3;
  float max_z_range = 5;
  // the size of a pillar
  float pillar_x_size = 0.05;
  float pillar_y_size = 0.05;
  float pillar_z_size = 0.2;
  int feature_num = 4;
  float z_offset = 0.0;
  int num_tasks = 6;

  std::vector<std::string> class_names = {
      "Pedestrian","Car","IGV-Full","Truck","Trailer-Empty","Trailer-Full","IGV-Empty","Crane","OtherVehicle","Cone","ContainerForklift",
      "Forklift","Lorry","ConstructionVehicle","WheelCrane"};
  std::string box_type = "lwh2lwh";  // lwh2lwh or lwh2wlh

  Params() {
  }

  void load_config(CenterPointDetectionConfig& model_cfg) {
    task_num_stride.clear();
    score_thresholds.clear();
    nms_iou_thresholds.clear();
    // assert (model_cfg.num_tasks() == NUM_TASKS);
    num_tasks = model_cfg.num_tasks();
    for (size_t i = 0; i < model_cfg.task_num_stride_size(); i++) {
      task_num_stride.emplace_back(model_cfg.task_num_stride(i));
    }
    num_classes = model_cfg.num_class();
    out_size_factor = model_cfg.out_size_factor();
    nms_pre_max_size = model_cfg.nms_pre_max_size();
    nms_post_max_size = model_cfg.nms_post_max_size();
    score_threshold = model_cfg.score_threshold();
    nms_iou_threshold = model_cfg.nms_iou_threshold();
    max_points_per_voxel = model_cfg.max_points_per_voxel();
    max_points = model_cfg.max_points();
    max_voxels = model_cfg.max_voxels_num();
    feature_num = model_cfg.feature_num();
    min_x_range = model_cfg.range_config().min_range().x();
    max_x_range = model_cfg.range_config().max_range().x();
    min_y_range = model_cfg.range_config().min_range().y();
    max_y_range = model_cfg.range_config().max_range().y();
    min_z_range = model_cfg.range_config().min_range().z();
    max_z_range = model_cfg.range_config().max_range().z();

    pillar_x_size = model_cfg.range_config().voxel_size().x();
    pillar_y_size = model_cfg.range_config().voxel_size().y();
    pillar_z_size = model_cfg.range_config().voxel_size().z();

    class_names.clear();
    for (size_t i = 0; i < model_cfg.class_names_size(); i++) {
      class_names.emplace_back(model_cfg.class_names(i));
    }

    for (size_t i = 0; i < model_cfg.score_threshold_config().score_threshold_size(); i++) {
      score_thresholds.emplace_back(model_cfg.score_threshold_config().score_threshold(i));
    }

    for (size_t i = 0; i < model_cfg.centerpoint_nms_iou_config().nms_iou_size(); i++) {
      nms_iou_thresholds.emplace_back(model_cfg.centerpoint_nms_iou_config().nms_iou(i));
    }

    assert (num_classes == class_names.size());
    voxel_size[0] = pillar_x_size;
    voxel_size[1] = pillar_y_size;
    pc_range[0] = min_x_range;
    pc_range[1] = min_y_range;
  }

  int getGridXSize() {
    return static_cast<int>(std::round((max_x_range - min_x_range) / pillar_x_size));
  }
  int getGridYSize() {
    return static_cast<int>(std::round((max_y_range - min_y_range) / pillar_y_size));
  }
  int getGridZSize() {
    return static_cast<int>(std::round((max_z_range - min_z_range) / pillar_z_size));
  }
};

}  // namespace centerpoint
}  // namespace lidar
}  // namespace perception
}  // namespace century
