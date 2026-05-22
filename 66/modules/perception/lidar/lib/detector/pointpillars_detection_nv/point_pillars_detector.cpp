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

#include "modules/perception/lidar/lib/detector/pointpillars_detection_nv/point_pillars_detector.h"

#include <cuda_runtime_api.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/lidar/common/lidar_object_util.h"

// #include
// "modules/perception/lidar/lib/detector/point_pillars_detection/params.h"

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetAbsolutePath;

void GetDeviceInfo(void) {
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  std::cout << "\nGPU has cuda devices: " << count << std::endl;
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    std::cout << "----device id: " << i << std::endl;
    std::cout << "GPU : " << prop.name << std::endl;
    std::cout << "Capbility: " << prop.major << ", " << prop.minor << std::endl;
    std::cout << "Global memory: " << (prop.totalGlobalMem >> 20) << std::endl;
    std::cout << "Const memory: " << (prop.totalConstMem >> 10) << std::endl;
    std::cout << "SM in a block: " << (prop.sharedMemPerBlock >> 10)
              << std::endl;
    std::cout << "warp size: " << (prop.warpSize) << std::endl;
    std::cout << "threads in a block: " << (prop.maxThreadsPerBlock)
              << std::endl;
    std::cout << "block dim: " << prop.maxThreadsDim[0] << ", "
              << prop.maxThreadsDim[1] << ", " << prop.maxThreadsDim[2]
              << std::endl;
    std::cout << "grid dim: (" << prop.maxGridSize[0] << ","
              << prop.maxGridSize[1] << "," << prop.maxGridSize[2] << ")"
              << std::endl;
  }
  //   printf("\n");
}

PointPillarsDetector::PointPillarsDetector() {}

PointPillarsDetector::~PointPillarsDetector() {
  checkRuntime(cudaStreamDestroy(stream_));
}

bool PointPillarsDetector::Init(const LidarDetectorInitOptions& options) {
  std::string config_file;
  if (options.cfg_file.empty()) {
    auto* config_manager = lib::ConfigManager::Instance();
    const lib::ModelConfig* model_config = nullptr;
    ACHECK(config_manager->GetModelConfig("PointCloudPreprocessor",
                                          &model_config));
    const std::string work_root = config_manager->work_root();

    config_file = GetAbsolutePath(work_root, "conf/perception/lidar");
  } else {
    config_file = options.cfg_file;
  }
  config_file = GetAbsolutePath(config_file, "pointpillars_detection.pb.txt");

  ACHECK(century::cyber::common::GetProtoFromFile(
      config_file, &pointpillars_detection_config_))
      << ", config_file: " << config_file;

  mem_size_ = pointpillars_detection_config_.mem_size();

  GetDeviceInfo();
  pointpillar_ptr_ = CreateCore();

  if (pointpillar_ptr_ == nullptr) {
    AERROR << "Core has been failed.";
    return false;
  }

  input_raw_.resize(mem_size_);
  pointpillar_ptr_->print();

  cudaStreamCreate(&stream_);
  return true;
}

int loadData(const char* file, void** data, unsigned int* length) {
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open()) {
    std::cout << "Can't open files: " << file << std::endl;
    return -1;
  }

  unsigned int len = 0;
  dataFile.seekg(0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg(0, dataFile.beg);

  char* buffer = new char[len];
  if (buffer == NULL) {
    std::cout << "Can't malloc buffer." << std::endl;
    dataFile.close();
    exit(EXIT_FAILURE);
  }

  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void*)buffer;
  *length = len;
  return 0;
}

void SavePointCloudToBin(const LidarFrame* point_cloud, int points_size,
                         const std::string& file_path) {
  std::ofstream bin_file(file_path, std::ios::out | std::ios::binary);
  if (!bin_file.is_open()) {
    std::cerr << "Failed to open file for writing: " << file_path << std::endl;
    return;
  }

  for (int i = 0; i < points_size; ++i) {
    bin_file.write(
        reinterpret_cast<const char*>(&(point_cloud->raw_cloud->at(i).x)),
        sizeof(float));
    bin_file.write(
        reinterpret_cast<const char*>(&(point_cloud->raw_cloud->at(i).y)),
        sizeof(float));
    bin_file.write(
        reinterpret_cast<const char*>(&(point_cloud->raw_cloud->at(i).z)),
        sizeof(float));
    bin_file.write(reinterpret_cast<const char*>(
                       &(point_cloud->raw_cloud->at(i).intensity)),
                   sizeof(float));
  }

  bin_file.close();
  std::cout << "Point cloud saved to: " << file_path << std::endl;
}

bool PointPillarsDetector::Detect(const LidarDetectorOptions& options,
                                  LidarFrame* frame) {
  // frame->segmented_objects.clear();
  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  auto points_size = frame->raw_cloud->size();
  original_cloud_ = frame->raw_cloud;

  if (points_size <= 0) {
    return false;
  }

  auto copy_points = [&](auto ind) {
    auto& point = frame->raw_cloud->at(ind);
    input_raw_[ind * 4 + 0] = point.x;
    input_raw_[ind * 4 + 1] = point.y;
    input_raw_[ind * 4 + 2] = point.z;
    input_raw_[ind * 4 + 3] = point.intensity;
  };

  for (auto ptx = 0u; ptx < points_size; ++ptx) {
    copy_points(ptx);
  }

  AINFO << "start pointpillar inference points_size : " << points_size;

  auto bboxes = pointpillar_ptr_->forward((float*)input_raw_.data(),
                                          points_size, stream_);
  GetObjects(bboxes, frame);
  AINFO << "pointpillar inference success : " << bboxes.size();

  return true;
}

void PointPillarsDetector::GetObjects(
    const std::vector<pointpillar::lidar::BoundingBox>& bboxes,
    LidarFrame* frame) noexcept {
  const auto num_objects = bboxes.size();

  // auto objects = frame->segmented_objects;
  // objects.clear();

  // base::ObjectPool::Instance().BatchGet(num_objects, &objects);
  AINFO << "pointpillar inference success : " << num_objects;
  // auto orignal_size = frame->raw_cloud->size();
  // objects.resize(num_objects);

  std::vector<std::shared_ptr<base::Object>> objects(num_objects);

  for (size_t i = 0; i < num_objects; i++) {
    std::shared_ptr<base::Object> object(new base::Object);

    object->id = i;

    float yaw = bboxes[i].rt;
    //  yaw += M_PI / 2;
    //  yaw = std::atan2(sinf(yaw), cosf(yaw));
    //  yaw = -yaw;

    // read params of bounding box
    float x = bboxes[i].x;
    float y = bboxes[i].y;
    float z = bboxes[i].z;
    float dx = bboxes[i].w;
    float dy = bboxes[i].l;
    float dz = bboxes[i].h;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;

    object->size[0] = dx;
    object->size[1] = dy;
    object->size[2] = dz;

    object->center[0] = x;
    object->center[1] = y;
    object->center[2] = z;
    object->confidence = bboxes[i].score;

    auto roll = 0.f;
    auto pitch = 0.f;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          base::PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);
          //  base::PointD ptd{point.x , point.y , point.z};
          //  object->polygon.push_back(ptd);
          base::PointD ptd;
          ptd.x = point.x;
          ptd.y = point.y;
          ptd.z = point.z;
          object->lidar_supplement.cloud_world.push_back(ptd);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectSubType(bboxes[i].id);

    // Mapping real label to type
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    // objects->emplace_back(object);

    // *obj = object;
    //  frame->segmented_objects.push_back(object);
    objects[i] = object;
  }

  for (auto begin = objects.begin(); begin != objects.end(); ++begin) {
    if (IsOriginInsideBBox(*begin)) {
      objects.erase(begin);
      break;
    }
  }

  {
    auto* mutex = base::Singleton<std::mutex>::GetInstance();
    std::lock_guard<std::mutex> lock(*mutex);
    frame->segmented_objects.insert(frame->segmented_objects.end(),
                                    objects.begin(), objects.end());
  }

  // for (size_t i = 0; i < num_objects; i++) {
  //   frame->segmented_objects.push_back(objects[i]);
  // }

  AINFO << "*****************pointpillar inference success : "
        << objects.size();
}

base::ObjectSubType PointPillarsDetector::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::PEDESTRIAN;  // 'Pedestrian'
    case 1:
      return base::ObjectSubType::CAR;  // 'Car'
    case 2:
      return base::ObjectSubType::IGV_FULL;  // 'IGV-Full'
    case 3:
      return base::ObjectSubType::TRUCK;  // 'Truck'
    case 4:
      return base::ObjectSubType::TRAILER_EMPTY;  // 'Trailer-Empty'
    case 5:
      return base::ObjectSubType::TRAILER_FULL;  // 'Trailer-Full'
    case 6:
      return base::ObjectSubType::IGV_EMPTY;  // 'IGV-Empty'
    case 7:
      return base::ObjectSubType::CRANE;  // 'Crane'
    case 8:
      return base::ObjectSubType::OTHER_VEHICLE;  // 'OtherVehicle'
    case 9:
      return base::ObjectSubType::CONE;  // 'Cone'
    case 10:
      return base::ObjectSubType::CONTAINER_FORKLIFT;  // 'ContainerForklift'
    case 11:
      return base::ObjectSubType::FORKLIFT;  // 'Forklift'
    case 12:
      return base::ObjectSubType::LORRY;  // 'Lorry'
    case 13:
      return base::ObjectSubType::
          CONSTRUCTION_VEHICLE;  // 'ConstructionVehicle'
    case 14:
      return base::ObjectSubType::WHEELCRANE;  // 'WheelCrane'
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

std::shared_ptr<pointpillar::lidar::Core>
PointPillarsDetector::CreateCore() noexcept {
  auto range_config = pointpillars_detection_config_.range_config();
  auto min_range_x = range_config.min_range().x();
  auto min_range_y = range_config.min_range().y();
  auto min_range_z = range_config.min_range().z();

  auto max_range_x = range_config.max_range().x();
  auto max_range_y = range_config.max_range().y();
  auto max_range_z = range_config.max_range().z();

  auto voxel_x = range_config.voxel_size().x();
  auto voxel_y = range_config.voxel_size().y();
  auto voxel_z = range_config.voxel_size().z();

  auto score_threshold_cfg =
      pointpillars_detection_config_.score_threshold_config();
  auto pointpillar_anchor_cfg =
      pointpillars_detection_config_.pointpillar_anchor_config();
  auto pointpillar_nms_iou_cfg =
      pointpillars_detection_config_.pointpillar_nms_iou_config();

  pointpillar::lidar::VoxelizationParameter vp;
  vp.min_range = nvtype::Float3(min_range_x, min_range_y, min_range_z);
  vp.max_range = nvtype::Float3(max_range_x, max_range_y, max_range_z);
  vp.voxel_size = nvtype::Float3(voxel_x, voxel_y, voxel_z);
  vp.grid_size =
      vp.compute_grid_size(vp.max_range, vp.min_range, vp.voxel_size);

  vp.max_voxels = pointpillars_detection_config_.max_voxels();
  vp.max_points_per_voxel =
      pointpillars_detection_config_.max_points_per_voxel();
  vp.max_points = pointpillars_detection_config_.max_points();
  vp.num_feature = pointpillars_detection_config_.num_feature();
  vp.intensity_threshold = pointpillars_detection_config_.intensity_threshold();

  pointpillar::lidar::PostProcessParameter pp;
  pp.min_range = vp.min_range;
  pp.max_range = vp.max_range;
  pp.feature_size = nvtype::Int2(vp.grid_size.x / 2, vp.grid_size.y / 2);
  pp.num_classes = pointpillars_detection_config_.num_classes();
  for (int i = 0; i < score_threshold_cfg.score_threshold_size(); i++) {
    pp.score_thresh_per_class.emplace_back(
        score_threshold_cfg.score_threshold(i));
  }
  pp.len_per_anchor = pointpillar_anchor_cfg.len_per_anchor();
  pp.num_anchors = pointpillar_anchor_cfg.point_pillar_anchor_size() * 2;
  pp.num_box_values = pointpillars_detection_config_.num_box_values();
  pp.dir_offset = pointpillars_detection_config_.dir_offset();
  for (int i = 0; i < pointpillar_anchor_cfg.point_pillar_anchor_size(); i++) {
    auto& anchor_data = pointpillar_anchor_cfg.point_pillar_anchor(i);
    pp.anchor_vector.insert(
        pp.anchor_vector.end(),
        {anchor_data.length(), anchor_data.width(), anchor_data.height(),
         anchor_data.anchor_first_degree(), anchor_data.length(),
         anchor_data.width(), anchor_data.height(),
         anchor_data.anchor_second_degree()});
    pp.anchor_bottom_heights_vector.emplace_back(
        anchor_data.anchor_bottom_heights());
  }
  for (int i = 0; i < pointpillar_nms_iou_cfg.nms_iou_size(); i++) {
    pp.nms_iou_thresh_per_class.emplace_back(
        pointpillar_nms_iou_cfg.nms_iou(i));
  }

  AINFO << "Range Config Parameters:"
        << "\nMin Range: (" << min_range_x << ", " << min_range_y << ", "
        << min_range_z << ")"
        << "\nMax Range: (" << max_range_x << ", " << max_range_y << ", "
        << max_range_z << ")"
        << "\nVoxel Size: (" << voxel_x << ", " << voxel_y << ", " << voxel_z
        << ")"
        << "\nGrid Size: (" << vp.grid_size.x << ", " << vp.grid_size.y << ", "
        << vp.grid_size.z << ")"
        << "\nFeature Size: (" << pp.feature_size.x << ", " << pp.feature_size.y
        << ")"
        << "\nMax Voxels: " << vp.max_voxels
        << "\nMax Points Per Voxel: " << vp.max_points_per_voxel
        << "\nMax Points: " << vp.max_points
        << "\nNum Feature: " << vp.num_feature;

  pointpillar::lidar::CoreParameter param;
  param.voxelization = vp;

#ifdef __aarch64__
  param.lidar_model = FLAGS_pointpillars_engine_path + "pointpillar_arm.plan";
#else
  param.lidar_model = FLAGS_pointpillars_engine_path + "pointpillar_x86.plan";
#endif

  param.lidar_post = pp;
  return pointpillar::lidar::create_core(param);
}

void PointPillarsDetector::GetBoxCorner(
    std::vector<float>& box_corner, std::vector<float>& box_rectangular,
    std::vector<std::shared_ptr<Object>>* objects) noexcept {
  auto num_objects = objects->size();
  const float quantize = 0.2f;
  const float width_enlarge_value = 0.0f;
  const float length_enlarge_value = 0.0f;

  AERROR << "2 box_corner size: " << box_corner.size();

  for (int i = 0; i < num_objects; ++i) {
    auto object = objects->at(i);
    float x = object->center[0];
    float y = object->center[1];
    float w = object->size[0];
    float l = object->size[1];
    float a = object->theta;

    if (quantize > 0) {
      w = ceil(w / quantize) * quantize;
      l = ceil(l / quantize) * quantize;
    }
    if (width_enlarge_value > 0) {
      w = w + width_enlarge_value;
    }
    if (length_enlarge_value > 0) {
      l = l + length_enlarge_value;
    }

    float cos_a = cos(a);
    float sin_a = sin(a);
    float hw = w * 0.5;
    float hl = l * 0.5;

    float left_up_x = (-hw) * cos_a + (-hl) * sin_a + x;
    float left_up_y = (-hw) * (-sin_a) + (-hl) * cos_a + y;
    float right_up_x = (-hw) * cos_a + (hl)*sin_a + x;
    float right_up_y = (-hw) * (-sin_a) + (hl)*cos_a + y;
    float right_down_x = (hw)*cos_a + (hl)*sin_a + x;
    float right_down_y = (hw) * (-sin_a) + (hl)*cos_a + y;
    float left_down_x = (hw)*cos_a + (-hl) * sin_a + x;
    float left_down_y = (hw) * (-sin_a) + (-hl) * cos_a + y;

    box_corner[i * 8 + 0] = left_up_x;
    box_corner[i * 8 + 1] = left_up_y;
    box_corner[i * 8 + 2] = right_up_x;
    box_corner[i * 8 + 3] = right_up_y;
    box_corner[i * 8 + 4] = right_down_x;
    box_corner[i * 8 + 5] = right_down_y;
    box_corner[i * 8 + 6] = left_down_x;
    box_corner[i * 8 + 7] = left_down_y;

    box_rectangular[i * 4] = std::min(
        std::min(std::min(left_up_x, right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 1] = std::min(
        std::min(std::min(left_up_y, right_up_y), right_down_y), left_down_y);
    box_rectangular[i * 4 + 2] = std::max(
        std::max(std::max(left_up_x, right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 3] = std::max(
        std::max(std::max(left_up_y, right_up_y), right_down_y), left_down_y);
  }
}

void PointPillarsDetector::GetBoxIndices(
    const std::vector<float>& box_corner,
    const std::vector<float>& box_rectangular,
    std::vector<std::shared_ptr<Object>>* objects) noexcept {
  // const int num_output_box_feature = 7;
  const float bottom_enlarge_height = 0.25f;
  const float top_enlarge_height = 0.25f;
  auto num_objects = objects->size();
  for (size_t point_idx = 0; point_idx < original_cloud_->size(); ++point_idx) {
    const auto& point = original_cloud_->at(point_idx);
    // if (model_param_.filter_ground_points() &&
    //     original_cloud_->points_label(point_idx) == static_cast<uint8_t>(
    //         LidarPointLabel::GROUND)) {
    //     continue;
    // }
    float px = point.x;
    float py = point.y;
    float pz = point.z;

    for (int box_idx = 0; box_idx < num_objects; box_idx++) {
      if (px < box_rectangular[box_idx * 4 + 0] ||
          px > box_rectangular[box_idx * 4 + 2]) {
        continue;
      }
      if (py < box_rectangular[box_idx * 4 + 1] ||
          py > box_rectangular[box_idx * 4 + 3]) {
        continue;
      }

      auto object = objects->at(box_idx);

      float z = object->center[2];
      float h = object->size[3];
      if (pz < (z - h / 2 - bottom_enlarge_height) ||
          pz > (z + h / 2 + top_enlarge_height)) {
        continue;
      }

      float x1 = box_corner[box_idx * 8 + 0];
      float x2 = box_corner[box_idx * 8 + 2];
      float x3 = box_corner[box_idx * 8 + 4];
      float x4 = box_corner[box_idx * 8 + 6];
      float y1 = box_corner[box_idx * 8 + 1];
      float y2 = box_corner[box_idx * 8 + 3];
      float y3 = box_corner[box_idx * 8 + 5];
      float y4 = box_corner[box_idx * 8 + 7];

      double angl1 = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
      double angl2 = (px - x2) * (y3 - y2) - (py - y2) * (x3 - x2);
      double angl3 = (px - x3) * (y4 - y3) - (py - y3) * (x4 - x3);
      double angl4 = (px - x4) * (y1 - y4) - (py - y4) * (x1 - x4);

      if ((angl1 <= 0 && angl2 <= 0 && angl3 <= 0 && angl4 <= 0) ||
          (angl1 >= 0 && angl2 >= 0 && angl3 >= 0 && angl4 >= 0)) {
        auto& object = objects->at(box_idx);
        object->lidar_supplement.cloud.push_back(point);
      }
    }
  }
}

PERCEPTION_REGISTER_LIDARDETECTOR(PointPillarsDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century