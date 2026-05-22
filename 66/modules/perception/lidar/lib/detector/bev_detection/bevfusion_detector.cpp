
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
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion_detector.h"

#include <dlfcn.h>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"

namespace {
constexpr double kWheelCraneLength = 12.79;
constexpr double kWheelCraneWidth = 2.98;
constexpr double kWheelCraneHeight = 4.0;
constexpr double kWheelCraneYaw = 2.99;
}  // namespace

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetAbsolutePath;

static nvtype::Float3 ConvertFloat3Pb2Float3(const Float3& float3_pb) {
  nvtype::Float3 float3;
  float3.x = float3_pb.x();
  float3.y = float3_pb.y();
  float3.z = float3_pb.z();
  return float3;
}

static nvtype::Int3 ConvertInt3Pb2Int3(const Int3& Int3_pb) {
  nvtype::Int3 int3;
  int3.x = Int3_pb.x();
  int3.y = Int3_pb.y();
  int3.z = Int3_pb.z();
  return int3;
}

static double convertYawToLidarFrame(const Eigen::Affine3d& lidar2world_pose,
                                     double obstacle_yaw_world) {
  Eigen::Matrix3d R = lidar2world_pose.rotation();
  double ego_yaw = std::atan2(R(1, 0), R(0, 0));

  double obstacle_yaw_lidar = obstacle_yaw_world - ego_yaw;

  obstacle_yaw_lidar =
      std::atan2(std::sin(obstacle_yaw_lidar), std::cos(obstacle_yaw_lidar));

  return obstacle_yaw_lidar;
}

BevFusionDetector::BevFusionDetector() { cudaStreamCreate(&stream_); }

BevFusionDetector::~BevFusionDetector() {
  checkRuntime(cudaStreamDestroy(stream_));
}

bool BevFusionDetector::Init(const LidarDetectorInitOptions& options) {
  std::string config_file;
  if (options.cfg_file.empty()) {
    auto* config_manager = lib::ConfigManager::Instance();
    const std::string work_root = config_manager->work_root();

    config_file = GetAbsolutePath(work_root, "conf/perception/lidar");
  } else {
    config_file = options.cfg_file;
  }

  auto detection_config_file =
      GetAbsolutePath(config_file, "bevfusion_detection.pb.txt");
  ACHECK(century::cyber::common::GetProtoFromFile(detection_config_file,
                                                  &bev_config_))
      << ", detection_config_file: " << detection_config_file;

  auto subtype_config_file =
      GetAbsolutePath(config_file, "object_subtype_mapping_config.pb.txt");
  ObjectSubTypeMapping object_subtype_mapping_config;
  ACHECK(century::cyber::common::GetProtoFromFile(
      subtype_config_file, &object_subtype_mapping_config))
      << ", subtype_config_file: " << subtype_config_file;

  for (const auto& mapping : object_subtype_mapping_config.subtype_mapping()) {
    object_subtype_mapping_[mapping.in_key()] =
        static_cast<base::ObjectSubType>(mapping.out_key());
  }

  CreateBevFusionCore();

  return true;
}

void BevFusionDetector::CreateBevFusionCore() noexcept {
  auto root_path = bev_config_.model_path();
  auto precision = bev_config_.voxel_param().precision_type();

  bevfusion::camera::NormalizationParameter normalization;
  normalization.image_width =
      bev_config_.norm_param().input_image_size().width();
  normalization.image_height =
      bev_config_.norm_param().input_image_size().height();
  normalization.output_width =
      bev_config_.norm_param().output_image_size().width();
  normalization.output_height =
      bev_config_.norm_param().output_image_size().width();
  normalization.num_camera = bev_config_.norm_param().camera_num();
  normalization.resize_lim = bev_config_.norm_param().resize_limit();
  normalization.interpolation = bevfusion::camera::Interpolation::Bilinear;

  float mean[3] = {0.485, 0.456, 0.406};
  float std[3] = {0.229, 0.224, 0.225};
  normalization.method =
      bevfusion::camera::NormMethod::mean_std(mean, std, 1 / 255.0f, 0.0f);

  auto voxel_param = bev_config_.voxel_param();
  bevfusion::lidar::VoxelizationParameter voxelization;
  voxelization.min_range = ConvertFloat3Pb2Float3(voxel_param.min_range());
  voxelization.max_range = ConvertFloat3Pb2Float3(voxel_param.max_range());
  voxelization.voxel_size = ConvertFloat3Pb2Float3(voxel_param.voxel_size());

  voxelization.grid_size = voxelization.compute_grid_size(
      voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
  voxelization.max_points_per_voxel = voxel_param.max_points_per_voxel();
  voxelization.max_points = voxel_param.max_points();
  voxelization.max_voxels = voxel_param.max_voxels();
  voxelization.num_feature = voxel_param.num_feature();

  bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxelization;
  auto lidar_backbone_path = voxel_param.lidar_backbone_onnx_path();
  scn.model = lidar_backbone_path;
  scn.order = bevfusion::lidar::CoordinateOrder::XYZ;

  if (precision == "int8") {
    scn.precision = bevfusion::lidar::Precision::Int8;
  } else {
    scn.precision = bevfusion::lidar::Precision::Float16;
  }

  auto geometry_param = bev_config_.geometry_param();

  bevfusion::camera::GeometryParameter geometry;
  geometry.xbound = ConvertFloat3Pb2Float3(geometry_param.x_bound());
  geometry.ybound = ConvertFloat3Pb2Float3(geometry_param.y_bound());
  geometry.zbound = ConvertFloat3Pb2Float3(geometry_param.z_bound());
  geometry.dbound = ConvertFloat3Pb2Float3(geometry_param.d_bound());
  geometry.image_width = geometry_param.input_image_size().width();
  geometry.image_height = geometry_param.input_image_size().height();
  geometry.feat_width = geometry_param.feature_size().width();
  geometry.feat_height = geometry_param.feature_size().height();
  geometry.num_camera = geometry_param.camera_num();
  geometry.geometry_dim = ConvertInt3Pb2Int3(geometry_param.dim());

  auto transbbox_param = bev_config_.trans_bbox_param();

  bevfusion::head::transbbox::TransBBoxParameter transbbox;
  transbbox.out_size_factor = transbbox_param.out_size_factor();
  transbbox.pc_range = {transbbox_param.pc_range().x(),
                        transbbox_param.pc_range().y()};
  transbbox.post_center_range_start =
      ConvertFloat3Pb2Float3(transbbox_param.post_center_range_start());
  transbbox.post_center_range_end =
      ConvertFloat3Pb2Float3(transbbox_param.post_center_range_end());
  transbbox.voxel_size = {transbbox_param.voxel_size().x(),
                          transbbox_param.voxel_size().y()};
  transbbox.confidence_threshold = transbbox_param.confidence_threshold();
  transbbox.sorted_bboxes = transbbox_param.sorted_bboxes();

  std::string head_plan_path;

#ifdef __aarch64__
  head_plan_path = root_path + "/head.bbox.arm.plan";
#else
  head_plan_path = root_path + "/head.bbox.x86.plan";
#endif

  transbbox.model = head_plan_path;

  bevfusion::CoreParameter param;

  std::string camera_backbone_path;
  std::string camera_fuser_path;
  std::string camera_vtransform_path;
#ifdef __aarch64__
  head_plan_path = root_path + "/head.bbox.arm.plan";
  camera_fuser_path = root_path + "/fuser.arm.plan";
  camera_vtransform_path = root_path + "/camera.vtransform.arm.plan";
#else
  head_plan_path = root_path + "/head.bbox.x86.plan";
  camera_fuser_path = root_path + "/fuser.x86.plan";
  camera_vtransform_path = root_path + "/camera.vtransform.plan";
#endif

  param.camera_model = head_plan_path;
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = camera_fuser_path;
  param.transbbox = transbbox;
  param.camera_vtransform = camera_vtransform_path;

  bevfusion_ptr_ = bevfusion::create_core(param);
  bevfusion_ptr_->print();
  bevfusion_ptr_->set_timer(true);

  AINFO << "create core, lidar_backbone_path: " << lidar_backbone_path
        << ", root_path: " << root_path
        << ", camera_fuser_path: " << camera_fuser_path
        << ", head_plan_path: " << head_plan_path;
}

bool BevFusionDetector::Detect(const LidarDetectorOptions& options,
                               LidarFrame* frame) {
  if (!bevfusion_ptr_) {
    AERROR << "bevfusion_ptr_ is null";
    return false;
  }

  auto points_size = frame->raw_cloud->size();
  auto original_cloud = frame->raw_cloud;

  if (points_size <= 0) {
    return false;
  }

  AINFO << "1 Start to preprocess raw point cloud data: " << points_size;
  auto lidar_tensor = GetLidarInputTensor(frame);

  std::vector<bevfusion::head::transbbox::BoundingBox> bboxes;
  if (!bev_config_.enable_camera()) {
    AINFO << "2 Start to lidar forward";

    bboxes = bevfusion_ptr_->forward(nullptr, lidar_tensor.ptr<nvtype::half>(),
                                     lidar_tensor.size(0), stream_);
  }

  if (bboxes.empty()) {
    return false;
  }
  AINFO << "3 Start to postprocess: " << bboxes.size();

  GetObjects(bboxes, frame);

  return true;
}

nv::Tensor BevFusionDetector::GetLidarInputTensor(LidarFrame* frame) noexcept {
  nv::Tensor points;
  int points_size = frame->raw_cloud->size();

  std::vector<half> input_raw;
  input_raw.reserve(points_size * 4);

  auto copy_points = [&](auto ind) {
    auto& point = frame->raw_cloud->at(ind);
    input_raw.push_back(static_cast<half>(point.y) * -1);
    input_raw.push_back(static_cast<half>(point.x));
    input_raw.push_back(static_cast<half>(point.z));
    input_raw.push_back(static_cast<half>(point.intensity));
  };

  for (auto ptx = 0u; ptx < points_size; ++ptx) {
    copy_points(ptx);
  }

  std::vector<int32_t> shape = {points_size, 4};
  points = nv::Tensor::create(shape, nv::DataType::Float16, false);
  memcpy(points.ptr(), input_raw.data(), input_raw.size() * sizeof(half));

  return points;
}

void BevFusionDetector::GetObjects(
    const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes,
    LidarFrame* frame) noexcept {
  const auto num_objects = bboxes.size();

  AINFO << "*****************pointpillar inference success : " << num_objects;

  std::vector<std::shared_ptr<base::Object>> objects(num_objects);

  for (size_t i = 0; i < num_objects; i++) {
    std::shared_ptr<base::Object> object(new base::Object);

    object->id = i;

    float yaw = bboxes[i].z_rotation - M_PI / 2;

    // read params of bounding box
    float x = bboxes[i].position.y;
    float y = -bboxes[i].position.x;
    float z = bboxes[i].position.z + bboxes[i].size.h / 2;
    float dx = bboxes[i].size.w;
    float dy = bboxes[i].size.l;
    float dz = bboxes[i].size.h;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;

    auto sub_type = object_subtype_mapping_[bboxes[i].id];
    object->sub_type = sub_type;

    // Mapping real label to type
    object->type = base::kSubType2TypeMap.at(object->sub_type);

    if (base::ObjectType::WHEELCRANE == object->type) {
      object->theta =
          convertYawToLidarFrame(frame->lidar2world_pose, kWheelCraneYaw);
      yaw = object->theta;

      dx = kWheelCraneLength;
      dy = kWheelCraneWidth;
      dz = kWheelCraneHeight;
      object->size[0] = dx;
      object->size[1] = dy;
      object->size[2] = dz;

      object->center[0] = x;
      object->center[1] = y;
      object->center[2] = z;
      object->direction[0] = cosf(object->theta);
      object->direction[1] = sinf(object->theta);
    } else {
      object->size[0] = dx;
      object->size[1] = dy;
      object->size[2] = dz;

      object->center[0] = x;
      object->center[1] = y;
      object->center[2] = z;
    }

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

    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    objects[i] = object;
  }

  {
    auto* mutex = base::Singleton<std::mutex>::GetInstance();
    std::lock_guard<std::mutex> lock(*mutex);
    frame->segmented_objects.insert(frame->segmented_objects.end(),
                                    objects.begin(), objects.end());
  }

  AINFO << "*****************pointpillar inference success : "
        << objects.size();
}

PERCEPTION_REGISTER_LIDARDETECTOR(BevFusionDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century
