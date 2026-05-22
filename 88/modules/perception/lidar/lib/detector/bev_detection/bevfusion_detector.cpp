
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
#include <sys/time.h>

#include <cstring>
#include <iomanip>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <yaml-cpp/yaml.h>

#include "bevfusion/lidar_preprocess.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/bev_detection/bev_debug_utils.h"

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

#ifdef __aarch64__
namespace {
bool EnableMappedHostAccess() noexcept {
  cudaDeviceProp prop;
  cudaError_t err = cudaGetDeviceProperties(&prop, 0);
  if (err != cudaSuccess) {
    AERROR << "Failed to query CUDA device properties: "
           << cudaGetErrorString(err);
    return false;
  }

  if (!prop.canMapHostMemory) {
    AINFO << "CUDA device cannot map host memory, fallback to device copy";
    return false;
  }

  err = cudaSetDeviceFlags(cudaDeviceMapHost);
  if (cudaSuccess == err || cudaErrorSetOnActiveProcess == err) {
    if (cudaErrorSetOnActiveProcess == err) {
      cudaGetLastError();
    }
    return true;
  }

  AERROR << "Failed to enable CUDA mapped host memory: "
         << cudaGetErrorString(err);
  return false;
}
}  // namespace
#endif

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

BevFusionDetector::BevFusionDetector() {
#ifdef __aarch64__
  use_mapped_host_points_ = EnableMappedHostAccess();
#endif
  cudaStreamCreate(&stream_);
}

BevFusionDetector::~BevFusionDetector() {
  FreeGpuPreprocessBuffers();
  checkRuntime(cudaStreamDestroy(stream_));
}

void BevFusionDetector::AllocGpuPreprocessBuffers(size_t num_points) {
  if (num_points <= gpu_preprocess_capacity_) return;

  FreeGpuPreprocessBuffers();
  gpu_preprocess_capacity_ = static_cast<size_t>(num_points * 1.2);

  checkRuntime(cudaMalloc(&d_points_float_,
                          gpu_preprocess_capacity_ * 4 * sizeof(float)));
  checkRuntime(
      cudaMalloc(&d_points_half_, gpu_preprocess_capacity_ * 4 * sizeof(half)));

  AINFO << "AllocGpuPreprocessBuffers: capacity=" << gpu_preprocess_capacity_;
}

void BevFusionDetector::FreeGpuPreprocessBuffers() {
  if (d_points_float_) {
    checkRuntime(cudaFree(d_points_float_));
    d_points_float_ = nullptr;
  }
  if (d_points_half_) {
    checkRuntime(cudaFree(d_points_half_));
    d_points_half_ = nullptr;
  }
#ifdef __aarch64__
  UnregisterMappedPoints();
#endif
  gpu_preprocess_capacity_ = 0;
}

#ifdef __aarch64__
bool BevFusionDetector::RegisterMappedPoints(
    const base::PointFCloud& cloud) noexcept {
  const auto& points = cloud.points();
  if (points.empty()) {
    return false;
  }

  const void* host_ptr = points.data();
  const size_t bytes = points.capacity() * sizeof(base::PointF);
  if (registered_points_host_ == host_ptr &&
      registered_points_bytes_ == bytes) {
    return mapped_points_device_ != nullptr;
  }

  UnregisterMappedPoints();

  cudaError_t err = cudaHostRegister(const_cast<void*>(host_ptr), bytes,
                                     cudaHostRegisterMapped);
  if (err != cudaSuccess) {
    AINFO << "cudaHostRegister failed, fallback to device copy: "
          << cudaGetErrorString(err);
    cudaGetLastError();
    return false;
  }

  void* device_ptr = nullptr;
  err = cudaHostGetDevicePointer(&device_ptr, const_cast<void*>(host_ptr), 0);
  if (err != cudaSuccess) {
    AINFO << "cudaHostGetDevicePointer failed, fallback to device copy: "
          << cudaGetErrorString(err);
    cudaHostUnregister(const_cast<void*>(host_ptr));
    cudaGetLastError();
    return false;
  }

  registered_points_host_ = host_ptr;
  registered_points_bytes_ = bytes;
  mapped_points_device_ = static_cast<const float*>(device_ptr);
  return true;
}

void BevFusionDetector::UnregisterMappedPoints() noexcept {
  if (nullptr == registered_points_host_) {
    return;
  }

  checkRuntime(cudaStreamSynchronize(stream_));
  cudaError_t err =
      cudaHostUnregister(const_cast<void*>(registered_points_host_));
  if (err != cudaSuccess) {
    AERROR << "cudaHostUnregister failed: " << cudaGetErrorString(err);
    cudaGetLastError();
  }

  registered_points_host_ = nullptr;
  registered_points_bytes_ = 0;
  mapped_points_device_ = nullptr;
}
#endif

bool BevFusionDetector::Init(const LidarDetectorInitOptions& options) {
  use_camera_ = options.use_camera;
  std::string config_file;
  if (options.cfg_file.empty()) {
    auto* config_manager = lib::ConfigManager::Instance();
    const std::string work_root = config_manager->work_root();

    config_file = GetAbsolutePath(work_root, "conf/perception/lidar");
  } else {
    config_file = options.cfg_file;
  }

  const char* detection_config_name =
      use_camera_ ? "bevfusion_detection_fusion.pb.txt"
                  : "bevfusion_detection.pb.txt";
  auto detection_config_file =
      GetAbsolutePath(config_file, detection_config_name);
  ACHECK(century::cyber::common::GetProtoFromFile(detection_config_file,
                                                  &bev_config_))
      << ", detection_config_file: " << detection_config_file;
  enable_timer_ = bev_config_.enable_timer();

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
  std::string root_path = bev_config_.model_path();
  if (use_camera_) {
    root_path = cyber::common::GetAbsolutePath(root_path, "lidar_camera_fusion");
  }
  auto precision = bev_config_.voxel_param().precision_type();

  bevfusion::camera::NormalizationParameter normalization;
  normalization.image_width =
      bev_config_.norm_param().input_image_size().width();
  normalization.image_height =
      bev_config_.norm_param().input_image_size().height();
  normalization.output_width =
      bev_config_.norm_param().output_image_size().width();
  normalization.output_height =
      bev_config_.norm_param().output_image_size().height();
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
  auto lidar_backbone_path =
      GetAbsolutePath(root_path, "lidar.backbone.xyz.onnx");
  scn.model = lidar_backbone_path;
  scn.order = bevfusion::lidar::CoordinateOrder::XYZ;

  if ("int8" == precision) {
    scn.precision = bevfusion::lidar::Precision::Int8;
  } else {
    scn.precision = bevfusion::lidar::Precision::Float16;
  }

  std::string head_plan_path;
  std::string camera_backbone_path;
  std::string camera_fuser_path;
  std::string camera_vtransform_path;

#ifdef __aarch64__
  head_plan_path = root_path + "/head.bbox.arm.plan";
  camera_backbone_path = root_path + "/camera.backbone.arm.plan";
  camera_fuser_path = root_path + "/fuser.arm.plan";
  camera_vtransform_path = root_path + "/camera.vtransform.arm.plan";
#else
  head_plan_path = root_path + "/head.bbox.x86.plan";
  camera_backbone_path = root_path + "/camera.backbone.x86.plan";
  camera_fuser_path = root_path + "/fuser.x86.plan";
  camera_vtransform_path = root_path + "/camera.vtransform.x86.plan";
#endif

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

  transbbox.model = head_plan_path;

  bevfusion::CoreParameter param;

  param.camera_model = camera_backbone_path;
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = camera_fuser_path;
  param.transbbox = transbbox;
  param.camera_vtransform = camera_vtransform_path;
  param.use_camera = use_camera_;

  bevfusion_ptr_ = bevfusion::create_core(param);
  bevfusion_ptr_->print();
  bevfusion_ptr_->set_timer(enable_timer_);
  bevfusion_ptr_->set_debug(false);

  if (param.use_camera) {
    common::CameraSensorConfig& camera_sensor_config =
        common::CameraSensorConfig::GetInstance();
    if (!camera_sensor_config.IsInitialized()) {
      AERROR << "Camera sensor config is not initialized";
    }
    std::vector<float> camera2lidar_vec;
    std::vector<float> camera_intrinsics_vec;
    std::vector<float> lidar2image_vec;
    std::vector<float> img_aug_matrix_vec;

  for (const auto& camera : camera_sensor_config.GetCameraNames()) {
    common::CameraParms camera_parms;
    if (!camera_sensor_config.GetCameraConfig(camera, camera_parms)) {
      AERROR << "Failed to get camera config for " << camera;
    }

    for (const auto& row : camera_parms.camera2lidar_vec) {
      camera2lidar_vec.insert(camera2lidar_vec.end(), row.begin(), row.end());
    }
    for (const auto& row : camera_parms.camera_intrinsics_4x4_vec) {
      camera_intrinsics_vec.insert(camera_intrinsics_vec.end(), row.begin(),
                                   row.end());
    }
    for (const auto& row : camera_parms.lidar2image_vec) {
      lidar2image_vec.insert(lidar2image_vec.end(), row.begin(), row.end());
    }
    for (const auto& row : camera_parms.img_aug_matrix_vec) {
      img_aug_matrix_vec.insert(img_aug_matrix_vec.end(), row.begin(),
                                row.end());
    }
  }
  std::vector<int32_t> shape = {
      1, static_cast<int32_t>(bev_config_.norm_param().camera_num()), 4, 4};
    auto camera2lidar = nv::Tensor::from_data_reference(
        camera2lidar_vec.data(), shape, nv::DataType::Float32, false);
    auto camera_intrinsics = nv::Tensor::from_data_reference(
        camera_intrinsics_vec.data(), shape, nv::DataType::Float32, false);
    auto lidar2image = nv::Tensor::from_data_reference(
        lidar2image_vec.data(), shape, nv::DataType::Float32, false);
    auto img_aug_matrix = nv::Tensor::from_data_reference(
        img_aug_matrix_vec.data(), shape, nv::DataType::Float32, false);

    camera2lidar.print_structured("camera2lidar", 4, 6);
    camera_intrinsics.print_structured("camera_intrinsics", 4, 6);
    lidar2image.print_structured("lidar2image", 4, 6);
    img_aug_matrix.print_structured("img_aug_matrix", 4, 6);
    bevfusion_ptr_->update(
        camera2lidar.ptr<float>(), camera_intrinsics.ptr<float>(),
        lidar2image.ptr<float>(), img_aug_matrix.ptr<float>(), stream_);
  }

  AINFO << "Start create core, lidar_backbone_path: "
        << lidar_backbone_path << ", root_path: " << root_path
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

  if (points_size <= 0) {
    return false;
  }

  int num_points = 0;
  float preprocess_time_ms = 0.0f;
  if (enable_timer_) {
    cudaEvent_t preprocess_begin = nullptr;
    cudaEvent_t preprocess_end = nullptr;
    checkRuntime(cudaEventCreate(&preprocess_begin));
    checkRuntime(cudaEventCreate(&preprocess_end));
    checkRuntime(cudaEventRecord(preprocess_begin, stream_));
    num_points = PreprocessLidarGpu(frame);
    checkRuntime(cudaEventRecord(preprocess_end, stream_));
    checkRuntime(cudaEventSynchronize(preprocess_end));
    checkRuntime(cudaEventElapsedTime(&preprocess_time_ms, preprocess_begin,
                                      preprocess_end));
    checkRuntime(cudaEventDestroy(preprocess_begin));
    checkRuntime(cudaEventDestroy(preprocess_end));
  } else {
    num_points = PreprocessLidarGpu(frame);
  }

  std::vector<bevfusion::head::transbbox::BoundingBox> bboxes;

  if (!bev_config_.enable_camera()) {
    bboxes = bevfusion_ptr_->forward(
        nullptr, reinterpret_cast<const nvtype::half*>(d_points_half_),
        num_points, stream_, preprocess_time_ms);
  } else {
    unsigned char* camera_data_ptrs[6];
    GetCameraInputTensor(frame, camera_data_ptrs);
    bboxes = bevfusion_ptr_->forward(
        const_cast<const unsigned char**>(camera_data_ptrs),
        reinterpret_cast<const nvtype::half*>(d_points_half_), num_points,
        stream_, preprocess_time_ms);
  }
  
  if (bboxes.empty()) {
    AERROR << "No bounding boxes";
    return false;
  }

  GetObjects(bboxes, frame);

  return true;
}

bool BevFusionDetector::DetectFromDump(const std::string& dump_dir) {
  if (!bevfusion_ptr_) {
    AERROR << "bevfusion_ptr_ is null";
    return false;
  }

  bevfusion_ptr_->set_debug(true);

  BevDebugUtils dbg;
  dbg.SetOutputDir(dump_dir);
  dbg.SetEnabled(true);

  std::vector<int64_t> camera_shape;
  nvtype::half* camera_images =
      dbg.LoadTensor("py/02_camera_images_normed.bin", camera_shape, stream_);
  if (camera_images == nullptr || camera_shape.size() != 4) {
    AERROR << "Failed to load normalized camera images from dump dir: "
           << dump_dir;
    return false;
  }

  std::vector<int64_t> lidar_shape;
  nvtype::half* lidar_points =
      dbg.LoadTensor("py/00_input_points.bin", lidar_shape, stream_);
  if (lidar_points == nullptr || lidar_shape.size() != 2) {
    AERROR << "Failed to load lidar points from dump dir: " << dump_dir;
    checkRuntime(cudaFree(camera_images));
    return false;
  }

  int num_points = static_cast<int>(lidar_shape[0]);
  auto bboxes = bevfusion_ptr_->forward_no_normalize(
      camera_images, lidar_points, num_points, stream_, 0.0f);

  checkRuntime(cudaStreamSynchronize(stream_));
  checkRuntime(cudaFree(camera_images));
  checkRuntime(cudaFree(lidar_points));

  dbg.CompareFinalBBoxes(bboxes);
  bevfusion_ptr_->set_debug(false);
  AINFO << "Dump validation finished, bbox count: " << bboxes.size();
  return true;
}

void BevFusionDetector::GetCameraInputTensor(
    LidarFrame* frame, unsigned char** camera_data_ptrs) noexcept {

  const auto& camera_ready_events = frame->camera_ready_events;

  // Use CUDA events for non-blocking cross-stream synchronization
  // This allows the inference stream to wait for all camera preprocessing
  // without blocking the CPU thread
  for (size_t i = 0; i < camera_ready_events.size(); ++i) {
    if (camera_ready_events[i] != 0) {
      checkRuntime(cudaStreamWaitEvent(
          stream_, reinterpret_cast<cudaEvent_t>(camera_ready_events[i]), 0));
    }
    camera_data_ptrs[i] = frame->camera_data[i];
  }
}

int BevFusionDetector::PreprocessLidarGpu(LidarFrame* frame) noexcept {
  const size_t num_points = frame->raw_cloud->size();
  if (0 == num_points) {
    return 0;
  }

  AllocGpuPreprocessBuffers(num_points);

  const float* src_host =
      reinterpret_cast<const float*>(frame->raw_cloud->points().data());

#ifdef __aarch64__
  if (use_mapped_host_points_ && RegisterMappedPoints(*frame->raw_cloud)) {
    bevfusion::lidar::LaunchConvertPointsToHalf(
        mapped_points_device_, d_points_half_, num_points,
        reinterpret_cast<cudaStream_t>(stream_));
  } else {
    const size_t bytes_float = num_points * 4 * sizeof(float);
    checkRuntime(cudaMemcpyAsync(d_points_float_, src_host, bytes_float,
                                 cudaMemcpyHostToDevice,
                                 reinterpret_cast<cudaStream_t>(stream_)));
    bevfusion::lidar::LaunchConvertPointsToHalf(
        d_points_float_, d_points_half_, num_points,
        reinterpret_cast<cudaStream_t>(stream_));
  }
#else
  const size_t bytes_float = num_points * 4 * sizeof(float);
  checkRuntime(cudaMemcpyAsync(d_points_float_, src_host, bytes_float,
                               cudaMemcpyHostToDevice,
                               reinterpret_cast<cudaStream_t>(stream_)));
  bevfusion::lidar::LaunchConvertPointsToHalf(
      d_points_float_, d_points_half_, num_points,
      reinterpret_cast<cudaStream_t>(stream_));
#endif

  return static_cast<int>(num_points);
}

void BevFusionDetector::GetObjects(
    const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes,
    LidarFrame* frame) noexcept {
  const auto num_objects = bboxes.size();

  std::vector<std::shared_ptr<base::Object>> objects(num_objects);

  for (size_t i = 0; i < num_objects; ++i) {
    std::shared_ptr<base::Object> object(new base::Object);

    object->id = i;

    float yaw = bboxes[i].z_rotation;

    // float x = bboxes[i].position.y;
    // float y = -bboxes[i].position.x;
    float x = bboxes[i].position.x;
    float y = bboxes[i].position.y;
    float z = bboxes[i].position.z + bboxes[i].size.h / 2;
    float dx = bboxes[i].size.w;
    float dy = bboxes[i].size.l;
    float dz = bboxes[i].size.h;

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

    object->lidar_supplement.raw_probs.emplace_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.emplace_back(Name());

    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
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
}

PERCEPTION_REGISTER_LIDARDETECTOR(BevFusionDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century
