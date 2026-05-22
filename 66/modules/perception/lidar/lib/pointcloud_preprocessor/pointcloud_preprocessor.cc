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
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

#include <cfloat>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/pcl_util.h"

using century::cyber::common::GetAbsolutePath;

namespace century {
namespace perception {
namespace lidar {

namespace {
constexpr float kPointInfThreshold = 1e2f;
constexpr float kDistanceThreshold = 900.0f;
constexpr float kMinDistanceThreshold = 0.04f;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

#pragma pack(push, 1)
struct PointPacked {
 private:
  float x_;
  float y_;
  float z_;
  uint16_t intensity_;
  uint16_t ring_;
  double timestamp_;

 public:
  // Default constructor
  PointPacked() = default;

  // Parameterized constructor
  PointPacked(float x, float y, float z, uint16_t intensity, uint16_t ring,
              double timestamp)
      : x_(x),
        y_(y),
        z_(z),
        intensity_(intensity),
        ring_(ring),
        timestamp_(timestamp) {}

  // ---- Accessors (Getters) ----
  inline float x() const noexcept { return x_; }
  inline float y() const noexcept { return y_; }
  inline float z() const noexcept { return z_; }
  inline uint16_t intensity() const noexcept { return intensity_; }
  inline uint16_t ring() const noexcept { return ring_; }
  inline double timestamp() const noexcept { return timestamp_; }

  inline void set_position(const float x, const float y,
                           const float z) noexcept {
    x_ = x;
    y_ = y;
    z_ = z;
  }

  // Compute squared distance to avoid expensive sqrt operations
  inline float distance_squared_2d() const noexcept {
    return x_ * x_ + y_ * y_;
  }

  inline float distance_squared_3d() const noexcept {
    return x_ * x_ + y_ * y_ + z_ * z_;
  }
};
#pragma pack(pop)

void PointCloudPreprocessor::SavePointcloudPcd(
    const std::shared_ptr<century::drivers::PointXYZIRTCloud const>&
        message) noexcept {
  if (!message || message->point_size() == 0) {
    return;
  }

  pcl::PointCloud<lidar::PointXYZIR>::Ptr pcl_cloud(
      new pcl::PointCloud<lidar::PointXYZIR>);
  pcl_cloud->reserve(message->point_size());

  for (int i = 0; i < message->point_size(); ++i) {
    const century::drivers::PointXYZIRT& pt = message->point(i);
    if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
      continue;
    }
    lidar::PointXYZIR pcl_pt;
    pcl_pt.x = pt.x();
    pcl_pt.y = pt.y();
    pcl_pt.z = pt.z();
    pcl_pt.ring = pt.ring();
    pcl_pt.intensity = static_cast<float>(pt.intensity());
    pcl_cloud->push_back(pcl_pt);
  }

  std::ostringstream oss;
  double timestamp = message->header().timestamp_sec();
  oss << topic_name_ << "_" << std::fixed << std::setprecision(6) << timestamp
      << ".pcd";
  std::string filename = oss.str();
  std::string pcd_file = "/century/data/pcd_output/" + filename;

  try {
    // if (topic_name_ == "lidar_bp_front_left") {
    AINFO << "Saving velodyne64 pointcloud to PCD file: " << pcd_file
          << ", point size: " << pcl_cloud->size();
    pcl::io::savePCDFileBinaryCompressed(pcd_file, *pcl_cloud);
    // }

  } catch (const std::exception& e) {
    std::cerr << "Failed to save PCD file: " << e.what() << std::endl;
  }
}

bool PointCloudPreprocessor::Init(
    const PointCloudPreprocessorInitOptions& options) {
  topic_name_ = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));

  AERROR << "root_path: " << root_path << ", work_root: " << work_root;
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, options.sensor_name);
  config_file = GetAbsolutePath(config_file, "pointcloud_preprocessor.conf");
  // PointCloudPreprocessorConfig config;
  ACHECK(century::cyber::common::GetProtoFromFile(config_file, &config_));
  use_det_ = config_.use_det();
  max_intensity_threshold_ = config_.max_intensity_threshold();
  filter_naninf_points_ = config_.filter_naninf_points();
  filter_nearby_box_points_ = config_.filter_nearby_box_points();
  box_forward_x_ = config_.box_forward_x();
  box_backward_x_ = config_.box_backward_x();
  box_forward_y_ = config_.box_forward_y();
  box_backward_y_ = config_.box_backward_y();

  range_around_forward_x_ = config_.range_around_forward_x();
  range_around_forward_y_ = config_.range_around_forward_y();
  range_around_backward_x_ = config_.range_around_backward_x();
  range_around_backward_y_ = config_.range_around_backward_y();
  original_intensity_threshold_ = config_.original_intensity_threshold();
  dust_intensity_threshold_ = config_.dust_intensity_threshold();
  is_intensity_filter_ = config_.is_intensity_filter();

  filter_high_z_points_ = config_.filter_high_z_points();
  z_threshold_ = config_.z_threshold();
  height_threshold_ = config_.height_threshold();

  ground_detector_ = BaseGroundDetectorRegisterer::GetInstanceByName(
      config_.ground_detector());

  AERROR << "config_.ground_detector(): " << config_.ground_detector();
  CHECK_NOTNULL(ground_detector_);
  GroundDetectorInitOptions ground_detector_init_options;
  ground_detector_init_options.sensor_name = options.sensor_name;
  ACHECK(ground_detector_->Init(ground_detector_init_options))
      << "Failed to init ground detector.";

  // init roi filter
  ego_filter_ = BaseROIFilterRegisterer::GetInstanceByName("EgoServiceFilter");
  CHECK_NOTNULL(ego_filter_);
  ROIFilterInitOptions roi_filter_init_options;
  roi_filter_init_options.ego_service_filter_configs.reserve(
      config_.range_filter_config().size());

  for (const auto& range : config_.range_filter_config()) {
    EgoServiceFilterConfig filter_config;
    filter_config.x = range.x();
    filter_config.y = range.y();
    filter_config.size = range.size();
    filter_config.enable_filter = range.enable_filter();
    roi_filter_init_options.ego_service_filter_configs.emplace_back(
        filter_config);
  }

  ACHECK(ego_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";

  AINFO << options.sensor_name << " init finished";

  return true;
}

void PointCloudPreprocessor::InitializeClouds(
    LidarFrame* frame,
    const std::shared_ptr<PointCloudInType const>& message) noexcept {
  // Initialize clouds if they are nullptr
  if (frame->cloud == nullptr) {
    frame->cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->vehicle_cloud == nullptr) {
    frame->vehicle_cloud = base::PointXYZIRTFCloudPool::Instance().Get();
  }
  if (frame->raw_cloud == nullptr) {
    frame->raw_cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->ego_cloud == nullptr) {
    frame->ego_cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }

  // Set timestamps
  frame->cloud->set_timestamp(message->header().timestamp_sec());
  frame->raw_cloud->set_timestamp(message->header().timestamp_sec());
  frame->ego_cloud->set_timestamp(message->header().timestamp_sec());
  frame->world_cloud->set_timestamp(message->header().timestamp_sec());

  if (config_.dump_pcd()) {
    // SavePointcloudPcd(message);
  }
}

void PointCloudPreprocessor::ReserveCloudMemory(LidarFrame* frame,
                                                size_t point_size) noexcept {
  frame->vehicle_cloud->reserve(point_size);
  frame->raw_cloud->reserve(point_size);
}

inline bool PointCloudPreprocessor::FilterEgoPoints(
    const base::PointXYZIRTF& point) noexcept {
  if (filter_nearby_box_points_ && point.x < box_forward_x_ &&
      point.x > box_backward_x_ && point.y < box_forward_y_ &&
      point.y > box_backward_y_) {
    return true;
  }
  return false;
}

inline bool PointCloudPreprocessor::FilterByIntensity(
    const base::PointXYZIRTF& point) noexcept {
  if (!is_intensity_filter_) {
    return false;
  }

  auto distance = point.x * point.x + point.y * point.y;
  if (point.intensity <= original_intensity_threshold_ &&
      distance < kDistanceThreshold) {
    return true;
  }

  if (point.intensity > max_intensity_threshold_) {
    return true;
  }

  return false;
}

inline bool PointCloudPreprocessor::FilterByRoi(
    const base::PointXYZIRTF& point) noexcept {
  auto roi_forward_x = config_.roi_forward_x();
  auto roi_backward_x = config_.roi_backward_x();
  auto roi_forward_y = config_.roi_forward_y();
  auto roi_backward_y = config_.roi_backward_y();

  if (point.x < roi_forward_x && point.x > roi_backward_x &&
      point.y < roi_forward_y && point.y > roi_backward_y) {
    return true;
  }
  return false;
}

inline bool PointCloudPreprocessor::FilterByRange(
    const base::PointXYZIRTF& point) noexcept {
  auto range_forward_x = config_.range_forward_x();
  auto range_backward_x = config_.range_backward_x();
  auto range_forward_y = config_.range_forward_y();
  auto range_backward_y = config_.range_backward_y();
  auto range_z_max = config_.range_z_max();
  auto range_z_min = config_.range_z_min();

  if (point.x < range_forward_x && point.x > range_backward_x &&
      point.y < range_forward_y && point.y > range_backward_y &&
      point.z < range_z_max && point.z > range_z_min) {
    return false;
  }

  return true;
}

void PointCloudPreprocessor::ProcessPoints(
    const std::shared_ptr<PointCloudInType const>& message, size_t point_size,
    LidarFrame* frame) noexcept {
  // Process each point in the point cloud

#ifdef POINTCLOUD_PACKED_DEFINITION
  std::vector<PointPacked> ppd(point_size);
  const std::string& raw_data = message->data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
#endif

  bool is_filter = false;
  for (size_t i = 0; i < point_size; ++i) {
#ifdef POINTCLOUD_PACKED_DEFINITION
    const auto& pt = ppd[i];
#else
    const auto& pt = message->point(i);
#endif
    base::PointXYZIRTF point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z() + config_.z_offset();
    point.intensity = static_cast<float>(pt.intensity());
    point.ring = pt.ring();
    point.timestamp = pt.timestamp();

    if (ShouldFilterPoint(point)) {
      continue;
    }

    // Process point data
    TransformPoint(point);

    if (FilterEgoPoints(point)) {
      continue;
    }

    if (FilterByIntensity(point)) {
      is_filter = true;
      continue;
    }

    if (FilterByRange(point)) {
      continue;
    }

    if (FilterByRoi(point)) {
      continue;
    }

    // Add the processed point to the cloud
    AddPointToCloud(frame, point, pt.timestamp(), i);
  }

  if (!is_filter) {
    AINFO << "No points were filtered based on intensity.";
  }

  // Finalize the cloud after processing
  FinalizeClouds(frame);
}

bool PointCloudPreprocessor::ShouldFilterPoint(
    const base::PointXYZIRTF& pt) noexcept {
  // Filter out invalid or unwanted points
  if (filter_naninf_points_ &&
      (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
    return true;
  }
  if (fabs(pt.x) > kPointInfThreshold || fabs(pt.y) > kPointInfThreshold ||
      fabs(pt.z) > kPointInfThreshold) {
    return true;
  }

  if (filter_high_z_points_ && pt.z > z_threshold_) {
    return true;
  }

  auto distance = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
  if (distance <= kMinDistanceThreshold) {
    return true;
  }

  return false;
}

inline base::PointXYZIRTF PointCloudPreprocessor::TransformPoint(
    base::PointXYZIRTF& pt) noexcept {
  Eigen::Vector3d vec3d_lidar(pt.x, pt.y, pt.z);
  // Eigen::Vector3d vec3d_imu = options_.sensor2novatel_extrinsics *
  // vec3d_lidar;
  Eigen::Vector3d vec3d_imu = options_.sensor2vehicle_extrinsics * vec3d_lidar;

  pt.x = vec3d_imu(0);
  pt.y = vec3d_imu(1);
  pt.z = vec3d_imu(2);

  return pt;
}

void PointCloudPreprocessor::AddPointToCloud(LidarFrame* frame,
                                             const base::PointXYZIRTF& point,
                                             double timestamp,
                                             size_t index) noexcept {
  // Add point to the vehicle cloud
  frame->vehicle_cloud->push_back(point, timestamp,
                                  std::numeric_limits<float>::max(), index, 0);

  // Add point to the raw cloud
  base::PointF detect_point;
  detect_point.x = point.x;
  detect_point.y = point.y;
  detect_point.z = point.z;
  detect_point.intensity = point.intensity;
  frame->raw_cloud->push_back(detect_point, timestamp,
                              std::numeric_limits<float>::max(), index, 0);
}

void PointCloudPreprocessor::FinalizeClouds(LidarFrame* frame) noexcept {
  // Resize the vehicle and raw cloud to their actual size
  frame->vehicle_cloud->resize(frame->vehicle_cloud->size());
  frame->raw_cloud->resize(frame->raw_cloud->size());

  // Copy the raw cloud to the final cloud
  *frame->cloud = *frame->raw_cloud;

  if (config_.ground_detector() == "SpatioTemporalGroundDetector" ||
      config_.ground_detector() == "SpatioTemporalGroundDetectorV2") {
    // Transform the cloud
    TransformCloud(frame->raw_cloud, Eigen::Affine3d::Identity(),
                   frame->world_cloud);
  }
}

bool PointCloudPreprocessor::Preprocess(
    const PointCloudPreprocessorOptions& options,
    const std::shared_ptr<PointCloudInType const>& message, LidarFrame* frame) {
  if (frame == nullptr || message == nullptr || message->point_size() == 0) {
    return false;
  }

  PERCEPTION_PERF_BLOCK_START();
  InitializeClouds(frame, message);

  options_ = options;
  size_t point_size = message->point_size();

  if (0 == point_size) {
    AERROR << "Received pointcloud is empty.";
    return true;
  }

  // Reserve space for the point clouds
  ReserveCloudMemory(frame, point_size);

  // Process points and filter
  ProcessPoints(message, point_size, frame);

  GroundDetectorOptions ground_options;

  ground_detector_->Detect(ground_options, frame);

  FilterPointCloud(frame);

  if (!use_det_) {
    frame->raw_cloud->resize(1);
  }

  ROIFilterOptions filter_options;
  ego_filter_->Filter(filter_options, frame);

  PERCEPTION_PERF_BLOCK_END("PointCloud Processing");

  return true;
}

bool PointCloudPreprocessor::TransformCloud(
    const base::PointFCloudPtr& local_cloud, const Eigen::Affine3d& pose,
    base::PointDCloudPtr world_cloud) const {
  if (local_cloud == nullptr) {
    return false;
  }
  world_cloud->clear();
  world_cloud->reserve(local_cloud->size());
  for (size_t i = 0; i < local_cloud->size(); ++i) {
    auto& pt = local_cloud->at(i);
    Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
    trans_point = pose * trans_point;
    base::PointD world_point;
    world_point.x = trans_point(0);
    world_point.y = trans_point(1);
    world_point.z = trans_point(2);
    world_point.intensity = pt.intensity;
    world_cloud->push_back(world_point, local_cloud->points_timestamp(i),
                           std::numeric_limits<float>::max(),
                           local_cloud->points_beam_id()[i], 0);
  }
  return true;
}

void PointCloudPreprocessor::FilterPointCloud(LidarFrame* frame) noexcept {
  const auto& incloud = frame->vehicle_cloud;
  frame->cloud->clear();
  auto& non_ground_indices = frame->non_ground_indices.indices;
  frame->cloud->reserve(non_ground_indices.size());
  origin_points_indices_.clear();
  origin_points_indices_.reserve(non_ground_indices.size());
  frame->ego_cloud->reserve(non_ground_indices.size());

  for (size_t i = 0; i < non_ground_indices.size(); i++) {
    int pt_index = non_ground_indices[i];

    base::PointF pt;
    pt.x = incloud->at(pt_index).x;
    pt.y = incloud->at(pt_index).y;
    pt.z = incloud->at(pt_index).z;
    pt.intensity = incloud->at(pt_index).intensity;
    auto timestamp = incloud->points_timestamp(pt_index);

    auto distance = pt.x * pt.x + pt.y * pt.y;

    if (pt.intensity < dust_intensity_threshold_ &&
        distance < kDistanceThreshold) {
      continue;
    }

    frame->cloud->push_back(pt, static_cast<double>(timestamp),
                            std::numeric_limits<float>::max(), i, 0);
    origin_points_indices_.push_back(pt_index);

    if (pt.x < range_around_forward_x_ && pt.x > range_around_backward_x_ &&
        pt.y < range_around_forward_y_ && pt.y > range_around_backward_y_) {
      frame->ego_cloud->push_back(pt);
    }
  }

  frame->ego_cloud->resize(frame->ego_cloud->size());
  frame->cloud->resize(frame->cloud->size());
  frame->non_ground_indices.indices = origin_points_indices_;
  frame->roi_indices.indices = origin_points_indices_;
}

PERCEPTION_REGISTER_POINTCLOUDPREPROCESSOR(PointCloudPreprocessor);

}  // namespace lidar
}  // namespace perception
}  // namespace century
