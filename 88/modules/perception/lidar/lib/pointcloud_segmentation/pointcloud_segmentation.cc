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
#include "modules/perception/lidar/lib/pointcloud_segmentation/pointcloud_segmentation.h"

#include <cfloat>
#include <iomanip>

#include "cyber/common/file.h"
#include "modules/perception/lidar/lib/pointcloud_segmentation/lidar_ransac_plane.h"

namespace century {
namespace perception {
namespace lidar {

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

using century::cyber::common::GetAbsolutePath;

bool PointcloudSegmentation::Init(
    const PointCloudPreprocessorInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, options.sensor_name);
  config_file = GetAbsolutePath(config_file, "pointcloud_segmentation.conf");

  ACHECK(century::cyber::common::GetProtoFromFile(config_file, &config_));

  auto filter_naninf_points_ = config_.filter_naninf_points();
  auto filter_nearby_box_points_ = config_.filter_nearby_box_points();
  auto box_forward_x_ = config_.box_forward_x();
  auto box_backward_x_ = config_.box_backward_x();
  auto box_forward_y_ = config_.box_forward_y();
  auto box_backward_y_ = config_.box_backward_y();

  // print config_
  AINFO << "*********PointcloudSegmentation::Init*********: "
        << options.sensor_name << ", config_file: " << config_file;
  AINFO << "filter_naninf_points_: " << filter_naninf_points_;
  AINFO << "filter_nearby_box_points_: " << filter_nearby_box_points_;
  AINFO << "box_forward_x_: " << box_forward_x_;
  AINFO << "box_backward_x_: " << box_backward_x_;
  AINFO << "box_forward_y_: " << box_forward_y_;
  AINFO << "box_backward_y_: " << box_backward_y_;
  AINFO << "*********PointcloudSegmentation::Init*********: "
        << options.sensor_name << ", config_file: " << config_file;
  return true;
}

bool PointcloudSegmentation::Preprocess(
    const PointCloudPreprocessorOptions& options,
    const std::shared_ptr<century::drivers::PointXYZIRTCloud const>& message,
    LidarFrame* frame) {
  AINFO << "*********PointcloudSegmentation::Preprocess*********: "
        << std::fixed << std::setprecision(10)
        << message->header().timestamp_sec();

  PointCloudTypePtr outcloud = std::make_shared<PointXYZIRTFCloud>();
  // FilterPointCloud(message->cloud(), outcloud);
  // auto flag = TransformCloud(message, frame);
  // if (!flag) {
  //   AERROR << "Failed to transform point cloud.";
  //   return false;
  // }
  auto size = message->point().size();
  AERROR << "Size : " << size;
  // frame->cloud->reserve(message->point().size());
  // auto count = frame->vehicle_cloud->size();
  // for (size_t i = 0; i < count; i++)
  // {
  //   /* code */
  // }

  return true;
}

bool PointcloudSegmentation::TransformCloud(
    const std::shared_ptr<century::drivers::PointXYZIRTCloud const>&
        local_cloud,
    LidarFrame* frame) {
  return true;
}

void PointcloudSegmentation::FilterPointCloud(
    const PointCloudTypePtr& incloud, PointCloudTypePtr& outcloud) noexcept {
  IndicesPtr ground_indices_(new pcl::Indices());
  FindGroundCandidates(outcloud, ground_indices_, 0.8);

  RansacPlane ranscaPlane;
  Eigen::Vector4f vec;
  auto flag = ranscaPlane.DoRansacPlaneNoAVX(outcloud, *ground_indices_, vec);
  if (!flag && ranscaPlane.m_inliersNumMax > ground_indices_->size() * 0.3) {
    // Eigen::Affine3f rectifiedPos = Eigen::Affine3f::Identity();
  } else {
  }

  FilterAndBuild(outcloud, out_cloud_, projection_dis_, indices_image_);
  RemoveGroundPoints(out_cloud_, out_cloud_);
  FilterNoise();

  std::vector<int> pc_indices;
  PointCloudTypePtr cloud_height_filtered(new PointXYZIRTFCloud());
  *cloud_height_filtered = *out_cloud_;
  GetPointCloudIndices(cloud_height_filtered, pc_indices);
  *outcloud = *cloud_height_filtered;
}

void PointcloudSegmentation::FindGroundCandidates(
    const PointCloudTypePtr& inCloud, IndicesPtr& outIndices,
    double cell_size) noexcept {
  const double range = 40;
  const int gridNum = static_cast<int>(std::ceil(range / cell_size) * 2);
  std::vector<std::vector<int>> gndImg(gridNum, std::vector<int>(gridNum, -1));
  const int inSize = inCloud->size();

  int ncount = 0;

  for (auto i = 0; i < inSize; ++i) {
    auto& pt = (*inCloud)[i];
    if (pt.z > 0.5 || pt.z < -0.5) {
      continue;
    }
    int x_coor = static_cast<int>(std::floor((pt.x + range) / cell_size));
    int y_coor = static_cast<int>(std::floor((pt.y + range) / cell_size));

    if (x_coor < 0 || x_coor >= gridNum || y_coor < 0 || y_coor >= gridNum) {
      continue;
    }

    int& idx = gndImg[y_coor][x_coor];
    ncount++;

    if (idx == -1 || (*inCloud)[idx].z > pt.z) {
      idx = i;
    }
  }

  FilterClosePoints(gndImg, gridNum * gridNum, outIndices);
}

void PointcloudSegmentation::FilterClosePoints(
    const std::vector<std::vector<int>>& indices, const int size,
    IndicesPtr& outIndices) noexcept {
  outIndices->resize(size);
  int count = 0;
  for (auto& i : indices) {
    for (auto& idx : i) {
      if (idx != -1) {
        outIndices->at(count) = idx;
        ++count;
      }
    }
  }
  outIndices->resize(count);
}

void PointcloudSegmentation::FilterAndBuild(
    const PointCloudTypePtr& inCloud, PointCloudTypePtr& outCloud,
    std::vector<float>& projectionDis, std::vector<int>& indicesImg) noexcept {
  if (inCloud == nullptr || inCloud->empty()) {
    return;
  }

  const int size = inCloud->size();
  const int img_col = std::floor(360 / 0.2);
  const int img_row = 32;
  const int max_id = img_row * img_col;
  indicesImg.assign(img_row * img_col, -1);
  outCloud->resize(size);
  projectionDis.resize(size);

  auto gt = [](double x, double y) { return x > y + DBL_EPSILON; };
  int count = 0;
  int count_lower_than_p3 = 0;
  int count_lower_than_n5 = 0;
  for (uint32_t i = 0; i < size; ++i) {
    auto& pt = inCloud->at(i);

    if (gt(pt.x, config_.range_forward_x()) ||
        gt(config_.range_backward_x(), pt.x) ||
        gt(pt.y, config_.range_forward_y()) ||
        gt(config_.range_backward_y(), pt.y) ||
        gt(pt.z, config_.range_z_max()) || gt(config_.range_z_min(), pt.z)) {
      continue;
    }

    if (gt(config_.roi_forward_x(), pt.x) &&
        gt(pt.x, config_.roi_backward_x()) &&
        gt(config_.roi_forward_y(), pt.y) &&
        gt(pt.y, config_.roi_backward_y())) {
      continue;
    }

    if (pt.z < 0.3) {
      count_lower_than_p3++;
    }
    if (pt.z < -0.3) {
      count_lower_than_n5++;
    }

    outCloud->at(count) = pt;

    projectionDis.at(count) = sqrt(pt.x * pt.x + pt.y * pt.y);

    float yaw = atan2(pt.y, pt.x) * RAD_TO_DEG + 180 + DBL_EPSILON;
    if (yaw < 0 || yaw > 360 || !std::isfinite(yaw)) {
      std::cout << "ERROR!Yaw is not appropriate! x: " << pt.x << " y: " << pt.y
                << " yaw: " << yaw << std::endl;
      continue;
    }
    int col = std::floor(yaw / 0.2);
    if (col == img_col) {
      col = 0;
    }
    if (pt.ring * img_col + col >= max_id) {
      continue;
    }
    indicesImg.at(pt.ring * img_col + col) = count;
    ++count;
  }
  outCloud->resize(count);
  projectionDis.resize(count);
}

bool PointcloudSegmentation::RemoveGroundPoints(
    const PointCloudTypePtr& inCloud, PointCloudTypePtr& outCloud) noexcept {
  return true;
}

bool PointcloudSegmentation::GetPointCloudIndices(
    PointCloudTypePtr& inCloud, std::vector<int>& indices) noexcept {
  return true;
}

void PointcloudSegmentation::GetObstaclePointCloud(
    double tr_x, double tr_y, const PointCloudTypePtr& inCloud,
    const std::vector<int>& indices, PointCloudTypePtr outCloud) noexcept {}

bool PointcloudSegmentation::FilterNoise() noexcept { return true; }

PERCEPTION_REGISTER_POINTCLOUDPREPROCESSOR(PointcloudSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace century
