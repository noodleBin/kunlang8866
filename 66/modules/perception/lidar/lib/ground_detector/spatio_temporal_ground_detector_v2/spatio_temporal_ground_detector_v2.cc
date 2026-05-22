/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector_v2/spatio_temporal_ground_detector_v2.h"

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector_v2/proto/spatio_temporal_ground_detector_config_v2.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/i_lib/geometry/i_plane.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_point_label.h"

namespace century {
namespace perception {
namespace lidar {

constexpr float kVehicleFrontX = 3.0f;
constexpr float kVehicleBackX = -3.0f;
constexpr float kVehicleRightY = 0.0f;
constexpr float kVehicleLeftY = 8.0f;
constexpr float kRangeTolerance = 0.1f;
constexpr float kGroundZRadius = 8.0f;
constexpr float kGroundZThreshold = 0.1f;
constexpr unsigned int kNrPointsElement = 3;
constexpr float kGroundConfidence = 1.0f;
constexpr size_t kGroundZAverageFrame = 5;

using century::cyber::common::GetProtoFromFile;

bool SpatioTemporalGroundDetectorV2::Init(
    const GroundDetectorInitOptions& options) {
  const lib::ModelConfig* model_config = nullptr;
  auto config_manager = lib::ConfigManager::Instance();
  ACHECK(config_manager->GetModelConfig("SpatioTemporalGroundDetectorV2",
                                        &model_config))
      << "Failed to get model config: SpatioTemporalGroundDetector";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = century::cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = century::cyber::common::GetAbsolutePath(
      config_file, "spatio_temporal_ground_detector_v2.conf");

  SpatioTemporalGroundDetectorConfigV2 config_params;
  ACHECK(GetProtoFromFile(config_file, &config_params))
      << "Failed to parse SpatioTemporalGroundDetectorConfigV2 config file.";
  ground_thres_ = config_params.ground_thres();
  use_roi_ = config_params.use_roi();
  use_semantic_ground_ = config_params.use_semantic_ground();
  single_ground_detect_ = config_params.single_ground_detect();
  use_ground_service_ = config_params.use_ground_service();
  near_range_dist_ = config_params.near_range_dist();
  near_range_ground_thres_ = config_params.near_range_ground_thres();
  middle_range_dist_ = config_params.middle_range_dist();
  middle_range_ground_thres_ = config_params.middle_range_ground_thres();
  parsing_height_buffer_ = config_params.parsing_height_buffer();
  debug_output_ = config_params.debug_output();

  param_ = new common::PlaneFitGroundDetectorParam;
  param_->roi_region_rad_x = config_params.roi_rad_x();
  param_->roi_region_rad_y = config_params.roi_rad_y();
  param_->roi_region_rad_z = config_params.roi_rad_z();
  param_->nr_grids_coarse = config_params.small_grid_size();
  param_->nr_grids_fine = config_params.big_grid_size();
  grid_size_ = config_params.small_grid_size();

  param_->roi_near_rad = config_params.near_range();
  param_->sample_region_z_lower = config_params.near_z_min();
  ori_sample_z_lower_ = config_params.near_z_min();
  param_->sample_region_z_upper = config_params.near_z_max();
  ori_sample_z_upper_ = config_params.near_z_max();
  param_->planefit_filter_threshold = config_params.z_compare_thres();
  param_->planefit_orien_threshold = config_params.planefit_orien_threshold();
  param_->planefit_dist_threshold_near =
      config_params.planefit_dist_thres_near();
  param_->planefit_dist_threshold_far = config_params.planefit_dist_thres_far();
  param_->candidate_filter_threshold = config_params.smooth_z_thres();
  param_->nr_inliers_min_threshold = config_params.inliers_min_threshold();
  param_->nr_smooth_iter = config_params.nr_smooth_iter();
  param_->use_math_optimize = config_params.use_math_optimize();
  param_->debug_output = config_params.debug_output();
  param_->single_frame_detect = config_params.single_ground_detect();

  pfdetector_ = new common::PlaneFitGroundDetector(*param_);
  pfdetector_->Init();

  point_attribute_.resize(default_point_size_);
  point_indices_temp_.resize(default_point_size_);
  data_.resize(default_point_size_ * kNrPointsElement);
  ground_height_signed_.resize(default_point_size_);

  ground_service_content_.Init(
      config_params.roi_rad_x(), config_params.roi_rad_y(),
      config_params.small_grid_size(), config_params.small_grid_size());
  return true;
}

bool SpatioTemporalGroundDetectorV2::ValidateInput(LidarFrame* frame) {
  if (nullptr == frame) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (nullptr == frame->cloud.get() || nullptr == frame->world_cloud.get()) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->empty()) {
    AERROR << "Input cloud none points.";
    return false;
  }
  if (frame->world_cloud->empty()) {
    AERROR << "Input world_cloud none points.";
    return false;
  }
  return true;
}

void SpatioTemporalGroundDetectorV2::UpdateCloudCenter(LidarFrame* frame) {
  cloud_center_(0) = frame->lidar2world_pose(0, 3);
  cloud_center_(1) = frame->lidar2world_pose(1, 3);
  cloud_center_(2) = frame->lidar2world_pose(2, 3);
}

size_t SpatioTemporalGroundDetectorV2::GetNumPoints(LidarFrame* frame) {
  if (frame->roi_indices.indices.empty()) {
    use_roi_ = false;
  }
  if (use_roi_) {
    return frame->roi_indices.indices.size();
  }
  return frame->world_cloud->size();
}

void SpatioTemporalGroundDetectorV2::ReallocateBuffers(size_t num_points) {
  if (num_points > default_point_size_) {
    default_point_size_ = num_points * 2;
    point_attribute_.resize(default_point_size_);
    point_indices_temp_.resize(default_point_size_);
    data_.resize(default_point_size_ * kNrPointsElement);
    ground_height_signed_.resize(default_point_size_);
  }
}

void SpatioTemporalGroundDetectorV2::CopyPointDataWithRoi(
    LidarFrame* frame, size_t num_points, unsigned int* data_id,
    unsigned int* valid_point_num) {
  for (size_t i = 0; num_points > i; ++i) {
    int index = frame->roi_indices.indices[i];
    if (use_semantic_ground_ && static_cast<PointSemanticLabel>(
                                    frame->cloud->points_semantic_label(index) &
                                    15) != PointSemanticLabel::GROUND) {
      point_attribute_[i] = std::make_pair(index, -1);
      continue;
    }
    point_attribute_[i] =
        std::make_pair(index, static_cast<int>(*valid_point_num));
    point_indices_temp_[(*valid_point_num)++] = index;
    if (single_ground_detect_) {
      const auto& pt = frame->cloud->at(index);
      data_[(*data_id)++] = static_cast<float>(pt.x);
      data_[(*data_id)++] = static_cast<float>(pt.y);
      data_[(*data_id)++] = static_cast<float>(pt.z);
    } else {
      const auto& pt = frame->world_cloud->at(index);
      data_[(*data_id)++] = static_cast<float>(pt.x - cloud_center_(0));
      data_[(*data_id)++] = static_cast<float>(pt.y - cloud_center_(1));
      data_[(*data_id)++] = static_cast<float>(pt.z - cloud_center_(2));
    }
  }
}

void SpatioTemporalGroundDetectorV2::CopyPointDataWithoutRoi(
    LidarFrame* frame, size_t num_points, unsigned int* data_id,
    unsigned int* valid_point_num) {
  for (size_t i = 0; num_points > i; ++i) {
    if (use_semantic_ground_ &&
        static_cast<PointSemanticLabel>(frame->cloud->points_semantic_label(i) &
                                        15) != PointSemanticLabel::GROUND) {
      point_attribute_[i] = std::make_pair(i, -1);
      continue;
    }
    point_attribute_[i] = std::make_pair(i, static_cast<int>(*valid_point_num));
    point_indices_temp_[(*valid_point_num)++] = static_cast<int>(i);
    if (single_ground_detect_) {
      const auto& pt = frame->cloud->at(i);
      data_[(*data_id)++] = static_cast<float>(pt.x);
      data_[(*data_id)++] = static_cast<float>(pt.y);
      data_[(*data_id)++] = static_cast<float>(pt.z);
    } else {
      const auto& pt = frame->world_cloud->at(i);
      data_[(*data_id)++] = static_cast<float>(pt.x - cloud_center_(0));
      data_[(*data_id)++] = static_cast<float>(pt.y - cloud_center_(1));
      data_[(*data_id)++] = static_cast<float>(pt.z - cloud_center_(2));
    }
  }
}

bool SpatioTemporalGroundDetectorV2::PreparePointData(
    LidarFrame* frame, size_t num_points, unsigned int* data_id,
    unsigned int* valid_point_num) {
  frame->non_ground_indices.indices.clear();
  ReallocateBuffers(num_points);

  if (use_roi_) {
    CopyPointDataWithRoi(frame, num_points, data_id, valid_point_num);
  } else {
    CopyPointDataWithoutRoi(frame, num_points, data_id, valid_point_num);
  }

  return true;
}

bool SpatioTemporalGroundDetectorV2::RunGroundDetector(
    LidarFrame* frame, unsigned int valid_point_num) {
  pfdetector_->ResetParams(ori_sample_z_lower_, ori_sample_z_upper_);
  if (use_semantic_ground_) {
    pfdetector_->UpdateParams(frame->parsing_ground_height,
                              parsing_height_buffer_, frame->timestamp);
  }

  if (!pfdetector_->Detect(data_.data(), ground_height_signed_.data(),
                           valid_point_num, kNrPointsElement)) {
    ADEBUG << "failed to call ground detector!";
    base::PointIndices& non_ground_indices = frame->non_ground_indices;
    non_ground_indices.indices.insert(
        non_ground_indices.indices.end(), point_indices_temp_.begin(),
        point_indices_temp_.begin() + valid_point_num);
    return false;
  }
  return true;
}

float SpatioTemporalGroundDetectorV2::GetPointHeight(
    LidarFrame* frame, size_t pc_index, float pc[3], int count,
    const std::function<bool(int, int)>& valid_index_func) {
  float z_dis = 0.0f;

  if (-1 == count) {
    int cur_row, cur_col = 0;
    std::vector<std::pair<int, int>> neighbors;
    pfdetector_->Pc2Voxel(pc[0], pc[1], pc[2], &cur_row, &cur_col);
    neighbors.emplace_back(cur_row, cur_col);
    neighbors.emplace_back(cur_row - 1, cur_col);
    neighbors.emplace_back(cur_row + 1, cur_col);
    neighbors.emplace_back(cur_row, cur_col - 1);
    neighbors.emplace_back(cur_row, cur_col + 1);
    float min_z = std::numeric_limits<float>::max();
    for (size_t k = 0; k < neighbors.size(); ++k) {
      int row = neighbors[k].first;
      int col = neighbors[k].second;
      if (!valid_index_func(row, col)) {
        continue;
      }
      const common::GroundPlaneLiDAR* plane =
          pfdetector_->GetGroundPlane(row, col);
      if (plane->IsValid()) {
        z_dis = common::IPlaneToPointSignedDistanceWUnitNorm(plane->params, pc);
        float abs_z_dis = std::fabs(z_dis);
        if (abs_z_dis < min_z) {
          min_z = abs_z_dis;
        }
      }
    }
    z_dis = min_z;
  } else {
    z_dis = ground_height_signed_.data()[count];
  }

  return z_dis;
}

float SpatioTemporalGroundDetectorV2::ComputeAdaptiveThreshold(
    const Eigen::Vector3d& pc_novatel) {
  float threshold = ground_thres_;

  bool in_near_y_range =
      pc_novatel(1) > kVehicleRightY && pc_novatel(1) < near_range_dist_;
  bool in_near_x_range = pc_novatel(0) > kVehicleBackX - kRangeTolerance &&
                         pc_novatel(0) < kVehicleFrontX + kRangeTolerance;
  if (in_near_y_range && in_near_x_range) {
    threshold = near_range_ground_thres_;
  }

  bool in_middle_y_range =
      pc_novatel(1) >= near_range_dist_ && pc_novatel(1) < middle_range_dist_;
  if (in_middle_y_range && in_near_x_range) {
    threshold = middle_range_ground_thres_;
  }

  return threshold;
}

void SpatioTemporalGroundDetectorV2::ClassifyAndUpdatePoints(
    LidarFrame* frame, size_t num_points, size_t* ground_z_value_count,
    float* ground_z_value,
    const std::function<bool(int, int)>& valid_index_func) {
  base::PointIndices& non_ground_indices = frame->non_ground_indices;

  for (size_t i = 0; num_points > i; ++i) {
    size_t pc_index = point_attribute_[i].first;
    int count = point_attribute_[i].second;

    float pc[3];
    if (single_ground_detect_) {
      const auto& pt = frame->cloud->at(pc_index);
      pc[0] = pt.x;
      pc[1] = pt.y;
      pc[2] = pt.z;
    } else {
      const auto& pt = frame->world_cloud->at(pc_index);
      pc[0] = pt.x - cloud_center_(0);
      pc[1] = pt.y - cloud_center_(1);
      pc[2] = pt.z - cloud_center_(2);
    }

    float z_dis = GetPointHeight(frame, pc_index, pc, count, valid_index_func);
    frame->cloud->mutable_points_height()->at(pc_index) = z_dis;
    frame->world_cloud->mutable_points_height()->at(pc_index) = z_dis;

    const auto& ppp = frame->cloud->at(pc_index);
    Eigen::Vector3d pc_novatel =
        frame->lidar2world_pose * Eigen::Vector3d(ppp.x, ppp.y, ppp.z);
    float threshold = ComputeAdaptiveThreshold(pc_novatel);

    if (z_dis > threshold) {
      non_ground_indices.indices.push_back(static_cast<int>(i));
    } else {
      frame->cloud->mutable_points_label()->at(pc_index) =
          static_cast<uint8_t>(LidarPointLabel::GROUND);
      frame->world_cloud->mutable_points_label()->at(pc_index) =
          static_cast<uint8_t>(LidarPointLabel::GROUND);
      float xy_dist = std::sqrt(ppp.x * ppp.x + ppp.y * ppp.y);
      if (xy_dist < kGroundZRadius && std::fabs(z_dis) < kGroundZThreshold) {
        ++(*ground_z_value_count);
        *ground_z_value += ppp.z;
      }
    }
  }
}

void SpatioTemporalGroundDetectorV2::UpdateGroundHeight(
    LidarFrame* frame, size_t ground_z_value_count, float ground_z_value) {
  if (origin_ground_z_array_.size() >= kGroundZAverageFrame) {
    origin_ground_z_array_.pop_front();
  }

  float ori_height_per_frame =
      ground_z_value / static_cast<float>(ground_z_value_count);
  origin_ground_z_array_.push_back(ori_height_per_frame);

  float ori_height_sum = 0.0f;
  for (auto value : origin_ground_z_array_) {
    ori_height_sum += value;
  }
  frame->original_ground_z =
      ori_height_sum / static_cast<float>(origin_ground_z_array_.size());

  AINFO << "This frame " << std::to_string(frame->timestamp)
        << " origin ground height is " << frame->original_ground_z;
}

void SpatioTemporalGroundDetectorV2::OutputDebugInfo(LidarFrame* frame) {
  std::ofstream out1;
  out1.open("semantic/" + std::to_string(frame->timestamp) + ".txt");
  if (out1.is_open()) {
    for (size_t i = 0; i < frame->cloud->size(); ++i) {
      int semantic_index = static_cast<int>(static_cast<PointSemanticLabel>(
          frame->cloud->points_semantic_label(i) & 15));
      int label_index = static_cast<uint8_t>(frame->cloud->points_label(i));
      if (single_ground_detect_) {
        const auto& pt = frame->cloud->at(i);
        out1 << pt.x << ", " << pt.y << ", " << pt.z << ", " << pt.intensity
             << ", " << semantic_index << ", " << label_index << std::endl;
      } else {
        const auto& pt = frame->world_cloud->at(i);
        out1 << pt.x - cloud_center_(0) << ", " << pt.y - cloud_center_(1)
             << ", " << pt.z - cloud_center_(2) << ", " << pt.intensity << ", "
             << semantic_index << ", " << label_index << std::endl;
      }
    }
  }
  out1.close();
}

void SpatioTemporalGroundDetectorV2::UpdateGroundService() {
  auto ground_service = SceneManager::Instance().Service("GroundService");
  if (nullptr == ground_service) {
    AINFO << "Failed to find ground service and cannot update.";
    return;
  }

  ground_service_content_.grid_center_ = cloud_center_;
  ground_service_content_.grid_.Reset();
  GroundNode* node_ptr = ground_service_content_.grid_.DataPtr();
  unsigned int rows = pfdetector_->GetGridDimY();
  unsigned int cols = pfdetector_->GetGridDimX();

  for (unsigned int r = 0; r < rows; ++r) {
    for (unsigned int c = 0; c < cols; ++c) {
      const common::GroundPlaneLiDAR* plane = pfdetector_->GetGroundPlane(r, c);
      if (plane->IsValid()) {
        unsigned int index = r * cols + c;
        GroundNode* node = node_ptr + index;
        node->params(0) = plane->params[0];
        node->params(1) = plane->params[1];
        node->params(2) = plane->params[2];
        node->params(3) = plane->params[3];
        node->confidence = kGroundConfidence;
      }
    }
  }
  ground_service->UpdateServiceContent(ground_service_content_);
}

bool SpatioTemporalGroundDetectorV2::Detect(
    const GroundDetectorOptions& options, LidarFrame* frame) {
  if (!ValidateInput(frame)) {
    return false;
  }

  UpdateCloudCenter(frame);
  size_t num_points = GetNumPoints(frame);

  unsigned int data_id = 0;
  unsigned int valid_point_num = 0;
  PreparePointData(frame, num_points, &data_id, &valid_point_num);

  AINFO << "spatial temporal seg: use roi " << use_roi_ << " roi points "
        << num_points << " and input of ground detector: " << valid_point_num;

  if (!RunGroundDetector(frame, valid_point_num)) {
    return false;
  }

  auto valid_index = [this](int row, int col) {
    return row >= 0 && row < static_cast<int>(grid_size_) && col >= 0 &&
           col < static_cast<int>(grid_size_);
  };

  size_t ground_z_value_count = 0;
  float ground_z_value = 0.0f;
  ClassifyAndUpdatePoints(frame, num_points, &ground_z_value_count,
                          &ground_z_value, valid_index);

  AINFO << "succeed to call ground detector with non ground points "
        << frame->non_ground_indices.indices.size();

  if (0 < ground_z_value_count) {
    UpdateGroundHeight(frame, ground_z_value_count, ground_z_value);
  }

  if (debug_output_) {
    OutputDebugInfo(frame);
  }

  if (use_ground_service_) {
    UpdateGroundService();
  }

  return true;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SpatioTemporalGroundDetectorV2);

}  // namespace lidar
}  // namespace perception
}  // namespace century
