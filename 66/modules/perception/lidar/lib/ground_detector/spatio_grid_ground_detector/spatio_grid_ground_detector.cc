
#include "modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector/spatio_grid_ground_detector.h"

#include <numeric>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "cyber/common/file.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_point_label.h"

namespace std {
template <>
struct hash<std::pair<int, int>> {
  size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};
}  // namespace std

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetProtoFromFile;

bool SpatioGridGroundDetector::Init(const GroundDetectorInitOptions& options) {
  const lib::ModelConfig* model_config = nullptr;
  auto config_manager = lib::ConfigManager::Instance();
  ACHECK(
      config_manager->GetModelConfig("SpatioGridGroundDetector", &model_config))
      << "Failed to get model config: SpatioGridGroundDetector";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = century::cyber::common::GetAbsolutePath(work_root, root_path);
  config_file =
      century::cyber::common::GetAbsolutePath(config_file, options.sensor_name);
  config_file = century::cyber::common::GetAbsolutePath(
      config_file, "spatio_grid_ground_detector.conf");

  // get config params
  SpatioGridGroundDetectorConfig config_params;
  ACHECK(GetProtoFromFile(config_file, &config_params))
      << "Failed to parse SpatioGridGroundDetectorConfig config file.";

  config_params_ = config_params;
  range_around_forward_x_ = config_params.range_around_forward_x();
  range_around_forward_y_ = config_params.range_around_forward_y();
  range_around_backward_x_ = config_params.range_around_backward_x();
  range_around_backward_y_ = config_params.range_around_backward_y();
  dust_intensity_threshold_ = config_params.dust_intensity_threshold();
  port_type_ = static_cast<PortType>(config_params.port_index());

  return true;
}

bool SpatioGridGroundDetector::Detect(const GroundDetectorOptions& options,
                                      LidarFrame* frame) {
  FilterPointCloud(frame);
  return true;
}

bool SpatioGridGroundDetector::RemoveGroundPointsWithoutDownSampling(
    const PointCloudTypePtr inCloud, std::vector<int>& non_ground_indices) {
  if (!inCloud || inCloud->empty()) {
    AERROR << "Input point cloud is empty.";
    return false;
  }

  non_ground_indices.clear();

  const float grid_size = config_params_.grid_size();
  const float x_min = config_params_.x_min();
  const float x_max = config_params_.x_max();
  const float y_min = config_params_.y_min();
  const float y_max = config_params_.y_max();
  const float height_threshold = config_params_.height_threshold();
  const float min_z_threshold = config_params_.min_z_threshold();
  const float min_exception_z_value = config_params_.min_exception_z_value();
  bool enable_around_filter = config_params_.enable_around_filter();

  const float filter_min_x = config_params_.filter_min_x();
  const float filter_max_x = config_params_.filter_max_x();
  const float filter_min_y = config_params_.filter_min_y();
  const float filter_max_y = config_params_.filter_max_y();

  int x_bins = static_cast<int>((x_max - x_min) / grid_size);
  int y_bins = static_cast<int>((y_max - y_min) / grid_size);

  using GridKey = std::pair<int, int>;
  std::unordered_map<GridKey, float, std::hash<GridKey>> grid_min_z;
  std::unordered_map<GridKey, std::vector<int>, std::hash<GridKey>> grid_points;

  for (size_t i = 0; i < inCloud->size(); ++i) {
    const auto& point = inCloud->at(i);

    if (point.x < x_min || point.x > x_max || point.y < y_min ||
        point.y > y_max) {
      continue;
    }
    
    if (point.z < -guass_ground_height_) {
      continue;
    }

    int x_idx = static_cast<int>((point.x - x_min) / grid_size);
    int y_idx = static_cast<int>((point.y - y_min) / grid_size);

    if (x_idx < 0 || x_idx >= x_bins || y_idx < 0 || y_idx >= y_bins) {
      continue;
    }

    GridKey key = {x_idx, y_idx};

    if (grid_min_z.find(key) == grid_min_z.end()) {
      grid_min_z[key] = std::numeric_limits<float>::max();
    }
    grid_min_z[key] = std::min(grid_min_z[key], point.z);

    grid_points[key].push_back(static_cast<int>(i));
  }

  for (const auto& grid : grid_points) {
    const GridKey& key = grid.first;
    const std::vector<int>& indices = grid.second;
    float min_z = grid_min_z[key];
    // if (min_z < min_exception_z_value) {
    //   min_z = min_exception_z_value;
    // }

    // (point.x > -9.5 && point.x < -5.5 && point.y > -3.5 &&
    //  point.y < 1.0)

    for (int idx : indices) {
      const auto& point = inCloud->at(idx);
      if (point.x < x_min || point.x > x_max || point.y < y_min ||
          point.y > y_max) {
        continue;
      }
  
      if (point.z < -guass_ground_height_) {
        continue;
      }
      if (point.z >= min_z_threshold) {
        non_ground_indices.push_back(idx);
      } else {
        if (enable_around_filter) {
          if (point.x > filter_min_x && point.x < filter_max_x &&
              point.y > filter_min_y && point.y < filter_max_y) {
            if (min_z < min_exception_z_value) {
              min_z = min_exception_z_value;
            }
          }
        } else {
          if (point.x > filter_min_x && point.x < filter_max_x &&
              point.y > filter_min_y && point.y < filter_max_y) {
            if (min_z < guass_ground_height_) {
              min_z = guass_ground_height_;
            }
          }
        }

        if (port_type_ == PortType::DONGJIAZHEN_PORT) {
          min_z = -guass_ground_height_;
        }

        if (point.z > min_z + height_threshold) {
          non_ground_indices.push_back(idx);
        }
      }
    }

    // TODO: float obstacles
    // if (min_z >= 0.15) {
    //   min_z = 0.15;
    // }

    // for (int idx : indices) {
    //   const auto& point = inCloud->at(idx);
    //   if (point.z >= min_z_threshold) {
    //     non_ground_indices.push_back(idx);
    //   } else if (point.z > min_z + height_threshold) {
    //     non_ground_indices.push_back(idx);
    //   }
    // }
  }

  return true;
}

void SpatioGridGroundDetector::FilterPointCloud(LidarFrame* frame) noexcept {
  IndicesPtr ground_indices_(new pcl::Indices());
  local2origin_indices_.clear();
  origin_points_indices_.clear();

  auto incloud = frame->raw_cloud;
  PointCloudTypePtr outcloud = base::PointFCloudPool::Instance().Get();
  // outcloud->reserve(incloud->size());
  *outcloud = *incloud;

  constexpr double cell_size = 0.4;
  FindGroundCandidates(incloud, ground_indices_, cell_size);
  RansacPlane ranscaPlane;
  Eigen::Vector4f vec;
  auto flag = ranscaPlane.DoRansacPlaneNoAVX(incloud, *ground_indices_, vec);

  if (!flag && ranscaPlane.m_inliersNumMax > ground_indices_->size() * 0.3) {
    Eigen::Affine3f rectifiedPos = Eigen::Affine3f::Identity();
    CalVecToTransform(vec, rectifiedPos);

    for (size_t i = 0; i < outcloud->size(); i++) {
      auto& pt = outcloud->at(i);
      Eigen::Vector3f pt3f(pt.x, pt.y, pt.z);
      pt3f = rectifiedPos * pt3f;
      pt.x = pt3f(0);
      pt.y = pt3f(1);
      pt.z = pt3f(2);
    }
    AINFO << "RansacPlane success ";
  } else {
    AINFO << "RansacPlane failed: ";
  }

  std::vector<int>& non_ground_indices = frame->non_ground_indices.indices;

  PERCEPTION_PERF_BLOCK_START();

  if (!RemoveGroundPointsWithoutDownSampling(outcloud, non_ground_indices)) {
    AERROR << "Failed to remove ground points without downsampling.";
    return;
  }

  PERCEPTION_PERF_BLOCK_END("RemoveGroundPointsWithoutDownsampling");
}

void SpatioGridGroundDetector::FindGroundCandidates(
    const PointCloudTypePtr& inCloud, IndicesPtr& outIndices,
    double cell_size) noexcept {
  const double range = 40;
  const int gridNum = static_cast<int>(std::ceil(range / cell_size) * 2);
  std::vector<std::vector<int>> gndImg(gridNum, std::vector<int>(gridNum, -1));
  const int inSize = inCloud->size();

  int ncount = 0;

  constexpr float max_z_threshold = 0.5f;
  constexpr float min_z_threshold = -0.5f;

  for (auto i = 0; i < inSize; ++i) {
    auto& pt = (*inCloud)[i];
    if (pt.z > max_z_threshold || pt.z < min_z_threshold) {
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

void SpatioGridGroundDetector::FilterClosePoints(
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

void SpatioGridGroundDetector::CalVecToTransform(
    const Eigen::Vector4f& vec, Eigen::Affine3f& pos) noexcept {
  Eigen::Vector3f vec3f(vec(0), vec(1), vec(2));
  float roll = atan2f(vec3f(1), vec3f(2));
  Eigen::AngleAxisf rollV = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  Eigen::Vector3f trsVec = rollV * vec3f;
  float pitch = -atan2f(trsVec(0), trsVec(2));
  pos.rotate(rollV * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));

  Eigen::Vector3f normal = vec3f.normalized();
  float d = vec(3);
  float distance = d / normal.norm();
  // Eigen::Vector3f translation = distance * normal;
  // pos.translate(translation);

  guass_ground_height_ = distance;
  // AERROR << "distance: " << distance;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SpatioGridGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century