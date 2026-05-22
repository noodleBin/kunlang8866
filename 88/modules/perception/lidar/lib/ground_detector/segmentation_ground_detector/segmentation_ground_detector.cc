#include "modules/perception/lidar/lib/ground_detector/segmentation_ground_detector/segmentation_ground_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "Eigen/Dense"

#include "modules/perception/lidar/lib/ground_detector/segmentation_ground_detector/proto/segmentation_ground_detector_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_point_label.h"

namespace century {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

namespace {
// Knuth multiplicative hash constant (2^32 / phi)
constexpr size_t kHashMultiplierX = 2654435761ULL;
// MurmurHash3 mix constant
constexpr size_t kHashMultiplierY = 2246822519ULL;
// boost::hash_combine golden-ratio seed
constexpr size_t kHashCombineSeed = 0x9e3779b9ULL;
}  // namespace

size_t SegmentationGroundDetector::CellKeyHash::operator()(
    const CellKey& key) const {
  size_t seed = static_cast<size_t>(key.x) * kHashMultiplierX;
  seed ^= static_cast<size_t>(key.y) * kHashMultiplierY + kHashCombineSeed +
          (seed << 6U) + (seed >> 2U);
  return seed;
}

bool SegmentationGroundDetector::Init(
    const GroundDetectorInitOptions& options) {
  auto* config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));

  std::string config_file = GetAbsolutePath(work_root, root_path);
  config_file =
      GetAbsolutePath(config_file, "segmentation_ground_detector.conf");

  SegmentationGroundDetectorConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config))
      << "Failed to parse segmentation ground detector config: " << config_file;

  near_range_ = config.near_range();
  near_grid_size_ = config.near_grid_size();
  far_grid_size_ = config.far_grid_size();
  weak_obstacle_threshold_ = config.weak_obstacle_threshold();
  strong_obstacle_threshold_ = config.strong_obstacle_threshold();
  far_obstacle_threshold_ = config.far_obstacle_threshold();
  neighbor_cell_radius_ = config.neighbor_cell_radius();
  plane_min_cells_ = config.plane_min_cells();
  plane_max_residual_ = config.plane_max_residual();
  local_gradient_threshold_ = config.local_gradient_threshold();
  gradient_adaptive_scale_ = config.gradient_adaptive_scale();
  roughness_adaptive_scale_ = config.roughness_adaptive_scale();
  adaptive_threshold_cap_ = config.adaptive_threshold_cap();
  weak_cluster_min_cells_ = config.weak_cluster_min_cells();
  weak_cluster_min_peak_ = config.weak_cluster_min_peak();
  x_min_ = config.x_min();
  x_max_ = config.x_max();
  y_min_ = config.y_min();
  y_max_ = config.y_max();
  z_min_ = config.z_min();
  z_max_ = config.z_max();
  max_ground_z_offset_ = config.max_ground_z_offset();
  roughness_gradient_ratio_ = config.roughness_gradient_ratio();
  enable_debug_log_ = config.enable_debug_log();

  ACHECK(near_grid_size_ > 0.0f);
  ACHECK(far_grid_size_ > 0.0f);
  ACHECK(weak_obstacle_threshold_ > 0.0f);
  ACHECK(strong_obstacle_threshold_ >= weak_obstacle_threshold_);
  ACHECK(far_obstacle_threshold_ > 0.0f);
  ACHECK(neighbor_cell_radius_ >= 0);
  ACHECK(plane_min_cells_ >= 3);
  ACHECK(gradient_adaptive_scale_ >= 0.0f);
  ACHECK(roughness_adaptive_scale_ >= 0.0f);
  ACHECK(adaptive_threshold_cap_ >= 0.0f);
  ACHECK(weak_cluster_min_cells_ >= 1);
  ACHECK(weak_cluster_min_peak_ >= weak_obstacle_threshold_);
  ACHECK(max_ground_z_offset_ > 0.0f);
  ACHECK(roughness_gradient_ratio_ >= 0.0f);
  return true;
}

bool SegmentationGroundDetector::BuildCellMaps(LidarFrame* frame,
                                               CellMap* near_cells,
                                               CellMap* far_cells) const {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(frame->raw_cloud);
  CellKey cell_key;
  for (size_t i = 0; i < frame->raw_cloud->size(); ++i) {
    const auto& point = (*frame->raw_cloud)[i];
    if (point.x < x_min_ || point.x > x_max_ || point.y < y_min_ ||
        point.y > y_max_ || point.z < z_min_ || point.z > z_max_) {
      continue;
    }
    const float radius = std::sqrt(point.x * point.x + point.y * point.y);
    const bool is_near = radius <= near_range_;
    const float grid_size = is_near ? near_grid_size_ : far_grid_size_;
    cell_key.x = static_cast<int>(std::floor((point.x - x_min_) / grid_size));
    cell_key.y = static_cast<int>(std::floor((point.y - y_min_) / grid_size));
    auto* cells = is_near ? near_cells : far_cells;
    auto& cell = (*cells)[cell_key];
    if (cell.indices.empty()) {
      cell.min_z = point.z;
      cell.center_x =
          x_min_ + (static_cast<float>(cell_key.x) + 0.5f) * grid_size;
      cell.center_y =
          y_min_ + (static_cast<float>(cell_key.y) + 0.5f) * grid_size;
    } else {
      cell.min_z = std::min(cell.min_z, point.z);
    }
    cell.indices.emplace_back(static_cast<int>(i));
    cell.sum_z += point.z;
    cell.sum_sq_z += point.z * point.z;
  }
  return true;
}

bool SegmentationGroundDetector::FitPlane(
    const std::vector<Eigen::Vector3f>& samples, float query_x, float query_y,
    float* ground_z) const {
  if (samples.size() < static_cast<size_t>(plane_min_cells_)) {
    return false;
  }
  Eigen::Matrix3f AtA = Eigen::Matrix3f::Zero();
  Eigen::Vector3f Atb = Eigen::Vector3f::Zero();
  for (const auto& sample : samples) {
    AtA(0, 0) += sample(0) * sample(0);
    AtA(0, 1) += sample(0) * sample(1);
    AtA(0, 2) += sample(0);
    AtA(1, 1) += sample(1) * sample(1);
    AtA(1, 2) += sample(1);
    AtA(2, 2) += 1.0f;
    Atb(0) += sample(0) * sample(2);
    Atb(1) += sample(1) * sample(2);
    Atb(2) += sample(2);
  }
  AtA(1, 0) = AtA(0, 1);
  AtA(2, 0) = AtA(0, 2);
  AtA(2, 1) = AtA(1, 2);
  const Eigen::Vector3f plane_coeff = AtA.ldlt().solve(Atb);
  float residual = 0.0f;
  for (const auto& sample : samples) {
    const float predicted = plane_coeff(0) * sample(0) +
                            plane_coeff(1) * sample(1) + plane_coeff(2);
    residual = std::max(residual, std::fabs(predicted - sample(2)));
  }
  if (residual > plane_max_residual_) {
    return false;
  }
  *ground_z =
      plane_coeff(0) * query_x + plane_coeff(1) * query_y + plane_coeff(2);
  return true;
}

void SegmentationGroundDetector::EstimateGround(CellMap* cells) const {
  std::vector<Eigen::Vector3f> samples;
  CellKey neighbor_key;
  for (auto& cell_entry : *cells) {
    auto& key = cell_entry.first;
    auto& cell = cell_entry.second;
    samples.clear();
    float min_neighbor_z = cell.min_z;
    float max_neighbor_z = cell.min_z;
    float smooth_ground_sum = 0.0f;
    int smooth_ground_count = 0;
    for (int delta_x = -neighbor_cell_radius_; delta_x <= neighbor_cell_radius_;
         ++delta_x) {
      for (int delta_y = -neighbor_cell_radius_;
           delta_y <= neighbor_cell_radius_; ++delta_y) {
        neighbor_key.x = key.x + delta_x;
        neighbor_key.y = key.y + delta_y;
        auto neighbor_iter = cells->find(neighbor_key);
        if (neighbor_iter == cells->end()) {
          continue;
        }
        const auto& neighbor = neighbor_iter->second;
        samples.emplace_back(neighbor.center_x, neighbor.center_y,
                             neighbor.min_z);
        smooth_ground_sum += neighbor.min_z;
        ++smooth_ground_count;
        min_neighbor_z = std::min(min_neighbor_z, neighbor.min_z);
        max_neighbor_z = std::max(max_neighbor_z, neighbor.min_z);
      }
    }
    float ground_z = cell.min_z;
    if (!FitPlane(samples, cell.center_x, cell.center_y, &ground_z) &&
        smooth_ground_count > 0) {
      ground_z = smooth_ground_sum / static_cast<float>(smooth_ground_count);
    }
    cell.ground_z = std::min(ground_z, cell.min_z + max_ground_z_offset_);
    cell.local_gradient = max_neighbor_z - min_neighbor_z;
    if (!cell.indices.empty()) {
      const float inv_n = 1.0f / static_cast<float>(cell.indices.size());
      const float mean = cell.sum_z * inv_n;
      const float variance =
          std::max(0.0f, cell.sum_sq_z * inv_n - mean * mean);
      cell.roughness = std::sqrt(variance);
    } else {
      cell.roughness = 0.0f;
    }
  }
}

void SegmentationGroundDetector::ApplyNearFieldHysteresis(
    CellMap* near_cells) const {
  std::queue<CellKey> seed_queue;
  CellKey current_cell;
  CellKey neighbor_key;
  std::queue<CellKey> component_queue;
  std::vector<CellKey> component_cells;
  for (auto& cell_entry : *near_cells) {
    if (CellObstacleFlag::kStrong == cell_entry.second.obstacle_flag) {
      cell_entry.second.keep = true;
      seed_queue.push(cell_entry.first);
    }
  }
  while (!seed_queue.empty()) {
    current_cell = seed_queue.front();
    seed_queue.pop();
    for (int delta_x = -neighbor_cell_radius_; delta_x <= neighbor_cell_radius_;
         ++delta_x) {
      for (int delta_y = -neighbor_cell_radius_;
           delta_y <= neighbor_cell_radius_; ++delta_y) {
        if (0 == delta_x && 0 == delta_y) {
          continue;
        }
        neighbor_key.x = current_cell.x + delta_x;
        neighbor_key.y = current_cell.y + delta_y;
        auto neighbor_iter = near_cells->find(neighbor_key);
        if (neighbor_iter == near_cells->end() || neighbor_iter->second.keep ||
            CellObstacleFlag::kWeak != neighbor_iter->second.obstacle_flag) {
          continue;
        }
        neighbor_iter->second.keep = true;
        seed_queue.push(neighbor_key);
      }
    }
  }

  std::unordered_map<CellKey, bool, CellKeyHash> visited;
  visited.reserve(near_cells->size());
  for (const auto& cell_entry : *near_cells) {
    if (CellObstacleFlag::kWeak != cell_entry.second.obstacle_flag ||
        cell_entry.second.keep) {
      continue;
    }
    if (visited.count(cell_entry.first) > 0) {
      continue;
    }

    std::queue<CellKey> empty_queue;
    std::swap(component_queue, empty_queue);
    component_cells.clear();
    float component_peak = 0.0f;
    int component_point_count = 0;
    component_queue.push(cell_entry.first);
    visited[cell_entry.first] = true;
    while (!component_queue.empty()) {
      current_cell = component_queue.front();
      component_queue.pop();
      auto current_iter = near_cells->find(current_cell);
      if (current_iter == near_cells->end() ||
          CellObstacleFlag::kWeak != current_iter->second.obstacle_flag ||
          current_iter->second.keep) {
        continue;
      }
      component_cells.emplace_back(current_cell);
      component_peak =
          std::max(component_peak, current_iter->second.max_delta_z);
      component_point_count +=
          static_cast<int>(current_iter->second.indices.size());
      for (int delta_x = -neighbor_cell_radius_; delta_x <= neighbor_cell_radius_;
           ++delta_x) {
        for (int delta_y = -neighbor_cell_radius_;
             delta_y <= neighbor_cell_radius_; ++delta_y) {
          if (0 == delta_x && 0 == delta_y) {
            continue;
          }
          neighbor_key.x = current_cell.x + delta_x;
          neighbor_key.y = current_cell.y + delta_y;
          if (visited.count(neighbor_key) > 0) {
            continue;
          }
          auto neighbor_iter = near_cells->find(neighbor_key);
          if (neighbor_iter == near_cells->end() ||
              CellObstacleFlag::kWeak != neighbor_iter->second.obstacle_flag ||
              neighbor_iter->second.keep) {
            continue;
          }
          visited[neighbor_key] = true;
          component_queue.push(neighbor_key);
        }
      }
    }

    const bool enough_cells =
        static_cast<int>(component_cells.size()) >= weak_cluster_min_cells_;
    const bool dense_close_component =
        component_point_count >= plane_min_cells_;
    if ((enough_cells || dense_close_component) &&
        component_peak >= weak_cluster_min_peak_) {
      for (const auto& cell_key : component_cells) {
        (*near_cells)[cell_key].keep = true;
      }
    }
  }
}

void SegmentationGroundDetector::CollectNonGroundIndices(
    LidarFrame* frame, const CellMap& near_cells,
    const CellMap& far_cells) const {
  auto& indices = frame->non_ground_indices.indices;
  indices.clear();
  indices.reserve(frame->raw_cloud->size());
  for (const auto& kv : near_cells) {
    const auto& cell = kv.second;
    for (int index : cell.indices) {
      const auto& point = (*frame->raw_cloud)[index];
      const float delta_z = point.z - cell.ground_z;
      if (cell.keep && delta_z >= weak_obstacle_threshold_) {
        indices.emplace_back(index);
      } else {
        (*frame->raw_cloud->mutable_points_label())[index] =
            static_cast<uint8_t>(LidarPointLabel::GROUND);
      }
    }
  }
  for (const auto& kv : far_cells) {
    const auto& cell = kv.second;
    for (int index : cell.indices) {
      const auto& point = (*frame->raw_cloud)[index];
      const float delta_z = point.z - cell.ground_z;
      if (delta_z >= far_obstacle_threshold_) {
        indices.emplace_back(index);
      } else {
        (*frame->raw_cloud->mutable_points_label())[index] =
            static_cast<uint8_t>(LidarPointLabel::GROUND);
      }
    }
  }
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
}

bool SegmentationGroundDetector::Detect(const GroundDetectorOptions& options,
                                        LidarFrame* frame) {
  if (nullptr == frame || nullptr == frame->raw_cloud ||
      frame->raw_cloud->empty()) {
    AERROR << "SegmentationGroundDetector input cloud is invalid.";
    return false;
  }

  CellMap near_cells;
  CellMap far_cells;
  BuildCellMaps(frame, &near_cells, &far_cells);
  EstimateGround(&near_cells);
  EstimateGround(&far_cells);

  for (auto& kv : near_cells) {
    auto& cell = kv.second;
    for (int index : cell.indices) {
      const auto& point = (*frame->raw_cloud)[index];
      cell.max_delta_z = std::max(cell.max_delta_z, point.z - cell.ground_z);
    }
    const float adaptive_offset =
        std::min(adaptive_threshold_cap_,
                 gradient_adaptive_scale_ * cell.local_gradient +
                     roughness_adaptive_scale_ * cell.roughness);
    const float weak_threshold = weak_obstacle_threshold_ + adaptive_offset;
    const float strong_threshold = strong_obstacle_threshold_ + adaptive_offset;
    if (cell.max_delta_z >= strong_threshold) {
      cell.obstacle_flag = CellObstacleFlag::kStrong;
    } else if (cell.max_delta_z >= weak_threshold &&
               (cell.local_gradient >= local_gradient_threshold_ ||
                cell.roughness >=
                    roughness_gradient_ratio_ * local_gradient_threshold_)) {
      cell.obstacle_flag = CellObstacleFlag::kWeak;
    }
  }
  for (auto& kv : far_cells) {
    auto& cell = kv.second;
    for (int index : cell.indices) {
      const auto& point = (*frame->raw_cloud)[index];
      cell.max_delta_z = std::max(cell.max_delta_z, point.z - cell.ground_z);
    }
  }

  ApplyNearFieldHysteresis(&near_cells);
  CollectNonGroundIndices(frame, near_cells, far_cells);

  if (enable_debug_log_) {
    AINFO << "SegmentationGroundDetector near cells: " << near_cells.size()
          << " far cells: " << far_cells.size()
          << " non-ground points: " << frame->non_ground_indices.indices.size();
  }
  return true;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SegmentationGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century
