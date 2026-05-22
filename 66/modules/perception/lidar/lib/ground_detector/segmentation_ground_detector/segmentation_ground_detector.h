#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "modules/perception/lidar/lib/interface/base_ground_detector.h"

namespace century {
namespace perception {
namespace lidar {

class SegmentationGroundDetector : public BaseGroundDetector {
 public:
  SegmentationGroundDetector() = default;
  ~SegmentationGroundDetector() override = default;

  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;

  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "SegmentationGroundDetector"; }

 private:
  struct CellKey {
    int x = 0;
    int y = 0;

    bool operator==(const CellKey& other) const {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash {
    size_t operator()(const CellKey& key) const;
  };

  enum class CellObstacleFlag : uint8_t {
    kNone = 0,
    kWeak = 1,
    kStrong = 2,
  };

  struct CellStats {
    std::vector<int> indices;
    float min_z = 0.0f;
    float sum_z = 0.0f;
    float sum_sq_z = 0.0f;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float ground_z = 0.0f;
    float max_delta_z = 0.0f;
    float local_gradient = 0.0f;
    float roughness = 0.0f;
    CellObstacleFlag obstacle_flag = CellObstacleFlag::kNone;
    bool keep = false;
  };

  using CellMap = std::unordered_map<CellKey, CellStats, CellKeyHash>;

  bool BuildCellMaps(LidarFrame* frame, CellMap* near_cells,
                     CellMap* far_cells) const;
  void EstimateGround(CellMap* cells) const;
  void ApplyNearFieldHysteresis(CellMap* near_cells) const;
  void CollectNonGroundIndices(LidarFrame* frame, const CellMap& near_cells,
                               const CellMap& far_cells) const;
  bool FitPlane(const std::vector<Eigen::Vector3f>& samples, float query_x,
                float query_y, float* ground_z) const;

  float near_range_ = 8.0f;
  float near_grid_size_ = 0.12f;
  float far_grid_size_ = 0.30f;
  float weak_obstacle_threshold_ = 0.08f;
  float strong_obstacle_threshold_ = 0.18f;
  float far_obstacle_threshold_ = 0.20f;
  int neighbor_cell_radius_ = 1;
  int plane_min_cells_ = 4;
  float plane_max_residual_ = 0.08f;
  float local_gradient_threshold_ = 0.03f;
  float gradient_adaptive_scale_ = 0.5f;
  float roughness_adaptive_scale_ = 1.0f;
  float adaptive_threshold_cap_ = 0.06f;
  int weak_cluster_min_cells_ = 3;
  float weak_cluster_min_peak_ = 0.10f;
  float x_min_ = -80.0f;
  float x_max_ = 80.0f;
  float y_min_ = -80.0f;
  float y_max_ = 80.0f;
  float z_min_ = -5.0f;
  float z_max_ = 5.0f;
  // Maximum vertical offset by which the fitted ground plane may exceed the
  // cell's lowest observed point.  Prevents overestimation on ramps/slopes.
  float max_ground_z_offset_ = 0.25f;
  float roughness_gradient_ratio_ = 0.5f;
  bool enable_debug_log_ = false;
};

}  // namespace lidar
}  // namespace perception
}  // namespace century
