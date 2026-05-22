#include "segmentation_ground_detector.h"

#include "gtest/gtest.h"

namespace century {
namespace perception {
namespace lidar {

TEST(SegmentationGroundDetectorTest,
     keep_dense_single_weak_component_for_close_cone) {
  SegmentationGroundDetector detector;
  detector.plane_min_cells_ = 4;
  detector.weak_cluster_min_cells_ = 3;
  detector.weak_cluster_min_peak_ = 0.10f;

  SegmentationGroundDetector::CellMap near_cells;
  SegmentationGroundDetector::CellStats weak_cell;
  weak_cell.obstacle_flag = SegmentationGroundDetector::CellObstacleFlag::kWeak;
  weak_cell.max_delta_z = 0.12f;
  weak_cell.indices = {0, 1, 2, 3};
  near_cells.emplace(SegmentationGroundDetector::CellKey{0, 0}, weak_cell);

  detector.ApplyNearFieldHysteresis(&near_cells);

  const auto cell_iter =
      near_cells.find(SegmentationGroundDetector::CellKey{0, 0});
  ASSERT_TRUE(cell_iter != near_cells.end());
  EXPECT_TRUE(cell_iter->second.keep);
}

TEST(SegmentationGroundDetectorTest,
     reject_sparse_single_weak_component_without_support) {
  SegmentationGroundDetector detector;
  detector.plane_min_cells_ = 4;
  detector.weak_cluster_min_cells_ = 3;
  detector.weak_cluster_min_peak_ = 0.10f;

  SegmentationGroundDetector::CellMap near_cells;
  SegmentationGroundDetector::CellStats weak_cell;
  weak_cell.obstacle_flag = SegmentationGroundDetector::CellObstacleFlag::kWeak;
  weak_cell.max_delta_z = 0.12f;
  weak_cell.indices = {0, 1, 2};
  near_cells.emplace(SegmentationGroundDetector::CellKey{0, 0}, weak_cell);

  detector.ApplyNearFieldHysteresis(&near_cells);

  const auto cell_iter =
      near_cells.find(SegmentationGroundDetector::CellKey{0, 0});
  ASSERT_TRUE(cell_iter != near_cells.end());
  EXPECT_FALSE(cell_iter->second.keep);
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
