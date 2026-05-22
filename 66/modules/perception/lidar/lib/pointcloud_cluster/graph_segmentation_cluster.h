#pragma once

#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/pointcloud_cluster/proto/pointcloud_cluster_config.pb.h"

#include "modules/perception/lidar/lib/pointcloud_cluster/proto/pointcloud_cluster_config.pb.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

namespace century {
namespace perception {
namespace lidar {

struct Range {
  float z;
  float range;
  int index;
  int row;
  int col;
};

class GraphSegmentationCluster : public BasePointCloudCluster {
 public:
  GraphSegmentationCluster() = default;
  ~GraphSegmentationCluster() override = default;

  bool Init(const ClusterInitOptions& options = ClusterInitOptions()) override;
  bool Cluster(const ClusterOptions& options,
               LidarFrame* frame) override;
  std::string Name() const override { return "GraphSegmentationCluster"; }

 private:
  bool use_map_manager_;
  void ToRangeImage(std::shared_ptr<base::AttributePointCloud<base::PointF>> points,
                                          std::vector<Range>& map) const;
  void MergeClusters(std::vector<int>* cluster_indices, int idx1,
    int idx2) const;
  std::vector<int> RangeSeg(std::shared_ptr<base::AttributePointCloud<base::PointF>> points,
    std::vector<Range>& map) const;

  bool MostFrequentValue(const std::vector<int>& values,
    std::vector<std::vector<int32_t> >& cluster_indices) const;

  int32_t row_num_ = 32;
  int32_t col_num_ = 1800;
  float f_down_ = -16.f;
  float f_up_ = 15.f;
  float phi_v_ = 1.0;
  float phi_h_ = 0.2;
  float sigma_v_ = 0.03;
  float sigma_h_ = 0.03;
  float horizontal_search_angle_ = 20;
  float vertical_search_angle_ = 10;
  int32_t min_points_size_ = 3;
  PointCloudClusterConfig cluster_config_;

  MapManager map_manager_;
  lidar::BaseROIFilter* roi_filter_;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;

};

}  // namespace lidar
}  // namespace perception
}  // namespace century 