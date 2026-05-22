#pragma once

#include "modules/perception/common/algorithm/data_structures/grid.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_object_filter.h"
#include "modules/perception/lidar/lib/object_filter_bank/cone_filter/proto/cone_filter_config.pb.h"
#include <utility>

namespace century {
namespace perception {
namespace lidar {

using algorithm::GridId3d;

class ConeFilter : public BaseObjectFilter {
 public:
  ConeFilter() = default;
  virtual ~ConeFilter() = default;

  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override;
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;
  std::string Name() const override { return "ConeFilter"; }
 private:
  base::Object ToObj(std::pair<GridId3d, std::pair<int, int>> grid, Eigen::Affine3d affine);
  double GetGridSize() const;
  double grid_range_;
  double grid_size_;
  double match_distance_;
  int appera_count_;
  int disappear_count_;
  std::vector<std::pair<GridId3d, std::pair<int, int>>> grids_;
  ConeFilterConfig cone_filter_config_;
};


}
}
}
