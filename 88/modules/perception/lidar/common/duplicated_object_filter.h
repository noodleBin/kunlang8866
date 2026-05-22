#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
namespace century {
namespace perception {
namespace lidar {
struct DuplicatedArea {
  bool is_ped;
  Eigen::Vector2d center;
  double yaw;
  double length;
  double width;
};
class DuplicatedObjectFilter {
  public:
  DuplicatedObjectFilter() = default;
  explicit DuplicatedObjectFilter(const double distance_diff): distance_diff_(distance_diff) {}
  virtual ~DuplicatedObjectFilter() = default;
  void AddPoint(const Eigen::Vector3d& point, const double length, const double width, const bool is_ped);
  bool IsDuplicated(const Eigen::Vector2d& point) const;

  private:
  std::vector<DuplicatedArea> src_area_;
  double distance_diff_= 1.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
}
}