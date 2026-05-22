//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include <Eigen/Dense>

namespace semantic_mapping {

//[TODO] access via resolution()
class MapParam {
   public:
   MapParam() = default;
  // geometric
  explicit MapParam(const double width) {
    double resolution_ = 0.1;
    map_size_ = Eigen::Vector2d(width, width);
    grid_size_ = Eigen::Vector2i(std::ceil(map_size_[0] / resolution_), std::ceil(map_size_[1] / resolution_));
  // pos_in_world, grid_in_gridMap
    grid_center_ = Eigen::Vector2i(std::ceil(grid_size_[0] / 2), std::ceil(grid_size_[1] / 2));
  }

  MapParam& operator=(const MapParam& param) {
    this->map_size_ = param.map_size_;
    this->grid_size_ = param.grid_size_;
    this->grid_center_ = param.grid_center_;
    return *this;
  }

  const double resolution_ = 0.1;
  Eigen::Vector2d map_size_{50.0,50.0};
  Eigen::Vector2i grid_size_{500.0,500.0} ;
  Eigen::Vector2i grid_center_{250,250} ;
 
};

class CellParam {
   public:
  int min_times_ = 2;
  double min_rate_ = 0.25;
  double res_ratio_ = 1;
};

}


