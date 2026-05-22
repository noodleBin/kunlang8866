/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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
 *
 *******************************************************************************/
#pragma once
#include <iostream>
#include "modules/perception/base/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "cyber/common/log.h"

#include <unordered_map>
#include <vector>
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>
#include <list>
namespace century {
namespace perception {
namespace lidar {
struct Point2D {
  float x;
  float y;
};
struct BoundingBox2D {
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
};
class GridMap {
  public:
  GridMap() = default;
  explicit GridMap(const double resolution, 
                   const uint32_t width, 
                   const uint32_t height, 
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  ~GridMap() {
    if(data_) {
      delete[] data_;
    }
  }
  void Reset();
  uint32_t GetWidth() const {
    return width_;
  }

  uint32_t GetHeight() const {
    return height_;
  }

  std::vector<int>* GetCellOccValue(const uint32_t x, const uint32_t y);
  bool IsCellOccupied(const uint32_t x, const uint32_t y) const ;
 private:
  uint32_t width_;
  uint32_t height_; 
  std::vector<int>* data_ = nullptr;
  const uint32_t margin_ = 20;
};

class OccupancyMap {
 public:
  OccupancyMap() = default;
  explicit OccupancyMap(const double resolution, 
                        const std::shared_ptr<base::AttributePointCloud<base::PointF>>& cloud);
  virtual ~OccupancyMap() {}
  std::shared_ptr<GridMap> GetGridMap() const {
    return grid_map_;
  }

 private:
  BoundingBox2D CalculateBoundingBox2D(const std::shared_ptr<base::AttributePointCloud<base::PointF>>& input_cloud);
  std::shared_ptr<GridMap> grid_map_ = nullptr;

};

}
}
}