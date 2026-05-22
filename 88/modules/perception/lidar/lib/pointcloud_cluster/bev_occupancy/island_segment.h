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
#include "modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy/occ_map_generator.h"
namespace century {
namespace perception {
namespace lidar {
struct Point2Index {
  uint32_t x;
  uint32_t y;
  size_t toIndex(uint32_t width) const {
    return y * width + x;
  }
};
class IslandSegment
{
 public:
  IslandSegment() = default;
  virtual ~IslandSegment();
  explicit IslandSegment(const std::shared_ptr<GridMap> grid_map);
  std::vector<std::vector<int>> FindIslandsIndices();
 private:
  std::vector<std::vector<Point2Index>> FindIslands();
  bool* visited_ = nullptr;
  std::shared_ptr<GridMap> grid_map_ = nullptr;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
};


}
}
}