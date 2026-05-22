#include "modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy/island_segment.h"

namespace century {
namespace perception {
namespace lidar {
IslandSegment::IslandSegment(const std::shared_ptr<GridMap> grid_map) : 
                            grid_map_(grid_map) {
  ACHECK (grid_map_) << "grid map is null";
  AINFO <<__FUNCTION__ ;
  width_ = grid_map_->GetWidth();
  height_ = grid_map_->GetHeight();
  visited_ = new bool[width_ * height_];
  for (uint32_t i = 0; i < width_ * height_; ++i) {
    visited_[i] = false;
  }
  
}

IslandSegment::~IslandSegment() {
  if (visited_ != nullptr) {
    delete[] visited_;
    visited_ = nullptr;
  }
}

std::vector<std::vector<Point2Index>> IslandSegment::FindIslands() {
  std::vector<std::vector<Point2Index>> islands;
  
  ACHECK(width_ != 0 && height_ != 0 && grid_map_);
  for (uint32_t y = 0; y < height_; ++y) {
    for (uint32_t x = 0; x < width_; ++x) {
      Point2Index current{x, y};
      size_t idx = current.toIndex(width_);
      if (grid_map_->IsCellOccupied(x,y) && !visited_[idx]) {
        std::vector<Point2Index> island;
        std::queue<Point2Index> to_explore;
        
        to_explore.push(current);
        visited_[idx] = true;
        
        while (!to_explore.empty()) {
          Point2Index p = to_explore.front();
          to_explore.pop();
          island.push_back(p);
          
          const int dx[] = {-1, 1, 0, 0};
          const int dy[] = {0, 0, -1, 1};
          
          for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            
            if (nx >= 0 && nx < static_cast<int>(width_) && 
                ny >= 0 && ny < static_cast<int>(height_)) {
                
              Point2Index neighbor{static_cast<uint32_t>(nx), static_cast<uint32_t>(ny)};
              size_t neighborIdx = neighbor.toIndex(width_);
              
              if (grid_map_->IsCellOccupied(nx,ny) && !visited_[neighborIdx]) {
                visited_[neighborIdx] = true;
                to_explore.push(neighbor);
              }
            }
          }
        }
        
        if (!island.empty()) {
          islands.push_back(island);
        }
      }
    }
  }

  return islands;
}
std::vector<std::vector<int>> IslandSegment::FindIslandsIndices() {
  
  auto indices = FindIslands();
  std::vector<std::vector<int>> islands;
  islands.reserve(indices.size());
  for (auto &index : indices) {
    std::vector<int> island;
    for (auto &point : index) {
      auto cur = grid_map_->GetCellOccValue(point.x, point.y);
      ACHECK(cur);
      island.insert(island.end(), std::make_move_iterator(cur->begin()), std::make_move_iterator(cur->end()));
    }
    islands.push_back(std::move(island));
  }
  return islands;

}
}
}
}