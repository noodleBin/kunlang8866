#include "modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy/occ_map_generator.h"
namespace century {
namespace perception {
namespace lidar {

GridMap::GridMap(const double resolution, const uint32_t width, const uint32_t height, 
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  width_ = static_cast<uint32_t>(width / resolution) + margin_;
  height_ = static_cast<uint32_t>(height / resolution) + margin_; 
  data_ = new std::vector<int>[width_ * height_];
  Reset();
  for (uint32_t i=0; i<cloud->points.size(); i++) {
    uint32_t grid_index = static_cast<uint32_t>((cloud->points.at(i).x) / resolution) + 
                          static_cast<uint32_t>((cloud->points.at(i).y) / resolution) * width_ ; 
    ACHECK(grid_index < width_ * height_)   
     << "grid_index "<< grid_index << ", width " << width_ << ", height " << height_;
    data_[grid_index].push_back(static_cast<int>(i));
  }

}

void GridMap::Reset() {
  ACHECK(data_ != nullptr) << "data_ is nullptr";
  for(uint32_t i = 0; i < width_ * height_; ++i) {
    data_[i].clear();
  }
  return;
}

std::vector<int>* GridMap::GetCellOccValue(const uint32_t x, const uint32_t y) {
  ACHECK(x < width_ && y < height_) 
  << "x "<< x << ", y " << y << ", width " << width_ << ", height " << height_;
  return &data_[y * width_ + x];
}

bool GridMap::IsCellOccupied(const uint32_t x, const uint32_t y) const {
  ACHECK(x < width_ && y < height_)
  << "x "<< x << ", y " << y << ", width " << width_ << ", height " << height_;
  return !data_[y * width_ + x].empty();
}

BoundingBox2D OccupancyMap::CalculateBoundingBox2D(const std::shared_ptr<base::AttributePointCloud<base::PointF>>& input_cloud) {
  BoundingBox2D bbox;
  for (const auto& point : input_cloud->points()) {
    if (point.x < bbox.min_x) bbox.min_x = point.x;
    if (point.x > bbox.max_x) bbox.max_x = point.x;
    if (point.y < bbox.min_y) bbox.min_y = point.y;
    if (point.y > bbox.max_y) bbox.max_y = point.y;
  }

  return bbox;
}
OccupancyMap::OccupancyMap(const double resolution, 
                           const std::shared_ptr<base::AttributePointCloud<base::PointF>>& cloud) {
  auto bbx = CalculateBoundingBox2D(cloud);
  AINFO << "bbx: " << bbx.min_x << " " << bbx.max_x << " " << bbx.min_y << " " << bbx.max_y << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  input_cloud->reserve(cloud->size());
  for(size_t i=0 ; i< cloud->points().size(); i++) {
    input_cloud->push_back({cloud->points().at(i).x - bbx.min_x, 
                            cloud->points().at(i).y - bbx.min_y, 
                            cloud->points().at(i).z });
  }
  auto width = bbx.max_x - bbx.min_x;
  auto height = bbx.max_y - bbx.min_y;
  grid_map_ = std::make_shared<GridMap>(resolution, width, height, input_cloud);
  
}
}
}
}