#include "modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy/bev_occupancy_cluster.h"
#include <vector>
#include <fstream>
#include <stdlib.h>
#include "cyber/common/log.h"

using namespace century::perception::lidar;
using namespace century::perception;
namespace {
bool FeedUpPcl(std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud, 
               const std::string file_name) {
  std::ifstream file(file_name);
  if (!file.is_open()) {
    AERROR << file_name <<" Open failed" << std::endl;
    return false;
  }
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    base::PointF point;
    point.x = std::stof(tokens[0]);
    point.y = std::stof(tokens[1]);
    point.z = std::stof(tokens[2]);
    cloud->push_back(point);
  }

  file.close();
  return true;
}
}


int main(int argc, char** argv) {

  if (argc < 2) {
    AERROR << "Missing input file name";
    return -1;
  }

  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud = nullptr;
  cloud.reset(new base::AttributePointCloud<base::PointF>()); 
  if (!FeedUpPcl(cloud, argv[1])) {
    AERROR << "Failed to read input file " << argv[1];
    return -1;
  }
 

  OccupancyMap occ_map(0.7, cloud);
  auto grid_map = occ_map.GetGridMap();
  AINFO <<" grid map width and height  "<< grid_map->GetWidth() << ", " << grid_map->GetHeight();
  IslandSegment segment(grid_map);
  std::vector<std::vector<int>> islands = segment.FindIslandsIndices();

  AINFO << "Number of islands: " << islands.size();
  for(const auto& island : islands) {
    Eigen::Vector2f centroid(0, 0);
    for (const auto& pt : island) {
      centroid += Eigen::Vector2f(cloud->points().at(pt).x , cloud->points().at(pt).y);
    }
    centroid = centroid /island.size(); 

    std::cout << centroid(0) << ", " << centroid(1) << std::endl;
  }
 
  return 0;
}