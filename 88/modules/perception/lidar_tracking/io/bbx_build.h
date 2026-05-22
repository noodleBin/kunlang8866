#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <array>
#include "modules/perception/base/point_cloud.h"
#include <list>
#include <unordered_map>
namespace century {
namespace perception {
namespace lidar {

 
class BoundingBoxBuild {
 public:
 
  BoundingBoxBuild() = default;
  explicit BoundingBoxBuild(const int32_t indice, 
                            const std::vector<Eigen::Vector2d>& polygon,
                            const Eigen::Vector3d& center, const double& height) ;
   
  bool MatchResult(const double& x, const double& y, const Eigen::Vector3f& size,
                   const double& yaw, const double& height_thres, const double& area_thres);
  int32_t GetIndice() const {
    return indice_;
  }
  double GetLength() const {
    return length_;
  }
  double GetHeight() const {
    return height_;
  }
  Eigen::Vector2d GetCenter() const {
    return center_;
  }
 private:

  std::vector<Eigen::Vector2d> polygon_;
  Eigen::Vector2d center_ = {0.0,0.0};
  double height_ = 0.0;
  double length_ = 0.0;
  int32_t indice_ = -1;
  double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
    return a(0) * b(1) - a(1) * b(0);
  }

  double dot(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
    return a(0) * b(0) + a(1) * b(1);
  }

  double length(const Eigen::Vector2d& p) const {
    return std::sqrt(p(0) * p(0) + p(1) * p(1));
  }

  Eigen::Vector2d normalize(const Eigen::Vector2d& p) const {
    double len = length(p);
    return {p(0) / len, p(1) / len};
  }

  std::vector<Eigen::Vector2d> convexHull();
  double PolygonArea(const std::vector<Eigen::Vector2d>& polygon);
  double TriangleArea(const  Eigen::Vector2d& a, const  Eigen::Vector2d& b, const  Eigen::Vector2d& c);
  bool IsPointInsideRectangle(const Eigen::Vector2d& p, const  std::array<Eigen::Vector2d, 4UL>& rect); 
  double LongestDiagonal(const std::vector<Eigen::Vector2d>& polygon);
};

class BBXBuilderMatcher {
 public:
  
  BBXBuilderMatcher() = default;
  virtual ~BBXBuilderMatcher() = default;
  static BBXBuilderMatcher* GetInstance() {
    static BBXBuilderMatcher bbx_build_matcher;
    return &bbx_build_matcher;
  }
  void Reset(const double using_point_cloud_polygon_thres, const double min_length_threshold, const double height_threshold);
  void FeedUpBBX(const int32_t indice, const double height,
                 const century::perception::base::PointCloud<century::perception::base::PointD>& polygon,
                 const Eigen::Vector3d& center) ;
  void MatchResult(const int32_t indice,
                   const Eigen::Vector3d& obstacle_center, 
                   const Eigen::Vector3f& obstacle_size, const double& yaw);
  void Update();

  int32_t GetMatchedObstacleFromCluster(const int32_t id) const {
    if(cluster_map_.count(id) == 0) {
      return -1;
    }
    return cluster_map_.at(id);
  }
  int32_t GetMatchedClusterFromObstacle (const int32_t id) const {
    if(obstacle_map_.count(id) == 0) {
      return -1;
    }
    return obstacle_map_.at(id);

  }
  private:
  double min_length_threshold_ = 3.0;
  double height_threshold_ = 0.5;
  double using_point_cloud_polygon_thres_ = -1.0;
  std::unordered_map<int32_t, int32_t> obstacle_map_;
  std::unordered_map<int32_t, int32_t> cluster_map_;
  std::unordered_map<int32_t, std::list<BoundingBoxBuild>>  obstacle_cluster_map_;

 
  std::list<BoundingBoxBuild> bbxs_;
 
  Eigen::Affine3d lidar_pose_;
  
  double ts_;
};


}
}
}