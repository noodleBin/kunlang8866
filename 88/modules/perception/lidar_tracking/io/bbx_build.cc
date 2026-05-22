#include "modules/perception/lidar_tracking/io/bbx_build.h"
#include "cyber/common/log.h"
#include <cmath>
namespace century {
namespace perception {
namespace lidar {

void BBXBuilderMatcher::Update() {
  if(using_point_cloud_polygon_thres_< 0.0) {
    return;
  }
 
  for(const auto& iter: obstacle_cluster_map_) {
    uint32_t min_length_indice = iter.second.front().GetIndice();
    double min_length = iter.second.front().GetLength();
    for( const auto& box: iter.second) {
      if(box.GetLength() > min_length) {
        min_length = box.GetLength();
        min_length_indice = box.GetIndice();
      }
    } 
    AINFO <<"=====================Update=====================";
    AINFO << std::fixed << iter.first <<":"<< min_length_indice << ", length:" << min_length ;

    obstacle_map_[iter.first] = min_length_indice;
    cluster_map_[min_length_indice] = iter.first; 
  }
  
}

void BBXBuilderMatcher::Reset(const double using_point_cloud_polygon_thres, 
                              const double min_length_threshold, const double height_threshold) {

  obstacle_cluster_map_.clear();
  obstacle_map_.clear();
  cluster_map_.clear();
  bbxs_.clear();
  using_point_cloud_polygon_thres_ = using_point_cloud_polygon_thres;
  min_length_threshold_ = min_length_threshold;
  height_threshold_ = height_threshold;
}


void BBXBuilderMatcher::FeedUpBBX(const int32_t indice, const double height,
                                  const century::perception::base::PointCloud<century::perception::base::PointD>& polygon,
                                  const Eigen::Vector3d& center) {
  
  ADEBUG << std::fixed << "bbx feedup======================================indice: " << indice << " height: " << height
        << " center: " << center(0) << " " << center(1);
   
  if((using_point_cloud_polygon_thres_ < 0.0) || 
    (std::hypotf(center(0), center(1)) > using_point_cloud_polygon_thres_)){
    return;
  }

  std::vector<Eigen::Vector2d> polygon2d;
  for(const auto& point : polygon) {
    polygon2d.emplace_back(point.x, point.y);
  }

  bbxs_.emplace_back(indice, polygon2d, center, height);
            
                                 
}
void BBXBuilderMatcher::MatchResult(const int32_t indice,
                                    const Eigen::Vector3d& obstacle_center, 
                                    const Eigen::Vector3f& obstacle_size,
                                    const double& yaw) {
  for(auto& bbx : bbxs_) {
    if(!bbx.MatchResult(obstacle_center(0), obstacle_center(1), obstacle_size, yaw, 
                        height_threshold_, min_length_threshold_)) {
      continue;
    }
    AINFO << std::fixed <<"MatchResult obstacle indice:" << indice 
          <<" center "<<obstacle_center(0) <<"," << obstacle_center(1)
          <<" match cluster "<< bbx.GetIndice() << " length: " << bbx.GetLength()
          << " height: " << bbx.GetHeight() << " center: " << bbx.GetCenter()(0) << " " << bbx.GetCenter()(1);
    obstacle_cluster_map_[indice].push_back (bbx);  
  }
  return ;
}

double BoundingBoxBuild::TriangleArea(const  Eigen::Vector2d& a, const  Eigen::Vector2d& b, const  Eigen::Vector2d& c) {
  return std::abs((a(0)*(b(1)-c(1)) + b(0)*(c(1)-a(1)) + c(0)*(a(1)-b(1)))/2.0);
}

bool BoundingBoxBuild::IsPointInsideRectangle(const Eigen::Vector2d& p, const  std::array<Eigen::Vector2d, 4UL>& rect) {

  double rectArea = TriangleArea(rect[0], rect[1], rect[2]) + 
                    TriangleArea(rect[0], rect[2], rect[3]);
  

  double areaSum =  TriangleArea(p, rect[0], rect[1]) +
                    TriangleArea(p, rect[1], rect[2]) +
                    TriangleArea(p, rect[2], rect[3]) +
                    TriangleArea(p, rect[3], rect[0]);
  
  return std::abs(areaSum - rectArea) < 1e-5;
}
  

BoundingBoxBuild::BoundingBoxBuild(const int32_t indice,
                                   const std::vector<Eigen::Vector2d>& polygon,
                                   const Eigen::Vector3d& center, const double& height) {
  center_ = Eigen::Vector2d(center(0), center(1));
  height_ = height;
  indice_ = indice;
  length_ = LongestDiagonal(polygon);

  return; 
}

bool BoundingBoxBuild::MatchResult( const double& x, const double& y, 
                                    const Eigen::Vector3f& size ,const double& yaw, 
                                    const double& height_thres, const double& min_length_threshold) {
 

  Eigen::Matrix2d rotation;
  rotation << cos(yaw), -sin(yaw),
              sin(yaw), cos(yaw);
                                      
  std::array<Eigen::Vector2d, 4UL> corners = {
    Eigen::Vector2d(size(0)/2.0, size(1)/2.0),
    Eigen::Vector2d(size(0)/2.0, -size(1)/2.0),
    Eigen::Vector2d(-size(0)/2.0, -size(1)/2.0),
    Eigen::Vector2d(-size(0)/2.0, size(1)/2.0)
  };

  for (auto& corner : corners) {
    corner = rotation * corner + Eigen::Vector2d(x,y);
  }


  ADEBUG <<"obstacle corner: ("<< corners[0](0) << "," << corners[0](1) <<")," 
         <<"              ("<< corners[1](0) << "," << corners[1](1) <<"),"
         <<"              ("<< corners[2](0) << "," << corners[2](1) <<"),"
         <<"              ("<< corners[3](0) << "," << corners[3](1) <<")";
  if(!IsPointInsideRectangle(center_, corners)) {
    return false;
  }

  if (std::abs(size(2) - height_) > height_thres || (size(1) - length_) > min_length_threshold
     || (length_- std::hypotf(size(0), size(1))) > min_length_threshold) {
    return false;
  }
  AINFO <<"center: (" << center_(0) << "," << center_(1) << ")"
        << " x: " << x << " y: " << y
        <<" Matched length is " << length_ << " height: " << height_
        <<" Obstacle length:" << size(0) <<" obstacle width:" << size(1);
  return  true;
}



double BoundingBoxBuild::PolygonArea(const std::vector<Eigen::Vector2d>& polygon) {
  int n = polygon.size();
  if (n < 3) {
      return 0.0;
  }

  double area = 0.0;
  for (int i = 0; i < n - 1; ++i) {
    area += polygon[i].x() * polygon[i + 1].y() - polygon[i].y() * polygon[i + 1].x();
  }
  area += polygon[n - 1].x() * polygon[0].y() - polygon[n - 1].y() * polygon[0].x();

  return 0.5 * std::abs(area);
}

double BoundingBoxBuild::LongestDiagonal(const std::vector<Eigen::Vector2d>& polygon) {
  if (polygon.size() < 3) {
    return 0.0; 
  }

  ADEBUG <<"==============polygon============: " << std::endl;
  ADEBUG << std::fixed<< center_(0) << "," << center_(1) << std::endl;
  for (auto& point : polygon) {
    ADEBUG << std::fixed << point(0) << "," << point(1);
  }
  double max_length = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    for (size_t j = i + 1; j < polygon.size(); ++j) {
      double length = (polygon[i]- polygon[j]).norm();
      if (length > max_length) {
        max_length = length;
      }
    }
  }

  return max_length;
}
 
}
}
}