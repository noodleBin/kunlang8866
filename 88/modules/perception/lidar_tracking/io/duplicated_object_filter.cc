#include "modules/perception/lidar_tracking/io/duplicated_object_filter.h"
namespace century {
namespace perception {
namespace lidar {
namespace {

double TriangleArea(const  Eigen::Vector2d& a, const  Eigen::Vector2d& b, const  Eigen::Vector2d& c) {
  return std::abs((a(0)*(b(1)-c(1)) + b(0)*(c(1)-a(1)) + c(0)*(a(1)-b(1)))/2.0);
}

bool IsPointInsideRectangle(const Eigen::Vector2d& p, const  std::array<Eigen::Vector2d, 4UL>& rect) {

  double rectArea = TriangleArea(rect[0], rect[1], rect[2]) + 
                   TriangleArea(rect[0], rect[2], rect[3]);
  

  double areaSum = TriangleArea(p, rect[0], rect[1]) +
                  TriangleArea(p, rect[1], rect[2]) +
                  TriangleArea(p, rect[2], rect[3]) +
                  TriangleArea(p, rect[3], rect[0]);
  
  return std::abs(areaSum - rectArea) < 1e-5;
}


std::array<Eigen::Vector2d, 4UL> CalculateCornerPoints( double centerX, double centerY, 
                                                        double yaw, double length, double width) {
  std::array<Eigen::Vector2d, 4UL> corners;
  
  double halfLength = length / 2.0;
  double halfWidth = width / 2.0;
  
  double sinYaw = sin(yaw);
  double cosYaw = cos(yaw);
  
  double localCorners[4][2] = {
      { halfLength,  halfWidth},  
      { halfLength, -halfWidth},  
      {-halfLength, -halfWidth}, 
      {-halfLength,  halfWidth} 
  };
  
  for (int i = 0; i < corners.size(); ++i) {
    double localX = localCorners[i][0];
    double localY = localCorners[i][1];
    
    corners[i](0) = localX * cosYaw - localY * sinYaw;
    corners[i](1) = localX * sinYaw + localY * cosYaw;
    
    corners[i](0) += centerX;
    corners[i](1) += centerY;
  }
  
  return corners;
}

}
void DuplicatedObjectFilter::AddPoint(const Eigen::Vector3d& point, const double length, 
                                      const double width, const bool is_ped) {
  DuplicatedArea area;
  area.length = length;
  area.width = width;
  area.center = point.head<2>();
  area.yaw = point(2);
  area.is_ped = is_ped;
  src_area_.push_back(std::move(area));
}

bool DuplicatedObjectFilter::IsDuplicated(const Eigen::Vector2d& point) const {
  for (const auto& p : src_area_) {
    if ((p.center - point).norm() < (p.is_ped ? distance_diff_/2 : distance_diff_)) {
      return true;
    }

    auto corners = CalculateCornerPoints(p.center(0), p.center(1), p.yaw, p.length, p.width);
    if(IsPointInsideRectangle(point, corners)) {
      return true;
    }
  }
  return false;
}


}
}
}