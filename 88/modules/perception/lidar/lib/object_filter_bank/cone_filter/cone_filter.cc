#include "modules/perception/lidar/lib/object_filter_bank/cone_filter/cone_filter.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/common/geometry/common.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "sys/time.h"

namespace century {
namespace perception {
namespace lidar {

namespace cone_filter {
struct Point {
  double x, y;
  Point(double x_, double y_) : x(x_), y(y_) {}
};
} // cone_fileter
using Point=cone_filter::Point;

double Cross(const Point& O, const Point& A, const Point& B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// （Andrew's monotone chain）
std::vector<Point> ConvexHull(std::vector<Point>& points) {
  if (points.size() <= 3) return points;
  
  std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  
  std::vector<Point> hull;
  for (int i = 0; i < points.size(); ++i) {
    while (hull.size() >= 2 && 
            Cross(hull[hull.size()-2], hull.back(), points[i]) <= 0) {
        hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  
  for (int i = points.size()-2, t = hull.size()+1; i >= 0; --i) {
    while (hull.size() >= t && 
            Cross(hull[hull.size()-2], hull.back(), points[i]) <= 0) {
        hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  
  hull.pop_back();
  return hull;
}

double CalculateArea(const std::vector<Point>& polygon) {
  double area = 0.0;
  int n = polygon.size();
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    area += polygon[i].x * polygon[j].y;
    area -= polygon[j].x * polygon[i].y;
  }
  return std::abs(area) / 2.0;
}

bool ConeFilter::Init(const ObjectFilterInitOptions& options) {
  std::string config_file;

  auto* config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(
      config_manager->GetModelConfig("PointCloudPreprocessor", &model_config));
  const std::string work_root = config_manager->work_root();

  config_file = century::cyber::common::GetAbsolutePath(work_root, "conf/perception/lidar");
  
  config_file =
      century::cyber::common::GetAbsolutePath(config_file, "cone_filter_conf.pb.txt");
  
  ACHECK(
      century::cyber::common::GetProtoFromFile(config_file, &cone_filter_config_))
       << ", config_file: " << config_file;
  grid_range_ = cone_filter_config_.grid_range();
  grid_size_ = cone_filter_config_.grid_size();
  appera_count_ = cone_filter_config_.appera_count();
  disappear_count_ = cone_filter_config_.disappear_count();
  match_distance_ = cone_filter_config_.match_distance();
  // std::cout << "get grid size1 " << GetGridSize() << " " << grid_size_ 
  //           << "grid_range_ " << grid_range_ << "appera_count_ " << appera_count_
  //           << "disappear_count_ " << disappear_count_
  //           << std::endl;
  return true;
}

void GetClass(base::ObjectType type) {
  switch (type) {
  case base::ObjectType::UNKNOWN:
  case base::ObjectType::UNKNOWN_MOVABLE:
  case base::ObjectType::UNKNOWN_UNMOVABLE:
    std::cout << "UNKNOWN" << std::endl;
    break;
  case base::ObjectType::PEDESTRIAN:
    std::cout << "PEDESTRIAN" << std::endl;
    break;
  case base::ObjectType::BICYCLE:

    std::cout << "BICYCLE" << std::endl;
    break;
  case base::ObjectType::VEHICLE:
    std::cout << "VEHICLE" << std::endl;
    break;
  case base::ObjectType::NARROW20FOOT:
    std::cout << "NARROW20FOOT" << std::endl;
    break;
  case base::ObjectType::DUMMY:
    std::cout << "DUMMY" << std::endl;
    break;
  case base::ObjectType::CONE:
    std::cout << "CONE" << std::endl;
    break;
  case base::ObjectType::STACKER:
    std::cout << "STACKER" << std::endl;
    break;
  case base::ObjectType::MAX_OBJECT_TYPE:
    std::cout << "MAX_OBJECT_TYPE" << std::endl;
    break;
  default:
    break;
  }

}

base::Object ConeFilter::ToObj(std::pair<GridId3d, std::pair<int, int>> grid, Eigen::Affine3d affine) {
  base::Object object;
  object.confidence = 0.1;
  object.lidar_supplement.is_in_roi = true;
  object.lidar_supplement.num_points_in_roi = 4;
  auto yaw = 0.f;
  auto roll = 0.f;
  auto pitch = 0.f;

  Eigen::Vector3d local_position = affine.inverse() * Eigen::Vector3d(grid.first.getCenter(0), grid.first.getCenter(1), grid.first.getCenter(2));
  auto x = local_position.x();
  auto y = local_position.y();
  auto z = local_position.z();
  auto dx = 0.5;
  auto dy = 0.5;
  auto dz = 0.5;
  Eigen::Quaternionf quater =
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(x, y, z);
  Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
  for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
    for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
      for (float vz : std::vector<float>{0, dz}) {
        Eigen::Vector3f v3f(vx, vy, vz);
        v3f = affine3f * v3f;
        base::PointF point;
        point.x = v3f.x();
        point.y = v3f.y();
        point.z = v3f.z();
        
        object.lidar_supplement.cloud.push_back(point);
        
        Eigen::Vector3d v3d(point.x, point.y, point.z);
        auto ptd_world = affine * v3d;
        base::PointD ptd;
        ptd.x = ptd_world.x();
        ptd.y = ptd_world.y();
        ptd.z = ptd_world.z();
        object.lidar_supplement.cloud_world.push_back(ptd);
      }
    }
  }

  // classification
  object.type = base::ObjectType::CONE;
  object.lidar_supplement.raw_probs.push_back(std::vector<float>(
      static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
  object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] =
      0.1;
  // object.lidar_supplement.raw_classification_methods.push_back(Name());
  // copy to type
  object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                          object.lidar_supplement.raw_probs.back().end());
  // copy to background objects
  return object;
}

double ConeFilter::GetGridSize() const{
  return grid_size_;
}

bool ConeFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
  struct timeval start, end;
  gettimeofday(&start, NULL);
  auto& ego_pose = frame->lidar2world_pose;
  auto ego_location =  century::common::math::Vec2d(ego_pose.translation().x(), ego_pose.translation().y());
  if (grids_.size() == 0) {
    // std::cout << "first frame " << grid_size_ << std::endl;
    algorithm::Container3d<base::ObjectType> container3d(grid_size_);
    
    for (auto it = frame->segmented_objects.begin(); it != frame->segmented_objects.end();) {
      if ((*it)->type == base::ObjectType::CONE) {
        
        auto box_size = (*it)->size(0) * (*it)->size(1);
        if (box_size >= 2) {
          it++;
          continue;
        }
        // GetClass((*it)->type);
        auto map_location = frame->lidar2world_pose * (*it)->center;
        // std::cout << " polygon size " << (*it)->polygon.size()
        //           << " point  " << (*it)->center(0) << " " << (*it)->center(1) << " " << (*it)->center(2)
        //           << " points address " << *(&map_location(0)) << " " << *(&map_location(0) + 1) << " " << *(&map_location(0) + 2)
        //           << " position " << map_location(0) << " " << map_location(1) << " " << map_location(2) << std::endl;
        container3d.emplace(&map_location(0), (*it)->type);
      }
      it++;
    }
    for (auto iter = container3d.begin(); iter != container3d.end(); iter++) {
      grids_.push_back({iter->first, {1,1}});
    }
  } else {
    algorithm::Container3d<base::ObjectType> container3d(grid_size_);
    for (auto it = frame->segmented_objects.begin(); it != frame->segmented_objects.end();) {
      {
        bool match_flag = false;
        auto map_location = frame->lidar2world_pose * (*it)->center;

        auto box_size = (*it)->size(0) * (*it)->size(1);
        if ((*it)->type == base::ObjectType::UNKNOWN) {
          std::vector<Point> points;
          for (int i = 0 ; i < (*it)->lidar_supplement.cloud.size(); i++) {
            points.emplace_back(Point((*it)->lidar_supplement.cloud.at(i).x, (*it)->lidar_supplement.cloud.at(i).y));
          }
          auto hull = ConvexHull(points);
          double area = CalculateArea(hull);
          box_size = area;
        }
        // GetClass((*it)->type);
        // std::cout << "box area " << box_size << std::endl;
        century::common::math::Vec2d point(map_location(0), map_location(1));
        for (auto iter_grid = grids_.begin(); iter_grid != grids_.end(); iter_grid++) {
           century::common::math::Vec2d point_grid(iter_grid->first.getCenter(0), iter_grid->first.getCenter(1));
          if (point_grid.DistanceTo(point) < match_distance_ and box_size < 2) {
            iter_grid->second.first ++;
            iter_grid->second.second = 1;
            match_flag = true;
            break;
          }
        }
        if (!match_flag) {
          if ((*it)->type == base::ObjectType::CONE) {
            container3d.emplace(&map_location(0), (*it)->type);
          }
          it++;
        } else {
          it = frame->segmented_objects.erase(it);
        }
      }
    }
    for (auto iter = container3d.begin(); iter != container3d.end(); iter++) {
      grids_.push_back({iter->first, {1,1}});
    }
  }

  for (auto iter_grid = grids_.begin(); iter_grid != grids_.end(); ) {
     century::common::math::Vec2d point_grid(iter_grid->first.getCenter(0), iter_grid->first.getCenter(1));
    iter_grid->second.second--;
    if (iter_grid->second.second <= (-1 * disappear_count_) || ego_location.DistanceTo(point_grid) > grid_range_) {
        iter_grid = grids_.erase(iter_grid); 
    } else {
        if (iter_grid->second.first >= appera_count_) {
          auto object = ToObj(*iter_grid, ego_pose);
          std::shared_ptr<base::Object> obj(new base::Object);
          object.sub_type = base::ObjectSubType::CONE;
          *obj = object;
          frame->segmented_objects.push_back(obj);
        }
        ++iter_grid; 
    }
  }
  gettimeofday(&end, NULL);
  long seconds = end.tv_sec - start.tv_sec;
  long useconds = end.tv_usec - start.tv_usec;
  double elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;
  AINFO << "cone detector time " << elapsed_ms <<" ms";
  
  return true;
}

PERCEPTION_REGISTER_OBJECTFILTER(ConeFilter);


}
}
}


