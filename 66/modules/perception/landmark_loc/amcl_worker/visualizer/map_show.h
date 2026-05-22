#pragma once
#include "semantic_mapping/mapping_2d.h"
#include "amcl/map/map.h"
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
namespace semantic_mapping{
class MapShow{
  public:
 MapShow() = default;
 ~MapShow() = default;
 explicit MapShow(const std::array<::map_t*,MapLayerSize>& map, const Eigen::Matrix4d& origin);
 void Display(const int flag = 0);
 void AddPoselist(const std::vector<Eigen::Vector3d>& poselist) ;
 void AddDrPoint(const Eigen::Vector3d& dr) ;
 void AddPathPoint(const Eigen::Vector3d& path, const bool converaged = true) ;
 void AddSensorPoint(const std::vector<Eigen::Vector3d>& sensor) ;
 void draw_occ_dist(const POINT_LABEL& map);
 void Consume();
  private:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
 void clear();
 void draw_occ_state(const std::vector<uint32_t>& map);

 std::array<::map_t*, MapLayerSize> maps_{nullptr};
 Eigen::Matrix4d origin_;
 cv::Mat* occ_img_ = nullptr;
 cv::Mat* dis_img_ = nullptr;
 const std::string map_save_path_ = "/century/data/log/semantic/";
 std::vector<Eigen::Vector3d> drlist_;
 std::vector<Eigen::Vector3d> poselist_;
 std::vector<std::pair<Eigen::Vector3d, bool>> pathpoints_;
 std::vector<Eigen::Vector3d> sensor_;
 
 double resolution_;
 Eigen::Vector2d center_;
 std::thread* thread_ = nullptr;
 std::mutex mutex_;
 std::condition_variable not_empty_;
};
}