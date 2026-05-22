#pragma once
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "pangolin/display/default_font.h"
#include "pangolin/gl/gl.h"
#include "pangolin/pangolin.h"

namespace century {
namespace perception {
namespace common {
namespace visualizer {

const std::map<int, std::string> kObjectTypeIntToStringMap = {
    {-1, "EGO"},
    {0, "UNKNOWN"},
    {1, "UNKNOWN_MOVABLE"},
    {2, "UNKNOWN_UNMOVABLE"},
    {3, "PEDESTRIAN"},
    {4, "BICYCLE"},
    {5, "VEHICLE"},
    {6, "NARROW20FOOT"},
    {7, "DUMMY"},
    {8, "CONE"},
    {9, "STACKER"},
    {10, "FORKLIFT_STACKER"},
    {11, "WHEELCRANE"}};

struct PointXYZI {
  float x, y, z;
  float intensity;
};

struct Pose {
  Eigen::Vector3f t;
  Eigen::Quaternionf q;  // rotation
};

struct BBox {
  // center, dimensions, yaw (around Z) or full rotation quaternion
  Eigen::Vector3f center;
  Eigen::Vector3f size;  // l, w, h (or x,y,z extents)
  Eigen::Quaternionf q;
  int id;
};

using MapInfo = std::vector<std::vector<PointXYZI>>;
using PolygonVector = std::vector<std::vector<PointXYZI>>;

// Thread-safe shared state
class SharedScene {
 public:
  // Point cloud (packed as x,y,z,intensity)
  std::vector<PointXYZI> cloud_;
  Pose pose_;
  std::vector<BBox> bboxes_;
  MapInfo maps_info_;
  PolygonVector polygons_;

  std::mutex m_;
  std::atomic<bool> has_update_{false};
};

// Main viewer class encapsulating VBO management
class PangolinViewer {
 public:
  PangolinViewer(const int& width, const int& height, bool aeb_dis_viz,
                 bool grid_viz);

  ~PangolinViewer();

  void PushPointCloud(const std::vector<PointXYZI>& pts);
  void PushPose(const Pose& p);
  void PushBBoxes(const std::vector<BBox>& boxes);
  void PushPolygons(const PolygonVector& polygons);
  void PushMap(const MapInfo& map_info);

  // Main loop (blocks)
  void Spin();

 private:
  void UploadToGPU();

  void DrawPointsGL();

  void DrawGrid();

  void DrawEgoBBOX(const Eigen::Quaternionf& q);
  void DrawBBox(const BBox& bb);

  void DrawPose(const Eigen::Vector3f& t, const Eigen::Quaternionf& q,
                float axis_scale = 1.0f);
  void DrawMap(MapInfo& maps_info);
  void DrawPolygons(PolygonVector& polygons);
  void DrawPolygonDistances(PolygonVector& polygons);
  void CalculatePolygonToEgoDistances(
      const PolygonVector& polygons,
      std::vector<std::pair<float, float>>& distances,
      std::vector<PointXYZI>& nearest_points);
  int window_width_, window_height_;
  pangolin::OpenGlRenderState s_cam_;
  pangolin::View* d_cam_;
  GLuint vbo_;
  size_t num_points_;
  std::vector<PointXYZI> points_local_;
  Pose pose_local_;
  std::vector<BBox> bboxes_local_;
  MapInfo maps_info_local_;
  PolygonVector polygons_local_;
  SharedScene scene_;
  bool aeb_dis_viz_{false};
  bool grid_viz_{false};
};

std::vector<PointXYZI> makeDummyCloud(float t);
std::vector<BBox> makeDummyBoxes(float t);

}  // namespace visualizer
}  // namespace common
}  // namespace perception
}  // namespace century