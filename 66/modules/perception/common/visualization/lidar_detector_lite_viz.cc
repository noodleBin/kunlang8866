#include "modules/perception/common/visualization/lidar_detector_lite_viz.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace century {
namespace perception {
namespace common {
namespace visualizer {

PangolinViewer::PangolinViewer(const int& width, const int& height,
                               bool aeb_dis_viz, bool grid_viz)
    : window_width_(width),
      window_height_(height),
      aeb_dis_viz_(aeb_dis_viz),
      grid_viz_(grid_viz) {
  pangolin::CreateWindowAndBind("Pangolin Viewer", window_width_,
                                window_height_);
  glEnable(GL_DEPTH_TEST);
  glewExperimental = GL_TRUE;
  glEnable(GL_MULTISAMPLE);
  GLenum err = glewInit();
  if (err != GLEW_OK) { /* initialization issue handled upstream */
    std::cerr << "Error initializing GLEW: " << glewGetErrorString(err)
          << std::endl;
  }
  s_cam_ = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(window_width_, window_height_, 420, 420,
                                 window_width_ / 2.0, window_height_ / 2.0, 0.1,
                                 1000),
      pangolin::ModelViewLookAt(0, -10, 5, 0, 0, 0, 0, 0, 1));
  d_cam_ = &pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0)
                .SetHandler(new pangolin::Handler3D(s_cam_));
  glGenBuffers(1, &vbo_);
  num_points_ = 0;
}

PangolinViewer::~PangolinViewer() { glDeleteBuffers(1, &vbo_); }

void PangolinViewer::Spin() {
  while (!pangolin::ShouldQuit()) {
    bool updated = false;
    {
      std::lock_guard<std::mutex> lg(scene_.m_);
      if (scene_.has_update_) {
        points_local_ = scene_.cloud_;
        pose_local_ = scene_.pose_;
        bboxes_local_ = scene_.bboxes_;
        maps_info_local_ = scene_.maps_info_;
        polygons_local_ = scene_.polygons_;
        scene_.has_update_ = false;
        updated = true;
      }
    }
    if (updated) {
      UploadToGPU();
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    d_cam_->Activate(s_cam_);
    if (grid_viz_) {
      DrawGrid();
    }
    DrawPointsGL();
    DrawPose(pose_local_.t, pose_local_.q, 1.0f);
    DrawEgoBBOX(pose_local_.q);
    DrawPolygons(polygons_local_);
    for (auto& bb : bboxes_local_) {
      DrawBBox(bb);
    }

    if (aeb_dis_viz_) {
      DrawPolygonDistances(polygons_local_);
    }

    DrawMap(maps_info_local_);
    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void PangolinViewer::UploadToGPU() {
  num_points_ = points_local_.size();
  if (0 == num_points_) return;
  std::vector<float> buf;
  buf.reserve(num_points_ * 6);
  for (auto& p : points_local_) {
    buf.emplace_back(p.x);
    buf.emplace_back(p.y);
    buf.emplace_back(p.z);
    if (p.intensity > 0.5f) {
      buf.emplace_back(0.0f);
      buf.emplace_back(1.0f);
      buf.emplace_back(0.0f);
    } else {
      buf.emplace_back(1.0f);
      buf.emplace_back(0.0f);
      buf.emplace_back(0.0f);
    }
  }
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, buf.size() * sizeof(float), buf.data(),
               GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PangolinViewer::PushPointCloud(const std::vector<PointXYZI>& pts) {
  std::lock_guard<std::mutex> lg(scene_.m_);
  scene_.cloud_ = pts;
}

void PangolinViewer::PushPose(const Pose& p) {
  std::lock_guard<std::mutex> lg(scene_.m_);
  scene_.pose_ = p;
}

void PangolinViewer::PushBBoxes(const std::vector<BBox>& boxes) {
  std::lock_guard<std::mutex> lg(scene_.m_);
  scene_.bboxes_ = boxes;
}

void PangolinViewer::PushPolygons(const PolygonVector& polygons) {
  std::lock_guard<std::mutex> lg(scene_.m_);
  scene_.polygons_.clear();
  scene_.polygons_ = polygons;
}

void PangolinViewer::PushMap(const MapInfo& map_info) {
  std::lock_guard<std::mutex> lg(scene_.m_);
  scene_.maps_info_.clear();
  scene_.maps_info_.insert(scene_.maps_info_.end(), map_info.begin(),
                           map_info.end());
  scene_.has_update_ = true;
}

void PangolinViewer::DrawPointsGL() {
  if (0 == num_points_) return;
  glPointSize(2.0f);
  glEnableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableClientState(GL_COLOR_ARRAY);
  glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), (void*)0);
  glColorPointer(3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));
  glDrawArrays(GL_POINTS, 0, (GLsizei)num_points_);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
}

void PangolinViewer::DrawGrid() {
  const float resolution = 0.2f;
  const int range = 100;
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  glColor3f(0.4f, 0.4f, 0.4f);
  Eigen::Matrix3f R = pose_local_.q.toRotationMatrix();
  for (int i = -range; i <= range; ++i) {
    float d = i * resolution;
    Eigen::Vector3f p1 = R * Eigen::Vector3f(-range * resolution, d, 0.0f);
    Eigen::Vector3f p2 = R * Eigen::Vector3f(range * resolution, d, 0.0f);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glVertex3f(p2.x(), p2.y(), p2.z());
    Eigen::Vector3f p3 = R * Eigen::Vector3f(d, -range * resolution, 0.0f);
    Eigen::Vector3f p4 = R * Eigen::Vector3f(d, range * resolution, 0.0f);
    glVertex3f(p3.x(), p3.y(), p3.z());
    glVertex3f(p4.x(), p4.y(), p4.z());
  }
  glEnd();
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  Eigen::Vector3f origin = pose_local_.t;
  Eigen::Vector3f x_axis = R * Eigen::Vector3f(5.0f, 0.0f, 0.0f) + origin;
  Eigen::Vector3f y_axis = R * Eigen::Vector3f(0.0f, 5.0f, 0.0f) + origin;
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(origin.x(), origin.y(), origin.z());
  glVertex3f(x_axis.x(), x_axis.y(), x_axis.z());
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(origin.x(), origin.y(), origin.z());
  glVertex3f(y_axis.x(), y_axis.y(), y_axis.z());
  glEnd();
}

void PangolinViewer::CalculatePolygonToEgoDistances(
    const PolygonVector& polygons,
    std::vector<std::pair<float, float>>& distances,
    std::vector<PointXYZI>& nearest_points) {
  distances.clear();
  nearest_points.clear();
  const float ego_length = 15.4f;
  const float ego_width = 3.2f;
  const float half_length = ego_length * 0.5f;
  const float half_width = ego_width * 0.5f;
  Eigen::Matrix3f R_inv = pose_local_.q.toRotationMatrix().transpose();

  for (const auto& polygon : polygons) {
    if (polygon.empty()) {
      continue;
    }

    Eigen::Vector3f center(0, 0, 0);
    for (const auto& point : polygon) {
      center += Eigen::Vector3f(point.x, point.y, point.z);
    }
    center /= polygon.size();
    Eigen::Vector3f center_local = R_inv * (center - pose_local_.t);

    bool polygon_in_front_or_back =
        (center_local.x() >= -half_length) && (center_local.x() <= half_length);

    float min_dist = std::numeric_limits<float>::max();
    float min_secondary_dist = std::numeric_limits<float>::max();
    PointXYZI nearest_point;
    float nearest_lon_dist = 0.0f;
    float nearest_lat_dist = 0.0f;

    for (const auto& point : polygon) {
      Eigen::Vector3f pt(point.x, point.y, point.z);
      Eigen::Vector3f pt_local = R_inv * (pt - pose_local_.t);

      float lon_dist = 0.0f;
      if (pt_local.x() > half_length) {
        lon_dist = pt_local.x() - half_length;
      } else if (pt_local.x() < -half_length) {
        lon_dist = pt_local.x() + half_length;
      }

      float lat_dist = 0.0f;
      if (pt_local.y() > half_width) {
        lat_dist = pt_local.y() - half_width;
      } else if (pt_local.y() < -half_width) {
        lat_dist = pt_local.y() + half_width;
      }

      float compare_dist;
      float secondary_dist;
      if (polygon_in_front_or_back) {
        compare_dist = std::abs(lon_dist);
        secondary_dist = std::abs(lat_dist);
      } else {
        compare_dist = std::abs(lat_dist);
        secondary_dist = std::abs(lon_dist);
      }

      if (compare_dist < min_dist - 1e-6f ||
          (std::abs(compare_dist - min_dist) < 1e-6f &&
           secondary_dist < min_secondary_dist)) {
        min_dist = compare_dist;
        min_secondary_dist = secondary_dist;
        nearest_point = point;
        nearest_lon_dist = lon_dist;
        nearest_lat_dist = lat_dist;
      }
    }

    distances.emplace_back(nearest_lon_dist, nearest_lat_dist);
    nearest_points.emplace_back(nearest_point);
  }
}

void PangolinViewer::DrawPolygonDistances(PolygonVector& polygons) {
  std::vector<std::pair<float, float>> nearest_distances;
  std::vector<PointXYZI> nearest_points;
  CalculatePolygonToEgoDistances(polygons, nearest_distances, nearest_points);

  pangolin::GlFont& font = pangolin::default_font();

  for (size_t i = 0; i < nearest_points.size() && i < nearest_distances.size();
       ++i) {
    const auto& point = nearest_points[i];
    const auto& dist = nearest_distances[i];

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "X:" << dist.first << " Y:" << dist.second;
    glColor3f(1.0f, 1.0f, 0.0f);
    font.Text(ss.str()).Draw(point.x, point.y, point.z + 0.5f);

    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(15.0f);
    glBegin(GL_POINTS);
    glVertex3f(point.x, point.y, point.z);
    glEnd();
  }
}

void PangolinViewer::DrawEgoBBOX(const Eigen::Quaternionf& q) {
  const float vehicle_length = 15.4f;
  const float vehicle_width = 3.2f;
  const float vehicle_height = 1.8f;
  BBox box;
  box.center << 0, 0, vehicle_height / 2;
  box.size << vehicle_length, vehicle_width, vehicle_height;
  box.q = q;
  box.id = -1;
  DrawBBox(box);
}

void PangolinViewer::DrawBBox(const BBox& bb) {
  Eigen::Vector3f c = bb.center;
  Eigen::Vector3f s = bb.size * 0.5f;
  Eigen::Matrix3f R = bb.q.toRotationMatrix();
  Eigen::Vector3f corners[8] = {
      {s.x(), s.y(), s.z()},   {s.x(), s.y(), -s.z()},  {s.x(), -s.y(), s.z()},
      {s.x(), -s.y(), -s.z()}, {-s.x(), s.y(), s.z()},  {-s.x(), s.y(), -s.z()},
      {-s.x(), -s.y(), s.z()}, {-s.x(), -s.y(), -s.z()}};
  Eigen::Vector3f wc[8];
  for (int i = 0; i < 8; ++i) {
    wc[i] = R * corners[i] + c;
  }
  const int edges[12][2] = {{0, 1}, {0, 2}, {0, 4}, {7, 5}, {7, 6}, {7, 3},
                            {1, 3}, {1, 5}, {2, 3}, {2, 6}, {4, 5}, {4, 6}};
  float color[3] = {1.0f, 0.0f, 0.0f};
  glColor3f(color[0], color[1], color[2]);
  glBegin(GL_LINES);
  for (int e = 0; e < 12; e++) {
    auto point_a = wc[edges[e][0]];
    auto point_b = wc[edges[e][1]];
    glVertex3f(point_a.x(), point_a.y(), point_a.z());
    glVertex3f(point_b.x(), point_b.y(), point_b.z());
  }
  glEnd();
  pangolin::GlFont& font = pangolin::default_font();
  constexpr float line_height = 0.3f;
  float x = c(0);
  float y = c(1);
  float z = c(2) + s.z() + line_height;
  glColor3f(1.0f, 1.0f, 1.0f);
  const std::string text_type = kObjectTypeIntToStringMap.at(bb.id);
  float distance = std::sqrt(c.x() * c.x() + c.y() * c.y() + c.z() * c.z());
  std::stringstream ss;
  ss << text_type << " ";
  std::string text = ss.str();
  font.Text(text).Draw(x, y, z);
  char dist_str[32];
  snprintf(dist_str, sizeof(dist_str), "%.1fm", distance);
  font.Text(std::string(dist_str)).Draw(x - line_height, y, z);
}

void PangolinViewer::DrawPose(const Eigen::Vector3f& t,
                              const Eigen::Quaternionf& q, float axis_scale) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T.block<3, 1>(0, 3) = t;
  glPushMatrix();
  glMultMatrixf(T.data());
  glBegin(GL_LINES);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(axis_scale, 0, 0);
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, axis_scale, 0);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, axis_scale);
  glEnd();
  glPopMatrix();
}

void PangolinViewer::DrawPolygons(PolygonVector& polygons) {
  glColor3f(1.0f, 0.0f, 0.0f);
  glLineWidth(2.0f);
  for (const auto& polygon : polygons) {
    if (polygon.size() < 2) {
      continue;
    }
    glBegin(GL_LINE_LOOP);
    for (const auto& point : polygon) {
      glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    float cx = 0, cy = 0, cz = 0;
    for (const auto& point : polygon) {
      cx += point.x;
      cy += point.y;
      cz += point.z;
    }
    cx /= polygon.size();
    cy /= polygon.size();
    cz /= polygon.size();

    pangolin::GlFont& font = pangolin::default_font();
    glColor3f(1.0f, 1.0f, 1.0f);
    const std::string text_type = kObjectTypeIntToStringMap.at(0);
    font.Text(text_type).Draw(cx, cy, cz + 0.5f);
  }
}

void PangolinViewer::DrawMap(MapInfo& maps_info) {
  glColor3f(0.0f, 0.0f, 1.0f);
  glLineWidth(2.0f);
  for (const auto& polygon : maps_info) {
    if (polygon.size() < 2) {
      continue;
    }
    glBegin(GL_LINE_LOOP);
    for (const auto& point : polygon) {
      glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
  }
}

std::vector<PointXYZI> makeDummyCloud(float t) {
  std::vector<PointXYZI> v;
  int N = 20000;
  v.reserve(N);
  for (int i = 0; i < N; ++i) {
    float a = (float)rand() / RAND_MAX * 2.0f * M_PI;
    float r = 5.0f * (float)rand() / RAND_MAX + 0.5f;
    PointXYZI p;
    p.x = r * cosf(a) + 0.5f * sinf(t * 0.5f + i * 0.0001f);
    p.y = r * sinf(a) + 0.5f * cosf(t * 0.3f + i * 0.0002f);
    p.z = ((float)rand() / RAND_MAX - 0.5f) * 1.5f;
    p.intensity = (float)rand() / RAND_MAX;
    v.emplace_back(p);
  }
  return v;
}

std::vector<BBox> makeDummyBoxes(float t) {
  std::vector<BBox> out;
  for (int i = 0; i < 5; ++i) {
    BBox b;
    b.center = Eigen::Vector3f((float)i * 2.0f - 4.0f + sinf(t * 0.3f + i),
                               cosf(t * 0.2f + i) * 1.0f, 0.5f);
    b.size = Eigen::Vector3f(1.6f, 0.8f, 1.2f);
    Eigen::AngleAxisf aa(t * 0.2f + i * 0.3f, Eigen::Vector3f::UnitZ());
    b.q = Eigen::Quaternionf(aa);
    b.id = i;
    out.emplace_back(b);
  }
  return out;
}

}  // namespace visualizer
}  // namespace common
}  // namespace perception
}  // namespace century
