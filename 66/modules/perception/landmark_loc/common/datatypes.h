//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>

#define MapLayerSize 7UL
namespace semantic_mapping {
typedef pcl::PointXYZL PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

class GridIndex {
 public:
  int x = 0;
  int y = 0;

  GridIndex() {
    x = 0;
    y = 0;
  }
  GridIndex(int x_, int y_) {
    x = x_;
    y = y_;
  }

  void SetIndex(int x_, int y_) {
    x = x_;
    y = y_;
  }
};

typedef enum POINT_LABEL: uint32_t {
  Num0 = 0,
  Num1,
  Num2,
  Num3,
  Num4,
  Num5,
  Num6,
  Num7,
  Num8,
  Num9,
  Slotline,
  Boundaryline,
  Arrow,
  Bump,
  ManholeCover,
  CircleCover,
  Init
} PointLabel;

class GridIndexLabel {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GridIndexLabel(){
    grid_id=GridIndex(0,0);
    }
    GridIndexLabel(Eigen::Vector3d pm_, GridIndex grid_id_,u_char r_,u_char g_,u_char b_):pm(pm_),grid_id(grid_id_),
   r(r_),g(g_),b(b_){}

   PointLabel point_label = Init;
   Eigen::Vector3d pm = Eigen::Vector3d::Zero();
   bool is_isTrust_flag = false;
   GridIndex grid_id;

   #if 0
   u_char r = 255;
   u_char g = 255; //background -white
   u_char b = 255;
   #endif

   #if 1
   u_char r = 0;
   u_char g = 0; //background -black
   u_char b = 0;
   #endif

   double timestamp = 0.0;
};


class EigenPose {
 public:
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_pitch_angle = Eigen::Quaterniond::Identity();
  bool has_pitch_angle = false;
  void set_q_pitch_angle(const Eigen::Quaterniond &q_pitch_angle_) {
    q_pitch_angle = q_pitch_angle_;
    has_pitch_angle = true;
  }
  EigenPose() {
    q = Eigen::Quaterniond::Identity();
    t = Eigen::Vector3d::Zero();
    q_pitch_angle = Eigen::Quaterniond::Identity();
    has_pitch_angle = false;
  }
  EigenPose(const Eigen::Quaterniond &q_, const Eigen::Vector3d &t_) {
    q = q_;
    t = t_;
    q_pitch_angle = Eigen::Quaterniond::Identity();
    has_pitch_angle = false;
  }
  Eigen::Matrix4d toMatrix() {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = q.toRotationMatrix();
    mat.block<3, 1>(0, 3) = t;

    return mat;
  }
};



template<class T>
struct point {
  inline point() : x(0), y(0) {}
  inline point(T _x, T _y) : x(_x), y(_y) {}
  T x, y;
};
template<class T>
inline point<T> operator+(const point<T> &p1, const point<T> &p2) {
  return point<T>(p1.x + p2.x, p1.y + p2.y);
}
template<class T>
inline point<T> operator-(const point<T> &p1, const point<T> &p2) {
  return point<T>(p1.x - p2.x, p1.y - p2.y);
}
template<class T>
inline point<T> operator*(const point<T> &p, const T &v) {
  return point<T>(p.x * v, p.y * v);
}
template<class T>
inline point<T> operator*(const T &v, const point<T> &p) {
  return point<T>(p.x * v, p.y * v);
}
template<class T>
inline T operator*(const point<T> &p1, const point<T> &p2) {
  return p1.x * p2.x + p1.y * p2.y;
}
template<class T>
struct pointcomparator {
  bool operator()(const point<T> &a, const point<T> &b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};
template<class T>
struct pointradialcomparator {
  point<T> origin;
  bool operator()(const point<T> &a, const point<T> &b) const {
    point<T> delta1 = a - origin;
    point<T> delta2 = b - origin;
    return (atan2(delta1.y, delta1.x) < atan2(delta2.y, delta2.x));
  }
};
template<class T>
inline point<T> max(const point<T> &p1, const point<T> &p2) {
  point<T> p = p1;
  p.x = p.x > p2.x ? p.x : p2.x;
  p.y = p.y > p2.y ? p.y : p2.y;
  return p;
}
template<class T>
inline point<T> min(const point<T> &p1, const point<T> &p2) {
  point<T> p = p1;
  p.x = p.x < p2.x ? p.x : p2.x;
  p.y = p.y < p2.y ? p.y : p2.y;
  return p;
}
template<class T, class F>
inline point<T> interpolate(const point<T> &p1, const F &t1, const point<T> &p2,
                            const F &t2, const F &t3) {
  F gain = (t3 - t1) / (t2 - t1);
  point<T> p = p1 + (p2 - p1) * gain;
  return p;
}
template<class T>
inline double distance(const point<T> &p1, const point<T> &p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

typedef point<int> Point2i;
typedef point<float> Point2f;
typedef point<double> Point2d;


// semantic + pointcloud + timestamp + pose
// sensordata is accumulated and synced
class ComposedSensorData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ComposedSensorData() =default;
  ~ComposedSensorData() =default;
  explicit ComposedSensorData(double timestamp_, const Eigen::Matrix4d &pose_odom_,
                             PointCloudPtr pointcloud_data_ = nullptr)
      : timestamp(timestamp_), pose_odom(pose_odom_), pointcloud_data(pointcloud_data_) {}
  //timestamp+others
  double timestamp = 0.0;

  //pose
  Eigen::Matrix4d pose_odom = Eigen::Matrix4d::Identity();

  //sensor
  PointCloudPtr pointcloud_data;
};

struct PoseWithConvariance {
  Eigen::Quaterniond rotation;
  Eigen::Vector3d position;
  double covariance[36];
};

enum LidarType : uint32_t {
  HeliosFrontLeft = 0,
  HeliosRearRight,
  BpFrontLeft,
  BpRearRight,
  LidarTypeSize
};


enum CameraType : uint32_t {
  CamFrontLeft = 0, 
  CamFrontMiddle,
  CamFrontRight,
  CamRearRight, 
  CamRearMiddle, 
  CamRearLeft,
  CamTypeSize
};

struct CameraParmeter {
  Eigen::Quaterniond camera_q[CamTypeSize];
  Eigen::Vector3d camera_t[CamTypeSize];
  cv::Mat camera_intrinsic[CamTypeSize];
  cv::Mat camera_distort[CamTypeSize];
  void Debug() {
    for(int i=0; i<CamTypeSize; i++) {
      std::cout << "CameraType: " << i << std::endl;
      std::cout << "camera_q: " << camera_q[i].w() << " " << camera_q[i].x() << " " << camera_q[i].y() << " " << camera_q[i].z() << std::endl;
      std::cout << "camera_t: " << camera_t[i].x() << " " << camera_t[i].y() << " " << camera_t[i].z() << std::endl;
      std::cout << "camera_intrinsic:  " << camera_intrinsic[i] << std::endl;
      std::cout << "camera_distort:  " << camera_distort[i] << std::endl;
    }

  }
};


}

