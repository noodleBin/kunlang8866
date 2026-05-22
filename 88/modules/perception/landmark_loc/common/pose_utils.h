//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "common/eigen_types.h"
#include "common/datatypes.h"

namespace semantic_mapping {
// Eigen::Matrix4d <-> SE2

class PoseUtils {
 public:
  static SE3 Eigen3DToSophus3D(const Eigen::Matrix4d &pose_3d_eigen) {
      SE3 pose_3d_sophus;
      Eigen::Matrix3d R = pose_3d_eigen.block<3, 3>(0, 0);
      Eigen::Vector3d t = pose_3d_eigen.block<3, 1>(0, 3);
      pose_3d_sophus = SE3(R, t);
      return pose_3d_sophus;
  }
  static SE2 Project2D(const SE3 &pose_3d) {
      SE2 pose_2d;
      pose_2d.translation() = Eigen::Vector2d(pose_3d.translation().x(), pose_3d.translation().y());
      pose_2d.so2() = Sophus::SO2d::exp(pose_3d.so3().log().z());
      return pose_2d;
  }
  static SE3 Embed3D(const SE2 &pose_2d) {
      SE3 pose_3d;
      pose_3d.translation() = Eigen::Vector3d(pose_2d.translation().x(), pose_2d.translation().y(), 0.0);
      pose_3d.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, pose_2d.so2().log()));
      return pose_3d;
  }
  static SE2 EigenMatToSE2(const Eigen::Matrix4d &T) {
      SE3 pose_3d_sophus = Eigen3DToSophus3D(T);
      SE2 pose_2d_sophus = Project2D(pose_3d_sophus);
      return pose_2d_sophus;
  }
  static Eigen::Matrix4d SE2ToEigenMat(const SE2 &pose_se2) {
      SE3 pose_3d_sophus = Embed3D(pose_se2);
      Eigen::Matrix4d T = pose_3d_sophus.matrix();
      return T;
  }

  static Eigen::Matrix4d toEigenMat(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = t[0];
    T(1, 3) = t[1];
    T(2, 3) = t[2];
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
  }
  static void convertEigenMat(const Eigen::Matrix4d &T, Eigen::Vector3d &t, Eigen::Quaterniond &q) {
    t = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(R);
    // q = R;
    return;
  }
  // Eigen::Vector3d[x, y, yaw]
  static Eigen::Vector3d EigenMatToVector3d(const Eigen::Matrix4d &T) {
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    PoseUtils::convertEigenMat(T, t, q);
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    quat_to_euler_zyx(q, yaw, pitch, roll);
    return Eigen::Vector3d(t[0], t[1], yaw);
  }
  static Eigen::Matrix4d Vector3dToEigenMat(const Eigen::Vector3d &pose_2d) {
    Eigen::Vector3d t(pose_2d.x(), pose_2d.y(), 0.0);
    Eigen::Quaterniond q = zyx_euler_to_quat(pose_2d.z(), 0.0, 0.0);
    return toEigenMat(t, q);
  }

  static cv::Mat toCvMat(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
    cv::Mat mat;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T(0, 3) = t[0];
    T(1, 3) = t[1];
    T(2, 3) = t[2];
    cv::eigen2cv(T, mat);
    mat.convertTo(mat, CV_32F);
    return mat;
  }

  static void quat_to_euler_zyx(const Eigen::Quaterniond &q, double &yaw, double &pitch,
                                double &roll) {
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();

    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    // type-2
    // Eigen::Vector3d eulerAngles = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return;
  }
  static Eigen::Quaterniond zyx_euler_to_quat(const double &yaw, const double &pitch,
                                              const double &roll) {
    double sy = sin(yaw * 0.5);
    double cy = cos(yaw * 0.5);
    double sp = sin(pitch * 0.5);
    double cp = cos(pitch * 0.5);
    double sr = sin(roll * 0.5);
    double cr = cos(roll * 0.5);
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;

    return Eigen::Quaterniond(w, x, y, z);
  }

  static void EigenPoseToMatrix(const EigenPose &pose, Eigen::Matrix4d &mat) {
    mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
    mat.block<3, 1>(0, 3) = pose.t;
  }

  static void PointToEigenVector3d(const PointType &p, Eigen::Vector3d &v) {
    v.x() = double(p.x);
    v.y() = double(p.y);
    v.z() = double(p.z);
  }

  static void EigenVector3dToPoint(const Eigen::Vector3d &v, PointType &p) {
    p.x = float(v.x());
    p.y = float(v.y());
    p.z = float(v.z());
  }

  static void TransformPoint(const Eigen::Matrix4d &pose,
                             const PointType &p_in, PointType &p_out) {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    PointToEigenVector3d(p_in, p);
    Eigen::Vector3d p_w = pose.block<3, 3>(0, 0) * p +
        pose.block<3, 1>(0, 3);  // to world frame
    EigenVector3dToPoint(p_w, p_out);
    p_out.label = p_in.label;
  }


};
}


