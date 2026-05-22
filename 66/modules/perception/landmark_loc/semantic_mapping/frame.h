//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include "common/eigen_types.h"

#include "common/pose_utils.h"
#include "common/datatypes.h"
namespace semantic_mapping {
/**
 * one frame of scan
 */
class Frame {
   public:
    Frame(const ComposedSensorData& sensor_data):timestamp_(sensor_data.timestamp),
                                            point_cloud_(sensor_data.pointcloud_data),
                                            pose_odom_(sensor_data.pose_odom) {

    }
   
    void set_id(std::size_t id) { id_ = id; }
    void set_keyframe_id(std::size_t keyframe_id) { keyframe_id_ = keyframe_id; }
    std::size_t id() const { return id_; }
    std::size_t keyframe_id() const { return keyframe_id_; }
    double timestamp() const { return timestamp_; }
    void set_pose_odom(const Eigen::Matrix4d &pose_odom) { pose_odom_ = pose_odom; }
    void set_pose_map(const Eigen::Matrix4d &pose_map) { pose_map_ = pose_map; }
    // void set_pose_submap(const Eigen::Matrix4d &pose_submap) { pose_submap_ = pose_submap; }
    Eigen::Matrix4d pose_odom() const { return pose_odom_; }
    Eigen::Matrix4d pose_map() const { return pose_map_; }
    // Eigen::Matrix4d pose_submap() const { return pose_submap_; }
    void set_pose_map_SE2(const SE2 &pose_map_SE2) {
        pose_map_ = PoseUtils::SE2ToEigenMat(pose_map_SE2);
    }
    SE2 pose_odom_SE2() const {
        return PoseUtils::EigenMatToSE2(pose_odom_);
    }
    SE2 pose_map_SE2() const {
        return PoseUtils::EigenMatToSE2(pose_map_);
    }

    PointCloudPtr scan() const {
        return point_cloud_;
    }
    // SE2 pose_submap_SE2() const {
    //     return PoseUtils::EigenMatToSE2(pose_submap_);
    // }

   private:
    size_t id_ = 0;               // scan id
    size_t keyframe_id_ = 0;      // keyframe id
    double timestamp_ = 0.0;        // timestamp
    PointCloudPtr point_cloud_;  // senaor data
    Eigen::Matrix4d pose_odom_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_map_ = Eigen::Matrix4d::Identity();   // pose，world to scan, T_w_c
    // Eigen::Matrix4d pose_submap_ = Eigen::Matrix4d::Identity();  // pose，submap to scan, T_s_c
};

}  // namespace semantic_mapping


