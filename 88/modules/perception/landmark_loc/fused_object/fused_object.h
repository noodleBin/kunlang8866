// Created by xiaxinrong on 2025/8/15.
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_map>
#include "common/datatypes.h"
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


namespace semantic_mapping {

class FusedObjectGenerator{
public:
  FusedObjectGenerator() = default ;
  ~FusedObjectGenerator() = default;
  bool Init(const std::vector<CameraType>& names,
            const std::string& extrinsic_folder, const bool filter_distance,
            const bool vis=false);
  bool Init(const std::unordered_set<CameraType>&  camera_types_register,
            const CameraParmeter cam_parameter, const bool filter_distance,
            const bool vis = false) ;

  
  
  pcl::PointCloud<pcl::PointXYZL>::Ptr RollOut(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud, 
                      const std::unordered_map<CameraType, cv::Mat>& segmentations, const std::string& ts_str="");             

private:
  bool SemanticFilter(const cv::Point3f lidat_pt, const cv::Point2f pt, const CameraType& camera_type, 
                      const uchar label, const int width, const int height, const double ts); // 255-semantic
  std::string original_image_folder_;
  std::unordered_set<CameraType>  camera_types_register_;
  Eigen::Quaterniond camera_q_[CamTypeSize];
  Eigen::Vector3d camera_t_[CamTypeSize];
  cv::Mat camera_intrinsic_[CamTypeSize];
  cv::Mat camera_distort_[CamTypeSize];
  bool vis_;
  bool filter_distance_ = false;


};

}