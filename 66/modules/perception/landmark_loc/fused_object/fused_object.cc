// Created by xiaxinrong on 2025/8/15.
#include "fused_object.h"
#include "data_adapter.h"
#include <iostream>
#include <algorithm> 
namespace semantic_mapping {
namespace {
  // Define the camera names as a constant array
const std::array<std::string, 6> camera_names = {"FrontLeft", "FrontMiddle", "FrontRight",
      "RearRight", "RearMiddle", "RearLeft"};
const float min_depth = 1.5;
const float max_depth = 30;
}

bool FusedObjectGenerator::Init(const std::unordered_set<CameraType>&  camera_types_register,
                                const CameraParmeter cam_parameter, const bool filter_distance,
                                const bool vis) {
  filter_distance_ = filter_distance;
  camera_types_register_ = camera_types_register;
  std::copy(cam_parameter.camera_q, cam_parameter.camera_q + CamTypeSize, camera_q_);
  std::copy(cam_parameter.camera_t, cam_parameter.camera_t + CamTypeSize, camera_t_);
  std::copy(cam_parameter.camera_intrinsic, cam_parameter.camera_intrinsic + CamTypeSize, camera_intrinsic_);
  std::copy(cam_parameter.camera_distort, cam_parameter.camera_distort + CamTypeSize, camera_distort_);
  vis_ = vis;
  return true;
}

bool FusedObjectGenerator::Init(const std::vector<CameraType>& names,
                                const std::string& parameter_folder, const bool filter_distance,
                                const bool vis){
  filter_distance_ = filter_distance;
  for(auto& iter : names){
    if(iter > CamTypeSize){
      std::cerr << "camera name is invalid" << std::endl;
      return false;
    }
    auto camera_name = camera_names[iter];
    std::vector<float> res = readExtrinsic(parameter_folder + camera_name + "_transform.pb.txt");
    if(res.size() < 7){
      return false;
    }
    std::map<std::string, float> intrinsic = readIntrinsic(parameter_folder + camera_name + "_param.xml");
    if(intrinsic.size() < 5){
      return false;
    }
    camera_q_[iter] = Eigen::Quaterniond(res[6], res[3], res[4], res[5]);;
    camera_t_[iter] =  Eigen::Vector3d(res[0], res[1], res[2]);
    camera_intrinsic_[iter] = (cv::Mat_<double>(3, 3) << intrinsic.at("fx"), 0.0, intrinsic.at("cx"), 0.0, intrinsic.at("fy"), intrinsic.at("cy"), 0.0, 0.0, 1.0);
    camera_distort_[iter] = (cv::Mat_<double>(4, 1) << intrinsic.at("k1"), intrinsic.at("k2"), intrinsic.at("k3"), intrinsic.at("k4"));
    camera_types_register_.insert(iter);
  }
  original_image_folder_ = parameter_folder + "../original_image/";
  LOG(INFO) << "fused object generator init success "<< original_image_folder_;
  vis_ = vis;

  return true;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr FusedObjectGenerator::RollOut(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud, 
                                                                   const std::unordered_map<CameraType, cv::Mat>& segmentations,
                                                                   const std::string& ts_str) {
  auto start = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZL>::Ptr out_result = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
  for(const auto& iter: segmentations){
   
    if(camera_types_register_.find(iter.first) == camera_types_register_.end()){
      std::cerr << "camera " << iter.first << " not found" << std::endl;
      return nullptr;
    }
    
    Eigen::Matrix4d camera_tf = Eigen::Matrix4d::Identity();
    camera_tf.block<3,3>(0,0) = camera_q_[iter.first].toRotationMatrix();
    camera_tf.block<3,1>(0,3) = camera_t_[iter.first];
    Eigen::Matrix4d Tx_C_L = camera_tf.inverse();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pcd =
    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*lidar_cloud, *transformed_pcd, Tx_C_L);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filted_pcd =
    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> lidar_index_;
    
    for (int i=0 ;i< transformed_pcd->size(); ++i) {
      if (transformed_pcd->points.at(i).z < 0) {
        continue;
      }
      filted_pcd->push_back(transformed_pcd->points.at(i));
      lidar_index_.push_back(i);
    }

    std::vector<int> target_index_;
    std::vector<cv::Point3f> pts_3d;

    for (int i=0 ;i< filted_pcd->size(); ++i) {
      const auto &p = filted_pcd->points.at(i);
      float depth = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
      if (depth > min_depth && depth < max_depth) {
        pts_3d.emplace_back(cv::Point3f(p.x, p.y, p.z));
        target_index_.push_back(i);
      }
    }
    //LOG(INFO) <<"project point size: "<< pts_3d.size() << std::endl;
    std::vector<cv::Point2f> pts_2d;
    cv::fisheye::projectPoints(pts_3d, pts_2d, cv::Vec3d(0,0,0), cv::Vec3d(0, 0, 0), camera_intrinsic_[iter.first],
                              camera_distort_[iter.first]);

    //cv::Mat image = iter.second.clone();
    cv::Mat* image = nullptr;
    if((vis_) && (iter.first == CamFrontRight)) {
      image = new cv::Mat(iter.second.rows, iter.second.cols, CV_8UC1, cv::Scalar(0));
      auto file_path = original_image_folder_ + camera_names.at(static_cast<uint32_t>(iter.first)) + "/" + ts_str + ".jpg";
      LOG(INFO) << "read image from : " << file_path;

      *image = cv::imread(file_path, cv::IMREAD_COLOR);
    }

    for (int i=0 ; i< pts_2d.size(); i++) {
      cv::Point2f& pt = pts_2d[i];
      const auto& lidar_pt = pts_3d[i];
      if (pt.x < 0 || pt.x >= iter.second.cols ||
          pt.y < 0 || pt.y >= iter.second.rows) {
        continue;
      }
      
      uchar label = iter.second.at<uchar>(pt.y, pt.x);
      if (label == 0) {
        continue;
      }

      double cur_ts = ts_str.empty() ? 0.0 : std::stod(ts_str);
      if(SemanticFilter(lidar_pt, pt, iter.first, label, iter.second.cols, iter.second.rows, cur_ts)) {
      //  LOG(INFO) << "filtered point at pixel: " << pt.x << ", " << pt.y << " with label: " << static_cast<int>(255-label);
        continue;
      }

      if (image) {
        cv::circle(*image,  pts_2d[i], 2, cv::Scalar(255), -1); 
      }

      int indice = lidar_index_.at(target_index_.at(i));
     
      out_result->emplace_back(lidar_cloud->points.at(indice).x, lidar_cloud->points.at(indice).y,
                               lidar_cloud->points.at(indice).z,label);
    
    }
    if(image) {
      EnsureDirectory("/century/data/log/source/");
      cv::imwrite("/century/data/log/source/" + camera_names.at(static_cast<uint32_t>(iter.first)) + 
                  "_" + ts_str + "_" + std::to_string(iter.first) + ".png", *image);
      delete image;
    }
  }
  //LOG(INFO) << "point size with semantic : " << out_result->size() << std::endl;
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  LOG(INFO)<< "Rollout time: " << duration.count() << " ms";
  return out_result;
}
//52:60493150 38:61633532 39:61633623
bool FusedObjectGenerator::SemanticFilter(const cv::Point3f lidar_pt, const cv::Point2f pt, const CameraType& camera_type, const uchar label,
                                          const int width, const int height, const double ts) {
  if(!filter_distance_) {
    return false;
  }
  const uchar actual_label = 255 - label;
  switch(camera_type) {
    case CamFrontLeft:
      if(sqrt(pow(lidar_pt.x, 2) + pow(lidar_pt.z, 2)) > 10.0) {
        return true;
      }
      if(actual_label == 12){
        return true;
      }
      if ((actual_label == 10) && 
        ((pt.x <  width* 2/3) || (pt.y < height/2))) {
        return true;
      }
      if(std::abs(ts) < 1e-6) {
        return false;
      } 
      
      // still hardcode for test
      if((ts < 61633532.0) || (ts > 61633623.0)){
        return true;
      }
      break;
    case CamFrontRight: 
      
      if(sqrt(pow(lidar_pt.x, 2) + pow(lidar_pt.z, 2)) > 10.0) {
        return true;
      }
      if(actual_label == 12){
        return true;
      }
      if ((actual_label == 10) && 
         ((pt.x >  width* 1/3) || (pt.y < height/2))) {
        return true;
      }
      if(std::abs(ts) < 1e-6) {
        return false;
      } 

      // still hardcode for test
      if((ts > 60493150.0) && (ts < 61633623.0)){
        return true;
      } 
      break;
    
    case CamFrontMiddle:
    case CamRearMiddle:
      if(actual_label == 10) {
        return true;
      } 
      if(std::abs(ts) < 1e-6) {
        return false;
      } 

      // still hardcode for test
      if(actual_label == 12){
        if((ts < 60493150.0) || (ts > 61633623.0)){
          return true;
        }
      }
      break;
    case CamRearLeft:
    case CamRearRight:
    case CamTypeSize:
      break;
  }
  return false;
     


}


             
}