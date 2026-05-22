#include "map_show.h"
#include "common/pose_utils.h"
namespace semantic_mapping{

MapShow::MapShow(const std::array<::map_t*,MapLayerSize>& map, const Eigen::Matrix4d& origin): maps_(map), origin_(origin) {
 
  width_ = map[0]->size_x;
  height_ = map[0]->size_y;
  resolution_ = map[0]->scale;
  center_ = Eigen::Vector2d(width_*1.0/2, height_*1.0/2);

  occ_img_ = new cv::Mat(height_, width_, CV_8UC3, cv::Scalar(127, 127, 127));
  dis_img_ = new cv::Mat(height_, width_, CV_8UC3, cv::Scalar(127, 127, 127));

  thread_ = new std::thread(&MapShow::Display, this, 0);
}

void MapShow::clear() {
  delete occ_img_;
  delete dis_img_;
  occ_img_ = new cv::Mat(height_, width_, CV_8UC3, cv::Scalar(127, 127, 127));
  dis_img_ = new cv::Mat(height_, width_, CV_8UC3, cv::Scalar(127, 127, 127));
}

void MapShow::Display(const int flag) {
  std::vector<uint32_t> map_layer;
  for(auto i=0; i < MapLayerSize; ++i) {
    map_layer.push_back(i);
  }
  //map_layer.push_back(POINT_LABEL::Slotline);
  //map_layer.push_back(POINT_LABEL::Boundaryline);
 // auto result = draw_overlay(map_layer);
 // cv::imwrite( map_save_path_ + "/blended.png", result);
  draw_occ_state(map_layer);
  while (true) {
    LOG(INFO) << "Display Waiting frame " << std::endl;
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return !poselist_.empty();}); 
    
   // AINFO << "consume" << std::endl;
  
    cv::Mat frame = occ_img_->clone();
   /// LOG(INFO) << "Display POINTS " << poselist_.size();;
   
    for (auto &pose : poselist_) {
      cv::Point2f point_pose(pose(0)/0.1 , pose(1)/0.1 );
     // std::cout << "posepoint " << std::fixed  << point_pose.x << " " << point_pose.y << std::endl;
      cv::circle(frame, point_pose , 1, cv::Scalar(0, 0, 255), -1);
    }

    for (auto &pose : pathpoints_) {
      cv::Point2f point_pose(pose.first(0)/0.1 , pose.first(1)/0.1);
     // std::cout << "pathpoint " << std::fixed  << point_pose.x << " " << point_pose.y << std::endl;
      cv::circle(frame, point_pose , 1, pose.second ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 0), -1);
    }

    for (auto &pose : sensor_) {
      cv::Point2f point_pose(pose(0)/0.1 , pose(1)/0.1);
     // std::cout << "pathpoint " << std::fixed  << point_pose.x << " " << point_pose.y << std::endl;
      cv::circle(frame, point_pose , 1, cv::Scalar(255, 0, 0), -1);
    }

    for (auto &pose : drlist_) {
      cv::Point2f point_pose(pose(0)/0.1 , pose(1)/0.1);
     // std::cout << "pathpoint " << std::fixed  << point_pose.x << " " << point_pose.y << std::endl;
      cv::circle(frame, point_pose , 1, cv::Scalar(0, 255, 255), -1);
    }

   
    cv::imshow("Dynamic Point Cloud", frame);
    cv::waitKey(1);
    poselist_.clear();
    lock.unlock();
}

  /*
  for(const auto& pose : poselist_) {
    LOG(INFO)<< std::fixed << (pose(0)+ center_(0)) <<","<<(pose(1) +center_(1));
    cv::Point2f point_pose((pose(0) + center_(0)) , (pose(1) + center_(1)));
    cv::circle(*occ_img_, point_pose , 1, cv::Scalar(0, 0, 255), -1);
  }
  
  cv::imwrite( map_save_path_ + "/occupancy.png", *occ_img_);
  */
 // draw_occ_dist(POINT_LABEL::Slotline);
 // cv::imwrite( map_save_path_ + "/distance.png", *dis_img_);
}

void MapShow::draw_occ_state(const std::vector<uint32_t>& map) {
  for(const auto& label: map){
    auto* map = maps_[static_cast<uint32_t>(label)];
    for (int y = 0; y < map->size_y; ++y) {
      for (int x = 0; x < map->size_x; ++x) {
        const map_cell_t* c = map->cells + y*map->size_x + x;
        cv::Vec3b color;
        if (c->occ_state > 0) { 
          color = {0, 0, 0}; // occupied
          occ_img_->at<cv::Vec3b>(y, x) = color; 
        } 
      }
    }
  }
  //cv::flip(*occ_img_, *occ_img_, 0);
}


void MapShow::draw_occ_dist(const POINT_LABEL& label) {

  auto* map = maps_[static_cast<uint32_t>(label)];
  for (int y = 0; y < map->size_y; ++y) {
    for (int x = 0; x < map->size_x; ++x) {
      const map_cell_t* c = map->cells + y*map->size_x + x;
    //  LOG(INFO) << "cell dist: " << c->occ_dist<<" for "<< x << "," << y;
   
      if( c->occ_dist == 0) {
        cv::Vec3b color(0, 0, 255);
        dis_img_->at<cv::Vec3b>(y, x) = color;
      } else if( c->occ_dist < 10){
        cv::Vec3b color(255, 0, 0);
        dis_img_->at<cv::Vec3b>(y, x) = color;
      } else if( c->occ_dist < 20){
        cv::Vec3b color(0, 255, 0);
        dis_img_->at<cv::Vec3b>(y, x) = color;
      }
      
     
    }
   // std::cout << std::endl;
  }
  cv::imwrite( map_save_path_ + "/distance.png", *dis_img_);
}

void MapShow::AddPoselist(const std::vector<Eigen::Vector3d>& poselist) {
  std::unique_lock<std::mutex> lock(mutex_);
  //  AINFO << "produce" << std::endl;
  poselist_ = poselist;
  lock.unlock();
  not_empty_.notify_one();
  
}

void MapShow::AddPathPoint(const Eigen::Vector3d& path, const bool converaged) {
  std::unique_lock<std::mutex> lock(mutex_);
  pathpoints_.push_back(std::pair<Eigen::Vector3d, bool>(path, converaged));
  lock.unlock();
  not_empty_.notify_one();
}

void MapShow::AddDrPoint(const Eigen::Vector3d& dr) {
  std::unique_lock<std::mutex> lock(mutex_);
  drlist_.push_back(dr);
  lock.unlock();
  not_empty_.notify_one();
}

void MapShow::AddSensorPoint(const std::vector<Eigen::Vector3d>& sensor) {
  std::unique_lock<std::mutex> lock(mutex_);
  sensor_ = sensor;
  lock.unlock();
  not_empty_.notify_one();
}
}