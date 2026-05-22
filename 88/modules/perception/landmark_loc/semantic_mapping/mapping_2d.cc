#include "mapping_2d.h"
#include <iostream>
#include <limits>
#include "object_occupancy_map.h"

namespace semantic_mapping {

bool Mapping2D::Init() {
  frame_id_ = 0;
  keyframe_id_ = 0;
  submap_id_ = 0;
  loop_closing_ = need_loop_closing_ ? std::make_shared<LoopClosing>() : nullptr;
  struct stat info;
  if (stat(map_save_path_.c_str(), &info) != 0) {
    mkdir(map_save_path_.c_str(), 0755);
  } 
  return true;
}

cv::Mat Mapping2D::ShowGlobalMap (){
// LOG(INFO) << "= ShowGlobalMap-begin =" << std::endl;

  Vec2f top_left = Vec2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  Vec2f bottom_right = Vec2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

  // const float submap_resolution = 20.0; 
  // const float submap_size = 50.0;        

  global_map_base_ = PoseUtils::EigenMatToVector3d(PoseUtils::SE2ToEigenMat(all_submaps_.at(0)->pose_odom()));
  LOG(INFO) << "global_map_base_: " << global_map_base_(0) <<","
            <<  global_map_base_(1) <<"," << global_map_base_(2) << std::endl;

  /// calculate global map scope
  for (auto m : all_submaps_) {
    float submap_size = m->object_occu_map().map_size()[0];
    Vec2d c = m->pose().translation();
    if (top_left[0] > c[0] - submap_size / 2) {
      top_left[0] = c[0] - submap_size / 2;
    }
    if (top_left[1] > c[1] - submap_size / 2) {
      top_left[1] = c[1] - submap_size / 2;
    }

    if (bottom_right[0] < c[0] + submap_size / 2) {
      bottom_right[0] = c[0] + submap_size / 2;
    }
    if (bottom_right[1] < c[1] + submap_size / 2) {
      bottom_right[1] = c[1] + submap_size / 2;
    }
  }
  if (top_left[0] > bottom_right[0] || top_left[1] > bottom_right[1]) {
    return cv::Mat();
  }
  LOG(INFO) << "global_map_size: " << top_left[0] << ", " << top_left[1] << "; " << bottom_right[0] << ", "
                << bottom_right[1] << std::endl;

  /// calculate  global map center, size, resolution
  Vec2f global_center = Vec2f((top_left[0] + bottom_right[0]) / 2.0, (top_left[1] + bottom_right[1]) / 2.0);
  global_map_origin_ = Eigen::Vector3d(top_left[0], top_left[1], 0.0);
  float phy_width = bottom_right[0] - top_left[0];   // phy size
  float phy_height = bottom_right[1] - top_left[1];  //   phy size
  LOG(INFO) << "global_center: " << global_center[0] << ", " << global_center[1] << "; " << phy_width << ", "
               << phy_height << std::endl;
  float global_map_resolution = 10;
 
  Vec2f c = global_center;
  int c_x = global_center[0] * global_map_resolution;
  int c_y = global_center[1] * global_map_resolution;
  global_center = Vec2f(c_x / global_map_resolution, c_y / global_map_resolution);  // grid center

  int width = int((bottom_right[0] - top_left[0]) * global_map_resolution + 0.5); //grid x
  int height = int((bottom_right[1] - top_left[1]) * global_map_resolution + 0.5); //grid y

  Vec2f center_image = Vec2f(width / 2, height / 2);
  // background, grey
  cv::Mat output_image(height, width, CV_8UC3, cv::Scalar(127, 127, 127));
  // LOG(INFO) << "global_map_param: " << global_map_resolution << "; " << global_center[0] << ", "
  //            << global_center[1] << "; " << width << ", " << height << std::endl;
  // LOG(INFO) << "center_image: " << center_image[0] << ", " << center_image[1] << std::endl;

  //@xinrong, grid_map
  // global map coordinator
  std::vector<Vec2i> render_data;
  render_data.reserve(width * height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      render_data.emplace_back(Vec2i(x, y));
    }
  }
  global_occupancy_grid_.resize(width*height);

  global_grid_size_ = Vec2i(width, height);

  LOG(INFO) << "= ShowGlobalMap-02 =" << std::endl;
  LOG(INFO) << "render_data: " << render_data.size() << std::endl;
  // submap rendering

  //std::for_each(std::execution::par_unseq, render_data.begin(), render_data.end(), [&](const Vec2i& xy) {
  for(const auto xy : render_data) {
    int x = xy[0], y = xy[1];
    Vec2f pw = (Vec2f(x, y) - center_image) / global_map_resolution + c;  // global coordinator

    //LOG(WARNING) << "all_submaps_: " << all_submaps_.size() << std::endl;
    int target_id = y * width + x;
    auto& target_cell = global_occupancy_grid_.at(target_id);
    for (auto& m : all_submaps_) {
      const double submap_resolution = 10;
      const Eigen::Vector2i& grid_center = m->object_occu_map().grid_center();

      Vec2f ps = m->pose().inverse().cast<float>() * pw;  // in submap
      Vec2i pt = (ps * submap_resolution + Vec2f(grid_center[0], grid_center[1])).cast<int>();
      
      if (pt[0] < 0 || pt[0] >= m->object_occu_map().grid_size()[0] || pt[1] < 0 ||
          pt[1] >= m->object_occu_map().grid_size()[1]) {
        continue;
      }

      auto& cells = m->object_occu_map().occupancy_map();
      // uchar value = m->occu_map().GetOccupancyGrid().at<uchar>(pt[1], pt[0]);
      // uchar value = m->object_occu_map().GetOccupancyGrid().at<uchar>(pt[1], pt[0]);
      int src_id = pt[1] * m->object_occu_map().grid_size()[0] + pt[0];
      auto& src_cell = cells.at(src_id);
      if(src_cell.occ_state() == ObjectType::Init) {
        continue;
      }
      target_cell.CatObjectCell(src_cell);
    }
  }

  LOG(INFO) << "= ShowGlobalMap-03 =" << width << "x" << height << std::endl;
  global_map_image_ = cv::Mat(height, width, CV_8UC3, cv::Vec3b(127, 127, 127));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int id = x + (height- y-1) * width;
      if(objectType_to_objectMode.count(global_occupancy_grid_[id].occ_state()) == 0) {
        continue;  // skip undefined types
      }
      auto object_mode = objectType_to_objectMode.at(global_occupancy_grid_[id].occ_state());
      if(objectType_to_bgr.count(object_mode) == 0) {
        continue;  // skip undefined types
      }
      global_map_image_.at<cv::Vec3b>(y, x) = objectType_to_bgr.at(object_mode);
    }
  }
  cv::flip(global_map_image_, global_map_image_, 0);
  cv::imwrite(map_save_path_ + "/global.png", global_map_image_);

  LOG(INFO) << "= ShowGlobalMap-end =" << std::endl;
  return output_image;
}
bool Mapping2D::ProcessScanKunlang(const ComposedSensorData& frame) {
 
  current_frame_ = std::make_shared<Frame>(frame);
  current_frame_->set_id(frame_id_++);

  // set pose
  LOG(INFO)<<"= ProcessScanKunlang = " << std::endl;
  current_frame_->set_pose_odom(frame.pose_odom);
  if (last_keyframe_ == nullptr) {
    current_frame_->set_pose_map(Eigen::Matrix4d::Identity());
  } else {
    Eigen::Matrix4d delta_pose_odom =
        last_keyframe_->pose_odom().inverse() * current_frame_->pose_odom();
    current_frame_->set_pose_map(last_keyframe_->pose_map() * delta_pose_odom);
  }
  if (NeedNewKeyFrame()) {
    AddKeyFrame();
    if (NeedNewSubmap()) {
      AddSubmap();
    }
    AddFrameInSubmap();
 }

  return true;
}


bool Mapping2D::NeedNewKeyFrame() {
  // LOG(INFO) << "= NeedNewKeyFrame" << std::endl;
  if (last_keyframe_ == nullptr) {
    return true;
  }

  Eigen::Matrix4d delta_pose_odom_ = last_keyframe_->pose_odom().inverse() * current_frame_->pose_odom();
  Eigen::Vector3d pose2d = PoseUtils::EigenMatToVector3d(delta_pose_odom_);
  if (sqrt(pow(pose2d[0], 2) + pow(pose2d[1], 2)) > keyframe_pos_th_ || pose2d[2] > keyframe_ang_th_) {
  return true;
  }

  return false;
}
void Mapping2D::AddKeyFrame() {
  LOG(INFO) << "= AddKeyFrame: " << keyframe_id_ << "; submap: " << submap_id_ << std::endl;
  current_frame_->set_keyframe_id(keyframe_id_++);
  last_keyframe_ = current_frame_;
}
//[TODO] add more conditions
bool Mapping2D::NeedNewSubmap() {
  // LOG(INFO) << "= NeedNewSubmap" << std::endl;
  if (current_submap_ == nullptr) {
    return true;
  }

  if (
    // current_submap_->HasOutsidePoints() ||
    current_submap_->num_keyframes() > th_num_kfs_) {
    return true;
  }

  return false;
}
bool Mapping2D::AddSubmap() {
  LOG(INFO) << "= AddSubmap" << std::endl;
  if (current_submap_ == nullptr) {
    LOG(INFO) << "submap pose is" << current_frame_->pose_map_SE2().log() << std::endl;
    current_submap_ = std::make_shared<Submap>(current_frame_->pose_map_SE2());
    current_submap_->set_pose_odom(current_frame_->pose_odom_SE2());
  } else {
    if (loop_closing_) {
      loop_closing_->AddFinishedSubmap(current_submap_);
    }
    auto last_submap = current_submap_;

    // debug
    cv::imwrite(map_save_path_ + "/submap_" + std::to_string(last_submap->id()) + ".png",
          // last_submap->occu_map().GetOccupancyGridBlackWhite());
          last_submap->object_occu_map().GetOccupancyGridColor());

    last_submap->MatchFrame(current_frame_);
    current_submap_ = std::make_shared<Submap>(current_frame_->pose_map_SE2());
    current_submap_->set_pose_odom(current_frame_->pose_odom_SE2());
    current_submap_->SetOccuFromOtherSubmap(last_submap);
    last_submap->set_finished(true);
  }

  current_submap_->set_id(submap_id_++);
  all_submaps_.emplace_back(current_submap_);
  if (loop_closing_) {
    loop_closing_->AddNewSubmap(current_submap_);
  }

  LOG(WARNING) << "create submap: " << current_submap_->id()
        << " with pose: " << current_submap_->pose().translation().transpose() << ", "
        << current_submap_->pose().so2().log();
  return true;
}
//@xinrong, current_frame need be ready here
void Mapping2D::AddFrameInSubmap() {
  LOG(INFO) << "= AddFrameInSubmap" << std::endl;

  current_submap_->MatchFrame(current_frame_);
  current_submap_->AddKeyFrame(current_frame_);

  LOG(WARNING) << "time_submap_add_keyframe[ms]: "<< std::fixed << current_frame_->timestamp() << std::endl;

  if (loop_closing_) {
    loop_closing_->AddNewFrame(current_frame_);
    LOG(WARNING) << "time_loopclosing_add_keyframe[ms]: " << std::fixed << current_frame_->timestamp() << std::endl;
  }
}

bool Mapping2D::SaveSemanticMap() {
  std::vector<ObjectCell> grid_map = global_occupancy_grid_;
  Eigen::Vector3d origin = GetMapOrigin();
  Eigen::Vector3d base = GetMapBase();
  MapParam param(width_);
  param.grid_size_ = GetGridSize();
  amcl_map_generator_.reset(new AmclMapGenerator(param, origin, base, map_save_path_));
  if(!amcl_map_generator_->AmclMapBuild(grid_map, true)) {
    LOG(ERROR) << "AmclMapBuild failed!" << std::endl;
    return false;
  }
  return true;  
}

const std::array<map_t*, MapLayerSize>& Mapping2D::GetAmclMap() {
  std::vector<ObjectCell> grid_map = global_occupancy_grid_;
  Eigen::Vector3d origin = GetMapOrigin();
  Eigen::Vector3d base = GetMapBase();
  MapParam param(width_);
  param.grid_size_ = GetGridSize();
  amcl_map_generator_.reset(new AmclMapGenerator(param, origin, base, map_save_path_));
  if(!amcl_map_generator_->AmclMapBuild(grid_map, false)) {
    LOG(ERROR) << "AmclMapBuild failed!" << std::endl;
    return {};
  }
  return amcl_map_generator_->GetAmclMap();  
}

}