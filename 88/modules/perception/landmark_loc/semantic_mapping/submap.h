//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include "frame.h"
#include "likelihood_filed.h"
#include "object_occupancy_map.h"

namespace semantic_mapping {


class Submap {
   public:
  Submap(const SE2& pose) : pose_(pose) {
    // occu_map_.set_pose(pose_);
    //[TODO] init with same map_param
    object_occu_map_ = ObjectOccupancyMap();
    field_ = LikelihoodField();
    object_occu_map_.set_pose(pose_);
    field_.set_pose(pose_);
  }

  void SetOccuFromOtherSubmap(std::shared_ptr<Submap> other) {
    auto frames_in_other = other->keyframes();

    LOG(WARNING) << "frames_in_other.size(): " << frames_in_other.size() << std::endl;
    for (size_t i = frames_in_other.size() - 10; i < frames_in_other.size(); ++i) {
      if (i > 0) {
        // occu_map_.AddLidarFrame(frames_in_other[i]);
        object_occu_map_.AddKeyFrame(frames_in_other[i]);
      }
    }
    // field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());
    field_.SetFieldImageFromOccuMap(object_occu_map_.GetOccupancyGrid());
  }


  bool MatchFrame(std::shared_ptr<Frame> frame) {
    //[DEBUG]
    // field_.SetSourceScan(frame->scan());
    // SE2 initial_pose_submap = frame->pose_submap_SE2();
    // field_.AlignG2O(initial_pose_submap);
    //@xinrong, T_s_c from odometry
    // initial pose
     // auto initial_pose_map = frame->pose_map_SE2();
     LOG(INFO) << "MatchFrame: " << std::endl;
    auto delta_pose_map = pose_.inverse() * frame->pose_map_SE2();
    //[TODO] alignment
    auto frame_pose_submap = delta_pose_map;
    auto estimated_frame_pose = pose_ * frame_pose_submap;  // T_w_c = T_w_s * T_s_c
    frame->set_pose_map_SE2(estimated_frame_pose);

    return true;
  }


  void AddKeyFrame(std::shared_ptr<Frame> frame) {
    keyframes_.emplace_back(frame);
    AddScanInOccupancyMap(frame);
  }

  void UpdateFramePoseWorld() {
    for (auto& frame : keyframes_) {
      auto delta_pose_map = pose_.inverse() * frame->pose_map_SE2();
      auto frame_pose_submap = delta_pose_map;
      auto frame_pose = pose_ * frame_pose_submap;
      frame->set_pose_map_SE2(frame_pose);
    }
  }


  bool HasOutsidePoints() const {
    // return occu_map_.HasOutsidePoints();
    return object_occu_map_.has_outside_pts();
  }

  void set_id(size_t id) { id_ = id; }
  size_t id() const { return id_; }
  // called by: LoopClosing::Optimize[]
  void set_pose(const SE2& pose){
    pose_ = pose;
    // occu_map_.set_pose(pose);
    object_occu_map_.set_pose(pose);
    field_.set_pose(pose);
  }
  void set_pose_odom(const SE2& pose) { 
    pose_odom_ = pose; 
  }
  SE2 pose() const { return pose_; }
  SE2 pose_odom() const { return pose_odom_; }

  std::vector<std::shared_ptr<Frame>>& keyframes() { return keyframes_; }
  size_t num_keyframes() const { return keyframes_.size(); }

  void set_finished(bool is_finished) {
    //[DEBUG]
    return;
    is_finished_= is_finished;
    if (is_finished_) {
      keyframes_.clear();
    }
  }
  bool is_finished() const { return is_finished_; }

  /// accessors
  // OccupancyMap& occu_map() { return occu_map_; }
  ObjectOccupancyMap& object_occu_map() { return object_occu_map_; }
  LikelihoodField& likelihood() { return field_; }

   private:

  void AddScanInOccupancyMap(std::shared_ptr<Frame> frame) {
     LOG(INFO) << "= AddScanInOccupancyMap =" << std::endl;
    // occu_map_.AddLidarFrame(frame, OccupancyMap::GridMethod::MODEL_POINTS);
    object_occu_map_.AddKeyFrame(frame); 
    // field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());       
    field_.SetFieldImageFromOccuMap(object_occu_map_.GetOccupancyGrid());     
  }

  size_t id_ = 0;
  bool is_finished_ = false;
  SE2 pose_;  // submap的pose, Tws
  SE2 pose_odom_; // for delta_pose_odom
          // frame_pose_submap: 1. before alignment: delta_pose_odom
          // 2. after alignment: delta_pose_map

  std::vector<std::shared_ptr<Frame>> keyframes_;  
  // LikelihoodField for tracking, MRLikelihoodField for loopclosing
  LikelihoodField field_;           

  // OccupancyMap occu_map_;
  //SemanticOccupancyMap semantic_occu_map_;
  ObjectOccupancyMap object_occu_map_;
};

}  // namespace semantic_mapping


