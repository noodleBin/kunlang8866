//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once
#include <glog/logging.h>
#include <Eigen/Dense>
#include <atomic>

#include "frame.h"
#include "object_data.h"
#include "map_param.h"
#include "common/eigen_types.h"
#include "config/semantic_mapping_config.h"

namespace semantic_mapping {

class ObjectCell {
   public:
   ObjectCell() = default;  
  explicit ObjectCell(int min_times, double min_rate, double res_ratio)
    : min_times_(min_times), min_rate_(min_rate), res_ratio_(res_ratio) {
    occ_state_ = ObjectType::Init;
    max_state_ = ObjectType::Init;
    visit_time_ = 0;
    is_updated_ = false;
    occ_times_.clear();
    occ_times_.emplace(ObjectType::Init, 0);
  }

  void Insert(ObjectType state) {
    if (occ_times_.find(state) != occ_times_.end()) {
      occ_times_[state]++;
    } else {
      occ_times_.emplace(state, 1);
    }

    if (occ_times_[state] > occ_times_[max_state_]) {
      max_state_ = state;
    }

    visit_time_++;
    is_updated_ = true;

    return;
  }
  
  void FastInsert(ObjectType state) {
    if(static_cast<uint32_t>(objectType_to_objectMode.at(state)) > static_cast<uint32_t>(objectMode_)) {
      occ_state_ = state;
      objectMode_ = objectType_to_objectMode.at(state);
      is_updated_ = true;
    }
    return;
  }

  void CatObjectCell(const ObjectCell& src) {
    for(const auto& iter : src.occ_times_) {
      if (occ_times_.find(iter.first) != occ_times_.end()) {
        occ_times_[iter.first] += iter.second;
      } else {
        occ_times_.emplace(iter.first, iter.second);
      }
      if (occ_times_[iter.first] > occ_times_[max_state_]) {
        max_state_ = iter.first;
      }
    }
    visit_time_ += src.visit_time_;
    is_updated_ = true;
    return;
  }

  ObjectType occ_state() {
    if (is_updated_) {
      UpdateOccState();
      is_updated_ = false;
    }
    return occ_state_;
  }

  const ObjectType& get_occ_state() const {
    return occ_state_;
  }

  const ObjectClass& get_occ_class() const {
    return objectType_to_class.at(occ_state_);
  }

   private:

  //[TODO] update by change not by read
  void UpdateOccState() {
    ObjectType max_num_state = ObjectType::Init;
    int max_num_times = 0;
    for(const auto& iter : occ_times_) {
      if(objectType_to_objectMode.at(iter.first) == ObjectMapCellMode::Number) {
        if(iter.second > max_num_times) {
          max_num_times = iter.second;
          max_num_state = iter.first;
        }
      }
    }
    if(max_num_times != 0) {
      occ_state_ = max_num_state;
      objectMode_ = ObjectMapCellMode::Number;
      return;
    }

    for(const auto& iter : occ_times_) {
      if(objectType_to_objectMode.at(iter.first) == ObjectMapCellMode::Sign) {
        if(iter.second > max_num_times) {
          max_num_times = iter.second;
          max_num_state = iter.first;
        }
      }
    }
    if(max_num_times != 0) {
      occ_state_ = max_num_state;
      objectMode_ = ObjectMapCellMode::Sign;
      return;
    }


    if (visit_time_ < min_times_ || 1.0 * occ_times_[max_state_] / visit_time_ < min_rate_) {
      occ_state_ = ObjectType::Init;
    } else {
      occ_state_ = max_state_;
    }
    objectMode_ = objectType_to_objectMode.at(occ_state_);
    return;
    

  }

  //
  int min_times_ = 2;
  double min_rate_ = 0.25;
  double res_ratio_ = 1.0;  // grid_map/pic

  ObjectType occ_state_ = ObjectType::Init;
  ObjectType max_state_ = ObjectType::Init;
  ObjectMapCellMode objectMode_ = ObjectMapCellMode::Init;

  int visit_time_ = 0;
  // <mode, times>
  std::unordered_map<ObjectType, int> occ_times_;

  //[TODO] better way?
  bool is_updated_ = false;
};


class ObjectOccupancyMap {
   public:

  ObjectOccupancyMap() : is_updated_(false) {
    map_param_ = MapParam(SemanticMappingConfig::GetInstance()->MapWidth()*1.0);
    occupancy_grid_.resize(map_param_.grid_size_[0] * map_param_.grid_size_[1],
                 ObjectCell(cell_param_.min_times_, cell_param_.min_rate_, cell_param_.res_ratio_));
    LOG(WARNING) << "map_param: " << map_param_.resolution_ << "; " << map_param_.map_size_[0] << ", "
           << map_param_.map_size_[1] << "; " << map_param_.grid_size_[0] << ", " << map_param_.grid_size_[1]
           << "; " << map_param_.grid_center_[0] << ", " << map_param_.grid_center_[1] << std::endl;
  }

  void AddKeyFrame(std::shared_ptr<Frame> frame) {
    PointCloudPtr scan = frame->scan();

    has_outside_pts_ = false;

    //LOG(INFO) << "scan->points.size(): " << scan->points.size() << std::endl;
    for (size_t i = 0; i < scan->points.size(); i++) {
      Eigen::Vector2i pt = WorldToGrid(frame->pose_map_SE2() * Vec2d(scan->points.at(i).x, scan->points.at(i).y));
      SetPoint(pt, scan->points.at(i).label);
    }

    is_updated_ = true;
  }

  void set_pose(const SE2 &pose) { pose_ = pose; }
  SE2 pose() const { return pose_; }
  double resolution() const { return map_param_.resolution_; }
  const Eigen::Vector2d& map_size() const { return map_param_.map_size_; }
  const Eigen::Vector2i& grid_size() const { return map_param_.grid_size_; }
  const Eigen::Vector2i& grid_center() const { return map_param_.grid_center_; }

  std::vector<ObjectCell>& occupancy_map() { return occupancy_grid_; }
  cv::Mat GetOccupancyGrid() {
    // LOG(INFO) << "= GetOccupancyGrid =" << std::endl;
    UpdateOccupancyGridCvMat();
    return occupancy_grid_cvMat_;
  }

  cv::Mat GetOccupancyGridColor() {
    // LOG(INFO) << "= GetOccupancyGrid =" << std::endl;
    UpdateOccupancyGridCvMat();
    return occupancy_grid_cvMat_bw_;
  }
 
  bool has_outside_pts() const { return has_outside_pts_; }

   private:
  //[TODO] after replace, delete it
  void UpdateOccupancyGridCvMat() {
    if (!is_updated_) {
      return;
    }

    occupancy_grid_cvMat_ = cv::Mat(map_param_.grid_size_[0], map_param_.grid_size_[1], CV_8U, 127);
    for (int y = 0; y < map_param_.grid_size_[1]; y++) {
        for (int x = 0; x < map_param_.grid_size_[0]; x++) {
            int id = x + (map_param_.grid_size_[1] - y - 1) * map_param_.grid_size_[0];

            if(objectType_to_objectMode.count(occupancy_grid_[id].occ_state()) == 0) {
              continue;  // skip undefined types
            }
            auto object_mode = objectType_to_objectMode.at(occupancy_grid_[id].occ_state());
            if(object_mode == ObjectMapCellMode::Line) {
              occupancy_grid_cvMat_.at<uchar>(y, x) = 126;
            }
        }
    }
    //cv::flip(occupancy_grid_cvMat_, occupancy_grid_cvMat_, 0);

    occupancy_grid_cvMat_bw_ =
      cv::Mat(map_param_.grid_size_[0], map_param_.grid_size_[1], CV_8UC3, cv::Vec3b(127, 127, 127));
    for (int y = 0; y < map_param_.grid_size_[1]; y++) {
      for (int x = 0; x < map_param_.grid_size_[0]; x++) {
        int id = x + (map_param_.grid_size_[1] - y - 1) * map_param_.grid_size_[0];
        if(objectType_to_objectMode.count(occupancy_grid_[id].occ_state()) == 0) {
          continue;  // skip undefined types
        }
        auto object_mode = objectType_to_objectMode.at(occupancy_grid_[id].occ_state());
        if(objectType_to_bgr.count(object_mode) == 0) {
          continue;  // skip undefined types
        }
        occupancy_grid_cvMat_bw_.at<cv::Vec3b>(y, x) = objectType_to_bgr.at(object_mode);
      }
    }
    //cv::flip(occupancy_grid_cvMat_bw_, occupancy_grid_cvMat_bw_, 0);

    is_updated_ = false;
  }

  Eigen::Vector2i WorldToGrid(const Eigen::Vector2d &pt_w) {
    // pt_world -> pt_map -> pt_grid
    Eigen::Vector2d pt_map = pose_.inverse() * pt_w;
    Eigen::Vector2i pt_grid = Eigen::Vector2i(std::ceil(pt_map[0] / map_param_.resolution_),
                          std::ceil(pt_map[1] / map_param_.resolution_)) +
                  map_param_.grid_center_;

    return pt_grid;
  }
  void WorldToMap() {}
  void MapToGrid() {}

  void SetPoint(const Eigen::Vector2i &pt_grid, uchar semantic) {
    if (pt_grid[0] < 0 || pt_grid[0] >= map_param_.grid_size_[0] || pt_grid[1] < 0 ||
      pt_grid[1] >= map_param_.grid_size_[1]) {
        has_outside_pts_ = true;
      return;
    }

   // LOG(INFO) << "pt_grid: " << pt_grid[0] << ", " << pt_grid[1] <<", "<< semantic << std::endl;
    int id = pt_grid[1] * map_param_.grid_size_[0] + pt_grid[0];
    occupancy_grid_[id].Insert(static_cast<ObjectType>(255-semantic));
  
  }

  std::vector<ObjectCell> occupancy_grid_;
  cv::Mat occupancy_grid_cvMat_bw_;
  cv::Mat occupancy_grid_cvMat_;

  SE2 pose_;
  bool has_outside_pts_ = false;  
  bool is_updated_ = false;

  // map_param
  MapParam map_param_;
  CellParam cell_param_;
};

}  // namespace semantic_mapping


