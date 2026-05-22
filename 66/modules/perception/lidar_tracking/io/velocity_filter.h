#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unordered_map>
#include <tuple>
#include <list>
namespace century {
namespace perception {
namespace lidar {

class VelocityFilter {
 public:
 static VelocityFilter* GetInstance() {
    static VelocityFilter filter_;
    return &filter_;
  }
  void Init(const double convariance_thres, const double mean_shift_thres) {
    mean_shift_thres_ = mean_shift_thres;
    convariance_thres_ = convariance_thres;
  }

  VelocityFilter()  = default;
  ~VelocityFilter() = default;
  bool FliterStaticHypothesis(const int32_t track_id, 
                              const Eigen::Vector2d& pose, 
                              const double ts) {
    // update context
    timestamp_map_[track_id] = ts;
    static_hypothesis_map_[track_id].push_back(pose);
    Eigen::Vector2d previous_center(0.0,0.0);
    if(mean_center_map_.count(track_id) > 0) {
      previous_center = mean_center_map_.at(track_id);
    }
    mean_center_map_[track_id] = GetStaticCenter(track_id);
    UpdateContext(ts);

   
    ADEBUG << "===============================track_id: ======================" 
          << track_id << " has "<<  static_hypothesis_map_[track_id].size()
          <<" Mean center for track_id: "<<  mean_center_map_.at(track_id).transpose();

    if(static_hypothesis_map_[track_id].size() < min_filter_size_) {
      return false;
    }

    if(static_hypothesis_map_[track_id].size() > max_filter_size_) {
      static_hypothesis_map_[track_id].pop_front();
    }

    // calculate the mean velocity
    if(mean_center_map_.at(track_id).isZero()) {
      AINFO << "current center for track_id: " << track_id << " iszero ";
      return false;
    } 

    const double convariance = GetConvariance(track_id);
    ADEBUG << std::fixed << track_id << " Convariance is " << convariance;

    if(previous_center.isZero()) {
      return false;
    }
    const auto mean_shift = (mean_center_map_.at(track_id) - previous_center).norm();
    ADEBUG << std::fixed << track_id << " Mean shift is " << mean_shift ;
    
    if(convariance > convariance_thres_) {
      AINFO <<"Convariance is not qualified";
      return false;
    }

    if(mean_shift > mean_shift_thres_) {
       AINFO <<"meanshift is not qualified";
      return false;
    }

    return true; 
  }
  private:

  double GetConvariance(const int32_t track_id) {
    if ((static_hypothesis_map_.count(track_id) == 0) || 
        (mean_center_map_.count(track_id) == 0)) {
      AERROR <<"Not find track id " << track_id;
      return error_convariance_;
    }
    double convariance = 0.0;
    for (const auto& iter : static_hypothesis_map_.at(track_id)) {
      convariance = convariance + (iter - mean_center_map_.at(track_id)).squaredNorm();
    }
   
   return convariance /static_hypothesis_map_.at(track_id).size();
  }

  Eigen::Vector2d GetStaticCenter(const int32_t track_id) {
    if(static_hypothesis_map_.count(track_id) == 0) {
      AERROR <<"Not find track id " << track_id;
      return Eigen::Vector2d(0.0,0.0);
    }

    Eigen::Vector2d center(0.0, 0.0);
    for(const auto& iter: static_hypothesis_map_.at(track_id)) {
      center = center + iter;
    }
    return (center / static_hypothesis_map_.at(track_id).size());
  }

  void UpdateContext(double ts) {
    for (auto it = timestamp_map_.begin(); it != timestamp_map_.end(); ) {
      if (ts - it->second > keep_duration_) {
        static_hypothesis_map_.erase(it->first);
        mean_center_map_.erase(it->first);
        it = timestamp_map_.erase(it); 
      } else {
        ++it;
      }
    }
  }

  std::unordered_map<int32_t, std::list<Eigen::Vector2d>> static_hypothesis_map_; 
  std::unordered_map<int32_t, double> timestamp_map_;
  std::unordered_map<int32_t, Eigen::Vector2d> mean_center_map_;
  double convariance_thres_ = 0.3;
  double mean_shift_thres_ = 0.1;
  const int32_t min_filter_size_ = 20;
  const int32_t max_filter_size_ = 20; 
  const double error_convariance_ = -1.0; 
  const double keep_duration_ = 1.0;
                                        
};
}
}
}