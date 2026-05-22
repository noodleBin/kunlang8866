//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once
#include <opencv2/core.hpp>

#include "common/eigen_types.h"
#include "common/datatypes.h"
#include "map_param.h"
#include "config/semantic_mapping_config.h"

namespace semantic_mapping {

class LikelihoodField {
   public:

    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    LikelihoodField() { 
      map_param_ = MapParam(SemanticMappingConfig::GetInstance()->MapWidth());
      BuildModel(); }

  
    void SetTargetScan(PointCloudPtr scan);


    void SetSourceScan(PointCloudPtr scan);


    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);


    bool AlignG2O(SE2& init_pose);

    cv::Mat GetFieldImage();

    bool HasOutsidePoints() const { return has_outside_pts_; }

    void set_pose(const SE2& pose) { pose_ = pose; }

   private:
    void BuildModel();

    SE2 pose_;  // T_W_S
    PointCloudPtr target_ = nullptr;
    PointCloudPtr source_ = nullptr;

    std::vector<ModelPoint> model_;  
    cv::Mat field_;                 
    bool has_outside_pts_ = false;   


    MapParam map_param_;
    // inline static const float resolution_ = 20; 
  };

}  // namespace semantic_mapping


