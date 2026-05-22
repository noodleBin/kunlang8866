//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once


#include <glog/logging.h>
//#include <execution>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "common/datatypes.h"
#include "frame.h"
#include "loop_closing.h"
#include "submap.h"

#include "common/eigen_types.h"
#include <sys/stat.h>
#include "amcl/include/amcl/map/map.h"
#include "semantic_mapping/amcl_map/generate_amcl_map.h"

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

namespace semantic_mapping {

class Submap;
class LoopClosing;


class Mapping2D {
   public:
  ~Mapping2D() = default;
  explicit Mapping2D(const double keyframe_pos_th,       
                     const double keyframe_ang_th,
                     const std::size_t th_num_kfs,
                     const std::string map_save_path,
                     const int map_width,
                     const bool need_loop_closing) {
    keyframe_pos_th_ = keyframe_pos_th;
    keyframe_ang_th_ = keyframe_ang_th;
    th_num_kfs_ = th_num_kfs;
    map_save_path_ = map_save_path;
    width_ = map_width * 1.0;
    need_loop_closing_ = need_loop_closing;
  }
  bool Init();
  bool ProcessScanKunlang(const ComposedSensorData& frame);
  cv::Mat ShowGlobalMap();

  // 1. save the last submap
  bool Shutdown();
  bool SaveSemanticMap();
  const std::array<map_t*,MapLayerSize>& GetAmclMap();
  Eigen::Vector3d GetMapOrigin() const { 
    return  global_map_origin_; 
  }

  Eigen::Vector3d GetMapBase() const { 
    return  global_map_base_; 
  }

  Eigen::Vector2i GetGridSize() const {
    return global_grid_size_;
  }
   private:

  bool NeedNewKeyFrame() ;
  void AddKeyFrame() ;
  //[TODO] add more conditions
  bool NeedNewSubmap() ;
  bool AddSubmap() ;

  void AddFrameInSubmap() ;

   private:
  // mode
  bool control_frame_time_ = true;
  double starting_frame_time_ = 0.0;
  double ending_frame_time_ = 0.0;


  size_t frame_id_ = 0;
  size_t keyframe_id_ = 0;
  size_t submap_id_ = 0;

  std::shared_ptr<AmclMapGenerator> amcl_map_generator_ = nullptr;
  std::shared_ptr<Frame> current_frame_ = nullptr;
  // std::shared_ptr<Frame> last_frame_ = nullptr;
  // SE2 motion_guess_;
  std::shared_ptr<Frame> last_keyframe_ = nullptr;
  std::shared_ptr<Submap> current_submap_ = nullptr;

  std::vector<std::shared_ptr<Submap>> all_submaps_;

  std::shared_ptr<LoopClosing> loop_closing_ = nullptr;  

  double width_ = 50.0;
  double keyframe_pos_th_ = 0.1;       
  double keyframe_ang_th_ = 5 * M_PI / 180;  
  std::size_t th_num_kfs_ = 50;
  std::string map_save_path_ = "/century/data/log/semantic/";
  std::vector<ObjectCell> global_occupancy_grid_;
  cv::Mat global_map_image_;
  Eigen::Vector3d global_map_origin_;
  Eigen::Vector3d global_map_base_;
  Eigen::Vector2i global_grid_size_;
  bool need_loop_closing_ = false;

};

}  // namespace semantic_mapping


