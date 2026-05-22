#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "cyber/common/file.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "modules/perception/landmark_loc/proto/amcl_recorder.pb.h"
#include <thread>
#include <condition_variable>
#include <list>

namespace landmark_loc{
class AmclRecorder {
  public:
  enum POSE_LABEL:uint32_t {
    PREDICTED = 0,  
    DR ,
    AMCL};
  static AmclRecorder* GetInstance() {
    static AmclRecorder recorder;
    return &recorder;
  }

   
    ~AmclRecorder();
    void AddData(const Eigen::Vector3d dr, const Eigen::Vector3d amcl, const double ts,
                 const std::vector<Eigen::Vector3d> particle, const Eigen::Vector3d cov, 
                 const uint32_t semantic_count, const uint32_t cluster_count);
    void AddResult(const Eigen::Vector3d predicted_pose, const double ts, const int indice = 0); 

  private:
    std::thread* recorder_thread_ = nullptr;
    void DoWork();
    AmclRecorder();
    bool EnsureDirectory(const std::string& path);
    bool inited_ = false;
    uint32_t seq_ = 0;
    uint32_t result_seq_ = 0;
    const uint32_t item_count_ = 1000;
    const uint32_t result_count_ = 1000;
    century::perception::landmark_loc::AmclPointPackages record_;
    std::shared_ptr<century::perception::landmark_loc::AmclPredictedPoses> dr_;
    std::shared_ptr<std::array<century::perception::landmark_loc::AmclPredictedPoses,2>> result_;
    std::shared_ptr<std::array<century::perception::landmark_loc::AmclPredictedPoses,2>> result_dump_;
    double dump_ts_ = 0.0;
    const std::string save_path_ = "/century/data/";
    std::atomic_bool stop_thread_{false};
    std::mutex mutex_;
    std::condition_variable not_empty_;
    const std::array<std::string,3UL> pose_label = {"/predicted_record/","/dr_record/","/amcl_record/"};
};
}