#pragma once
#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_map>
#include <glog/logging.h>
#include "common/datatypes.h"
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include "common/eigen_types.h"
#include "amcl_worker/amcl_pose_safety_checker.h"
#include "visualizer/map_show.h"
#include "amcl_worker/recorder/amcl_recorder.h"


namespace semantic_mapping {
using namespace amcl;
class AmclWorker {
  public:
  AmclWorker() = default;
  explicit AmclWorker(const std::string map_save_path, 
                      const int min_particle_num, 
                      const int max_particle_num,
                      const double hit,
                      const std::array<double, 3> particle_cov,
                      const double rand,
                      const double sigma,
                      const bool enable_pose_safety_check,
                      const double safety_dis,
                      const bool force_update,
                      const bool need_view,
                      const bool need_recorder) {
    map_save_path_ = map_save_path;
    min_particles_ = min_particle_num;
    max_particles_ = max_particle_num;
    z_hit = hit;
    x_convariance_ = particle_cov.at(0);
    y_convariance_ = particle_cov.at(1);
    yaw_convariance_ = particle_cov.at(2);
    z_rand = rand;
    sigma_hit = sigma;
    enable_pose_safety_check_ = enable_pose_safety_check;
    safety_dis_ = safety_dis;
    force_update_ = force_update;
    need_view_ = need_view;
    recorder_amcl_ = need_recorder;
    LOG(INFO) << "AmclWorker config: map_save_path=" << map_save_path_
              << ", min_particles=" << min_particles_
              << ", max_particles=" << max_particles_
              << ", z_hit=" << z_hit
              << ", particle_cov=[" << x_convariance_ << ", "
              << y_convariance_ << ", " << yaw_convariance_ << "]"
              << ", z_rand=" << z_rand
              << ", sigma_hit=" << sigma_hit
              << ", enable_pose_safety_check=" << enable_pose_safety_check_
              << ", safety_dis=" << safety_dis_
              << ", force_update=" << force_update_
              << ", need_view=" << need_view_
              << ", recorder_amcl=" << recorder_amcl_;
  }

  ~AmclWorker() = default;
  

  bool Init(std::vector<std::tuple<double, Eigen::Matrix4d>>& traj);
  std::shared_ptr<Eigen::VectorXd> ReceiveSensorData(const Eigen::Matrix4d& odom,
                                                     const Eigen::Matrix4d& dr_odom,
                                                     const pcl::PointCloud<pcl::PointXYZL>::Ptr data,
                                                     const double ts);
  void HandleMapMessage();
  void SetInitPose(const Eigen::Matrix4d& init);
  bool FetchDrPose(const double& ts, Eigen::Matrix4d& dr);
  std::vector<Eigen::Vector3d> GetPoseCollect() const;
  bool NeedInit();

  private:

  std::array<::map_t*,MapLayerSize> map_;
  semantic_mapping::PoseWithConvariance last_published_pose_;
  std::vector<std::tuple<double, Eigen::Matrix4d>> traj_;
  AMCLLaser* sensor_handler_ = nullptr;
  AMCLOdom* odom_ = nullptr;
  double z_hit = 0.99;
  double z_rand = 0.01;
  double sigma_hit = 0.01;
  int min_particles_ = 1000;
  int max_particles_ = 5000;
  const double alpha_slow = 0.001;
  const double alpha_fast = 0.1;
  const double pf_z = 0.99;
  const double pf_err = 0.99;
  const double alpha1 = 0.05;
  const double alpha2 = 0.05;
  const double alpha3 = 0.02;
  const double alpha4 = 0.02;
  const double alpha5 = 0.02;
  const double d_thresh = 0.2;
  const double a_thresh = M_PI/6.0;
  std::string map_save_path_ = "/century/data/log/semantic/";

  pf_t *pf_ = nullptr;
  bool pf_init_ = false;
  int semantic_continue_lacked_= 0;
  int target_semantic_count_ = 100;
  const odom_model_t odom_model_type = ODOM_MODEL_DIFF;
  pf_vector_t pf_odom_pose_;
  pf_vector_t pf_hyp_pose_;
  bool force_update_ = true;
  int resample_count_ = 0;
  int resample_interval_ = 3;
  bool selective_resampling_ = false;
  Eigen::Matrix4d origin_;
  Eigen::Matrix4d base_ = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Vector3d> pose_collect_;
  MapShow* visualizer_ = nullptr;
  double x_convariance_ = 0.5;
  double y_convariance_ = 0.5;
  double yaw_convariance_ = 0.07;
  bool need_view_ = false;
  bool recorder_amcl_ = false;
  bool enable_pose_safety_check_ = false;
  Eigen::Vector3d pre_gt_pose_{0.0,0.0,0.0};
  AmclPoseSafetyChecker pose_safety_checker_;
  double last_pose_eval_ts_ = 0.0;
  double safety_dis_ = 0.35;
   
  static std::vector<std::pair<int,int> > free_space_indices;
  static pf_vector_t uniformPoseGenerator(void* arg);
 
  bool ConvertPcl2SensorData(const pcl::PointCloud<pcl::PointXYZL>::Ptr data, AMCLLaserData& target);
  bool CheckPoseSafety(const Eigen::Vector3d& amcl_pose,
                       const pf_vector_t& gt_delta,
                       const double ts);



  
};


}
