#include "amcl_worker.h"
#include "common/pose_utils.h"
#include <glog/logging.h>
#include "cyber/common/file.h"
#include "modules/perception/landmark_loc/proto/amcl_map.pb.h"
//extern std::vector<pf_vector_t> test_points;
namespace semantic_mapping {
namespace {
static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}
double quat2yaw(const Eigen::Quaterniond& q)
{
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

}

std::vector<std::pair<int,int> > AmclWorker::free_space_indices;

void AmclWorker::SetInitPose(const Eigen::Matrix4d& init) {
  LOG(INFO) << "set init pose Pose ";
  auto init_pose_matrix = origin_.inverse() * base_.inverse() * init;
  auto init_pose = PoseUtils::EigenMatToVector3d(init_pose_matrix);
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose[0];
  pf_init_pose_mean.v[1] = init_pose[1];
  pf_init_pose_mean.v[2] = init_pose[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = x_convariance_/2;
  pf_init_pose_cov.m[1][1] = y_convariance_/2;
  pf_init_pose_cov.m[2][2] = yaw_convariance_/2;
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pose_safety_checker_.Reset();
  last_pose_eval_ts_ = 0.0;
}

pf_vector_t AmclWorker::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;

  return p;
}


void AmclWorker::HandleMapMessage() {

  //free space detect
  sensor_handler_ = new AMCLLaser(10, map_);
  free_space_indices.resize(0);
  LOG(INFO) << "map size: " << map_[0]->size_x << " " << map_[0]->size_y;
  for(int i = 0; i < map_[0]->size_x; i++) {
    for(int j = 0; j < map_[0]->size_y; j++) {
      bool is_freespace = true;
      for(const auto& it : map_) {
        if(it->cells[MAP_INDEX(it,i,j)].occ_state != 0) {
          is_freespace = false;
        }
      }
      if(is_freespace) {
        free_space_indices.push_back(std::make_pair(i,j));
      }
    }  
  }
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow, alpha_fast,
    (pf_init_model_fn_t)AmclWorker::uniformPoseGenerator, (void *)map_[0]);
  pf_set_selective_resampling(pf_, selective_resampling_);
  pf_->pop_err = pf_err;
  pf_->pop_z = pf_z;

  odom_ = new AMCLOdom();
  odom_->SetModel(odom_model_type, alpha1, alpha2, alpha3, alpha4, alpha5);

  sensor_handler_->SetModelLikelihoodField(z_hit, z_rand, sigma_hit);
  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose_.position(0);
  pf_init_pose_mean.v[1] = last_published_pose_.position(1);
  Eigen::Matrix3d R = last_published_pose_.rotation.toRotationMatrix();
  pf_init_pose_mean.v[2] =  std::atan2(R(1,0), R(0,0));
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose_.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose_.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose_.covariance[6*5+5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pose_safety_checker_.Reset();
  last_pose_eval_ts_ = 0.0;
  
  visualizer_ = need_view_ ? (new MapShow(map_, origin_)) : nullptr;
 // visualizer_->draw_occ_dist(POINT_LABEL::Slotline);
}



bool AmclWorker::Init(std::vector<std::tuple<double, Eigen::Matrix4d>>& traj) {
  traj_.swap(traj);
  //for(const auto& it : traj_) {
  //  LOG(DBG) << std::fixed << "traj pose: " << std::get<1>(it).transpose() << " at " << std::get<0>(it);
  //}
  // load map
  century::perception::landmark_loc::AmclMap amcl_map;
  if(!century::cyber::common::GetProtoFromBinaryFile(map_save_path_ + "/amcl_map.pb.bin", &amcl_map)) {
    std::cout << "Failed to load amcl map from " << map_save_path_ + "/amcl_map.pb.bin" << std::endl;
    return false;
  }

  origin_ = PoseUtils::Vector3dToEigenMat(Eigen::Vector3d(amcl_map.origin_x(), amcl_map.origin_y(), 0.0));
  base_ = PoseUtils::Vector3dToEigenMat(Eigen::Vector3d(amcl_map.base_x(), amcl_map.base_y(),amcl_map.base_yaw()));  

  for(int i=0; i< amcl_map.layer_size(); ++i) {
    map_.at(i) = map_alloc();
    auto* map = map_.at(i);
    map->size_x = amcl_map.width();
    map->size_y = amcl_map.height();
    map->scale = amcl_map.scale();
    map->origin_x = amcl_map.origin_x();
    map->origin_y = amcl_map.origin_y();
    map->max_occ_dist = amcl_map.max_occ_distance();
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);
    memset( map->cells, 0, sizeof(map_cell_t) * map->size_x * map->size_y);
    for(int x=0; x< map->size_x; ++x) {
      for(int y=0; y< map->size_y; ++y) {
        int index = x + y * map->size_x;
        map->cells[index].occ_state = amcl_map.layer(i).cells(index).occ_state();
        map->cells[index].occ_dist = amcl_map.layer(i).cells(index).occ_distance();
      }
    }

  }

  return true;
}

bool AmclWorker::FetchDrPose(const double& ts, Eigen::Matrix4d& dr) {
  //const auto base = std::get<1>(traj_.at(0));
  for(const auto& it : traj_) {
    if(ts < std::get<0>(it)) {
      dr =  origin_.inverse()* base_.inverse() * std::get<1>(it);
     // std::cout << "fetch dr pose in traj at "<< std::fixed << ts << " as "<< std::get<1>(it).transpose() << std::endl;
      return true;
    }
  }
  std::cout << "not fetch dr pose in traj at "<< std::fixed << ts << std::endl; 
  return false;

}

bool AmclWorker::ConvertPcl2SensorData(const pcl::PointCloud<pcl::PointXYZL>::Ptr data, AMCLLaserData& target) {
  for(int i=0; i< target.range_count; ++i) {
    double x = data->points[i].x;
    double y = data->points[i].y;
    target.ranges[i][0] = std::sqrt(x * x + y * y);
    target.ranges[i][1] = std::atan2(y, x);
    target.labels[i] = 255 - data->points[i].label;
   // LOG(INFO) << "point: " << x << " " << y << " range: " << target.ranges[i][0] << " bearing: " << target.ranges[i][1] << " label: " << target.labels[i];
    assert( target.labels[i] >= 0 &&  target.labels[i]< 16);
  }
  return true;
}

bool AmclWorker::NeedInit() {

  if(semantic_continue_lacked_ <= 10) {
    if(target_semantic_count_ > 80) {
      semantic_continue_lacked_ = 0;
    } else {
      semantic_continue_lacked_++;
    }
    return false;
  } else {
    if(target_semantic_count_ > 80) {
      semantic_continue_lacked_ = 0;
      LOG(INFO) << "AMCL need init recovered.";
     return true;
    } else {
      semantic_continue_lacked_++;
      return false;
    }
  }

}

bool AmclWorker::CheckPoseSafety(const Eigen::Vector3d& amcl_pose,
                                 const pf_vector_t& gt_delta,
                                 const double ts) {
  if (!enable_pose_safety_check_) {
    last_pose_eval_ts_ = ts;
    return true;
  }
  const double pose_eval_dt =
      last_pose_eval_ts_ > 1e-6 ? (ts - last_pose_eval_ts_) : 0.0;
  double abs_speed_mps = 0.0;
  if (pose_eval_dt > 1e-3) {
    abs_speed_mps = std::hypot(gt_delta.v[0], gt_delta.v[1]) / pose_eval_dt;
  }
  const Eigen::Vector3d pose_cov(last_published_pose_.covariance[6 * 0 + 0],
                                 last_published_pose_.covariance[6 * 1 + 1],
                                 last_published_pose_.covariance[6 * 5 + 5]);
  const auto safety_result =
      pose_safety_checker_.Evaluate(amcl_pose, ts, abs_speed_mps, pose_cov);
  if (!safety_result.accepted) {
    LOG(WARNING) << "Reject AMCL pose by jump safety check: "
                 << safety_result.reason
                 << ", dt=" << safety_result.dt_sec
                 << ", jump_xy=" << safety_result.translation_jump_m
                 << ", limit_xy=" << safety_result.translation_limit_m
                 << ", jump_yaw=" << safety_result.yaw_jump_rad
                 << ", limit_yaw=" << safety_result.yaw_limit_rad
                 << ", speed=" << abs_speed_mps
                 << ", cov=" << pose_cov.transpose()
                 << ", severe=" << safety_result.severe;
    return false;
  }
  last_pose_eval_ts_ = ts;
  return true;
}

std::shared_ptr<Eigen::VectorXd> AmclWorker::ReceiveSensorData(const Eigen::Matrix4d& odom, const Eigen::Matrix4d& dr,  
                                                               const pcl::PointCloud<pcl::PointXYZL>::Ptr old_sensor, 
                                                               const double ts){
  auto start = std::chrono::steady_clock::now();
  auto dr_odom = odom.isApprox(Eigen::Matrix4d::Identity()) ? dr : odom;
  if(!odom.isApprox(Eigen::Matrix4d::Identity())) {
    LOG(INFO) << "receiving odom:" << std::endl;
  }
  Eigen::Matrix4d dr_pose_matrix ;
  Eigen::Matrix4d gt_pose_matrix ;
  if(traj_.empty()) {
    dr_pose_matrix = origin_.inverse()* base_.inverse() * dr_odom;
    gt_pose_matrix = origin_.inverse()* base_.inverse() * dr;
  } else if(!FetchDrPose(ts, dr_pose_matrix)) {
    std::cout <<"not found pose for "<< std::fixed << ts << std::endl;
    return nullptr;
  }

  auto dr_pose = PoseUtils::EigenMatToVector3d(dr_pose_matrix);
  auto gt_pose = PoseUtils::EigenMatToVector3d(gt_pose_matrix);
  LOG(INFO) << std::fixed << "dr pose: " << dr_pose.transpose() << " at " << ts;

  pf_vector_t pose = pf_vector_zero();
  pose.v[0] = dr_pose[0];
  pose.v[1] = dr_pose[1];
  pose.v[2] = dr_pose[2];

  pf_vector_t delta = pf_vector_zero();
  pf_vector_t gt_delta = pf_vector_zero();
  if(pf_init_) {
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);
    bool update = fabs(delta.v[0]) > d_thresh || fabs(delta.v[1]) > d_thresh ||  fabs(delta.v[2]) > a_thresh;
    update = update || force_update_;
    gt_delta.v[0] = gt_pose[0] - pre_gt_pose_[0];
    gt_delta.v[1] = gt_pose[1] - pre_gt_pose_[1];
    gt_delta.v[2] = angle_diff(gt_pose[2], pre_gt_pose_[2]);
    if(!update) {
      std::cout <<"no necessary to update pf "<< std::endl;
      return nullptr;
    }
  }

  bool force_publication = true;
  if(!pf_init_) {
    pf_odom_pose_ = pose;
    pf_hyp_pose_ = pose;
    pf_init_ = true;
    resample_count_ = 0;
  } else {
    AMCLOdomData odata;
    odata.pose = pf_vector_coord_add(delta, pf_hyp_pose_);
    odata.delta = delta;
    if(!odom_->UpdateAction(pf_, (AMCLSensorData*)&odata)) {
      AINFO << "Failed to update pf with odom data";
      return nullptr;
    }
  }

  bool resampled = false;
  AMCLLaserData ldata(old_sensor->points.size());
  ConvertPcl2SensorData(old_sensor,ldata);
  ldata.sensor = sensor_handler_;
  ldata.sensor->pose = pose;
  sensor_handler_->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
  auto current_hyp_pose = pf_vector_coord_add(delta, pf_hyp_pose_);
  (void)(current_hyp_pose);
  pf_odom_pose_ = pose;
  pre_gt_pose_ = gt_pose;
  pf_update_resample(pf_);
  resampled = true;
  
  std::vector<Eigen::Vector3d> cloud_pose;
  if (visualizer_) {
    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    cloud_pose.resize(set->sample_count);
    std::unordered_map<int, int> cluster_cloud_count;
    for(int i=0;i<set->sample_count;i++) {
      cluster_cloud_count[set->samples[i].cluster_id]++;
      cloud_pose.push_back(Eigen::Vector3d(set->samples[i].pose.v[0], set->samples[i].pose.v[1], set->samples[i].pose.v[2]));
    }
    for(auto& c: cluster_cloud_count) {
      LOG(INFO) << "cluster id: " << c.first << " count: " << c.second;
    }
    visualizer_->AddPoselist(cloud_pose);
  }

  int hyp_count = 0;
  if(resampled || force_publication) {
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    target_semantic_count_ = 0;
    for(hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++) {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      int semantic_count = 0;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov, &semantic_count)) {
        LOG(INFO) << "Couldn't get stats on cluster" << hyp_count;
        break;
      }
      LOG(INFO) <<" Current weight is "<< std::fixed << weight << " Semantic count is "<< semantic_count;
      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;
      if(hyps[hyp_count].weight > max_weight) {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
        target_semantic_count_ = semantic_count;
      }
    }

    if(max_weight > 0.0) {  
      semantic_mapping::PoseWithConvariance p;
      p.position[0] = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.position[1] = hyps[max_weight_hyp].pf_pose_mean.v[1];
      p.rotation = Eigen::AngleAxisd(hyps[max_weight_hyp].pf_pose_mean.v[2], Eigen::Vector3d::UnitZ());
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++) {
        for(int j=0; j<2; j++) {
          p.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      p.covariance[6*5+5] = set->cov.m[2][2];

     // pose_pub_.publish(p);
      last_published_pose_ = p;
      pf_hyp_pose_.v[0] = p.position[0];
      pf_hyp_pose_.v[1] = p.position[1];
      pf_hyp_pose_.v[2] = quat2yaw(p.rotation);

      Eigen::Vector3d temp(pf_hyp_pose_.v[0],  pf_hyp_pose_.v[1],  pf_hyp_pose_.v[2]);                     
      std::vector<Eigen::Vector3d> points;
      points.resize(old_sensor->points.size());
      for(size_t i=0;i<old_sensor->points.size();i++) {
        Eigen::Vector3d pt(old_sensor->points[i].x, old_sensor->points[i].y, 0.0);
        auto map_pt = PoseUtils::EigenMatToVector3d(PoseUtils::Vector3dToEigenMat(temp) * PoseUtils::Vector3dToEigenMat(pt));
        points.push_back(map_pt);
      }
        
      if(visualizer_) {
        visualizer_->AddPathPoint(temp, target_semantic_count_ > 10); 
        visualizer_->AddSensorPoint(points);
        visualizer_->AddDrPoint(gt_pose);
      } 

      if(last_published_pose_.covariance[6*0+0] > x_convariance_ || 
          last_published_pose_.covariance[6*1+1] > x_convariance_ || 
          last_published_pose_.covariance[6*5+5] > yaw_convariance_) {
       LOG(INFO) << "=================AMCL pose covariance(" 
                 << last_published_pose_.covariance[6*0+0] << " "
                 << last_published_pose_.covariance[6*1+1] << " "
                 << last_published_pose_.covariance[6*5+5] 
                 << ")too large, not converged.======================";
       return nullptr; 
      }
      if (!CheckPoseSafety(temp, gt_delta, ts)) {
        return nullptr;
      }

      if(recorder_amcl_) {
        landmark_loc::AmclRecorder::GetInstance()->AddData(gt_pose, temp, ts,  cloud_pose, 
                          Eigen::Vector3d(last_published_pose_.covariance[6*0+0], 
                            last_published_pose_.covariance[6*1+1], 
                            last_published_pose_.covariance[6*5+5]),
                          target_semantic_count_, pf_->sets[pf_->current_set].cluster_count);
      }
      LOG(INFO) << "Semantic counts: " << target_semantic_count_ << " "
                << "Cluster counts: " << hyp_count << " "
                << "Max weight: " << max_weight << " "
                << "Mean: "<< hyps[max_weight_hyp].pf_pose_mean.v[0] << " "
                << hyps[max_weight_hyp].pf_pose_mean.v[1] << " "
                << hyps[max_weight_hyp].pf_pose_mean.v[2] << " "
                << "Cov: " << last_published_pose_.covariance[6*0+0] << " "
                << last_published_pose_.covariance[6*1+1] << " "
                << last_published_pose_.covariance[6*5+5];
      auto end = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      LOG(INFO)<< "Amcl time: " << duration.count() << " ms";
      ACHECK(temp(1) - gt_pose(1) < safety_dis_) << "Amcl failed DIFF too big " 
                                                 << temp(1) - gt_pose(1);
      const Eigen::Vector3d pose_2d = PoseUtils::EigenMatToVector3d(
          base_ * origin_ * PoseUtils::Vector3dToEigenMat(temp));

      auto result = std::make_shared<Eigen::VectorXd>(6);
      (*result) << pose_2d(0), pose_2d(1), pose_2d(2),
          last_published_pose_.covariance[6 * 0 + 0],
          last_published_pose_.covariance[6 * 1 + 1],
          last_published_pose_.covariance[6 * 5 + 5];
      return result;
    }
    else {
     LOG(INFO) <<"No pose published";
    }
  }
  return nullptr;
}


  
}
