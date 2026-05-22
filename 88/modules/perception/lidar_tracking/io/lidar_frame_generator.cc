#include "modules/perception/lidar_tracking/io/lidar_frame_generator.h"
#include "modules/perception/lidar_tracking/io/duplicated_object_filter.h"
#include "modules/perception/lidar_tracking/io/bbx_build.h"

#include <limits>
namespace century {
namespace perception {
namespace lidar {


Eigen::Matrix3d  LidarFrameGenerator::GetRotationMatrix(double yaw, double pitch, double roll) {  
  Eigen::Matrix3d Rz;
  Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw),  cos(yaw), 0,
        0,           0,          1;
  Eigen::Matrix3d Ry;
  Ry << cos(pitch),  0,  sin(pitch),
        0,          1,          0,
        -sin(pitch), 0,  cos(pitch);


  Eigen::Matrix3d Rx;
  Rx << 1,          0,           0,
        0,  cos(roll), -sin(roll),
        0,  sin(roll),  cos(roll);

  Eigen::Matrix3d R = Rz * Ry * Rx;
  return R;
}

Eigen::Matrix3d LidarFrameGenerator::QuaternionToRotationMatrix(const double x, const double y, const double z, const double w) {

  Eigen::Matrix3d rotationMatrix;
  rotationMatrix(0, 0) = 1 - 2 * y * y - 2 * z * z;
  rotationMatrix(0, 1) = 2 * x * y - 2 * w * z;
  rotationMatrix(0, 2) = 2 * x * z + 2 * w * y;

  rotationMatrix(1, 0) = 2 * x * y + 2 * w * z;
  rotationMatrix(1, 1) = 1 - 2 * x * x - 2 * z * z;
  rotationMatrix(1, 2) = 2 * y * z - 2 * w * x;

  rotationMatrix(2, 0) = 2 * x * z - 2 * w * y;
  rotationMatrix(2, 1) = 2 * y * z + 2 * w * x;
  rotationMatrix(2, 2) = 1 - 2 * x * x - 2 * y * y;

  return rotationMatrix;
}
std::shared_ptr<LidarFrameMessage> LidarFrameGenerator::GenerateFrame() {
  if(!use_detector_pose_ && localization_estimate_traj_.empty()) {
    AINFO << "localization_estimate_traj is empty!" << std::endl;
    return nullptr;
  }
  if (perception_data_traj_.empty()) {
    AINFO << "perception_data_traj_ is empty!" << std::endl;
    return nullptr;
  }
  auto perception_data = perception_data_traj_.front();
  if (!use_detector_pose_) {
    if(perception_data->header().timestamp_sec() > localization_estimate_traj_.back()->header().timestamp_sec()) {
      AINFO << std::fixed<< "perception_data timestamp "<<perception_data->header().timestamp_sec() 
                <<" is bigger than localization_estimate timestamp " 
                << localization_estimate_traj_.back()->header().timestamp_sec() << std::endl;
      if((perception_data->header().timestamp_sec() - localization_estimate_traj_.back()->header().timestamp_sec()) > 5.0) {
        perception_data_traj_.clear();
      }
      return nullptr;
    }
  }

  perception_data_traj_.pop_front();

  auto lidar_frame = std::make_shared<LidarFrameMessage>();
  lidar_frame->source_perception_obstacles_ = perception_data;

  lidar_frame->lidar_frame_.reset(new LidarFrame());
  std::shared_ptr<Eigen::Affine3d> lidar_world_pose = nullptr;
  if (!use_detector_pose_) {
    lidar_world_pose = SyncLoclizationEstimate(perception_data->header().timestamp_sec());
  } else {
    auto& transform = perception_data->lidar2world();
    lidar_world_pose.reset(new Eigen::Affine3d());
    *lidar_world_pose = Eigen::Affine3d::Identity();
    // Set translation
    const double dr_z_zero = 0.0; // using 0.0 instead of dr->pose().position().z()
    lidar_world_pose->translation() = Eigen::Vector3d(transform.tx(), 
                                            transform.ty(), 
                                            dr_z_zero);
    // Set rotation 
    lidar_world_pose->rotate(QuaternionToRotationMatrix(transform.qx(),
                                                transform.qy(), 
                                                transform.qz(), 
                                                transform.qw()));
  }
  
  if(!lidar_world_pose) {
    AINFO << std::fixed << "fetch lidar_world_pose failed! " 
              << perception_data->header().timestamp_sec() << std::endl;
    return nullptr;
  }
  lidar_frame->lidar_frame_->lidar2world_pose = *lidar_world_pose;
  lidar_frame->timestamp_ = perception_data->header().timestamp_sec();
  lidar_frame->lidar_frame_->timestamp = perception_data->header().timestamp_sec();
  //lidar_frame->lidar_frame_->lidar2novatel_extrinsics = extrinsic_;

  BBXBuilderMatcher::GetInstance()->Reset(using_point_cloud_polygon_thres_, min_length_threshold_,height_diff_threshold_);

  for(int i=0; i< perception_data->perception_obstacle_size(); i++) {
    static int32_t n_counter = 0;
    auto obstacle = perception_data->perception_obstacle(i);
    std::shared_ptr<base::Object> object;
    object.reset(new base::Object());
  
    Eigen::Affine3d global_pose,local_pose;
    object->anchor_point = is_local_cooridinate_ ?  
                            Eigen::Vector3d(obstacle.position().x(), obstacle.position().y(), obstacle.position().z()):
                            lidar_frame->lidar_frame_->lidar2world_pose.inverse()*
                            Eigen::Vector3d(obstacle.position().x(), obstacle.position().y(), obstacle.position().z());
    Eigen::Matrix3d rotation_matrix = lidar_frame->lidar_frame_->lidar2world_pose.rotation().inverse() *
                                      GetRotationMatrix(obstacle.theta(),0,0);
    double local_yaw = is_local_cooridinate_ ? obstacle.theta() : std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    object->id = n_counter++;
    auto local_center =  Eigen::Vector3d(object->anchor_point[0], object->anchor_point[1], object->anchor_point[2]);
    object->center = local_center;
    object->size = Eigen::Vector3f(obstacle.length(), obstacle.width(), obstacle.height());
    object->lidar_supplement.is_background = false;
    object->size(0) = obstacle.length();
    object->size(1) = obstacle.width();
    object->size(2) = obstacle.height();
    object->theta = local_yaw;
    object->direction(0) = cos(local_yaw);
    object->direction(1) = sin(local_yaw);
    object->direction(2) = 0;
    object->lidar_supplement.height_above_ground = obstacle.height_above_ground();

    object->polygon.clear();
    object->polygon.reserve(obstacle.polygon_point_size());
    for(int j=0; j< obstacle.polygon_point_size(); j++) {
      auto cur_point = is_local_cooridinate_ ? Eigen::Vector3d(obstacle.polygon_point(j).x(), 
                                                                   obstacle.polygon_point(j).y(), 
                                                                   obstacle.polygon_point(j).z()) : 
                          lidar_frame->lidar_frame_->lidar2world_pose.inverse() * Eigen::Vector3d(obstacle.polygon_point(j).x(), 
                                                                                                  obstacle.polygon_point(j).y(), 
                                                                                                  obstacle.polygon_point(j).z());
      century::perception::base::PointD point{cur_point(0), cur_point(1), cur_point(2), 0.0};
      object->polygon.push_back(point);
    }

    object->type = type_mapping.count(obstacle.type()) ? type_mapping[obstacle.type()] : base::ObjectType::UNKNOWN;
    object->sub_type = sub_type_mapping.count(obstacle.sub_type()) ? sub_type_mapping[obstacle.sub_type()] : base::ObjectSubType::UNKNOWN;
    lidar_frame->lidar_frame_->segmented_objects.push_back(object);
  }

  DuplicatedObjectFilter duplicate_filter(duplicated_distance_thres_);
  std::vector<BoundingBoxBuild> bbx_builders;
  for(const auto& iter : lidar_frame->lidar_frame_->segmented_objects) {
    if(iter->type !=  base::ObjectType::UNKNOWN && iter->type !=  base::ObjectType::WHEELCRANE) {
      duplicate_filter.AddPoint(Eigen::Vector3d(iter->center(0), iter->center(1), iter->theta), iter->size(0), iter->size(1),
                                iter->type == base::ObjectType::PEDESTRIAN);
    } else {
      BBXBuilderMatcher::GetInstance()->FeedUpBBX(iter->id, iter->size(2), iter->polygon, iter->center);
    }
  }
  for(auto& iter : lidar_frame->lidar_frame_->segmented_objects) {
    
    if(iter->type < base::ObjectType::VEHICLE) { 
      continue;
    }
    BBXBuilderMatcher::GetInstance()->MatchResult(iter->id, iter->center, iter->size, iter->theta);
  }
  BBXBuilderMatcher::GetInstance()->Update();
  auto& vec = lidar_frame->lidar_frame_->segmented_objects;
  vec.erase(std::remove_if(vec.begin(), vec.end(),
            [&](century::perception::base::ObjectPtr iter) {
              return ((iter->type ==  base::ObjectType::UNKNOWN) && 
                       BBXBuilderMatcher::GetInstance()->GetMatchedObstacleFromCluster(iter->id) == -1 &&
                      (duplicate_filter.IsDuplicated(Eigen::Vector2d(iter->center(0), iter->center(1)))));
             }), vec.end());

  if(lidar_frame->lidar_frame_->segmented_objects.empty()) {
    AINFO << "lidar_frame->lidar_frame_->segmented_objects is empty" << std::endl;
  //  return nullptr;
  }

  return lidar_frame;
}


std::shared_ptr<Eigen::Affine3d> LidarFrameGenerator::CreateTransformation(const LocalizationPtr dr) {
  if(!dr) {
    AINFO << "dr is null" << std::endl;
    return nullptr;
  }
  std::shared_ptr<Eigen::Affine3d> transform = nullptr;
  transform.reset(new Eigen::Affine3d());
  *transform = Eigen::Affine3d::Identity();
  // Set translation
  const double dr_z_zero = 0.0; // using 0.0 instead of dr->pose().position().z()
  transform->translation() = Eigen::Vector3d(dr->pose().position().x(), 
                                          dr->pose().position().y(), 
                                          dr_z_zero);
  // Set rotation 
  transform->rotate(QuaternionToRotationMatrix(dr->pose().orientation().qx(),
                                              dr->pose().orientation().qy(), 
                                              dr->pose().orientation().qz(), 
                                              dr->pose().orientation().qw())); 
  return transform;
}

std::shared_ptr<Eigen::Affine3d> LidarFrameGenerator::SyncLoclizationEstimate(double object_time) {
  if(localization_estimate_traj_.empty()) {
    AINFO << "localization_estimate_traj_ is empty" << std::endl;
    return nullptr;
  }
  auto object_synced_state = localization_estimate_traj_.back();
  double minimum_time_diff = std::numeric_limits<float>::max();

  for (auto& lidar_state : localization_estimate_traj_) {
    if (fabs(object_time - lidar_state->header().timestamp_sec()) < minimum_time_diff) {
      minimum_time_diff = fabs(object_time - lidar_state->header().timestamp_sec());
      object_synced_state = lidar_state;
    }

    if (minimum_time_diff < std::numeric_limits<float>::min()) {
      break;
    }
  }
  // to do interpolate ..
  AINFO << std::fixed << "dr | lidar time: " << object_synced_state->header().timestamp_sec() << " | " << object_time << std::endl;
  return CreateTransformation(object_synced_state);

}


void SensorFrameMessagePostProcess(std::shared_ptr<SensorFrameMessage>& out_message, const double using_point_cloud_polygon_thres) {
  if(!out_message) {
    return;
  }
  
  for(auto& iter : out_message->frame_->objects) {
    const double vel_filter_thres = (iter->type == base::ObjectType::PEDESTRIAN) ? 0.6 : 0.8;
    if(iter->type == base::ObjectType::VEHICLE || iter->type == base::ObjectType::STACKER
              || iter->type == base::ObjectType::FORKLIFT_STACKER || iter->type == base::ObjectType::PEDESTRIAN
              || iter->type == base::ObjectType::WHEELCRANE) {
      bool is_static_hypothesis = VelocityFilter::GetInstance()->FliterStaticHypothesis(iter->track_id, 
                                            Eigen::Vector2d(iter->center(0), iter->center(1)), 
                                            out_message->frame_->timestamp);
      if(is_static_hypothesis && (iter->velocity.head<2>().norm() < vel_filter_thres)) {
        AINFO << "set velocity to zero for obstacle id: " << iter->track_id
              << " with low velocity: " << iter->velocity(0) << ", " << iter->velocity(1) << std::endl;
        iter->velocity.setZero();
      }
    }
  }

  if(using_point_cloud_polygon_thres < 0.0) {
    return;
  }
  
  for(auto& iter : out_message->frame_->objects) {
    if(iter->type == base::ObjectType::VEHICLE) {
      auto cluster_id = BBXBuilderMatcher::GetInstance()->GetMatchedClusterFromObstacle(iter->id);
      if(cluster_id != -1) {
        for(auto& cluster_object : out_message->frame_->objects) {
          if((cluster_object->id == cluster_id) && (cluster_object->type == base::ObjectType::UNKNOWN) &&
            (abs(cluster_object->velocity(0)) < 1e-6 && abs(cluster_object->velocity(1)) < 1e-6)) {
            AINFO << "matched cluster object id: " << cluster_object->id 
                  << " with obstacle id: " << iter->id 
                  << " as cluster object velocity: " << cluster_object->velocity(0) 
                  << ", " << cluster_object->velocity(1) 
                  << " instead of obstacle velocity: " << iter->velocity(0) << ", " << iter->velocity(1) << std::endl;
            iter->velocity = cluster_object->velocity;
          }
        }
      }
    }
  }

  auto& vec = out_message->frame_->objects;
  vec.erase(std::remove_if(vec.begin(), vec.end(),
  [&](century::perception::base::ObjectPtr iter) {
    return ((iter->type == base::ObjectType::UNKNOWN) && 
             BBXBuilderMatcher::GetInstance()->GetMatchedObstacleFromCluster(iter->id) != -1);
   }), vec.end());
}

}
}
}
