#pragma once 
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/lidar_tracking/io/velocity_filter.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>
namespace century {
namespace perception {
namespace lidar {
using onboard::SensorFrameMessage;
using onboard::LidarFrameMessage;
using century::perception::PerceptionObstacles;
using century::prediction::PredictionObstacles;
using century::localization::LocalizationEstimate;

using PerceptionObstacleptr = std::shared_ptr<PerceptionObstacles>;
using PredictionObstaclePtr = std::shared_ptr<PredictionObstacles>;
using PredictionObstacleReaderPtr =
    std::shared_ptr<century::cyber::Reader<PredictionObstacles>>;
using PerceptionObstacleReaderPtr =
    std::shared_ptr<century::cyber::Reader<PerceptionObstacles>>;
using LidarFrameMessagePtr = std::shared_ptr<century::perception::onboard::SensorFrameMessage>;
using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
using LocalizationReaderPtr =
    std::shared_ptr<century::cyber::Reader<LocalizationEstimate>>;
class LidarFrameGenerator {
 public:
  LidarFrameGenerator() = default;
  ~LidarFrameGenerator() = default;
  explicit LidarFrameGenerator(const Eigen::Affine3d& extrinsic, 
                               const double duplicated_distance_thres,
                               const double using_point_cloud_polygon_thres,
                               const double min_length_threshold,
                               const double height_diff_threshold,
                               const bool local_coordinate,
                               const bool use_detector_pose): 
                              extrinsic_(extrinsic), 
                              duplicated_distance_thres_(duplicated_distance_thres), 
                              using_point_cloud_polygon_thres_(using_point_cloud_polygon_thres),
                              min_length_threshold_(min_length_threshold), 
                              height_diff_threshold_(height_diff_threshold), 
                              is_local_cooridinate_(local_coordinate),
                              use_detector_pose_(use_detector_pose) {}

  void Reset() {
    localization_estimate_traj_.clear();
  }

  void SplicePerceptionData(std::list<PerceptionObstacleptr>& prediction_data) {
    perception_data_traj_.splice(perception_data_traj_.end(), prediction_data);
  }

  void SpliceLocalizationEstimate(std::list<LocalizationPtr>& localization_estimate) {
    if(!visual_anchor_point_) {
      visual_anchor_point_ = localization_estimate.front();
    }
    localization_estimate_traj_.splice(localization_estimate_traj_.end(),
                                       localization_estimate);
    // only keep 10s localization data
    if(localization_estimate_traj_.empty()) {
      return;
    }
    const uint32_t max_dr_size = 1200;
    while(localization_estimate_traj_.size() > max_dr_size) {
      localization_estimate_traj_.pop_front();
    }
    return;
  }

  const LocalizationPtr GetVisualAnchorPoint() const {
    return visual_anchor_point_;
  }
  std::shared_ptr<LidarFrameMessage> GenerateFrame() ;
  private:
    LocalizationPtr visual_anchor_point_ = nullptr;
    std::list<LocalizationPtr> localization_estimate_traj_;
    std::list<PerceptionObstacleptr> perception_data_traj_; 
    Eigen::Matrix3d  GetRotationMatrix(double yaw, double pitch, double roll);
    std::shared_ptr<Eigen::Affine3d> SyncLoclizationEstimate(const double ts);
    std::shared_ptr<Eigen::Affine3d> CreateTransformation(const LocalizationPtr dr);
    Eigen::Matrix3d QuaternionToRotationMatrix(const double x, const double y, const double z, const double w);
    Eigen::Affine3d extrinsic_ = Eigen::Affine3d::Identity(); // lidar to vehicle
    Eigen::Affine3d transform_ = Eigen::Affine3d::Identity();
    double duplicated_distance_thres_ = 0.0;
    double using_point_cloud_polygon_thres_ = -1.0;
    double min_length_threshold_ = 3.0;
    double height_diff_threshold_ = 0.5;
    bool is_local_cooridinate_ = false;
    bool use_detector_pose_ = false;
    std::unordered_map<century::perception::PerceptionObstacle::Type, century::perception::base::ObjectType> type_mapping = {
      {century::perception::PerceptionObstacle::PEDESTRIAN, century::perception::base::ObjectType::PEDESTRIAN},
      {century::perception::PerceptionObstacle::BICYCLE, century::perception::base::ObjectType::BICYCLE},
      {century::perception::PerceptionObstacle::VEHICLE, century::perception::base::ObjectType::VEHICLE},
      {century::perception::PerceptionObstacle::CONE, century::perception::base::ObjectType::CONE},
      {century::perception::PerceptionObstacle::FORKLIFT_STACKER, century::perception::base::ObjectType::FORKLIFT_STACKER},
      {century::perception::PerceptionObstacle::STACKER, century::perception::base::ObjectType::STACKER},
      {century::perception::PerceptionObstacle::WHEELCRANE, century::perception::base::ObjectType::WHEELCRANE},

    };
   
    std::unordered_map<century::perception::PerceptionObstacle::SubType, century::perception::base::ObjectSubType> sub_type_mapping = {
      {century::perception::PerceptionObstacle::ST_CAR, century::perception::base::ObjectSubType::CAR},
      {century::perception::PerceptionObstacle::ST_VAN, century::perception::base::ObjectSubType::VAN},
      {century::perception::PerceptionObstacle::ST_TRUCK, century::perception::base::ObjectSubType::TRUCK},
      {century::perception::PerceptionObstacle::ST_BUS, century::perception::base::ObjectSubType::BUS},
      {century::perception::PerceptionObstacle::ST_CYCLIST, century::perception::base::ObjectSubType::CYCLIST},
      {century::perception::PerceptionObstacle::ST_MOTORCYCLIST, century::perception::base::ObjectSubType::MOTORCYCLIST},
      {century::perception::PerceptionObstacle::ST_TRICYCLIST, century::perception::base::ObjectSubType::TRICYCLIST},
      {century::perception::PerceptionObstacle::ST_PEDESTRIAN, century::perception::base::ObjectSubType::PEDESTRIAN},
      {century::perception::PerceptionObstacle::ST_TRAFFICCONE, century::perception::base::ObjectSubType::TRAFFICCONE}
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void SensorFrameMessagePostProcess(std::shared_ptr<SensorFrameMessage>& out_message, const double using_point_cloud_polygon_thres);

}
}
}