#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "modules/perception/lidar_tracking/interface/base_multi_target_tracker.h"
#include "modules/perception/lidar_tracking/fast_tracker/lidar_track_type.h"
#include "modules/perception/lidar_tracking/fast_tracker/ekf/lidar_track_ekf_filter.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
namespace century {
namespace perception {
namespace lidar {

class TrackerEngine : public BaseMultiTargetTracker {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
  TrackerEngine() = default;
  virtual ~TrackerEngine() = default;
  /**
   * @brief Init mlf engine
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MultiTargetTrackerInitOptions& options =
                  MultiTargetTrackerInitOptions()) override;

  /**
   * @brief Track segmented objects from multiple lidar sensors
   *
   * @param options tracker options
   * @param frame lidar frame
   * @return true
   * @return false
   */
  bool Track(const MultiTargetTrackerOptions& options,
              LidarFrame* frame) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "FastEngine"; };

  private:
  std::shared_ptr<Track::Ekf::EkfMultiObjectTracking> filter_;
  bool ConvertToObject(const LidarFrame* frame, Track::ros_interface::DetectObjects3D& objects);
  bool DetectObjects2GlobMeasurements(const Track::ros_interface::DetectObjects3D& lidar_objects,
                                        Track::Ekf::Meastructs& o_glob_lidar_measurements);
  bool b_is_track_init_ = false;        
  double last_predicted_time_ = 0.0;
  Track::Ekf::MultiClassObjectTrackingConfig config_;

  std::unordered_map<century::perception::base::ObjectType, Track::ros_interface::ObjectClass> type_mapping_ = {
    {century::perception::base::ObjectType::PEDESTRIAN, Track::ros_interface::ObjectClass::PEDESTRIAN},
    {century::perception::base::ObjectType::BICYCLE, Track::ros_interface::ObjectClass::BICYCLE},
    {century::perception::base::ObjectType::VEHICLE, Track::ros_interface::ObjectClass::TRUCK}
  };

  std::unordered_map<Track::Ekf::ObjectClass, century::perception::base::ObjectType> from_type_mapping_ = {
    {Track::Ekf::ObjectClass::PEDESTRIAN, century::perception::base::ObjectType::PEDESTRIAN },
    {Track::Ekf::ObjectClass::BICYCLE, century::perception::base::ObjectType::BICYCLE },
    {Track::Ekf::ObjectClass::TRUCK, century::perception::base::ObjectType::VEHICLE }
  };

}  ;
}
}
}