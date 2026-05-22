#include "modules/perception/lidar_tracking/fast_tracker/fast_engine.h"
#include "modules/perception/lidar_tracking/fast_tracker/config/lidar_track_config.h"
#include "modules/perception/lidar_tracking/fast_tracker/ekf/lidar_track_ekf_filter.h"
#include "modules/perception/lidar_tracking/fast_tracker/util/lidar_track_util.h"
namespace century {
namespace perception {
namespace lidar {
bool TrackerEngine::Init(const MultiTargetTrackerInitOptions& options) {
  (void)options;
  b_is_track_init_ = false;
  const std::string config_file_path = options.config_path +"/"+options.config_file;
  Track::Config::ConfigParser config_parser(config_file_path);
  config_ = *config_parser.GetLidarTrackConfig();
  if(config_parser.GetLidarTrackConfig()) {
    filter_.reset(new Track::Ekf::EkfMultiObjectTracking(config_));
  }
  return true;

}

bool TrackerEngine::DetectObjects2GlobMeasurements(const Track::ros_interface::DetectObjects3D& lidar_objects,
                                                    Track::Ekf::Meastructs& o_glob_lidar_measurements) {
  int i_det_size = lidar_objects.object.size();
  o_glob_lidar_measurements.meas.resize(i_det_size);
  o_glob_lidar_measurements.time_stamp = lidar_objects.header.stamp;

  for (int i = 0; i < i_det_size; i++) {
    // Type conversion from ros interface to mc_mot interface
    Track::Util::ConvertDetectObjectToMeastruct(lidar_objects.object[i], o_glob_lidar_measurements.meas[i]);

    // 1. Measured angle based time compensation.
    if (config_.cal_detection_individual_time == true) {
      Track::Util::AngleBasedTimeCompensation(o_glob_lidar_measurements.meas[i], config_);
    }
  }
  return true;

}

bool TrackerEngine::ConvertToObject(const LidarFrame* frame, Track::ros_interface::DetectObjects3D& lidar_objects) {

  lidar_objects.header.frame_id = "vehicle";
  lidar_objects.header.stamp = frame->timestamp;
  
  for(size_t i=0 ; i<frame->segmented_objects.size(); i++) {
    auto cur_frame = frame->segmented_objects[i];
    Track::ros_interface::DetectObject3D object;
    object.id = cur_frame->id;
    object.classification = (type_mapping_.count(cur_frame->type) == 1) ? 
                            type_mapping_[cur_frame->type] : Track::ros_interface::ObjectClass::UNKNOWN;
    object.dimension.length = cur_frame->size(0);
    object.dimension.width = cur_frame->size(1);
    object.dimension.height = cur_frame->size(2);
    auto global_anchor_point =  frame->lidar2world_pose  * cur_frame->anchor_point;
    
    auto global_direction = frame->lidar2world_pose .rotation()* cur_frame->direction.cast<double>();
    object.state.x = global_anchor_point(0);
    object.state.y = global_anchor_point(1);
    object.state.z = global_anchor_point(2);
    object.state.yaw = atan2(global_direction(1), global_direction(0));
    lidar_objects.object.push_back(object);
  }
  return true;
}
bool TrackerEngine::Track(const MultiTargetTrackerOptions& options, LidarFrame* frame) {
  (void)options;
  if (!filter_) {
    return false;
  }
  Track::ros_interface::DetectObjects3D lidar_objects;
  if(!ConvertToObject(frame, lidar_objects)) {
    return false;
  }
  if(b_is_track_init_) {
    double dt = lidar_objects.header.stamp - last_predicted_time_;
    filter_->RunPrediction(dt);
    last_predicted_time_ = lidar_objects.header.stamp;
  }

  Track::Ekf::Meastructs meas_structs;


    // when using localization source and using global coordinate tracking.
    // convert local coordinate detection into global coordinate.
  if (DetectObjects2GlobMeasurements(lidar_objects, meas_structs) == false) {
    std::cout << "CANNOT CONVERT EGO TO GLOBAL. NO LIDAR STATES"<< std::endl;
    return false;
  }
  

  // Run measurment update in algorithm
  filter_->RunUpdate(meas_structs);

  if(b_is_track_init_ == false){
    b_is_track_init_ = true;
    last_predicted_time_ = lidar_objects.header.stamp;
    std::cout  << "Init Tracker"<< std::endl;
  }

  Track::Ekf::TrackStructs mot_track_structs = filter_->GetTrackResults();

  // fill in frame
  frame->tracked_objects.resize(mot_track_structs.track.size());
  for (size_t i = 0; i < mot_track_structs.track.size(); i++) {
    auto& cur_obj = frame->tracked_objects.at(i);
    cur_obj->track_id = mot_track_structs.track[i].track_id;
    cur_obj->type = from_type_mapping_.count(mot_track_structs.track[i].classification) == 1 ? 
                    from_type_mapping_.at(mot_track_structs.track[i].classification) :  century::perception::base::ObjectType::UNKNOWN;
    cur_obj->anchor_point(0) = mot_track_structs.track[i].state_vec(0);
    cur_obj->anchor_point(1) = mot_track_structs.track[i].state_vec(1);
    cur_obj->anchor_point(2) = 0.0;
    cur_obj->direction(0) = cos(mot_track_structs.track[i].state_vec(3));
    cur_obj->direction(1) = sin(mot_track_structs.track[i].state_vec(3));
    cur_obj->direction(2) = 0.0;
    cur_obj->size(0) = mot_track_structs.track[i].dimension.length;
    cur_obj->size(1) = mot_track_structs.track[i].dimension.width;
    cur_obj->size(2) = mot_track_structs.track[i].dimension.height;
    cur_obj->velocity(0) = mot_track_structs.track[i].state_vec(4);
    cur_obj->velocity(1) = mot_track_structs.track[i].state_vec(5);
  }

  return true;

}
 
  
  

}
}
}  