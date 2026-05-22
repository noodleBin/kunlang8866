/****************************************************************************/
// Module:    lidar_track_util.cpp
// Description: lidar_track_util
//
// Authors: xiaxinrong
// Date: March 3th 2025
/****************************************************************************/
#include "modules/perception/lidar_tracking/fast_tracker/util/lidar_track_util.h"
#include "modules/perception/lidar_tracking/fast_tracker/ekf/lidar_track_ekf_filter.h"
#include <sys/stat.h>
namespace Track {
namespace Util {

  double CalculateYawDotProduct(const double& yaw1, const double& yaw2) {
    // Calculate direction vectors
    const double x1 = std::cos(yaw1);
    const double y1 = std::sin(yaw1);
    const double x2 = std::cos(yaw2);
    const double y2 = std::sin(yaw2);
  
    // Calculate dot product
    const double dot_product = x1 * x2 + y1 * y2;
    return dot_product;
  }
  
  double CalculateYawCrossProduct(const double& yaw1, const double& yaw2) {
    // Create unit direction vectors using the two yaw angles
    const double x1 = std::cos(yaw1);
    const double y1 = std::sin(yaw1);
    const double x2 = std::cos(yaw2);
    const double y2 = std::sin(yaw2);
  
    // Calculate the cross product of 2D vectors
    const double cross_product = x1 * y2 - y1 * x2;
  
    return cross_product;
  }

bool FileExists(const std::string& filename) {
  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0);
}

void ConvertDetectObjectToMeastruct(const Track::ros_interface::DetectObject3D& detect_object, Track::Ekf::Meastruct& meas) {
  meas.id = detect_object.id;
  meas.detection_confidence = detect_object.confidence_score;
  meas.classification = Track::Ekf::ObjectClass(detect_object.classification);

  meas.state.time_stamp = detect_object.state.header.stamp;
  meas.state.x = detect_object.state.x;
  meas.state.y = detect_object.state.y;
  meas.state.z = detect_object.state.z;
  meas.state.yaw = detect_object.state.yaw;

  meas.dimension.height = detect_object.dimension.height;
  meas.dimension.width = detect_object.dimension.width;
  meas.dimension.length = detect_object.dimension.length;
}
Track::Ekf::ObjectState GetSyncedLidarState(double object_time, 
                                        const std::deque<Track::Ekf::ObjectState>& deque_lidar_state) {
  Track::Ekf::ObjectState object_synced_state = deque_lidar_state.back();
  double minimum_time_diff = FLT_MAX;

  for (auto& lidar_state : deque_lidar_state) {
    if (fabs(object_time - lidar_state.time_stamp) < minimum_time_diff) {
      minimum_time_diff = fabs(object_time - lidar_state.time_stamp);
      object_synced_state = lidar_state;
    }

    if (minimum_time_diff < FLT_MIN) {
      break;
    }
  }

  // to do : need interpolate

  return object_synced_state;
}

Track::Ekf::ObjectState PredictNextState(const Track::Ekf::ObjectState& state_t_minus_1,
                                      const Track::Ekf::ObjectState& state_t) {
  Track::Ekf::ObjectState state_t_plus_1;

  // Calculate time interval
  double delta_t = state_t.time_stamp - state_t_minus_1.time_stamp;

  // Predict time
  state_t_plus_1.time_stamp = state_t.time_stamp + delta_t;

  // Predict position
  state_t_plus_1.x = state_t.x + state_t.v_x * delta_t + 0.5 * state_t.a_x * delta_t * delta_t;
  state_t_plus_1.y = state_t.y + state_t.v_y * delta_t + 0.5 * state_t.a_y * delta_t * delta_t;
  state_t_plus_1.z = state_t.z + state_t.v_z * delta_t + 0.5 * state_t.a_z * delta_t * delta_t;

  // Predict velocity
  state_t_plus_1.v_x = state_t.v_x + state_t.a_x * delta_t;
  state_t_plus_1.v_y = state_t.v_y + state_t.a_y * delta_t;
  state_t_plus_1.v_z = state_t.v_z + state_t.a_z * delta_t;

  // Maintain acceleration at T time
  state_t_plus_1.a_x = state_t.a_x;
  state_t_plus_1.a_y = state_t.a_y;
  state_t_plus_1.a_z = state_t.a_z;

  // Predict angle
  state_t_plus_1.roll = state_t.roll + state_t.roll_rate * delta_t;
  state_t_plus_1.pitch = state_t.pitch + state_t.pitch_rate * delta_t;
  state_t_plus_1.yaw = state_t.yaw + state_t.yaw_rate * delta_t;

  // Predict angular velocity
  state_t_plus_1.roll_rate = state_t.roll_rate;
  state_t_plus_1.pitch_rate = state_t.pitch_rate;
  state_t_plus_1.yaw_rate = state_t.yaw_rate;

  return state_t_plus_1;
}

void TransformMeasLiDAR2Global(Track::Ekf::Meastruct& i_meas,
                               const std::deque<Track::Ekf::ObjectState>& deque_lidar_state,
                               const std::vector<double>& T_dr_lidar) {
  Track::Ekf::ObjectState object_synced_state = GetSyncedLidarState(i_meas.state.time_stamp, deque_lidar_state);

//  std::cout << "dr pose " << std::fixed << std::setprecision(6) << object_synced_state.time_stamp <<" "
//            << object_synced_state.x << " " << object_synced_state.y << " " << object_synced_state.yaw << std::endl;
// consider extrinsic[2] as z value in the world
  Eigen::Affine3d world_to_dr_affine =
        CreateTransformation(object_synced_state.x, object_synced_state.y, T_dr_lidar.at(2), 0, 0,
                            object_synced_state.yaw);

  Eigen::Affine3d lidar_to_dr_affine =
        CreateTransformation(T_dr_lidar.at(0), T_dr_lidar.at(1), T_dr_lidar.at(2), 
                             T_dr_lidar.at(3), T_dr_lidar.at(4), T_dr_lidar.at(5));

  Eigen::Affine3d lidar_to_object_affine =
        CreateTransformation(i_meas.state.x, i_meas.state.y, i_meas.state.z, 0, 0, i_meas.state.yaw);

  Eigen::Affine3d world_to_object_affine = world_to_dr_affine * lidar_to_dr_affine * lidar_to_object_affine;
  Eigen::Vector3d world_to_object_translation = world_to_object_affine.translation();
  Eigen::Matrix3d world_to_object_rotation = world_to_object_affine.rotation();

  i_meas.state.x = world_to_object_translation(0);
  i_meas.state.y = world_to_object_translation(1);
  i_meas.state.z = world_to_object_translation(2);
  i_meas.state.yaw = atan2(world_to_object_rotation(1, 0), world_to_object_rotation(0, 0));
}

void AngleBasedTimeCompensation(Track::Ekf::Meastruct& i_meas,const Track::Ekf::MultiClassObjectTrackingConfig& config) {
  double object_angle_ego_ccw_rad, object_angle_behind_cw_rad;
  double dt_from_start_scan_sec;

  object_angle_ego_ccw_rad = atan2(i_meas.state.y, i_meas.state.x); // -pi ~ pi
  object_angle_behind_cw_rad = M_PI - object_angle_ego_ccw_rad;     // 0 ~ 2pi

  dt_from_start_scan_sec = config.lidar_rotation_period * (object_angle_behind_cw_rad / (2.0 * M_PI)); // 0 ~ 0.1 sec

  if (config.lidar_sync_scan_start == true) {
    i_meas.state.time_stamp += dt_from_start_scan_sec; // Using start packet lidar
  }
  else {
    i_meas.state.time_stamp -= (config.lidar_rotation_period - dt_from_start_scan_sec);
  }
}

Eigen::Affine3d CreateTransformation(double x, double y, double z, double roll,
                                                             double pitch, double yaw) {
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  // Set translation
  transform.translation() << x, y, z;

  // Set rotation (yaw -> pitch -> roll order)
  transform.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  transform.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

  return transform;
}

void DetectObjects2LocalMeasurements(Track::ros_interface::DetectObjects3D lidar_objects,
                                     Track::Ekf::Meastructs& o_local_lidar_measurements) {

  int i_det_size = lidar_objects.object.size();
  o_local_lidar_measurements.meas.resize(i_det_size);
  o_local_lidar_measurements.time_stamp = lidar_objects.header.stamp;

  for (int i = 0; i < i_det_size; i++) {
    ConvertDetectObjectToMeastruct(lidar_objects.object[i], o_local_lidar_measurements.meas[i]);
  }
}


void ConvertTrackGlobalToLocal(Track::Ekf::TrackStructs& track_structs, Track::Ekf::ObjectState synced_lidar_state, 
                              const bool global_track) {
  double cos_yaw = std::cos(synced_lidar_state.yaw);
  double sin_yaw = std::sin(synced_lidar_state.yaw);

  //double glob_lidar_vx = synced_lidar_state.v_x;
  //double glob_lidar_vy = synced_lidar_state.v_y;

  for (auto& track : track_structs.track) {
    if (track.is_init == true) {

      // Position in track
      double x_track = track.state_vec(S_X);
      double y_track = track.state_vec(S_Y);

      //double x_lidar, y_lidar;

      //double glob_rel_vx = 0.0;
      //double glob_rel_vy = 0.0;   

      if(global_track == false) { // Local Tracker. Compensate absolute vel, acc, yawrate. No Orienation change
        // Lidar coordinate position is same as track position
        double x_lidar = x_track;
        double y_lidar = y_track;

        track.state_vec(S_X) = x_lidar;
        track.state_vec(S_Y) = y_lidar;

        // Convert Global lidar state to local coordinate
        double local_lidar_vx = cos_yaw * synced_lidar_state.v_x + sin_yaw * synced_lidar_state.v_y;
        double local_lidar_vy = -sin_yaw * synced_lidar_state.v_x + cos_yaw * synced_lidar_state.v_y;

        double local_lidar_ax = cos_yaw * synced_lidar_state.a_x + sin_yaw * synced_lidar_state.a_y;
        double local_lidar_ay = -sin_yaw * synced_lidar_state.a_x + cos_yaw * synced_lidar_state.a_y;

        // Compensate velocity by yawrate
        double yaw_rate_lidar = synced_lidar_state.yaw_rate;
        double v_add_x = -yaw_rate_lidar * y_lidar;
        double v_add_y =  yaw_rate_lidar * x_lidar;

        // Compensate velocity
        track.state_vec(S_VX) = track.state_vec(S_VX) + local_lidar_vx + v_add_x;
        track.state_vec(S_VY) = track.state_vec(S_VY) + local_lidar_vy + v_add_y;

        // Compensate acceleration
        track.state_vec(S_AX) = track.state_vec(S_AX) + local_lidar_ax;
        track.state_vec(S_AY) = track.state_vec(S_AY) + local_lidar_ay;

        // Convert yaw rate
        track.state_vec(S_YAW_RATE) = track.state_vec(S_YAW_RATE) + yaw_rate_lidar;

      } else { // Global Tracker. Compensate orientation
        // Relative position from Lidar (convert to Lidar coordinates)
        double x_relative = x_track - synced_lidar_state.x;
        double y_relative = y_track - synced_lidar_state.y;

        double x_relative_measurement = track.pre_associated_measurement(0) - synced_lidar_state.x;
        double y_relative_measurement = track.pre_associated_measurement(1) - synced_lidar_state.y;

        // Relative measurement position in Lidar coordinates
        track.pre_associated_measurement(0) = cos_yaw * x_relative_measurement + sin_yaw * y_relative_measurement;
        track.pre_associated_measurement(1) = -sin_yaw * x_relative_measurement + cos_yaw * y_relative_measurement;
        track.pre_associated_measurement(2) = track.pre_associated_measurement(2) - synced_lidar_state.yaw;

        // Relative position in Lidar coordinates
        double x_lidar = cos_yaw * x_relative + sin_yaw * y_relative;
        double y_lidar = -sin_yaw * x_relative + cos_yaw * y_relative;

        track.state_vec(S_X) = x_lidar;
        track.state_vec(S_Y) = y_lidar;
        track.state_vec(S_YAW) = track.state_vec(S_YAW) - synced_lidar_state.yaw;

        // Convert global velocity to Lidar coordinates
        track.state_vec(S_VX) = cos_yaw * track.state_vec(S_VX) + sin_yaw * track.state_vec(S_VY);
        track.state_vec(S_VY) = -sin_yaw * track.state_vec(S_VX) + cos_yaw * track.state_vec(S_VY);

        // Convert acceleration to Lidar coordinates
        track.state_vec(S_AX) = cos_yaw * track.state_vec(S_AX) + sin_yaw * track.state_vec(S_AY);
        track.state_vec(S_AY) = -sin_yaw * track.state_vec(S_AX) + cos_yaw * track.state_vec(S_AY);
      }
    }
  }
}
}
}