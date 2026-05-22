/****************************************************************************/
// Module:    lidar_track_util.hpp
// Description: lidar_track_util
//
// Authors: xiaxinrong
// Date: March 3th 2025
/****************************************************************************/
#pragma once 
#include <deque>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include "modules/perception/lidar_tracking/fast_tracker/lidar_track_type.h"
namespace Track {
namespace Ekf {
  struct MultiClassObjectTrackingConfig; 
  struct ObjectState;
  struct Meastruct;
  struct TrackStructs;
  struct Meastructs;
}

namespace Util {
bool FileExists(const std::string& filename);

Track::Ekf::ObjectState GetSyncedLidarState(double object_time, const std::deque<Track::Ekf::ObjectState>& deque_lidar_state);

Track::Ekf::ObjectState PredictNextState(const Track::Ekf::ObjectState& state_t_minus_1, const Track::Ekf::ObjectState& state_t);

void TransformMeasLiDAR2Global(Track::Ekf::Meastruct& i_meas,
                               const std::deque<Track::Ekf::ObjectState>& deque_lidar_state,
                               const std::vector<double>& T_dr_lidar);

void AngleBasedTimeCompensation(Track::Ekf::Meastruct& i_meas, const Track::Ekf::MultiClassObjectTrackingConfig& config);

Eigen::Affine3d CreateTransformation(double x, double y, double z, double roll, double pitch, double yaw);

void ConvertTrackGlobalToLocal(Track::Ekf::TrackStructs& track_structs, Track::Ekf::ObjectState synced_lidar_state, const bool global_track); 

void DetectObjects2LocalMeasurements(Track::ros_interface::DetectObjects3D lidar_objects, Track::Ekf::Meastructs& o_local_lidar_measurements);

void ConvertDetectObjectToMeastruct(const Track::ros_interface::DetectObject3D& detect_object, Track::Ekf::Meastruct& meas);

double CalculateYawDotProduct(const double &yaw1, const double &yaw2);

double CalculateYawCrossProduct(const double &yaw1, const double &yaw2);

}
}