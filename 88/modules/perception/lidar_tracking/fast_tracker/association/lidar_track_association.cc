
#include "modules/perception/lidar_tracking/fast_tracker/association/lidar_track_association.h"
#include "modules/perception/lidar_tracking/fast_tracker/ekf/lidar_track_ekf_filter.h"
namespace Track {
namespace Association {
using namespace Track::Ekf;
double DistanceCalculator::RolloutDistance(const Track::Ekf::TrackStruct& track, 
                                          const Track::Ekf::Meastruct& messurement) {
  double l2_distance = CalculateDistance(messurement.state, track.state_vec);
  double maha_distance = CalculateMahalanobisDistance(messurement.state, track);
  if (track.is_init == false || l2_distance > max_association_dist_m_)
    maha_distance = 1000.0; // Increase distance for uninitialized tracks to avoid pairing

  if ((messurement.classification == ObjectClass::PEDESTRIAN &&
      track.GetRepClass() != ObjectClass::PEDESTRIAN) ||
    (track.GetRepClass() == ObjectClass::PEDESTRIAN &&
      messurement.classification != ObjectClass::PEDESTRIAN)) {
    maha_distance = 1000.0; // Associate only among pedestrians
  }

  if (messurement.classification == ObjectClass::PEDESTRIAN && l2_distance > 2.0) {
    maha_distance = 1000.0; // Shorter association distance for pedestrians
  }
  return maha_distance;

}


double DistanceCalculator::CalculateDistance(const ObjectState &state1,
                                             const ObjectState &state2) {
  return std::sqrt(std::pow(state1.x - state2.x, 2) + std::pow(state1.y - state2.y, 2));
}

double DistanceCalculator::CalculateDistance(const ObjectState &state1, 
                                             const Eigen::Vector8d &state2) {
  return std::sqrt(std::pow(state1.x - state2(0), 2) + std::pow(state1.y - state2(1), 2));
}

double DistanceCalculator::CalculateMahalanobisDistance(const ObjectState &state1,
                                                        const TrackStruct &track) {
  Eigen::Vector2d mean_diff;
  mean_diff << state1.x - track.state_vec(S_X), state1.y - track.state_vec(S_Y);

  Eigen::Matrix2d covariance;
  covariance << track.state_cov(S_X, S_X), track.state_cov(S_X, S_Y),
  track.state_cov(S_Y, S_X), track.state_cov(S_Y, S_Y); 
  // Use the top 2x2 part of state_cov as the covariance matrix

  // Calculate the inverse of the covariance matrix
  Eigen::Matrix2d inv_covariance = covariance.inverse();

  // Calculate Mahalanobis distance
  double mahalanobis_distance = std::sqrt(mean_diff.transpose() * inv_covariance * mean_diff);

  if (mahalanobis_distance > mean_diff.norm() * 3.0) {
    mahalanobis_distance = mean_diff.norm() * 3.0;
  }
  if (mahalanobis_distance < mean_diff.norm() / 5.0) {
    mahalanobis_distance = mean_diff.norm() / 5.0;
  }
  return mahalanobis_distance;
}

}
}