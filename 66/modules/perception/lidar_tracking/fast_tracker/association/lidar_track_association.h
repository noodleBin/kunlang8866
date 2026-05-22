#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace Track {
namespace Ekf {
  struct TrackStruct;
  struct Meastruct;
  struct ObjectState;
}
namespace Association {

class DistanceCalculator {
  public:
  DistanceCalculator() = default;
  explicit DistanceCalculator(double max_association_dist_m) : max_association_dist_m_(max_association_dist_m) {};
  virtual ~DistanceCalculator() = default;

  double RolloutDistance(const Track::Ekf::TrackStruct& track, 
                         const Track::Ekf::Meastruct& messurement);
 private:
  double CalculateDistance(const Track::Ekf::ObjectState &state1, const Track::Ekf::ObjectState &state2) ;

  double CalculateDistance(const Track::Ekf::ObjectState &state1, const Eigen::Matrix<double, 8, 1> &state2);

  double CalculateMahalanobisDistance(const  Track::Ekf::ObjectState &state1,
                                      const  Track::Ekf::TrackStruct &track);
  double max_association_dist_m_;

};
}
}