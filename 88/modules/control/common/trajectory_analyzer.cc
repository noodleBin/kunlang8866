/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/control/common/trajectory_analyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/search.h"
#include "modules/control/common/control_gflags.h"

using namespace century::common::math;
using century::common::PathPoint;
using century::common::TrajectoryPoint;

namespace century {
namespace control {
namespace {
constexpr double kDistanceThreshold = 2.5;
constexpr int kLessTrajPointNum = 3;
constexpr double kMinTimeStep = 0.1;
constexpr double kMinTrajectoryLength = 0.1;
// Squared distance from the point to (x, y).
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

// TrajectoryPoint ---> PathPoint
PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  if (point.has_path_point()) {
    return point.path_point();
  } else {
    return PathPoint();
  }
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory,
    const bool use_for_lat_control) {
  header_time_ = planning_published_trajectory->header().timestamp_sec();
  seq_num_ = planning_published_trajectory->header().sequence_num();
  bool need_diagonal = planning_published_trajectory->has_need_diagonal() &&
                       planning_published_trajectory->need_diagonal();
      // need_diagonal=false;
  bool need_forward_control_logic =
      planning_published_trajectory->has_gear() &&
      (canbus::Chassis::GEAR_DRIVE == planning_published_trajectory->gear() ||
       (FLAGS_reverse_heading_control &&
        canbus::Chassis::GEAR_REVERSE ==
            planning_published_trajectory->gear()));
  if (FLAGS_enable_fill_trajectory_point &&
      century::planning::ADCTrajectory::LANEFOLLOW ==
          planning_published_trajectory->trajectory_scenario() &&
      !need_diagonal && need_forward_control_logic) {
    size_t traj_size = planning_published_trajectory->trajectory_point().size();
    auto path_array =
        planning_published_trajectory->debug().planning_data().path();
    google::protobuf::RepeatedPtrField<century::common::PathPoint> path_data;
    for (int i = 0; i < path_array.size(); ++i) {
      if ("Planning PathData" == path_array[i].name()) {
        path_data = path_array[i].path_point();
        break;
      }
    }
    bool enable_use_path_data = false;

    size_t path_data_size = path_data.size();
    if (path_data_size >= kLessTrajPointNum) {
      if (traj_size >= kLessTrajPointNum) {
        auto start_trajectory_point =
            planning_published_trajectory->trajectory_point().begin();
        auto end_trajectory_point =
            planning_published_trajectory->trajectory_point().rbegin();

        if ((!start_trajectory_point->has_path_point() ||
             !start_trajectory_point->path_point().has_s() ||
             !end_trajectory_point->has_path_point() ||
             !end_trajectory_point->path_point().has_s())) {
          enable_use_path_data = true;

        } else {
          if ((std::abs(end_trajectory_point->path_point().s() -
                        start_trajectory_point->path_point().s()) <
               kMinTrajectoryLength)) {
            enable_use_path_data = true;
          }
        }
      } else {
        enable_use_path_data = true;
      }
    }

    if (enable_use_path_data) {
      TrajectoryPoint trajectory_point_temp;

      for (size_t i = 0; i < path_data_size; ++i) {
        trajectory_point_temp.mutable_path_point()->CopyFrom(path_data[i]);

        trajectory_point_temp.set_v(0.0);
        trajectory_point_temp.set_a(0.0);
        trajectory_point_temp.set_relative_time(kMinTimeStep * i);
        trajectory_point_temp.set_da(0.0);
        trajectory_point_temp.set_steer(0.0);
        trajectory_points_.push_back(trajectory_point_temp);
      }

    } else {
      for (int i = 0;
           i < planning_published_trajectory->trajectory_point_size(); ++i) {
        trajectory_points_.push_back(
            planning_published_trajectory->trajectory_point(i));
      }
    }

  } else {
    for (int i = 0; i < planning_published_trajectory->trajectory_point_size();
         ++i) {
      trajectory_points_.push_back(
          planning_published_trajectory->trajectory_point(i));
      if (planning_published_trajectory->gear() ==
              canbus::Chassis::GEAR_REVERSE &&
          use_for_lat_control) {
        trajectory_points_.back().mutable_path_point()->set_s(
            -trajectory_points_.back().path_point().s());
        trajectory_points_.back().mutable_path_point()->set_theta(
            NormalizeAngle(trajectory_points_.back().path_point().theta() +
                           M_PI));
      }
    }
  }
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                    const double y) const {
  return QueryNearestPathPoint(x, y);
}

PathPoint TrajectoryAnalyzer::QueryNearestPathPoint(const double x,
                                                    const double y) const {
  CHECK_GT(trajectory_points_.size(), 0U);

  // nearest point index
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;
  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point().s() -
                trajectory_points_[index_end].path_point().s()) <=
          kMathEpsilon) {
    return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
  }

  // interpolate
  return TrajectoryPointToPathPoint(FindMinDistancePoint(
      trajectory_points_[index_start], trajectory_points_[index_end], x, y));
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();

  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;  // lateral
                                                                 // error
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s() + dot_rd_nd;

  double delta_theta = theta - ref_point.theta();
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    AERROR << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa()
           << " ref_point.x:" << ref_point.x()
           << " ref_point.y:" << ref_point.y() << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time() < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);

  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  if (FLAGS_query_forward_time_point_only) {
    return *it_low;
  } else {
    auto it_lower = it_low - 1;
    if (it_low->relative_time() - t < t - it_lower->relative_time()) {
      return *it_low;
    }
    return *it_lower;
  }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  CHECK_GT(trajectory_points_.size(), 0U);

  // nearest point index
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;
  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_end].path_point().s() -
                trajectory_points_[index_start].path_point().s()) <
          kMathEpsilon) {
    return trajectory_points_[index_min];
  }

  // interpolate
  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByS(const double s) const {
  CHECK_GT(trajectory_points_.size(), 0U);

  TrajectoryPoint target_point;

  // nearest point of trajectory
  auto func_comp = [](const TrajectoryPoint &point, const double s) {
    return point.path_point().s() < s;
  };
  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), s, func_comp);

  if (it_low == trajectory_points_.begin()) {  // first point
    auto nearest_point = trajectory_points_.front();
    double expand_s = nearest_point.path_point().s() - s;
    double central_angle = expand_s * nearest_point.path_point().kappa();
    double circular_cutting_angle = central_angle / 2.0;
    double chord_length = expand_s;
    if (std::fabs(nearest_point.path_point().kappa()) > kMathEpsilon) {
      chord_length = std::fmin(std::fabs(2.0 * sin(circular_cutting_angle) /
                                         nearest_point.path_point().kappa()),
                               chord_length);
    }
    double theta = NormalizeAngle(nearest_point.path_point().theta() -
                                  circular_cutting_angle);
    double x = nearest_point.path_point().x() - chord_length * cos(theta);
    double y = nearest_point.path_point().y() - chord_length * sin(theta);
    double z = nearest_point.path_point().z();
    target_point.mutable_path_point()->set_x(x);
    target_point.mutable_path_point()->set_y(y);
    target_point.mutable_path_point()->set_z(z);
    target_point.mutable_path_point()->set_theta(theta);
    target_point.mutable_path_point()->set_s(s);
    target_point.mutable_path_point()->set_kappa(
        nearest_point.path_point().kappa());
    target_point.mutable_path_point()->set_dkappa(0.0);
    target_point.mutable_path_point()->set_ddkappa(0.0);
    target_point.set_v(nearest_point.v());
    target_point.set_a(nearest_point.a());
    target_point.set_relative_time(
        nearest_point.relative_time() -
        std::fabs(expand_s / (kMathEpsilon + std::fabs(nearest_point.v()))));
  } else if (it_low == trajectory_points_.end()) {  // last point
    auto nearest_point = trajectory_points_.back();
    double expand_s = s - nearest_point.path_point().s();
    double central_angle = expand_s * nearest_point.path_point().kappa();
    double circular_cutting_angle = central_angle / 2.0;
    double chord_length = expand_s;
    if (std::fabs(nearest_point.path_point().kappa()) > kMathEpsilon) {
      chord_length = std::fmin(std::fabs(2.0 * sin(circular_cutting_angle) /
                                         nearest_point.path_point().kappa()),
                               chord_length);
    }
    double theta = NormalizeAngle(nearest_point.path_point().theta() +
                                  circular_cutting_angle);
    double x = nearest_point.path_point().x() + chord_length * cos(theta);
    double y = nearest_point.path_point().y() + chord_length * sin(theta);
    double z = nearest_point.path_point().z();
    target_point.mutable_path_point()->set_x(x);
    target_point.mutable_path_point()->set_y(y);
    target_point.mutable_path_point()->set_z(z);
    target_point.mutable_path_point()->set_theta(theta);
    target_point.mutable_path_point()->set_s(s);
    target_point.mutable_path_point()->set_kappa(
        nearest_point.path_point().kappa());
    target_point.mutable_path_point()->set_dkappa(0.0);
    target_point.mutable_path_point()->set_ddkappa(0.0);
    target_point.set_v(nearest_point.v());
    target_point.set_a(nearest_point.a());
    target_point.set_relative_time(
        nearest_point.relative_time() +
        std::fabs(expand_s / (kMathEpsilon + std::fabs(nearest_point.v()))));
  } else {  // iterpolate
    auto it_lower = it_low - 1;
    if (std::fabs(it_low->path_point().s() - it_lower->path_point().s()) <
        kMathEpsilon) {
      target_point = *it_low;
    } else {
      target_point.mutable_path_point()->CopyFrom(
          InterpolateUsingLinearApproximation(it_lower->path_point(),
                                              it_low->path_point(), s));
      target_point.set_v(lerp(it_lower->v(), it_lower->path_point().s(),
                              it_low->v(), it_low->path_point().s(), s));
      target_point.set_a(lerp(it_lower->a(), it_lower->path_point().s(),
                              it_low->a(), it_low->path_point().s(), s));
      target_point.set_relative_time(
          lerp(it_lower->relative_time(), it_lower->path_point().s(),
               it_low->relative_time(), it_low->path_point().s(), s));
    }
  }
  return target_point;
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

TrajectoryPoint TrajectoryAnalyzer::FindMinDistancePoint(
    const TrajectoryPoint &p0, const TrajectoryPoint &p1, const double x,
    const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    double px = lerp(p0.path_point().x(), p0.path_point().s(),
                     p1.path_point().x(), p1.path_point().s(), s);
    double py = lerp(p0.path_point().y(), p0.path_point().s(),
                     p1.path_point().y(), p1.path_point().s(), s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  TrajectoryPoint p;
  double s = GoldenSectionSearch(dist_square, p0.path_point().s(),
                                 p1.path_point().s());
  p.mutable_path_point()->set_s(s);
  p.mutable_path_point()->set_x(lerp(p0.path_point().x(), p0.path_point().s(),
                                     p1.path_point().x(), p1.path_point().s(),
                                     s));
  p.mutable_path_point()->set_y(lerp(p0.path_point().y(), p0.path_point().s(),
                                     p1.path_point().y(), p1.path_point().s(),
                                     s));
  p.mutable_path_point()->set_z(lerp(p0.path_point().z(), p0.path_point().s(),
                                     p1.path_point().z(), p1.path_point().s(),
                                     s));
  p.mutable_path_point()->set_theta(
      slerp(p0.path_point().theta(), p0.path_point().s(),
            p1.path_point().theta(), p1.path_point().s(), s));
  // approximate the curvature at the intermediate point
  p.mutable_path_point()->set_kappa(
      lerp(p0.path_point().kappa(), p0.path_point().s(),
           p1.path_point().kappa(), p1.path_point().s(), s));
  p.set_v(lerp(p0.v(), p0.path_point().s(), p1.v(), p1.path_point().s(), s));
  p.set_a(lerp(p0.a(), p0.path_point().s(), p1.a(), p1.path_point().s(), s));
  p.set_relative_time(lerp(p0.relative_time(), p0.path_point().s(),
                           p1.relative_time(), p1.path_point().s(), s));
  return p;
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  CHECK_GT(trajectory_points_.size(), 0U);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point());
    trajectory_points_[i].mutable_path_point()->set_x(com.x());
    trajectory_points_[i].mutable_path_point()->set_y(com.y());
  }
}

Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPositionInterpolation(
    const double x, const double y, const double theta) const {
  std::vector<common::TrajectoryPoint> trajectory_points_temp;
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    // It may be risky to filter track points by theta
    if (std::abs(trajectory_points_[i].path_point().x() - x) <
            kDistanceThreshold &&
        std::abs(trajectory_points_[i].path_point().y() - y) <
            kDistanceThreshold &&
        std::abs(common::math::NormalizeAngle(
            trajectory_points_[i].path_point().theta() - theta)) < M_PI / 2) {
      trajectory_points_temp.emplace_back(trajectory_points_[i]);
    }
  }
  if (trajectory_points_temp.size() < 2) {
    double d_min = std::numeric_limits<double>::max();
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
      double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
      if (d_temp < d_min) {
        d_min = d_temp;
        index_min = i;
      }
    }
    return trajectory_points_[index_min];
  } else {
    double d_min = std::numeric_limits<double>::max();
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_temp.size(); ++i) {
      double d_temp = PointDistanceSquare(trajectory_points_temp[i], x, y);
      if (d_temp < d_min) {
        d_min = d_temp;
        index_min = i;
      }
    }

    size_t index_start = index_min == 0 ? index_min : index_min - 1;
    size_t index_end = index_min + 1 == trajectory_points_temp.size()
                           ? index_min
                           : index_min + 1;

    constexpr double kEpsilon = 0.001;
    if (index_start == index_end ||
        std::fabs(trajectory_points_temp[index_start].path_point().s() -
                  trajectory_points_temp[index_end].path_point().s()) <=
            kEpsilon) {
      return trajectory_points_temp[index_start];
    }

    PathPoint point_temp = TrajectoryPointToPathPoint(
        FindMinDistancePoint(trajectory_points_temp[index_start],
                             trajectory_points_temp[index_end], x, y));
    TrajectoryPoint trajectory_point_start =
        trajectory_points_temp[index_start];
    TrajectoryPoint trajectory_point_end = trajectory_points_temp[index_end];
    TrajectoryPoint trajectory_point_temp = trajectory_point_start;
    trajectory_point_temp.mutable_path_point()->CopyFrom(point_temp);

    trajectory_point_temp.set_v(common::math::lerp(
        trajectory_point_start.v(), trajectory_point_start.path_point().s(),
        trajectory_point_end.v(), trajectory_point_end.path_point().s(),
        trajectory_point_temp.path_point().s()));
    trajectory_point_temp.set_a(common::math::lerp(
        trajectory_point_start.a(), trajectory_point_start.path_point().s(),
        trajectory_point_end.a(), trajectory_point_end.path_point().s(),
        trajectory_point_temp.path_point().s()));
    trajectory_point_temp.set_relative_time(
        common::math::lerp(trajectory_point_start.relative_time(),
                           trajectory_point_start.path_point().s(),
                           trajectory_point_end.relative_time(),
                           trajectory_point_end.path_point().s(),
                           trajectory_point_temp.path_point().s()));
    trajectory_point_temp.set_da(common::math::lerp(
        trajectory_point_start.da(), trajectory_point_start.path_point().s(),
        trajectory_point_end.da(), trajectory_point_end.path_point().s(),
        trajectory_point_temp.path_point().s()));
    trajectory_point_temp.set_steer(common::math::lerp(
        trajectory_point_start.steer(), trajectory_point_start.path_point().s(),
        trajectory_point_end.steer(), trajectory_point_end.path_point().s(),
        trajectory_point_temp.path_point().s()));

    return trajectory_point_temp;
  }
}

TrajectoryPoint
TrajectoryAnalyzer::QueryNearestPointByRelativeTimeInterpolation(
    const double t) const {
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    if (trajectory_points_[i].relative_time() < t) {
      index_min = i;
    } else {
      break;
    }
  }

  size_t index_start = index_min;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  constexpr double kEpsilon = 0.001;
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].relative_time() -
                trajectory_points_[index_end].relative_time()) <= kEpsilon) {
    return trajectory_points_[index_start];
  }

  TrajectoryPoint trajectory_point_start = trajectory_points_[index_start];
  TrajectoryPoint trajectory_point_end = trajectory_points_[index_end];

  PathPoint trajectory_point_start_point = trajectory_point_start.path_point();
  PathPoint trajectory_point_end_point = trajectory_point_end.path_point();

  PathPoint point_temp = trajectory_point_start_point;
  point_temp.set_s(common::math::lerp(
      trajectory_point_start_point.s(), trajectory_point_start.relative_time(),
      trajectory_point_end_point.s(), trajectory_point_end.relative_time(), t));
  point_temp.set_x(common::math::lerp(
      trajectory_point_start_point.x(), trajectory_point_start.relative_time(),
      trajectory_point_end_point.x(), trajectory_point_end.relative_time(), t));
  point_temp.set_y(common::math::lerp(
      trajectory_point_start_point.y(), trajectory_point_start.relative_time(),
      trajectory_point_end_point.y(), trajectory_point_end.relative_time(), t));
  point_temp.set_theta(
      common::math::slerp(trajectory_point_start_point.theta(),
                          trajectory_point_start.relative_time(),
                          trajectory_point_end_point.theta(),
                          trajectory_point_end.relative_time(), t));
  // approximate the curvature at the intermediate point
  point_temp.set_kappa(
      common::math::lerp(trajectory_point_start_point.kappa(),
                         trajectory_point_start.relative_time(),
                         trajectory_point_end_point.kappa(),
                         trajectory_point_end.relative_time(), t));

  TrajectoryPoint trajectory_point_temp = trajectory_point_start;
  trajectory_point_temp.mutable_path_point()->CopyFrom(point_temp);

  trajectory_point_temp.set_v(common::math::lerp(
      trajectory_point_start.v(), trajectory_point_start.relative_time(),
      trajectory_point_end.v(), trajectory_point_end.relative_time(), t));
  trajectory_point_temp.set_a(common::math::lerp(
      trajectory_point_start.a(), trajectory_point_start.relative_time(),
      trajectory_point_end.a(), trajectory_point_end.relative_time(), t));
  trajectory_point_temp.set_relative_time(t);
  trajectory_point_temp.set_da(common::math::lerp(
      trajectory_point_start.da(), trajectory_point_start.relative_time(),
      trajectory_point_end.da(), trajectory_point_end.relative_time(), t));
  trajectory_point_temp.set_steer(common::math::lerp(
      trajectory_point_start.steer(), trajectory_point_start.relative_time(),
      trajectory_point_end.steer(), trajectory_point_end.relative_time(), t));

  return trajectory_point_temp;
}
TrajectoryPoint
TrajectoryAnalyzer::QueryNearestPointByAbsoluteTimeInterpolation(
    const double t) const {
  return QueryNearestPointByRelativeTimeInterpolation(t - header_time_);
}
}  // namespace control
}  // namespace century
