/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

/**
 * @file openspace_util.cc
 **/

#include "modules/planning/tasks/utils/openspace_util.h"

#include <algorithm>
#include <cmath>

namespace century {
namespace planning {

OpenspaceUtil::OpenspaceUtil() { openspace_traj_debug_flag_ = false; }

void OpenspaceUtil::World2Origin(
    const century::common::math::Vec2d& origin_point,
    const double origin_heading, century::common::math::Vec2d& target_point) {
  target_point -= origin_point;
  target_point.SelfRotate(-origin_heading);
  return;
}

void OpenspaceUtil::Origin2World(
    const century::common::math::Vec2d& origin_point,
    const double origin_heading, century::common::math::Vec2d& target_point) {
  target_point.SelfRotate(origin_heading);
  target_point += origin_point;
  return;
}

std::vector<double> OpenspaceUtil::CalculateHeadings(
    const std::vector<century::common::math::Vec2d>& points) {
  std::vector<double> headings;
  if (points.empty()) {
    return headings;
  }
  size_t n = points.size();
  headings.reserve(points.size());

  if (n > 1) {
    Vec2d vec_to_next = points[1] - points[0];
    headings.emplace_back(vec_to_next.Angle());
  } else {
    headings.emplace_back(0.0);
  }

  for (size_t i = 1; i < n - 1; ++i) {
    Vec2d vec_from_prev = points[i] - points[i - 1];
    headings.emplace_back(vec_from_prev.Angle());
  }

  if (n > 1) {
    Vec2d vec_from_prev = points[n - 1] - points[n - 2];
    headings.emplace_back(vec_from_prev.Angle());
  }
  CHECK_EQ(headings.size(), points.size())
      << "headings and input_points result size not equal";
  return headings;
}

void OpenspaceUtil::RemoveSamePoints(
    std::vector<century::common::math::Vec2d> *points) {
  if (points->empty()) {
    return;
  }
  std::unordered_set<Vec2d, Vec2dHash, Vec2dEqual> seen_points;
  std::vector<Vec2d> result;
  for (const auto &point : *points) {
    if (seen_points.find(point) == seen_points.end()) {
      seen_points.insert(point);
      result.emplace_back(point);
    }
  }
  *points = result;
}

/**
 * reverse heading
 * @param traj FLU, (-PI, PI]
 * default: FLU, (-PI, PI]
 * mode = 0: FLU, (-PI, PI]
 * mode = 1: FLU, [0, 2*PI)
 */
void OpenspaceUtil::HeadingReversal(DiscretizedTrajectory& traj,
                                    const int& mode) {
  for (size_t i = 0; i < traj.size(); i++) {
    traj.at(i).mutable_path_point()->set_theta(
        HeadingReversal(traj.at(i).path_point().theta(), mode));
  }
}

/**
 * reverse heading
 * @param orin_heading FLU, (-PI, PI]
 * default: FLU, (-PI, PI]
 * mode = 0: FLU, (-PI, PI]
 * mode = 1: FLU, [0, 2*PI)
 */
double OpenspaceUtil::HeadingReversal(const double& orin_heading,
                                      const int& mode) {
  double rever_heading = 0.0;
  if (0 == mode) {
    if (orin_heading > 0) {
      rever_heading = orin_heading - M_PI;
    } else {
      rever_heading = orin_heading + M_PI;
    }
  } else if (1 == mode) {
    rever_heading = orin_heading - M_PI;
    if (rever_heading < 0) {
      rever_heading = rever_heading + 2 * M_PI;
    }
  }
  return rever_heading;
}

// calculate curvature based on three points
double OpenspaceUtil::CalculateCurvatureByThreePoints(
    const century::common::PathPoint& p0, const century::common::PathPoint& p1,
    const century::common::PathPoint& p2) {
  double area = std::abs((p1.x() - p0.x()) * (p2.y() - p0.y()) -
                         (p2.x() - p0.x()) * (p1.y() - p0.y())) /
                2.0;

  double a =
      std::sqrt(std::pow(p1.x() - p0.x(), 2) + std::pow(p1.y() - p0.y(), 2));
  double b =
      std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
  double c =
      std::sqrt(std::pow(p2.x() - p0.x(), 2) + std::pow(p2.y() - p0.y(), 2));

  if (area < 1e-6 || a * b * c < 1e-6) {
    return 0.0;
  }
  double curvature = 4.0 * area / (a * b * c);
  double cross_product = (p1.x() - p0.x()) * (p2.y() - p0.y()) -
                         (p2.x() - p0.x()) * (p1.y() - p0.y());

  if (cross_product < 0) {
    curvature = -curvature;
  }

  return curvature;
}

void OpenspaceUtil::UpdateTrajectoryCurvature(
    std::vector<common::TrajectoryPoint>* trajectory) {
  if (trajectory->size() < 3) {
    return;
  }

  if (trajectory->size() >= 2) {
    const auto& p0 = trajectory->at(0).path_point();
    const auto& p1 = trajectory->at(1).path_point();
    double dx = p1.x() - p0.x();
    double dy = p1.y() - p0.y();
    double ds = std::sqrt(dx * dx + dy * dy);

    if (ds > 1e-6) {
      double curvature = 0.0;
      trajectory->at(0).mutable_path_point()->set_kappa(curvature);
    }
  }

  for (size_t i = 1; i < trajectory->size() - 1; ++i) {
    const auto& p0 = trajectory->at(i - 1).path_point();
    const auto& p1 = trajectory->at(i).path_point();
    const auto& p2 = trajectory->at(i + 1).path_point();

    double curvature =
        OpenspaceUtil::CalculateCurvatureByThreePoints(p0, p1, p2);
    trajectory->at(i).mutable_path_point()->set_kappa(curvature);
  }

  if (trajectory->size() >= 2) {
    size_t last = trajectory->size() - 1;
    double curvature = 0.0;
    trajectory->at(last).mutable_path_point()->set_kappa(curvature);
  }
}

void OpenspaceUtil::CalculateLineCurvatureBaseOnThreePoint(
    std::vector<common::TrajectoryPoint>* curve) {
  std::size_t size = curve->size();
  if (size < 3) {
    for (auto& pt : *curve) {
      pt.mutable_path_point()->set_kappa(0.0001);
    }
  } else {
    for (std::size_t i = 0; i + 1 < size; ++i) {
      common::TrajectoryPoint& state1 = curve->at(i);
      common::TrajectoryPoint& state2 = curve->at(i + 1);
      if (i + 2 < size) {
        common::TrajectoryPoint& state3 = curve->at(i + 2);
        curve->at(i + 1).mutable_path_point()->set_kappa(
            CalculateThreePointsCurvature(
                state1.path_point(), state2.path_point(), state3.path_point()));
      }
    }
    curve->at(0).mutable_path_point()->set_kappa(
        curve->at(1).path_point().kappa());
    curve->back().mutable_path_point()->set_kappa(
        curve->at(size - 2).path_point().kappa());
  }
}

float OpenspaceUtil::CalculateThreePointsCurvature(
    const century::common::PathPoint& first_point,
    const century::common::PathPoint& second_point,
    const century::common::PathPoint& third_point) {
  return CalculateStableCurvature(first_point.x(), first_point.y(),
                                  second_point.x(), second_point.y(),
                                  third_point.x(), third_point.y());
}

double OpenspaceUtil::CalculateStableCurvature(double x0, double y0, double x1,
                                               double y1, double x2,
                                               double y2) {
  double dx1 = x1 - x0;
  double dy1 = y1 - y0;
  double dx2 = x2 - x1;
  double dy2 = y2 - y1;

  double norm1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double norm2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (norm1 < kMinPointDistance || norm2 < kMinPointDistance) {
    return 0.0;
  }
  double ux1 = dx1 / norm1;
  double uy1 = dy1 / norm1;
  double ux2 = dx2 / norm2;
  double uy2 = dy2 / norm2;
  double dot_product = ux1 * ux2 + uy1 * uy2;
  double cross_product = ux1 * uy2 - uy1 * ux2;

  dot_product = std::max(-1.0, std::min(1.0, dot_product));
  double delta_theta = std::acos(dot_product);
  if (cross_product < 0) {
    delta_theta = -delta_theta;
  }

  double avg_arc_length = (norm1 + norm2) / 2.0;
  if (avg_arc_length < kMinPointDistance) {
    return 0.0;
  }

  double curvature = delta_theta / avg_arc_length;
  if (std::abs(curvature) > kMaxValidCurvature) {
    return 0.0;
  }

  if (std::abs(curvature) > kMaxOpenspaceKappa) {
    curvature = (curvature > 0) ? kMaxOpenspaceKappa : -kMaxOpenspaceKappa;
  }

  return curvature;
}

std::vector<double> OpenspaceUtil::CalculateSmoothedCurvatures(
    const std::vector<double>& x_values, const std::vector<double>& y_values,
    std::vector<double>& s_values) {
  size_t n = x_values.size();
  std::vector<double> curvatures(n, 0.0);

  std::vector<double> corrected_s = s_values;
  bool s_monotonic = true;
  for (size_t i = 1; i < n; ++i) {
    double dist = std::sqrt(std::pow(x_values[i] - x_values[i - 1], 2) +
                            std::pow(y_values[i] - y_values[i - 1], 2));
    if (dist < kMinPointDistance) {
      corrected_s[i] = corrected_s[i - 1] + kMinPointDistance;
      s_monotonic = false;
    } else if (s_values[i] <= s_values[i - 1]) {
      corrected_s[i] = corrected_s[i - 1] + dist;
      s_monotonic = false;
    }
  }
  if (!s_monotonic) {
    s_values.clear();
    corrected_s[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
      double dist = std::sqrt(std::pow(x_values[i] - x_values[i - 1], 2) +
                              std::pow(y_values[i] - y_values[i - 1], 2));
      corrected_s[i] = corrected_s[i - 1] + std::max(dist, kMinPointDistance);
    }
    s_values = std::move(corrected_s);
  }

  if (n < 5) {
    for (size_t i = 1; i < n - 1; ++i) {
      double dx_ds = (x_values[i + 1] - x_values[i - 1]) /
                     (s_values[i + 1] - s_values[i - 1]);
      double dy_ds = (y_values[i + 1] - y_values[i - 1]) /
                     (s_values[i + 1] - s_values[i - 1]);
      double d2x_ds2 = (x_values[i + 1] - 2 * x_values[i] + x_values[i - 1]) /
                       std::pow((s_values[i + 1] - s_values[i - 1]) / 2.0, 2);
      double d2y_ds2 = (y_values[i + 1] - 2 * y_values[i] + y_values[i - 1]) /
                       std::pow((s_values[i + 1] - s_values[i - 1]) / 2.0, 2);

      double denominator = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
      if (denominator > kMathEpsilon) {
        curvatures[i] =
            std::abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator;
      }
    }
    return curvatures;
  }

  for (size_t i = 2; i < n - 2; ++i) {
    double h_minus2 = s_values[i] - s_values[i - 2];
    double h_minus1 = s_values[i] - s_values[i - 1];
    double h_plus1 = s_values[i + 1] - s_values[i];
    double h_plus2 = s_values[i + 2] - s_values[i];

    double dx_ds = CalculateFirstDerivativeNonUniform(
        x_values[i - 2], x_values[i - 1], x_values[i], x_values[i + 1],
        x_values[i + 2], h_minus2, h_minus1, h_plus1, h_plus2);

    double dy_ds = CalculateFirstDerivativeNonUniform(
        y_values[i - 2], y_values[i - 1], y_values[i], y_values[i + 1],
        y_values[i + 2], h_minus2, h_minus1, h_plus1, h_plus2);

    double d2x_ds2 = CalculateSecondDerivativeNonUniform(
        x_values[i - 2], x_values[i - 1], x_values[i], x_values[i + 1],
        x_values[i + 2], h_minus2, h_minus1, h_plus1, h_plus2);

    double d2y_ds2 = CalculateSecondDerivativeNonUniform(
        y_values[i - 2], y_values[i - 1], y_values[i], y_values[i + 1],
        y_values[i + 2], h_minus2, h_minus1, h_plus1, h_plus2);

    double denominator = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
    if (denominator > kMathEpsilon) {
      curvatures[i] = std::abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator;
    }
  }

  return curvatures;
}

double OpenspaceUtil::CalculateFirstDerivativeNonUniform(
    double x_minus2, double x_minus1, double x_i, double x_plus1,
    double x_plus2, double h_minus2, double h_minus1, double h_plus1,
    double h_plus2) {
  double weight_minus2 = 1.0 / (h_minus2 * (h_minus2 - h_minus1) *
                                (h_minus2 - h_plus1) * (h_minus2 - h_plus2));
  double weight_minus1 = 1.0 / (h_minus1 * (h_minus1 - h_minus2) *
                                (h_minus1 - h_plus1) * (h_minus1 - h_plus2));
  double weight_plus1 = 1.0 / (h_plus1 * (h_plus1 - h_minus2) *
                               (h_plus1 - h_minus1) * (h_plus1 - h_plus2));
  double weight_plus2 = 1.0 / (h_plus2 * (h_plus2 - h_minus2) *
                               (h_plus2 - h_minus1) * (h_plus2 - h_plus1));

  return weight_minus2 * x_minus2 + weight_minus1 * x_minus1 +
         weight_plus1 * x_plus1 + weight_plus2 * x_plus2;
}

double OpenspaceUtil::CalculateSecondDerivativeNonUniform(
    double x_minus2, double x_minus1, double x_i, double x_plus1,
    double x_plus2, double h_minus2, double h_minus1, double h_plus1,
    double h_plus2) {
  double h1 = h_minus1;  // s_i - s_{i-1}
  double h2 = h_plus1;   // s_{i+1} - s_i
  double h3 = h_minus2;  // s_i - s_{i-2} = h1 + (s_{i-1} - s_{i-2})
  double h4 = h_plus2;   // s_{i+2} - s_i = h2 + (s_{i+2} - s_{i+1})

  if (std::abs(h1 - h2) < kMathEpsilon && std::abs(h3 - h4) < kMathEpsilon) {
    return (-x_minus2 + 16 * x_minus1 - 30 * x_i + 16 * x_plus1 - x_plus2) /
           (12 * h1 * h1);
  } else {
    double d2x_ds2_1 = (x_plus1 - 2 * x_i + x_minus1) / (h1 * h2);
    double d2x_ds2_2 = (x_plus2 - 2 * x_plus1 + x_i) / (h2 * (h2 + h4 - h2));
    double d2x_ds2_3 = (x_i - 2 * x_minus1 + x_minus2) / (h1 * (h1 + h3 - h1));
    return 0.5 * d2x_ds2_1 + 0.25 * d2x_ds2_2 + 0.25 * d2x_ds2_3;
  }
}

void OpenspaceUtil::ApplyCurvatureRateConstraints(
    std::vector<double>& curvatures, const std::vector<double>& s_values,
    double max_curvature_rate) {
  if (curvatures.size() < 2) {
    return;
  }

  for (size_t i = 1; i < curvatures.size(); ++i) {
    double delta_s = s_values[i] - s_values[i - 1];
    if (delta_s < kMathEpsilon) continue;

    double curvature_rate = (curvatures[i] - curvatures[i - 1]) / delta_s;
    if (std::abs(curvature_rate) > max_curvature_rate) {
      double sign = (curvature_rate > 0) ? 1.0 : -1.0;
      curvatures[i] = curvatures[i - 1] + sign * max_curvature_rate * delta_s;
    }
  }

  for (size_t i = curvatures.size() - 2; i > 0; --i) {
    double delta_s = s_values[i + 1] - s_values[i];
    if (delta_s < kMathEpsilon) continue;
    double curvature_rate = (curvatures[i + 1] - curvatures[i]) / delta_s;
    if (std::abs(curvature_rate) > max_curvature_rate) {
      double sign = (curvature_rate > 0) ? 1.0 : -1.0;
      curvatures[i] = curvatures[i + 1] - sign * max_curvature_rate * delta_s;
    }
  }
}

void OpenspaceUtil::ExtractTrajectoryData(
    const std::vector<century::common::TrajectoryPoint>& trajectory,
    std::vector<double>* x_values, std::vector<double>* y_values,
    std::vector<double>* s_values) {
  x_values->clear();
  y_values->clear();
  s_values->clear();
  for (const auto& point : trajectory) {
    const auto& path_point = point.path_point();
    x_values->emplace_back(path_point.x());
    y_values->emplace_back(path_point.y());
    s_values->emplace_back(path_point.s());
  }
}

std::vector<double> OpenspaceUtil::CalculateCurvatureChangeRate(
    const std::vector<double>& curvatures,
    const std::vector<double>& s_values) {
  std::vector<double> dkappa_values(curvatures.size(), 0.0);
  for (size_t i = 1; i < curvatures.size() - 1; ++i) {
    double delta_s = s_values[i + 1] - s_values[i - 1];
    if (delta_s > kMathEpsilon) {
      dkappa_values[i] = (curvatures[i + 1] - curvatures[i - 1]) / delta_s;
    }
  }

  return dkappa_values;
}

void OpenspaceUtil::UpdateTrajectoryWithCurvature(
    std::vector<century::common::TrajectoryPoint>* trajectory,
    const std::vector<double>& curvatures,
    const std::vector<double>& dkappa_values) {
  for (size_t i = 0; i < trajectory->size(); ++i) {
    auto* path_point = (*trajectory)[i].mutable_path_point();
    path_point->set_kappa(curvatures[i]);
    if (i < dkappa_values.size()) {
      path_point->set_dkappa(dkappa_values[i]);
    }
  }
}

/**
 * brief: current_pt move beyond point goal_pt while moving from starting point
 * start_pt towards target point goal_pt
 * @param start_pt start point
 * @param goal_pt goal point
 * @param current_pt current point
 * @return true means passed，false means not passed
 */
bool OpenspaceUtil::HasPassed(const century::common::PathPoint& start_pt,
                              const century::common::PathPoint& goal_pt,
                              const century::common::PathPoint& current_pt) {
  double vx = goal_pt.x() - start_pt.x();
  double vy = goal_pt.y() - start_pt.y();
  double wx = current_pt.x() - start_pt.x();
  double wy = current_pt.y() - start_pt.y();

  double dot_vw = vx * wx + vy * wy;
  double dot_vv = vx * vx + vy * vy;
  const double epsilon = 1e-9;

  // if v·w > v·v，means current_pt is passed goal_pt
  return dot_vw > dot_vv - epsilon;
}

bool OpenspaceUtil::CalcTrajS(
    const bool& is_reverse,
    century::planning::DiscretizedTrajectory& current_traj) {
  if (current_traj.empty()) {
    return false;
  }
  double dis = 0;
  current_traj.at(0).mutable_path_point()->set_s(dis);
  for (size_t i = 1; i < current_traj.size(); ++i) {
    dis += std::hypot(current_traj.at(i).path_point().x() -
                          current_traj.at(i - 1).path_point().x(),
                      current_traj.at(i).path_point().y() -
                          current_traj.at(i - 1).path_point().y());
    current_traj.at(i).mutable_path_point()->set_s(is_reverse ? -dis : dis);
  }
  return true;
}

bool OpenspaceUtil::PolyfitTrajectory(
    const bool is_reverse, const double step,
    DiscretizedTrajectory* polyfit_trajectory_ptr,
    DiscretizedTrajectory** current_trajectory_ptr) {
  DiscretizedTrajectory& current_trajectory = **current_trajectory_ptr;
  if (current_trajectory.empty()) {
    AERROR << "Current trajectory is empty!";
    return false;
  }

  polyfit_trajectory_ptr->clear();
  polyfit_trajectory_ptr->shrink_to_fit();

  std::vector<double> s_list, x_list, y_list;
  s_list.reserve(current_trajectory.size());
  x_list.reserve(current_trajectory.size());
  y_list.reserve(current_trajectory.size());

  for (const auto& pt : current_trajectory) {
    s_list.emplace_back(pt.path_point().s());
    x_list.emplace_back(pt.path_point().x());
    y_list.emplace_back(pt.path_point().y());
  }

  const double s_start = s_list.front();
  const double s_end = s_list.back();

  QuinticSpline2D spline(s_list, x_list, y_list);

  double s = s_start;
  while ((s_end - s) * step >= 0) {
    double x, y;
    double dx, dy, ddx, ddy, dddx, dddy;
    spline.CalcPosition(s, &x, &y);
    spline.CalcDerivatives(s, &dx, &dy, &ddx, &ddy, &dddx, &dddy);

    TrajectoryPoint point;
    century::common::PathPoint* path_point = point.mutable_path_point();

    path_point->set_x(x);
    path_point->set_y(y);
    path_point->set_s(s);

    double denom = std::pow(dx * dx + dy * dy, 1.5);
    double kappa = 0.0;
    if (denom > 1e-6) {
      kappa = (dx * ddy - dy * ddx) / denom;
    }
    path_point->set_kappa(kappa);

    double denom_ds = std::pow(dx * dx + dy * dy, 1.5);
    double kappa_ds = 0.0;
    if (denom_ds > 1e-6) {
      double numerator = (dx * dddy - dy * dddx);
      kappa_ds = numerator / denom_ds -
                 3.0 * kappa * (dx * ddx + dy * ddy) / (dx * dx + dy * dy);
    }
    path_point->set_dkappa(kappa_ds);

    polyfit_trajectory_ptr->emplace_back(point);
    s += step;
  }

  *current_trajectory_ptr = polyfit_trajectory_ptr;
  if (polyfit_trajectory_ptr->empty()) {
    AERROR << "QuinticSpline failed: result is empty, return false."
           << ", and start s = " << s_start << ", end s = " << s_end;
    return false;
  }

  AINFO << "QuinticSpline success, polyfit_trajectory_ptr->size() = "
        << polyfit_trajectory_ptr->size() << ", start s = " << s_start
        << ", end s = " << s_end;
  return true;
}

void OpenspaceUtil::SetOpenSpaceTrajectory(
    century::planning::Frame* frame,
    century::planning::DependencyInjector* injector,
    ADCTrajectory* const ptr_trajectory_pb) {
  const auto& publishable_trajectory =
      frame->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame->open_space_info().publishable_trajectory_data().second;
  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_gear(publishable_trajectory_gear);
  auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

  if (injector->vehicle_state()->vehicle_state().driving_mode() !=
      century::canbus::Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
    engage_advice->set_advice(century::common::EngageAdvice::READY_TO_ENGAGE);
    engage_advice->set_reason(
        "Ready to engage when staring with OPEN_SPACE_PLANNER");
  } else {
    engage_advice->set_advice(century::common::EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason("Keep engage while in parking");
  }
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_parking()
      ->set_status(MainParking::IN_PARKING);

  ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
      frame->open_space_info().latency_stats());
  openspace_traj_debug_flag_ = true;
}

}  // namespace planning
}  // namespace century