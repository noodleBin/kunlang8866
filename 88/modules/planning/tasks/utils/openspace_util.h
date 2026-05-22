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
 * @file openspace_util.h
 * @brief This file defines the openspace_util class.
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/smoothing_spline/quintic_spline_2d.h"

namespace century {
namespace planning {

using century::common::TrajectoryPoint;

namespace {
constexpr double kMathEpsilon = 0.001;
constexpr double kMaxOpenspaceKappa = 0.25;
constexpr double kMinPointDistance = 0.001;
constexpr double kMaxValidCurvature = 10.0;
}  // namespace

namespace {
template <typename T>
void CalculateLineCurvatureBaseOnThreePoint(std::vector<T>& curve) {
  std::size_t size = curve.size();
  if (size < 3) {
    for (T& pt : curve) {
      pt.set_kappa(0.0001);
    }
  } else {
    for (std::size_t i = 0; i + 1 < size; ++i) {
      T& state1 = curve.at(i);
      T& state2 = curve.at(i + 1);
      if (i + 2 < size) {
        T& state3 = curve.at(i + 2);
        curve.at(i + 1).set_kappa(
            CalculateThreePointsCurvature(state1, state2, state3));
      }
    }
    curve.at(0).set_kappa(curve.at(1).kappa());
    curve.back().set_kappa(curve.at(size - 2).kappa());
  }
}

template <typename Point2d = common::math::Vec2d,
          typename std::enable_if<std::is_base_of<
              common::math::Vec2d, Point2d>::value>::type* = nullptr>
float CalculateThreePointsCurvature(const Point2d& first_point,
                                    const Point2d& second_point,
                                    const Point2d& third_point) {
  // caculate the radius of the circle first
  float a, b, c;  // three sides
  float delta_x, delta_y;
  float s;  // semiperimeter
  float K;

  // side one
  delta_x = second_point.x() - first_point.x();
  delta_y = second_point.y() - first_point.y();
  a = sqrt(delta_x * delta_x + delta_y * delta_y);

  // side two
  delta_x = third_point.x() - second_point.x();
  delta_y = third_point.y() - second_point.y();
  b = sqrt(delta_x * delta_x + delta_y * delta_y);

  // side three
  delta_x = first_point.x() - third_point.x();
  delta_y = first_point.y() - third_point.y();
  c = sqrt(delta_x * delta_x + delta_y * delta_y);

  if (a < 1e-03 || b < 1e-03 || c < 1e-03) {
    return 0;
  }

  // semiperimeter
  s = (a + b + c) * 0.5;
  K = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
  float curvature = 4 * K / (a * b * c);

  float rotate_direction = (second_point.x() - first_point.x()) *
                               (third_point.y() - second_point.y()) -
                           (second_point.y() - first_point.y()) *
                               (third_point.x() - second_point.x());
  if (rotate_direction < 0) {
    // clockwise
    curvature = -curvature;
  }
  return curvature;
}

template <typename T>
void ConvertPointsToLineSegments(
    const std::vector<T>& points,
    std::vector<common::math::LineSegment2d>& linesegments) {
  size_t vertices_num = points.size();
  for (size_t i = 0; i + 1 < vertices_num; ++i) {
    common::math::LineSegment2d line_segment =
        common::math::LineSegment2d(points[i], points[i + 1]);
    linesegments.emplace_back(line_segment);
  }
  return;
}

template <typename T>
void ConvertPointsToLineSegments(
    const std::vector<std::vector<T>>& points_vec,
    std::vector<std::vector<common::math::LineSegment2d>>& linesegments_vec) {
  std::vector<common::math::LineSegment2d> temp_linesegments;
  temp_linesegments.clear();
  for (const auto& points : points_vec) {
    ConvertPointsToLineSegments(points, temp_linesegments);
    linesegments_vec.emplace_back(temp_linesegments);
  }
  return;
}

}  // namespace

class OpenspaceUtil {
 public:
  OpenspaceUtil();

  ~OpenspaceUtil() {};

  bool GetOpenspaceDebugFlag() { return openspace_traj_debug_flag_; };

  void SetOpenSpaceTrajectory(
      century::planning::Frame* frame,
      century::planning::DependencyInjector* injector,
      ADCTrajectory* const ptr_trajectory_pb);

  static void World2Origin(const century::common::math::Vec2d& origin_point,
                           const double origin_heading,
                           century::common::math::Vec2d& target_point);

  static void Origin2World(const century::common::math::Vec2d& origin_point,
                           const double origin_heading,
                           century::common::math::Vec2d& target_point);

  static bool HasPassed(const century::common::PathPoint& start_pt,
                        const century::common::PathPoint& goal_pt,
                        const century::common::PathPoint& current_pt);

  static std::vector<double> CalculateHeadings(
      const std::vector<century::common::math::Vec2d>& points);

  static void RemoveSamePoints(std::vector<century::common::math::Vec2d>* points);

  static void HeadingReversal(DiscretizedTrajectory& traj, const int& mode = 0);

  static double HeadingReversal(const double& orin_heading,
                                const int& mode = 0);

  static double CalculateCurvatureByThreePoints(
      const century::common::PathPoint& p0,
      const century::common::PathPoint& p1,
      const century::common::PathPoint& p2);

  static void UpdateTrajectoryCurvature(
      std::vector<common::TrajectoryPoint>* trajectory);

  static void CalculateLineCurvatureBaseOnThreePoint(
      std::vector<common::TrajectoryPoint>* curve);

  static float CalculateThreePointsCurvature(
      const century::common::PathPoint& first_point,
      const century::common::PathPoint& second_point,
      const century::common::PathPoint& third_point);

  static double CalculateStableCurvature(double x0, double y0, double x1,
                                         double y1, double x2, double y2);

  static std::vector<double> CalculateSmoothedCurvatures(
      const std::vector<double>& x_values, const std::vector<double>& y_values,
      std::vector<double>& s_values);

  static double CalculateFirstDerivativeNonUniform(
      double x_minus2, double x_minus1, double x_i, double x_plus1,
      double x_plus2, double h_minus2, double h_minus1, double h_plus1,
      double h_plus2);

  static double CalculateSecondDerivativeNonUniform(
      double x_minus2, double x_minus1, double x_i, double x_plus1,
      double x_plus2, double h_minus2, double h_minus1, double h_plus1,
      double h_plus2);

  static void ApplyCurvatureRateConstraints(std::vector<double>& curvatures,
                                            const std::vector<double>& s_values,
                                            double max_curvature_rate);

  static void ExtractTrajectoryData(
      const std::vector<century::common::TrajectoryPoint>& trajectory,
      std::vector<double>* x_values, std::vector<double>* y_values,
      std::vector<double>* s_values);

  static std::vector<double> CalculateCurvatureChangeRate(
      const std::vector<double>& curvatures,
      const std::vector<double>& s_values);

  static void UpdateTrajectoryWithCurvature(
      std::vector<century::common::TrajectoryPoint>* trajectory,
      const std::vector<double>& curvatures,
      const std::vector<double>& dkappa_values);

  bool CalcTrajS(const bool& is_reverse,
                 century::planning::DiscretizedTrajectory& current_traj);

  bool PolyfitTrajectory(const bool is_reverse, const double step,
                         DiscretizedTrajectory* polyfit_trajectory_ptr,
                         DiscretizedTrajectory** current_trajectory_ptr);

 private:
  bool openspace_traj_debug_flag_ = false;

  private:
   struct Vec2dHash {
     std::size_t operator()(const Vec2d& point) const {
       std::size_t h1 = std::hash<double>{}(point.x());
       std::size_t h2 = std::hash<double>{}(point.y());
       return h1 ^ (h2 << 1);
     }
   };

   struct Vec2dEqual {
     bool operator()(const Vec2d& a, const Vec2d& b) const {
       const double tolerance = 1e-6;
       return std::abs(a.x() - b.x()) < tolerance &&
              std::abs(a.y() - b.y()) < tolerance;
     }
   };
};
}  // namespace planning
}  // namespace century