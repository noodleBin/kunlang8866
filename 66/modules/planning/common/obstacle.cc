/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/common/obstacle.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "cyber/time/clock.h"

namespace century {
namespace planning {

using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::LineSegment2d;
using century::cyber::Clock;
using century::common::math::Vec2d;
using century::common::util::FindOrDie;
using century::perception::PerceptionObstacle;
using century::prediction::ObstaclePriority;

namespace {
const double kStBoundaryDeltaS = 0.2;        // meters
const double kStBoundarySparseDeltaS = 1.0;  // meters
const double kStBoundaryDeltaT = 0.05;       // seconds
constexpr double kMathEpsilon = 1e-10;
constexpr size_t kMinNumObsFootprint = 3;
const double kAcousticVelocity = 340.0;
constexpr double kSkipLDistanceFactor = 0.4;
constexpr double kSparseMappingS = 20.0;
constexpr double kMinLargeVehicleHeight = 1.8;
}  // namespace

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kStop, 500}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

size_t Next(size_t at, size_t num_points) {
  return at >= num_points - 1 ? 0 : at + 1;
}
size_t Prev(size_t at, size_t num_points) {
  return at == 0 ? num_points - 1 : at - 1;
}

bool IsConvexPoints(const std::vector<Vec2d>& points) {
  size_t num_points = points.size();
  if (num_points < kMinNumObsFootprint) {
    return false;
  }

  // Make sure the points are in counter clockwise order.
  double area = 0.0;
  for (size_t i = 1; i < num_points; ++i) {
    area += common::math::CrossProd(points[0], points[i - 1], points[i]);
  }

  bool is_counter_clockwise = true;
  if (area < 0) {
    is_counter_clockwise = false;
    area = -area;
  }
  area *= 0.5;
  if (area < kMathEpsilon) {
    return false;
  }

  // Check convexity.
  bool is_convex = true;
  for (size_t i = 0; i < num_points; ++i) {
    double cross_prod =
        is_counter_clockwise
            ? common::math::CrossProd(points[Prev(i, num_points)], points[i],
                                      points[Next(i, num_points)])
            : common::math::CrossProd(points[Next(i, num_points)], points[i],
                                      points[Prev(i, num_points)]);
    if (cross_prod < -kMathEpsilon) {
      is_convex = false;
      break;
    }
  }
  return is_convex;
}

bool CheckTinyArea(const std::vector<Vec2d>& points) {
  size_t num_points = points.size();
  if (num_points < kMinNumObsFootprint) {
    return true;
  }

  // Make sure the points are in counter clockwise order.
  double area = 0.0;
  for (size_t i = 1; i < num_points; ++i) {
    area += CrossProd(points[0], points[i - 1], points[i]);
  }

  if (area < 0) {
    area = -area;
  }
  area *= 0.5;
  if (area < kMathEpsilon) {
    return true;
  }
  return false;
}

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static, const double theta)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle) {
  CalculateSpeedHeading();
  perception_bounding_box_ = {{perception_obstacle_.position().x(),
                               perception_obstacle_.position().y()},
                              speed_heading_,
                              perception_obstacle_.length(),
                              perception_obstacle_.width()};
  is_caution_level_obstacle_ = (obstacle_priority == ObstaclePriority::CAUTION);
  std::vector<common::math::Vec2d> polygon_points;
  if (FLAGS_use_navigation_mode ||
      perception_obstacle.polygon_point_size() <= 2) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    ACHECK(perception_obstacle.polygon_point_size() > 2)
        << "object " << id << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }
  ACHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull.\n"
      << perception_obstacle.DebugString();

  is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);
  is_virtual_ = (perception_obstacle.id() < 0);
// AINFO<<"is_virtual_ = "<<is_virtual_;
// AINFO<<"is_static_ = "<<is_static_;
  if (perception_obstacle.has_velocity()) {
    if (std::isnan(perception_obstacle.velocity().x()) ||
        std::isnan(perception_obstacle.velocity().y()) ||
        perception_obstacle.velocity().x() > kAcousticVelocity ||
        perception_obstacle.velocity().y() > kAcousticVelocity) {
      const auto perception_id = std::to_string(perception_obstacle.id());
      AERROR << "id:" << perception_id << " invalid obstacle velocity:"
             << perception_obstacle.velocity().DebugString();
      speed_ = 0.0;
    } else {
      speed_ = std::hypot(perception_obstacle.velocity().x(),
                          perception_obstacle.velocity().y());
    }
  } else {
    speed_ = 0.0;
  }

  if (perception_obstacle.has_acceleration()) {
    if (std::isnan(perception_obstacle.acceleration().x()) ||
        std::isnan(perception_obstacle.acceleration().y())) {
      const auto perception_id = std::to_string(perception_obstacle.id());
      AERROR << "id:" << perception_id << " invalid obstacle acceleration:"
             << perception_obstacle.acceleration().DebugString();
      acceleration_ = 0.0;
    } else {
      acceleration_ = std::hypot(perception_obstacle.acceleration().x(),
                                 perception_obstacle.acceleration().y());
    }
  } else {
    acceleration_ = 0.0;
  }
}

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static, const double theta)
    : Obstacle(id, perception_obstacle, obstacle_priority, is_static, theta) {
  // use speed_heading_
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s +=
        common::util::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  // if trajectory point no set heading,use speed heading.
  if ((speed_ >= FLAGS_min_dynamic_obstacle_speed) &&
      std::fabs(point.path_point().theta()) < kMathEpsilon) {
    return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                               speed_heading_, perception_obstacle_.length(),
                               perception_obstacle_.width());
  }
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.length(),
                             perception_obstacle_.width());
}

common::math::Polygon2d Obstacle::GetPolygon(
    const common::TrajectoryPoint& point) const {
  double d_x = point.path_point().x() - perception_bounding_box_.center_x();
  double d_y = point.path_point().y() - perception_bounding_box_.center_y();
  const auto& traj_start_point = GetPointAtTime(0.0);
  double d_theta =
      point.path_point().theta() - traj_start_point.path_point().theta();
  std::vector<Vec2d> points;
  points.clear();
  for (const auto& point : perception_polygon_.points()) {
    double o_x = point.x() - perception_bounding_box_.center_x();
    double o_y = point.y() - perception_bounding_box_.center_y();
    double x = o_x * std::cos(d_theta) - o_y * std::sin(d_theta);
    double y = o_x * std::sin(d_theta) + o_y * std::cos(d_theta);
    x += perception_bounding_box_.center_x();
    y += perception_bounding_box_.center_y();
    const auto new_point = Vec2d(x + d_x, y + d_y);
    points.emplace_back(new_point);
  }

  if (CheckTinyArea(points)) {
    AERROR << "perception_id_: " << perception_id_
           << " after conversion, is very small.";
    AERROR << "trajectory point: x: " << std::setprecision(9)
           << point.path_point().x() << "y:" << std::setprecision(9)
           << point.path_point().y();
    AERROR << "center: x: " << std::setprecision(9)
           << perception_bounding_box_.center_x()
           << "y:" << std::setprecision(9)
           << perception_bounding_box_.center_y();
    for (const auto& point : perception_obstacle_.polygon_point()) {
      AERROR << "input obstacle origin point: x:" << std::setprecision(9)
             << point.x() << " y:" << std::setprecision(9) << point.y();
    }
    return perception_polygon_;
  }

  return common::math::Polygon2d(points);
}

bool Obstacle::IsValidPerceptionObstacle(const PerceptionObstacle& obstacle) {
  const auto& obs_id = std::to_string(obstacle.id());
  if (obstacle.length() <= 0.0) {
    AERROR << "invalid obstacle id:" << obs_id
           << " length:" << obstacle.length();
    return false;
  }
  if (obstacle.width() <= 0.0) {
    AERROR << "invalid obstacle id:" << obs_id << " width:" << obstacle.width();
    return false;
  }
  if (obstacle.height() <= 0.0) {
    AERROR << "invalid obstacle id:" << obs_id
           << " height:" << obstacle.height();
    return false;
  }
  if (std::isnan(obstacle.position().x()) ||
      std::isnan(obstacle.position().y())) {
    AERROR << "invalid obstacle id:" << obs_id
           << " x:" << obstacle.position().x()
           << " y:" << obstacle.position().y();
    return false;
  }

  std::vector<Vec2d> polygon_points;
  for (const auto& pt : obstacle.polygon_point()) {
    if (std::isnan(pt.x()) || std::isnan(pt.y())) {
      AERROR << "invalid obstacle id:" << obs_id
             << " polygon point:" << pt.DebugString();
      return false;
    }
    polygon_points.emplace_back(pt.x(), pt.y());
  }
  if (!IsConvexPoints(polygon_points)) {
    AERROR << "invalid obstacle id:" << obs_id << " is not convex obstacle.";
    common::math::Polygon2d perception_polygon;
    if (!common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon)) {
      AERROR << "ComputeConvexHull errror, id" << obs_id;
      return false;
    }
  }
  return true;
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle())) {
      ADEBUG << "Invalid perception obstacle: "
             << prediction_obstacle.perception_obstacle().DebugString();
      continue;
    }
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    if (prediction_obstacle.trajectory().empty() ||
        prediction_obstacle.is_static()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle(),
                       prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      continue;
    }

    int trajectory_index = 0;
    for (int i = 0; i < prediction_obstacle.trajectory().size(); ++i) {
      const auto& trajectory = prediction_obstacle.trajectory().at(i);
      if (i > 0 && FLAGS_only_use_one_trajectory) {
        continue;
      }
      bool is_valid_trajectory = true;
      for (const auto& point : trajectory.trajectory_point()) {
        if (!IsValidTrajectoryPoint(point)) {
          AERROR << "obj is not valid:" << perception_id;
          ADEBUG << " TrajectoryPoint: " << trajectory.ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        continue;
      }

      double speed_theta = trajectory.trajectory_point(0).path_point().theta();
      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      // AINFO << "prediction_obstacle.is_static() = "
      //       << prediction_obstacle.is_static();
      obstacles.emplace_back(
          new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(),
                       trajectory, prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static(), speed_theta));
      ++trajectory_index;
    }
  }
  return obstacles;
}

std::unique_ptr<Obstacle> Obstacle::CreateWheelCraneObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::WHEELCRANE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->perception_obstacle_.set_timestamp(Clock::NowInSeconds());
  obstacle->is_static_ = true;
   obstacle->is_virtual_ = false;
   obstacle->is_caution_ = false;
   obstacle->is_caution_level_obstacle_ = false;
  return std::unique_ptr<Obstacle>(obstacle);
}
std::unique_ptr<Obstacle> Obstacle::CreateStackerObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::STACKER);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->perception_obstacle_.set_timestamp(Clock::NowInSeconds());
  obstacle->is_static_ = true;
   obstacle->is_virtual_ = false;
   obstacle->is_caution_ = false;
   obstacle->is_caution_level_obstacle_ = false;
  return std::unique_ptr<Obstacle>(obstacle);
}
std::unique_ptr<Obstacle> Obstacle::CreateStackerObstaclesWithID(
    const std::string& id, const common::math::Box2d& obstacle_box,
    const double& speed) {
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(
      speed * std::cos(obstacle_box.heading()));
  perception_obstacle.mutable_velocity()->set_y(
      speed * std::sin(obstacle_box.heading()));
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(perception::PerceptionObstacle::STACKER);
  perception_obstacle.set_tracking_time(1.0);
  double velocity_x = perception_obstacle.velocity().x();
  double velocity_y = perception_obstacle.velocity().y();
  double speed_heading =
      common::math::NormalizeAngle(std::atan2(velocity_y, velocity_x));

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->perception_obstacle_.set_timestamp(Clock::NowInSeconds());
  obstacle->is_static_ = true;
  if (std::fabs(speed) > 0.1) {
    obstacle->is_static_ = false;
  }
  obstacle->speed_heading_ = speed_heading;
  obstacle->is_virtual_ = false;
  obstacle->is_caution_ = false;
  obstacle->is_caution_level_obstacle_ = false;
  return std::unique_ptr<Obstacle>(obstacle);
}
std::unique_ptr<Obstacle> Obstacle::CreateStaticObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->perception_obstacle_.set_timestamp(Clock::NowInSeconds());
  obstacle->is_static_ = true;
   obstacle->is_virtual_ = false;
   obstacle->is_caution_ = false;
   obstacle->is_caution_level_obstacle_ = false;
  return std::unique_ptr<Obstacle>(obstacle);
}
std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
  sl_boundary_ = sl_boundary;
}

double Obstacle::MinRadiusStopDistance(
    const common::VehicleParam& vehicle_param, const double adc_l) const {
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  static constexpr double stop_distance_buffer = 0.5;
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  // double lateral_diff =
  //     vehicle_param.width() * 0.5 +
  //     std::max(std::fabs(sl_boundary_.start_l()),
  //                                            std::fabs(sl_boundary_.end_l()));
  double lateral_diff =
      std::fabs(adc_l) + std::max(std::fabs(sl_boundary_.start_l()),
                                  std::fabs(sl_boundary_.end_l()));
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  double stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;
  stop_distance -= vehicle_param.front_edge_to_center();
  stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
  stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
  return stop_distance;
}

void Obstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                            const double adc_start_s) {
  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double half_adc_width = adc_param.width() / 2;
  if (is_static_ || trajectory_.trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s();
    double end_s = sl_boundary_.end_s();

    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }

    if (perception::PerceptionObstacle::UNKNOWN ==
            perception_obstacle_.type() ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
            perception_obstacle_.type()) {
      if (!reference_line.IsBlockRoad(perception_bounding_box_,
                                      perception_polygon_, half_adc_width)) {
        return;
      }
    } else {
      if (!reference_line.IsBlockRoad(perception_bounding_box_,
                                      half_adc_width)) {
        return;
      }
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = STBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}

bool Obstacle::BuildStPolygonPointsFromSLBoundary(
    const ReferenceLine& reference_line, const double adc_start_s,
    const common::TrajectoryPoint& first_traj_point,
    const common::TrajectoryPoint& second_traj_point,
    const common::math::Box2d& object_moving_box,
    const SLBoundary& object_boundary,
    std::vector<std::pair<STPoint, STPoint>>* const polygon_points) {
  const double st_boundary_delta_s =
      (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
          ? kStBoundarySparseDeltaS
          : kStBoundaryDeltaS;
  const double object_s_diff =
      object_boundary.end_s() - object_boundary.start_s();
  if (object_s_diff < st_boundary_delta_s) {
    return false;
  }
  const double delta_t =
      second_traj_point.relative_time() - first_traj_point.relative_time();
  double low_s = std::max(object_boundary.start_s() - adc_half_length_, 0.0);
  bool has_low = false;
  double high_s =
      std::min(object_boundary.end_s() + adc_half_length_, FLAGS_st_max_s);
  bool has_high = false;
  while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
    if (!has_low) {
      auto low_ref = reference_line.GetReferencePoint(low_s);
      has_low = object_moving_box.HasOverlap(
          {low_ref, low_ref.heading(), adc_length_,
           adc_width_ + FLAGS_nonstatic_obstacle_nudge_l_buffer});
      low_s += st_boundary_delta_s;
    }
    if (!has_high) {
      auto high_ref = reference_line.GetReferencePoint(high_s);
      has_high = object_moving_box.HasOverlap(
          {high_ref, high_ref.heading(), adc_length_,
           adc_width_ + FLAGS_nonstatic_obstacle_nudge_l_buffer});
      high_s -= st_boundary_delta_s;
    }
  }
  if (has_low && has_high) {
    low_s -= st_boundary_delta_s;
    high_s += st_boundary_delta_s;
    double low_t =
        (first_traj_point.relative_time() +
         std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
             delta_t);
    polygon_points->emplace_back(
        std::make_pair(STPoint{low_s - adc_start_s, low_t},
                       STPoint{high_s - adc_start_s, low_t}));
    double high_t =
        (first_traj_point.relative_time() +
         std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
             delta_t);
    if (high_t - low_t > 0.05) {
      polygon_points->emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, high_t},
                         STPoint{high_s - adc_start_s, high_t}));
    }
  }
  return true;
}

bool Obstacle::BuildTrajectoryStPolygonPoints(
    const ReferenceLine& reference_line, const double adc_start_s,
    std::vector<std::pair<STPoint, STPoint>>* const polygon_points) {
  const double object_width = perception_obstacle_.width();
  const double object_length = perception_obstacle_.length();
  const auto& trajectory_points = trajectory_.trajectory_point();

  SLBoundary last_sl_boundary;
  int last_index = 0;

  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double object_moving_box_length =
        object_length + common::util::DistanceXY(first_point, second_point);

    common::math::Vec2d center((first_point.x() + second_point.x()) * 0.5,
                               (first_point.y() + second_point.y()) * 0.5);
    common::math::Box2d object_moving_box(
        center, first_point.theta(), object_moving_box_length, object_width);
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    // roughly skip points that are too close to last_sl_boundary box
    const double distance_xy =
        common::util::DistanceXY(trajectory_points[last_index].path_point(),
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;
    }

    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) * 0.5;
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    const double end_s = (i == 1) ? reference_line.Length()
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);

    SLBoundary object_boundary;
    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }

    // update history record
    last_sl_boundary = object_boundary;
    last_index = i;

    // skip if object is entirely on one side of reference line.
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width_ * 0.5;

    if (!IsCautionLevelObstacle() &&
        (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
             skip_l_distance ||
         std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
             -skip_l_distance)) {
      continue;
    }

    if (!IsCautionLevelObstacle() && object_boundary.end_s() < 0) {
      // skip if behind reference line
      continue;
    }

    bool flag = BuildStPolygonPointsFromSLBoundary(
        reference_line, adc_start_s, first_traj_point, second_traj_point,
        object_moving_box, object_boundary, polygon_points);
    if (!flag) {
      continue;
    }
  }
  return true;
}

bool Obstacle::BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                         const double adc_start_s,
                                         STBoundary* const st_boundary) {
  if (!IsValidObstacle(perception_obstacle_)) {
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception_obstacle_.DebugString();
    return false;
  }
  const auto& trajectory_points = trajectory_.trajectory_point();
  if (trajectory_points.empty()) {
    AWARN << "object " << id_ << " has no trajectory points";
    return false;
  }

  std::vector<std::pair<STPoint, STPoint>> polygon_points;
  BuildTrajectoryStPolygonPoints(reference_line, adc_start_s, &polygon_points);

  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = STBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}

const STBoundary& Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}

const STBoundary& Obstacle::path_st_boundary() const {
  return path_st_boundary_;
}

const std::vector<std::string>& Obstacle::decider_tags() const {
  return decider_tags_;
}

const std::vector<ObjectDecisionType>& Obstacle::decisions() const {
  return decisions_;
}

bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

ObjectDecisionType Obstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  // stop compiler complaining
}

const ObjectDecisionType& Obstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

const ObjectDecisionType& Obstacle::LateralDecision() const {
  return lateral_decision_;
}

bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

ObjectDecisionType Obstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      DCHECK(lhs.nudge().type() == rhs.nudge().type())
          << "could not merge left nudge and right nudge";
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
  return lhs;
}

bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

void Obstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

std::string Obstacle::DebugString() const {
  std::stringstream ss;
  ss << "Obstacle id: " << id_;
  for (size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitudinal decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}

const SLBoundary& Obstacle::PerceptionSLBoundary() const {
  return sl_boundary_;
}

void Obstacle::set_path_st_boundary(const STBoundary& boundary) {
  path_st_boundary_ = boundary;
  path_st_boundary_initialized_ = true;
}

void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType type) {
  path_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseStBoundary() { path_st_boundary_ = STBoundary(); }

void Obstacle::SetReferenceLineStBoundary(const STBoundary& boundary) {
  reference_line_st_boundary_ = boundary;
}

void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = STBoundary();
}

bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

void Obstacle::CheckLaneBlocking(const ReferenceLine& reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }
  DCHECK(sl_boundary_.has_start_s());
  DCHECK(sl_boundary_.has_end_s());
  DCHECK(sl_boundary_.has_start_l());
  DCHECK(sl_boundary_.has_end_l());

  if (sl_boundary_.start_l() * sl_boundary_.end_l() < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  const double driving_width = reference_line.GetDrivingWidth(sl_boundary_);
  auto vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

  if (reference_line.IsOnLane(sl_boundary_) &&
      driving_width <
          vehicle_param.width() + FLAGS_static_obstacle_nudge_l_buffer) {
    is_lane_blocking_ = true;
    return;
  }

  is_lane_blocking_ = false;
}

void Obstacle::SetLaneChangeBlocking(const bool is_distance_clear) {
  is_lane_change_blocking_ = is_distance_clear;
}

const std::pair<double, double>& Obstacle::GetLaneWidthBaseOnCenter(
    const ReferenceLine& reference_line) {
  if (!has_set_lane_width_) {
    double lane_left_width = FLAGS_default_reference_line_width * 0.5;
    double lane_right_width = FLAGS_default_reference_line_width * 0.5;
    double center_s = (sl_boundary_.start_s() + sl_boundary_.end_s()) * 0.5;
    if (!reference_line.GetLaneWidth(center_s, &lane_left_width,
                                     &lane_right_width)) {
      AWARN << "Failed to get lane width at obs_s = " << center_s;
    }
    lane_width_base_on_obs_center_.first = lane_left_width;
    lane_width_base_on_obs_center_.second = lane_right_width;
    has_set_lane_width_ = true;
  }
  return lane_width_base_on_obs_center_;
}

void Obstacle::CalculateSpeedHeading() {
  // AINFO << "OBS ID  = " << perception_obstacle_.id();
  if (perception_obstacle_.has_velocity()) {
    if (std::isnan(perception_obstacle_.velocity().x()) ||
        std::isnan(perception_obstacle_.velocity().y()) ||
        perception_obstacle_.velocity().x() > kAcousticVelocity ||
        perception_obstacle_.velocity().y() > kAcousticVelocity) {
      const auto perception_id = std::to_string(perception_obstacle_.id());
      AERROR << "id:" << perception_id << " invalid obstacle velocity:"
             << perception_obstacle_.velocity().DebugString();
      speed_ = 0.0;
    } else {
      speed_ = std::hypot(perception_obstacle_.velocity().x(),
                          perception_obstacle_.velocity().y());
    }
  }
  // AINFO << "get obs speed_ = " << speed_;
  if (speed_ >= FLAGS_min_dynamic_obstacle_speed) {
    double velocity_x = perception_obstacle_.velocity().x();
    double velocity_y = perception_obstacle_.velocity().y();
    speed_heading_ =
        common::math::NormalizeAngle(std::atan2(velocity_y, velocity_x));
    // AINFO << "  use trajectory  speed_heading_ = " << speed_heading_;
  } else {
    speed_heading_ = common::math::NormalizeAngle(perception_obstacle_.theta());
  }
  // AINFO << "speed_heading_ = " << speed_heading_;
  // AINFO << "perception_obstacle_.theta() = " << perception_obstacle_.theta();
}

double Obstacle::HeadingDiffWithLane(
    const ReferenceLine& reference_line) const {
  auto ref_point = reference_line.GetReferencePoint(sl_boundary_.start_s());
  const auto& traj_points = trajectory_.trajectory_point();
  double obs_theta = traj_points.empty() ? perception_obstacle_.theta()
                                         : traj_points[0].path_point().theta();
  return common::math::AngleDiff(ref_point.heading(), obs_theta);
}

bool Obstacle::IsLargeVehicle(const double check_length) const {
  if (perception::PerceptionObstacle::VEHICLE != perception_obstacle_.type()) {
    return false;
  }
  if ((sl_boundary_.end_s() - sl_boundary_.start_s() > check_length) ||
      perception_obstacle_.height() > kMinLargeVehicleHeight) {
    return true;
  }
  return false;
}

bool Obstacle::IsSmallUnknown(const double check_width) const {
  if (perception::PerceptionObstacle::UNKNOWN != perception_obstacle_.type()) {
    return false;
  }
  if (sl_boundary_.end_l() - sl_boundary_.start_l() < check_width) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace century
