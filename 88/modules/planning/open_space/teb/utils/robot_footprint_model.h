/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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

/*
 * @file
 */

#pragma once

#include <algorithm>
#include <limits>

#include <boost/smart_ptr.hpp>

#include "modules/planning/open_space/teb/g2o_types/teb_obstacles.h"
#include "modules/planning/open_space/teb/utils/pose_se2.h"

namespace century {
namespace planning {

/**
 * @class BaseRobotFootprintModel
 * @brief Abstract class that defines the interface for robot footprint/contour
 * models
 *
 * The robot model class is currently used in optimization only, since
 * taking the navigation stack footprint into account might be
 * inefficient. The footprint is only used for checking feasibility.
 */
class BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  BaseRobotFootprintModel() {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~BaseRobotFootprintModel() {}

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const = 0;

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const = 0;

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const BaseRobotFootprintModel>
    RobotFootprintModelConstPtr;

/**
 * @class PointRobotShape
 * @brief Class that defines a point-robot
 *
 * Instead of using a CircularRobotFootprint this class might
 * be utitilzed and the robot radius can be added to the mininum distance
 * parameter. This avoids a subtraction of zero each time a distance is
 * calculated.
 */
class PointRobotFootprint : public BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  PointRobotFootprint() {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~PointRobotFootprint() {}

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const {
    return obstacle->GetMinimumDistance(current_pose.position());
  }

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const {
    return obstacle->GetMinimumSpatioTemporalDistance(current_pose.position(),
                                                      t);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() { return 0.0; }
};

/**
 * @class CircularRobotFootprint
 * @brief Class that defines the a robot of circular shape
 */
class CircularRobotFootprint : public BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param radius radius of the robot
   */
  explicit CircularRobotFootprint(double radius) : radius_(radius) {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~CircularRobotFootprint() {}

  /**
   * @brief Set radius of the circular robot
   * @param radius radius of the robot
   */
  void SetRadius(double radius) { radius_ = radius; }

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const {
    return obstacle->GetMinimumDistance(current_pose.position()) - radius_;
  }

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const {
    return obstacle->GetMinimumSpatioTemporalDistance(current_pose.position(),
                                                      t) -
           radius_;
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() { return radius_; }

 private:
  double radius_;
};

/**
 * @class TwoCirclesRobotFootprint
 * @brief Class that approximates the robot with two shifted circles
 */
class TwoCirclesRobotFootprint : public BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param front_offset shift the center of the front circle along the robot
   * orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite
   * robot orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  TwoCirclesRobotFootprint(double front_offset, double front_radius,
                           double rear_offset, double rear_radius)
      : front_offset_(front_offset),
        front_radius_(front_radius),
        rear_offset_(rear_offset),
        rear_radius_(rear_radius) {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~TwoCirclesRobotFootprint() {}

  /**
   * @brief Set parameters of the contour/footprint
   * @param front_offset shift the center of the front circle along the robot
   * orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite
   * robot orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  void SetParameters(double front_offset, double front_radius,
                     double rear_offset, double rear_radius) {
    front_offset_ = front_offset;
    front_radius_ = front_radius;
    rear_offset_ = rear_offset;
    rear_radius_ = rear_radius;
  }

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const {
    Eigen::Vector2d dir = current_pose.OrientationUnitVec();
    double dist_front = obstacle->GetMinimumDistance(current_pose.position() +
                                                     front_offset_ * dir) -
                        front_radius_;
    double dist_rear = obstacle->GetMinimumDistance(current_pose.position() -
                                                    rear_offset_ * dir) -
                       rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const {
    Eigen::Vector2d dir = current_pose.OrientationUnitVec();
    double dist_front = obstacle->GetMinimumSpatioTemporalDistance(
                            current_pose.position() + front_offset_ * dir, t) -
                        front_radius_;
    double dist_rear = obstacle->GetMinimumSpatioTemporalDistance(
                           current_pose.position() - rear_offset_ * dir, t) -
                       rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() {
    double min_longitudinal =
        std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
    double min_lateral = std::min(rear_radius_, front_radius_);
    return std::min(min_longitudinal, min_lateral);
  }

 private:
  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;
};

/**
 * @class LineRobotFootprint
 * @brief Class that approximates the robot with line segment (zero-width)
 */
class LineRobotFootprint : public BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class (Eigen Version)
   * @param line_start start coordinates (only x and y) of the line (w.r.t.
   * robot center at (0,0))
   * @param line_end end coordinates (only x and y) of the line (w.r.t. robot
   * center at (0,0))
   */
  LineRobotFootprint(const Eigen::Vector2d& line_start,
                     const Eigen::Vector2d& line_end) {
    SetLine(line_start, line_end);
  }

  /**
   * @brief Virtual destructor.
   */
  virtual ~LineRobotFootprint() {}

  /**
   * @brief Set vertices of the contour/footprint (Eigen version)
   * @param vertices footprint vertices (only x and y) around the robot center
   * (0,0) (do not repeat the first and last vertex at the end)
   */
  void SetLine(const Eigen::Vector2d& line_start,
               const Eigen::Vector2d& line_end) {
    line_start_ = line_start;
    line_end_ = line_end;
  }

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    TransformToWorld(current_pose, &line_start_world, &line_end_world);
    return obstacle->GetMinimumDistance(line_start_world, line_end_world);
  }

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    TransformToWorld(current_pose, &line_start_world, &line_end_world);
    return obstacle->GetMinimumSpatioTemporalDistance(line_start_world,
                                                      line_end_world, t);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() {
    return 0.0;  // lateral distance = 0.0
  }

 private:
  /**
   * @brief Transforms a line to the world frame manually
   * @param current_pose Current robot pose
   * @param[out] line_start line_start_ in the world frame
   * @param[out] line_end line_end_ in the world frame
   */
  void TransformToWorld(const PoseSE2& current_pose,
                        Eigen::Vector2d* line_start_world,
                        Eigen::Vector2d* line_end_world) const {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    line_start_world->x() =
        current_pose.x() + cos_th * line_start_.x() - sin_th * line_start_.y();
    line_start_world->y() =
        current_pose.y() + sin_th * line_start_.x() + cos_th * line_start_.y();
    line_end_world->x() =
        current_pose.x() + cos_th * line_end_.x() - sin_th * line_end_.y();
    line_end_world->y() =
        current_pose.y() + sin_th * line_end_.x() + cos_th * line_end_.y();
  }

  Eigen::Vector2d line_start_;
  Eigen::Vector2d line_end_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class PolygonRobotFootprint
 * @brief Class that approximates the robot with a closed polygon
 */
class PolygonRobotFootprint : public BaseRobotFootprintModel {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param vertices footprint vertices (only x and y) around the robot center
   * (0,0) (do not repeat the first and last vertex at the end)
   */
  explicit PolygonRobotFootprint(const Point2dContainer& vertices)
      : vertices_(vertices) {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~PolygonRobotFootprint() {}

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot center
   * (0,0) (do not repeat the first and last vertex at the end)
   */
  void SetVertices(const Point2dContainer& vertices) { vertices_ = vertices; }

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double CalculateDistance(const PoseSE2& current_pose,
                                   const TebObstacle* obstacle) const {
    Point2dContainer polygon_world(vertices_.size());
    TransformToWorld(current_pose, &polygon_world);
    return obstacle->GetMinimumDistance(polygon_world);
  }

  /**
   * @brief Estimate the distance between the robot and the predicted location
   * of an obstacle at time t
   * @param current_pose robot pose, from which the distance to the obstacle is
   * estimated
   * @param obstacle Pointer to the dynamic obstacle (constant velocity model is
   * assumed)
   * @param t time, for which the predicted distance to the obstacle is
   * calculated
   * @return Euclidean distance to the robot
   */
  virtual double EstimateSpatioTemporalDistance(const PoseSE2& current_pose,
                                                const TebObstacle* obstacle,
                                                double t) const {
    Point2dContainer polygon_world(vertices_.size());
    TransformToWorld(current_pose, &polygon_world);
    return obstacle->GetMinimumSpatioTemporalDistance(polygon_world, t);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double GetInscribedRadius() {
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d center(0.0, 0.0);

    if (vertices_.size() <= 2) return 0.0;

    for (size_t i = 0; i < vertices_.size() - 1; ++i) {
      // compute distance from the robot center point to the first vertex
      double vertex_dist = vertices_[i].norm();
      double edge_dist =
          DistancePointToSegment2D(center, vertices_[i], vertices_[i + 1]);
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    }

    // we also need to check the last vertex and the first vertex
    double vertex_dist = vertices_.back().norm();
    double edge_dist =
        DistancePointToSegment2D(center, vertices_.back(), vertices_.front());
    return std::min(min_dist, std::min(vertex_dist, edge_dist));
  }

 private:
  /**
   * @brief Transforms a polygon to the world frame manually
   * @param current_pose Current robot pose
   * @param[out] polygon_world polygon in the world frame
   */
  void TransformToWorld(const PoseSE2& current_pose,
                        Point2dContainer* polygon_world) const {
    // double cos_th = std::cos(current_pose.theta());
    // double sin_th = std::sin(current_pose.theta());
    // for (std::size_t i = 0; i < vertices_.size(); ++i) {
    //   polygon_world->at(i).x() = current_pose.x() + cos_th * vertices_[i].x()
    //   -
    //                              sin_th * vertices_[i].y();
    //   polygon_world->at(i).y() = current_pose.y() + sin_th * vertices_[i].x()
    //   +
    //                              cos_th * vertices_[i].y();
    // }

    // lwt: now adc polygon is alreay generate in out , and directly use
    // and obstacle is in local_frame, diretly judge
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
      polygon_world->at(i).x() = current_pose.x() + vertices_[i].x();
      polygon_world->at(i).y() = current_pose.y() + vertices_[i].y();
    }
  }

  Point2dContainer vertices_;
};

}  // namespace planning
}  // namespace century
