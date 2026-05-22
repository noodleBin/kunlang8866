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

#include <complex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include "modules/planning/open_space/teb/utils/distance_calculations.h"
#include "modules/planning/open_space/teb/utils/teb_types.h"

namespace century {
namespace planning {

/**
 * @class TebObstacle
 * @brief Abstract class that defines the interface for modelling obstacles
 */
class TebObstacle {
 public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  TebObstacle()
      : dynamic_(false), centroid_velocity_(Eigen::Vector2d::Zero()) {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~TebObstacle() {}

  /** @name Centroid coordinates (abstract, obstacle type depending) */
  //@{

  /**
   * @brief Get centroid coordinates of the obstacle
   * @return Eigen::Vector2d containing the centroid
   */
  virtual const Eigen::Vector2d& GetCentroid() const = 0;

  /**
   * @brief Get centroid coordinates of the obstacle as complex number
   * @return std::complex containing the centroid coordinate
   */
  virtual std::complex<double> GetCentroidCplx() const = 0;

  //@}

  /** @name Collision checking and distance calculations (abstract, obstacle
   * type depending) */
  //@{

  /**
   * @brief Check if a given point collides with the obstacle
   * @param position 2D reference position that should be checked
   * @param min_dist Minimum distance allowed to the obstacle to be collision
   * free
   * @return \c true if position is inside the region of the obstacle or if the
   * minimum distance is lower than min_dist
   */
  virtual bool CheckCollision(const Eigen::Vector2d& position,
                              double min_dist) const = 0;

  /**
   * @brief Check if a given line segment between two points intersects with the
   * obstacle (and additionally keeps a safty distance \c min_dist)
   * @param line_start 2D point for the end of the reference line
   * @param line_end 2D point for the end of the reference line
   * @param min_dist Minimum distance allowed to the obstacle to be
   * collision/intersection free
   * @return \c true if given line intersects the region of the obstacle or if
   * the minimum distance is lower than min_dist
   */
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const = 0;

  /**
   * @brief Get the minimum euclidean distance to the obstacle (point as
   * reference)
   * @param position 2d reference position
   * @return The nearest possible distance to the obstacle
   */
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const = 0;

  /**
   * @brief Get the minimum euclidean distance to the obstacle (line as
   * reference)
   * @param line_start 2d position of the begin of the reference line
   * @param line_end 2d position of the end of the reference line
   * @return The nearest possible distance to the obstacle
   */
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const = 0;

  /**
   * @brief Get the minimum euclidean distance to the obstacle (polygon as
   * reference)
   * @param polygon Vertices (2D points) describing a closed polygon
   * @return The nearest possible distance to the obstacle
   */
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const = 0;

  /**
   * @brief Get the closest point on the boundary of the obstacle w.r.t. a
   * specified reference position
   * @param position reference 2d position
   * @return closest point on the obstacle boundary
   */
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const = 0;

  //@}

  /** @name Velocity related methods for non-static, moving obstacles */
  //@{

  /**
   * @brief Get the estimated minimum spatiotemporal distance to the moving
   * obstacle using a constant velocity model (point as reference)
   * @param position 2d reference position
   * @param t time, for which the minimum distance to the obstacle is estimated
   * @return The nearest possible distance to the obstacle at time t
   */
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const = 0;

  /**
   * @brief Get the estimated minimum spatiotemporal distance to the moving
   * obstacle using a constant velocity model (line as reference)
   * @param line_start 2d position of the begin of the reference line
   * @param line_end 2d position of the end of the reference line
   * @param t time, for which the minimum distance to the obstacle is estimated
   * @return The nearest possible distance to the obstacle at time t
   */
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const = 0;

  /**
   * @brief Get the estimated minimum spatiotemporal distance to the moving
   * obstacle using a constant velocity model (polygon as reference)
   * @param polygon Vertices (2D points) describing a closed polygon
   * @param t time, for which the minimum distance to the obstacle is estimated
   * @return The nearest possible distance to the obstacle at time t
   */
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const = 0;

  /**
   * @brief Predict position of the centroid assuming a constant velocity model
   * @param[in]  t         time in seconds for the prediction (t>=0)
   * @param[out] position  predicted 2d position of the centroid
   */
  virtual void PredictCentroidConstantVelocity(
      double t, Eigen::Ref<Eigen::Vector2d> position) const {
    position = GetCentroid() + t * GetCentroidVelocity();
  }

  /**
   * @brief Check if the obstacle is a moving with a (non-zero) velocity
   * @return \c true if the obstacle is not marked as static, \c false otherwise
   */
  bool IsDynamic() const { return dynamic_; }

  /**
   * @brief Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
   * @remarks Setting the velocity using this function marks the obstacle as
   * dynamic (@see IsDynamic)
   * @param vel 2D vector containing the velocities of the centroid in x and y
   * directions
   */
  void SetCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d>& vel) {
    centroid_velocity_ = vel;
    dynamic_ = true;
  }

  /**
   * @brief Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
   * @remarks Setting the velocity using this function marks the obstacle as
   * dynamic (@see IsDynamic)
   * @param velocity geometry_msgs::TwistWithCovariance containing the velocity
   * of the obstacle
   * @param orientation geometry_msgs::QuaternionStamped containing the
   * orientation of the obstacle
   */
  void SetCentroidVelocity(const TwistWithCovariance& velocity,
                           const Quaternion& orientation) {
    // Set velocity, if obstacle is moving
    Eigen::Vector2d vel;
    vel.coeffRef(0) = velocity.twist.linear.x();
    vel.coeffRef(1) = velocity.twist.linear.y();

    // If norm of velocity is less than 0.001, consider obstacle as not dynamic
    // TODO(all): Get rid of constant
    if (vel.norm() < 0.001) {
      return;
    }

    // currently velocity published by stage is already given in the map frame
    //    double yaw = tf::getYaw(orientation.quaternion);
    //    AINFO("Yaw: %f", yaw);
    //    Eigen::Rotation2Dd rot(yaw);
    //    vel = rot * vel;
    SetCentroidVelocity(vel);
  }

  void SetCentroidVelocity(const TwistWithCovariance& velocity,
                           const QuaternionStamped& orientation) {
    SetCentroidVelocity(velocity, orientation.quaternion);
  }

  /**
   * @brief Get the obstacle velocity (vx, vy) (w.r.t. to the centroid)
   * @returns 2D vector containing the velocities of the centroid in x and y
   * directions
   */
  const Eigen::Vector2d& GetCentroidVelocity() const {
    return centroid_velocity_;
  }

  //@}

  /** @name Helper Functions */
  //@{

  /**
   * @brief Convert the obstacle to a polygon message
   *
   * Convert the obstacle to a corresponding polygon msg.
   * Point obstacles have one vertex, lines have two vertices
   * and polygons might are implictly closed such that the start vertex must not
   * be repeated.
   * @param[out] polygon the polygon message
   */
  virtual void ToPolygonMsg(Polygon* polygon) = 0;

  virtual void ToTwistWithCovarianceMsg(
      TwistWithCovariance* twistWithCovariance) {
    if (dynamic_) {
      twistWithCovariance->twist.linear.x() = centroid_velocity_(0);
      twistWithCovariance->twist.linear.y() = centroid_velocity_(1);
    } else {
      twistWithCovariance->twist.linear.x() = 0;
      twistWithCovariance->twist.linear.y() = 0;
    }
  }

  //@}

 protected:
  bool dynamic_;  //!< Store flag if obstacle is dynamic (resp. a moving
                  //!< obstacle)
  Eigen::Vector2d
      centroid_velocity_;  //!< Store the corresponding velocity (vx, vy) of the
                           //!< centroid (zero, if _dynamic is \c true)

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<TebObstacle> ObstaclePtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const TebObstacle> ObstacleConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef std::vector<ObstaclePtr> ObstContainer;

/**
 * @class PointObstacle
 * @brief Implements a 2D point obstacle
 */
class PointObstacle : public TebObstacle {
 public:
  /**
   * @brief Default constructor of the point obstacle class
   */
  PointObstacle() : TebObstacle(), pos_(Eigen::Vector2d::Zero()) {}

  /**
   * @brief Construct PointObstacle using a 2d position vector
   * @param position 2d position that defines the current obstacle position
   */
  explicit PointObstacle(const Eigen::Ref<const Eigen::Vector2d>& position)
      : TebObstacle(), pos_(position) {}

  /**
   * @brief Construct PointObstacle using x- and y-coordinates
   * @param x x-coordinate
   * @param y y-coordinate
   */
  PointObstacle(double x, double y)
      : TebObstacle(), pos_(Eigen::Vector2d(x, y)) {}

  // implements CheckCollision() of the base class
  virtual bool CheckCollision(const Eigen::Vector2d& point,
                              double min_dist) const {
    return GetMinimumDistance(point) < min_dist;
  }

  // implements CheckLineIntersection() of the base class
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const {
    // Distance Line - Circle
    // refer to
    // http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
    Eigen::Vector2d a = line_end - line_start;  // not normalized!  a=y-x
    Eigen::Vector2d b = pos_ - line_start;      // b=m-x

    // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to
    // 0<=t<=1
    CHECK_NE(a.dot(a), 0.0);
    double t = a.dot(b) / a.dot(a);
    if (t < 0)
      t = 0;  // bound t (since a is not normalized, t can be scaled between 0
              // and 1 to parametrize the line
    else if (t > 1)
      t = 1;
    Eigen::Vector2d nearest_point = line_start + a * t;

    // check collision
    return CheckCollision(nearest_point, min_dist);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const {
    return (position - pos_).norm();
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const {
    return DistancePointToSegment2D(pos_, line_start, line_end);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const {
    return DistancePointToPolygon2D(pos_, polygon);
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const {
    return pos_;
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const {
    return (pos_ + t * centroid_velocity_ - position).norm();
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const {
    return DistancePointToSegment2D(pos_ + t * centroid_velocity_, line_start,
                                    line_end);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const {
    return DistancePointToPolygon2D(pos_ + t * centroid_velocity_, polygon);
  }

  // implements PredictCentroidConstantVelocity() of the base class
  virtual void PredictCentroidConstantVelocity(
      double t, Eigen::Ref<Eigen::Vector2d> position) const {
    position = pos_ + t * centroid_velocity_;
  }

  // implements GetCentroid() of the base class
  virtual const Eigen::Vector2d& GetCentroid() const { return pos_; }

  // implements GetCentroidCplx() of the base class
  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(pos_[0], pos_[1]);
  }

  // Accessor methods
  const Eigen::Vector2d& position() const {
    return pos_;
  }  //!< Return the current position of the obstacle (read-only)
  Eigen::Vector2d& position() {
    return pos_;
  }  //!< Return the current position of the obstacle
  double& x() {
    return pos_.coeffRef(0);
  }  //!< Return the current x-coordinate of the obstacle
  const double& x() const {
    return pos_.coeffRef(0);
  }  //!< Return the current y-coordinate of the obstacle (read-only)
  double& y() {
    return pos_.coeffRef(1);
  }  //!< Return the current x-coordinate of the obstacle
  const double& y() const {
    return pos_.coeffRef(1);
  }  //!< Return the current y-coordinate of the obstacle (read-only)

  // implements ToPolygonMsg() of the base class
  virtual void ToPolygonMsg(Polygon* polygon) {
    polygon->points.resize(1);
    polygon->points.front().x = pos_.x();
    polygon->points.front().y = pos_.y();
    polygon->points.front().z = 0;
  }

 protected:
  Eigen::Vector2d pos_;  //!< Store the position of the PointObstacle

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class CircularObstacle
 * @brief Implements a 2D circular obstacle (point obstacle plus radius)
 */
class CircularObstacle : public TebObstacle {
 public:
  /**
   * @brief Default constructor of the circular obstacle class
   */
  CircularObstacle() : TebObstacle(), pos_(Eigen::Vector2d::Zero()) {}

  /**
   * @brief Construct CircularObstacle using a 2d center position vector and
   * radius
   * @param position 2d position that defines the current obstacle position
   * @param radius radius of the obstacle
   */
  CircularObstacle(const Eigen::Ref<const Eigen::Vector2d>& position,
                   double radius)
      : TebObstacle(), pos_(position), radius_(radius) {}

  /**
   * @brief Construct CircularObstacle using x- and y-center-coordinates and
   * radius
   * @param x x-coordinate
   * @param y y-coordinate
   * @param radius radius of the obstacle
   */
  CircularObstacle(double x, double y, double radius)
      : TebObstacle(), pos_(Eigen::Vector2d(x, y)), radius_(radius) {}

  // implements CheckCollision() of the base class
  virtual bool CheckCollision(const Eigen::Vector2d& point,
                              double min_dist) const {
    return GetMinimumDistance(point) < min_dist;
  }

  // implements CheckLineIntersection() of the base class
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const {
    // Distance Line - Circle
    // refer to
    // http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
    Eigen::Vector2d a = line_end - line_start;  // not normalized!  a=y-x
    Eigen::Vector2d b = pos_ - line_start;      // b=m-x

    // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to
    // 0<=t<=1
    CHECK_NE(a.dot(a), 0.0);
    double t = a.dot(b) / a.dot(a);
    if (t < 0)
      t = 0;  // bound t (since a is not normalized, t can be scaled between 0
              // and 1 to parametrize the line
    else if (t > 1)
      t = 1;
    Eigen::Vector2d nearest_point = line_start + a * t;

    // check collision
    return CheckCollision(nearest_point, min_dist);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const {
    return (position - pos_).norm() - radius_;
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const {
    return DistancePointToSegment2D(pos_, line_start, line_end) - radius_;
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const {
    return DistancePointToPolygon2D(pos_, polygon) - radius_;
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const {
    return pos_ + radius_ * (position - pos_).normalized();
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const {
    return (pos_ + t * centroid_velocity_ - position).norm() - radius_;
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const {
    return DistancePointToSegment2D(pos_ + t * centroid_velocity_, line_start,
                                    line_end) -
           radius_;
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const {
    return DistancePointToPolygon2D(pos_ + t * centroid_velocity_, polygon) -
           radius_;
  }

  // implements PredictCentroidConstantVelocity() of the base class
  virtual void PredictCentroidConstantVelocity(
      double t, Eigen::Ref<Eigen::Vector2d> position) const {
    position = pos_ + t * centroid_velocity_;
  }

  // implements GetCentroid() of the base class
  virtual const Eigen::Vector2d& GetCentroid() const { return pos_; }

  // implements GetCentroidCplx() of the base class
  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(pos_[0], pos_[1]);
  }

  // Accessor methods
  const Eigen::Vector2d& position() const {
    return pos_;
  }  //!< Return the current position of the obstacle (read-only)
  Eigen::Vector2d& position() {
    return pos_;
  }  //!< Return the current position of the obstacle
  double& x() {
    return pos_.coeffRef(0);
  }  //!< Return the current x-coordinate of the obstacle
  const double& x() const {
    return pos_.coeffRef(0);
  }  //!< Return the current y-coordinate of the obstacle (read-only)
  double& y() {
    return pos_.coeffRef(1);
  }  //!< Return the current x-coordinate of the obstacle
  const double& y() const {
    return pos_.coeffRef(1);
  }  //!< Return the current y-coordinate of the obstacle (read-only)
  double& radius() {
    return radius_;
  }  //!< Return the current radius of the obstacle
  const double& radius() const {
    return radius_;
  }  //!< Return the current radius of the obstacle

  // implements ToPolygonMsg() of the base class
  virtual void ToPolygonMsg(Polygon* polygon) {
    // TODO(roesmann): the polygon message type cannot describe a "perfect"
    // circle
    // We could switch to ObstacleMsg if required somewhere...
    polygon->points.resize(1);
    polygon->points.front().x = pos_.x();
    polygon->points.front().y = pos_.y();
    polygon->points.front().z = 0;
  }

 protected:
  Eigen::Vector2d pos_;  //!< Store the center position of the CircularObstacle
  double radius_ = 0.0;  //!< Radius of the obstacle

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class LineObstacle
 * @brief Implements a 2D line obstacle
 */

class LineObstacle : public TebObstacle {
 public:
  //! Abbrev. for a container storing vertices (2d points defining the edge
  //! points of the polygon)
  typedef std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>>
      VertexContainer;

  /**
   * @brief Default constructor of the point obstacle class
   */
  LineObstacle() : TebObstacle() {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }

  /**
   * @brief Construct LineObstacle using 2d position vectors as start and end of
   * the line
   * @param line_start 2d position that defines the start of the line obstacle
   * @param line_end 2d position that defines the end of the line obstacle
   */
  LineObstacle(const Eigen::Ref<const Eigen::Vector2d>& line_start,
               const Eigen::Ref<const Eigen::Vector2d>& line_end)
      : TebObstacle(), start_(line_start), end_(line_end) {
    CalcCentroid();
  }

  /**
   * @brief Construct LineObstacle using start and end coordinates
   * @param x1 x-coordinate of the start of the line
   * @param y1 y-coordinate of the start of the line
   * @param x2 x-coordinate of the end of the line
   * @param y2 y-coordinate of the end of the line
   */
  LineObstacle(double x1, double y1, double x2, double y2) : TebObstacle() {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    CalcCentroid();
  }

  // implements CheckCollision() of the base class
  virtual bool CheckCollision(const Eigen::Vector2d& point,
                              double min_dist) const {
    return GetMinimumDistance(point) <= min_dist;
  }

  // implements CheckLineIntersection() of the base class
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const {
    return CheckLineSegmentsIntersection2D(line_start, line_end, start_, end_);
  }

  // implements GetMinimumDistance() of the base class
  // line min dist
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const {
    return DistancePointToSegment2D(position, start_, end_);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const {
    return DistanceSegmentToSegment2D(start_, end_, line_start, line_end);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const {
    return DistanceSegmentToPolygon2D(start_, end_, polygon);
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const {
    return ClosestPointOnLineSegment2D(position, start_, end_);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistancePointToSegment2D(position, start_ + offset, end_ + offset);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistanceSegmentToSegment2D(start_ + offset, end_ + offset,
                                      line_start, line_end);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistanceSegmentToPolygon2D(start_ + offset, end_ + offset, polygon);
  }

  // implements GetCentroid() of the base class
  virtual const Eigen::Vector2d& GetCentroid() const { return centroid_; }

  // implements GetCentroidCplx() of the base class
  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }

  // Access or modify line
  const Eigen::Vector2d& Start() const { return start_; }
  void SetStart(const Eigen::Ref<const Eigen::Vector2d>& start) {
    start_ = start;
    CalcCentroid();
  }
  const Eigen::Vector2d& End() const { return end_; }
  void SetEnd(const Eigen::Ref<const Eigen::Vector2d>& end) {
    end_ = end;
    CalcCentroid();
  }

  // implements ToPolygonMsg() of the base class
  virtual void ToPolygonMsg(Polygon* polygon) {
    polygon->points.resize(2);
    polygon->points.front().x = start_.x();
    polygon->points.front().y = start_.y();

    polygon->points.back().x = end_.x();
    polygon->points.back().y = end_.y();
    polygon->points.back().z = polygon->points.front().z = 0;
  }

 protected:
  void CalcCentroid() { centroid_ = 0.5 * (start_ + end_); }

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;

  Eigen::Vector2d centroid_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class PillObstacle
 * @brief Implements a 2D pill/stadium/capsular-shaped obstacle (line +
 * distance/radius)
 */

class PillObstacle : public TebObstacle {
 public:
  /**
   * @brief Default constructor of the point obstacle class
   */
  PillObstacle() : TebObstacle() {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }

  /**
   * @brief Construct LineObstacle using 2d position vectors as start and end of
   * the line
   * @param line_start 2d position that defines the start of the line obstacle
   * @param line_end 2d position that defines the end of the line obstacle
   */
  PillObstacle(const Eigen::Ref<const Eigen::Vector2d>& line_start,
               const Eigen::Ref<const Eigen::Vector2d>& line_end, double radius)
      : TebObstacle(), start_(line_start), end_(line_end), radius_(radius) {
    CalcCentroid();
  }

  /**
   * @brief Construct LineObstacle using start and end coordinates
   * @param x1 x-coordinate of the start of the line
   * @param y1 y-coordinate of the start of the line
   * @param x2 x-coordinate of the end of the line
   * @param y2 y-coordinate of the end of the line
   */
  PillObstacle(double x1, double y1, double x2, double y2, double radius)
      : TebObstacle(), radius_(radius) {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    CalcCentroid();
  }

  // implements CheckCollision() of the base class
  virtual bool CheckCollision(const Eigen::Vector2d& point,
                              double min_dist) const {
    return GetMinimumDistance(point) <= min_dist;
  }

  // implements CheckLineIntersection() of the base class
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const {
    return CheckLineSegmentsIntersection2D(line_start, line_end, start_, end_);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const {
    return DistancePointToSegment2D(position, start_, end_) - radius_;
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const {
    return DistanceSegmentToSegment2D(start_, end_, line_start, line_end) -
           radius_;
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const {
    return DistanceSegmentToPolygon2D(start_, end_, polygon) - radius_;
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const {
    Eigen::Vector2d closed_point_line =
        ClosestPointOnLineSegment2D(position, start_, end_);
    return closed_point_line +
           radius_ * (position - closed_point_line).normalized();
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistancePointToSegment2D(position, start_ + offset, end_ + offset) -
           radius_;
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistanceSegmentToSegment2D(start_ + offset, end_ + offset,
                                      line_start, line_end) -
           radius_;
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const {
    Eigen::Vector2d offset = t * centroid_velocity_;
    return DistanceSegmentToPolygon2D(start_ + offset, end_ + offset, polygon) -
           radius_;
  }

  // implements GetCentroid() of the base class
  virtual const Eigen::Vector2d& GetCentroid() const { return centroid_; }

  // implements GetCentroidCplx() of the base class
  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }

  // Access or modify line
  const Eigen::Vector2d& Start() const { return start_; }
  void SetStart(const Eigen::Ref<const Eigen::Vector2d>& start) {
    start_ = start;
    CalcCentroid();
  }
  const Eigen::Vector2d& End() const { return end_; }
  void SetEnd(const Eigen::Ref<const Eigen::Vector2d>& end) {
    end_ = end;
    CalcCentroid();
  }

  // implements ToPolygonMsg() of the base class
  virtual void ToPolygonMsg(Polygon* polygon) {
    // Currently, we only export the line
    // TODO(roesmann): export whole pill
    polygon->points.resize(2);
    polygon->points.front().x = start_.x();
    polygon->points.front().y = start_.y();

    polygon->points.back().x = end_.x();
    polygon->points.back().y = end_.y();
    polygon->points.back().z = polygon->points.front().z = 0;
  }

 protected:
  void CalcCentroid() { centroid_ = 0.5 * (start_ + end_); }

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  double radius_ = 0.0;

  Eigen::Vector2d centroid_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class PolygonObstacle
 * @brief Implements a polygon obstacle with an arbitrary number of vertices
 * @details If the polygon has only 2 vertices, than it is considered as a line,
 * 	    otherwise the polygon will always be closed (a connection between
 * the first and the last vertex is included automatically).
 */
class PolygonObstacle : public TebObstacle {
 public:
  /**
   * @brief Default constructor of the polygon obstacle class
   */
  PolygonObstacle() : TebObstacle(), finalized_(false) {
    centroid_.setConstant(NAN);
  }

  /**
   * @brief Construct polygon obstacle with a list of vertices
   */
  explicit PolygonObstacle(const Point2dContainer& vertices)
      : TebObstacle(), vertices_(vertices) {
    FinalizePolygon();
  }

  /* FIXME Not working at the moment due to the aligned allocator version of
  std::vector
    * And it is C++11 code that is disabled atm to ensure compliance with ROS
  indigo/jade template <typename... Vector2dType> PolygonObstacle(const
  Vector2dType&... vertices) : _vertices({vertices...})
  {
    CalcCentroid();
    _finalized = true;
  }
  */

  // implements CheckCollision() of the base class
  virtual bool CheckCollision(const Eigen::Vector2d& point,
                              double min_dist) const {
    // line case
    if (NoVertices() == 2) return GetMinimumDistance(point) <= min_dist;

    // check if point is in the interior of the polygon
    // point in polygon test - raycasting
    // (http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html)
    // using the following algorithm we may obtain false negatives on
    // edge-cases, but that's ok for our purposes
    int i, j;
    bool c = false;
    for (i = 0, j = NoVertices() - 1; i < NoVertices(); j = i++) {
      CHECK_NE(vertices_.at(j).y(), vertices_.at(i).y());
      if (((vertices_.at(i).y() > point.y()) !=
           (vertices_.at(j).y() > point.y())) &&
          (point.x() < (vertices_.at(j).x() - vertices_.at(i).x()) *
                               (point.y() - vertices_.at(i).y()) /
                               (vertices_.at(j).y() - vertices_.at(i).y()) +
                           vertices_.at(i).x()))
        c = !c;
    }
    if (c > 0) return true;

    // If this statement is reached, the point lies outside the polygon or maybe
    // on its edges Let us check the minium distance as well
    return min_dist == 0 ? false : GetMinimumDistance(point) < min_dist;
  }

  /**
   * @brief Check if a given line segment between two points intersects with the
   * obstacle (and additionally keeps a safty distance \c min_dist)
   * @param line_start 2D point for the end of the reference line
   * @param line_end 2D point for the end of the reference line
   * @param min_dist Minimum distance allowed to the obstacle to be
   * collision/intersection free
   * @remarks we ignore \c min_dist here
   * @return \c true if given line intersects the region of the obstacle or if
   * the minimum distance is lower than min_dist
   */
  virtual bool CheckLineIntersection(const Eigen::Vector2d& line_start,
                                     const Eigen::Vector2d& line_end,
                                     double min_dist = 0) const;

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& position) const {
    return DistancePointToPolygon2D(position, vertices_);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Eigen::Vector2d& line_start,
                                    const Eigen::Vector2d& line_end) const {
    return DistanceSegmentToPolygon2D(line_start, line_end, vertices_);
  }

  // implements GetMinimumDistance() of the base class
  virtual double GetMinimumDistance(const Point2dContainer& polygon) const {
    return DistancePolygonToPolygon2D(polygon, vertices_);
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d GetClosestPoint(
      const Eigen::Vector2d& position) const;

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& position, double t) const {
    Point2dContainer pred_vertices;
    PredictVertices(t, &pred_vertices);
    return DistancePointToPolygon2D(position, pred_vertices);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
      double t) const {
    Point2dContainer pred_vertices;
    PredictVertices(t, &pred_vertices);
    return DistanceSegmentToPolygon2D(line_start, line_end, pred_vertices);
  }

  // implements GetMinimumSpatioTemporalDistance() of the base class
  virtual double GetMinimumSpatioTemporalDistance(
      const Point2dContainer& polygon, double t) const {
    Point2dContainer pred_vertices;
    PredictVertices(t, &pred_vertices);
    return DistancePolygonToPolygon2D(polygon, pred_vertices);
  }

  virtual void PredictVertices(double t,
                               Point2dContainer* pred_vertices) const {
    // Predict obstacle (polygon) at time t
    pred_vertices->resize(vertices_.size());
    Eigen::Vector2d offset = t * centroid_velocity_;
    for (std::size_t i = 0; i < vertices_.size(); i++) {
      pred_vertices->at(i) = vertices_[i] + offset;
    }
  }

  // implements GetCentroid() of the base class
  virtual const Eigen::Vector2d& GetCentroid() const {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return centroid_;
  }

  // implements GetCentroidCplx() of the base class
  virtual std::complex<double> GetCentroidCplx() const {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return std::complex<double>(centroid_.coeffRef(0), centroid_.coeffRef(1));
  }

  // implements ToPolygonMsg() of the base class
  virtual void ToPolygonMsg(Polygon* polygon);

  /** @name Define the polygon */
  ///@{

  // Access or modify polygon
  const Point2dContainer& Vertices() const {
    return vertices_;
  }  //!< Access vertices container (read-only)
  Point2dContainer& Vertices() {
    return vertices_;
  }  //!< Access vertices container

  /**
   * @brief Add a vertex to the polygon (edge-point)
   * @remarks You do not need to close the polygon (do not repeat the first
   * vertex)
   * @warning Do not forget to call finalizePolygon() after adding all vertices
   * @param vertex 2D point defining a new polygon edge
   */
  void PushBackVertex(const Eigen::Ref<const Eigen::Vector2d>& vertex) {
    vertices_.push_back(vertex);
    finalized_ = false;
  }

  /**
   * @brief Add a vertex to the polygon (edge-point)
   * @remarks You do not need to close the polygon (do not repeat the first
   * vertex)
   * @warning Do not forget to call finalizePolygon() after adding all vertices
   * @param x x-coordinate of the new vertex
   * @param y y-coordinate of the new vertex
   */
  void PushBackVertex(double x, double y) {
    vertices_.push_back(Eigen::Vector2d(x, y));
    finalized_ = false;
  }

  /**
   * @brief Call finalizePolygon after the polygon is created with the help of
   * pushBackVertex() methods
   */
  void FinalizePolygon() {
    FixPolygonClosure();
    CalcCentroid();
    finalized_ = true;
  }

  /**
   * @brief Clear all vertices (Afterwards the polygon is not valid anymore)
   */
  void ClearVertices() {
    vertices_.clear();
    finalized_ = false;
  }

  /**
   * @brief Get the number of vertices defining the polygon (the first vertex is
   * counted once)
   */
  int NoVertices() const { return static_cast<int>(vertices_.size()); }

  ///@}

 protected:
  void FixPolygonClosure();  //!< Check if the current polygon contains the
                             //!< first vertex twice (as start and end) and in
                             //!< that case erase the last redundant one.

  void CalcCentroid();  //!< Compute the centroid of the polygon (called inside
                        //!< finalizePolygon())

  Point2dContainer
      vertices_;  //!< Store vertices defining the polygon (@see pushBackVertex)
  Eigen::Vector2d centroid_;  //!< Store the centroid coordinates of the polygon
                              //!< (@see CalcCentroid)

  bool finalized_;  //!< Flat that keeps track if the polygon was finalized
                    //!< after adding all vertices

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
