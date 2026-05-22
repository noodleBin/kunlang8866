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
 * @file curve_math.h
 **/

#pragma once

#include <cmath>
// #include <stdexcept>
#include <iostream>
namespace century {
namespace planning {

class CirclePoint {
 public:
  CirclePoint() : x_(0), y_(0) {}
  CirclePoint(double x, double y) : x_(x), y_(y) {}
  double x() const { return x_; }
  double y() const { return y_; }

 private:
  double x_;
  double y_;
};

class Circle {
 public:
  Circle(const CirclePoint &a, const CirclePoint &b, const CirclePoint &c)
      : center_(0, 0) {
    double Ax = b.x() - a.x();
    double Ay = b.y() - a.y();
    double Bx = c.x() - a.x();
    double By = c.y() - a.y();

    double D = 2 * (Ax * By - Ay * Bx);
    if (D == 0) {
      //   AINFO << "The points are collinear and cannot determine a unique
      //   circle.";
      radius_ = 0.0;
      //   throw std::invalid_argument(
      //       "The points are collinear and cannot determine a unique
      //       circle.");
    } else {
      double Cx = ((Ax * Ax + Ay * Ay) * (a.x() + c.x()) -
                   (Bx * Bx + By * By) * (a.x() + b.x())) /
                  D;
      double Cy = ((Ax * Ax + Ay * Ay) * (a.y() + c.y()) -
                   (Bx * Bx + By * By) * (a.y() + b.y())) /
                  D;
      center_ = CirclePoint(Cx, Cy);
      radius_ =
          std::sqrt((Cx - b.x()) * (Cx - b.x()) + (Cy - b.y()) * (Cy - b.y()));
    }
  }

  double radius() const { return radius_; }
  CirclePoint center() const { return center_; }

 private:
  CirclePoint center_;
  double radius_;
};

class CurveMath {
 public:
  CurveMath() = delete;
  /**
   * @brief Compute the curvature (kappa) given curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @return the curvature
   */
  static double ComputeCurvature(const double dx, const double d2x,
                                 const double dy, const double d2y);

  /**
   * @brief Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  static double ComputeCurvatureDerivative(const double dx, const double d2x,
                                           const double d3x, const double dy,
                                           const double d2y, const double d3y);
};

}  // namespace planning
}  // namespace century
