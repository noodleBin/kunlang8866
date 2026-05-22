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

#include "modules/planning/open_space/teb/g2o_types/teb_obstacles.h"

namespace century {
namespace planning {

void PolygonObstacle::FixPolygonClosure() {
  if (vertices_.size() < 2) return;

  if (vertices_.front().isApprox(vertices_.back())) vertices_.pop_back();
}

void PolygonObstacle::CalcCentroid() {
  if (vertices_.empty()) {
    centroid_.setConstant(NAN);
    printf(
        "PolygonObstacle::CalcCentroid(): number of vertices is empty. the "
        "resulting centroid is a vector "
        "of NANs.");
    return;
  }

  // if polygon is a point
  if (NoVertices() == 1) {
    centroid_ = vertices_.front();
    return;
  }

  // if polygon is a line:
  if (NoVertices() == 2) {
    centroid_ = 0.5 * (vertices_.front() + vertices_.back());
    return;
  }

  // otherwise:

  centroid_.setZero();

  // calculate centroid (see wikipedia
  // http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
  double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
  for (int i = 0; i < NoVertices() - 1; ++i) {
    A += vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) -
         vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1);
  }
  A +=
      vertices_.at(NoVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1) -
      vertices_.at(0).coeffRef(0) * vertices_.at(NoVertices() - 1).coeffRef(1);
  A *= 0.5;

  if (A != 0) {
    for (int i = 0; i < NoVertices() - 1; ++i) {
      double aux =
          (vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) -
           vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1));
      centroid_ += (vertices_.at(i) + vertices_.at(i + 1)) * aux;
    }
    double aux = (vertices_.at(NoVertices() - 1).coeffRef(0) *
                      vertices_.at(0).coeffRef(1) -
                  vertices_.at(0).coeffRef(0) *
                      vertices_.at(NoVertices() - 1).coeffRef(1));
    centroid_ += (vertices_.at(NoVertices() - 1) + vertices_.at(0)) * aux;
    centroid_ /= (6 * A);
  } else {
    // A == 0 -> all points are placed on a 'perfect' line
    // seach for the two outer points of the line with the maximum distance
    // inbetween
    int i_cand = 0;
    int j_cand = 0;
    double max_dist = 0;
    for (int i = 0; i < NoVertices(); ++i) {
      // start with j=i+1
      for (int j = i + 1; j < NoVertices(); ++j) {
        double dist = (vertices_[j] - vertices_[i]).norm();
        if (dist > max_dist) {
          max_dist = dist;
          i_cand = i;
          j_cand = j;
        }
      }
    }
    // calc centroid of that line
    centroid_ = 0.5 * (vertices_[i_cand] + vertices_[j_cand]);
  }
}

Eigen::Vector2d PolygonObstacle::GetClosestPoint(
    const Eigen::Vector2d& position) const {
  // the polygon is a point
  if (NoVertices() == 1) {
    return vertices_.front();
  }

  if (NoVertices() > 1) {
    Eigen::Vector2d new_pt =
        ClosestPointOnLineSegment2D(position, vertices_.at(0), vertices_.at(1));

    // real polygon and not a line
    if (NoVertices() > 2) {
      double dist = (new_pt - position).norm();
      Eigen::Vector2d closest_pt = new_pt;

      // check each polygon edge
      // skip the first one, since we already checked it (new_pt)
      for (int i = 1; i < NoVertices() - 1; ++i) {
        new_pt = ClosestPointOnLineSegment2D(position, vertices_.at(i),
                                             vertices_.at(i + 1));
        double new_dist = (new_pt - position).norm();
        if (new_dist < dist) {
          dist = new_dist;
          closest_pt = new_pt;
        }
      }
      // afterwards we check the edge between goal and start (close polygon)
      new_pt = ClosestPointOnLineSegment2D(position, vertices_.back(),
                                           vertices_.front());
      double new_dist = (new_pt - position).norm();
      if (new_dist < dist)
        return new_pt;
      else
        return closest_pt;
    } else {
      return new_pt;  // closest point on line segment
    }
  }
  printf(
      "PolygonObstacle::GetClosestPoint() cannot find any closest point. "
      "Polygon ill-defined?");
  return Eigen::Vector2d::Zero();  // todo: maybe boost::optional?
}

bool PolygonObstacle::CheckLineIntersection(const Eigen::Vector2d& line_start,
                                            const Eigen::Vector2d& line_end,
                                            double min_dist) const {
  // Simple strategy, check all edge-line intersections until an intersection is
  // found... check each polygon edge
  for (int i = 0; i < NoVertices() - 1; ++i) {
    if (CheckLineSegmentsIntersection2D(line_start, line_end, vertices_.at(i),
                                        vertices_.at(i + 1)))
      return true;
  }
  if (NoVertices() == 2)  // if polygon is a line
    return false;

  return CheckLineSegmentsIntersection2D(
      line_start, line_end, vertices_.back(),
      vertices_.front());  // otherwise close polygon
}

// implements ToPolygonMsg() of the base class
void PolygonObstacle::ToPolygonMsg(Polygon* polygon) {
  polygon->points.resize(vertices_.size());
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    polygon->points[i].x = vertices_[i].x();
    polygon->points[i].y = vertices_[i].y();
    polygon->points[i].z = 0;
  }
}

}  // namespace planning
}  // namespace century
