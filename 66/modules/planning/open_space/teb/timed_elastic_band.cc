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

#include "modules/planning/open_space/teb/timed_elastic_band.h"

#include <algorithm>
#include <limits>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"

namespace century {
namespace planning {

/**
 * estimate the time to move from start to end.
 * Assumes constant velocity for the motion.
 */
double EstimateDeltaT(const PoseSE2& start, const PoseSE2& end,
                      double max_vel_x, double max_vel_theta) {
  double dt_constant_motion = 0.1;
  if (max_vel_x > 0) {
    double trans_dist = (end.position() - start.position()).norm();
    dt_constant_motion = trans_dist / max_vel_x;
  }
  if (max_vel_theta > 0) {
    double rot_dist =
        std::abs(g2o::normalize_theta(end.theta() - start.theta()));
    dt_constant_motion = std::max(dt_constant_motion, rot_dist / max_vel_theta);
  }
  return dt_constant_motion;
}

TimedElasticBand::TimedElasticBand() {}

TimedElasticBand::~TimedElasticBand() {
  AERROR << "Destructor Timed_Elastic_Band...";
  ClearTimedElasticBand();
}

void TimedElasticBand::AddPose(const PoseSE2& pose, bool fixed) {
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TimedElasticBand::AddPose(
    const Eigen::Ref<const Eigen::Vector2d>& position, double theta,
    bool fixed) {
  VertexPose* pose_vertex = new VertexPose(position, theta, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TimedElasticBand::AddPose(double x, double y, double theta, bool fixed) {
  VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TimedElasticBand::AddTimeDiff(double dt, bool fixed) {
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
  timediff_vec_.push_back(timediff_vertex);
  return;
}

void TimedElasticBand::AddPoseAndTimeDiff(double x, double y, double angle,
                                          double dt) {
  if (SizePoses() != SizeTimeDiffs()) {
    AddPose(x, y, angle, false);
    AddTimeDiff(dt, false);
  } else {
    AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff "
              "describes the time difference between last conf and given conf";
  }

  return;
}

void TimedElasticBand::AddPoseAndTimeDiff(const PoseSE2& pose, double dt) {
  if (SizePoses() != SizeTimeDiffs()) {
    AddPose(pose, false);
    AddTimeDiff(dt, false);
  } else {
    AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff "
              "describes the time difference between last conf and given conf";
  }

  return;
}

void TimedElasticBand::AddPoseAndTimeDiff(
    const Eigen::Ref<const Eigen::Vector2d>& position, double theta,
    double dt) {
  if (SizePoses() != SizeTimeDiffs()) {
    AddPose(position, theta, false);
    AddTimeDiff(dt, false);
  } else {
    AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff "
              "describes the time difference between last conf and given conf";
  }

  return;
}

void TimedElasticBand::DeletePose(int index) {
  delete pose_vec_.at(index);
  pose_vec_.at(index) = nullptr;
  pose_vec_.erase(pose_vec_.begin() + index);
}

void TimedElasticBand::DeletePoses(int index, int number) {
  for (int i = index; i < index + number; ++i) {
    delete pose_vec_.at(i);
    pose_vec_.at(i) = nullptr;
  }
  pose_vec_.erase(pose_vec_.begin() + index,
                  pose_vec_.begin() + index + number);
}

void TimedElasticBand::DeleteTimeDiff(int index) {
  delete timediff_vec_.at(index);
  timediff_vec_.at(index) = nullptr;
  timediff_vec_.erase(timediff_vec_.begin() + index);
}

void TimedElasticBand::DeleteTimeDiffs(int index, int number) {
  for (int i = index; i < index + number; ++i) {
    delete timediff_vec_.at(i);
    timediff_vec_.at(i) = nullptr;
  }
  timediff_vec_.erase(timediff_vec_.begin() + index,
                      timediff_vec_.begin() + index + number);
}

void TimedElasticBand::InsertPose(int index, const PoseSE2& pose) {
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
}

void TimedElasticBand::InsertPose(
    int index, const Eigen::Ref<const Eigen::Vector2d>& position,
    double theta) {
  VertexPose* pose_vertex = new VertexPose(position, theta);
  pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
}

void TimedElasticBand::InsertPose(int index, double x, double y, double theta) {
  VertexPose* pose_vertex = new VertexPose(x, y, theta);
  pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
}

void TimedElasticBand::InsertTimeDiff(int index, double dt) {
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin() + index, timediff_vertex);
}

void TimedElasticBand::ClearTimedElasticBand() {
  for (PoseSequence::iterator pose_it = pose_vec_.begin();
       pose_it != pose_vec_.end(); ++pose_it) {
    delete *pose_it;
    *pose_it = nullptr;
  }
  pose_vec_.clear();

  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin();
       dt_it != timediff_vec_.end(); ++dt_it) {
    delete *dt_it;
    *dt_it = nullptr;
  }
  timediff_vec_.clear();
}

void TimedElasticBand::SetPoseVertexFixed(int index, bool status) {
  pose_vec_.at(index)->setFixed(status);
}

void TimedElasticBand::SetTimeDiffVertexFixed(int index, bool status) {
  timediff_vec_.at(index)->setFixed(status);
}

void TimedElasticBand::AutoResize(double dt_ref, double dt_hysteresis,
                                  int min_samples, int max_samples,
                                  bool fast_mode) {
  /// iterate through all TEB states and add/remove states!
  bool modified = true;

  // actually it should be while(), but we want to make sure to
  // not get stuck in some oscillation, hence max 100 repitions.
  for (int rep = 0; rep < 100 && modified; ++rep) {
    modified = false;

    // TimeDiff connects Point(i) with Point(i+1)
    for (int i = 0; i < SizeTimeDiffs(); ++i) {
      if (TimeDiff(i) > dt_ref + dt_hysteresis &&
          SizeTimeDiffs() < max_samples) {
        if (TimeDiff(i) > 2 * dt_ref) {
          double newtime = 0.5 * TimeDiff(i);

          TimeDiff(i) = newtime;
          InsertPose(i + 1, PoseSE2::Average(Pose(i), Pose(i + 1)));
          InsertTimeDiff(i + 1, newtime);

          i--;  // check the updated pose diff again
          modified = true;
        } else {
          if (i < SizeTimeDiffs() - 1) {
            Timediffs().at(i + 1)->dt() += Timediffs().at(i)->dt() - dt_ref;
          }
          Timediffs().at(i)->dt() = dt_ref;
        }
      } else if (TimeDiff(i) < dt_ref - dt_hysteresis &&
                 SizeTimeDiffs() > min_samples) {
        // only remove samples if size
        // is larger than min_samples.

        // delete point
        if (i < (SizeTimeDiffs() - 1)) {
          TimeDiff(i + 1) = TimeDiff(i + 1) + TimeDiff(i);
          DeleteTimeDiff(i);
          DeletePose(i + 1);
        } else {
          // last motion should be adjusted, shift time to the interval
          // before
          TimeDiff(i - 1) += TimeDiff(i);
          DeleteTimeDiff(i);
          DeletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
  }
}

double TimedElasticBand::GetSumOfAllTimeDiffs() const {
  double time = 0;

  for (TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin();
       dt_it != timediff_vec_.end(); ++dt_it) {
    time += (*dt_it)->dt();
  }
  return time;
}

double TimedElasticBand::GetSumOfTimeDiffsUpToIdx(int index) const {
  double time = 0;

  for (int i = 0; i < index; ++i) {
    time += timediff_vec_.at(i)->dt();
  }

  return time;
}

double TimedElasticBand::GetAccumulatedDistance() const {
  double dist = 0;

  for (int i = 1; i < SizePoses(); ++i) {
    dist += (Pose(i).position() - Pose(i - 1).position()).norm();
  }
  return dist;
}

bool TimedElasticBand::InitTrajectoryToGoal(const PoseSE2& start,
                                            const PoseSE2& goal,
                                            double diststep, double max_vel_x,
                                            int min_samples,
                                            bool guess_backwards_motion) {
  if (!IsInit()) {
    // add starting point
    AddPose(start);
    // StartConf is a fixed constraint during optimization
    SetPoseVertexFixed(0, true);

    double timestep = 0.1;

    if (diststep != 0) {
      Eigen::Vector2d point_to_goal = goal.position() - start.position();
      // direction to goal
      double dir_to_goal = std::atan2(point_to_goal[1], point_to_goal[0]);
      double dx = diststep * std::cos(dir_to_goal);
      double dy = diststep * std::sin(dir_to_goal);
      double orient_init = dir_to_goal;
      // check if the goal is behind the start pose (w.r.t. start
      // orientation)
      if (guess_backwards_motion &&
          point_to_goal.dot(start.OrientationUnitVec()) < 0) {
        orient_init = g2o::normalize_theta(orient_init + M_PI);
      }
      // TODO(all): timestep ~ max_vel_x_backwards for backwards motions

      double dist_to_goal = point_to_goal.norm();
      // ignore negative values
      double no_steps_d = dist_to_goal / std::abs(diststep);
      unsigned int no_steps = (unsigned int)std::floor(no_steps_d);

      if (max_vel_x > 0) timestep = diststep / max_vel_x;

      // start with 1! starting point had index 0
      for (unsigned int i = 1; i <= no_steps; i++) {
        // if (i == no_steps && no_steps_d == (float)no_steps) break;
        if (i == no_steps && no_steps_d == static_cast<float>(no_steps)) {
          break;
        }

        // if last conf (depending on stepsize) is equal to goal conf
        // -> leave loop
        AddPoseAndTimeDiff(start.x() + i * dx, start.y() + i * dy, orient_init,
                           timestep);
      }
    }

    // if number of samples is not larger than min_samples, insert manually
    if (SizePoses() < min_samples - 1) {
      AERROR << "initTEBtoGoal(): number of generated samples is less than "
                "specified by min_samples. Forcing the insertion of more "
                "samples...";
      // subtract goal point that will be added later
      while (SizePoses() < min_samples - 1) {
        // simple strategy: interpolate between the current pose and the
        // goal
        PoseSE2 intermediate_pose = PoseSE2::Average(BackPose(), goal);
        if (max_vel_x > 0) {
          timestep =
              (intermediate_pose.position() - BackPose().position()).norm() /
              max_vel_x;
        }

        // let the optimier correct the timestep(TODO: better initialization
        AddPoseAndTimeDiff(intermediate_pose, timestep);
      }
    }

    // add goal
    if (max_vel_x > 0) {
      timestep = (goal.position() - BackPose().position()).norm() / max_vel_x;
    }
    AddPoseAndTimeDiff(goal, timestep);  // add goal point
    // GoalConf is a fixed constraint during optimization
    SetPoseVertexFixed(SizePoses() - 1, true);
  } else {
    // size!=0
    AERROR << "Cannot init TEB between given configuration and goal, because "
              "TEB vectors are not empty or TEB is already initialized (call "
              "this function before adding states yourself)!";
    AERROR << "Number of TEB configurations:" << (unsigned int)SizePoses()
           << ",Number of TEB timediffs:" << (unsigned int)SizeTimeDiffs();
    return false;
  }
  return true;
}

bool TimedElasticBand::InitTrajectoryToGoal(
    const std::vector<PoseStamped>& plan, double max_vel_x,
    double max_vel_theta, bool estimate_orient, int min_samples,
    bool guess_backwards_motion) {
  if (!IsInit()) {
    PoseSE2 start(plan.front().pose.position.x, plan.front().pose.position.y,
                  plan.front().pose.orientation.w);
    PoseSE2 goal(plan.back().pose.position.x, plan.back().pose.position.y,
                 plan.back().pose.orientation.w);
    // --------------------------------------------

    // add starting point with given orientation
    AddPose(start);
    // StartConf is a fixed constraint during optimization
    SetPoseVertexFixed(0, true);

    bool backwards = false;
    if (guess_backwards_motion &&
        (goal.position() - start.position()).dot(start.OrientationUnitVec()) <
            0) {
      // check if the goal is behind the start pose (w.r.t. start
      // orientation)
      backwards = true;
      // TODO(all): dt ~ max_vel_x_backwards for backwards motions
    }

    for (int i = 1; i < static_cast<int>(plan.size()) - 1; ++i) {
      double yaw;
      if (estimate_orient) {
        // get yaw from the orientation of the distance vector between
        // pose_{i+1} and pose_{i}
        double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
        double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
        yaw = std::atan2(dy, dx);
        if (backwards) {
          yaw = g2o::normalize_theta(yaw + M_PI);
        }
      } else {
        // Eigen::Quaterniond q{
        //     plan[i].pose.orientation.w, plan[i].pose.orientation.x,
        //     plan[i].pose.orientation.y, plan[i].pose.orientation.z};
        // Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2, 1, 0);
        // yaw = eulerAngle[0];

        // test interface
        yaw = plan[i].pose.orientation.w;
      }
      PoseSE2 intermediate_pose(plan[i].pose.position.x,
                                plan[i].pose.position.y, yaw);
      double dt = EstimateDeltaT(BackPose(), intermediate_pose, max_vel_x,
                                 max_vel_theta);
      AddPoseAndTimeDiff(intermediate_pose, dt);
    }

    // if number of samples is not larger than min_samples, insert manually
    if (SizePoses() < min_samples - 1) {
      AERROR << "initTEBtoGoal(): number of generated samples is less than "
                "specified by min_samples. Forcing the insertion of more "
                "samples...";
      // subtract goal point that will be added later
      while (SizePoses() < min_samples - 1) {
        // simple strategy: interpolate between the current pose and the
        // goal
        PoseSE2 intermediate_pose = PoseSE2::Average(BackPose(), goal);
        double dt = EstimateDeltaT(BackPose(), intermediate_pose, max_vel_x,
                                   max_vel_theta);
        // let the optimier correct the timestep(TODO: better initialization
        AddPoseAndTimeDiff(intermediate_pose, dt);
      }
    }

    // Now add final state with given orientation
    double dt = EstimateDeltaT(BackPose(), goal, max_vel_x, max_vel_theta);
    AddPoseAndTimeDiff(goal, dt);
    // GoalConf is a fixed constraint during optimization
    SetPoseVertexFixed(SizePoses() - 1, true);
  } else {
    // size!=0
    AERROR << "Cannot init TEB between given configuration and goal, because "
              "TEB vectors are not empty or TEB is already initialized (call "
              "this function before adding states yourself)!";
    AERROR << "Number of TEB configurations:" << (unsigned int)SizePoses()
           << ",Number of TEB timediffs:" << (unsigned int)SizeTimeDiffs();
    return false;
  }

  return true;
}

int TimedElasticBand::FindClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance,
    int begin_idx) const {
  int n = SizePoses();
  if (begin_idx < 0 || begin_idx >= n) {
    return -1;
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = begin_idx; i < n; i++) {
    double dist_sq = (ref_point - Pose(i).position()).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }

  if (distance) {
    *distance = std::sqrt(min_dist_sq);
  }

  return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d>& ref_line_start,
    const Eigen::Ref<const Eigen::Vector2d>& ref_line_end,
    double* distance) const {
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < SizePoses(); i++) {
    Eigen::Vector2d point = Pose(i).position();
    double dist = DistancePointToSegment2D(point, ref_line_start, ref_line_end);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  if (distance) {
    *distance = min_dist;
  }

  return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(
    const Point2dContainer& vertices, double* distance) const {
  if (vertices.empty())
    return 0;
  else if (vertices.size() == 1)
    return FindClosestTrajectoryPose(vertices.front());
  else if (vertices.size() == 2)
    return FindClosestTrajectoryPose(vertices.front(), vertices.back());

  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < SizePoses(); i++) {
    Eigen::Vector2d point = Pose(i).position();
    double dist_to_polygon = std::numeric_limits<double>::max();
    for (int j = 0; j < static_cast<int>(vertices.size()) - 1; ++j) {
      dist_to_polygon = std::min(
          dist_to_polygon,
          DistancePointToSegment2D(point, vertices[j], vertices[j + 1]));
    }
    dist_to_polygon = std::min(
        dist_to_polygon,
        DistancePointToSegment2D(point, vertices.back(), vertices.front()));
    if (dist_to_polygon < min_dist) {
      min_dist = dist_to_polygon;
      min_idx = i;
    }
  }

  if (distance) {
    *distance = min_dist;
  }

  return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(const TebObstacle& obstacle,
                                                double* distance) const {
  const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
  if (pobst) return FindClosestTrajectoryPose(pobst->position(), distance);

  const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
  if (lobst)
    return FindClosestTrajectoryPose(lobst->Start(), lobst->End(), distance);

  const PolygonObstacle* polyobst =
      dynamic_cast<const PolygonObstacle*>(&obstacle);
  if (polyobst)
    return FindClosestTrajectoryPose(polyobst->Vertices(), distance);

  return FindClosestTrajectoryPose(obstacle.GetCentroid(), distance);
}

void TimedElasticBand::UpdateAndPruneTEB(
    boost::optional<const PoseSE2&> new_start,
    boost::optional<const PoseSE2&> new_goal, int min_samples) {
  // first and simple approach: change only start confs (and virtual start
  // conf for inital velocity) TEST if optimizer can handle this "hard"
  // placement

  // if (new_start && SizePoses() > 0) {
  //   // find nearest state (using l2-norm) in order to prune the trajectory
  //   // (remove already passed states)
  //   double dist_cache = (new_start->position() - Pose(0).position()).norm();
  //   double dist;
  //   // satisfy min_samples, otherwise max 10 samples
  //   int lookahead = std::min<int>(SizePoses() - min_samples, 10);

  //   int nearest_idx = 0;
  //   for (int i = 1; i <= lookahead; ++i) {
  //     dist = (new_start->position() - Pose(i).position()).norm();
  //     if (dist < dist_cache) {
  //       dist_cache = dist;
  //       nearest_idx = i;
  //     } else {
  //       break;
  //     }
  //   }

  //   // prune trajectory at the beginning (and extrapolate sequences at the
  //   // end if the horizon is fixed)
  //   if (nearest_idx > 0) {
  //     // nearest_idx is equal to the number of samples to be removed (since
  //     // it counts from 0 ;-) ) WARNING delete starting at pose 1, and
  //     // overwrite the original pose(0) with new_start, since Pose(0) is
  //     // fixed during optimization!
  //     DeletePoses(1, nearest_idx);  // delete first states such that the
  //                                   // closest state is the new first one
  //     DeleteTimeDiffs(1,
  //                     nearest_idx);  // delete corresponding time differences
  //   }

  //   // update start
  //   //Pose(0) = *new_start;
  // }

  // if (new_goal && SizePoses() > 0) {
  //   BackPose() = *new_goal;
  // }
}

bool TimedElasticBand::IsTrajectoryInsideRegion(double radius,
                                                double max_dist_behind_robot,
                                                int skip_poses) {
  if (SizePoses() <= 0) return true;

  double radius_sq = radius * radius;
  double max_dist_behind_robot_sq =
      max_dist_behind_robot * max_dist_behind_robot;
  Eigen::Vector2d robot_orient = Pose(0).OrientationUnitVec();

  for (int i = 1; i < SizePoses(); i = i + skip_poses + 1) {
    Eigen::Vector2d dist_vec = Pose(i).position() - Pose(0).position();
    double dist_sq = dist_vec.squaredNorm();

    if (dist_sq > radius_sq) {
      AERROR << "outside robot";
      return false;
    }

    // check behind the robot with a different distance, if specified (or
    // >=0)
    if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 &&
        dist_sq > max_dist_behind_robot_sq) {
      AERROR << "outside robot behind";
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace century
