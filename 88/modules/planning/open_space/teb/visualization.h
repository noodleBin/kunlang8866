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

// std
#include <iterator>
#include <string>
#include <vector>

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

// teb stuff
#include "modules/planning/open_space/teb/timed_elastic_band.h"
#include "modules/planning/open_space/teb/utils/robot_footprint_model.h"
#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

class TebOptimalPlanner;  //!< Forward Declaration

/**
 * @class TebVisualization
 * @brief Visualize stuff from the teb_local_planner
 */
class TebVisualization {
 public:
  /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
  TebVisualization();

  /**
   * @brief Constructor that initializes the class and registers topics
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the TebConfig class for parameters
   */
  explicit TebVisualization(const TebConfig& cfg);

  /**
   * @brief Initializes the class and registers topics.
   *
   * Call this function if only the default constructor has been called before.
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the TebConfig class for parameters
   */
  void Initialize(const TebConfig& cfg);

  /** @name Publish to topics */
  //@{

  /**
   * @brief Publish a given global plan to the ros topic \e ../../global_plan
   * @param global_plan Pose array describing the global plan
   */
  void PublishGlobalPlan(const std::vector<PoseStamped>& global_plan) const;

  /**
   * @brief Publish a given local plan to the ros topic \e ../../local_plan
   * @param local_plan Pose array describing the local plan
   */
  void PublishLocalPlan(const std::vector<PoseStamped>& local_plan) const;

  /**
   * @brief Publish Timed_Elastic_Band related stuff (local plan, pose
   * sequence).
   *
   * Given a Timed_Elastic_Band instance, publish the local plan to  \e
   * ../../local_plan and the pose sequence to  \e ../../teb_poses.
   * @param teb const reference to a Timed_Elastic_Band
   */
  void PublishLocalPlanAndPoses(const TimedElasticBand& teb) const;

  /**
   * @brief Publish the robot footprints related to infeasible poses
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseRobotFootprintModel
   */
  void PublishInfeasibleRobotPose(const PoseSE2& current_pose,
                                  const BaseRobotFootprintModel& robot_model);

  /**
   * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
   * @todo Move filling of the marker message to polygon class in order to avoid
   * checking types.
   * @param obstacles Obstacle container
   */
  void PublishObstacles(const ObstContainer& obstacles) const;

  /**
   * @brief Publish via-points to the ros topic \e ../../teb_markers
   * @param via_points via-point container
   */
  void PublishViaPoints(
      const std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>>& via_points,
      const std::string& ns = "ViaPoints") const;

  /**
   * @brief Publish multiple Tebs from a container class (publish as marker
   * message).
   *
   * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
   * @param ns Namespace for the marker objects
   */
  void PublishTebContainer(
      const std::vector<boost::shared_ptr<TebOptimalPlanner>>& teb_planner,
      const std::string& ns = "TebContainer");

  /**
   * @brief Publish a feedback message (multiple trajectory version)
   *
   * The feedback message contains the all planned trajectory candidates (e.g.
   * if planning in distinctive topologies is turned on). Each trajectory is
   * composed of the sequence of poses, the velocity profile and temporal
   * information. The feedback message also contains a list of active obstacles.
   * @param teb_planners container with multiple tebs (resp. their planner
   * instances)
   * @param selected_trajectory_idx Idx of the currently selected trajectory in
   * \c teb_planners
   * @param obstacles Container of obstacles
   */
  void PublishFeedbackMessage(
      const std::vector<boost::shared_ptr<TebOptimalPlanner>>& teb_planners,
      unsigned int selected_trajectory_idx, const ObstContainer& obstacles);

  /**
   * @brief Publish a feedback message (single trajectory overload)
   *
   * The feedback message contains the planned trajectory
   * that is composed of the sequence of poses, the velocity profile and
   * temporal information. The feedback message also contains a list of active
   * obstacles.
   * @param teb_planner the planning instance
   * @param obstacles Container of obstacles
   */
  void PublishFeedbackMessage(const TebOptimalPlanner& teb_planner,
                              const ObstContainer& obstacles);

 protected:
  /**
   * @brief Small helper function that checks if initialize() has been called
   * and prints an error message if not.
   * @return \c true if not initialized, \c false if everything is ok
   */
  bool PrintErrorWhenNotInitialized() const;
  const TebConfig*
      cfg_;  //!< Config class that stores and manages all related parameters

  bool initialized_;  //!< Keeps track about the correct initialization of this
                      //!< class

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the TebVisualization
typedef boost::shared_ptr<TebVisualization> TebVisualizationPtr;
//! Abbrev. for shared instances of the TebVisualization (read-only)
typedef boost::shared_ptr<const TebVisualization> TebVisualizationConstPtr;

}  // namespace planning
}  // namespace century
