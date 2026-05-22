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

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/normal_util.h"
#include "modules/planning/common/util/math_util.h"
#include "modules/planning/common/util/util.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star_bidirectional.h"

namespace century {
namespace planning {
struct PointWithCost {
  common::math::Vec2d adc_point = {0.0, 0.0};
  double adc_heading = 0.0;
  double cost = 0.0;
  std::string DebugString() const {
    return absl::StrCat("adc_point:", adc_point.DebugString(),
                        ", adc_heading:", adc_heading, ", cost:", cost);
  }
};

struct Vec2dHash {
  std::size_t operator()(const Vec2d &point) const {
    std::size_t h1 = std::hash<double>{}(point.x());
    std::size_t h2 = std::hash<double>{}(point.y());
    return h1 ^ (h2 << 1);
  }
};

struct Vec2dEqual {
  bool operator()(const Vec2d &a, const Vec2d &b) const {
    const double tolerance = 1e-6;
    return std::abs(a.x() - b.x()) < tolerance &&
           std::abs(a.y() - b.y()) < tolerance;
  }
};

enum ParkingSpacePosition {
  MIDDLE = 0,        // middle of lane
  LEFT = 1,          // left of lane
  RIGHT = 2,         // right of lane
};

class OpenSpaceRoiDecider : public Decider {
 public:
  OpenSpaceRoiDecider(const TaskConfig &config,
                      const std::shared_ptr<DependencyInjector> &injector);

  /*
   * @brief process function
   * @param frame  data frame
   */
  century::common::Status Process(Frame *frame) override;

 private:
  static bool SelectTargetDeadEndJunction(
      std::vector<hdmap::JunctionInfoConstPtr> *junctions,
      const century::common::PointENU &dead_end_point,
      hdmap::JunctionInfoConstPtr *target_junction);

  static bool SelectJunction(
      std::vector<hdmap::JunctionInfoConstPtr> *junctions,
      const century::common::PointENU &point,
      hdmap::JunctionInfoConstPtr *target_junction);

  century::common::Status ProcessParkingScenario(
      std::array<Vec2d, 4>& spot_vertices, hdmap::Path& nearby_path,
      std::vector<std::vector<Vec2d>>& roi_boundary, std::string& ms);

  century::common::Status ProcessParkAndGoScenario(
      std::array<Vec2d, 4>& spot_vertices, hdmap::Path& nearby_path,
      std::vector<std::vector<Vec2d>>& roi_boundary, std::string& msg);

  century::common::Status ProcessRescueScenario(
      const hdmap::Path& nearby_path,
      std::vector<std::vector<Vec2d>>& roi_boundary, std::string& msg);

  bool GetJunctionBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                           std::vector<std::vector<common::math::Vec2d>>
                               *const roi_deadend_boundary);

  bool GetDeadEndSpot(Frame *const frame, hdmap::JunctionInfoConstPtr *junction,
                      std::vector<common::math::Vec2d> *dead_end_vertices);
  void SetDeadEndOrigin(
      Frame *const frame,
      const std::vector<common::math::Vec2d> &dead_end_vertices);
  void SetDeadEndPose(
      Frame *const frame,
      const std::vector<common::math::Vec2d> &dead_end_vertices);
  bool GetDeadEndBoundary(
      Frame *const frame,
      const std::vector<common::math::Vec2d> &dead_end_vertices,
      const hdmap::Path &nearby_path,
      std::vector<std::vector<common::math::Vec2d>>
          *const roi_deadend_boundary);

  void GetInLaneEndPoint(hdmap::LaneInfoConstPtr laneinfo,
                         common::PointENU *left_end_point,
                         common::PointENU *right_end_point);
  void GetOutLaneStartPoint(hdmap::LaneInfoConstPtr laneinfo,
                            common::PointENU *left_start_point,
                            common::PointENU *right_start_point);
  void GetInLaneBoundaryPoints(
      hdmap::LaneInfoConstPtr lane_info, const hdmap::Path &nearby_path,
      std::vector<common::PointENU> *const in_left_boundary_points,
      std::vector<common::PointENU> *const in_right_boundary_points);
  void GetOutLaneBoundaryPoints(
      hdmap::LaneInfoConstPtr lane_info, const hdmap::Path &nearby_path,
      std::vector<common::PointENU> *const out_left_boundary_points,
      std::vector<common::PointENU> *const out_right_boundary_points);

  /*
   * @brief generate the path by vehicle location and return the target parking
   * spot and corner points on that path
   * @param frame  data frame
   * @param vertices   parking space corner points
   * @param nearby_path   parking space nearest path
   */
  bool GetParkingSpot(Frame *const frame,
                      std::array<common::math::Vec2d, 4> *vertices,
                      hdmap::Path *nearby_path);

  bool GetParkingSpotTest(Frame *const frame,
                          std::array<common::math::Vec2d, 4> *vertices,
                          hdmap::Path *nearby_path);

  // @brief get path from reference line and return vertices of pullover spot
  bool GetPullOverSpot(Frame *const frame,
                       std::array<common::math::Vec2d, 4> *vertices,
                       hdmap::Path *nearby_path);

  /*
   * @brief Set an origin to normalize the problem for later computation
   *        left_down corner point as origin point
   *        parallel lane direction as origin heading
   * @param frame  data frame
   * @param vertices  target parking space corner points
   */
  bool SetOrigin(Frame *const frame,
                 const std::array<common::math::Vec2d, 4> &vertices);

  /*
   * @brief set origin pose
   * @param frame   data frame
   * @param nearby_path   reference line
   */
  bool SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path);

  /*
   * @brief set end pose in origin axis
   * @param frame   data frame
   * @param vertices   target parking space corner points
   * */
  bool SetParkingSpotEndPose(
      Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices);

  void SetPullOverSpotEndPose(Frame *const frame);

  /*
   * @brief generate end pose
   * @param frame   data frame
   */
  bool SetParkAndGoEndPose(Frame *const frame);
  void SetOriginFromAdcPose(Frame *const frame, const hdmap::Path &nearby_path);
  void SetStopLineParkingEndPose(Frame *const frame);
  void SetRescueOriginPose(Frame *const frame, const hdmap::Path &nearby_path);
  void SetRescueEndPose(Frame *const frame);

  /* 
   * @brief Get road boundaries of both sides
   * @param nearby_path  nearest path
   * @param center_line_s  parking space center s
   * @param origin_point  origin point
   * @param origin_heading  origin heading
   * @param left_lane_boundary            left road boundary point(origin)
   * @param right_lane_boundary           right road boundary point(origin)
   * @param center_lane_boundary_left     left sample point
   * @param center_lane_boundary_right    right sample point
   * @param center_lane_s_left       left sample s
   * @param center_lane_s_right      right sample s
   * @param left_lane_road_width     left road width
   * @param right_lane_road_width    right road width
   */
  void GetRoadBoundary(
      const hdmap::Path &nearby_path, const double center_line_s,
      const common::math::Vec2d &origin_point, const double origin_heading,
      std::vector<common::math::Vec2d> *left_lane_boundary,
      std::vector<common::math::Vec2d> *right_lane_boundary,
      std::vector<common::math::Vec2d> *center_lane_boundary_left,
      std::vector<common::math::Vec2d> *center_lane_boundary_right,
      std::vector<double> *center_lane_s_left,
      std::vector<double> *center_lane_s_right,
      std::vector<double> *left_lane_road_width,
      std::vector<double> *right_lane_road_width);


  /// @brief get electric fence boundary from map as roi boundary
  /// @param vehicle_state: vehicle position
  /// @param electric_fences_boundary: electric fence polygon points
  bool GetRoiBoundaryFromElectricFence(
      const century::common::VehicleState &vehicle_state,
      std::vector<std::vector<Vec2d>> &electric_fences_boundary);

  // @brief Get the Road Boundary From Map object
  bool GetRoadBoundaryFromMap(
      const hdmap::Path &nearby_path, const double center_line_s,
      const common::math::Vec2d &origin_point, const double origin_heading,
      std::vector<common::math::Vec2d> *left_lane_boundary,
      std::vector<common::math::Vec2d> *right_lane_boundary,
      std::vector<common::math::Vec2d> *center_lane_boundary_left,
      std::vector<common::math::Vec2d> *center_lane_boundary_right,
      std::vector<double> *center_lane_s_left,
      std::vector<double> *center_lane_s_right,
      std::vector<double> *left_lane_road_width,
      std::vector<double> *right_lane_road_width);

  /*
   * @brief Check single-side curb and add key points to the boundary
   * @param nearby_path  nearest path
   * @param check_point_s   sample s
   * @param start_s
   * @param end_s
   * @param is_anchor point   anchor point flag
   * @param is_left_curb      compute left boundary point flag
   * @param center_lane_boundary    sample points list
   * @param curb_lane_boundary       boundary points list
   * @param center_lane_s      sample s list
   * @param road_width      road width list
   */
  void AddBoundaryKeyPoint(
      const hdmap::Path &nearby_path, const double check_point_s,
      const double start_s, const double end_s, const bool is_anchor_point,
      const bool is_left_curb,
      std::vector<common::math::Vec2d> *center_lane_boundary,
      std::vector<common::math::Vec2d> *curb_lane_boundary,
      std::vector<double> *center_lane_s, std::vector<double> *road_width);

  /*
   * @brief "Region of Interest", load map boundary for open space scenario
   * @param frame   data frame
   * @param vertices is an array consisting four points describing the
   *        boundary of spot in box. Four points are in sequence of left_down,
   *        left_up, right_up, right_down
   * ------------------------------------------------------------------
   *
   *                 --> lane_direction
   *
   * ----------------left_down        left_top--------------------------
   *                -                  -
   *                -                  -
   *                -                  -
   *                -                  -
   *                right_down-------right_top
   * @param nearby_path   nearest path
   * @param roi_parking_boundary   drivable area
   */
  bool GetParkingBoundary(Frame *const frame,
                          const std::array<common::math::Vec2d, 4> &vertices,
                          const hdmap::Path &nearby_path,
                          std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary);

  bool BuildParkingBoundaryPoints(Frame* const frame,
                                  const std::array<Vec2d, 4>& vertices,
                                  const hdmap::Path& nearby_path,
                                  const Vec2d& origin_point,
                                  double origin_heading,
                                  std::vector<Vec2d>* boundary_points);

  bool SetRoiBoundaryAndCheckVehicle(Frame* frame,
                                     const std::vector<Vec2d>& boundary_points,
                                     const Vec2d& origin_point,
                                     double origin_heading);

  bool GetPullOverBoundary(Frame *const frame,
                           const std::array<common::math::Vec2d, 4> &vertices,
                           const hdmap::Path &nearby_path,
                           std::vector<std::vector<common::math::Vec2d>>
                               *const roi_parking_boundary);

  bool GetParkAndGoBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                            std::vector<std::vector<common::math::Vec2d>>
                                *const roi_parking_boundary);

  bool GetRescueBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                         std::vector<std::vector<common::math::Vec2d>>
                             *const roi_parking_boundary);

  void GetLaneBoundaryPoints(
      hdmap::LaneInfoConstPtr lane_info, const bool &is_left_bound,
      const hdmap::Path &nearby_path,
      std::vector<common::math::Vec2d> *const boundary_points);

  // @brief Get rescue lane boundaries of both sides
  void GetRescueLaneBoundary(
      const hdmap::Path &nearby_path, const common::math::Vec2d &origin_point,
      const double origin_heading,
      std::vector<common::math::Vec2d> *left_lane_boundary,
      std::vector<common::math::Vec2d> *right_lane_boundary);

  // @brief just for GetRescueLaneBoundaryPoints
  void GetRescueLaneBoundaryPoints(
      hdmap::LaneInfoConstPtr lane_info, const bool &is_left_bound,
      const hdmap::Path &nearby_path,
      std::vector<common::math::Vec2d> *const boundary_points);

  /*
   * @brief check whether the target parking space is on the lane
   * @param lane  the lane
   * @param target_parking_spot   target parking spot information
   * */
  void SearchTargetParkingSpotOnLane(
      const hdmap::LaneInfoConstPtr &lane,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

  // @brief search target parking spot on the path by vehicle location, if
  // no return a nullptr in target_parking_spot
  // find target parking space in nearby_path
  void SearchTargetParkingSpotOnPath(
      const hdmap::Path &nearby_path,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

  /*
   * @brief check distance to parking space is less than threshold
   * @param frame   data frame
   * @param nearby_path  nearest path
   * @param target_parking_spot   target parking space
   * @param vertices  target parking space corner points
   *                  (left_down, left_top, right_top, right_down)
   */
  bool CheckDistanceToParkingSpot(
      Frame *const frame, const hdmap::Path &nearby_path,
      const hdmap::ParkingSpaceInfoConstPtr &target_parking_spot,
      std::array<Vec2d, 4> *vertices);

  /*
   * @brief Helper function for fuse line segments into convex vertices set
   *        delete duplicate points
   * @param line_segments_vec   polygon
   */
  bool FuseLineSegments(
      std::vector<std::vector<common::math::Vec2d>> *line_segments_vec);

  // @brief main process to compute and load info needed by open space planner
  bool FormulateBoundaryConstraints(
      const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
      Frame *const frame);

  // @brief Represent the obstacles in vertices and load it into
  // obstacles_vertices_vec_ in clock wise order. Take different approach
  // towards warm start and distance approach
  bool LoadObstacleInVertices(
      const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
      Frame *const frame);

  /*
   * @brief filter obstacle
   * @param frame    data frame
   * @param obstacle   obstacle
   * */
  bool FilterOutObstacle(const Frame &frame, const Obstacle &obstacle);

  double GetObstacleDist(const common::math::Vec2d &point,
                         const double &ref_heading);
  // @brief Transform the vertice presentation of the obstacles into linear
  // inequality as Ax>b
  bool LoadObstacleInHyperPlanes(Frame *const frame);

  // @brief Helper function for LoadObstacleInHyperPlanes()
  bool GetHyperPlanes(const size_t &obstacles_num,
                      const Eigen::MatrixXi &obstacles_edges_num,
                      const std::vector<std::vector<common::math::Vec2d>>
                          &obstacles_vertices_vec,
                      Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

  /**
   * @brief check if vehicle is parked in a parking lot
   * @param (adc_init_x, adc_int_y, adc_init_heading)   vehicle position
   * @param nearby_path    reference line
   * @return true adc parked in a parking lot
   * @return false adc parked at a pull-over spot
   */
  bool IsInParkingLot(const double adc_init_x, const double adc_init_y,
                      const double adc_init_heading,
                      const hdmap::Path &nearby_path,
                      std::array<common::math::Vec2d, 4> *parking_lot_vertices);

  /**
   * @brief Get the Park Spot From Map object
   * @param parking_lot   parking space information
   * @param nearby_path   reference line
   * @param vertices      four corner points
   */
  bool GetParkSpotFromMap(hdmap::ParkingSpaceInfoConstPtr parking_lot,
                          const hdmap::Path &nearby_path,
                          std::array<common::math::Vec2d, 4> *vertices);

  // @brief Check if adc blocked with front obs
  bool CheckADCIsBlockedWithSurroundObstacles(
      const common::math::Vec2d adc_position, const double adc_heading,
      const Frame *frame, const double front_obstacle_buffer,
      const double shift_dist = 0);

  // @brief interpolate boundary
  void InterpolateBoundary(const double &s_interval,
                           const double &heading_interval,
                           std::vector<common::math::Vec2d> *boundary);

// @brief get operation goal point for rescue
  bool GetOperationGoalPoint(const Frame *const frame,
                             std::vector<PointWithCost> &goal_points);

  // @brief remove same point
  void RemoveSamePointsInBoundary(
      std::vector<century::common::math::Vec2d> *boundary);

  // @brief generate goals for rescue
  void GenerateSampleGoals(const common::SLPoint &start_point,
                           const double &long_start_s, const double &long_end_s,
                           const double &long_interval_s, const Frame *frame,
                           const ReferenceLine &reference_line);

  // @brief reuse last frame info: main end pose
  void ReuseLastInfo(Frame *const frame);

  bool IsReverseDriving();

 private:
  // @brief parking_spot_id from routing
  bool first_enter_park_and_go_scenario_ = true;
  double target_offset_ = 0.0;
  std::string target_parking_spot_id_;
  hdmap::ParkingSpaceInfoConstPtr target_parking_spot_ = nullptr;
  ParkingSpacePosition parking_space_position_ = ParkingSpacePosition::LEFT;

  const hdmap::HDMap *hdmap_ = nullptr;

  std::unique_ptr<OpenspaceCommon> openspace_common_ =
      std::make_unique<OpenspaceCommon>();

  century::common::VehicleParam vehicle_params_;

  ThreadSafeIndexedObstacles *obstacles_by_frame_;

  common::VehicleState vehicle_state_;

  // For DeadEnd Scenario
  bool routing_in_flag_ = true;
  common::PointENU dead_end_point_;
  common::PointENU routing_end_point_;
  common::VehicleState temp_state_;
  common::PointENU routing_target_point_;
  // DeadEnd Scenario

  bool is_first_debug_ = true;
  std::vector<PointWithCost> goals_vector_;

  // For Rescue Scenario
  common::math::Vec2d rescue_end_point_;
  double rescue_end_point_heading_ = 0.0;
  bool is_try_to_second_plan_ = false;
  bool is_already_second_plan_ = false;
  std::vector<century::hdmap::Id> lane_ids_;
  std::vector<century::hdmap::Id> neighbor_lane_ids_;
  // Rescue Scenario

  bool is_first_in_debug_ = true;
};

}  // namespace planning
}  // namespace century
