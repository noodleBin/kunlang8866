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
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
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
#include "modules/planning/open_space/teb/optimal_planner.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"

namespace century {
namespace planning {
struct BlockShift {
  double l = 0.0;
  double s = 0.0;
};

struct CheckData {
  double stamp;
  bool has_speed;
  bool has_fallback;
};

class TEBPlannerDecider : public Decider {
  struct PointWithCost {
    PointWithCost(const common::math::Vec2d &point, double heading,
                  double point_cost)
        : adc_point(point), adc_heading(heading), cost(point_cost) {}

    common::math::Vec2d adc_point = {0.0, 0.0};
    double adc_heading = 0.0;
    double cost = 0.0;
    std::string DebugString() const {
      return absl::StrCat("adc_point:", adc_point.DebugString(),
                          ", adc_heading:", adc_heading, ", cost:", cost);
    }
  };

 public:
  TEBPlannerDecider(const TaskConfig &config,
                    const std::shared_ptr<DependencyInjector> &injector);

 private:
  century::common::Status Process(Frame *frame) override;

 private:
  // @brief Set an origin to normalize the problem for later computation
  void SetRescueOriginPose(Frame *const frame);
  void OutRoadStartLogic(bool *const prefer_replan);
  void PullOverNearRoutingEndLogic(bool *const prefer_replan);
  void SetRescueEndPose(Frame *const frame);
  bool CalFallBackReplan();
  void NoFindGoalsProcess();
  void SetRescueEndPoseThread(Frame *const frame);
  void SetRescueBackEndPose(Frame *const frame);
  void SetRescueBackEndPoseThread(Frame *const frame);
  void SetCommonOriginPose(Frame *const frame);

  void SetUturnEndPose(Frame *const frame);
  void SetUturnEndPoseThread(Frame *const frame);

  bool CheckFallbackReplan();
  bool CheckPreferReplan(Frame *const frame);
  bool CheckStopLongReplan();
  void CalculateFirstIntoRescue(bool *first_enable_pullover,
                                bool *first_enable_rescue);
  // @brief Get road boundaries of both sides
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

  // @brief Check single-side curb and add key points to the boundary
  void AddBoundaryKeyPoint(
      const hdmap::Path &nearby_path, const double check_point_s,
      const double start_s, const double end_s, const bool is_anchor_point,
      const bool is_left_curb,
      std::vector<common::math::Vec2d> *center_lane_boundary,
      std::vector<common::math::Vec2d> *curb_lane_boundary,
      std::vector<double> *center_lane_s, std::vector<double> *road_width);

  bool GetRescueBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                         std::vector<std::vector<common::math::Vec2d>>
                             *const roi_parking_boundary);

  bool GenerateBoundary(
      Frame *const frame,
      std::vector<std::vector<common::math::Vec2d>> *const roi_boundary);

  bool GetAcdOnLaneHalfWidth(common::SLPoint *const adc_position_sl,
                             double *const current_road_left_line_l,
                             double *const current_road_right_line_l);

  void CalOpenRoadTebRoiLeftAndRightPoint(
      Frame *const frame, const common::SLPoint &adc_position_sl,
      const double &current_road_left_line_l,
      const double &current_road_right_line_l,
      std::vector<common::math::Vec2d> *const left_lane_boundary,
      std::vector<common::math::Vec2d> *const right_lane_boundary);

  void CalXYBoundary(
      const std::vector<common::math::Vec2d> &left_lane_boundary,
      const std::vector<common::math::Vec2d> &right_lane_boundary,
      Frame *const frame);

  bool GenerateDefaultLaneBoundary(Frame *const frame,
                                   std::vector<std::vector<common::math::Vec2d>>
                                       *const roi_parking_boundary);

  bool GenerateNoHdMapBoundary(Frame *const frame,
                               std::vector<std::vector<common::math::Vec2d>>
                                   *const roi_parking_boundary);

  bool JudgeInPlayStreet();

  void CalPlayStreetTebRoiLeftAndRightPoint(
      Frame *const frame, const common::SLPoint &adc_position_sl,
      const double &current_road_left_line_l,
      const double &current_road_right_line_l,
      std::vector<common::math::Vec2d> *const left_lane_boundary,
      std::vector<common::math::Vec2d> *const right_lane_boundary);

  void RoiOffLaneRoutingLogic(double *const roi_left_line_l,
                              double *const roi_right_line_l,
                              const double &current_road_left_line_l,
                              const double &current_road_right_line_l);

  void UseConfigParamRoi(double *const roi_left_line_l,
                         double *const roi_right_line_l);
  bool GenerateNoHdMapFrenetBoundary(
      Frame *const frame, std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary);
  void ExtendCurrentLane(double *const roi_left_line_l,
                         double *const roi_right_line_l,
                         const double &extend_dis);

  bool GetTrafficLightCurrentLaneExtendBoundary(
      Frame *const frame, std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary);

  bool GetTrafficLightJunctionBoundary(
      Frame *const frame, std::vector<std::vector<common::math::Vec2d>>
                              *const roi_deadend_boundary);

  bool GetDriveAreaJunctionBoundary(
      Frame *const frame, std::vector<std::vector<common::math::Vec2d>>
                              *const roi_deadend_boundary);
  void ReducePointSpacing(
      const std::vector<common::math::Vec2d> &point_boundary_temp,
      const double &interpolate_step,
      std::vector<common::math::Vec2d> *const point_boundary_interpolate);
  bool CalDriveAreaSLBoundary(
      Frame *const frame,
      const std::vector<common::SLPoint> &sl_point_boundary_interpolate,
      std::vector<std::vector<common::math::Vec2d>>
          *const roi_parking_boundary);
  void CalDriveAreaROIL(
      const std::vector<common::SLPoint> &sl_point_left_boundary_filter,
      const std::vector<common::SLPoint> &sl_point_right_boundary_filter,
      const double &current_s, const double &roi_left_line_l,
      const double &roi_right_line_l, double *const left_roi_l_,
      double *const right_roi_l_);

  bool GetPlayStreetJunctionBoundary(
      Frame *const frame, std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary);

  void CalPlayStreetJunctionTebRoiLeftAndRightPoint(
      Frame *const frame, const bool &has_left_neighbor_lane,
      const common::SLPoint &adc_position_sl,
      const double &current_road_left_line_l,
      const double &current_road_right_line_l,
      std::vector<common::math::Vec2d> *const left_lane_boundary,
      std::vector<common::math::Vec2d> *const right_lane_boundary);

  bool GetLeftNeighborLane(Frame *const frame);

  bool CalLeftWidthForNeighborLane(Frame *const frame,
                                   double *const left_neighbor_lane_width);

  bool GenerateXYbounds(Frame *const frame,
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
  void CalPreLaneBoundary(
      hdmap::LaneInfoConstPtr lane_info, const bool &is_left_bound,
      const double *start_s, double *pre_start_s, double *pre_end_s,
      hdmap::Id *pre_lane_id, hdmap::LaneInfoConstPtr pre_lane_info,
      std::vector<common::math::Vec2d> *const boundary_points);

  void CheckAdcIsBlocked(Frame *const frame);

  // @brief Helper function for fuse line segments into convex vertices set
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

  void DontUsePolygonPlan(century::planning::Frame *const frame,
                          const Obstacle &obstacle,
                          size_t *perception_obstacles_num,
                          std::vector<std::vector<century::common::math::Vec2d>>
                              *obstacles_vertices_vec,
                          std::vector<std::vector<century::common::math::Vec2d>>
                              *pure_obstacles_vertices_vec);

  void UsePolygonPlan(century::planning::Frame *const frame,
                      const Obstacle &obstacle,
                      size_t *perception_obstacles_num,
                      std::vector<std::vector<century::common::math::Vec2d>>
                          *obstacles_vertices_vec,
                      std::vector<std::vector<century::common::math::Vec2d>>
                          *pure_obstacles_vertices_vec,
                      size_t *point_num,
                      std::vector<size_t> *const obstacle_points_num);

  void EnableUseCostMap(Frame *const frame, size_t *point_num,
                        size_t *perception_obstacles_num,
                        std::vector<size_t> *obstacle_points_num,
                        std::vector<std::vector<century::common::math::Vec2d>>
                            *obstacles_vertices_vec,
                        std::vector<std::vector<century::common::math::Vec2d>>
                            *pure_obstacles_vertices_vec);

  bool FilterOutObstacle(const Frame &frame, const Obstacle &obstacle);

  bool FilterOutCostMap(const Frame &frame,
                        const century::common::math::Polygon2d &polygon);

  bool FilterOutObstacleWithoutHdMap(const Frame &frame,
                                     const Obstacle &obstacle);
  double CalcBackDistance(const double lat_offset, const double lon_offset,
                          const double min_stop_distance);

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
  bool CheckADCIsBlockedTypeWithSurroundObstacles(
      const common::math::Vec2d adc_position, const double adc_heading,
      const Frame *frame, const double front_obstacle_buffer,
      const double shift_dist);
  // @brief Check if adc blocked with front obs
  bool CheckADCIsBlockedWithSurroundObstacles(
      const common::math::Vec2d adc_position, const double adc_heading,
      const Frame *frame, const double front_obstacle_buffer,
      const double lat_buffer, const double shift_dist = 0.0,
      const bool preset_buffer = false);

  // @brief interpolate boundary
  void InterpolateBoundary(const double &s_interval,
                           const double &heading_interval,
                           std::vector<common::math::Vec2d> *boundary);

  bool PreEndPoseReplayLogic(Frame *const frame,
                             const century::planning::Frame *previous_frame,
                             const bool &is_pullover_ready,
                             const bool &is_in_city_road);

  bool TryAddGoalSample(double s, double l, double prefer_l,
                        double lateral_cost_ratio,
                        const common::SLPoint& start_point,
                        const common::SLPoint& adc_position_sl,
                        const Vec2d& end_pose_x_y,
                        bool pre_end_pose_check_valid, TEBTarStatus tar_status,
                        bool is_rescue, bool is_pullover_ready,
                        const Frame* frame,
                        const ReferenceLine& reference_line);

  // @brief generate goals for rescue
  void GenerateSampleGoals(const common::SLPoint &start_point,
                           const double &long_start_s, const double &long_end_s,
                           const double &long_interval_s, const Frame *frame,
                           const ReferenceLine &reference_line);

  void GenerateSampleGoals(const common::SLPoint &start_point,
                           const Frame *frame,
                           const ReferenceLine &reference_line);

  // @brief generate back goal for rescue
  bool CalLatSampleRange(const ReferenceLine &reference_line,
                         const double &start_s, double *const left,
                         double *const right);
  void GenerateBackSampleGoals(const common::SLPoint &start_point,
                               const double &long_end_s, const Frame *frame,
                               const ReferenceLine &reference_line);

  bool CheckGoalIsValid(const common::math::Vec2d &point,
                        const double &ref_heading);

  void SetGoalToEndPose(Frame *const frame);
  BlockShift CalcMinLateralDistance(const common::math::Vec2d &adc_position,
                                    const double &adc_heading,
                                    const common::math::Polygon2d &obj);

  void SetStopLineParkingEndPose(Frame *const frame);

  bool GetJunctionBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                           std::vector<std::vector<common::math::Vec2d>>
                               *const roi_deadend_boundary);
  bool GetDeadEndSpot(Frame *const frame, hdmap::JunctionInfoConstPtr *junction,
                      std::vector<common::math::Vec2d> *dead_end_vertices);

  static bool SelectJunction(
      std::vector<hdmap::JunctionInfoConstPtr> *junctions,
      const century::common::PointENU &point,
      hdmap::JunctionInfoConstPtr *target_junction);

  static bool SelectDriveAreaJunction(
      Frame *const frame, std::vector<hdmap::JunctionInfoConstPtr> *junctions,
      const century::common::PointENU &point,
      hdmap::JunctionInfoConstPtr *target_junction);

  void SaveHistoryStaticObs(const Frame *frame,
                            ThreadSafeIndexedObstacles *obstacles_by_frame_,
                            std::deque<std::vector<century::planning::Obstacle>>
                                *const history_obs_lists_ptr_);

  bool CheckBlockedWithCostmap(const common::math::Polygon2d &adc_polygon,
                               const common::math::Vec2d adc_position,
                               const double adc_heading, bool preset_buffer,
                               bool block_shift_calc_inner);

  void StableStaticObsWithHistory(
      const Frame *frame,
      std::deque<std::vector<century::planning::Obstacle>>
          *const history_obs_lists_ptr_,
      ThreadSafeIndexedObstacles *obstacles_by_frame_);

  void DebugStaticObsWithHistory(
      const Frame *frame, ThreadSafeIndexedObstacles *obstacles_by_frame_);

  bool DetermainADCIsBlockedWithFrontObstacles(
      const common::math::Polygon2d &obj,
      const common::math::Vec2d adc_position, const double adc_heading,
      const double lon_buffer, const double lat_buffer);

  void CalDriveAreaSegmentBoundary(
      Frame *const frame,
      const std::vector<common::SLPoint> &sl_point_left_boundary_filter,
      const std::vector<common::SLPoint> &sl_point_right_boundary_filter,
      const common::SLPoint &adc_position_sl, const double &roi_left_line_l,
      const double &roi_right_line_l,
      std::vector<std::vector<common::math::Vec2d>>
          *const roi_parking_boundary);

  void DeterMainJuncionInPlayStreet();

  void CalculateFrameObstacles(
      Frame *const frame, size_t *perception_obstacles_num, size_t *point_num,
      std::vector<std::vector<century::common::math::Vec2d>>
          *obstacles_vertices_vec,
      std::vector<std::vector<century::common::math::Vec2d>>
          *pure_obstacles_vertices_vec,
      std::vector<size_t> *const obstacle_points_num);
  bool CheckVehicleStopLongTime(Frame *frame);
  bool CheckVehicleStopLongTime(Frame *frame, const double &stop_speed,
                                const double &check_stop_time_windows,
                                const double &check_stop_time_min,
                                const double &stop_long_time_valid_thr);
  bool StopLongTimeReportAndExit(Frame *frame);

 private:
  // @brief parking_spot_id from routing
  std::string target_parking_spot_id_;

  const hdmap::HDMap *hdmap_ = nullptr;

  century::common::VehicleParam vehicle_params_;

  ThreadSafeIndexedObstacles *obstacles_by_frame_;

  common::VehicleState vehicle_state_;
  century::planning::RescueStatus *rescue_status_;

  double adc_path_heading_ = 0.0;
  // @brief open_space end configuration in order of x, y, heading and speed.
  // Speed is set to be always zero now for parking
  std::vector<double> last_open_space_end_pose_;

  std::vector<PointWithCost> goals_vector_;

  // ------------------------------------------------------------------
  // replan related code
  // Only when this is true and the target point is not empty, can we truly
  // prepare for re planning. Here is the ready sign, but there is still a need
  // to be prepared to select points
  bool ready_to_replan_ = false;
  // count for fallback times
  int fallback_replan_count_ = 0;
  // count for stop long time
  int stop_long_count_ = 0;
  // ------------------------------------------------------------------

  common::math::Vec2d rescue_end_point_;
  double rescue_end_point_heading_ = 0.0;
  std::vector<century::hdmap::Id> lane_ids_;
  // ------------Just For Rescue Scenario-------------------------
  double adc_ref_heading_ = 0;
  std::vector<double> ROI_xy_boundary_;
  std::vector<std::vector<common::math::Vec2d>> roi_boundary_;

  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;
  std::vector<std::string> obstacle_id_;
  bool last_select_back_pose_ = false;
  bool is_pullover_ready_ = false;
  double block_shift_l_ = 0.0;
  double block_shift_s_ = 0.0;

  // Teb Stable Static Obs
  std::vector<century::planning::Obstacle> current_obs_list_;
  std::deque<std::vector<century::planning::Obstacle>> history_obs_lists_;
  std::deque<std::vector<century::planning::Obstacle>> *history_obs_lists_ptr_ =
      &history_obs_lists_;
  std::vector<common::math::Polygon2d> perception_polygon_list_;

  bool block_shift_calc_ = false;
  bool junction_in_play_street_ = false;
  bool is_in_junction_ = false;
  bool start_collision_ = false;
  bool preset_long_buffer_ = 0.0;
  bool preset_lat_buffer_ = 0.0;
  bool is_in_city_road_ = false;
  bool fail_to_select_goal_ = false;

  double left_roi_l_ = 0.0;
  double right_roi_l_ = 0.0;
  int start_roi_s_ = 0;
  int end_roi_s_ = 0;
  std::deque<CheckData> vehicle_speed_deque_;
  std::deque<CheckData> fallback_deque_;
};

}  // namespace planning
}  // namespace century
