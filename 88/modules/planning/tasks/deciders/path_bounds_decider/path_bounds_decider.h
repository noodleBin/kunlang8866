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

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"

#include "modules/planning/math/curve1d/cubic_polynomial_curve1d.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace century {
namespace planning {

constexpr double kPathBoundsDeciderResolution = 0.5;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kDefaultRoadWidth = 20.0;
// TODO(all): Update extra tail point base on vehicle speed.
constexpr int kNumExtraTailBoundPoint = 20;
constexpr double kPulloverLonSearchCoeff = 1.5;
constexpr double kPulloverLatSearchCoeff = 1.25;

class PathBoundsDecider : public Decider {
 public:
  enum class LaneBorrowInfo {
    LEFT_BORROW,
    NO_BORROW,
    RIGHT_BORROW,
  };
  PathBoundsDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);

 private:
  enum BoundsType {
    DEFAULT_PATH_BOUNDS = 0,
    FALL_BACK_PATH_BOUNDS,
    PULL_OVER_PATH_BOUNDS,
    LANE_CHANGE_PATH_BOUNDS,
    REGULAR_PATH_BOUNDS
  };
  enum LaneChangeDirection { NO_CHANGE = 0, LEFT_CHANGE, RIGHT_CHANGE };
  enum DirectionDecision { NONE = 0, LEFT, RIGHT };
  struct PathBoundPoint {
    explicit PathBoundPoint(const double in_s, const double in_l_min,
                            const double in_l_max)
        : s(in_s), l_min(in_l_min), l_max(in_l_max) {}

    double s;
    double l_min;
    double l_max;
  };
  // PathBound contains a vector of PathBoundPoints.
  using PathBound = std::vector<PathBoundPoint>;

  /** @brief Obstacle edge include:
   *   1. the limit edge: cover the change of obstacle's size caused by
   * perception
   *   2. the final edge: bigger than limit edge with buffer
   *   If obstacle not out of the limit edge, then not update the final edge,
   *   otherwise update the limit edge base on obstacle size and update the
   *   final edge base on limit edge:
   *
   *    _________final edge________
   *   |                           |
   *   |.........limit edge........|
   *   |                           |
   *   |         ********          |
   *   |        *        *         |
   *   |       * obstacle *        |
   *   |       *          *        |
   *   |        **********         |
   *   |                           |
   *   |.........limit edge........|
   *   |                           |
   *   |_________final edge________|
   *
   */
  struct ObstacleEdge {
    explicit ObstacleEdge(const bool in_is_start_s, const double in_s,
                          const double in_l_min, const double in_l_max,
                          const std::string& in_obstacle_id)
        : is_start_s(in_is_start_s),
          s(in_s),
          l_min(in_l_min),
          l_max(in_l_max),
          obstacle_id(in_obstacle_id) {}

    bool is_start_s;
    double s;      // it's the final edge of start_s
    double l_min;  // it's the final edge of start_l
    double l_max;  // it's the final edge of end_l
    std::string obstacle_id;
    double l_min_safe_limit;  // it's the limit edge of start_l
    double l_max_safe_limit;  // it's the limit edge of end_l
    double timestamp;
    double lon_start_buffer;  // the start_s buffer of the final edge
    double origin_start_s;
    double origin_start_l;
    double origin_end_l;
    DirectionDecision direction_decision;
  };

  struct PullOverConfiguration {
    double x;
    double y;
    double theta;
    int index;
  };

  struct LaneBorrowBound {
    bool is_pass_from_left;
    double l;
  };

  /** @brief Every time when Process function is called, it will:
   *   1. Initialize.
   *   2. Generate Fallback Path Bound.
   *   3. Generate Regular Path Bound(s).
   */
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called every frame when executing PathBoundsDecider.

  /** @brief The initialization function.
   */
  void InitPathBoundsDecider(const Frame& frame,
                             const ReferenceLineInfo& reference_line_info);
  double GetRoutingEndS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info);
  common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
      const common::TrajectoryPoint& traj_point);

  void ComputeRefLineEndState(ReferenceLineInfo* reference_line_info);
  void ComputeWideRoadKeepRightEndState();

  void GetCandidateRegularPathBounds(
      ReferenceLineInfo* reference_line_info,
      std::vector<PathBoundary>* const path_bounds);

  /** @brief The regular path boundary generation considers the ADC itself
   *   and other static environments:
   *   - ADC's position (lane-changing considerations)
   *   - lane info
   *   - static obstacles
   *   The philosophy is: static environment must be and can only be taken
   *   care of by the path planning.
   * @param reference_line_info
   * @param lane_borrow_info: which lane to borrow.
   * @param The generated regular path_boundary, if there is one.
   * @param The blocking obstacle's id. If none, then it's not modified.
   * @return common::Status
   */
  common::Status GenerateRegularPathBound(
      const ReferenceLineInfo& reference_line_info,
      const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
      std::string* const blocking_obstacle_id,
      std::string* const borrow_lane_type);

  /** @brief The fallback path only considers:
   *   - ADC's position (so that boundary must contain ADC's position)
   *   - lane info
   *   It is supposed to be the last resort in case regular path generation
   *   fails so that speed decider can at least have some path and won't
   *   fail drastically.
   *   Therefore, it be reliable so that optimizer will not likely to
   *   fail with this boundary, and therefore doesn't consider any static
   *   obstacle. When the fallback path is used, stopping before static
   *   obstacles should be taken care of by the speed decider. Also, it
   *   doesn't consider any lane-borrowing.
   * @param reference_line_info
   * @param The generated fallback path_boundary, if there is one.
   * @return common::Status
   */
  common::Status GenerateFallbackPathBound(
      const ReferenceLineInfo& reference_line_info,
      std::vector<PathBoundary>* const path_bounds);

  common::Status GenerateLaneChangePathBound(
      ReferenceLineInfo* reference_line_info,
      std::vector<PathBoundary>* const path_bounds);

  common::Status GeneratePullOverPathBound(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      std::vector<PathBoundary>* const path_bounds);

  int IsPointWithinPathBound(const ReferenceLineInfo& reference_line_info,
                             const double x, const double y,
                             const PathBound& path_bound);

  bool FindDestinationPullOverS(const Frame& frame,
                                const ReferenceLineInfo& reference_line_info,
                                const PathBound& path_bound,
                                double* pull_over_s);
  bool FindEmergencyPullOverS(const ReferenceLineInfo& reference_line_info,
                              double* pull_over_s);

  bool GetPullOverPosition(const Frame& frame,
                           const ReferenceLineInfo& reference_line_info,
                           const PathBound& path_bound, int* const curr_idx);
  bool SearchPullOverPosition(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const PathBound& path_bound,
      PullOverConfiguration* const pull_over_configuration);

  bool SearchFeasibleLocationForPullOver(
      const ReferenceLineInfo& reference_line_info, bool search_backward,
      int first_point_idx, const PathBound& path_bound,
      PullOverConfiguration* const pull_over_configuration);

  bool GetValidPallOverLocationIndex(
      const ReferenceLineInfo& reference_line_info, const PathBound& path_bound,
      double pull_over_space_length, double pull_over_space_width,
      bool search_backward, int idx, bool* const is_feasible_window,
      int* const curr_idx);

  void SetFeasiblePullOverConfiguration(
      const ReferenceLineInfo& reference_line_info, const PathBound& path_bound,
      double pull_over_space_width, int pull_over_idx,
      PullOverConfiguration* const pull_over_configuration);

  void TrimCandidatePullOverPathBound(
      PathBound* const pull_over_path_bound, const int curr_idx,
      std::vector<PathBoundary>* const path_bounds);

  void RecordPullOverDebugInfo(ReferenceLineInfo* reference_line_info);

  /** @brief Remove redundant path bounds in the following manner:
   *   - if "left" is contained by "right", remove "left"; vice versa.
   */
  void RemoveRedundantPathBoundaries(
      std::vector<PathBoundary>* const candidate_path_boundaries);

  bool IsContained(const std::vector<std::pair<double, double>>& lhs,
                   const std::vector<std::pair<double, double>>& rhs);

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called when generating path bounds.

  /** @brief Initializes an empty path boundary.
   */
  bool InitPathBoundary(const ReferenceLineInfo& reference_line_info,
                        const LaneBorrowInfo& lane_borrow_info,
                        PathBound* const path_bound, bool isFallBack = false);

  /** @brief Refine the boundary based on the road-info.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   */
  bool GetBoundaryFromRoads(const ReferenceLineInfo& reference_line_info,
                            PathBound* const path_bound);

  /** @brief Refine the boundary based on the ADC position and velocity.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   */
  bool GetBoundaryFromADC(const ReferenceLineInfo& reference_line_info,
                          double ADC_extra_buffer, PathBound* const path_bound);

  /** @brief Refine the boundary based on lane-info and ADC's location.
   *   It will comply to the lane-boundary. However, if the ADC itself
   *   is out of the given lane(s), it will adjust the boundary
   *   accordingly to include ADC's current position.
   */
  bool GetBoundaryFromLanesAndADC(const ReferenceLineInfo& reference_line_info,
                                  const LaneBorrowInfo& lane_borrow_info,
                                  double ADC_buffer,
                                  PathBound* const path_bound,
                                  std::string* const borrow_lane_type,
                                  bool is_fallback = false,
                                  bool is_lanechange = false);
  bool GetValidLaneChangePathBound(
      CubicPolynomialCurve1d* const left_path_res,
      CubicPolynomialCurve1d* const right_path_res,
      PathBoundPoint* const lane_change_start_point,
      PathBoundPoint* const lane_change_end_point);

  void GetCurrentLaneWidth(
      const ReferenceLineInfo& reference_line_info, const double curr_s,
      double* const curr_lane_left_width, double* const curr_lane_right_width,
      double* const past_lane_left_width, double* const past_lane_right_width,
      bool* const is_left_lane_boundary, bool* const is_right_lane_boundary);

  void GetNeighborLaneWidth(const ReferenceLineInfo& reference_line_info,
                            const LaneBorrowInfo& use_lane_borrow_info,
                            const bool in_near_junction_lane_borrow_scenario,
                            bool is_lanechange,
                            const double curr_lane_left_width,
                            const double curr_lane_right_width,
                            const double curr_s,
                            double* const solid_lane_begin_s,
                            double* const solid_lane_free_check_length,
                            double* const curr_neighbor_lane_width,
                            bool* const borrowing_reverse_lane);

  void GetFallbackLaneBorrowInfo(const bool is_fallback,
                                 LaneBorrowInfo* const use_lane_borrow_info);
  bool IsObstacleBlockAdc(const Obstacle* obstacle);
  void GetNeighborLaneWidthBaseOnBorrowInfo(
      const ReferenceLineInfo& reference_line_info, const double curr_s,
      const LaneBorrowInfo& lane_borrow_info, bool is_lanechange,
      const double curr_lane_left_width, const double curr_lane_right_width,
      double* const curr_neighbor_lane_width,
      bool* const borrowing_reverse_lane);

  bool GetLeftNeighborLaneInfo(const ReferenceLineInfo& reference_line_info,
                               const double curr_s,
                               const double curr_lane_left_width,
                               const double curr_lane_right_width,
                               hdmap::Id* const neighbor_lane_id,
                               double* const distance_between_ref,
                               double* const curr_neighbor_lane_width,
                               bool* const borrowing_reverse_lane);

  bool GetRightNeighborLaneInfo(const ReferenceLineInfo& reference_line_info,
                                const double curr_s,
                                const double curr_lane_left_width,
                                const double curr_lane_right_width,
                                hdmap::Id* const neighbor_lane_id,
                                double* const distance_between_ref,
                                double* const curr_neighbor_lane_width,
                                bool* const borrowing_reverse_lane);

  void GetLaneChangeNeighborLaneInfo(
      const ReferenceLineInfo& reference_line_info, const double curr_s,
      const double curr_lane_left_width, const double curr_lane_right_width,
      hdmap::Id* const neighbor_lane_id, double* const distance_between_ref,
      double* const curr_neighbor_lane_width,
      bool* const borrowing_reverse_lane);

  void GetMergeLaneWidth(const ReferenceLineInfo& reference_line_info,
                         const double curr_s, const hdmap::Id neighbor_lane_id,
                         const double distance_between_ref,
                         double* const curr_neighbor_lane_width);

  void GetLateralDecisionLaneBound(
      const LaneBorrowInfo& lane_borrow_info, const double curr_s,
      const double curr_lane_left_width, const double curr_lane_right_width,
      const double curr_neighbor_lane_width,
      const bool vaild_lane_change_path_bound,
      const CubicPolynomialCurve1d& left_path_res,
      const CubicPolynomialCurve1d& right_path_res,
      const PathBoundPoint& lane_change_start_point,
      const PathBoundPoint& lane_change_end_point,
      double* const curr_left_bound_lane, double* const curr_right_bound_lane);

  void GetLaneBoundIncludeADC(const ReferenceLineInfo& reference_line_info,
                              const double curr_s, const double ADC_buffer,
                              const bool is_fallback, const bool is_lanechange,
                              const double lane_change_buffer,
                              const double curr_left_bound_lane,
                              const double curr_right_bound_lane,
                              double* const curr_left_bound,
                              double* const curr_right_bound);

  bool IsNeedKeepRight(const ReferenceLineInfo& reference_line_info);
  bool IsNeedKeepRightForMixedFlow(
      const ReferenceLineInfo& reference_line_info);
  bool NeedConsiderReverseObstacle(const ReferenceLineInfo& reference_line_info,
                                   const Obstacle* const obstacle);

  double GetEndStateLForKeepRight(const ReferenceLineInfo& reference_line_info);
  double GetEndStateLForLaneborrow(
      const ReferenceLineInfo& reference_line_info);
  hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;

  /** @brief Update left boundary by lane_left_width
   *   This is for normal pull-over, which uses lane boundary as left boundary
   *   and road_boundary for right boundary
   */
  void UpdatePullOverBoundaryByLaneBoundary(
      const ReferenceLineInfo& reference_line_info,
      PathBound* const path_bound);

  void ConvertBoundarySAxisFromLaneCenterToRefLine(
      const ReferenceLineInfo& reference_line_info,
      PathBound* const path_bound);

  double GetLaneChangeAdcBuffer(const ReferenceLineInfo& reference_line_info);

  void GetBoundaryFromLaneChangeForbiddenZone(
      const ReferenceLineInfo& reference_line_info,
      PathBound* const path_bound);

  bool NeedStopChangingLane(const ReferenceLineInfo& reference_line_info);

  /** @brief Refine the boundary based on static obstacles. It will make sure
   *   the boundary doesn't contain any static obstacle so that the path
   *   generated by optimizer won't collide with any static obstacle.
   */
  bool GetBoundaryFromStaticObstacles(const PathDecision& path_decision,
                                      const LaneBorrowInfo& lane_borrow_info,
                                      PathBound* const path_boundaries,
                                      std::string* const blocking_obstacle_id);

  void UpdatePathBoundBaseOnObstacleEdge(
      const std::vector<ObstacleEdge>& obstacles_edges,
      PathBound* const path_boundaries, int* const path_blocked_idx,
      std::string* const blocking_obstacle_id, const LaneBorrowInfo& lane_borrow_info);

  void SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles,
      const LaneBorrowInfo& lane_borrow_info,
      std::vector<ObstacleEdge>* const obstacles_edges);

  bool NoNeedPassStaticObstacle(const Obstacle* const obstacle,
                                double* const not_passable_s);

  bool CheckObstacleIsCrossing(const Obstacle& obstacle);

  bool CheckObstacleCanCauseHeadShake(const Obstacle& obstacle);

  void ChooseCurrentPathBoundDirection(
      const ObstacleEdge& obstacle_edge, PathBoundPoint* const path_bound_point,
      std::multiset<double>* const left_bounds,
      std::multiset<double, std::greater<double>>* const right_bounds,
      std::unordered_map<std::string, bool>* const direction,
      double* const center_line, const LaneBorrowInfo& lane_borrow_info);

  bool UpdateAndCheckPathBoundary(double left_bound, double right_bound,
                                  PathBoundPoint* const path_bound_point,
                                  double* const center_line);

  void UpdateNarrowAreaStatus();

  void DecidePassDirections(
      double l_min, double l_max,
      const std::vector<ObstacleEdge>& new_entering_obstacles,
      std::vector<std::vector<bool>>* const pass_directions);

  /////////////////////////////////////////////////////////////////////////////
  // Below are several helper functions:

  /** @brief Get the distance between ADC's center and its edge.
   * @return The distance.
   */
  double GetBufferBetweenADCCenterAndEdge();

  /** @brief Update the path_boundary at "idx"
   *         It also checks if ADC is blocked (lmax < lmin).
   *  @param The current index of the path_bounds
   *  @param The minimum left boundary (l_max)
   *  @param The maximum right boundary (l_min)
   *  @param The path_boundaries (its content at idx will be updated)
   *  @param Is the left bound comes from lane boundary
   *  @param Is the right bound comes from lane boundary
   *  @return If path is good, true; if path is blocked, false.
   */
  bool UpdatePathBoundaryWithBuffer(PathBoundPoint* const path_bound_point,
                                    double left_bound, double right_bound,
                                    bool is_left_lane_bound = false,
                                    bool is_right_lane_bound = false,
                                    LaneBorrowInfo lane_borrow_info =
                                        LaneBorrowInfo::NO_BORROW,
                                    bool is_lanechange = false);

  /** @brief Update the path_boundary at "idx", as well as the new center-line.
   *         It also checks if ADC is blocked (lmax < lmin).
   *  @param The current index of the path_bounds
   *  @param The minimum left boundary (l_max)
   *  @param The maximum right boundary (l_min)
   *  @param The path_boundaries (its content at idx will be updated)
   *  @param The center_line (to be updated)
   *  @return If path is good, true; if path is blocked, false.
   */
  bool UpdatePathBoundaryAndCenterLineWithBuffer(
      PathBoundPoint* const path_bound_point, double left_bound,
      double right_bound, double* const center_line,
      const LaneBorrowInfo& lane_borrow_info);

  /** @brief Update the path_boundary at "idx", It also checks if
             ADC is blocked (lmax < lmin).
   *  @param The current index of the path_bounds
   *  @param The minimum left boundary (l_max)
   *  @param The maximum right boundary (l_min)
   *  @param The path_boundaries (its content at idx will be updated)
   *  @return If path is good, true; if path is blocked, false.
   */
  bool UpdatePathBoundary(size_t idx, double left_bound, double right_bound,
                          PathBound* const path_boundaries);

  /** @brief Trim the path bounds starting at the idx where path is blocked.
   */
  void TrimPathBounds(const int path_blocked_idx,
                      PathBound* const path_boundaries);

  /** @brief Print out the path bounds for debugging purpose.
   */
  void PathBoundsDebugString(const PathBound& path_boundaries);

  bool CheckLaneBoundaryType(const ReferenceLineInfo& reference_line_info,
                             const double check_s,
                             const LaneBorrowInfo& lane_borrow_info);

  void RecordDebugInfo(const PathBound& path_boundaries,
                       const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);

  bool UpdateObstacleSL(Obstacle* obs, const ReferenceLine& ref_line);

  void UpdateTrackObstaclesEdge(
      const DirectionDecision& direction_decision, const Obstacle* obstacle,
      std::vector<ObstacleEdge>* const obstacles_edges, const double lat_buffer,
      const double lon_start_buffer, const double lon_end_buffer);

  void UpdateExistObstacleEdge(const DirectionDecision& direction_decision,
                               const Obstacle* obstacle,
                               std::vector<ObstacleEdge>* const obstacles_edges,
                               const double lat_buffer,
                               const double lon_start_buffer,
                               const double lon_end_buffer);

  void UpdateOvertakenObstacleEdge(
      const Obstacle* obstacle,
      std::vector<ObstacleEdge>* const obstacles_edges, const double lat_buffer,
      const double lon_start_buffer, const double lon_end_buffer);

  void UpdateStartMovingObstacles(const ReferenceLineInfo& reference_line_info);

  void TrimTrackObstaclesEdge();

  bool ComupteLaneChangeKeyPoint(PathBoundPoint* const start_point,
                                 PathBoundPoint* const end_point);
  bool CheckIsCanIntoLaneChange(double* const start_s, double* const end_s);
  bool NeedUpdateOvertakeObstacleEdge(const Obstacle* obstacle);
  bool CheckObsIsCanBeIgnored(const LaneBorrowInfo& lane_borrow_info,
                              const Obstacle* obstacle,
                              double* const not_passable_s,
                              bool* const keep_self_path_straight);
  bool IsNeedBeIgnoredNearJunction(const SLBoundary& obs_sl);
  bool IsNeedIgnoreInCrowdTraffic(const Obstacle* obstacle);
  double GetLaneChangeStartSValue();
  double GetLaneChangeEndSValue(const double start_s);
  double GetRemainLaneChangeTime(const double start_s);

  double GetPublicRoadPathLength(const LaneBorrowInfo& lane_borrow_info);

  void GetLaneBorrowInfoList(
      const ReferenceLineInfo& reference_line_info,
      std::vector<LaneBorrowInfo>* const lane_borrow_info_list);

  bool IsNeedKeepSelfPathStraightBeforeLaneBorrow(
      const LaneBorrowInfo& lane_borrow_info, const Obstacle* obstacle);

  void UpdateKeepSelfPathStraightCount();

  bool CanExtendCurrentLaneLeftBound(const double curr_s);

  bool IsWillPassMergeLane();
  bool IsNeedLaneChangePassMergeLane();

  void GenerateLaneChangePassMergeLanePathBound(
      const ReferenceLineInfo& reference_line_info,
      PathBound* const path_bound);

  void MakeObsDirectionDecision(const LaneBorrowInfo& lane_borrow_info,
                                const Obstacle* obstacle,
                                DirectionDecision* const decision);
  void CheckNeedConsiderIgv(const ReferenceLine& reference_line);
  void IsAdcNearTurn();
  bool IsHigherIGV(const Obstacle& obstacle);

 private:
  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_ld_ = 0.0;
  double adc_l_to_lane_center_ = 0.0;
  double adc_lane_width_ = 0.0;
  double adc_theta_ = 0;
  double adc_v_ = 0;
  bool is_need_to_keep_right_ = false;
  double nearst_obs_s_ = 0;
  double min_lateral_diff_ = std::numeric_limits<double>::max();
  BoundsType bounds_type_ = DEFAULT_PATH_BOUNDS;
  static std::map<std::string, ObstacleEdge> track_obstacle_edge_;

  int lane_change_safe_count_ = 0;
  bool into_lane_change_ = false;
  double lane_change_pre_adc_lateral_l_ = 0.0;
  double lane_change_pre_adc_lateral_v_ = 0.0;
  double start_time_into_lane_change_ = 0.0;
  double routing_end_remain_dis_ = 0.0;
  double merge_lane_end_s_ = std::numeric_limits<double>::max();
  double merge_lane_start_s_ = std::numeric_limits<double>::max();
  double routing_end_s_ = std::numeric_limits<double>::max();
  std::string the_first_merge_lane_id_ = "";
  LaneChangeDirection lane_change_direction_;

  FRIEND_TEST(PathBoundsDeciderTest, InitPathBoundary);
  FRIEND_TEST(PathBoundsDeciderTest, GetBoundaryFromLanesAndADC);
};

}  // namespace planning
}  // namespace century
