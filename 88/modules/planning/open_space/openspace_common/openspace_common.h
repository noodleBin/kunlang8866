/**
 * @file openspace_common.h
 * @author
 * @brief
 * @version 0.1
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <limits>
#include <climits>

#include "cyber/time/clock.h"
#include "cyber/common/log.h"

#include "modules/planning/proto/openspace_config/openspace_common.pb.h"

#include "modules/common/util/point_factory.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/normal_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/tasks/utils/openspace_util.h"

namespace {
constexpr double kSqrt_2 = sqrt(2.0);
constexpr double kPenalizeForward = 10.0;
constexpr double kBaseRatio = 1.0;
constexpr double kMinTurnRadius = 4.0;
constexpr int kMeter2Decimeter = 10;
constexpr int kDrawVoronoFieldGrayscale = 255;
constexpr double kFront2Center = 2.4;
constexpr double kWillOverTime = 0.8;
constexpr double kNearEndDis = 5.0;
constexpr double kEnableVoronoiTimeCnt = 1.5;
}  // namespace

const float INF = std::numeric_limits<float>::infinity();

using century::common::math::Box2d;
using century::common::math::NormalizeAngle;
using century::common::math::Vec2d;
using century::cyber::Clock;

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
  std::vector<double> t;

  size_t result_size = 0;
  size_t explored_node_num = 0;
  size_t run_node_num = 0;
  size_t search_rs_num = 0;
  double astar_total_time = 0.0;
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  double rs_node_x = 0.0;
  double rs_node_y = 0.0;
  double rs_node_phi = 0.0;
  double traj_kappa_contraint_ratio = 0.0;
  // true: gear = D, false: gear = R
  bool gear = true;
};

class ObsPoint {
 public:
  ObsPoint(int x, int y) : x_(x), y_(y) {}
  bool operator!=(const ObsPoint& other) const {
    return x_ != other.x_ || y_ != other.y_;
  }
  bool operator==(const ObsPoint& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }
  void SetX(int value) { x_ = value; }
  void SetY(int value) { y_ = value; }
  const int X() const { return x_; }
  const int Y() const { return y_; }

 private:
  int x_;
  int y_;
};

namespace century {
namespace planning {

/**
 * @brief OpenspaceCommon, process openspace related common function
 */
class OpenspaceCommon {
 public:
  /**
   * @brief constructor
   */
  OpenspaceCommon();
  /**
   * @brief deconstructor
   */
  ~OpenspaceCommon() = default;

 public:
  enum class TurnType {
    NONE = 0,
    LEFT,
    RIGHT,
    STRAIGHT,
    U_TURN,
  };

  // process the points of a single boundary (left or right)
  template <typename BoundaryPointsT, typename RoadBoundaryT>
  void ProcessBoundaryPoints(const RoadBoundaryT& road_boundary,
                             const hdmap::MapPathPoint& check_point,
                             double ignore_road_boundary_dis_thre,
                             BoundaryPointsT boundary_points) {
    const Vec2d check_point_vec2d(check_point.x(), check_point.y());
    for (const auto& line_point : road_boundary.line_points) {
      Vec2d point(line_point.x(), line_point.y());
      double dist = point.DistanceTo(check_point_vec2d);
      if (dist > ignore_road_boundary_dis_thre && boundary_points->size() > 1) {
        continue;
      }
      boundary_points->emplace_back(std::move(point));
    }
  }

 public:
  /**
   * @brief hybrid a star debug status
   */
  enum class HybridAStarDebugStatus {
    NORMAL = 0,
    START_COLLISION = 1,
    END_COLLISION = 2,
    PLAN_OVERTIME = 3,
    PLAN_OVERTIME2 = 4,
    SEARCH_FAIL = 5,
    GET_BIDIRECTION_RESULT_FAILED,
    GET_SINGLE_RESULT_FAILED,
    PLAN_ITERATION_EXCEEDED,
    OTHERS,

    BIDIRECTION_SAERCH_SUCCESS = 50,
    FALLBACK_TO_SINGLE_DIRECTION_SEARCH,
    FALLBACK_SWAP_START_END_SEARCH,
  };

  bool isInSpecialArea(const century::common::VehicleState& vehicle_state,
                       const OpenspaceCommonConfig& common_config);

  bool IsAreaContainAdc(const century::common::VehicleState& vehicle_state,
                        const OpenspaceCommonConfig& area_config,
                        const SpecialAreaType& target_area);

  bool OffsetBoundaryWithTurnType(
      const planning::Frame* const frame,
      const century::common::VehicleState& vehicle_state,
      std::vector<century::common::math::Vec2d>& left_lane_boundary,
      std::vector<century::common::math::Vec2d>& right_lane_boundary,
      const double& offset_config);

  bool CalTheta(HybridAStartResult& result);

  void CalTheta(const bool is_reverse,
                planning::DiscretizedTrajectory* trajectory);

  double ConfirmFinalTheta(const double& orin_heading,
                           const double& is_backward_traj,
                           const double& is_reverse);

  void CalTrajS(std::vector<century::common::TrajectoryPoint>* trajectory,
                const bool is_gear_reverse = false);

  hdmap::Lane::LaneTurn GetTurnType() { return turn_type_; }

  static bool is_reverse() { return is_reverse_driving_; }

  static void set_is_reverse(const bool is_reverse) {
    is_reverse_driving_ = is_reverse;
  }

  static bool is_reverse_routing() { return is_reverse_routing_; }

  static planning::OpenspaceReason get_openspace_reason() {
    return openspace_reason_;
  }

  static void set_openspace_reason(const planning::OpenspaceReason reason) {
    openspace_reason_ = reason;
  }

  static bool IsRoutingReverseDriving(
      const century::planning::LocalView& local_view);

  // true: lateral error or heading error too large
  bool CheckAdcErrorStates(const century::common::VehicleState& vehicle_state,
                           const TrajGearPair& traj_info);

  bool CheckLateralErrorState(
      const common::math::Vec2d& ego_pt,
      const century::planning::DiscretizedTrajectory& trajectory);

  bool CheckHeadingErrorState(
      const common::math::Vec2d& ego_pt, const double& ego_heading,
      const century::planning::DiscretizedTrajectory& trajectory);

  bool SelectNeighborTurnTypeLane(
    std::vector<std::string>& neigh_turn_lane_ids);

  void TraverseLaneSuccessorsWithDepth(
      const std::pair<std::string, ReferenceLineInfo::LaneType>&
          start_lane_id_map,
      const int max_depth,
      const century::hdmap::Lane::LaneTurn target_turn_type,
      std::vector<std::string>* result);

  void FilterNeighborLanesAccordingToTurnType(
      century::hdmap::Lane::LaneTurn turn_type,
      std::unordered_map<std::string, ReferenceLineInfo::LaneType>&
          neigh_lane_id_map);

  bool FindTurnTypeInTargetLane(const std::vector<century::hdmap::Id>& lane_ids,
                                century::hdmap::Lane::LaneTurn& type);

  bool SelectNeighborLaneInfos(
      const std::vector<century::hdmap::Id>& lane_ids,
      std::unordered_map<std::string, ReferenceLineInfo::LaneType>&
          neigh_lane_id_map,
      std::vector<century::hdmap::Id>& select_neigh_lane_ids);

  bool GetNeighborLaneBoundaryFromMap(
      const hdmap::Path& nearby_path,
      std::vector<century::hdmap::Id> neighbor_lane_ids,
      const double center_line_s, const double longi_range_start_s,
      const double longi_range_end_s, const double search_radius,
      const double ignore_road_boundary_dis_thre, const double step,
      const Vec2d& origin_point, const double origin_heading,
      std::vector<Vec2d>* left_lane_boundary,
      std::vector<Vec2d>* right_lane_boundary);

  bool UpdateBoundaryWithNeighborLane(
      const century::hdmap::Lane::LaneTurn turn_type,
      const std::vector<century::common::math::Vec2d>& neighbor_left_boundaries,
      const std::vector<century::common::math::Vec2d>&
          neighbor_right_boundaries,
      std::vector<century::common::math::Vec2d>& left_boundaries,
      std::vector<century::common::math::Vec2d>& right_boundaries);

  bool ExpandNeighborLaneIdForBoundary(
      const century::common::VehicleStateProvider* const vehicle_state,
      const planning::Frame* frame, const hdmap::HDMap* hd_map,
      const double curr_s, std::vector<century::hdmap::Id>& target_lane_ids,
      std::vector<century::hdmap::Id>& neigh_lane_ids,
      double* const neighbor_lane_width);

  std::unordered_set<HybridAStarDebugStatus>* MutableHADebugStatus() {
    return &ha_debug_status_;
  }

  const std::unordered_set<HybridAStarDebugStatus>& GetHADebugStatus() const {
    return ha_debug_status_;
  }

 private:
  OpenspaceCommonConfig common_config_;

  const hdmap::HDMap* hdmap_ = nullptr;
  century::common::VehicleParam vehicle_param_;
  century::common::VehicleState vehicle_state_;

  static bool is_reverse_driving_;
  static bool is_reverse_routing_;

  century::hdmap::Lane::LaneTurn turn_type_ =
      century::hdmap::Lane::NO_TURN;

  static planning::OpenspaceReason openspace_reason_;
  std::unordered_set<HybridAStarDebugStatus> ha_debug_status_;
};

}  // namespace planning
}  // namespace century
