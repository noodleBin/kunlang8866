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

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/open_space_info.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace century {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

typedef std::vector<std::pair<int, std::vector<century::common::Point3D>>>
    StaticAreaPolygons;
using BoundaryMap =
    std::unordered_multimap<std::string,
                            std::tuple<std::string, double, double>>;

enum ReroutingType {
  NO_SPECIAL = 0u,
  HUMAN_SHAPED = 1u,
  DEAD_END_ROAD = 2u,
  LOOP_RUNING = 3u,
  PART_HUMAN_SHAPED = 4u,
  SECOND_REROUTING = 5u,
  D7_BLOCK = 6u,
  REVERSE = 7u,
};

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state,
        ReferenceLineProvider *reference_line_provider);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);

  virtual ~Frame() = default;

  const common::TrajectoryPoint &PlanningStartPoint() const;

  common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints,
      const EgoInfo *ego_info);

  common::Status InitForOpenSpace(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  const std::list<ReferenceLineInfo> &reference_line_info() const;
  std::list<ReferenceLineInfo> *mutable_reference_line_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *FindTargetReferenceLineInfo();

  const ReferenceLineInfo *FindFailedReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const std::vector<const Obstacle *> open_space_roi_obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);
  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id,
      const century::common::math::Vec2d &center, const double heading,
      const double length, const double width);
  const Obstacle *CreateStackerObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id,
      const century::common::math::Vec2d &center, const double heading,
      const double length, const double width);
  const Obstacle *CreateStackerObstacleWithID(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id,
      const century::common::math::Vec2d &center, const double heading,
      const double length, const double width, const double speed);
  const Obstacle *CreateWheelCraneObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id,
      const century::common::math::Vec2d &center, const double heading,
      const double length, const double width);
  void AddOpenSpaceRoiObstacle(const Obstacle &obstacle);

  const BoundaryMap &GetBoundaryMap() { return multi_boundary_map_; }

  void AddBoundary(const std::string &base_id, const std::string &obs_id,
                   double first_time, double last_time) {
    multi_boundary_map_.emplace(base_id,
                                std::make_tuple(obs_id, first_time, last_time));
  }
  bool CheckReroutingInput();
  void FillFirstPoint(routing::RoutingRequest &request);
  bool FillEndPoint(routing::RoutingRequest &request,
                 const ReroutingType &type,
                 double move_distance = 0.0);
  bool Rerouting(PlanningContext *planning_context,
                 const ReroutingType type = ReroutingType::NO_SPECIAL,
                 double move_distance = 0.0);
  bool FillBackwardPoint(routing::RoutingRequest &request,
                         double move_distance);

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  void set_current_frame_planned_trajectory(
      ADCTrajectory current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ =
        std::move(current_frame_planned_trajectory);
  }

  const ADCTrajectory &current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }

  // x y heading
  void set_dynamic_adc_position(
      const century::common::Point3D &dynamic_adc_position) {
    dynamic_adc_position_ = dynamic_adc_position;
  }

  const century::common::Point3D &dynamic_adc_position() const {
    return dynamic_adc_position_;
  }

  void set_current_frame_planned_path(
      DiscretizedPath current_frame_planned_path) {
    current_frame_planned_path_ = std::move(current_frame_planned_path);
  }

  const DiscretizedPath &current_frame_planned_path() const {
    return current_frame_planned_path_;
  }

  const bool is_sim_control() const { return is_sim_control_; }
  void set_is_sim_control(const bool is_sim_control) {
    is_sim_control_ = is_sim_control;
  }
  const double planning_start_time() const { return planning_start_time_; }
  void set_planning_start_time(const double planning_start_time) {
    planning_start_time_ = planning_start_time;
  }

  const bool is_near_destination() const { return is_near_destination_; }

  const bool IsNeedEnterAstar() const { return is_need_enter_astar_; }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);

  const LocalView &local_view() const { return local_view_; }

  const common::VehicleState &vehicle_state() { return vehicle_state_; }

  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }

  const StaticAreaPolygons &static_area_polygon() {
    return static_area_polygon_;
  }

  const std::vector<common::math::Polygon2d> &costmap_polygons() {
    return costmap_polygons_;
  }

  std::vector<common::math::Polygon2d> &filter_costmap_polygons() {
    return costmap_polygons_;
  }

  const OpenSpaceInfo &open_space_info() const { return open_space_info_; }

  OpenSpaceInfo *mutable_open_space_info() { return &open_space_info_; }

  void GetSignal(const std::string &traffic_light_id,
                 perception::TrafficLight *const traffic_results) const;

  bool NoGreenTrafficLight(const std::string &traffic_light_id);

  bool IsNewestSuperTrafficLight();

  bool GetSignalCenterFromId(const std::string &traffic_light_id,
                             century::common::PointENU *light_pose);

  const DrivingAction &GetPadMsgDrivingAction() const {
    return pad_msg_driving_action_;
  }

  void SetPlanningContext(PlanningContext *context);

  void SetCurrentTrafficLightId(const std::string &traffic_light_id);
  const std::string &GetCurrentTrafficLightId() const;
  void UpdateOvertakeStatus(const OvertakeStatus::Status &overtake_direction);
  void UpdateLaneBorrowLaneId(const std::string &lane_borrow_lane_id);
  void UpdateLaneChangeLaneId(const std::string &lane_change_lane_id);
  void SetOvertakeReportState(const ADCTrajectory::OvertakeReportState &state) {
    overtake_state_ = state;
  }
  const ADCTrajectory::OvertakeReportState &GetOvertakeReportState() const {
    return overtake_state_;
  }

  void SetRecordTime(double time) { record_time_ = time; }
  double GetRecordTime() const { return record_time_; }

  template <typename T>
  bool WithinBound(T start, T end, T value) {
    return value >= start && value <= end;
  }
  double stop_line_s_ = 0.0;
  perception::TrafficLight::Color signal_color_;
  enum class MixedTrafficType {
    UNKNOWN,
    STATIC_ROAD_MEETED,
    DYNAMIC_OBSTACLE_MEETED,
  };
  MixedTrafficType mixed_traffic_type_ = MixedTrafficType::UNKNOWN;

  enum class FaultReport {
    TEB_NORMAL = 0,
    TEB_WARRING = 1,
    TEB_STOP_LONG = 2,
    TEB_MORE_FAILED = 3,
    PRP_NORMAL = 4,
    PRP_WARRING = 5,
    PRP_FAILED = 6,
  };
  FaultReport fault_report_ = FaultReport::PRP_NORMAL;

  enum class AccLevel {
    ACC_NORMAL = 0,
    ACC_ONE_LEVEL = 1,
    ACC_TWO_LEVEL = 2,
    ACC_THREE_LEVEL = 3,
    ACC_FOUR_LEVEL = 4,
  };
  AccLevel acc_level_ = AccLevel::ACC_NORMAL;

 private:
  common::Status InitFrameData(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);

  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
                               const std::list<hdmap::RouteSegments> &segments);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle(const EgoInfo *ego_info) const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

  void ReadTrafficLights();
  void ReadSuperTrafficLights();

  void ReadPadMsgDrivingAction();
  void ResetPadMsgDrivingAction();

 private:
  static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;
  century::common::Point3D dynamic_adc_position_;

  bool is_near_destination_ = false;
  bool is_need_enter_astar_ = false;
  bool is_sim_control_ = false;
  double planning_start_time_ = 0.0;

  double record_time_ = 0.0;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;
  ThreadSafeIndexedObstacles open_space_roi_obstacles_;

  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  std::unordered_map<std::string, const mcloud::SuperTrafficLight *>
      super_traffic_lights_;

  std::unordered_map<std::string, const perception::TrafficLightInPixelStatus *>
      traffic_insight_status_;

  // current frame published trajectory
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  DiscretizedPath current_frame_planned_path_;

  ReferenceLineProvider *reference_line_provider_ = nullptr;

  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  PlanningContext *context_;

  std::string current_traffic_light_id_ = "-1";

  StaticAreaPolygons static_area_polygon_;
  std::vector<common::math::Polygon2d> costmap_polygons_;

  ADCTrajectory::OvertakeReportState overtake_state_;

  BoundaryMap multi_boundary_map_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

}  // namespace planning
}  // namespace century
