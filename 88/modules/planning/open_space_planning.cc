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

#include "modules/planning/open_space_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <sstream>
#include <utility>

#include "gtest/gtest_prod.h"

#include "absl/strings/str_cat.h"

#include "modules/map/proto/map_junction.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/kalman_filter.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_diff_value.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_value.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_speed_distance_history.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_stabilization_for_teb_speed.h"
#include "modules/planning/common/historical_tracking_algorithms/start_up_vehicle_position_history.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/traffic_rules/traffic_decider.h"

namespace century {
namespace planning {
using century::canbus::Chassis;
using century::common::EngageAdvice;
using century::common::ErrorCode;
using century::common::FrenetFramePoint;
using century::common::PathPoint;
using century::common::PointENU;
using century::common::SLPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::VehicleState;
using century::common::VehicleStateProvider;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
using century::cyber::Rate;
using century::dreamview::Chart;
using century::hdmap::HDMapUtil;
using century::hdmap::Junction;
using century::hdmap::JunctionInfoConstPtr;
using century::perception::PerceptionObstacle;
using century::planning::Obstacle;
using century::planning::scenario::Stage;
using century::planning::scenario::rescue::RescueStageTeb;
using century::planning_internal::SLFrameDebug;
using century::planning_internal::SpeedPlan;
using century::planning_internal::STGraphDebug;
using century::prediction::ObstaclePriority;
using century::routing::RoutingRequest;

namespace {
constexpr int kSleepTime = 50;                    // in milliseconds
constexpr double kTimeThreshold = 0.14;           // in seconds
constexpr double kFallbackOncetime = 0.01;        // in seconds
constexpr double kDistanceToDestination = 5;      // m
constexpr double KPlanningErrorTotalTime = 20.0;  // s
constexpr double KNormalStopTime = 20.0;          // s
constexpr double KStopLineStopTime = 50.0;        // s
constexpr double KLaneBorrowFailTime = 1.0;       // s
constexpr int KStopEnsureCount = 5;
constexpr double KStopSpeedThreshold = 0.05;  // m/s
constexpr int kMaxTaskFailureCount = 5;
constexpr size_t kMaxFailurePathPointNum = 100;
constexpr double kRoadWidthBuffer = 0.3;
constexpr double kAdcPolygonBuffer = 0.0;
constexpr int kStepThreshold = 50;
constexpr double kDecelerationThreshold = -0.5;
constexpr double kSpeedPlanChartMaxS = 70.0;
constexpr double kSpeedPlanChartMaxV = 10.0;
constexpr double kSpeedPlanChartHalfMaxS = 0.5 * kSpeedPlanChartMaxS;
constexpr double kSpeedPlanChartHalfMaxV = 0.5 * kSpeedPlanChartMaxV;
constexpr double kSpeedPlanChartQuarterMaxS = 0.25 * kSpeedPlanChartMaxS;
constexpr double kSpeedPlanChartQuarterMaxV = 0.25 * kSpeedPlanChartMaxV;
constexpr double kSpeedPlanChartEighthMaxS = 0.125 * kSpeedPlanChartMaxS;
constexpr double kSpeedPlanChartEighthMaxV = 0.125 * kSpeedPlanChartMaxV;
constexpr double kFreezeTime = 10.0;
constexpr double kMaxMixedFlowSpeed = 4.0;
constexpr double kJunctionSearchRadius = 1.0;
constexpr double kStopedSpeed = 1e-3;
constexpr double kEStopSpeed = 0.15;
constexpr double kReferenceLineSampleStep = 0.1;
constexpr double kReachedStationTime = 2.0;  // s
constexpr int kPerceptionId = 9000000;
constexpr double kLargeAeraBuffer = 2.0;
constexpr double kMaxDisplayThreshold = 2.1;
constexpr double kMinDisplayThreshold = -1.1;
constexpr double kWindowSizeThreshold = 10.0;
constexpr double kCheckTrajectoryTime = 600;             // ms
constexpr double kCheckTrajectoryTimePublicRoad = 3100;  // ms
constexpr double kSingleLaneWidthThr = 4.5;
constexpr double kDistanceToOther = 30;        // m
constexpr double kDistanceToTrafficLine = 70;  // m
constexpr double kInvalidTrajectoryPercent = 0.8;
constexpr double KLowSpeedThreshold = 0.5;  // m/s
constexpr int kMinTrajSize = 5;             // s
constexpr double kDistanceReach = 5;
constexpr double kMinTrajctoryLength = 0.3;  // m
constexpr double kOffLaneLatitudebuffer = 0.0;
}  // namespace

OpenSpacePlanning::~OpenSpacePlanning() {
  std::lock_guard<std::mutex> frame_lock(frame_mutex_);

  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_teb_history()->Clear();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_routing_.Clear();
  injector_->ego_info()->Clear();
  FallbackStop();
}

std::string OpenSpacePlanning::Name() const { return "open_space_planning"; }

Status OpenSpacePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  planner_dispatcher_->Init();

  ACHECK(century::cyber::common::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;
  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);

    // clear planning history
    injector_->history()->Clear();

    // clear planning status
    injector_->planning_context()->mutable_planning_status()->Clear();
  }
  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), hdmap_);
  reference_line_provider_->SetPlanningContext(injector_->planning_context());
  reference_line_provider_->Start();
  std::shared_ptr<century::cyber::Node> node(century::cyber::CreateNode(
      config_.topic_config().planning_trajectory_topic()));
  planning_writer_ = node->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());
  if (FLAGS_enable_fallback_planning_thread) {
    fallback_planning_thread_.reset(new cyber::Thread(
        "fallback_plan", &OpenSpacePlanning::FallbackPlanningThread, this));
    ADEBUG << "OpenSpacePlanning::FallbackPlanningThread is created.";
  }
  // dispatch planner
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  start_time_ = Clock::NowInSeconds();
  planning_error_start_time_ = Clock::NowInSeconds();
  return planner_->Init(config_);
}

bool OpenSpacePlanning::IsJunctionContainAdc(
    const VehicleState& vehicle_state,
    const hdmap::JunctionInfo& junction_info) const {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  Box2d adc_box(Vec2d(vehicle_state.x(), vehicle_state.y()),
                vehicle_state.heading(),
                vehicle_param.length() - kAdcPolygonBuffer * 2,
                vehicle_param.width() - kAdcPolygonBuffer * 2);
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(adc_box));
}

Status OpenSpacePlanning::IsAdcInRoad(ReferenceLineInfo* reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_adc_located_in_junction(false);
  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  reference_line_info->ResetCommonJunction();
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, kJunctionSearchRadius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (Junction::COMMON_JUNCTION == ptr_junction->junction().type() ||
          Junction::IN_ROAD == ptr_junction->junction().type() ||
          Junction::CROSS_ROAD == ptr_junction->junction().type()) {
        mutable_path_decider_status->set_adc_located_in_junction(true);
      }
      if (Junction::COMMON_JUNCTION == ptr_junction->junction().type()) {
        reference_line_info->SetCommonJunction();
      }
      if (Junction::COMMON_JUNCTION == ptr_junction->junction().type() &&
          IsJunctionContainAdc(vehicle_state, *ptr_junction)) {
        return Status::OK();
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double curr_road_left_width = 0, curr_road_right_width = 0;
  reference_line_info->GetRoadWidthBasedAdc(&curr_road_left_width,
                                            &curr_road_right_width);
  if (adc_sl.start_l() < -curr_road_right_width - kRoadWidthBuffer ||
      adc_sl.end_l() > curr_road_left_width + kRoadWidthBuffer) {
    const std::string msg = "ADC is outside of the road.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

void OpenSpacePlanning::IsAdcInCommonJunction(
    ReferenceLineInfo* reference_line_info) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, kJunctionSearchRadius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if ((Junction::COMMON_JUNCTION == ptr_junction->junction().type() ||
           Junction::IN_ROAD == ptr_junction->junction().type()) &&
          IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction)) {
        // in commonjunction
        reference_line_info->SetIsAdcInCommonJunction(true);
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
  bool is_near_traffic_light =
      reference_line_info->IsNearTrafficLightStopLine();
  bool is_in_common_junction = reference_line_info->IsAdcInCommonJunction();
  ADEBUG << "is_near_traffic_light = " << is_near_traffic_light;
  ADEBUG << "is_in_common_junction = " << is_in_common_junction;
  if (injector_->is_in_traffic_light_junction_) {
    if (!is_in_common_junction) {
      injector_->is_in_traffic_light_junction_ = false;
    }
  } else {
    if (is_near_traffic_light && is_in_common_junction) {
      injector_->is_in_traffic_light_junction_ = true;
    }
  }
  ADEBUG << "injector_->is_in_traffic_light_junction_ = "
         << injector_->is_in_traffic_light_junction_;
}
Status OpenSpacePlanning::IsAdcDeviateLaneDirection(
    const ReferenceLineInfo* reference_line_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double adc_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  double adc_heading = injector_->vehicle_state()->heading();
  double ref_heading =
      reference_line_info->reference_line().GetReferencePoint(adc_s).heading();
  ADEBUG << "adc_heading: " << adc_heading;
  ADEBUG << "ref_heading: " << ref_heading;
  bool near_ref_angle = std::fabs(century::common::math::AngleDiff(
                            adc_heading, ref_heading)) < M_PI_2;
  if (!near_ref_angle) {
    const std::string msg = "ADC deviation from the direction of the road.";
    AERROR << msg;
    ptr_trajectory_pb->set_lane_status(ADCTrajectory::OPPOSITE_LANE_DIRECTION);
    injector_->is_adc_deviate_lane_direction_ = true;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ptr_trajectory_pb->set_lane_status(ADCTrajectory::FOLLOW_LANE_DIRECTION);
  return Status::OK();
}

void OpenSpacePlanning::AddModifiedObstacle(
    ReferenceLineInfo* const reference_line_info,
    const std::vector<std::pair<std::string, double>>& corrected_obstacles) {
  PathDecision* const path_decision = reference_line_info->path_decision();
  for (const auto& obstacle : corrected_obstacles) {
    auto* mutable_obstacle = path_decision->Find(obstacle.first);
    if (nullptr != mutable_obstacle) {
      if (mutable_obstacle->IsModifiedVelocity() &&
          mutable_obstacle->HasTrajectory()) {
        // AINFO << "has modified obs";
        std::string modified_obs_id = static_cast<std::string>(
            std::to_string(mutable_obstacle->Perception().id() +
                           kPerceptionId) +
            static_cast<std::string>("_0"));
        perception::PerceptionObstacle modified_obs_perception_obstacle;
        modified_obs_perception_obstacle.set_id(
            mutable_obstacle->Perception().id() + kPerceptionId);
        modified_obs_perception_obstacle.CopyFrom(
            mutable_obstacle->Perception());
        double velocity_heading = mutable_obstacle->Perception().theta();
        double velocity_x =
            mutable_obstacle->ModifiedVelocity() * std::cos(velocity_heading);
        double velocity_y =
            mutable_obstacle->ModifiedVelocity() * std::sin(velocity_heading);
        // AINFO << "before_velocity_x = "
        //       << mutable_obstacle->Perception().velocity().x();
        // AINFO << "before_velocity_y = "
        //       << mutable_obstacle->Perception().velocity().y();
        // AINFO << "modified_velocity_x = " << velocity_x;
        // AINFO << "modified_velocity_y = " << velocity_y;

        modified_obs_perception_obstacle.mutable_velocity()->set_x(velocity_x);
        modified_obs_perception_obstacle.mutable_velocity()->set_y(velocity_y);

        prediction::Trajectory modified_obs_trajectory;
        auto& trajectory_points =
            mutable_obstacle->Trajectory().trajectory_point();

        for (int i = 0; i < trajectory_points.size(); ++i) {
          const auto& traj_point = trajectory_points[i];
          const auto& ori_point = trajectory_points[0].path_point();
          double relative_time = traj_point.relative_time();

          // common::TrajectoryPoint modified_obs_traj_point;
          auto* modified_obs_traj_point =
              modified_obs_trajectory.add_trajectory_point();
          modified_obs_traj_point->set_v(mutable_obstacle->ModifiedVelocity());
          modified_obs_traj_point->set_a(0.0);
          modified_obs_traj_point->set_da(traj_point.da());
          modified_obs_traj_point->set_steer(traj_point.steer());
          modified_obs_traj_point->mutable_gaussian_info()->CopyFrom(
              traj_point.gaussian_info());
          modified_obs_traj_point->set_relative_time(relative_time);
          // common::PathPoint modified_obs_path_point;
          auto modified_obs_path_point =
              modified_obs_traj_point->mutable_path_point();
          // AINFO << "relative_time = " << relative_time;
          double x = ori_point.x() + velocity_x * relative_time;
          double y = ori_point.y() + velocity_y * relative_time;
          modified_obs_path_point->set_x(x);
          modified_obs_path_point->set_y(y);
          // AINFO << "x = " << std::setprecision(9) << x
          //       << "   y = " << std::setprecision(9) << y;
        }

        prediction::ObstaclePriority modified_obs_obstacle_priority;
        Obstacle modified_obs(modified_obs_id, modified_obs_perception_obstacle,
                              modified_obs_trajectory,
                              ObstaclePriority::CAUTION,
                              mutable_obstacle->IsStatic());

        reference_line_info->AddObstacle(&modified_obs);
        auto* result_obstacle = path_decision->Find(modified_obs.Id());
        result_obstacle->SetPerceptionId(mutable_obstacle->Perception().id() +
                                         kPerceptionId);
        // AINFO << "result_obstacle: " << result_obstacle->PerceptionId();
        // AINFO << "result_obstacle = " << result_obstacle->speed();
        // AINFO << "result_obstacle_s_start = "
        //       << result_obstacle->PerceptionSLBoundary().start_s();
        // AINFO << "is_static = " << result_obstacle->IsStatic();
        // AINFO << "bounding_box = "
        //       << result_obstacle->PerceptionBoundingBox().center_x();
      }
    }
  }
}

Status OpenSpacePlanning::InitTEBFrame(
    const uint32_t sequence_num, const TrajectoryPoint& planning_start_point,
    const VehicleState& vehicle_state) {
  frame_teb_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                             vehicle_state, reference_line_provider_.get()));
  if (frame_teb_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }
  frame_teb_->SetPlanningContext(injector_->planning_context());

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;

  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    const std::string msg = "Failed to create reference line";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  AERROR << "reference_lines.size(): " << reference_lines.size();
  DCHECK_EQ(reference_lines.size(), segments.size());

  auto forward_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto& ref_line : reference_lines) {
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  // ------------Just Edited For DeadEnd Scenario-------------------------
  if (!wait_flag_) {
    for (auto& seg : segments) {
      if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                      FLAGS_look_backward_distance, forward_limit)) {
        const std::string msg = "Fail to shrink routing segments.";
        AERROR << msg;
        // Add planning error need restart function
        if (nullptr != local_view_.routing &&
            local_view_.routing->has_header() &&
            std::abs(Clock::NowInSeconds() -
                     local_view_.routing->header().timestamp_sec()) <
                FLAGS_signal_expire_time_sec &&
            vehicle_state.linear_velocity() < kEStopSpeed) {
          return Status(ErrorCode::PLANNING_ERROR_NEED_RESTART, msg);
        } else {
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
      }
    }
  }
  // ------------Just Edited For DeadEnd Scenario-------------------------
  auto status_teb = frame_teb_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());

  if (!status_teb.ok()) {
    AERROR << "failed to init frame_teb:" << status_teb.ToString();
    return status_teb;
  }

  return Status::OK();
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OpenSpacePlanning::GenerateStopTrajectory(
    ADCTrajectory* ptr_trajectory_pb) {
  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);
    ptr_trajectory_pb->clear_trajectory_point();
  }

  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double max_t = FLAGS_fallback_total_time;  // 3.0 s
  const double unit_t = FLAGS_fallback_time_unit;  // 0.1 s

  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

bool OpenSpacePlanning::JudgeCarInDeadEndJunction(
    std::vector<JunctionInfoConstPtr>* junctions, const Vec2d& car_position,
    JunctionInfoConstPtr* target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    return false;
  }
  for (size_t i = 0; i < junction_num; ++i) {
    if (Junction::DEAD_END == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      // judge dead end point in the select junction
      if (polygon.IsPointIn(car_position)) {
        *target_junction = junctions->at(i);
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

bool OpenSpacePlanning::DeadEndHandle(const PointENU& dead_end_point,
                                      const VehicleState& vehicle_state) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(dead_end_point, kJunctionSearchRadius,
                                 &junctions) != 0) {
    ADEBUG << "Fail to get junctions from base_map.";
    return false;
  }
  if (junctions.size() <= 0) {
    ADEBUG << "No junction from map";
    return false;
  }
  Vec2d car_position;
  car_position.set_x(vehicle_state.x());
  car_position.set_y(vehicle_state.y());
  if (!JudgeCarInDeadEndJunction(&junctions, car_position, &junction)) {
    ADEBUG << "Target Dead End not found";
    return false;
  }
  return true;
}

void OpenSpacePlanning::GetScopeParams(const bool is_large_scope,
                                       double* left_scope, double* right_scope,
                                       double* front_scope,
                                       double* rear_scope) {
  CHECK_NOTNULL(left_scope);
  CHECK_NOTNULL(right_scope);
  CHECK_NOTNULL(front_scope);
  CHECK_NOTNULL(rear_scope);
  *left_scope = FLAGS_left_scope_dis_for_correct_speed;
  *right_scope = FLAGS_right_scope_dis_for_correct_speed;
  *front_scope = FLAGS_front_scope_dis_for_correct_speed;
  *rear_scope = FLAGS_rear_scope_dis_for_correct_speed;
  if (is_large_scope) {
    *left_scope += kLargeAeraBuffer;
    *right_scope += kLargeAeraBuffer;
    *front_scope += kLargeAeraBuffer;
    *rear_scope += kLargeAeraBuffer;
  }
}

bool OpenSpacePlanning::GetScopeState(const Obstacle& obstacle,
                                      const ReferenceLineInfo* ref_line_info,
                                      const bool is_large_scope,
                                      const double left_scope,
                                      const double right_scope,
                                      const double front_scope,
                                      const double rear_scope) {
  const SLBoundary& adc_sl = ref_line_info->AdcSlBoundary();
  const auto& obs_sl = obstacle.PerceptionSLBoundary();

  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  bool is_lateral_in_scope = !(obs_sl.start_l() > adc_half_width + left_scope ||
                               obs_sl.end_l() < -adc_half_width - right_scope);

  bool is_longitudinal_in_scope =
      obs_sl.start_s() < adc_sl.end_s() + front_scope &&
      obs_sl.end_s() > adc_sl.start_s() - rear_scope;
  bool is_in_scope = false;
  if (is_lateral_in_scope && is_longitudinal_in_scope) {
    is_in_scope = true;
  }
  bool without_lat_overlap =
      obs_sl.start_l() > adc_sl.end_l() || adc_sl.start_l() > obs_sl.end_l();

  double lon_diff_dis_without_overlap = obs_sl.end_s() - adc_sl.start_s();

  double lat_diff_dis_without_overlap = std::max(
      obs_sl.start_l() - adc_sl.end_l(), adc_sl.start_l() - obs_sl.end_l());

  if (!is_large_scope && without_lat_overlap &&
      lon_diff_dis_without_overlap > 0.0 &&
      (lat_diff_dis_without_overlap >
       FLAGS_aspect_range_ratio_for_correct_speed *
           lon_diff_dis_without_overlap)) {
    is_in_scope = false;
  }
  ADEBUG << "is_lateral_in_scope: " << is_lateral_in_scope;
  ADEBUG << "is_longitudinal_in_scope: " << is_longitudinal_in_scope;
  ADEBUG << "is_in_scope: " << is_in_scope;
  ADEBUG << "obs l: (" << obs_sl.start_l() << ", " << obs_sl.end_l() << ")";
  ADEBUG << "obs s: (" << obs_sl.start_s() << ", " << obs_sl.end_s() << ")";
  ADEBUG << "adc s: (" << adc_sl.start_s() << ", " << adc_sl.end_s() << ")";
  return is_in_scope;
}

bool OpenSpacePlanning::IsObstacleInLargeScope(
    const Obstacle& obstacle, const ReferenceLineInfo* ref_line_info) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(true, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret = GetScopeState(obstacle, ref_line_info, true, left_scope, right_scope,
                      front_scope, rear_scope);
  ADEBUG << "IsObstacleInLargeScope: " << ret;
  return ret;
}

bool OpenSpacePlanning::IsObstacleInScope(
    const Obstacle& obstacle, const ReferenceLineInfo* ref_line_info) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(false, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret = GetScopeState(obstacle, ref_line_info, false, left_scope, right_scope,
                      front_scope, rear_scope);
  ADEBUG << "IsObstacleInScope: " << ret;
  return ret;
}

void OpenSpacePlanning::FindUnreasonableSpeedObstacles(
    ReferenceLineInfo* ref_line_info,
    std::vector<std::pair<std::string, double>>* corrected_obstacles) {
  static ObstacleSpeedDistanceHistory obstacles_history(20UL);
  ADEBUG << "FindUnreasonableSpeedObstacles!";
  CHECK_NOTNULL(ref_line_info);
  CHECK_NOTNULL(corrected_obstacles);
  const auto& ObstacleItems =
      ref_line_info->path_decision()->obstacles().Items();
  for (auto* obstacle : ObstacleItems) {
    ADEBUG << "obstacle Id: " << obstacle->Id();
    if (!IsObstacleInLargeScope(*obstacle, ref_line_info)) {
      continue;
    }
    double correced_speed = obstacle->speed();
    bool has_correced =
        obstacles_history.CorrectObstacleSpeed(*obstacle, &correced_speed);

    if (!IsObstacleInScope(*obstacle, ref_line_info)) {
      continue;
    }
    if (has_correced && correced_speed < obstacle->speed()) {
      corrected_obstacles->emplace_back(obstacle->Id(), correced_speed);
    }
  }
}

void OpenSpacePlanning::RunOnce(const LocalView& local_view,
                                ADCTrajectory* const ptr_trajectory_pb) {
  // when rerouting, reference line might not be updated. In this case, planning
  // module maintains not-ready until be restarted.
  Status status;
  VehicleState vehicle_state;
  std::vector<TrajectoryPoint> stitching_trajectory;
  std::string replan_reason;

  double start_timestamp = 0.0;
  double start_system_timestamp = 0.0;
  uint32_t frame_num = 0;

  if (!PrepareAndUpdateVehicleState(local_view, ptr_trajectory_pb,
                                    &vehicle_state,
                                    &stitching_trajectory,
                                    &replan_reason,
                                    &start_timestamp,
                                    &start_system_timestamp,
                                    &frame_num)) {
    return;
  }
  HysteresisInterval::SetSequenceNum(frame_num);
  ObstacleHistoryValue::SetSequenceNum(frame_num);
  ObstacleSpeedDistanceHistory::SetSequenceNum(frame_num);
  ObstacleHistoryDiffValue::SetSequenceNum(frame_num);
  ObstacleStabilizationForTEBSpeed::SetSequenceNum(frame_num);
  StartUpVehiclePositionHistory::SetSequenceNum(frame_num);
  injector_->SetSequenceNum(frame_num);
  if (!BuildFrameAndTrafficProcess(ptr_trajectory_pb, vehicle_state,
                                   stitching_trajectory, frame_num,
                                   start_timestamp)) {
    return;
  }

  if (!injector_->pullover_finished) {
    if (!GenerateTEBThread(stitching_trajectory, ptr_trajectory_pb)) {
      finish_status_ = true;
      calculation_result_ = false;
      ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      frame_teb_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
      JudgeArrivedStationImmediately(ptr_trajectory_pb);
      RemoteDecider(ptr_trajectory_pb);
      injector_->ResetTaskFailureInfo();
      const uint32_t n = frame_teb_->SequenceNum();
      AERROR << "is_blocked_by_dynamic_obj: "
             << frame_teb_->open_space_info().is_blocked_by_dynamic_obj();
      injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
      return;
    }
  }

  if (!frame_teb_->open_space_info().is_on_open_space_trajectory() &&
      !injector_->pullover_finished) {
    injector_->is_adc_out_lane_ = false;
    injector_->is_adc_deviate_lane_direction_ = false;
    // check adc in common junction
    Status outlane_status(ErrorCode::OK);
    for (auto& ref_line_info : *frame_teb_->mutable_reference_line_info()) {
      if (!ref_line_info.IsChangeLanePath()) {
        outlane_status = IsAdcInRoad(&ref_line_info);
        if (outlane_status.ok()) {
          outlane_status =
              IsAdcDeviateLaneDirection(&ref_line_info, ptr_trajectory_pb);
          injector_->is_adc_out_lane_ = !outlane_status.ok();
        } else {
          injector_->is_adc_out_lane_ = true;
        }
      }
    }

    if (injector_->is_adc_out_lane_ &&
        ptr_trajectory_pb->debug().planning_data().scenario().scenario_type() !=
            ScenarioConfig::RESCUE_TEB) {
      status = outlane_status;
      ptr_trajectory_pb->set_lane_status(ADCTrajectory::OUTSIDE_LANE);
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason("adc out road");
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateStopTrajectory(ptr_trajectory_pb);
      frame_teb_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
      JudgeArrivedStationImmediately(ptr_trajectory_pb);
      // pub if ADC is away from the road.
      RemoteDecider(ptr_trajectory_pb);
      const uint32_t n = frame_teb_->SequenceNum();
      AERROR << "is_blocked_by_dynamic_obj: "
             << frame_teb_->open_space_info().is_blocked_by_dynamic_obj();
      injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
      finish_status_ = true;
      calculation_result_ = false;
      return;
    }
  }

  FinalizePlanningResult(ptr_trajectory_pb, stitching_trajectory, replan_reason,
                         start_timestamp, start_system_timestamp, status);
  return;
}

bool OpenSpacePlanning::FinalizePlanningResult(
    ADCTrajectory* const ptr_trajectory_pb,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    const std::string& replan_reason, const double start_timestamp,
    const double start_system_timestamp, common::Status& status) {
  for (int i = 0; i < ptr_trajectory_pb->trajectory_point_size(); ++i) {
    if (i > kStepThreshold) {
      break;
    }
    if (ptr_trajectory_pb->trajectory_point(i).a() < kDecelerationThreshold) {
      if (ADCTrajectory::AD_PARKING != ptr_trajectory_pb->ad_behavior() &&
          ADCTrajectory::AD_REVERSING != ptr_trajectory_pb->ad_behavior()) {
        ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_BRAKING);
      }
      break;
    }
  }
  ptr_trajectory_pb->set_overtake_state(frame_teb_->GetOvertakeReportState());
  // request remote control if traffic light is not ok.
  for (auto& ref_line_info : *frame_teb_->mutable_reference_line_info()) {
    if (ref_line_info.TrafficLightRequest()) {
      request_remote_for_traffic_light_ = true;
      break;
    }
    if (ref_line_info.LaneBorrowFailRequest()) {
      lane_borrow_failed_ = true;
      AERROR << "Lane borrow failed, need remote request.";
      break;
    }
  }

  auto mutable_mixed_traffic = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_mixed_traffic();
  bool history_mixed_traffic = mutable_mixed_traffic->history_mixed_traffic();
  double time_diff =
      Clock::NowInSeconds() - mutable_mixed_traffic->keep_timestamp();
  // hold on 10 sec.
  if (history_mixed_traffic && time_diff <= kFreezeTime &&
      Frame::MixedTrafficType::UNKNOWN != frame_teb_->mixed_traffic_type_) {
    is_mixed_traffic_ = true;
    mutable_mixed_traffic->set_history_mixed_traffic(is_mixed_traffic_);
    ptr_trajectory_pb->set_is_mixed_traffic_scenario(is_mixed_traffic_);
  } else {
    // has many Non-motorized vehicle in the current lane or  has retrograde
    // obstacle in the non motor vehicle lane.
    if (frame_teb_->mixed_traffic_type_ ==
        Frame::MixedTrafficType::DYNAMIC_OBSTACLE_MEETED) {
      is_mixed_traffic_ = true;
      mutable_mixed_traffic->set_history_mixed_traffic(is_mixed_traffic_);
      ptr_trajectory_pb->set_is_mixed_traffic_scenario(is_mixed_traffic_);
      mutable_mixed_traffic->set_keep_timestamp(Clock::NowInSeconds());
    } else {
      is_mixed_traffic_ = false;
      mutable_mixed_traffic->set_history_mixed_traffic(is_mixed_traffic_);
      ptr_trajectory_pb->set_is_mixed_traffic_scenario(is_mixed_traffic_);
    }
  }
  if (FLAGS_enable_enter_mixed_flow_mode && FLAGS_enable_use_radical_decision &&
      is_mixed_traffic_ &&
      injector_->vehicle_state()->linear_velocity() < kMaxMixedFlowSpeed) {
    injector_->is_can_enter_mixed_flow_ = true;
  } else {
    injector_->is_can_enter_mixed_flow_ = false;
  }

  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000;
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
  if (!status.ok()) {
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  } else {
    is_planning_error_ = false;
  }

  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

  if (frame_teb_->open_space_info().is_on_open_space_trajectory() || true) {
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ptr_trajectory_pb->set_trajectory_scenario(ADCTrajectory::OPENSPACE);
    frame_teb_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    ptr_trajectory_pb->set_trajectory_scenario(ADCTrajectory::LANEFOLLOW);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_teb_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    // True to enable planning smoother among different planning cycles.
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_teb_history(),
                                frame_teb_.get(), ptr_trajectory_pb);
    }
  }

  // reference line recovery only one frame
  bool complete_dead_end =
      frame_teb_.get()->open_space_info().destination_reached();
  if (complete_dead_end) {
    if (reference_line_provider_->IsStop()) {
      reference_line_provider_->Start();
    }
    AERROR << "complete_dead_end reuse reference line";
    wait_flag_ = false;
  }
  JudgeReachTargetPoint(ptr_trajectory_pb);
  // RemoteDecider: planning_error,traffic_light,near_traffic_light_stop_line.
  RemoteDecider(ptr_trajectory_pb);
  if (canbus::Chassis::GEAR_REVERSE == ptr_trajectory_pb->gear() &&
      ADCTrajectory::AD_PARKING != ptr_trajectory_pb->ad_behavior()) {
    ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_REVERSING);
  }
  const uint32_t n = frame_teb_->SequenceNum();
  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);
    AERROR << "is_blocked_by_dynamic_obj: "
           << frame_teb_->open_space_info().is_blocked_by_dynamic_obj();
    injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
  }
  finish_status_ = true;
  calculation_result_ = true;
  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    adc_trajectory_pb_thread_.CopyFrom(*ptr_trajectory_pb);
  }
  return true;
}

bool OpenSpacePlanning::BuildFrameAndTrafficProcess(
    ADCTrajectory* const ptr_trajectory_pb,
    const common::VehicleState& vehicle_state,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    uint32_t frame_num, double start_timestamp) {
  Status status =
      InitTEBFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_teb_->obstacles());
  }

  if (FLAGS_enable_openspace_record_debug) {
    frame_teb_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }

  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {
    if (FLAGS_publish_estop) {
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());

      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());

      GenerateStopTrajectory(ptr_trajectory_pb);
    }

    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

    FillPlanningPb(start_timestamp, ptr_trajectory_pb);

    frame_teb_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);

    JudgeArrivedStationImmediately(ptr_trajectory_pb);
    RemoteDecider(ptr_trajectory_pb);

    injector_->ResetTaskFailureInfo();
    injector_->frame_teb_history()->Add(frame_teb_->SequenceNum(),
                                        std::move(frame_teb_));

    finish_status_ = true;
    calculation_result_ = false;
    return false;
  }

  std::vector<std::pair<std::string, double>> corrected_obstacles;

  for (auto& ref_line_info : *frame_teb_->mutable_reference_line_info()) {
    if (FLAGS_enable_correct_obstacle_speed &&
        !ref_line_info.IsChangeLanePath()) {
      FindUnreasonableSpeedObstacles(&ref_line_info, &corrected_obstacles);
    }
  }

  for (auto& ref_line_info : *frame_teb_->mutable_reference_line_info()) {
    for (auto& corrected_obstacle : corrected_obstacles) {
      auto* mutable_obstacle =
          ref_line_info.path_decision()->Find(corrected_obstacle.first);

      if (mutable_obstacle != nullptr) {
        mutable_obstacle->SetIsModifiedVelocity(true);
        mutable_obstacle->SetModifiedVelocity(corrected_obstacle.second);
      }
    }

    AddModifiedObstacle(&ref_line_info, corrected_obstacles);
  }

  for (auto& ref_line_info : *frame_teb_->mutable_reference_line_info()) {
    /**
     * After the ADC stops, when the speed of the obstacle ahead is less than a
     * threshold value and the distance between the ADC and the obstacle is less
     * than another threshold value, the ADC does not start.
     */
    // TODO(he zhiguo): This causes integration tests to fail.
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);

    auto traffic_status =
        traffic_decider.Execute(frame_teb_.get(), &ref_line_info, injector_);

    if (!ref_line_info.IsChangeLanePath()) {
      IsAdcInCommonJunction(&ref_line_info);
    }

    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
    }

    if (ref_line_info.IsNearTrafficLightStopLine()) {
      is_near_traffic_light_stop_line_ = true;
    }
  }

  return true;
}

bool OpenSpacePlanning::PrepareAndUpdateVehicleState(
    const LocalView& local_view, ADCTrajectory* const ptr_trajectory_pb,
    VehicleState* vehicle_state,
    std::vector<common::TrajectoryPoint>* stitching_trajectory,
    std::string* replan_reason, double* start_timestamp,
    double* start_system_timestamp, uint32_t* frame_num) {
  static bool failed_to_update_reference_line = false;
  static int stopped_count = 0;

  local_view_ = local_view;
  request_remote_for_traffic_light_ = false;
  is_near_traffic_light_stop_line_ = false;
  is_planning_error_ = true;
  lane_borrow_failed_ = false;
  is_sim_control_ = false;
  is_auto_state_ = false;

  if (local_view_.chassis.get()->header().module_name() == "SimControl") {
    is_sim_control_ = true;
  }
  if (local_view_.chassis.get()->driving_mode() ==
      Chassis::COMPLETE_AUTO_DRIVE) {
    is_auto_state_ = true;
  }

  if (std::fabs(local_view_.chassis->speed_mps()) < KStopSpeedThreshold) {
    stopped_count++;
    if (stopped_count >= KStopEnsureCount) {
      is_stopped_ = true;
    }
  } else {
    is_stopped_ = false;
    stopped_count = 0;
  }

  *start_timestamp = Clock::NowInSeconds();
  start_timestamp_ = *start_timestamp;

  *start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();

  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  const VehicleState& vehicle_state_old =
      injector_->vehicle_state()->vehicle_state();

  double diff_time = injector_->DiffTimeStamp(Clock::NowInSeconds());
  double adc_speed = vehicle_state_old.linear_velocity();
  double adc_acc = vehicle_state_old.linear_acceleration();
  bool is_forward = adc_speed >= 0.0;

  injector_->UpdateAdcSpeedStatus(adc_speed);
  injector_->UpdateDrivingDistance(vehicle_state_old.x(),
                                   vehicle_state_old.y(),
                                   is_forward);

  double adc_distance = injector_->GetDrivingDistance();

  injector_->GetVehicleStateKalman()->EstimateWithMeasurement(
      diff_time, adc_distance, adc_speed, adc_acc);

  std::array<double, 3UL> state_estimate;
  injector_->GetVehicleStateKalman()->GetStateEstimate(&state_estimate);

  double update_acc = adc_speed < kStopedSpeed ? 0.0 : state_estimate[2];
  injector_->vehicle_state()->UpdateAcceleration(update_acc);

  *vehicle_state = injector_->vehicle_state()->vehicle_state();
  vehicle_state_fallback_ = *vehicle_state;

  if (!status.ok() || !util::IsVehicleStateValid(*vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed or the vehicle state is out dated.";
    AERROR << msg;

    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()->set_reason(msg);

    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

    FillPlanningPb(*start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    RemoteDecider(ptr_trajectory_pb);

    injector_->ResetTaskFailureInfo();
    finish_status_ = true;
    calculation_result_ = false;
    return false;
  }

  double latency_time = *start_timestamp - vehicle_state->timestamp();
  if (latency_time > 0.0 &&
      latency_time < FLAGS_message_latency_threshold) {
    *vehicle_state = AlignTimeStamp(*vehicle_state, *start_timestamp);
  }

  failed_to_update_reference_line =
      (!reference_line_provider_->UpdatedReferenceLine());

  if (failed_to_update_reference_line) {
    const std::string msg =
        "Failed to update reference line after rerouting.";
    AERROR << msg;

    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()->set_reason(msg);

    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

    FillPlanningPb(*start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    JudgeArrivedStationImmediately(ptr_trajectory_pb);
    RemoteDecider(ptr_trajectory_pb);

    injector_->ResetTaskFailureInfo();
    finish_status_ = true;
    calculation_result_ = false;
    return false;
  }

  reference_line_provider_->UpdateVehicleState(*vehicle_state);

  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);

  const bool is_replan =
      !TrajectoryStitcher::ComputeStitchingTrajectory(
          *vehicle_state,
          *start_timestamp,
          planning_cycle_time,
          FLAGS_trajectory_stitching_preserved_length,
          true,
          last_publishable_trajectory_.get(),
          stitching_trajectory,
          replan_reason,
          injector_);

  injector_->SetReplanState(is_replan);
  injector_->ResetReinitStartPoint();

  injector_->ego_info()->Update(stitching_trajectory->back(),
                                *vehicle_state);

  *frame_num = static_cast<uint32_t>(seq_num_teb_++);

  return true;
}

// check and set whether the route destination is reached
bool OpenSpacePlanning::JudgeReachTargetPoint(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  destination->set_has_reached_destination(false);
  destination->set_has_reached_station(false);
  if (injector_->pullover_using_) {
    AINFO << "JudgeFinished: OpenSpace";
    CheckOpenSpaceReachTargetPoint(ptr_trajectory_pb);
    if (injector_->pullover_finished) {
      GenerateStopTrajectory(ptr_trajectory_pb);
    }
  } else if (
      !FLAGS_enable_use_pullover_mode ||
      (FLAGS_enable_use_pullover_mode && !injector_->pullover_using_ &&
       (!FLAGS_enable_TEB_thread /*|| !injector_->is_in_play_street*/))) {
    AINFO << "JudgeFinished: PRP";
    CheckPublicRoadReachTargetPoint(ptr_trajectory_pb);
  }
  // set reached station if need arrived_station_immediately
  if (destination->arrived_station_immediately()) {
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_station(true);
    AERROR << "arrived_station_immediately: GenerateStopTrajectory.";
    GenerateStopTrajectory(ptr_trajectory_pb);
  }

  // clear routing header
  if (destination->has_reached_station()) {
    if (!has_reached_station_) {
      has_reached_station_ = true;
      reached_station_time_ = Clock::NowInSeconds();
    }
  } else {
    has_reached_station_ = false;
  }

  if (has_reached_station_ &&
      Clock::NowInSeconds() - reached_station_time_ >= kReachedStationTime) {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);

    local_view_.routing->clear_header();
    has_reached_station_ = false;
  }
  return destination->has_reached_station();
}

bool OpenSpacePlanning::JudgeArrivedStationImmediately(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  destination->set_has_reached_destination(false);
  destination->set_has_reached_station(false);

  // set reached station if need arrived_station_immediately
  if (destination->arrived_station_immediately()) {
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_station(true);
    AERROR << "arrived_station_immediately: GenerateStopTrajectory.";
  }

  // clear routing header
  if (destination->has_reached_station()) {
    if (!has_reached_station_) {
      has_reached_station_ = true;
      reached_station_time_ = Clock::NowInSeconds();
    }
  } else {
    has_reached_station_ = false;
  }

  if (has_reached_station_ &&
      Clock::NowInSeconds() - reached_station_time_ >= kReachedStationTime) {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);

    local_view_.routing->clear_header();
    has_reached_station_ = false;
  }

  return destination->has_reached_station();
}

void OpenSpacePlanning::CheckOpenSpaceReachTargetPoint(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  destination->set_has_reached_destination(true);
  ptr_trajectory_pb->set_has_reached_destination(true);

  if (!injector_->pullover_finished) {
    has_pullover_finished_flag_ = false;
    pullover_finished_start_time_ = Clock::NowInSeconds();
  }
  // 2. check stop
  if (injector_->pullover_finished && !has_pullover_finished_flag_) {
    has_pullover_finished_flag_ = true;
    pullover_finished_start_time_ = Clock::NowInSeconds();
    injector_->set_enable_rescue_pullover(false);
  }

  bool pullover_reached_vaild = false;
  if ((Clock::NowInSeconds() - pullover_finished_start_time_) >
          kReachedStationTime &&
      has_pullover_finished_flag_) {
    pullover_reached_vaild = true;
  }

  AINFO << "Pullover_Finished: " << injector_->pullover_finished
        << " | Reached_Vaild: " << pullover_reached_vaild;

  if (injector_->pullover_finished && pullover_reached_vaild) {
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_station(true);
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);

    AERROR << "OpenSpace_Finished!";
  }
  return;
}

void OpenSpacePlanning::CheckPublicRoadReachTargetPoint(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  // 1. check distance
  bool near_target_distance = false;
  common::PointENU target_point;
  size_t waypoint_num = local_view_.routing->routing_request().waypoint_size();
  if (waypoint_num > 0) {
    target_point = frame_teb_->local_view()
                       .routing->routing_request()
                       .waypoint()
                       .at(waypoint_num - 1)
                       .pose();
    const common::VehicleState& car_position = frame_teb_->vehicle_state();
    double distance_to_vehicle =
        std::hypot(car_position.x() - target_point.x(),
                   car_position.y() - target_point.y());
    near_target_distance =
        FLAGS_enable_use_pullover_mode
            ? injector_->is_reach_goal_
            : (distance_to_vehicle < FLAGS_threshold_distance_for_destination);
    ADEBUG << "distance_to_vehicle: " << distance_to_vehicle;
    ADEBUG << "FLAGS_threshold_distance_for_destination: "
           << FLAGS_threshold_distance_for_destination;
    ADEBUG << "near_target_distance: " << near_target_distance;
  }

  // 2. check angle
  bool near_target_angle = false;
  if (frame_teb_->DriveReferenceLineInfo() && waypoint_num > 0) {
    const ReferenceLine& reference_line =
        frame_teb_->DriveReferenceLineInfo()->reference_line();
    ReferencePoint point =
        reference_line.GetReferencePoint(target_point.x(), target_point.y());
    const auto& target_heading = point.heading();
    const common::VehicleState& car_position = frame_teb_->vehicle_state();
    const auto& adc_heading = car_position.heading();
    near_target_angle = std::fabs(century::common::math::AngleDiff(
                            target_heading, adc_heading)) < 0.5 * M_PI_4;
    ADEBUG << "target_heading: " << target_heading;
    ADEBUG << "adc_heading: " << adc_heading;
    ADEBUG << "near_target_angle: " << near_target_angle;
  }

  // 3. check stop
  ADEBUG << "is_stopped_: " << is_stopped_;
  if (near_target_distance && near_target_angle && is_stopped_) {
    destination->set_has_reached_destination(true);
    AERROR << "lane follow has_reached_destination.";
  }

  bool reached_destination = destination->has_reached_destination();
  AERROR << "reached_destination: " << reached_destination;
  ptr_trajectory_pb->set_has_reached_destination(reached_destination);

  // Arrival routing site (also requires a condition, whether this route is an
  // final route)
  if (reached_destination &&
      local_view_.routing->routing_request().final_route()) {
    ptr_trajectory_pb->set_has_reached_station(reached_destination);
    destination->set_has_reached_station(reached_destination);
    AERROR << "reached station: " << reached_destination;
  }
}

void OpenSpacePlanning::ExportReferenceLineDebug(
    planning_internal::Debug* debug) {
  if (!FLAGS_enable_openspace_record_debug) {
    return;
  }
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  for (auto& reference_line_info : *frame_teb_->mutable_reference_line_info()) {
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    // store kappa and dkappa for performance evaluation
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;
    double dkappa_rms = 0.0;
    double kappa_max_abs = std::numeric_limits<double>::lowest();
    double dkappa_max_abs = std::numeric_limits<double>::lowest();
    for (const auto& reference_point : reference_points) {
      double kappa_sq = reference_point.kappa() * reference_point.kappa();
      double dkappa_sq = reference_point.dkappa() * reference_point.dkappa();
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = std::fmax(kappa_max_abs, kappa_sq);
      dkappa_max_abs = std::fmax(dkappa_max_abs, dkappa_sq);
    }
    double reference_points_size = static_cast<double>(reference_points.size());
    if (0.0 != reference_points_size) {
      rl_debug->set_kappa_rms(std::sqrt(kappa_rms / reference_points_size));
      rl_debug->set_dkappa_rms(std::sqrt(dkappa_rms / reference_points_size));
    }
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();

    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();
    double average_offset = 0.0;
    double sample_count = 0.0;
    for (double s = 0.0; s < reference_line_info.reference_line().Length();
         s += kReferenceLineSampleStep) {
      double left_width = reference_line_path.GetLaneLeftWidth(s);
      double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::fabs(left_width - right_width);
      is_off_road = is_off_road ? is_off_road
                                : (left_width < adc_half_width ||
                                   right_width < adc_half_width);
      minimum_boundary = std::fmin(left_width, minimum_boundary);
      minimum_boundary = std::fmin(right_width, minimum_boundary);
      ++sample_count;
    }
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(
        average_offset / std::fmax(sample_count, FLAGS_numerical_epsilon));
  }
}

Status OpenSpacePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_openspace_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_teb_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_teb_->mutable_open_space_info()->sync_debug_instance();
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_teb_.get(),
                               ptr_trajectory_pb);

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  if (frame_teb_->open_space_info().is_on_open_space_trajectory()) {
    SetOpenSpaceTrajectory(ptr_trajectory_pb);
  } else {
    if (!frame_teb_->FindDriveReferenceLineInfo()) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      Status ret_status(common::PLANNING_ERROR, msg);
      ret_status.merge_error_message(status);
      return ret_status;
    }
    SetPublicRoadTrajectory(current_time_stamp, stitching_trajectory,
                            ptr_trajectory_pb);
  }
  return status;
}

void OpenSpacePlanning::SetOpenSpaceTrajectory(
    ADCTrajectory* const ptr_trajectory_pb) {
  frame_teb_->mutable_open_space_info()->sync_debug_instance();
  const auto& publishable_trajectory =
      frame_teb_->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame_teb_->open_space_info().publishable_trajectory_data().second;
  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

  auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

  // enable start auto from open_space planner.
  if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
      Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
    engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
    engage_advice->set_reason(
        "Ready to engage when staring with OPEN_SPACE_PLANNER");
  } else {
    engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason("Keep engage while in parking");
  }
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_parking()
      ->set_status(MainParking::IN_PARKING);

  // added latency_stats for open space planner
  ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
      frame_teb_->open_space_info().latency_stats());

  if (FLAGS_enable_openspace_record_debug) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    frame_teb_->mutable_open_space_info()->RecordDebug(ptr_debug);
    ADEBUG << "Open space debug information added!";
    ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                         ptr_debug);
  }
}

void OpenSpacePlanning::SetOpenSpaceTrajectoryThread(
    ADCTrajectory* const ptr_trajectory_pb) {
  frame_teb_->mutable_open_space_info()->sync_debug_instance();
  // frame_teb_->mutable_open_space_info()->sync_debug_instance();

  const auto& publishable_trajectory =
      frame_teb_->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame_teb_->open_space_info().publishable_trajectory_data().second;
  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

  auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

  // enable start auto from open_space planner.
  if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
      Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
    engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
    engage_advice->set_reason(
        "Ready to engage when staring with OPEN_SPACE_PLANNER");
  } else {
    engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason("Keep engage while in parking");
  }
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_parking()
      ->set_status(MainParking::IN_PARKING);

  // added latency_stats for open space planner
  ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
      frame_teb_->open_space_info().latency_stats());

  if (FLAGS_enable_openspace_record_debug) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    frame_teb_->mutable_open_space_info()->RecordDebug(ptr_debug);
    // frame_teb_->mutable_open_space_info()->RecordDebug(ptr_debug);

    ADEBUG << "Open space debug information added!";
    ExportOpenSpaceChartThread(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                               ptr_debug);
  }
}

void OpenSpacePlanning::SetPublicRoadTrajectory(
    const double current_time_stamp,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  const auto* best_ref_info = frame_teb_->FindDriveReferenceLineInfo();
  const auto* target_ref_info = frame_teb_->FindTargetReferenceLineInfo();

  // Store current frame stitched path for possible speed fallback in next
  // frames
  DiscretizedPath current_frame_planned_path;
  for (const auto& trajectory_point : stitching_trajectory) {
    current_frame_planned_path.emplace_back(
        std::move(trajectory_point.path_point()));
  }
  const auto& best_ref_path = best_ref_info->path_data().discretized_path();
  ADEBUG << "best_ref_path size:" << best_ref_path.size();
  CHECK_GT(best_ref_path.size(), 0UL);
  const std::string StBoundsTaskName =
      TaskConfig::TaskType_Name(TaskConfig::ST_BOUNDS_DECIDER);
  auto ref_path_begin_iterator = best_ref_path.begin();
  auto ref_path_end_iterator = best_ref_path.end();
  std::advance(ref_path_begin_iterator, 1);
  ADEBUG << "TaskFailureName: " << injector_->GetTaskFailureName();
  if (injector_->GetTaskFailureCount() > kMaxTaskFailureCount &&
      StBoundsTaskName == injector_->GetTaskFailureName()) {
    ref_path_end_iterator = ref_path_begin_iterator;
    std::advance(ref_path_end_iterator,
                 std::min(best_ref_path.size(), kMaxFailurePathPointNum) - 1);
  }
  std::copy(ref_path_begin_iterator, ref_path_end_iterator,
            std::back_inserter(current_frame_planned_path));
  ADEBUG << "current_frame_planned_path size:"
         << current_frame_planned_path.size();
  frame_teb_->set_current_frame_planned_path(current_frame_planned_path);
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  ptr_debug->MergeFrom(best_ref_info->debug());
  if (FLAGS_export_chart) {
    ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
  } else {
    ExportReferenceLineDebug(ptr_debug);
    // Export additional ST-chart for failed lane-change speed planning
    const auto* failed_ref_info = frame_teb_->FindFailedReferenceLineInfo();
    if (failed_ref_info) {
      ExportFailedLaneChangeSTChart(failed_ref_info->debug(), ptr_debug);
    }
  }

  UpdateTrajectory(best_ref_info, target_ref_info, ptr_trajectory_pb);

  // Add debug information.
  AddPlanningRecordDebug(best_ref_info, ptr_trajectory_pb);

  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);
    last_publishable_trajectory_.reset(new PublishableTrajectory(
        current_time_stamp, best_ref_info->trajectory()));

    ADEBUG << "current_time_stamp: " << current_time_stamp;

    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));

    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  }
  best_ref_info->ExportEngageAdvice(ptr_trajectory_pb->mutable_engage_advice(),
                                    injector_->planning_context());

  // clac p2p distance
  auto* p2p_distance = ptr_trajectory_pb->mutable_p2p_distance();
  p2p_distance->set_total(local_view_.routing->measurement().distance());
  p2p_distance->set_remain(CalcRemainDistance(best_ref_info));

  if (is_dangerous_road_) {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::DANGEROUS_ROAD);
  } else if (best_ref_info->NoGreenInJunction()) {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::NO_GREEN_IN_JUNCTION);
  } else {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::WARNING_NONE);
  }
}

void OpenSpacePlanning::AddPlanningRecordDebug(
    const ReferenceLineInfo* best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (FLAGS_enable_openspace_record_debug) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points =
        best_ref_info->reference_line().reference_points();
    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool empty_path = true;
    for (const auto& reference_point : reference_points) {
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
      if (empty_path) {
        path_point->set_s(0.0);
        empty_path = false;
      } else {
        double dx = reference_point.x() - prev_x;
        double dy = reference_point.y() - prev_y;
        s += std::hypot(dx, dy);
        path_point->set_s(s);
      }
      prev_x = reference_point.x();
      prev_y = reference_point.y();
    }
  }
}

void OpenSpacePlanning::UpdateTrajectory(
    const ReferenceLineInfo* best_ref_info,
    const ReferenceLineInfo* target_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (best_ref_info->GetEStopStatus().is_estop) {
    EStop* estop = ptr_trajectory_pb->mutable_estop();
    estop->set_is_estop(best_ref_info->GetEStopStatus().is_estop);
    estop->set_reason(best_ref_info->GetEStopStatus().reason);
  }
  ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_ref_info->latency_stats());
  // set right of way status
  ptr_trajectory_pb->set_right_of_way_status(
      best_ref_info->GetRightOfWayStatus());

  for (const auto& id : best_ref_info->TargetLaneId()) {
    ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
  }

  for (const auto& id : target_ref_info->TargetLaneId()) {
    ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
  }

  ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

  if (FLAGS_enable_rss_info) {
    *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
  }

  best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                injector_->planning_context());

  const auto turn_signal =
      ptr_trajectory_pb->decision().vehicle_signal().turn_signal();
  injector_->is_turn_right_ = common::VehicleSignal::TURN_RIGHT == turn_signal;

  if (ADCTrajectory::AD_NONE == ptr_trajectory_pb->ad_behavior() ||
      ADCTrajectory::AD_TURNING_LEFT == ptr_trajectory_pb->ad_behavior() ||
      ADCTrajectory::AD_TURNING_RIGHT == ptr_trajectory_pb->ad_behavior()) {
    if (common::VehicleSignal::TURN_LEFT == turn_signal) {
      ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_TURNING_LEFT);
    } else if (common::VehicleSignal::TURN_RIGHT == turn_signal) {
      ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_TURNING_RIGHT);
    }
  }

  ptr_trajectory_pb->set_map_area_type(best_ref_info->GetLaneType());
}

double OpenSpacePlanning::CalcRemainDistance(
    const ReferenceLineInfo* reference_line_info) {
  auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = reference_line_info->LocateLaneInfo(s);

  // check dangerous road
  is_dangerous_road_ = false;
  if (nullptr != locate_lane && locate_lane->lane().has_is_dangerous_road() &&
      locate_lane->lane().is_dangerous_road()) {
    is_dangerous_road_ = true;
  }

  auto& ego_pose = injector_->vehicle_state()->pose().position();
  auto& goal_pose =
      local_view_.routing->routing_request().waypoint().rbegin()->pose();
  auto& reference_line = reference_line_info->reference_line();

  if (nullptr == locate_lane) {
    return common::util::DistanceXY(ego_pose, goal_pose);
  }

  double remain_distance = 0.0;
  bool find_ego_passage = false;
  std::string ego_lane_id = locate_lane->lane().id().id();
  for (int i = 0; i < local_view_.routing->road_size(); ++i) {
    if (find_ego_passage) {
      for (auto& lane_segment :
           local_view_.routing->road(i).passage(0).segment()) {
        remain_distance += (lane_segment.end_s() - lane_segment.start_s());
      }
    } else {
      if (IsEgoPassage(ego_lane_id, local_view_.routing->road(i))) {
        // calc ego passage distance(goal & not  goal)
        if (i == (local_view_.routing->road_size() - 1)) {
          SLPoint goal_sl;
          if (reference_line.XYToSL(goal_pose, &goal_sl)) {
            return goal_sl.s() - adc_sl_boundary.end_s();
          } else {
            return common::util::DistanceXY(ego_pose, goal_pose);
          }
        } else {
          const auto& lane_id =
              local_view_.routing->road(i).passage(0).segment().rbegin()->id();
          auto lane_info_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
          if (nullptr == lane_info_ptr) {
            for (auto& lane_segment :
                 local_view_.routing->road(i).passage(0).segment()) {
              remain_distance +=
                  (lane_segment.end_s() - lane_segment.start_s());
            }
          } else {
            const auto& road_end_pose = *(lane_info_ptr->points().rbegin());
            SLPoint road_end_sl;
            if (reference_line.XYToSL(road_end_pose, &road_end_sl)) {
              remain_distance += road_end_sl.s() - adc_sl_boundary.end_s();
            } else {
              remain_distance += common::util::DistanceXY(
                  ego_pose, *lane_info_ptr->points().rbegin());
            }
          }
        }
        find_ego_passage = true;
      }
    }
  }

  if (common::util::IsFloatEqual(remain_distance, 0.0)) {
    AERROR << "the remain_distance is zero!";
    remain_distance = common::util::DistanceXY(ego_pose, goal_pose);
  }

  return remain_distance;
}

bool OpenSpacePlanning::IsEgoPassage(const std::string& ego_lane_id,
                                     const routing::RoadSegment& road_segment) {
  for (int i = 0; i < road_segment.passage_size(); ++i) {
    for (int j = 0; j < road_segment.passage(i).segment_size(); ++j) {
      if (0 == std::strcmp(ego_lane_id.c_str(),
                           road_segment.passage(i).segment(j).id().c_str())) {
        return true;
      }
    }
  }
  return false;
}

bool OpenSpacePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  if (!config.standard_planning_config().has_planner_public_road_config()) {
    return false;
  }
  // TODO(All): check other config params
  return true;
}

void PopulateChartOptions(double x_min, double x_max, std::string x_label,
                          double y_min, double y_max, std::string y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddSTGraph(const STGraphDebug& st_graph, Chart* chart) {
  if (st_graph.name() == "SPEED_HEURISTIC_OPTIMIZER") {
    chart->set_title("DP Planning");
  } else if (st_graph.name() == "PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER") {
    chart->set_title("NLP Planning");
  } else if (st_graph.name() == "PIECEWISE_JERK_SPEED_OPTIMIZER") {
    chart->set_title("QP Planning");
  } else {
    chart->set_title("Planning S-T Graph");
  }
  PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 60.0, "s (meter)",
                       false, chart);

  for (const auto& boundary : st_graph.boundary()) {
    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("V-S Graph");

  double max_s = 0.0;
  double max_v = 0.0;
  ADEBUG << "chart speed_plans size: " << speed_plans.size();
  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
      max_s = std::max(max_s, point.s());
      max_v = std::max(max_v, point.v());
    }

    // Set chartJS's dataset properties
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() ==
        TaskConfig::TaskType_Name(TaskConfig::SPEED_HEURISTIC_OPTIMIZER)) {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() ==
               TaskConfig::TaskType_Name(
                   TaskConfig::PIECEWISE_JERK_SPEED_OPTIMIZER)) {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    } else if (speed_plan.name() ==
               TaskConfig::TaskType_Name(
                   TaskConfig::PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER)) {
      (*properties)["color"] = "\"rgba(235, 155, 155, 1)\"";
    } else if (speed_plan.name() == "limit") {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }
  }
  ADEBUG << "chart max_s: " << max_s;
  ADEBUG << "chart max_v: " << max_v;

  if (max_s < kSpeedPlanChartEighthMaxS && max_v < kSpeedPlanChartEighthMaxV) {
    PopulateChartOptions(0.0, kSpeedPlanChartEighthMaxS, "s (meter)", 0.0,
                         kSpeedPlanChartEighthMaxV, "v (m/s)", true, chart);
    ADEBUG << "set Eighth chart : s(" << kSpeedPlanChartEighthMaxS << "), v("
           << kSpeedPlanChartEighthMaxV << ").";
  } else if (max_s < kSpeedPlanChartQuarterMaxS &&
             max_v < kSpeedPlanChartQuarterMaxV) {
    PopulateChartOptions(0.0, kSpeedPlanChartQuarterMaxS, "s (meter)", 0.0,
                         kSpeedPlanChartQuarterMaxV, "v (m/s)", true, chart);
    ADEBUG << "set Quarter chart : s(" << kSpeedPlanChartQuarterMaxS << "), v("
           << kSpeedPlanChartQuarterMaxV << ").";
  } else if (max_s < kSpeedPlanChartHalfMaxS &&
             max_v < kSpeedPlanChartHalfMaxV) {
    PopulateChartOptions(0.0, kSpeedPlanChartHalfMaxS, "s (meter)", 0.0,
                         kSpeedPlanChartHalfMaxV, "v (m/s)", true, chart);
    ADEBUG << "set Half chart : s(" << kSpeedPlanChartHalfMaxS << "), v("
           << kSpeedPlanChartHalfMaxV << ").";
  } else {
    PopulateChartOptions(0.0, kSpeedPlanChartMaxS, "s (meter)", 0.0,
                         kSpeedPlanChartMaxV, "v (m/s)", true, chart);
    ADEBUG << "set Full chart : s(" << kSpeedPlanChartMaxS << "), v("
           << kSpeedPlanChartMaxV << ").";
  }
}

void OpenSpacePlanning::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

void OpenSpacePlanning::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }
  AddSpeedPlan(src_data.speed_plan(), dst_data->add_chart());
}

void OpenSpacePlanning::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_openspace_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    AddPartitionedTrajectory(debug_info, debug_chart);
    AddStitchSpeedProfile(debug_chart);
    AddPublishedSpeed(trajectory_pb, debug_chart);
    AddPublishedAcceleration(trajectory_pb, debug_chart);
    // AddFallbackTrajectory(debug_info, debug_chart);
  }
}
void OpenSpacePlanning::ExportOpenSpaceChartThread(
    const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_openspace_record_debug) {
    AddOpenSpaceOptimizerResultThread(debug_info, debug_chart);
    AddPartitionedTrajectoryThread(debug_info, debug_chart);
    AddStitchSpeedProfileThread(debug_chart);
    AddPublishedSpeedThread(trajectory_pb, debug_chart);
    AddPublishedAccelerationThread(trajectory_pb, debug_chart);
    // AddFallbackTrajectory(debug_info, debug_chart);
  }
}

void OpenSpacePlanning::PathPointNormalizing(double rotate_angle,
                                             const Vec2d& translate_origin,
                                             double* x, double* y,
                                             double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void OpenSpacePlanning::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  if (open_space_debug.xy_boundary().size() < 4) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Trajectory Optimizer Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  // Set Same window size for x-Axis and y-Axis.
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(1.0);

  AddOpenSpaceObstacleDebugInfo(debug_info, debug_chart);
  AddOpenSpaceSmoothLineDebugInfo(debug_info, debug_chart);
  AddOpenSpaceWarmStartDebugInfo(debug_info, debug_chart);
  AddOpenSpaceStartEndPointDebugInfo(debug_info, debug_chart);
  AddOpenSpaceROILineDebugInfo(debug_info, debug_chart);
  AddOpenSpaceEndHeadingLineDebugInfo(debug_info, debug_chart);
}
void OpenSpacePlanning::AddOpenSpaceOptimizerResultThread(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  if (open_space_debug.xy_boundary().size() < 4) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Trajectory Optimizer Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  // Set Same window size for x-Axis and y-Axis.
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(1.0);

  AddOpenSpaceObstacleDebugInfo(debug_info, debug_chart);
  AddOpenSpaceSmoothLineDebugInfo(debug_info, debug_chart);
  AddOpenSpaceWarmStartDebugInfo(debug_info, debug_chart);
  AddOpenSpaceStartEndPointDebugInfo(debug_info, debug_chart);
  AddOpenSpaceROILineDebugInfo(debug_info, debug_chart);
  AddOpenSpaceEndHeadingLineDebugInfo(debug_info, debug_chart);
}

void OpenSpacePlanning::AddOpenSpaceObstacleDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    // output too much
    obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
    obstacle_index += 1;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }
}

void OpenSpacePlanning::AddOpenSpaceSmoothLineDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto smoothed_trajectory = debug_info.planning_data()
                                 .open_space()
                                 .astar_optimization_info()
                                 .smoothed_trajectory();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("Smooth");
  for (const auto& point : smoothed_trajectory.vehicle_motion_point()) {
    const auto x = point.trajectory_point().path_point().x();
    const auto y = point.trajectory_point().path_point().y();
    // Draw vehicle trajectory points
    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(x);
    point_debug->set_y(y);
  }

  // Set chartJS's dataset properties
  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddOpenSpaceWarmStartDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto warm_start_trajectory = debug_info.planning_data()
                                   .open_space()
                                   .astar_optimization_info()
                                   .warm_start_trajectory();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("WarmStart");
  for (const auto& point : warm_start_trajectory.vehicle_motion_point()) {
    auto* point_debug = warm_start_line->add_point();
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  // Set chartJS's dataset properties
  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddOpenSpaceStartEndPointDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  // Show Start End Point
  auto* start_end_line = chart->add_line();
  start_end_line->set_label("StartEndPoint");
  auto* start_point_debug = start_end_line->add_point();
  start_point_debug->set_x(
      open_space_debug.hybrid_search_info().start_point().path_point().x());
  start_point_debug->set_y(
      open_space_debug.hybrid_search_info().start_point().path_point().y());
  auto* end_point_debug = start_end_line->add_point();
  end_point_debug->set_x(
      open_space_debug.hybrid_search_info().end_point().path_point().x());
  end_point_debug->set_y(
      open_space_debug.hybrid_search_info().end_point().path_point().y());

  // Set chartJS's dataset properties
  auto* start_end_properties = start_end_line->mutable_properties();
  (*start_end_properties)["borderWidth"] = "2";
  (*start_end_properties)["pointRadius"] = "5";
  (*start_end_properties)["lineTension"] = "0";
  (*start_end_properties)["fill"] = "false";
  (*start_end_properties)["showLine"] = "true";
  (*start_end_properties)["pointStyle"] = "crossRot";
  (*start_end_properties)["borderColor"] = "rgba(50, 250, 50, 1)";
}

void OpenSpacePlanning::AddOpenSpaceROILineDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  // Show ROI Point
  auto* roi_line = chart->add_line();
  roi_line->set_label("ROI Point");
  auto* left_down = roi_line->add_point();
  left_down->set_x(open_space_debug.xy_boundary(0));
  left_down->set_y(open_space_debug.xy_boundary(2));
  auto* right_down = roi_line->add_point();
  right_down->set_x(open_space_debug.xy_boundary(1));
  right_down->set_y(open_space_debug.xy_boundary(2));
  auto* right_up = roi_line->add_point();
  right_up->set_x(open_space_debug.xy_boundary(1));
  right_up->set_y(open_space_debug.xy_boundary(3));
  auto* left_up = roi_line->add_point();
  left_up->set_x(open_space_debug.xy_boundary(0));
  left_up->set_y(open_space_debug.xy_boundary(3));
  auto* left_down2 = roi_line->add_point();
  left_down2->set_x(open_space_debug.xy_boundary(0));
  left_down2->set_y(open_space_debug.xy_boundary(2));

  // Set chartJS's dataset properties
  auto* roi_line_properties = roi_line->mutable_properties();
  (*roi_line_properties)["borderWidth"] = "2";
  (*roi_line_properties)["pointRadius"] = "5";
  (*roi_line_properties)["lineTension"] = "0";
  (*roi_line_properties)["fill"] = "false";
  (*roi_line_properties)["showLine"] = "true";
  (*roi_line_properties)["pointStyle"] = "rectRounded";
  (*roi_line_properties)["borderColor"] = "rgba(250, 50, 50, 1)";
}

void OpenSpacePlanning::AddOpenSpaceEndHeadingLineDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  // add yaw of start and end pose
  double heading_length = 1.0;
  auto* end_heading_line = chart->add_line();
  end_heading_line->set_label("heading");
  auto* end_point_start_debug = end_heading_line->add_point();
  end_point_start_debug->set_x(
      open_space_debug.hybrid_search_info().end_point().path_point().x());
  end_point_start_debug->set_y(
      open_space_debug.hybrid_search_info().end_point().path_point().y());
  auto* end_point_heading_debug = end_heading_line->add_point();
  double heading_delta_x =
      heading_length * std::cos(open_space_debug.hybrid_search_info()
                                    .end_point()
                                    .path_point()
                                    .theta());
  double heading_delta_y =
      heading_length * std::sin(open_space_debug.hybrid_search_info()
                                    .end_point()
                                    .path_point()
                                    .theta());
  end_point_heading_debug->set_x(
      open_space_debug.hybrid_search_info().end_point().path_point().x() +
      heading_delta_x);
  end_point_heading_debug->set_y(
      open_space_debug.hybrid_search_info().end_point().path_point().y() +
      heading_delta_y);
  // Set chartJS's dataset properties
  auto* end_heading_line_properties = end_heading_line->mutable_properties();
  (*end_heading_line_properties)["borderWidth"] = "2";
  (*end_heading_line_properties)["pointRadius"] = "5";
  (*end_heading_line_properties)["lineTension"] = "0";
  (*end_heading_line_properties)["fill"] = "false";
  (*end_heading_line_properties)["showLine"] = "true";
  (*end_heading_line_properties)["pointStyle"] = "crossRot";
  (*end_heading_line_properties)["borderColor"] = "rgba(50, 250, 250, 1)";
}

void OpenSpacePlanning::AddPartitionedTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.hybrid_search_info().chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_teb_->vehicle_state();
  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned Trajectory");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  // Draw vehicle state
  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  // Draw the chosen trajectories
  DrawOpenSpaceChosenTrajectory(debug_info, debug_chart);

  // Draw trajectory stitching point (line with only one point)
  DrawOpenSpaceTrajectoryStitchPoint(debug_info, debug_chart);

  // Draw fallback trajectory compared with the partitioned and potential
  // collision_point (line with only one point)
  if (open_space_debug.hybrid_search_info().is_fallback_trajectory()) {
    DrawOpenSpaceFallbackTrajectory(debug_info, debug_chart);
  }
}

void OpenSpacePlanning::AddPartitionedTrajectoryThread(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.hybrid_search_info().chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_teb_->vehicle_state();
  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned Trajectory");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  // Draw vehicle state
  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  // Draw the chosen trajectories
  DrawOpenSpaceChosenTrajectory(debug_info, debug_chart);

  // Draw trajectory stitching point (line with only one point)
  DrawOpenSpaceTrajectoryStitchPoint(debug_info, debug_chart);

  // Draw fallback trajectory compared with the partitioned and potential
  // collision_point (line with only one point)
  if (open_space_debug.hybrid_search_info().is_fallback_trajectory()) {
    DrawOpenSpaceFallbackTrajectory(debug_info, debug_chart);
  }
}

void OpenSpacePlanning::DrawOpenSpaceChosenTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.hybrid_search_info().chosen_trajectory().trajectory();
  if (chosen_trajectories.empty()) {
    return;
  }
  const auto& chosen_trajectory = chosen_trajectories[0];
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_trajectory.trajectory_point()) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }
  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";

  // Draw partitioned trajectories
  size_t partitioned_trajectory_label = 0;
  for (const auto& partitioned_trajectory :
       open_space_debug.hybrid_search_info()
           .partitioned_trajectories()
           .trajectory()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_trajectory_label));
    ++partitioned_trajectory_label;
    for (const auto& point : partitioned_trajectory.trajectory_point()) {
      auto* point_debug = partition_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";
  }
}

void OpenSpacePlanning::DrawOpenSpaceTrajectoryStitchPoint(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  auto* stitching_line = chart->add_line();
  stitching_line->set_label("TrajectoryStitchingPoint");
  auto* trajectory_stitching_point = stitching_line->add_point();
  trajectory_stitching_point->set_x(open_space_debug.hybrid_search_info()
                                        .trajectory_stitching_point()
                                        .path_point()
                                        .x());
  trajectory_stitching_point->set_y(open_space_debug.hybrid_search_info()
                                        .trajectory_stitching_point()
                                        .path_point()
                                        .y());
  // Set chartJS's dataset properties
  auto* stitching_properties = stitching_line->mutable_properties();
  (*stitching_properties)["borderWidth"] = "3";
  (*stitching_properties)["pointRadius"] = "5";
  (*stitching_properties)["lineTension"] = "0";
  (*stitching_properties)["fill"] = "true";
  (*stitching_properties)["showLine"] = "true";
  if (open_space_debug.hybrid_search_info().is_fallback_trajectory()) {
    (*stitching_properties)["showLine"] = "true";
    (*stitching_properties)["pointStyle"] = "cross";
  }
}

void OpenSpacePlanning::DrawOpenSpaceFallbackTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  auto* collision_line = chart->add_line();
  collision_line->set_label("FutureCollisionPoint");
  auto* future_collision_point = collision_line->add_point();
  future_collision_point->set_x(open_space_debug.hybrid_search_info()
                                    .future_collision_point()
                                    .path_point()
                                    .x());
  future_collision_point->set_y(open_space_debug.hybrid_search_info()
                                    .future_collision_point()
                                    .path_point()
                                    .y());
  // Set chartJS's dataset properties
  auto* collision_properties = collision_line->mutable_properties();
  (*collision_properties)["borderWidth"] = "3";
  (*collision_properties)["pointRadius"] = "8";
  (*collision_properties)["lineTension"] = "0";
  (*collision_properties)["fill"] = "true";

  const auto& fallback_trajectories =
      open_space_debug.hybrid_search_info().fallback_trajectory().trajectory();
  if (fallback_trajectories.empty() ||
      fallback_trajectories[0].trajectory_point().empty()) {
    return;
  }
  const auto& fallback_trajectory = fallback_trajectories[0];
  // has to define chart boundary first
  auto* fallback_line = chart->add_line();
  fallback_line->set_label("Fallback");
  for (const auto& point : fallback_trajectory.trajectory_point()) {
    auto* point_debug = fallback_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }
  // Set chartJS's dataset properties
  auto* fallback_properties = fallback_line->mutable_properties();
  (*fallback_properties)["borderWidth"] = "3";
  (*fallback_properties)["pointRadius"] = "2";
  (*fallback_properties)["lineTension"] = "0";
  (*fallback_properties)["fill"] = "false";
  (*fallback_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddStitchSpeedProfile(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_teb_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  const auto& last_trajectory = injector_->frame_teb_history()
                                    ->Latest()
                                    ->current_frame_planned_trajectory();
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddStitchSpeedProfileThread(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_teb_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  const auto& last_trajectory = injector_->frame_teb_history()
                                    ->Latest()
                                    ->current_frame_planned_trajectory();
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddPublishedSpeed(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE) {
      point_debug->set_y(point.v());
    }
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE) {
      point_debug->set_y(-point.v());
    }
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(0.0);
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(0.0);
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddPublishedSpeedThread(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(kWindowSizeThreshold);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(kMaxDisplayThreshold);
  options->mutable_y()->set_max(kMinDisplayThreshold);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE) {
      point_debug->set_y(point.v());
    }
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE) {
      point_debug->set_y(-point.v());
    }
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(0.0);
  point_debug_up->set_y(kMaxDisplayThreshold);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(0.0);
  point_debug_down->set_y(kMinDisplayThreshold);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

VehicleState OpenSpacePlanning::AlignTimeStamp(
    const VehicleState& vehicle_state, const double curr_timestamp) const {
  // TODO(Jinyun): use the same method in trajectory stitching
  //               for forward prediction
  auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
      curr_timestamp - vehicle_state.timestamp());

  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

void OpenSpacePlanning::AddPublishedAcceleration(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("Acceleration (m/s^2)");

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());
  }
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(0.0);
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(0.0);
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}
void OpenSpacePlanning::AddPublishedAccelerationThread(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  // if open space info provider success run
  if (!frame_teb_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(kWindowSizeThreshold);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(kMaxDisplayThreshold);
  options->mutable_y()->set_max(kMinDisplayThreshold);
  options->mutable_y()->set_label_string("Acceleration (m/s^2)");

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());
  }
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(0.0);
  point_debug_up->set_y(kMaxDisplayThreshold);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(0.0);
  point_debug_down->set_y(kMinDisplayThreshold);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

bool OpenSpacePlanning::CheckAndPubStopTrajectory(
    const double start_timestamp, ReferenceLineInfo* const reference_line_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(ptr_trajectory_pb);

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }

    const PerceptionObstacle& perception_obstacle = obstacle->Perception();
    const auto& obs_id = perception_obstacle.id();
    PerceptionObstacle::Type obs_type = perception_obstacle.type();
    std::string obs_type_name = PerceptionObstacle_Type_Name(obs_type);
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_speed = obstacle->speed();
    double distance = obs_sl.start_s() - adc_sl.end_s();

    bool no_lateral_overlap =
        adc_sl.end_l() + FLAGS_static_obstacle_nudge_l_buffer <
            obs_sl.start_l() ||
        adc_sl.start_l() - FLAGS_static_obstacle_nudge_l_buffer >
            obs_sl.end_l();

    if (obs_speed < FLAGS_stop_speed_buffer &&
        adc_speed < FLAGS_max_stop_speed && !no_lateral_overlap &&
        distance > 0.0 && distance < FLAGS_stop_distance_buffer) {
      std::stringstream ss;
      ss << "Stopped by the obstacle_id[" << obs_id << "] type["
         << obs_type_name << "].";
      std::string msg = ss.str();
      AERROR << msg;

      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateStopTrajectory(ptr_trajectory_pb);

      return true;
    }
  }

  return false;
}

void OpenSpacePlanning::RemoteDecider(ADCTrajectory* const ptr_trajectory_pb) {
  bool planning_error_request = false;
  bool stop_long_time_request = false;
  bool lane_borrow_request = false;
  bool teb_failed_request = false;

  // 1. long time planning failed. 20.0s
  if (!is_planning_error_) {
    planning_error_start_time_ = Clock::NowInSeconds();
  } else {
    if (Clock::NowInSeconds() - planning_error_start_time_ >
        KPlanningErrorTotalTime) {
      planning_error_request = true;
      AERROR << "planning error request.";
    }
  }

  // 2.It is required to request remote takeover when the vehicle stops for 50s
  // near the traffic lights.
  // or 20s not near the traffic lights.
  if (!is_stopped_) {
    stop_start_time_ = Clock::NowInSeconds();
    CancelRemoteRequest(ptr_trajectory_pb);
    return;
  } else {
    if (Clock::NowInSeconds() - stop_start_time_ > KNormalStopTime &&
        !is_near_traffic_light_stop_line_) {
      stop_long_time_request = true;
      AERROR << "stop long time request for normal stop.";
    }
    if (Clock::NowInSeconds() - stop_start_time_ > KStopLineStopTime &&
        is_near_traffic_light_stop_line_) {
      stop_long_time_request = true;
      AERROR << "stop long time request for near the traffic light stop line.";
    }
    if (Clock::NowInSeconds() - stop_start_time_ > KLaneBorrowFailTime &&
        lane_borrow_failed_) {
      lane_borrow_request = true;
      AERROR << "lane borrow fail request";
    }
  }

  // 3. teb stop long or teb more failed request
  if (nullptr != frame_teb_ &&
      (Frame::FaultReport::TEB_STOP_LONG == frame_teb_->fault_report_ ||
       Frame::FaultReport::TEB_MORE_FAILED == frame_teb_->fault_report_)) {
    teb_failed_request = true;
    AERROR << "teb stop long or teb more failed request request";
  }

  if (planning_error_request || stop_long_time_request ||
      request_remote_for_traffic_light_ || lane_borrow_request ||
      teb_failed_request) {
    PubRemoteRequest(lane_borrow_request, ptr_trajectory_pb);
  }
}

void OpenSpacePlanning::PubRemoteRequest(
    const bool lane_borrow_request, ADCTrajectory* const ptr_trajectory_pb) {
  // if have reached your station, no need send remote request
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  if (destination->has_reached_station() ||
      destination->arrived_station_immediately()) {
    return;
  }

  ADCTrajectory::RemotePub* remote_status =
      ptr_trajectory_pb->mutable_remote_status();
  remote_status->set_is_enabled(true);
  ADCTrajectory::RemoteScenario remote_scenario =
      ADCTrajectory::REMOTE_PUB_UNKNOWN;
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  if (request_remote_for_traffic_light_) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_TRAFFIC_LIGHT_UNKNOWN;
  } else if (lane_borrow_request) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_AVOID_OBS;
  } else if (nullptr != frame_teb_ &&
             Frame::FaultReport::TEB_STOP_LONG == frame_teb_->fault_report_) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_TEB_STOP_LONG;
  } else if (nullptr != frame_teb_ &&
             Frame::FaultReport::TEB_MORE_FAILED == frame_teb_->fault_report_) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_TEB_MORE_FAILED;
  } else if (ScenarioConfig::PARK_AND_GO == scenario_type) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_PARK_OUT;
  } else if (ScenarioConfig::VALET_PARKING == scenario_type) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_PARK_IN;
  } else if (ScenarioConfig::DEADEND_TURNAROUND == scenario_type) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_UTURN;
  } else if (ScenarioConfig::LANE_FOLLOW == scenario_type) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_STOP_LONG;
  } else if (ScenarioConfig::RESCUE_TEB == scenario_type) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_TEB_STOP_LONG;
  } else {
    remote_scenario = ADCTrajectory::REMOTE_PUB_STOP_LONG;
  }
  remote_status->set_remote_scenario(remote_scenario);

  if (ADCTrajectory::REMOTE_PUB_STOP_LONG == remote_scenario) {
    ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_PARKING);
  }
}

void OpenSpacePlanning::CancelRemoteRequest(
    ADCTrajectory* const ptr_trajectory_pb) {
  ADCTrajectory::RemotePub* remote_status =
      ptr_trajectory_pb->mutable_remote_status();
  remote_status->set_is_enabled(false);
  ADCTrajectory::RemoteScenario remote_scenario =
      ADCTrajectory::REMOTE_PUB_UNKNOWN;
  remote_status->set_remote_scenario(remote_scenario);
}

void OpenSpacePlanning::FallbackPlanningThread() {
  // Rate rate(10.0);
  while (!is_fallback_planning_thread_stop_) {
    std::this_thread::yield();
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));

    if (CheckFallbackContinue()) {
      continue;
    }

    // last frame
    std::vector<FrenetFramePoint> last_frenet_frame_points;
    std::vector<common::PathPoint> last_discretized_path_points;
    std::vector<Obstacle> path_obstacles;
    std::vector<Obstacle> obstacles;
    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      last_frenet_frame_points.clear();
      last_discretized_path_points.clear();
      path_obstacles.clear();
      obstacles.clear();
    }

    SLBoundary adc_sl_boundary;
    SLPoint last_sl_point;
    SLPoint sl_point_of_match_point_in_last_frame;
    double last_time_stamp = -1.0;
    bool has_last_frame = false;

    //  get last frame and last trajectorypub
    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      if (nullptr == injector_->frame_teb_history()->Latest()) {
        AERROR << " last frame not valid";
        continue;
      }
      if (!GetLastFramePoint(&last_time_stamp, &last_sl_point,
                             &sl_point_of_match_point_in_last_frame)) {
        continue;
      }
      GetLastFrameInfo(&last_frenet_frame_points, &last_discretized_path_points,
                       &path_obstacles, &obstacles, &adc_sl_boundary);
      has_last_frame = true;
    }

    if (!has_last_frame) {
      ADEBUG << "No last frame.";
      continue;
    }

    DiscretizedTrajectory reference_trajectory;
    if (!GetReferenceTrajectoryFromLastFrame(
            last_frenet_frame_points, last_discretized_path_points,
            path_obstacles, obstacles, adc_sl_boundary, last_time_stamp,
            last_sl_point, sl_point_of_match_point_in_last_frame,
            &reference_trajectory)) {
      continue;
    }

    ADCTrajectory adc_trajectory;
    ADCTrajectory adc_trajectory_test;
    adc_trajectory.mutable_header()->set_timestamp_sec(start_timestamp_);
    adc_trajectory.mutable_trajectory_point()->CopyFrom(
        {reference_trajectory.begin(), reference_trajectory.end()});
    if (!reference_trajectory.empty()) {
      const auto& last_tp = reference_trajectory.back();
      adc_trajectory.set_total_path_length(
          last_tp.path_point().s() -
          reference_trajectory.front().path_point().s());
      adc_trajectory.set_total_path_time(last_tp.relative_time());
    }

    // pub jectory
    adc_trajectory.set_gear(canbus::Chassis::GEAR_DRIVE);
    PlanningBase::FillPlanningPb(start_timestamp_, &adc_trajectory);
    planning_writer_->Write(adc_trajectory);
  }

  ADEBUG << "The fallback planning task is completed.";
  return;
}

bool OpenSpacePlanning::GetLastFramePoint(
    double* const last_time_stamp, SLPoint* const last_sl_point,
    SLPoint* const sl_point_of_match_point_in_last_frame) {
  const auto* last_frame = injector_->frame_teb_history()->Latest();
  if (!last_frame->DriveReferenceLineInfo()) {
    AERROR << " last frame havn't reference_line_info";
    return false;
  }
  if (last_publishable_trajectory_ == nullptr ||
      last_publishable_trajectory_.get() == nullptr) {
    AINFO << "have no last_publishable_trajectory";
    return false;
  }

  // get last frame init point
  PathPoint last_init_point = last_frame->PlanningStartPoint().path_point();
  if (!last_init_point.has_x()) {
    ADEBUG << "Have not init point.";
    return false;
  }
  Vec2d last_xy_point(last_init_point.x(), last_init_point.y());
  // get last frame init sl point
  if (!last_frame->DriveReferenceLineInfo()->reference_line().XYToSL(
          last_xy_point, last_sl_point)) {
    ADEBUG << "Fail to create last_init_sl_point";
    return false;
  }

  const PublishableTrajectory last_publishable_trajectory =
      *last_publishable_trajectory_.get();
  *last_time_stamp = last_publishable_trajectory.header_time();
  std::vector<TrajectoryPoint> stitching_trajectory;
  TrajectoryStitcher::ComputeFallbackStitchingTrajectory(
      vehicle_state_fallback_, Clock::NowInSeconds(), kFallbackOncetime,
      &last_publishable_trajectory, &stitching_trajectory);
  auto matched_point = stitching_trajectory.back();
  // match_point xy and match_point sl in last frame
  Vec2d match_point_xy_point(matched_point.path_point().x(),
                             matched_point.path_point().y());
  if (!last_frame->DriveReferenceLineInfo()->reference_line().XYToSL(
          match_point_xy_point, sl_point_of_match_point_in_last_frame)) {
    ADEBUG << "Fail to create sl_point_of_match_point_in_last_frame";

    return false;
  }

  return true;
}

void OpenSpacePlanning::GetLastFrameInfo(
    std::vector<FrenetFramePoint>* const last_frenet_frame_points,
    std::vector<common::PathPoint>* const last_discretized_path_points,
    std::vector<Obstacle>* const path_obstacles,
    std::vector<Obstacle>* const obstacles, SLBoundary* const adc_sl_boundary) {
  const auto* last_frame = injector_->frame_teb_history()->Latest();
  for (auto point = last_frame->DriveReferenceLineInfo()
                        ->path_data()
                        .frenet_frame_path()
                        .begin();
       point != last_frame->DriveReferenceLineInfo()
                    ->path_data()
                    .frenet_frame_path()
                    .end();
       point++) {
    last_frenet_frame_points->emplace_back(std::move(*point));
  }

  for (auto point = last_frame->DriveReferenceLineInfo()
                        ->path_data()
                        .discretized_path()
                        .begin();
       point != last_frame->DriveReferenceLineInfo()
                    ->path_data()
                    .discretized_path()
                    .end();
       point++) {
    last_discretized_path_points->emplace_back(std::move(*point));
  }

  *adc_sl_boundary = last_frame->DriveReferenceLineInfo()->AdcSlBoundary();

  const auto& upstream_path_obstacles =
      last_frame->DriveReferenceLineInfo()->path_decision().obstacles().Items();
  for (uint i = 0; i < upstream_path_obstacles.size(); i++) {
    obstacles->emplace_back(std::move(*upstream_path_obstacles.at(i)));
    path_obstacles->emplace_back(std::move(*upstream_path_obstacles.at(i)));
  }
}

bool OpenSpacePlanning::GetReferenceTrajectoryFromLastFrame(
    const std::vector<FrenetFramePoint>& last_frenet_frame_points,
    const std::vector<common::PathPoint>& last_discretized_path_points,
    const std::vector<Obstacle>& path_obstacles,
    const std::vector<Obstacle>& obstacles, const SLBoundary& adc_sl_boundary,
    const double& last_time_stamp, const SLPoint& last_sl_point,
    const SLPoint& sl_point_of_match_point_in_last_frame,
    DiscretizedTrajectory* const reference_trajectory) {
  double veh_rel_time =
      Clock::NowInSeconds() - last_time_stamp + kFallbackOncetime;
  const auto* last_frame = injector_->frame_teb_history()->Latest();
  common::TrajectoryPoint last_init_point =
      last_frame->current_frame_planned_trajectory().trajectory_point().at(0);

  const double diff_s = last_init_point.v() * veh_rel_time +
                        0.5 * last_init_point.a() * veh_rel_time * veh_rel_time;

  // get path data
  std::vector<common::FrenetFramePoint> frenet_path;
  PathData path_data;
  DiscretizedPath discretized_path;
  double get_path_time_1 = 0.0;
  double loop = 0.0;

  util::SetFallBackLastFramePath(
      last_discretized_path_points, last_frenet_frame_points, last_sl_point,
      diff_s, &frenet_path, sl_point_of_match_point_in_last_frame,
      &discretized_path, &get_path_time_1, &loop);
  path_data.SetFallbackDiscretizedPath(std::move(discretized_path));

  // get nearest obstacles
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double front_nearest_distance = util::GetFrontNearestObstaclesDistance(
      path_obstacles, obstacles, frenet_path, diff_s, adc_sl_boundary,
      last_init_point, sl_point_of_match_point_in_last_frame, vehicle_param);
  // set speed data
  SpeedData speed_data;
  if (!util::SetFallBackSpeedData(last_init_point, front_nearest_distance,
                                  &speed_data, is_sim_control_)) {
    ADEBUG << "get speed data invalid";
    return false;
  }

  if (!util::CombinePathAndSpeedProfile(
          0.0, sl_point_of_match_point_in_last_frame.s(),
          path_data.discretized_path(), speed_data, reference_trajectory)) {
    return false;
  }
  return true;
}

bool OpenSpacePlanning::CheckFallbackContinue() {
  // no auto state
  if (!is_auto_state_ && !is_sim_control_) {
    return true;
  }

  double distance_to_destination = std::sqrt(
      (route_end_.pose().x() - vehicle_state_fallback_.pose().position().x()) *
          (route_end_.pose().x() -
           vehicle_state_fallback_.pose().position().x()) +
      (route_end_.pose().y() - vehicle_state_fallback_.pose().position().y()) *
          (route_end_.pose().y() -
           vehicle_state_fallback_.pose().position().y()));
  if (distance_to_destination < kDistanceToDestination) {
    return true;
  }

  double start_time = Clock::NowInSeconds();
  bool is_planning_timed_out = start_time - start_timestamp_ > kTimeThreshold;
  if (!is_planning_timed_out) {
    return true;
  }

  if (!routing_is_ready_) {
    return true;
  }

  return false;
}

void OpenSpacePlanning::FallbackStop() {
  is_fallback_planning_thread_stop_ = true;
  if (fallback_planning_thread_ != nullptr &&
      fallback_planning_thread_->Joinable()) {
    fallback_planning_thread_->Join();
  }
}

void OpenSpacePlanning::GeneratePreferredLane() {
  auto* overtake_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_overtake();

  overtake_status->clear_preferred_lane_ids();
  overtake_status->clear_all_route_lane_ids();

  // Get preferred lane ids
  for (int i = 0; i < last_routing_.road_size(); ++i) {
    if (last_routing_.road(i).passage_size() > 0 &&
        routing::FORWARD ==
            last_routing_.road(i).passage(0).change_lane_type()) {
      for (int n = 0; n < last_routing_.road(i).passage(0).segment_size();
           ++n) {
        overtake_status->add_preferred_lane_ids(
            last_routing_.road(i).passage(0).segment(n).id());
      }
    } else if (last_routing_.road(i).passage_size() > 1) {
      for (int n = 0; n < last_routing_.road(i).passage(1).segment_size();
           ++n) {
        overtake_status->add_preferred_lane_ids(
            last_routing_.road(i).passage(1).segment(n).id());
      }
    }
  }

  // Get all route lane ids
  for (int i = 0; i < last_routing_.road_size(); ++i) {
    for (int n = 0; n < last_routing_.road(i).passage_size(); ++n) {
      for (int m = 0; m < last_routing_.road(i).passage(n).segment_size();
           ++m) {
        overtake_status->add_all_route_lane_ids(
            last_routing_.road(i).passage(n).segment(m).id());
      }
    }
  }
}

// Generate route lane information
void OpenSpacePlanning::GenerateRouteLaneInfo() {
  auto* route_lane_info =
      injector_->planning_context()->mutable_route_lane_info();
  route_lane_info->clear();
  for (int i = 0; i < last_routing_.road(0).passage_size(); ++i) {
    std::vector<RouteLaneInfo> passage_lane;
    for (int j = 0; j < last_routing_.road(0).passage(i).segment_size(); ++j) {
      RouteLaneInfo one_lane;
      one_lane.lane_id = last_routing_.road(0).passage(i).segment(j).id();
      one_lane.can_exit = last_routing_.road(0).passage(i).can_exit();
      passage_lane.emplace_back(one_lane);
    }
    route_lane_info->emplace_back(passage_lane);
  }

  for (int i = 1; i < last_routing_.road_size(); ++i) {
    for (int m = 0; m < last_routing_.road(i).passage_size(); ++m) {
      std::vector<RouteLaneInfo> passage_lane;
      for (int n = 0; n < last_routing_.road(i).passage(m).segment_size();
           ++n) {
        RouteLaneInfo one_lane;
        one_lane.lane_id = last_routing_.road(i).passage(m).segment(n).id();
        one_lane.can_exit = last_routing_.road(i).passage(m).can_exit();
        passage_lane.emplace_back(one_lane);
      }
      AddPassageLaneInfo(route_lane_info, passage_lane);
    }
  }

  // Generate route lane index
  auto* route_lane_index =
      injector_->planning_context()->mutable_route_lane_index();
  route_lane_index->clear();
  for (size_t i = 0; i < route_lane_info->size(); ++i) {
    for (size_t j = 0; j < (*route_lane_info)[i].size(); ++j) {
      route_lane_index->emplace((*route_lane_info)[i][j].lane_id,
                                std::make_pair(i, j));
    }
  }

  // print debug info
  AINFO << "route lane info [lane_id]<can_exit><is_merge>:";
  for (size_t i = 0; i < route_lane_info->size(); ++i) {
    std::string print_info = std::to_string(i + 1) + ": ";
    for (size_t j = 0; j < (*route_lane_info)[i].size(); ++j) {
      auto ptr_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
          hdmap::MakeMapId((*route_lane_info)[i][j].lane_id));
      if (ptr_lane == nullptr) {
        AERROR << "failed to get lane by id: " +
                      (*route_lane_info)[i][j].lane_id;
        print_info = print_info + "(ERROR:" + "[" +
                     (*route_lane_info)[i][j].lane_id + "]" + ") ";
        continue;
      }
      print_info = print_info + "[" + (*route_lane_info)[i][j].lane_id + "]<" +
                   std::to_string((*route_lane_info)[i][j].can_exit) + "><" +
                   std::to_string(ptr_lane->lane().is_merge()) + ">";
    }
    AINFO << print_info;
  }
}

void OpenSpacePlanning::AddPassageLaneInfo(
    std::vector<std::vector<RouteLaneInfo>>* route_lane_info,
    const std::vector<RouteLaneInfo>& passage_lane) {
  bool find_matching = false;
  size_t matching_index = 0;
  for (size_t i = 0; i < route_lane_info->size(); ++i) {
    auto ptr_back_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
        hdmap::MakeMapId((*route_lane_info)[i].back().lane_id));
    if (nullptr != ptr_back_lane &&
        !ptr_back_lane->lane().successor_id().empty()) {
      for (int n = 0; n < ptr_back_lane->lane().successor_id_size(); ++n) {
        if (ptr_back_lane->lane().successor_id(n).id() ==
            passage_lane.front().lane_id) {
          find_matching = true;
          matching_index = i;
          break;
        }
      }
    }
    if (find_matching) {
      break;
    }
  }

  if (find_matching) {
    for (size_t i = 0; i < passage_lane.size(); ++i) {
      (*route_lane_info)[matching_index].emplace_back(passage_lane[i]);
    }
  } else {
    route_lane_info->emplace_back(passage_lane);
  }
}

void OpenSpacePlanning::CheckOffLaneDeparture() {
  injector_->is_off_lane_depart_ = false;
  const auto& start_lane_id = last_routing_.road(0).passage(0).segment(0).id();
  hdmap::LaneInfoConstPtr ptr_start_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
          hdmap::MakeMapId(start_lane_id));
  if (nullptr == ptr_start_lane ||
      ptr_start_lane->lane().type() != hdmap::Lane::PLAY_STREET) {
    return;
  }

  double s = 0.0;
  double l = 0.0;
  if (!ptr_start_lane->GetProjection(
          {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()},
          &s, &l)) {
    return;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  ptr_start_lane->GetWidth(s, &left_width, &right_width);

  if (l > left_width || l < -right_width) {
    injector_->is_off_lane_depart_ = true;
    AINFO << "is_off_lane_depart_ is true, because the center point "
             "of the rear axle of the vehicle is outside the lane.";
    return;
  }

  // If the center point of the rear axle of the vehicle is within the lane,
  // check again whether the four corner points of the vehicle are within the
  // lane.
  const auto& adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y(),
      injector_->vehicle_state()->heading(), 0.0, kOffLaneLatitudebuffer);
  const auto& corners = adc_box.GetAllCorners();
  for (const auto corner : corners) {
    double corner_s = 0.0;
    double corner_l = 0.0;
    if (!ptr_start_lane->GetProjection(corner, &corner_s, &corner_l)) {
      continue;
    }

    double corner_left_width = 0.0;
    double corner_right_width = 0.0;
    ptr_start_lane->GetWidth(corner_s, &corner_left_width, &corner_right_width);

    if (corner_l > corner_left_width || corner_l < -corner_right_width) {
      injector_->is_off_lane_depart_ = true;
      AINFO << "is_off_lane_depart_ is true, because there is a "
               "corner of the vehicle outside the lane.";
      return;
    }
  }
}

bool OpenSpacePlanning::GenerateTEBThread(
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  static double openspace_success_time = Clock::NowInSeconds();

  ADCTrajectory adc_trajectory_pb_thread_temp;
  static century::planning::Frame* frame_teb;
  CHECK_NOTNULL(injector_);
  // UpdateTEBFrameInfo();
  frame_teb = frame_teb_.get();

  CHECK_NOTNULL(frame_teb);

  // if (FLAGS_enable_openspace_record_debug) {
  //   frame_teb_->RecordInputDebug(adc_trajectory_pb_thread_temp.mutable_debug());
  // }

  adc_trajectory_pb_thread_temp.mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - openspace_success_time);
  auto* frame_history = injector_->frame_teb_history()->Latest();
  if (nullptr == frame_history || nullptr == frame_teb) {
    // if (nullptr != frame_teb) {
    //   const uint32_t n = frame_teb_->SequenceNum();
    //   injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
    // }
    AERROR << "frame_history or frame_teb is nullptr";

    return false;
  }

  century::planning::ScenarioConfig config_teb;
  // Create RescueStageTeb
  century::cyber::common::GetProtoFromFile(
      FLAGS_scenario_rescue_teb_config_file, &config_teb);

  // // static Stage stage_teb(config_teb.stage_config()[0], injector_);
  static RescueStageTeb rescue_stage_teb(config_teb.stage_config()[0],
                                         injector_);
  UpdateRescueStatusInfo(frame_teb);
  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    if (!last_use_teb_trajectory_) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_rescue()
          ->set_is_first_init(true);
      frame_teb->mutable_open_space_info()->set_is_rescue_mode(true);
    }
  }
  scenario::rescue::RescueTebContext context_teb;
  context_teb.scenario_config.CopyFrom(config_teb.rescue_config());
  rescue_stage_teb.SetFrame(frame_teb);
  rescue_stage_teb.SetInjector(injector_);
  rescue_stage_teb.SetsScenarioConfig(context_teb.scenario_config);

  // const uint32_t n = frame_teb_->SequenceNum();

  const auto& history_frame = injector_->frame_teb_history()->Latest();
  if (history_frame) {
    if (!history_frame->open_space_info().is_on_open_space_trajectory() ||
        !injector_->need_to_rescue_thread()) {
      openspace_success_time = Clock::NowInSeconds();
      injector_->is_teb_overtime_ = false;
      rescue_stage_teb.ClearDataThread();
    }
  }
  injector_->is_teb_overtime_ =
      Clock::NowInSeconds() - openspace_success_time > FLAGS_rescue_max_time;

  if (FLAGS_enable_openspace_record_debug) {
    UpdateTEBOpenSpaceInfo(&adc_trajectory_pb_thread_temp,
                           stitching_trajectory);
  }

  // if (!CalculationTEBTrajectory(frame_teb, &openspace_success_time,
  //                               &rescue_stage_teb)) {
  //   AERROR << "CalculationTEBTrajectory field";
  //   return false;
  // }
  century::common::TrajectoryPoint planning_init_point;
  auto ret = rescue_stage_teb.Process(planning_init_point, frame_teb);
  if (ret == scenario::Stage::StageStatus::ERROR) {
    // if (ret == scenario::Stage::StageStatus::FINISHED ||
    //   ret == scenario::Stage::StageStatus::ERROR) {
    AERROR << "CalculationTEBTrajectory field";
    return false;
  }

  SetOpenSpaceTrajectoryThread(&adc_trajectory_pb_thread_temp);

  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    ptr_trajectory_pb->CopyFrom(adc_trajectory_pb_thread_temp);
  }
  // AERROR << "frame_teb_->open_space_info().open_space_provider_success(): "
  //        << frame_teb_->open_space_info().open_space_provider_success();

  // injector_->frame_teb_history()->Add(n, std::move(frame_teb_));

  return true;
}

void OpenSpacePlanning::UpdateTEBOriginInfo(
    century::planning::Frame* frame_teb) {
  std::lock_guard<std::mutex> frame_lock(frame_mutex_);

  const auto rescue_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_rescue();

  rescue_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->vehicle_state().x());
  rescue_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->vehicle_state().y());
  rescue_status->mutable_adc_init_position()->set_z(0.0);
  rescue_status->set_adc_init_heading(
      injector_->vehicle_state()->vehicle_state().heading());

  auto* teb_common = injector_->planning_context()
                         ->mutable_planning_status()
                         ->mutable_teb_common();
  // if (std::isnan(teb_common->adc_init_position().x())) {
  teb_common->Clear();

  teb_common->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->vehicle_state().x());
  teb_common->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->vehicle_state().y());
  teb_common->mutable_adc_init_position()->set_z(0.0);
  teb_common->set_adc_init_heading(
      injector_->vehicle_state()->vehicle_state().heading());
  // }
  frame_teb->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(rescue_status->adc_init_heading()));
  frame_teb->mutable_open_space_info()->mutable_origin_point()->set_x(
      rescue_status->adc_init_position().x());
  frame_teb->mutable_open_space_info()->mutable_origin_point()->set_y(
      rescue_status->adc_init_position().y());
  AERROR << "frame_teb.open_space_info_origin_point_x(): "
         << frame_teb->open_space_info().origin_point().x()
         << "frame_teb.open_space_info_origin_point_y(): "
         << frame_teb->open_space_info().origin_point().y();

  // static auto last_vehicle_state_x =
  //     injector_->vehicle_state()->vehicle_state().x();
  // static auto last_vehicle_state_y =
  //     injector_->vehicle_state()->vehicle_state().y();
  // static auto last_vehicle_state_heading =
  //     injector_->vehicle_state()->vehicle_state().heading();
  // auto current_vehicle_state_x =
  //     injector_->vehicle_state()->vehicle_state().x();
  // auto current_vehicle_state_y =
  //     injector_->vehicle_state()->vehicle_state().y();
  // auto current_vehicle_state_heading =
  //     injector_->vehicle_state()->vehicle_state().heading();

  // if (((current_vehicle_state_x - last_vehicle_state_x) *
  //          (current_vehicle_state_x - last_vehicle_state_x) +
  //      (current_vehicle_state_y - last_vehicle_state_y) *
  //          (current_vehicle_state_y - last_vehicle_state_y)) > 0.25) {
  //   AERROR << "error_x: " << current_vehicle_state_x - last_vehicle_state_x
  //          << " error_y: " << current_vehicle_state_y - last_vehicle_state_y
  //          << " error_heading: "
  //          << current_vehicle_state_heading - last_vehicle_state_heading
  //          << " need_to_rescue: " << injector_->need_to_rescue();
  // }
  // AERROR << "current scenario_type: "
  //        << injector_->planning_context()
  //               ->planning_status()
  //               .scenario()
  //               .scenario_type();
  // last_vehicle_state_x = injector_->vehicle_state()->vehicle_state().x();
  // last_vehicle_state_y = injector_->vehicle_state()->vehicle_state().y();
  // last_vehicle_state_heading =
  //     injector_->vehicle_state()->vehicle_state().heading();

  // AINFO << "injector_->vehicle_state()->x()"
  //       << injector_->vehicle_state()->vehicle_state().x();
}

bool OpenSpacePlanning::CalculationTEBTrajectory(
    century::planning::Frame* frame_teb, double* openspace_success_time,
    RescueStageTeb* rescue_stage_teb) {
  // const uint32_t n = frame_teb_->SequenceNum();
  injector_->teb_adc_is_out_lane_ =
      rescue_stage_teb->CalVehicleIsOutRoad(frame_teb);
  if (rescue_stage_teb->CheckReachGoal()) {
    *openspace_success_time = Clock::NowInSeconds();
    AINFO << "TEB thread reach goal";

    // injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
    return false;
  }
  if (injector_->is_teb_overtime_ ||
      rescue_stage_teb->StopLongTimeReportAndExit(frame_teb)) {
    *openspace_success_time = Clock::NowInSeconds();
    AERROR << "TEB thread overtime or stop long time";
    // injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
    return false;
  }
  bool task_failed = false;
  if (!rescue_stage_teb->ExecuteTaskOnOpenSpace(frame_teb)) {
    task_failed = true;
    AERROR << "TEB thread execute task error";
  }

  if (rescue_stage_teb->TaskFailedReportAndExit(task_failed, frame_teb)) {
    // injector_->frame_teb_history()->Add(n, std::move(frame_teb_));
    injector_->set_rescue_replan(true);
    AERROR << "TEB Planning Failed too more.";
    return false;
  }
  return true;
}
void OpenSpacePlanning::UpdateTEBOpenSpaceInfo(
    ADCTrajectory* const adc_trajectory_pb_thread_temp,
    const std::vector<TrajectoryPoint>& stitching_trajectory) {
  auto* ptr_debug = adc_trajectory_pb_thread_temp->mutable_debug();

  ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
      stitching_trajectory.back());
  frame_teb_->mutable_open_space_info()->set_debug(ptr_debug);
  frame_teb_->mutable_open_space_info()->sync_debug_instance();

  auto scenario_debug = adc_trajectory_pb_thread_temp->mutable_debug()
                            ->mutable_planning_data()
                            ->mutable_scenario();
  scenario_debug->set_scenario_type(ScenarioConfig::RESCUE_TEB);
  scenario_debug->set_stage_type(ScenarioConfig::RESCUE_TEB_ADJUST);
}

void OpenSpacePlanning::UpdateRescueStatusInfo(Frame* frame_teb) {
  frame_teb->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  static bool runonce = false;
  {
    // Update teb scene location information
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    auto* rescue_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_rescue();
    if (!runonce || std::isnan(rescue_status->adc_init_position().x())) {
      rescue_status->Clear();
      century::planning::RescueStatus pb_rescue_status;
      if (!century::cyber::common::GetProtoFromFile(
              FLAGS_rescue_status_config_file, &pb_rescue_status)) {
        AERROR << "----Failed to load txt rescue status from "
               << FLAGS_rescue_status_config_file;
      } else {
        ADEBUG << "----Loaded txt rescue status from "
               << FLAGS_rescue_status_config_file;
      }

      rescue_status->CopyFrom(pb_rescue_status);
      UpdateTEBOriginInfo(frame_teb);

      runonce = true;
    }
    AERROR << "need_to_rescue_thread: " << injector_->need_to_rescue_thread();
    if (!injector_->need_to_rescue_thread()) {
      UpdateTEBOriginInfo(frame_teb);
    }
  }
}

void OpenSpacePlanning::UpdateTEBFrameInfo() {
  // get the data in injector_ and initialize frame_teb
  std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
  Status status1 = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);
  // const uint32_t frame_num = static_cast<uint32_t>(seq_num_);
  // TrajectoryPoint trajectory_point_temp =
  // injector_->ego_info()->start_point(); const auto& vehicle_state =
  // injector_->vehicle_state()->vehicle_state(); init frame_teb_ auto status =
  // InitTEBFrame(frame_num, trajectory_point_temp, vehicle_state);
}

}  // namespace planning
}  // namespace century
