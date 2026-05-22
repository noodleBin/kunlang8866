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

#include "modules/planning/on_lane_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <sstream>
#include <sys/stat.h>
#include <utility>

#include "gflags/gflags.h"
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
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/danger_stay_away_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_heading_history.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_diff_value.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_value.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_speed_distance_history.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_stabilization_for_teb_speed.h"
#include "modules/planning/common/historical_tracking_algorithms/start_up_vehicle_position_history.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
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
using century::common::VehicleConfigHelper;
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
constexpr double KPlanningErrorTotalTime = 40.0;  // s
constexpr double KNormalStopTime = 90.0;          // s
constexpr double KStopLineStopTime = 60.0;        // s
constexpr double KLaneBorrowFailTime = 60.0;       // s
constexpr double KGateBlockStopTime = 10.0;       // s
constexpr int KStopEnsureCount = 5;
constexpr double KStopSpeedThreshold = 0.05;  // m/s
constexpr double KStopSpeedForReverseRouting = 0.5;
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
constexpr double kDistanceofDisplayObs = 10.0;
constexpr double kOffLaneLatitudebuffer = 0.0;
constexpr double kOptimizedDeltaT = 0.1;
constexpr double kDeltaT = 0.1;
constexpr double kCollisionLateralBuffer = 0.4;
constexpr double kBackTrajectoryStopDistance = 5.0;
constexpr double kBackTrajectoryMaxDistance = 50.0;
constexpr double kCollisionLonBuffer = 0.4;
constexpr double kCollisionBuffer = 1.0;
constexpr double kEpsilon = 0.00001;
constexpr double kDistanceFrontAdc = 3.0;
constexpr double kDistanceRearAdc = 3.0;
constexpr double kMinStopDistance = 0.2;
constexpr double kMinStopSpeed = 0.2;
constexpr double kOppositeRoutingAngleThreshold = M_PI_2;
constexpr double kDistanceToFrontObs = 20.0;
constexpr double kReverseTargetspeed = 1.0;
constexpr double kComfortDecel = 0.5;
constexpr double kComfortAcc = 0.5;
constexpr double kMinBreakingTime = 0.1;
constexpr double kEffectiveTime = 5.0;
constexpr double kMinTrajectoryLength = 6.0;
constexpr double kDistanceToBackPoint = 3.0;
constexpr double kStopSpeed = 0.1;
constexpr double kMaxPredictionTime = 4.9;
constexpr double kTopBullArriveDistance = 0.5;
constexpr double kTopBullArriveSpeed = 0.5;
constexpr double kDistanceBeforeTurn = 10.0;
constexpr double kDistanceBeforePedes = 30.0;
constexpr double kKappaThreshold = 0.005;
constexpr double kLdis2Pedesthres = 3.0;
constexpr double kLowSOCThres = 30.0;
constexpr double kLowTirePressureThres = 80.0;
constexpr double kHighTirePressureThres = 125.0;
constexpr double kLowSpeedThres = 0.05;
constexpr double kTurnWheelAngleThres = 15.0;
constexpr double kDistanceBeforeBorrow = 50.0;
constexpr double kBorrowRequestSpeed = 1.5;
constexpr uint8_t kRoutingTurnMaxCount = 30;
constexpr uint8_t kStackerLonSafeCount = 5;
constexpr double kReverseStopSpeed = 0.1;
constexpr double kLateralBufferForCollision = 0.4;
constexpr double kLonBufferForCollision = 0.4;
constexpr double kDistanceToBorrowObs = 15.0;
constexpr double kLateralDistanceToDestination = 5.0;
constexpr double kDestinationNoBorrowDistance = 40.0;
constexpr double kTiny = 0.001;
constexpr int kBackwardCount = 50;
constexpr double kStackerConsiderDistance = 12.0;
constexpr double kStackerDisappear = 100.0;
constexpr double kStartConsiderStackerDistance = 50.0;
constexpr double kReroutingMaxDistance = 60.0;
constexpr double kHumanReroutingMaxDistance = 40.0;
constexpr double kStackerLength = 3.0;
constexpr int16_t KobsConfirmCnt = 3;
constexpr int kMaxHuamanShapedLevel = 4;
constexpr double kReroutingMinDistance = 1.5;
constexpr int kObsBlockageCnt = 10;
constexpr int kBlockageCnt = 30;
constexpr int kMaxReroutingTimes = 3;
constexpr double kStackerRealLength = 8.1;
constexpr double kStackerRealWidth = 4.4;
constexpr double kStackerFrontToLocPoint = 0.8;
constexpr double kWheelCraneHeading = 1.43;
constexpr double kWheelCraneRealLength = 23.47;
constexpr double kWheelCraneRealWidth = 6.9;
constexpr double kWheelCraneFrontToLocPoint = 0.0;
constexpr double kDistanceForDestination = 0.4;
constexpr double kDistanceCenterLineX = -3228.05732755;
constexpr double kDistanceCenterLineY = 2302.37255269;
constexpr double kHeadingCenterLine = -0.15;
constexpr double kDistanceToreturnDistance = 30.0;
constexpr double kAutoBorrowLateralBuffer = 1.0;
constexpr double kAutoBorrowConsiderSpeed = 0.5;
constexpr double kAutoBorrowConsiderTrajectoryLength = 3.0;
constexpr double kSpeedForTemporary = 3.0;
constexpr double kStopLength = 0.3;
constexpr double kPassObsDistance = 2.0;
constexpr int DisappearBoundaryObsCount = 10;
constexpr int AdcBlockCount = 10;
constexpr int AdcStableStopCount = 10;
constexpr int MaxCount = 100;
constexpr double kOutRoutingConsiderLateralDistance = 20.0;
constexpr double kReroutingPointX = -3162.72663274;
constexpr double kReroutingPointY = 2183.39117151;
constexpr double kExpresswayJunctionMaxSearchRadius = 9.0;
constexpr double kExpresswayJunctionMinSearchRadius = 6.0;
constexpr double kUpDistacne= 0.51;
constexpr double kCenterDistacne= 0.41;
constexpr double kCenterLowDistacne= 0.40;
constexpr double kLowDistacne= 0.30;
constexpr double kLowUpDistacne= 0.21;
constexpr double kLowLowDistacne= 0.19;
constexpr double kDistacneToWheelCraneStopWall= 10.0;
constexpr int kNoWheelCraneCountTimes = 50;
constexpr int kLargeCountTimes = 100;
constexpr double kRoutingPassPointTolerance = 1.5;
constexpr double kEastInPointX = -2489.20589709;
constexpr double kEastInPointY = 2179.06870724;
constexpr double kEastOutPointX = -2454.57071474;
constexpr double kEastOutPointY = 2215.59209459;
constexpr double kWestInPointX = -3294.05569423;
constexpr double kWestInPointY = 2320.40632442;
constexpr double kWestOutPointX = -3329.25727938;
constexpr double kWestOutPointY = 2367.73936793;

bool IsLanePassPoint(const hdmap::LaneInfoConstPtr& lane,
                     const Vec2d& point, const double tolerance) {
  if (lane == nullptr) {
    return false;
  }
  double lane_s = 0.0;
  double lane_l = 0.0;
  if (!lane->GetProjection(point, &lane_s, &lane_l)) {
    return false;
  }
  if (std::fabs(lane_l) >= tolerance || lane_s < -1.0 ||
      lane_s > lane->total_length() + 1.0) {
    return false;
  }
  return true;
}
}  // namespace

OnLanePlanning::~OnLanePlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_routing_.Clear();
  pending_routing_.Clear();
  has_pending_routing_ = false;
  injector_->ego_info()->Clear();
  FallbackStop();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

Status OnLanePlanning::Init(const PlanningConfig& config) {
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

  // clear planning history
  injector_->history()->Clear();

  // clear planning status
  injector_->planning_context()->mutable_planning_status()->Clear();

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
        "fallback_plan", &OnLanePlanning::FallbackPlanningThread, this));
    ADEBUG << "OnLanePlanning::FallbackPlanningThread is created.";
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

bool OnLanePlanning::IsGateJunctionContainAdc() const {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, kJunctionSearchRadius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (Junction::DongJiaZhen_Gate == ptr_junction->junction().type()) {
        if (IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                                 *ptr_junction)) {
          AINFO << "gate junction conrain adc ";
          return true;
        }
      }else{
        continue;
      }
    }
  }
  return false;
}

bool OnLanePlanning::IsVehInSpecificJunction(int junction_type) {
  std::vector<JunctionInfoConstPtr> junctions;
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, 0.0, &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (ptr_junction->junction().type() == junction_type &&
          IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction, true)) {
        return true;
      }
    }
  }
  return false;
}

bool OnLanePlanning::IsExpresswayJunctionWithoutLaneChange() {
  // size_t waypoint_num =
  // local_view_.routing->routing_request().waypoint_size();
  if (local_view_.routing != nullptr &&
      local_view_.routing->routing_request().waypoint_size() > 1) {
    size_t waypoint_num =
        local_view_.routing->routing_request().waypoint_size();
    auto target_point = local_view_.routing->routing_request()
                            .waypoint()
                            .at(waypoint_num - 1)
                            .pose();
    auto local_routing_type =
        local_view_.routing->routing_request().local_routing_type();
    bool is_human_shape =
        local_routing_type == routing::ROUTING_J4_TO_T4T5_EAST ||
        local_routing_type == routing::ROUTING_T4T5_TO_J4_WEST ||
        local_routing_type == routing::ROUTING_J4_TO_T4T5_MIDDLE ||
        local_routing_type == routing::ROUTING_T4T5_TO_J4_MIDDLE ||
        local_routing_type == routing::ROUTING_J4_TO_T4T5 ||
        local_routing_type == routing::ROUTING_T4T5_TO_J4 ||
        local_routing_type == routing::ROUTING_J4_TO_T4T5_EAST_NEAR;
    const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
    common::PointENU routing_end_point;
    // adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
    // adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
    routing_end_point.set_x(target_point.x());
    routing_end_point.set_y(target_point.y());
    std::vector<JunctionInfoConstPtr> junctions;
    if (0 == base_map_ptr->GetJunctions(routing_end_point,
                                        kExpresswayJunctionMaxSearchRadius,
                                        &junctions)) {
      for (const auto& ptr_junction : junctions) {
        bool is_point_in_iunction = ptr_junction->polygon().IsPointIn(
            {routing_end_point.x(), routing_end_point.y()});
        if (is_point_in_iunction) {
          if (Junction::J4_1 == ptr_junction->junction().type() &&
              !is_human_shape) {
            if (IsVehInSpecificJunction(Junction::J4_2_EXPRESSWAY)) {
              return true;
            }
          } else {
            continue;
          }
        }
      }
    }
    if (0 == base_map_ptr->GetJunctions(routing_end_point,
                                        kExpresswayJunctionMinSearchRadius,
                                        &junctions)) {
      for (const auto& ptr_junction : junctions) {
        bool is_point_in_iunction = ptr_junction->polygon().IsPointIn(
            {routing_end_point.x(), routing_end_point.y()});
        if (is_point_in_iunction) {
          continue;
        }
        if (Junction::DongJiaZhen_B1_EXPRESSWAY ==
            ptr_junction->junction().type()) {
          if (IsVehInSpecificJunction(Junction::DongJiaZhen_B1_EXPRESSWAY)) {
            return true;
          }
        } else if (Junction::DongJiaZhen_C_EXPRESSWAY ==
                   ptr_junction->junction().type()) {
          if (IsVehInSpecificJunction(Junction::DongJiaZhen_C_EXPRESSWAY)) {
            return true;
          }
        } else if (Junction::DongJiaZhen_D1_EXPRESSWAY ==
                   ptr_junction->junction().type()) {
          if (IsVehInSpecificJunction(Junction::DongJiaZhen_D1_EXPRESSWAY)) {
            return true;
          }
        } else if (Junction::DongJiaZhen_D2_EXPRESSWAY ==
                   ptr_junction->junction().type()) {
          if (IsVehInSpecificJunction(Junction::DongJiaZhen_D2_EXPRESSWAY)) {
            return true;
          }
        } else {
          continue;
        }
      }
    }
  }
  return false;
}
bool OnLanePlanning::IsNeedStopForTrain() {
  bool need_stop_for_train = false;
  // get adc's junction
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, kJunctionSearchRadius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction)) {
      } else {
        continue;
      }
      if (Junction::BLOCKING_AREA_J1 == ptr_junction->junction().type()) {
        for (auto& blocking_area :
             (*local_view_.blocking_area_response).blocking_areas()) {
          if (blocking_area.blocking_area_response_type() ==
              planning::BLOCKING_AREA_J1) {
            need_stop_for_train = true;
            break;
          }
        }
      }
      if (Junction::BLOCKING_AREA_J2J3 == ptr_junction->junction().type()) {
        for (auto& blocking_area :
             (*local_view_.blocking_area_response).blocking_areas()) {
          if (blocking_area.blocking_area_response_type() ==
              planning::BLOCKING_AREA_J2J3) {
            need_stop_for_train = true;
            break;
          }
        }
      }
      if (Junction::BLOCKING_AREA_J4 == ptr_junction->junction().type()) {
        for (auto& blocking_area :
             (*local_view_.blocking_area_response).blocking_areas()) {
          if (blocking_area.blocking_area_response_type() ==
              planning::BLOCKING_AREA_J4) {
            need_stop_for_train = true;
            break;
          }
        }
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
  return need_stop_for_train;
}
bool OnLanePlanning::IsJunctionContainAdc(
    const VehicleState& vehicle_state,
    const hdmap::JunctionInfo& junction_info, bool use_half_width) const {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(vehicle_state.heading());
  ego_center_map_frame.set_x(vehicle_state.x() + ego_center_map_frame.x());
  ego_center_map_frame.set_y(vehicle_state.y() + ego_center_map_frame.y());
  Box2d adc_box(
      ego_center_map_frame, vehicle_state.heading(),
      use_half_width ? vehicle_param.length() / 2 : vehicle_param.length(),
      use_half_width ? vehicle_param.width() / 2 : vehicle_param.width());
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(adc_box));
}

Status OnLanePlanning::IsAdcInRoad(ReferenceLineInfo* reference_line_info) {
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

void OnLanePlanning::IsAdcInCommonJunction(
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
        AINFO << "IN COMMON JUNCTION";
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
Status OnLanePlanning::IsAdcDeviateLaneDirection(
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

void OnLanePlanning::CreateWheelCraneObstacleWithPerception(
    ReferenceLineInfo* const reference_line_info) {
  // only for J4
  auto stackers_info = injector_->stackers_info();
  // is operation task tpye
  bool is_operation_task_type =
      ((routing::RAILWAY_OPERATIONAREA_DYNAMIC ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::YARD_OPERATIONAREA_DYNAMIC ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_FRONT ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_BACK ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_RIGHT ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_LEFT ==
        local_view_.routing->routing_request().task_type()) ||
       (local_view_.routing->routing_request().task_type() ==
            routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
        local_view_.routing->routing_request().task_type() ==
            routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
        local_view_.routing->routing_request().task_type() ==
            routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
        local_view_.routing->routing_request().task_type() ==
            routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1));
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const double adc_length =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length();
  // destination no overlap with wheelcrane
  // perception wheelcrane in J4
  // get all perception wheelcrane
  auto* path_decision = reference_line_info->path_decision();
  // bool is_need_create_new_stacker = true;
  std::vector<std::pair<std::string, common::math::Vec2d>> new_obstacles;
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (nullptr == obstacle) {
      continue;
    }
    if (obstacle->Perception().type() !=
        perception::PerceptionObstacle::WHEELCRANE) {
      continue;
    }
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    for (auto stacker_boundary : reference_line_info->WheelcraneBoundaryMap()) {
      if (stacker_boundary.first.empty()) {
      }
      const auto& perception_stacker_sl = stacker_boundary.second;
      bool is_lon_overlap =
          !(perception_stacker_sl.start_s() > obs_sl.end_s() ||
            perception_stacker_sl.end_s() < obs_sl.start_s());
      bool is_lat_overlap =
          !(perception_stacker_sl.start_l() > obs_sl.end_l() ||
            perception_stacker_sl.end_l() < obs_sl.start_l());
      if (is_lon_overlap && is_lat_overlap) {
        continue;
      }
    }
    // wheelcrane is in J4
    const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
    common::PointENU obs_point_enu;
    obs_point_enu.set_x(obstacle->Perception().position().x());
    obs_point_enu.set_y(obstacle->Perception().position().y());
    std::vector<JunctionInfoConstPtr> junctions;
    // obs is in junction?
    bool obs_in_J4 = false;
    if (0 == base_map_ptr->GetJunctions(obs_point_enu, kJunctionSearchRadius,
                                        &junctions)) {
      for (const auto& ptr_junction : junctions) {
        if (Junction::BLOCKING_AREA_J4 != ptr_junction->junction().type()) {
          continue;
        }
        const auto& polygon = ptr_junction->polygon();
        if (polygon.HasOverlap(obstacle->PerceptionPolygon())) {
          obs_in_J4 = true;
        } else {
          continue;
        }
      }
    }
    if (!obs_in_J4) {
      continue;
    }
    common::math::Vec2d stacker_point =
        Vec2d(obstacle->Perception().position().x(),
              obstacle->Perception().position().y());
    double stacker_heading = kWheelCraneHeading;
    double stacker_length = kWheelCraneRealLength;
    double stacker_width = kWheelCraneRealWidth;
    std::string stacker_id = "wheelcrane_" + obstacle->Id();
    // center to loc point distance
    double shift_distance = 0.0;
    // Compute the ADC bounding box.
    double dirX = std::cos(kHeadingCenterLine);
    double dirY = std::sin(kHeadingCenterLine);
    double abX = stacker_point.x() - kDistanceCenterLineX;
    double abY = stacker_point.y() - kDistanceCenterLineY;
    double projectionLength = abX * dirX + abY * dirY;
    double projection_x = kDistanceCenterLineX + projectionLength * dirX;
    double projection_y = kDistanceCenterLineY + projectionLength * dirY;
    Box2d stacker_box({projection_x, projection_y}, stacker_heading,
                      stacker_length, stacker_width);
    // reverse so need add -
    Vec2d shift_vec{shift_distance * std::cos(stacker_heading),
                    shift_distance * std::sin(stacker_heading)};
    stacker_box.Shift(shift_vec);
    SLBoundary stacker_sl;
    if (!reference_line.GetSLBoundary(stacker_box, &stacker_sl)) {
      AERROR << "Failed to get stacker sl boundary : " << stacker_id;
      continue;
    }
    common::SLPoint first_sl;
    if (reference_line.XYToSL(
            local_view_.routing->routing_request().waypoint().at(0).pose(),
            &first_sl)) {
      if (first_sl.s() > stacker_sl.start_s() - adc_length &&
          first_sl.s() < stacker_sl.end_s() + adc_length &&
          first_sl.l() > stacker_sl.start_l() &&
          first_sl.l() < stacker_sl.end_l()) {
        continue;
      }
    }

    if (is_operation_task_type) {
      common::SLPoint dest_sl;
      if (reference_line.XYToSL(
              local_view_.routing->routing_request()
                  .waypoint()
                  .at(local_view_.routing->routing_request().waypoint().size() -
                      1)
                  .pose(),
              &dest_sl)) {
        if (dest_sl.s() > stacker_sl.start_s() - adc_length &&
            dest_sl.s() < stacker_sl.end_s() + adc_length &&
            dest_sl.l() > stacker_sl.start_l() &&
            dest_sl.l() < stacker_sl.end_l()) {
          continue;
        }
      }
    }
    common::math::Vec2d center(stacker_box.center_x(), stacker_box.center_y());
    std::pair<std::string, common::math::Vec2d> obs;
    obs.first = stacker_id;
    obs.second = center;
    new_obstacles.emplace_back(obs);
  }
  double stacker_heading = kWheelCraneHeading;
  double stacker_length = kWheelCraneRealLength;
  double stacker_width = kWheelCraneRealWidth;
  for (const auto& obs : new_obstacles) {
    auto stacker_id = obs.first;
    auto center = obs.second;
    auto* obstacle_new = frame_->CreateWheelCraneObstacle(
        reference_line_info, stacker_id, center, stacker_heading,
        stacker_length, stacker_width);
    if (!obstacle_new) {
      AERROR << "Failed to create obstacle [" << obstacle_new->Id() << "]";
      continue;
    }
    if (nullptr == obstacle_new) {
      continue;
    }
    auto* path_obstacle = reference_line_info->AddObstacle(obstacle_new);
    if (!path_obstacle) {
      AERROR << "Failed to create path_obstacle: " << obstacle_new->Id();
      continue;
    }
  }
}
void OnLanePlanning::CreateWheelCraneObstacle(
    ReferenceLineInfo* const reference_line_info) {
  auto stackers_info = injector_->stackers_info();
  // is operation task tpye
  bool is_operation_task_type =
      ((routing::RAILWAY_OPERATIONAREA_DYNAMIC ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::YARD_OPERATIONAREA_DYNAMIC ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_FRONT ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_BACK ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_RIGHT ==
        local_view_.routing->routing_request().task_type()) ||
       (routing::TINY_ADJUSTMENT_LEFT ==
        local_view_.routing->routing_request().task_type()) ||
       (local_view_.routing->routing_request().task_type() ==
            routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
        local_view_.routing->routing_request().task_type() ==
            routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
        local_view_.routing->routing_request().task_type() ==
            routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
        local_view_.routing->routing_request().task_type() ==
            routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1));
  const ReferenceLine& reference_line = reference_line_info->reference_line();
    const double adc_length =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length();
  std::unordered_map<std::string, SLBoundary> wheelcrane_boundary_map;
  for (const auto& stacker_info : stackers_info.stacker_info()) {
    if (stacker_info.stacker_type() != StackerType::WHEELCRANE) {
      continue;
    }
    common::math::Vec2d stacker_point = Vec2d(stacker_info.stacker_point().x(),
                                              stacker_info.stacker_point().y());
    double stacker_heading = stacker_info.stacker_point().heading();
    double stacker_length = kWheelCraneRealLength;
    double stacker_width = kWheelCraneRealWidth;
    std::string stacker_id = stacker_info.stacker_id();
    // center to loc point distance
    double shift_distance = 0.0;
    // Compute the ADC bounding box.
    double dirX = std::cos(kHeadingCenterLine);
    double dirY = std::sin(kHeadingCenterLine);
    double abX = stacker_point.x() - kDistanceCenterLineX;
    double abY = stacker_point.y() - kDistanceCenterLineY;
    double projectionLength = abX * dirX + abY * dirY;
    double projection_x = kDistanceCenterLineX + projectionLength * dirX;
    double projection_y = kDistanceCenterLineY + projectionLength * dirY;
    Box2d stacker_box({projection_x, projection_y}, stacker_heading,
                      stacker_length, stacker_width);
    // reverse so need add -
    Vec2d shift_vec{shift_distance * std::cos(stacker_heading),
                    shift_distance * std::sin(stacker_heading)};
    stacker_box.Shift(shift_vec);
    SLBoundary stacker_sl;
    if (!reference_line.GetSLBoundary(stacker_box, &stacker_sl)) {
      AERROR << "Failed to get stacker sl boundary : " << stacker_id;
      continue;
    }
    wheelcrane_boundary_map.emplace(stacker_id, stacker_sl);
    AINFO << "stacker_sl.start_s = " << stacker_sl.start_s()
          << "  stacker_sl.end_s = " << stacker_sl.end_s()
          << "  stacker_sl.start_l = " << stacker_sl.start_l()
          << "  stacker_sl.end_l = " << stacker_sl.end_l();
    common::SLPoint first_sl;
    if (reference_line.XYToSL(
            local_view_.routing->routing_request().waypoint().at(0).pose(),
            &first_sl)) {
      AINFO << "stacker_sl.start_s = " << stacker_sl.start_s()
            << "  stacker_sl.end_s = " << stacker_sl.end_s()
            << "  stacker_sl.start_l = " << stacker_sl.start_l()
            << "  stacker_sl.end_s = " << stacker_sl.end_l();
      AINFO << "first_s  = " << first_sl.s();
      AINFO << "first_l  = " << first_sl.l();
      if (first_sl.s() > stacker_sl.start_s() - adc_length &&
          first_sl.s() < stacker_sl.end_s() + adc_length &&
          first_sl.l() > stacker_sl.start_l() &&
          first_sl.l() < stacker_sl.end_l()) {
        AINFO << "first point in wheelcrane ,no create obs.";
        continue;
      }
    }

    if (is_operation_task_type) {
      // SLBoundary stacker_sl;
      // if (!reference_line.GetSLBoundary(stacker_box, &stacker_sl)) {
      //   AERROR << "Failed to get stacker sl boundary : " << stacker_id;
      //   continue;
      // }
      common::SLPoint dest_sl;
      if (reference_line.XYToSL(
              local_view_.routing->routing_request()
                  .waypoint()
                  .at(local_view_.routing->routing_request().waypoint().size() -
                      1)
                  .pose(),
              &dest_sl)) {
        AINFO << "stacker_sl.start_s = " << stacker_sl.start_s()
              << "  stacker_sl.end_s = " << stacker_sl.end_s()
              << "  stacker_sl.start_l = " << stacker_sl.start_l()
              << "  stacker_sl.end_s = " << stacker_sl.end_l();
        AINFO<<"dest_s  = "<<dest_sl.s();
         AINFO<<"dest_l  = "<<dest_sl.l();
        if (dest_sl.s() > stacker_sl.start_s() &&
            dest_sl.s() < stacker_sl.end_s() &&
            dest_sl.l() > stacker_sl.start_l() &&
            dest_sl.l() < stacker_sl.end_l()) {
          AINFO << "target point in wheelcrane ,no create obs.";
          continue;
        }
      }
    }

    AINFO << "CREATE WHEELCRANE OBS";
    common::math::Vec2d center(stacker_box.center_x(), stacker_box.center_y());
    auto* obstacle = frame_->CreateWheelCraneObstacle(
        reference_line_info, stacker_id, center, stacker_heading,
        stacker_length, stacker_width);
    if (!obstacle) {
      AERROR << "Failed to create obstacle [" << obstacle->Id() << "]";
      continue;
    }
    auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
    if (!path_obstacle) {
      AERROR << "Failed to create path_obstacle: " << obstacle->Id();
      continue;
    }
    // AINFO << " create path_obstacle: " << obstacle->Id();
  }
    reference_line_info->SetWheelcraneBoundaryMap(wheelcrane_boundary_map);
}

void OnLanePlanning::CreateStackerObstacleWithID(
    ReferenceLineInfo* const reference_line_info) {
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double min_stacker_start_s = std::numeric_limits<double>::max();
  std::string nearst_stacker_id = "";
  double nearst_stacker_heading = 0.0;
  double nearst_stacker_speed = 0.0;
  auto stackers_info = injector_->stackers_info();
  std::unordered_map<std::string, SLBoundary> stacker_boundary_map;
  SLBoundary nearst_stacker_sl;
  common::math::Vec2d nearst_center(0.0, 0.0);
  // AINFO<<"adc_sl.start_s = "<<adc_sl.start_s()<<"   adc_sl.end_s =
  // "<<adc_sl.end_s()
  // << "   adc_sl.start_l = "<<adc_sl.start_l()<<"   adc_sl.end_l =
  // "<<adc_sl.end_l();
  bool is_get_consider_stacker = false;
  // double stacker_speed = 0.0;
  for (const auto& stacker_info : stackers_info.stacker_info()) {
        if (stacker_info.stacker_type() == StackerType::WHEELCRANE) {
      continue;
    }
    common::math::Vec2d stacker_point = Vec2d(stacker_info.stacker_point().x(),
                                              stacker_info.stacker_point().y());
    const ReferenceLine& reference_line = reference_line_info->reference_line();
    double stacker_heading = stacker_info.stacker_point().heading();
    double stacker_speed = stacker_info.speed();
    double stacker_length = kStackerRealLength;
    double stacker_width = kStackerRealWidth;
    std::string stacker_id = stacker_info.stacker_id();
    // center to loc point distance
    double shift_distance = stacker_length * 0.5 - kStackerFrontToLocPoint;
    // Compute the ADC bounding box.
    Box2d stacker_box({stacker_point.x(), stacker_point.y()}, stacker_heading,
                      stacker_length, stacker_width);
    // reverse so need add -
    Vec2d shift_vec{-shift_distance * std::cos(stacker_heading),
                    -shift_distance * std::sin(stacker_heading)};
    stacker_box.Shift(shift_vec);
    SLBoundary stacker_sl;
    if (!reference_line.GetSLBoundary(stacker_box, &stacker_sl)) {
      AERROR << "Failed to get stacker sl boundary : " << stacker_id;
      continue;
    }
    AINFO<<"stacker_id = "<<stacker_id;
    AINFO<<"stacker_heading = "<<stacker_heading;
    AINFO << "stacker_sl.start_s = " << stacker_sl.start_s()
          << "  stacker_sl.end_s = " << stacker_sl.end_s()
          << "  stacker_sl.start_l = " << stacker_sl.start_l()
          << "  stacker_sl.end_l = " << stacker_sl.end_l();
    stacker_boundary_map.emplace("stacker_"+stacker_id, stacker_sl);
    if (stacker_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
      continue;
    }
    if (stacker_sl.start_s()- adc_end_s > kStartConsiderStackerDistance) {
      continue;
    }

    double consider_start_l = adc_sl.start_l() - kStackerConsiderDistance;
    double consider_end_l = adc_sl.end_l() + kStackerConsiderDistance;
    bool is_need_consider_stacker = !(stacker_sl.start_l() > consider_end_l ||
                                      consider_start_l > stacker_sl.end_l());
    ADEBUG << "need consider stacker " << stacker_id << "   "
          << is_need_consider_stacker;
    if (is_need_consider_stacker) {
      if (stacker_sl.start_s() < min_stacker_start_s) {
        min_stacker_start_s = stacker_sl.start_s();
        nearst_stacker_id = stacker_id;
        nearst_stacker_heading = stacker_heading;
        nearst_stacker_speed = stacker_speed;
        is_get_consider_stacker = true;
        nearst_stacker_sl = stacker_sl;
        nearst_center.set_x(stacker_box.center_x());
        nearst_center.set_y(stacker_box.center_y());
        // stacker_speed = stacker_info.speed();
      }
    }
  }
  reference_line_info->SetStackerBoundaryMap(stacker_boundary_map);
  if (!is_get_consider_stacker) {
    AINFO << "no get consider stacker";
    return;
  }

  ADEBUG<<"CREATE STACKER OBS with id";
  nearst_stacker_id = "stacker_" + nearst_stacker_id;
  ADEBUG << "nearst_stacker_id = " << nearst_stacker_id;
  auto* obstacle = frame_->CreateStackerObstacleWithID(
      reference_line_info, nearst_stacker_id, nearst_center,
      nearst_stacker_heading, kStackerRealLength, kStackerRealWidth,nearst_stacker_speed);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << obstacle->Id() << "]";
    return;
  }
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << obstacle->Id();
    return;
  }
  AINFO<<"CREATE OBS ID = "<<obstacle->Id();
}

void OnLanePlanning::CreateStackerObstacle(
    ReferenceLineInfo* const reference_line_info) {
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  double min_stacker_start_s = std::numeric_limits<double>::max();
  std::string nearst_stacker_id = "";
  double nearst_stacker_heading = 0.0;
  auto stackers_info = injector_->stackers_info();
  std::unordered_map<std::string, SLBoundary> stacker_boundary_map;
  SLBoundary nearst_stacker_sl;
  common::math::Vec2d nearst_center(0.0, 0.0);
  // AINFO<<"adc_sl.start_s = "<<adc_sl.start_s()<<"   adc_sl.end_s =
  // "<<adc_sl.end_s()
  // << "   adc_sl.start_l = "<<adc_sl.start_l()<<"   adc_sl.end_l =
  // "<<adc_sl.end_l();
  bool is_get_consider_stacker = false;
  for (const auto& stacker_info : stackers_info.stacker_info()) {
    common::math::Vec2d stacker_point = Vec2d(stacker_info.stacker_point().x(),
                                              stacker_info.stacker_point().y());
    const ReferenceLine& reference_line = reference_line_info->reference_line();
    double stacker_heading = stacker_info.stacker_point().heading();
    double stacker_length = kStackerRealLength;
    double stacker_width = kStackerRealWidth;
    std::string stacker_id = stacker_info.stacker_id();
    // center to loc point distance
    double shift_distance = stacker_length * 0.5 - kStackerFrontToLocPoint;
    // Compute the ADC bounding box.
    Box2d stacker_box({stacker_point.x(), stacker_point.y()}, stacker_heading,
                      stacker_length, stacker_width);
    // reverse so need add -
    Vec2d shift_vec{-shift_distance * std::cos(stacker_heading),
                    -shift_distance * std::sin(stacker_heading)};
    stacker_box.Shift(shift_vec);
    SLBoundary stacker_sl;
    if (!reference_line.GetSLBoundary(stacker_box, &stacker_sl)) {
      AERROR << "Failed to get stacker sl boundary : " << stacker_id;
      continue;
    }
    AINFO << "stacker_sl.start_s = " << stacker_sl.start_s()
          << "  stacker_sl.end_s = " << stacker_sl.end_s()
          << "  stacker_sl.start_l = " << stacker_sl.start_l()
          << "  stacker_sl.end_s = " << stacker_sl.end_l();
    stacker_boundary_map.emplace(stacker_id, stacker_sl);
    if (stacker_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
      continue;
    }
    if (stacker_sl.start_s()- adc_end_s > kStartConsiderStackerDistance) {
      continue;
    }

    double consider_start_l = adc_sl.start_l() - kStackerConsiderDistance;
    double consider_end_l = adc_sl.end_l() + kStackerConsiderDistance;
    bool is_need_consider_stacker = !(stacker_sl.start_l() > consider_end_l ||
                                      consider_start_l > stacker_sl.end_l());
    AINFO << "need consider stacker " << stacker_id << "   "
          << is_need_consider_stacker;
    if (is_need_consider_stacker) {
      if (stacker_sl.start_s() < min_stacker_start_s) {
        min_stacker_start_s = stacker_sl.start_s();
        nearst_stacker_id = stacker_id;
        nearst_stacker_heading = stacker_heading;
        is_get_consider_stacker = true;
        nearst_stacker_sl = stacker_sl;
        nearst_center.set_x(stacker_point.x());
        nearst_center.set_y(stacker_point.y());
      }
    }
  }
  if (!is_get_consider_stacker) {
    AINFO << "no get consider stacker";
    return;
  }
  // check is overlap
  auto* path_decision = reference_line_info->path_decision();
  // bool is_need_create_new_stacker = true;
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Perception().type() ==
            perception::PerceptionObstacle::STACKER &&
        obstacle->Perception().type() ==
            perception::PerceptionObstacle::FORKLIFT_STACKER) {
      const auto& perception_stacker_sl = obstacle->PerceptionSLBoundary();
      // lon and lat overlap
      bool is_lon_overlap =
          !(perception_stacker_sl.start_s() > nearst_stacker_sl.end_s() ||
            perception_stacker_sl.end_s() < nearst_stacker_sl.start_s());
      bool is_lat_overlap =
          !(perception_stacker_sl.start_l() > nearst_stacker_sl.end_l() ||
            perception_stacker_sl.end_l() < nearst_stacker_sl.start_l());
      if (is_lon_overlap && is_lat_overlap) {
        AINFO << "perception stacker and loc stacker is overlap,no create new "
                 "stacker";
        // is_need_create_new_stacker = false;
        return;
      }
    }
  }
  AINFO<<"CREATE STACKER OBS";
  auto* obstacle = frame_->CreateStackerObstacle(
      reference_line_info, nearst_stacker_id, nearst_center,
      nearst_stacker_heading, kStackerRealLength, kStackerRealWidth);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << obstacle->Id() << "]";
    return;
  }
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << obstacle->Id();
    return;
  }
}

void OnLanePlanning::AddStackerObstacle(
    ReferenceLineInfo* const reference_line_info){
      AINFO<<"keep satcker still";
    if(nullptr == injector_){
      return;
    }
    
    if(injector_->target_stacker_info_.second.first != ""){
      double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
    double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    const auto &adc_sl = reference_line_info->AdcSlBoundary();
    std::string stacker_id = "";
    double min_stacker_start_s = std::numeric_limits<double>::max();
    std::string nearst_stacker_id = "";
    bool is_get_consider_stacker = false;
    // const ReferenceLine &reference_line = reference_line_info->reference_line();
    auto* path_decision = reference_line_info->path_decision();
    for (const auto *obstacle : path_decision->obstacles().Items()) {
      if (obstacle->Perception().type() !=
              perception::PerceptionObstacle::STACKER &&
          obstacle->Perception().type() !=
              perception::PerceptionObstacle::FORKLIFT_STACKER&&
            obstacle->Perception().type() !=
              perception::PerceptionObstacle::WHEELCRANE) {
        continue;
      }
      stacker_id = obstacle->Id();
      // AINFO << "obstacle_id[<< " << obstacle->Id() << "] type["
      //       << PerceptionObstacle_Type_Name(obstacle->Perception().type()) << "]";
      const auto &stacker_sl = obstacle->PerceptionSLBoundary();
      bool is_wheel_crane = obstacle->Perception().type() ==
                            perception::PerceptionObstacle::WHEELCRANE;
      double consider_distance = kStartConsiderStackerDistance;
      if (is_wheel_crane) {
        consider_distance = FLAGS_wheelcrane_consider_distance;
      }
      if (stacker_sl.start_s() - adc_end_s > consider_distance) {
        // AINFO << "large distance ,no consider";
        continue;
      }
      if (stacker_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
        // AINFO << "back stacker ,no consider";
        continue;
      }
      double consider_start_l = adc_sl.start_l() - kStackerConsiderDistance;
      double consider_end_l = adc_sl.end_l() + kStackerConsiderDistance;
      bool is_need_consider_stacker = !(stacker_sl.start_l() > consider_end_l ||
                                        consider_start_l > stacker_sl.end_l());
      if (is_need_consider_stacker) {
        // AINFO << "need consider stacker " << stacker_id << "   "
        //       << is_need_consider_stacker;
        if (stacker_sl.start_s() < min_stacker_start_s) {
          min_stacker_start_s = stacker_sl.start_s();
          nearst_stacker_id = stacker_id;
          is_get_consider_stacker = true;
        }
      }
    }    
    if(is_get_consider_stacker){
       AINFO<<"has need consider stacker";
      injector_->stacker_change_times_ = 0;
      return;
    }else{
      // AINFO<<"no need consider stacker, need create new one";
      injector_->stacker_change_times_++;
    }
     AINFO<<"injector_->stacker_change_times_ = "<<injector_->stacker_change_times_;
    int stacker_disappear_count = kStackerDisappear;
    if(injector_->pass_stacker_response().pass_stacker_response_type() ==
      planning::PassStackerResponseType::PASS){
        stacker_disappear_count = 10;
    }else{

    }

    if(injector_->stacker_change_times_ > stacker_disappear_count ){
       AINFO<<"stacker disappear more than 5 frames, no need create new one";
      return;
    }




    const auto& stacker_polygon = injector_->target_stacker_info_.second.second;
    SLBoundary target_tacker_slboundary;
    if (!reference_line_info->reference_line().GetSLBoundary(stacker_polygon, &target_tacker_slboundary)) {
      AINFO << "no get sl boundary stacker";
      return;
    }
    double center_s = (target_tacker_slboundary.start_s() + target_tacker_slboundary.end_s()) * 0.5;
    double center_l = (target_tacker_slboundary.start_l() + target_tacker_slboundary.end_l()) * 0.5;
    const auto& reference_point =
        reference_line_info->reference_line().GetNearestReferencePoint(center_s);
    common::SLPoint sl;
    sl.set_s(center_s);
    sl.set_l(center_l);
    common::math::Vec2d xy;
    if(!reference_line_info->reference_line().SLToXY(sl, &xy)){
      return;
    }
    common::math::Vec2d center(xy.x(), xy.y());
    double heading = reference_point.heading();
    double length = std::max(target_tacker_slboundary.end_s()-target_tacker_slboundary.start_s(),kStackerLength);
    double width = std::max(target_tacker_slboundary.end_l()-target_tacker_slboundary.start_l(),kStackerLength);
    std::string obstacle_id = injector_->target_stacker_info_.second.first;
    if(injector_->target_stacker_info_.second.first.find("new") == std::string::npos){
      obstacle_id = injector_->target_stacker_info_.second.first+"new";
    }
    AINFO<<"CreateStackerObstacle";
    auto* obstacle = frame_->CreateStackerObstacle(
        reference_line_info, obstacle_id, center, heading, length, width);
    if (!obstacle) {
      AERROR << "Failed to create obstacle [" << obstacle_id << "]";
      return;
    }
    auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
    if (!path_obstacle) {
      AERROR << "Failed to create path_obstacle: " << obstacle_id;
      return;
    }
    }
    return;

}

void OnLanePlanning::AddFlowerBedObstacle(
    ReferenceLineInfo* const reference_line_info) {
  // TODO(zongxingguo): Add strategy for samll diatance.
  AINFO << "========AddFlowerBedObstacle=====";
  for (int i = 0; i < 6; ++i) {
    auto obstacle_id = "FLOWER_BED_0";
    double x = -2605.33;
    double y = 2184.96;
    if (i == 0) {
      x = -2605.33;
      y = 2184.96;
      obstacle_id = "FLOWER_BED_0";
    } else if (i == 1) {
      x = -2724.49;
      y = 2202.82;
      obstacle_id = "FLOWER_BED_1";
    } else if (i == 2) {
      x = -2843.53;
      y = 2220.86;
      obstacle_id = "FLOWER_BED_2";
    } else if (i == 3) {
      x = -2950.93;
      y = 2237.02;
      obstacle_id = "FLOWER_BED_3";
    } else if (i == 4) {
      x = -3075.58;
      y = 2255.72;
      obstacle_id = "FLOWER_BED_4";
    }
    common::math::Vec2d center(x, y);
    double heading = -0.12;
    double length = 15.0;
    double width = 4.0;
    auto* obstacle = frame_->CreateStaticObstacle(
        reference_line_info, obstacle_id, center, heading, length, width);
    if (!obstacle) {
      AERROR << "Failed to create obstacle [" << obstacle_id << "]";
      return;
    }
    auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
    if (!path_obstacle) {
      AERROR << "Failed to create path_obstacle: " << obstacle_id;
      return;
    }
  }
}

void OnLanePlanning::AddModifiedObstacle(
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
        double velocity_heading = mutable_obstacle->SpeedHeading();
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
          modified_obs_path_point->set_theta(velocity_heading);
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

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

  if (nullptr == frame_) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  frame_->SetPlanningContext(injector_->planning_context());
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    const std::string msg = "Failed to create reference line";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  DCHECK_EQ(reference_lines.size(), segments.size());

  auto forward_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto& ref_line : reference_lines) {
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          FLAGS_look_backward_distance, forward_limit) &&
        !injector_->need_to_rescue()) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  if (!wait_flag_ && !injector_->need_to_rescue()) {
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

  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());
  if (!status.ok() &&
      !injector_->need_to_rescue()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

void OnLanePlanning::GenerateSlowBreakingTrajectory(
    ADCTrajectory* ptr_trajectory_pb) {
  ptr_trajectory_pb->clear_trajectory_point();

  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double adc_speed = vehicle_state.linear_velocity();
  const double max_t = FLAGS_fallback_total_time;        // 3.0 s
  const double unit_t = FLAGS_fallback_time_unit;        // 0.1 s
  const double decel_max = FLAGS_fallback_deceleration;  //-2.0
  double break_total_time = std::sqrt(std::fabs(adc_speed / decel_max));
  const double min_t = kMinBreakingTime;
  const double heading = vehicle_state.heading();
  AINFO << "break_total_time = " << break_total_time;
  if (break_total_time < min_t || adc_speed < kStopedSpeed) {
    AINFO << "-------stop---------";
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
    return;
  } else if (break_total_time >= min_t && break_total_time < max_t) {
    for (double t = 0.0; t < break_total_time; t += unit_t) {
      TrajectoryPoint tp;
      auto* path_point = tp.mutable_path_point();
      double delta_s = t * adc_speed + 0.5 * decel_max * t * t;
      path_point->set_x(vehicle_state.x() + delta_s * std::cos(heading));
      path_point->set_y(vehicle_state.y() + delta_s * std::sin(heading));
      path_point->set_theta(vehicle_state.heading());
      path_point->set_s(delta_s);
      tp.set_v(adc_speed + decel_max * t);
      tp.set_a(decel_max);
      tp.set_relative_time(t);
      auto next_point = ptr_trajectory_pb->add_trajectory_point();
      next_point->CopyFrom(tp);
    }
    double delta_s = break_total_time * adc_speed +
                     0.5 * decel_max * break_total_time * break_total_time;
    double x = vehicle_state.x() + delta_s * std::cos(heading);
    double y = vehicle_state.y() + delta_s * std::sin(heading);
    for (double t = break_total_time; t < max_t; t += unit_t) {
      TrajectoryPoint tp;
      auto* path_point = tp.mutable_path_point();
      path_point->set_x(x);
      path_point->set_y(y);
      path_point->set_theta(vehicle_state.heading());
      path_point->set_s(delta_s);
      tp.set_v(0.0);
      tp.set_a(0.0);
      tp.set_relative_time(t);
      auto next_point = ptr_trajectory_pb->add_trajectory_point();
      next_point->CopyFrom(tp);
    }
    return;
  } else {
    AINFO << "-------break---------";
    for (double t = 0.0; t < break_total_time; t += unit_t) {
      TrajectoryPoint tp;
      auto* path_point = tp.mutable_path_point();
      double delta_s = t * adc_speed + 0.5 * decel_max * t * t;
      path_point->set_x(vehicle_state.x() + delta_s * std::cos(heading));
      path_point->set_y(vehicle_state.y() + delta_s * std::sin(heading));
      path_point->set_theta(vehicle_state.heading());
      path_point->set_s(delta_s);
      tp.set_v(adc_speed + decel_max * t);
      tp.set_a(decel_max);
      tp.set_relative_time(t);
      auto next_point = ptr_trajectory_pb->add_trajectory_point();
      next_point->CopyFrom(tp);
    }
    return;
  }
}

bool OnLanePlanning::HandleOppositeDirectionRouting(
    Status& status, const double start_timestamp,
    const routing::RoutingResponse* routing_candidate,
    const bool is_different_routing,
    const VehicleState& previous_vehicle_state,
    const VehicleState& vehicle_state,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (routing_candidate == nullptr || !is_different_routing) {
    return false;
  }

  const auto& new_routing = *routing_candidate;
  if (!IsSameOppositeDirectionRouting(new_routing)) {
    is_opposite_direction_routing_braking_ = false;
    is_opposite_direction_routing_brake_done_ = false;
    opposite_direction_routing_seq_num_ = new_routing.header().sequence_num();
    opposite_direction_routing_timestamp_ = new_routing.header().timestamp_sec();
  }

  if (is_opposite_direction_routing_brake_done_) {
    return false;
  }

  if (!is_opposite_direction_routing_braking_) {
    const auto& heading_vehicle_state =
        previous_vehicle_state.timestamp() > 0.0 ? previous_vehicle_state
                                                 : vehicle_state;
    if (!IsOppositeDirectionNewRouting(heading_vehicle_state, new_routing)) {
      return false;
    }
    is_opposite_direction_routing_braking_ = true;
  }

  if (std::fabs(vehicle_state.linear_velocity()) <=
      KStopSpeedForReverseRouting) {
    AINFO << "opposite-direction routing braking finished. linear_velocity="
          << vehicle_state.linear_velocity();
    is_opposite_direction_routing_braking_ = false;
    is_opposite_direction_routing_brake_done_ = true;
    const std::string msg =
        "keep current gear for one cycle after stopping before applying "
        "opposite-direction routing.";
    injector_->is_new_routing_for_replan_ = false;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(Chassis::GEAR_REVERSE ==
                                                  vehicle_gear_);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
    ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_BRAKING);
    return true;
  }

  const std::string msg =
      "need to brake before applying opposite-direction routing.";
  AINFO << msg << " linear_velocity=" << vehicle_state.linear_velocity();
  injector_->is_new_routing_for_replan_ = false;
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_not_ready()
      ->set_reason(msg);
  status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
  ptr_trajectory_pb->set_gear(vehicle_gear_);
  ptr_trajectory_pb->set_is_backward_trajectory(Chassis::GEAR_REVERSE ==
                                                vehicle_gear_);
  FillPlanningPb(start_timestamp, ptr_trajectory_pb);
  GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
  ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_BRAKING);
  return true;
}

bool OnLanePlanning::IsSameOppositeDirectionRouting(
    const routing::RoutingResponse& routing) const {
  return routing.header().sequence_num() == opposite_direction_routing_seq_num_ &&
         std::fabs(routing.header().timestamp_sec() -
                   opposite_direction_routing_timestamp_) < kEpsilon;
}

bool OnLanePlanning::IsOppositeDirectionNewRouting(
    const VehicleState& vehicle_state,
    const routing::RoutingResponse& new_routing) const {
  double routing_heading = 0.0;
  const auto& routing_request = new_routing.routing_request();
  if (routing_request.waypoint_size() > 0 &&
      routing_request.waypoint(0).has_heading()) {
    routing_heading = routing_request.waypoint(0).heading();
  } else {
    if (new_routing.road_size() == 0 ||
        new_routing.road(0).passage_size() == 0 ||
        new_routing.road(0).passage(0).segment_size() == 0) {
      return false;
    }
    const auto& first_segment = new_routing.road(0).passage(0).segment(0);
    if (first_segment.id().empty()) {
      return false;
    }
    auto first_lane =
        hdmap_->GetLaneById(hdmap::MakeMapId(first_segment.id()));
    if (nullptr == first_lane) {
      return false;
    }
    const double lane_s =
        first_segment.has_start_s() ? first_segment.start_s() : 0.0;
    routing_heading = first_lane->Heading(
        std::min(std::max(lane_s, 0.0), first_lane->total_length()));
  }

  double vehicle_heading = common::math::NormalizeAngle(vehicle_state.heading());
  if (vehicle_state.linear_velocity() < 0.0) {
    vehicle_heading = common::math::NormalizeAngle(vehicle_heading + M_PI);
  }
  const double angle_diff =
      std::fabs(common::math::AngleDiff(vehicle_heading, routing_heading));
  AINFO << "new routing heading check: vehicle_heading=" << vehicle_heading
        << " routing_heading=" << routing_heading
        << " angle_diff=" << angle_diff
        << " linear_velocity=" << vehicle_state.linear_velocity();
  return angle_diff > kOppositeRoutingAngleThreshold;
}

bool OnLanePlanning::ExpandRoutingSegmentsToVehicleState(
    const VehicleState& vehicle_state,
    routing::RoutingResponse* routing) const {
  if (nullptr == routing || nullptr == hdmap_) {
    return false;
  }

  const Vec2d adc_point(vehicle_state.x(), vehicle_state.y());
  double best_distance = std::numeric_limits<double>::infinity();
  double best_segment_gap = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  routing::LaneSegment* best_segment = nullptr;
  hdmap::LaneInfoConstPtr best_lane = nullptr;

  for (int road_index = 0; road_index < routing->road_size(); ++road_index) {
    auto* road = routing->mutable_road(road_index);
    for (int passage_index = 0; passage_index < road->passage_size();
         ++passage_index) {
      auto* passage = road->mutable_passage(passage_index);
      for (int lane_index = 0; lane_index < passage->segment_size();
           ++lane_index) {
        auto* segment = passage->mutable_segment(lane_index);
        if (segment->id().empty()) {
          continue;
        }
        auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment->id()));
        if (lane == nullptr) {
          continue;
        }

        double lane_s = 0.0;
        double lane_l = 0.0;
        if (!lane->GetProjection(adc_point, &lane_s, &lane_l)) {
          continue;
        }
        if (lane_s < -kEpsilon || lane_s > lane->total_length() + kEpsilon) {
          continue;
        }

        const double segment_start_s =
            segment->has_start_s() ? segment->start_s() : 0.0;
        const double segment_end_s =
            segment->has_end_s() ? segment->end_s() : lane->total_length();
        double segment_gap = 0.0;
        if (lane_s < segment_start_s) {
          segment_gap = segment_start_s - lane_s;
        } else if (lane_s > segment_end_s) {
          segment_gap = lane_s - segment_end_s;
        }
        const double distance = std::fabs(lane_l);
        if (segment_gap < best_segment_gap ||
            (std::fabs(segment_gap - best_segment_gap) < kEpsilon &&
             distance < best_distance)) {
          best_segment_gap = segment_gap;
          best_distance = distance;
          best_s = std::min(std::max(lane_s, 0.0), lane->total_length());
          best_segment = segment;
          best_lane = lane;
        }
      }
    }
  }

  if (nullptr == best_segment || nullptr == best_lane) {
    return false;
  }

  const double original_start_s =
      best_segment->has_start_s() ? best_segment->start_s() : 0.0;
  const double original_end_s =
      best_segment->has_end_s() ? best_segment->end_s()
                                : best_lane->total_length();
  bool updated = false;
  if (best_s < original_start_s) {
    best_segment->set_start_s(best_s);
    updated = true;
  }
  if (best_s > original_end_s) {
    best_segment->set_end_s(best_s);
    updated = true;
  }

  if (updated) {
    AINFO << "Expand routing segment for stopped vehicle. lane_id="
          << best_segment->id() << " segment_s=[" << original_start_s << ", "
          << original_end_s << "] new_segment_s=["
          << (best_segment->has_start_s() ? best_segment->start_s() : 0.0)
          << ", "
          << (best_segment->has_end_s() ? best_segment->end_s()
                                        : best_lane->total_length())
          << "] vehicle_s=" << best_s;
  }
  return updated;
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) {
  ptr_trajectory_pb->clear_trajectory_point();

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

bool OnLanePlanning::JudgeCarInDeadEndJunction(
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

bool OnLanePlanning::UpdateTopNullFlagsRuntime(bool is_top_null,
                                               int action_type,
                                               double reverse_distance) {
  if (action_type < 0 || action_type > 3) {
    AERROR << "invalid top_null action_type: " << action_type
           << ", expected in [0, 3].";
    return false;
  }
  if (reverse_distance < 0.0) {
    AERROR << "invalid top_null reverse_distance: " << reverse_distance
           << ", expected >= 0.0.";
    return false;
  }

  const char* bool_value = is_top_null ? "true" : "false";
  const std::string action_type_str = std::to_string(action_type);
  const std::string reverse_distance_str = std::to_string(reverse_distance);

  google::SetCommandLineOption("top_null_default_is_top_bull", bool_value);
  google::SetCommandLineOption("top_bull_default_action_type",
                               action_type_str.c_str());
  google::SetCommandLineOption("top_bull_default_reverse_distance",
                               reverse_distance_str.c_str());

  ADEBUG << "Updated top_null runtime flags: is_top_null=" << bool_value
        << ", action_type=" << action_type_str
        << ", reverse_distance=" << reverse_distance_str;
  return true;
}

void OnLanePlanning::TryReloadPlanningConfIfChanged() {
  constexpr char kPlanningConfPath[] =
      "/century/modules/planning/conf/planning.conf";
  constexpr double kReloadCheckIntervalSec = 0.5;
  const double now_sec = Clock::NowInSeconds();
  if (now_sec - last_planning_conf_check_time_sec_ < kReloadCheckIntervalSec) {
    return;
  }
  last_planning_conf_check_time_sec_ = now_sec;

  struct stat file_stat {};
  if (stat(kPlanningConfPath, &file_stat) != 0) {
    AWARN << "Failed to stat planning conf: " << kPlanningConfPath;
    return;
  }

  const int64_t current_mtime_sec = static_cast<int64_t>(file_stat.st_mtime);
  if (last_planning_conf_mtime_sec_ < 0) {
    last_planning_conf_mtime_sec_ = current_mtime_sec;
    return;
  }

  if (current_mtime_sec == last_planning_conf_mtime_sec_) {
    return;
  }

  google::SetCommandLineOption("flagfile", kPlanningConfPath);
  last_planning_conf_mtime_sec_ = current_mtime_sec;
  // Force re-apply runtime flags in this frame after flagfile reloaded.
  top_null_runtime_initialized_ = false;
  AINFO << "Detected planning.conf update, reloaded flagfile: "
        << kPlanningConfPath;
}

bool OnLanePlanning::CheckTopBullRerouting() {
  if (frame_ == nullptr || injector_ == nullptr ||
      injector_->planning_context() == nullptr ||
      local_view_.routing == nullptr) {
    return false;
  }
  const auto& routing_request = local_view_.routing->routing_request();
  const auto& rerouting_info = routing_request.rerouting_info();
  if (rerouting_info.reverse_rerouting().is_rerouting()) {
    return false;
  }
  const auto& planning_status =
      injector_->planning_context()->planning_status();
  if (planning_status.has_rerouting() &&
      planning_status.rerouting().has_routing_request() &&
      planning_status.rerouting()
          .routing_request()
          .rerouting_info()
          .reverse_rerouting()
          .is_rerouting()) {
    ADEBUG << "TopBull: pending reverse rerouting request already exists, skip "
             "duplicated top bull rerouting.";
    return false;
  }
  if (!planning_status.has_top_bull()) {
    return false;
  }
  const auto& top_bull = planning_status.top_bull();
  if (!top_bull.is_in_top_bull() || top_bull.ego_complete_action()) {
    return false;
  }
  if (top_bull.action_type() != planning::TopBullStatus::REVERSE &&
      top_bull.action_type() != planning::TopBullStatus::BORROW) {
    return false;
  }
  if (top_bull.action_type() == planning::TopBullStatus::BORROW) {
    const auto& borrow_response = injector_->borrow_response();
    if (borrow_response.has_response() &&
        borrow_response.response_type() == planning::ResponseType::ACCEPT &&
        borrow_response.block_obs_id() == top_bull.blocking_igv_id()) {
      ADEBUG << "TopBull: borrow already accepted for blocking igv "
            << top_bull.blocking_igv_id()
            << ", skip reverse rerouting to avoid repeated reverse.";
      return false;
    }
  }
  ADEBUG << "CheckTopBullRerouting, action_type=" << top_bull.action_type()
        << " reverse_distance=" << top_bull.reverse_distance()
        << " blocking_igv_id=" << top_bull.blocking_igv_id();
  if (!frame_->Rerouting(injector_->planning_context(), ReroutingType::REVERSE,
                         top_bull.reverse_distance())) {
    AERROR << "TopBull: failed to send rerouting request";
    return false;
  }
  return true;
}

bool OnLanePlanning::CheckAndCleanupTopBullReverseState(
    ADCTrajectory* const ptr_trajectory_pb) {
  if (local_view_.routing == nullptr) {
    return false;
  }
  if (!local_view_.routing->routing_request()
           .rerouting_info()
           .reverse_rerouting()
           .is_rerouting()) {
    AINFO << "TopBull: not in reverse rerouting, skip top bull reverse state "
             "cleanup.";
    return false;
  }

  auto* planning_status =
      injector_->planning_context()->mutable_planning_status();
  if (nullptr == planning_status || !planning_status->has_top_bull()) {
    ADEBUG << "TopBull: no top bull status, skip top bull reverse state cleanup.";
    return false;
  }

  const auto& top_bull = planning_status->top_bull();
  if (!top_bull.is_in_top_bull()) {
    ADEBUG << "TopBull: not in top bull, skip top bull reverse state cleanup.";
    return false;
  }

  const auto action_type = top_bull.action_type();
  if (action_type != planning::TopBullStatus::REVERSE &&
      action_type != planning::TopBullStatus::BORROW) {
    ADEBUG << "TopBull: action type is not reverse or borrow, skip top bull "
              "reverse state cleanup.";
    return false;
  }
  if (action_type == planning::TopBullStatus::REVERSE) {
    planning_status->mutable_top_bull()->set_ego_complete_action(true);
  }
  ADEBUG<<"planning_status->mutable_top_bull()->set_ego_complete_action(true)";
  injector_->set_use_reverse_trajectory(false);
  injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);

  if (action_type == planning::TopBullStatus::BORROW) {
    need_reset_borrow_state_ = true;
    ADEBUG << "clear borrow info ,no need reverse trajectory,wait for next "
              "frame reset borrow state";
    ADEBUG << "auto borrow for only one obs.";
    planning::BorrowResponse borrow_response;
    if (ptr_trajectory_pb != nullptr) {
      ptr_trajectory_pb->set_borrow_request(false);
    }
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id(top_bull.blocking_igv_id());
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
  }
  return true;
}

bool OnLanePlanning::CalTopBullReverseBorrowPoint(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto planning_status = injector_->planning_context()->mutable_planning_status();
  if (nullptr == planning_status) {
    return false;
  }
  if (!planning_status->has_top_bull()) {
    is_first_enter_top_bull_ = false;
    return false;
  }

  const auto &top_bull = planning_status->top_bull();
  if (!top_bull.is_in_top_bull()) {
    is_first_enter_top_bull_ = false;
  }
  if (!top_bull.is_in_top_bull() ||
      top_bull.action_type() != planning::TopBullStatus::BORROW) {
    return false;
  }
  planning::ReferenceLineInfo *best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto &reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() && reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == best_ref_info) {
    return false;
  }
  ADEBUG<<" injector_->borrow_response().block_obs_id() = "<< injector_->borrow_response().block_obs_id();
  ADEBUG<<" top_bull.blocking_igv_id() = "<< top_bull.blocking_igv_id();
  if (injector_->borrow_response().has_response() &&
      injector_->borrow_response().response_type() ==
          planning::ResponseType::ACCEPT &&
      injector_->borrow_response().block_obs_id() ==
          top_bull.blocking_igv_id()) {
    ADEBUG << "already reset borrow state,and auto borrow.";
    const std::string obs_id = injector_->borrow_response().block_obs_id();
    const auto& igv_boundary_map = best_ref_info->IgvBoundaryMap();
    auto obs_sl_it = igv_boundary_map.find(obs_id);
    if (obs_sl_it == igv_boundary_map.end()) {
      AWARN << "TopBull: cannot find blocking obs in IgvBoundaryMap, obs_id="
            << obs_id;
      return false;
    }
    double obs_end_s = obs_sl_it->second.end_s();
    double adc_start_s = best_ref_info->AdcSlBoundary().start_s();
    if (obs_end_s < adc_start_s) {
      planning_status->mutable_top_bull()->set_ego_complete_action(true);
      AINFO << "TopBull: set ego_complete_action true for " << obs_id
            << " obs_end_s=" << obs_end_s << " adc_start_s=" << adc_start_s;
    }

    return false;
  }

  double reverse_distance = top_bull.reverse_distance();
  if (reverse_distance <= kTopBullArriveDistance) {
    need_reset_borrow_state_ = true;
    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    ADEBUG << "clear borrow info ,no need reverse trajectory,wait for next "
             "frame reset borrow state";
    ADEBUG << "auto borrow for only one obs.";
    planning::BorrowResponse borrow_response;
    ptr_trajectory_pb->set_borrow_request(false);
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id(top_bull.blocking_igv_id());
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
    return false;
  }
  const auto &adc_sl = best_ref_info->AdcSlBoundary();
  double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  common::SLPoint reverse_stop_sl;
  reverse_stop_sl.set_s(center_s - reverse_distance);
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  reverse_stop_sl.set_l(center_l);

  common::math::Vec2d reverse_stop_xy;
  const auto &reference_line = best_ref_info->reference_line();
  if (!reference_line.SLToXY(reverse_stop_sl, &reverse_stop_xy)) {
    return false;
  }

  if(!injector_->use_reverse_trajectory()&&!is_first_enter_top_bull_){
  common::PointENU reverse_stop_point;
  reverse_stop_point.set_x(reverse_stop_xy.x());
  reverse_stop_point.set_y(reverse_stop_xy.y());
  injector_->set_reverse_stop_point(reverse_stop_point);
  injector_->set_use_reverse_trajectory(true);
  is_first_enter_top_bull_ = true;
  }

  double adc_x = injector_->vehicle_state()->pose().position().x();
  double adc_y = injector_->vehicle_state()->pose().position().y();
  double distance_to_stop_point =
      std::sqrt((adc_x - injector_->reverse_stop_point().x()) *
                    (adc_x - injector_->reverse_stop_point().x()) +
                (adc_y - injector_->reverse_stop_point().y()) *
                    (adc_y - injector_->reverse_stop_point().y()));

  ADEBUG << "distance_to_stop_point = " << distance_to_stop_point;
  if (distance_to_stop_point < kTopBullArriveDistance &&
      std::fabs(injector_->vehicle_state()->linear_velocity()) <
          kTopBullArriveSpeed) {
    need_reset_borrow_state_ = true;
    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    ADEBUG << "clear borrow info ,no need reverse trajectory,wait for next "
              "frame reset borrow state";
    planning::BorrowResponse borrow_response;
    ptr_trajectory_pb->set_borrow_request(false);
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id(top_bull.blocking_igv_id());
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
    return true;
  }

  common::SLPoint adc_center_sl;
  if (reference_line.XYToSL(common::math::Vec2d(adc_x, adc_y), &adc_center_sl)) {
    if (adc_center_sl.s() < reverse_stop_sl.s()) {
    need_reset_borrow_state_ = true;
    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    ADEBUG << "clear borrow info ,no need reverse trajectory,wait for next "
             "frame reset borrow state";
    planning::BorrowResponse borrow_response;
    ptr_trajectory_pb->set_borrow_request(false);
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id(top_bull.blocking_igv_id());
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
    }
  }
  double adc_heading = injector_->vehicle_state()->heading();
  double end_x = injector_->reverse_stop_point().x();
  double end_y = injector_->reverse_stop_point().y();
  double dx = adc_x - end_x;
  double dy = adc_y - end_y;
  double heading_end_to_adc = std::atan2(dy, dx);  // calculate the heading
  double diff_theta_between_adc_and_end_point =
      common::math::NormalizeAngle(heading_end_to_adc - adc_heading);
  if (std::fabs(diff_theta_between_adc_and_end_point) > M_PI_2) {
    need_reset_borrow_state_ = true;
    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    ADEBUG << "clear borrow info ,no need reverse trajectory,wait for next "
              "frame reset borrow state";
    planning::BorrowResponse borrow_response;
    ptr_trajectory_pb->set_borrow_request(false);
    borrow_response.set_response_type(planning::ResponseType::ACCEPT);
    borrow_response.set_block_obs_id(top_bull.blocking_igv_id());
    borrow_response.set_has_response(true);
    injector_->set_borrow_response(borrow_response);
  }
  return true;
}

bool OnLanePlanning::DeadEndHandle(const PointENU& dead_end_point,
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

void OnLanePlanning::GetScopeParams(const bool is_large_scope,
                                    double* left_scope, double* right_scope,
                                    double* front_scope, double* rear_scope) {
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

bool OnLanePlanning::GetScopeState(const Obstacle& obstacle,
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

bool OnLanePlanning::IsObstacleInLargeScope(
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

bool OnLanePlanning::IsObstacleInScope(const Obstacle& obstacle,
                                       const ReferenceLineInfo* ref_line_info) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(false, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret = GetScopeState(obstacle, ref_line_info, false, left_scope, right_scope,
                      front_scope, rear_scope);
  ADEBUG << "IsObstacleInScope: " << ret;
  return ret;
}

void OnLanePlanning::FindUnreasonableSpeedObstacles(
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

void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
  // when rerouting, reference line might not be updated. In this case, planning
  // module maintains not-ready until be restarted.
  static bool failed_to_update_reference_line = false;
  local_view_ = local_view;
  const auto incoming_routing = local_view.routing;
  if (incoming_routing != nullptr) {
    if (!util::IsDifferentRouting(last_routing_, *incoming_routing)) {
      if (has_pending_routing_) {
        AINFO << "Clear pending routing because active routing is already up to "
              << "date. seq_num=" << incoming_routing->header().sequence_num();
        pending_routing_.Clear();
        has_pending_routing_ = false;
      }
    } else if (!has_pending_routing_ ||
               util::IsDifferentRouting(pending_routing_, *incoming_routing)) {
      pending_routing_ = *incoming_routing;
      has_pending_routing_ = true;
      ADEBUG << "Stage routing as pending. seq_num="
            << pending_routing_.header().sequence_num() << " timestamp_sec="
            << pending_routing_.header().timestamp_sec();
    }
  }
  if (has_pending_routing_) {
    const routing::RoutingResponse* runtime_routing =
        last_routing_.road_size() > 0 ? &last_routing_ : &pending_routing_;
    local_view_.routing =
        std::make_shared<routing::RoutingResponse>(*runtime_routing);
  }
  TryReloadPlanningConfIfChanged();

  request_remote_for_traffic_light_ = false;
  is_near_traffic_light_stop_line_ = false;
  is_planning_error_ = true;
  lane_borrow_failed_ = false;

  static int stopped_count = 0;
  is_sim_control_ = false;
  is_auto_state_ = false;
  if (local_view_.chassis.get()->header().module_name() == "SimControl") {
    is_sim_control_ = true;
  }
  if (local_view_.chassis.get()->driving_mode() ==
      Chassis::COMPLETE_AUTO_DRIVE) {
    is_auto_state_ = true;
  }
  if (!is_auto_state_ && injector_ != nullptr && 0) {
    injector_->enable_self_borrow_ = false;
    injector_->self_borrow_check_times_ = 0;
    // AINFO << "injector_->self_borrow_check_times_  = "
    //       << injector_->self_borrow_check_times_;
    AINFO << "no auto model,clear borrow response.";
    planning::BorrowResponse borrow_response;
    borrow_response.set_response_type(planning::ResponseType::UNTREATED);
    borrow_response.set_block_obs_id("");
    borrow_response.set_has_response(false);
    injector_->set_borrow_response(borrow_response);
    injector_->borrow_response().clear_block_obs_id();
    injector_->set_use_reverse_trajectory(false);
    injector_
        ->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
  } else if (!is_auto_state_ && injector_ != nullptr) {
    injector_->set_use_reverse_trajectory(false);
    injector_
        ->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
  }
  if (injector_ != nullptr) {
    injector_->is_auto_state_ = is_auto_state_;
    injector_->SetReroutinForHuman(false);
  }
  if (std::fabs(local_view_.chassis->speed_mps()) < KStopSpeedThreshold) {
    injector_->adc_stop_count_++;
    injector_->adc_stop_count_ = std::min(200,injector_->adc_stop_count_);
    stopped_count++;
    if (stopped_count >= KStopEnsureCount) {
      is_stopped_ = true;
    }
  } else {
    injector_->adc_stop_count_ = 0;
    is_stopped_ = false;
    stopped_count = 0;
  }

  const double start_timestamp = Clock::NowInSeconds();
  start_timestamp_ = start_timestamp;
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  // localization
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

  bool is_reverse_routing_drive = false;
  if (local_view_.routing != nullptr) {
    // if true,need change localization's heading.
    // 1.directly reverse. 2.T4T5backward request from planning. 3.backward
    // first in J4_east.
    is_reverse_routing_drive =
        local_view_.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_FROM_PLANNING_REROUTING ||
        local_view_.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_DIRECTLY ||
        local_view_.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_NEED_PLANNING_REROUTING ||
        local_view_.routing->routing_request().task_type() ==
            routing::TWO_ROUTING_BACK || 
        local_view_.routing->routing_request().is_backward();
  }
  // AINFO<<"is_reverse_routing_drive = "<<is_reverse_routing_drive;
  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis,
      is_reverse_routing_drive);

  vehicle_gear_ = injector_->vehicle_state()->vehicle_state().gear();
  // if (vehicle_gear_ == Chassis::GEAR_REVERSE) {
  //   AINFO << "Reverse gear.";
  // } else if (vehicle_gear_ == Chassis::GEAR_DRIVE) {
  //   AINFO << "Drive gear.";
  // }
  // borrow response
  // The trajectory of the previous frame is updated only when there is a
  // request.
  if (last_publishable_trajectory_ != nullptr && injector_ != nullptr &&
      local_view_.borrow_response != nullptr) {
    // AINFO << "last_publishable_trajectory_->HasBorrowRequest() = "
    //       << last_publishable_trajectory_->HasBorrowRequest();
    // AINFO << "(*local_view_.borrow_response).has_response() = "
    //       << (*local_view_.borrow_response).has_response();
    // AINFO << "LAST TIME = "
    //       << local_view_.borrow_response->header().timestamp_sec();
    // AINFO << "NOW TIME = " << Clock::NowInSeconds();
    is_new_stacker_response_ = false;
    if (last_publishable_trajectory_->HasBorrowRequest() &&
        (*local_view_.borrow_response).has_response() &&
        Clock::NowInSeconds() -
                local_view_.borrow_response->header().timestamp_sec() <
            kEffectiveTime) {
      AINFO << "LAST HAS REQUEST ,HAS NEW RESPONSE,UPDATE";
      injector_->set_borrow_response((*local_view_.borrow_response));
      if ("STOP_REASON_WHEEL_CRANE" ==
          injector_->borrow_response().block_obs_id()) {
        // need reset borrow for wheelcrane
        is_new_stacker_response_ = true;
      }
    }
  }

  if (last_publishable_trajectory_ != nullptr && injector_ != nullptr &&
      local_view_.pass_stacker_response != nullptr) {
    AINFO << "last_publishable_trajectory_->HasPassStackerRequest() = "
          << last_publishable_trajectory_->HasPassStackerRequest().request_for_pass_stacker();
    AINFO << "(*local_view_.pass_stacker_response).has_response() = "
          << (*local_view_.pass_stacker_response).has_response();
    AINFO << "LAST TIME = "
          << local_view_.pass_stacker_response->header().timestamp_sec();
    AINFO << "NOW TIME = " << Clock::NowInSeconds();

    if ((last_publishable_trajectory_->HasPassStackerRequest()
             .request_for_pass_stacker() ||
         is_in_passing_) &&
        (*local_view_.pass_stacker_response).has_response() &&
        Clock::NowInSeconds() -
                local_view_.pass_stacker_response->header().timestamp_sec() <
            kEffectiveTime) {
      AINFO << "LAST HAS REQUEST ,HAS NEW PASS STACKER RESPONSE,UPDATE";
      injector_->set_pass_stacker_response(
          (*local_view_.pass_stacker_response));
      // if no get borrow response ,cause core dump
      //     if((*local_view_.borrow_response).has_has_response()){
      //   (*local_view_.borrow_response).set_block_obs_id("pass_stacker_obs");
      //         injector_->set_borrow_response((*local_view_.borrow_response));
      //     }
      if (!injector_->is_pass_dynamic_stacker_) {
      AINFO << "ACCEPT BORROW";
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::ACCEPT);
      // if has new borrow id
      borrow_response.set_block_obs_id("pass_stacker_obs");
      if (local_view_.borrow_response->header().timestamp_sec() >
          local_view_.pass_stacker_response->header().timestamp_sec()) {
        if ((*local_view_.borrow_response).has_block_obs_id()) {
          borrow_response.set_block_obs_id(
              (*local_view_.borrow_response).block_obs_id());
        }
      }
      borrow_response.set_has_response(true);
      injector_->set_borrow_response(borrow_response);
      is_new_stacker_response_ = true;
      // injector_->borrow_response().set_block_obs_id("pass_stacker_obs");
      AINFO << "injector_->borrow_response().block_id = "
            << injector_->borrow_response().block_obs_id();
      }
    }
  }
  if (FLAGS_enable_use_pass_stacker) {
    if (injector_->pass_stacker_response().pass_stacker_response_type() ==
            planning::PassStackerResponseType::PASS &&
        !injector_->is_pass_dynamic_stacker_) {
      AINFO<<"ACCEPT BORROW";
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::ACCEPT);
      borrow_response.set_block_obs_id("pass_stacker_obs");
            if (local_view_.borrow_response->header().timestamp_sec() >
          local_view_.pass_stacker_response->header().timestamp_sec()) {
        if ((*local_view_.borrow_response).has_block_obs_id()) {
          borrow_response.set_block_obs_id(
              (*local_view_.borrow_response).block_obs_id());
        }

      }

      borrow_response.set_has_response(true);
      injector_->set_borrow_response(borrow_response);
      AINFO << "injector_->borrow_response().block_id = "
            << injector_->borrow_response().block_obs_id();
      // only one times
      if ("STOP_REASON_WHEEL_CRANE" ==
          injector_->borrow_response().block_obs_id()) {
        if (last_publishable_trajectory_ != nullptr) {
          if (last_publishable_trajectory_->HasPassStackerRequest()
                  .request_for_pass_stacker()) {
            is_new_stacker_response_ = true;
          }
        }
      }
      injector_->is_manual_pass_stacker_ = true;
    } else {
      // static to dynamic large 10s ,need rerequest
      AINFO << "injector_->stacker_static_to_dynamic_times_  = "
            << injector_->stacker_static_to_dynamic_times_;
    }
  }

  if (injector_ != nullptr &&
      local_view_.stackers_info != nullptr) {
      AINFO<<"update stackers info";
      const double delay = Clock::NowInSeconds() -
                           local_view_.stackers_info->header().timestamp_sec();
      if (std::abs(delay) > 10.0) {
        AERROR << "stacker info msg is expired, delay " << delay << "seconds;";
        // need clear.
        // injector_->set_stackers_info((*local_view_.stackers_info));
        injector_->stackers_info().Clear();
      } else {
        AINFO<<"update stacker info";
        planning::StackersInfo stackersinfo;
        auto size = (*local_view_.stackers_info).stacker_info().size();
        AINFO<<"size = "<<size;
        for (size_t i = 0; i < size; i++) {
          auto stacker_info = (*local_view_.stackers_info).stacker_info().at(i);
          double stacker_info_delay =
              Clock::NowInSeconds() - stacker_info.header().timestamp_sec();
          if (stacker_info_delay > 10.0) {
            AINFO << "stacker time is delay : " << stacker_info.stacker_id();
            continue;
          }
          AINFO<<"add stacker info";
          auto* add_stacker =  stackersinfo.add_stacker_info();
          add_stacker->CopyFrom(stacker_info);
        }
         injector_->set_stackers_info(stackersinfo);
      }
  }
  if (injector_ != nullptr &&
      local_view_.v2x_info != nullptr) {
      // AINFO<<"update v2x info";
      injector_->set_v2x_info((*local_view_.v2x_info));
  }

  // stop for tempora
  if (local_view_.temporary_parking_request != nullptr &&
      std::fabs(injector_->vehicle_state()->linear_velocity()) <
          kSpeedForTemporary) {
    if ((*local_view_.temporary_parking_request).need_stop()) {
      const std::string msg = "need to stop for temporary_parking.";
      AERROR << msg;
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      ptr_trajectory_pb->set_gear(vehicle_gear_);
      ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                    Chassis::GEAR_REVERSE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
      return;
    }
  }

  if (nullptr != local_view_.multi_path_temp_stop_request &&
      std::fabs(injector_->vehicle_state()->linear_velocity()) <
          kSpeedForTemporary) {
    if ((*local_view_.multi_path_temp_stop_request).need_stop()) {
      const std::string msg = "need to stop for multi_path_temp_stop.";
      AINFO << msg;
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      ptr_trajectory_pb->set_gear(vehicle_gear_);
      ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                    Chassis::GEAR_REVERSE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
      if ((*local_view_.multi_path_temp_stop_request).need_clear_routing()) {
        auto* destination = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_destination();
        ptr_trajectory_pb->set_has_reached_station(true);
        destination->set_has_reached_station(true);
        local_view_.routing->clear_header();
      }
      return;
    }
  }

  bool need_stop_for_train = false;
  if (local_view_.blocking_area_response != nullptr) {
    if ((*local_view_.blocking_area_response).need_stop()) {
      need_stop_for_train = IsNeedStopForTrain();
    }
  }
  // check adc in gate area
  injector_->is_adc_in_gate_junction_ = IsGateJunctionContainAdc();
  if (!FLAGS_allow_narrow_pass) {
    injector_->is_adc_in_gate_junction_ = false;
  }
  if (FLAGS_enable_expressway_priority) {
    injector_->is_adc_in_expressway_junction_ =
        IsExpresswayJunctionWithoutLaneChange();
  } else {
    injector_->is_adc_in_expressway_junction_ = false;
  }
  AINFO<<"injector_->is_adc_in_gate_junction_ = "<<injector_->is_adc_in_gate_junction_;
  // AINFO << "need_stop_for_train = " << need_stop_for_train;
  if (need_stop_for_train) {
    const std::string msg = "need to stop for train.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
    return;
  }

  const VehicleState& vehicle_state_old =
      injector_->vehicle_state()->vehicle_state();
  double diff_time = injector_->DiffTimeStamp(Clock::NowInSeconds());
  double adc_speed = vehicle_state_old.linear_velocity();
  double adc_acc = vehicle_state_old.linear_acceleration();
  bool is_forward = adc_speed < 0.0 ? false : true;
  // Update adc speed status
  injector_->UpdateAdcSpeedStatus(adc_speed);
  // Update driving distance
  injector_->UpdateDrivingDistance(vehicle_state_old.x(), vehicle_state_old.y(),
                                   is_forward);
  double adc_distance = injector_->GetDrivingDistance();
  injector_->GetVehicleStateKalman()->EstimateWithMeasurement(
      diff_time, adc_distance, adc_speed, adc_acc);
  std::array<double, 3UL> state_estimate;
  injector_->GetVehicleStateKalman()->GetStateEstimate(&state_estimate);
  double update_acc = adc_speed < kStopedSpeed ? 0.0 : state_estimate[2];
  injector_->vehicle_state()->UpdateAcceleration(update_acc);
  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
  if (std::isnan(update_acc)) {
    AERROR << "update acc is nan";
    std::string kalman_debug;
    injector_->GetVehicleStateKalman()->GetDebugString(&kalman_debug);
    AINFO << kalman_debug;
    AINFO << "orignal dis = " << adc_distance;
    AINFO << "orignal speed = " << adc_speed;
    AINFO << "orignal acc = " << adc_acc;
    AINFO << "estimate dis = " << state_estimate[0];
    AINFO << "estimate speed = " << state_estimate[1];
    AINFO << "estimate acc = " << state_estimate[2];
  }
  
  size_t waypoint_num =
      local_view_.routing->routing_request().waypoint().size();
  if (local_view_.routing->routing_request()
          .dead_end_info()
          .dead_end_routing_type() == routing::ROUTING_IN) {
    dead_end_point_ = local_view_.routing->routing_request()
                          .waypoint()
                          .at(waypoint_num - 1)
                          .pose();
  } else if (local_view_.routing->routing_request()
                 .dead_end_info()
                 .dead_end_routing_type() == routing::ROUTING_OUT) {
    dead_end_point_ =
        local_view_.routing->routing_request().waypoint().at(0).pose();
  }

  if (DeadEndHandle(dead_end_point_, vehicle_state) && !wait_flag_) {
    // do not use reference line
    auto* scenario = injector_->planning_context()
                         ->mutable_planning_status()
                         ->mutable_scenario();
    if (ScenarioConfig::DEADEND_TURNAROUND == scenario->scenario_type()) {
      wait_flag_ = true;
      AERROR << "dead end do not use reference line";
      reference_line_provider_->Wait();
    }
  }

  const double vehicle_state_timestamp = vehicle_state.timestamp();

  // RemoteDecider: if vehicle update failed or vehicle state not ok.
  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    // pub if vehicle update failed or vehicle state not ok.
    RemoteDecider(ptr_trajectory_pb);
    injector_->ResetTaskFailureInfo();
    return;
  }
  const VehicleState previous_vehicle_state = vehicle_state_fallback_;
  vehicle_state_fallback_ = vehicle_state;
  // message_latency_threshold 0.02 s
  double latency_time = start_timestamp - vehicle_state_timestamp;
  if (latency_time > 0.0 && latency_time < FLAGS_message_latency_threshold) {
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }
  const routing::RoutingResponse* routing_candidate =
      has_pending_routing_ ? &pending_routing_ : incoming_routing.get();
  const bool is_different_routing =
      routing_candidate != nullptr &&
      util::IsDifferentRouting(last_routing_, *routing_candidate);
  if (HandleOppositeDirectionRouting(status, start_timestamp,
                                     routing_candidate,
                                     is_different_routing,
                                     previous_vehicle_state, vehicle_state,
                                     ptr_trajectory_pb)) {
    return;
  }
  routing::RoutingResponse new_routing_candidate;
  const routing::RoutingResponse* applied_routing = local_view_.routing.get();
  bool should_apply_new_routing = false;
  if (is_different_routing && routing_candidate != nullptr) {
    new_routing_candidate = *routing_candidate;
    if (IsSameOppositeDirectionRouting(new_routing_candidate) &&
        is_opposite_direction_routing_brake_done_) {
      ExpandRoutingSegmentsToVehicleState(vehicle_state,
                                          &new_routing_candidate);
    }
    if (!reference_line_provider_->CanCreateReferenceLineForRouting(
            new_routing_candidate, vehicle_state)) {
      const std::string msg =
          "New routing does not cover current vehicle position.";
      if (has_pending_routing_ && last_routing_.road_size() > 0) {
        injector_->is_new_routing_for_replan_ = false;
        ADEBUG << "Keep routing pending until current scene exits. seq_num="
              << new_routing_candidate.header().sequence_num()
              << " timestamp_sec="
              << new_routing_candidate.header().timestamp_sec();
        applied_routing = &last_routing_;
      } else {
        AERROR << msg
               << " seq_num=" << new_routing_candidate.header().sequence_num()
               << " timestamp_sec="
               << new_routing_candidate.header().timestamp_sec();
        injector_->is_new_routing_for_replan_ = false;
        ptr_trajectory_pb->mutable_decision()
            ->mutable_main_decision()
            ->mutable_not_ready()
            ->set_reason(msg);
        status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
        ptr_trajectory_pb->set_gear(vehicle_gear_);
        ptr_trajectory_pb->set_is_backward_trajectory(Chassis::GEAR_REVERSE ==
                                                      vehicle_gear_);
        FillPlanningPb(start_timestamp, ptr_trajectory_pb);
        GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
        ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_BRAKING);
        RemoteDecider(ptr_trajectory_pb);
        injector_->ResetTaskFailureInfo();
        return;
      }
    } else {
      applied_routing = &new_routing_candidate;
      should_apply_new_routing = true;
    }
  }
  if (should_apply_new_routing) {
    const auto& new_routing = *applied_routing;
    const Vec2d east_in_point{kEastInPointX, kEastInPointY};
    const Vec2d east_out_point{kEastOutPointX, kEastOutPointY};
    const Vec2d west_in_point{kWestInPointX, kWestInPointY};
    const Vec2d west_out_point{kWestOutPointX, kWestOutPointY};
    std::vector<hdmap::LaneInfoConstPtr> route_lanes;
    west_in_ = false;
    west_out_ = false;
    east_in_ = false;
    east_out_ = false;
    for (int road_index = 0; road_index < new_routing.road_size();
         ++road_index) {
      const auto& road_segment = new_routing.road(road_index);
      for (int passage_index = 0; passage_index < road_segment.passage_size();
           ++passage_index) {
        const auto& passage = road_segment.passage(passage_index);
        for (int lane_index = 0; lane_index < passage.segment_size();
             ++lane_index) {
          if (passage.segment(lane_index).id().empty()) {
            AERROR << "Current lane_index" << lane_index
                   << " | Failed to get lane id!";
            break;
          }
          auto lane = hdmap_->GetLaneById(
              hdmap::MakeMapId(passage.segment(lane_index).id()));
          if (nullptr == lane) {
            AERROR << "Current lane_index" << lane_index
                   << " | Failed to get lane!";
            break;
          }
          all_lane_ids_.insert(passage.segment(lane_index).id());
          AINFO << "passage.segment(lane_index).id() = "
                << passage.segment(lane_index).id();
          route_lanes.emplace_back(lane);
        }
      }
    }

    auto FindFirstHitLaneIndex = [&](const Vec2d& point) -> int {
      for (size_t idx = 0U; idx < route_lanes.size(); ++idx) {
        if (IsLanePassPoint(route_lanes[idx], point,
                            kRoutingPassPointTolerance)) {
          return static_cast<int>(idx);
        }
      }
      return -1;
    };
    const size_t lane_count = route_lanes.size();
    const int east_in_idx = FindFirstHitLaneIndex(east_in_point);
    const int east_out_idx = FindFirstHitLaneIndex(east_out_point);
    const int west_in_idx = FindFirstHitLaneIndex(west_in_point);
    const int west_out_idx = FindFirstHitLaneIndex(west_out_point);

    const bool east_pair_hit = east_in_idx >= 0 && east_out_idx >= 0;
    const bool west_pair_hit = west_in_idx >= 0 && west_out_idx >= 0;

    east_in_ = east_pair_hit && east_in_idx < east_out_idx;
    east_out_ = east_pair_hit && east_out_idx < east_in_idx;
    west_in_ = west_pair_hit && west_in_idx < west_out_idx;
    west_out_ = west_pair_hit && west_out_idx < west_in_idx;

    ADEBUG << "routing point classification: west_in=" << west_in_
          << " west_out=" << west_out_ << " east_in=" << east_in_
          << " east_out=" << east_out_
          << " east_pair_hit=" << east_pair_hit
          << " west_pair_hit=" << west_pair_hit
          << " east_idx=(" << east_in_idx << "," << east_out_idx << ")"
          << " west_idx=(" << west_in_idx << "," << west_out_idx << ")"
          << " lane_count=" << lane_count;
  }

  static uint8_t new_routing_count = 0;
  if (should_apply_new_routing) {
    injector_->is_new_routing_for_replan_ = true;
    last_routing_ = *applied_routing;
    ADEBUG << "last_routing_:" << last_routing_.ShortDebugString();
    if (has_pending_routing_ &&
        !util::IsDifferentRouting(pending_routing_, *applied_routing)) {
      pending_routing_.Clear();
      has_pending_routing_ = false;
    }
    local_view_.routing =
        std::make_shared<routing::RoutingResponse>(*applied_routing);
    const auto& top_bull = injector_->planning_context()->mutable_planning_status()->top_bull();
    bool is_top_bull = top_bull.is_in_top_bull();
    auto action_type = top_bull.action_type();
    auto block_obs_id = top_bull.block_obs_id();
    auto random_number = top_bull.random_number();
    auto reverse_distance = top_bull.reverse_distance();
    auto blocking_igv_id = top_bull.blocking_igv_id();
    auto ego_start_action_time = top_bull.ego_start_action_time();
    auto ego_complete_action = top_bull.ego_complete_action();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_rerouting()
        ->set_is_new_routing(true);
     auto mutable_top_bull = injector_->planning_context()->mutable_planning_status()->mutable_top_bull();
      mutable_top_bull->set_is_in_top_bull(is_top_bull);
      mutable_top_bull->set_action_type(action_type);
      mutable_top_bull->set_block_obs_id(block_obs_id);
      mutable_top_bull->set_random_number(random_number);
      mutable_top_bull->set_reverse_distance(reverse_distance);
      mutable_top_bull->set_blocking_igv_id(blocking_igv_id);
      mutable_top_bull->set_ego_start_action_time(ego_start_action_time);
      mutable_top_bull->set_ego_complete_action(ego_complete_action);
    AINFO << "[new routing] is new routing!";
    reference_line_provider_->UpdateRoutingResponse(*applied_routing);
    planner_->Init(config_);
    injector_->pullover_finished = false;
    has_pullover_finished_flag_ = false;
    injector_->pullover_end_trace_ = false;

    //  Restart after a new route
    wait_flag_ = false;
    if (reference_line_provider_->IsStop()) {
      reference_line_provider_->Start();
      AERROR << "new route use reference line";
    }
    AINFO << "new routing clear stacker response.";
    planning::PassStackerResponse pass_stacker_response;
    pass_stacker_response.set_pass_stacker_response_type(
        planning::PassStackerResponseType::ORIGINAL);
    pass_stacker_response.set_has_response(false);
    injector_->set_pass_stacker_response(pass_stacker_response);
    if(!mutable_top_bull->is_in_top_bull()){
     AINFO << "new routing clear borrow response.";
    planning::BorrowResponse borrow_response;
    borrow_response.set_response_type(planning::ResponseType::UNTREATED);
    borrow_response.set_block_obs_id("");
    borrow_response.set_has_response(false);
    injector_->set_borrow_response(borrow_response);
    }
    injector_->set_is_new_routing(true);
    new_routing_count = 0;
    if (last_routing_.road_size() > 0) {
      // Generate preferred lane information
      GeneratePreferredLane();
      // Generate route lane information
      GenerateRouteLaneInfo();
      // Check is off lane departure
      CheckOffLaneDeparture();
    }
  } else {
    injector_->is_new_routing_for_replan_ = false;
    if (++new_routing_count >= kRoutingTurnMaxCount) {
      injector_->set_is_new_routing(false);
      new_routing_count = kRoutingTurnMaxCount;
    }
  }
  // AINFO<<"injector_->is_new_routing_for_replan_ = "<<injector_->is_new_routing_for_replan_;
  // check if routing is ready
  if (waypoint_num > 0) {
    route_end_ =
        applied_routing->routing_request().waypoint().at(waypoint_num - 1);
    routing_is_ready_ = true;
  }

  failed_to_update_reference_line =
      (!reference_line_provider_->UpdatedReferenceLine());
  // early return when reference line fails to update after rerouting
  // RemoteDecider: if failed to update reference line after rerouting.
  if (failed_to_update_reference_line) {
    const std::string msg = "Failed to update reference line after rerouting.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    JudgeArrivedStationImmediately(ptr_trajectory_pb);
    // pub if failed to update reference line after rerouting.
    RemoteDecider(ptr_trajectory_pb);
    injector_->ResetTaskFailureInfo();
    if ((nullptr != frame_ &&
         !frame_->open_space_info().is_on_open_space_trajectory())) {
      AERROR << "failed to update reference line, return.";
      return;
    }
  }

  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  // planning_loop_rate 10
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);
  // AINFO<<"injector_ ->is_new_routing = "<<injector_->is_new_routing();
  
  std::string replan_reason;
  std::vector<TrajectoryPoint> stitching_trajectory;
  // trajectory_stitching_preserved_length 20
  ADEBUG << "IsPoorStatusOfTaskFailure: "
         << injector_->IsPoorStatusOfTaskFailure();
  ADEBUG << "is need to reinit start point: "
         << injector_->NeedReinitStartPoint();
  const bool is_replan = !TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      FLAGS_trajectory_stitching_preserved_length, true,
      last_publishable_trajectory_.get(), &stitching_trajectory, &replan_reason,
      injector_);
  // continus replan may cause break slow.
  if (is_replan) {
    replan_count_++;
    if (replan_count_ >= 3) {
      AERROR << "need stop for replan frequent";
    }
  } else {
    replan_count_ = 0;
  }
  ADEBUG << "stitching_trajectory.back().x = "
         << stitching_trajectory.back().path_point().x();
  ADEBUG << "stitching_trajectory.back().y = "
         << stitching_trajectory.back().path_point().y();
  ADEBUG << "VEHICLE = " << vehicle_state.x() << " " << vehicle_state.y();
  injector_->SetReplanState(is_replan);
  injector_->ResetReinitStartPoint();

  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
  // AINFO<<"is_replan = "<<is_replan;
  // AINFO << "stitching_trajectory.back().HEADING = "
  //       << stitching_trajectory.back().path_point().theta();
  // AINFO << "vehicle heading = " << vehicle_state.heading();
  // AINFO << "VEHICLE KAPPA = " << vehicle_state.kappa();
  // AINFO << "stitching_trajectory.back().path_point().KAPPA = "
  //       << stitching_trajectory.back().path_point().kappa();
  // Reversing without re planning the curvature will result in crossing the boundary.
  if(injector_->need_back_ward_ || is_need_backward_for_turn_blocking_
  || injector_->use_reverse_trajectory()){
    stitching_trajectory.back().mutable_path_point()->set_kappa(0.0);
  }

  // AINFO<<"vehicle state speed = "<<vehicle_state.linear_velocity();
  // AINFO<<"start point speed = "<<stitching_trajectory.back().v();
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  ADEBUG << "frame_num: " << frame_num;
  HysteresisInterval::SetSequenceNum(frame_num);
  ObstacleHistoryValue::SetSequenceNum(frame_num);
  ObstacleSpeedDistanceHistory::SetSequenceNum(frame_num);
  ObstacleHistoryDiffValue::SetSequenceNum(frame_num);
  ObstacleStabilizationForTEBSpeed::SetSequenceNum(frame_num);
  StartUpVehiclePositionHistory::SetSequenceNum(frame_num);
  ObstacleHeadingHistory::SetSequenceNum(frame_num);
  injector_->SetSequenceNum(frame_num);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
    frame_->set_is_sim_control(is_sim_control_);
    frame_->set_planning_start_time(start_timestamp);
  }

  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  // RemoteDecider: if failed to InitFrame.
  if (!status.ok()) {
    AERROR << status.ToString();
    // publish_estop, false, publish estop decision in planning
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
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    JudgeArrivedStationImmediately(ptr_trajectory_pb);
    RemoteDecider(ptr_trajectory_pb);
    injector_->ResetTaskFailureInfo();
    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      const uint32_t n = frame_->SequenceNum();
      injector_->frame_history()->Add(n, std::move(frame_));
    }
    return;
  }

  // sapecial area hand to auto, rerouting,
  bool is_auto_state_change = false;
  static bool last_is_auto_state_ = true;
  if (last_is_auto_state_ != is_auto_state_) {
    last_is_auto_state_ = is_auto_state_;
    is_auto_state_change = true;
  }
  if (local_view_.routing != nullptr && is_auto_state_change) {
    if (local_view_.routing->routing_request().rerouting_info().is_rerouting() &&
        local_view_.routing->routing_request().multi_routing_type() !=
            routing::PROCESSINGSTART &&
        is_auto_state_) {
      if (injector_ != nullptr) {
        if (injector_->planning_context() != nullptr) {
          AINFO << "auto state change to two routing,send rerouting request";
          if (frame_ != nullptr &&
              !frame_->Rerouting(injector_->planning_context(),
                                 ReroutingType::SECOND_REROUTING)) {
            AERROR << "Failed to send rerouting request";
          }
        }
      }
    }
  }

  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    bool is_tiny_adjust_type =false;
      if (local_view_.routing != nullptr) {
       is_tiny_adjust_type = (routing::TINY_ADJUSTMENT_FRONT ==
         local_view_.routing->routing_request().task_type()) ||
        (routing::TINY_ADJUSTMENT_BACK ==
         local_view_.routing->routing_request().task_type()) ||
        (routing::TINY_ADJUSTMENT_RIGHT ==
         local_view_.routing->routing_request().task_type()) ||
        (routing::TINY_ADJUSTMENT_LEFT ==
         local_view_.routing->routing_request().task_type());}
    ref_line_info.SetIsTinyAdjustType(is_tiny_adjust_type);
  }

  std::vector<std::pair<std::string, double>> corrected_obstacles;
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    ref_line_info.SetInitPointHeading(
        stitching_trajectory.back().path_point().theta());
    if (FLAGS_enable_correct_obstacle_speed &&
        !ref_line_info.IsChangeLanePath()) {
      // find the unreasonable speed obstacle
      FindUnreasonableSpeedObstacles(&ref_line_info, &corrected_obstacles);
    }
  }

  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    ADEBUG << "Is change lane reference line: "
           << ref_line_info.IsChangeLanePath();
    // modify the unreasonable obstacle speed
    for (auto& corrected_obstacle : corrected_obstacles) {
      auto* mutable_obstacle =
          ref_line_info.path_decision()->Find(corrected_obstacle.first);
      if (nullptr != mutable_obstacle) {
        mutable_obstacle->SetIsModifiedVelocity(true);
        mutable_obstacle->SetModifiedVelocity(corrected_obstacle.second);
        AINFO << "Modify obstacle[" << corrected_obstacle.first
              << "] speed from " << mutable_obstacle->speed() << " to "
              << corrected_obstacle.second;
      }
    }
    AddModifiedObstacle(&ref_line_info, corrected_obstacles);
    if (FLAGS_enable_add_obs) {
      AddFlowerBedObstacle(&ref_line_info);
    }
    if (FLAGS_enable_use_pass_stacker) {
      CreateStackerObstacleWithID(&ref_line_info);
      // need to open
      CreateWheelCraneObstacle(&ref_line_info);
      CreateWheelCraneObstacleWithPerception(&ref_line_info);
      AddStackerObstacle(&ref_line_info);
    } else {
      CreateStackerObstacle(&ref_line_info);
      AddStackerObstacle(&ref_line_info);
    }
  }
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    ref_line_info.SetIsNewStackerSesponse(is_new_stacker_response_);
  }
  if (local_view_.barrier != nullptr) {
    for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
      ref_line_info.SetBarrier(*local_view_.barrier);
    }
  }
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    ref_line_info.SetIsWestIn(west_in_);
    ref_line_info.SetIsWestOut(west_out_);
    ref_line_info.SetIsEastIn(east_in_);
    ref_line_info.SetIsEastOut(east_out_);
  }

  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    static StartUpVehiclePositionHistory vehicle_history(20UL);
    static ObstacleHeadingHistory obs_heading_history(50UL);
    if (!ref_line_info.IsChangeLanePath()) {
      vehicle_history.SetReferenceLine(&ref_line_info);
      vehicle_history.Update();
      obs_heading_history.SetReferenceLine(&ref_line_info);
      obs_heading_history.Update();
      const auto& ObstacleItems =
          ref_line_info.path_decision()->obstacles().Items();
      for (auto* obstacle : ObstacleItems) {
        bool is_start_up = vehicle_history.GetVehicleIsStartUp(*obstacle);
        bool is_safe_to_sidepass =
            obs_heading_history.GetIsSafeSidepass(*obstacle);
        auto* mutable_obstacle =
            ref_line_info.path_decision()->Find(obstacle->Id());
        if (nullptr != mutable_obstacle) {
          mutable_obstacle->SetStartUpState(is_start_up);
          mutable_obstacle->SetSidepassState(is_safe_to_sidepass);
        }
        ADEBUG << "obstacle id: " << obstacle->Id();
        ADEBUG << "IsStartUp: " << is_start_up;
        ADEBUG << "is_safe_to_sidepass: " << is_safe_to_sidepass;
      }
    }
  }

  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    /**
     * After the ADC stops, when the speed of the obstacle ahead is less than a
     * threshold value and the distance between the ADC and the obstacle is less
     * than another threshold value, the ADC does not start.
     */
    // TODO(he zhiguo): This causes integration tests to fail.
    // if (CheckAndPubStopTrajectory(start_timestamp, &ref_line_info,
    //                               ptr_trajectory_pb)) {
    //   ADEBUG << "ADC is stopped by the front obstacle.";
    //   return;
    // }
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status =
        traffic_decider.Execute(frame_.get(), &ref_line_info, injector_);

    if (!ref_line_info.IsChangeLanePath()) {
      IsAdcInCommonJunction(&ref_line_info);
    }

    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }

    // check is car near traffic light stop line.
    if (ref_line_info.IsNearTrafficLightStopLine()) {
      is_near_traffic_light_stop_line_ = true;
      ADEBUG << "Reference line " << ref_line_info.Lanes().Id()
             << ", is_near_traffic_light_stop_line is true";
    }
  }

  const double delay =
      Clock::NowInSeconds() -
      local_view_.prediction_obstacles->header().timestamp_sec();
  if (std::abs(delay) > FLAGS_prediction_expire_time_sec) {
    AERROR << "Prediction obstacles msg is expired, delay " << delay
           << "seconds;";
    const std::string msg = "Prediction obstacles msg is expired.";
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
    return;
  }
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    ref_line_info.SetIsNeedResetBorrowState(need_reset_borrow_state_);
  }

  const bool desired_is_top_null = FLAGS_top_bull_default_is_top_bull;
  const int desired_action_type = FLAGS_top_bull_default_action_type;
  const double desired_reverse_distance = FLAGS_top_bull_default_reverse_distance;
  if (!top_null_runtime_initialized_ ||
      desired_is_top_null != last_top_null_is_top_null_ ||
      desired_action_type != last_top_null_action_type_ ||
      std::abs(desired_reverse_distance - last_top_null_reverse_distance_) >
          1e-6) {
    if (UpdateTopNullFlagsRuntime(desired_is_top_null, desired_action_type,
                                  desired_reverse_distance)) {
      top_null_runtime_initialized_ = true;
      last_top_null_is_top_null_ = desired_is_top_null;
      last_top_null_action_type_ = desired_action_type;
      last_top_null_reverse_distance_ = desired_reverse_distance;
    }
  }

  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);
  need_reset_borrow_state_ = false;
  ptr_trajectory_pb->set_need_diagonal(false);

  // check is passing gate but is close
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    if (ref_line_info.IsPassingGate()) {
      bool is_need_stop = false;
      if ((ref_line_info.IsWestIn() || ref_line_info.IsWestOut()) &&
          (!ref_line_info.Barrier().west_north() ||
           !ref_line_info.Barrier().west_south())) {
        is_need_stop = true;
      }
      if ((ref_line_info.IsEastIn() || ref_line_info.IsEastOut()) &&
          (!ref_line_info.Barrier().east_north() ||
           !ref_line_info.Barrier().east_south())) {
        is_need_stop = true;
      }
      if (is_need_stop) {
        const std::string msg = "stop for gate close.";
        ptr_trajectory_pb->mutable_decision()
            ->mutable_main_decision()
            ->mutable_not_ready()
            ->set_reason(msg);
        status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
        ptr_trajectory_pb->set_gear(vehicle_gear_);
        ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                      Chassis::GEAR_REVERSE);
        FillPlanningPb(start_timestamp, ptr_trajectory_pb);
        GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
        return;
      }
    }
  }

  double diagonal_heading = vehicle_state.heading();

  // if (frame_->reference_line_info().size() == 1) {
  auto reference_line_info = frame_->reference_line_info().begin();
  while (reference_line_info != frame_->reference_line_info().end()) {
    if (reference_line_info->NeedDiagonal()) {
      // AINFO<<"need diagonal";
      ptr_trajectory_pb->set_need_diagonal(true);
      // AINFO<<"need diagonal";
      auto adc_sl_boundary = reference_line_info->AdcSlBoundary();
      double center_s =
          (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
      // AINFO << "center_s = " << center_s;
      const auto& reference_point =
          reference_line_info->reference_line().GetReferencePoint(center_s);
      diagonal_heading = reference_point.heading();

      ptr_trajectory_pb->set_diagonal_heading(diagonal_heading);
      if(reference_line_info->IsInDiagonalRoad()){
        //need to change heading use diagonal road front and back lane heading.
        ptr_trajectory_pb->set_diagonal_heading(reference_line_info->DiagonalRoadHeading());
      }
      if(FLAGS_allow_smi_diagonal){
        for (int i = 0; i < ptr_trajectory_pb->trajectory_point_size(); ++i) {
          ptr_trajectory_pb->mutable_trajectory_point(i)
              ->mutable_path_point()
              ->set_theta(diagonal_heading);
        }
      }

      break;
    } else {
      // AINFO<<"no need diagonal";
    }
    ++reference_line_info;
  }

  StartUpVehiclePositionHistory::ResetUpdateState();
  ObstacleHeadingHistory::ResetUpdateState();

  if (!frame_->open_space_info().is_on_open_space_trajectory() &&
      !injector_->pullover_finished) {
    injector_->is_adc_out_lane_ = false;
    injector_->is_adc_deviate_lane_direction_ = false;

    ADEBUG << "frame_->mutable_reference_line_info() = "
           << frame_->mutable_reference_line_info()->size();

    // check adc in common junction
    Status outlane_status(ErrorCode::OK);
    for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
      if (!ref_line_info.IsChangeLanePath()) {
        outlane_status = IsAdcInRoad(&ref_line_info);
        if (outlane_status.ok()) {
          outlane_status =
              IsAdcDeviateLaneDirection(&ref_line_info, ptr_trajectory_pb);
          injector_->is_adc_out_lane_ = !outlane_status.ok();

        } else {
          // injector_->is_adc_out_lane_ = true;
        }
        StartUpVehiclePositionHistory::SetFrenetPath(
            ref_line_info.path_data().frenet_frame_path());
        ObstacleHeadingHistory::SetFrenetPath(
            ref_line_info.path_data().frenet_frame_path());
      }
    }

    if (injector_->is_adc_out_lane_) {
      status = outlane_status;
      ptr_trajectory_pb->set_lane_status(ADCTrajectory::OUTSIDE_LANE);
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason("adc out road");
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      ptr_trajectory_pb->set_gear(vehicle_gear_);
    // ref_line no update ,so adc_heading with ref has large diff_heading.
    // estop use no gear,so now is D,no change gear.
      ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                    Chassis::GEAR_REVERSE);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateStopTrajectory(ptr_trajectory_pb);
      frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
      JudgeArrivedStationImmediately(ptr_trajectory_pb);
      // pub if ADC is away from the road.
      RemoteDecider(ptr_trajectory_pb);
      {
        std::lock_guard<std::mutex> frame_lock(frame_mutex_);
        const uint32_t n = frame_->SequenceNum();
        injector_->frame_history()->Add(n, std::move(frame_));
      }

      return;
    }
  }

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
  ptr_trajectory_pb->set_overtake_state(frame_->GetOvertakeReportState());

  // request remote control if traffic light is not ok.
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    if (ref_line_info.TrafficLightRequest()) {
      request_remote_for_traffic_light_ = true;
      ADEBUG << "Reference line " << ref_line_info.Lanes().Id()
             << ", request_remote_for_traffic_light is true";
      break;
    }
    if (ref_line_info.LaneBorrowFailRequest()) {
      lane_borrow_failed_ = true;
      ADEBUG << "Lane borrow failed, need remote request.";
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
      Frame::MixedTrafficType::UNKNOWN != frame_->mixed_traffic_type_) {
    is_mixed_traffic_ = true;
    mutable_mixed_traffic->set_history_mixed_traffic(is_mixed_traffic_);
    ptr_trajectory_pb->set_is_mixed_traffic_scenario(is_mixed_traffic_);
  } else {
    // has many Non-motorized vehicle in the current lane or  has retrograde
    // obstacle in the non motor vehicle lane.
    if (frame_->mixed_traffic_type_ ==
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
  // AINFO << "ADC is in the mixed traffic scenario:[" << is_mixed_traffic_
  //       << "].";

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
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();

  status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
  if (!status.ok()) {
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
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

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    ptr_trajectory_pb->set_trajectory_scenario(ADCTrajectory::OPENSPACE);
    ptr_trajectory_pb->set_is_backward_trajectory(
        OpenspaceCommon::is_reverse_routing());
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

    // if rerouting from planning to backward or routing need directly backward.
    if (local_view_.routing != nullptr) {
      auto request = local_view_.routing->routing_request();
      if (request.task_type() ==
              routing::BACKWARD_ROUTING_FROM_PLANNING_REROUTING ||
          request.task_type() == routing::BACKWARD_ROUTING_DIRECTLY ||
          request.task_type() ==
              routing::BACKWARD_ROUTING_NEED_PLANNING_REROUTING ||
          request.task_type() == routing::TWO_ROUTING_BACK ||
          request.is_backward()) {
        ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_REVERSE);
        ptr_trajectory_pb->set_is_backward_trajectory(true);
      }
    }

    ptr_trajectory_pb->set_trajectory_scenario(ADCTrajectory::LANEFOLLOW);

    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    CheckStopForWheelcrane(ptr_trajectory_pb);
    CheckEnableBorrow(ptr_trajectory_pb);
    CheckOutRouting(ptr_trajectory_pb);
    if (FLAGS_enable_rerouting_for_block) {
      CheckIsNeedToReroutingForBlock();
    }
    SetIsAutoDrive(ptr_trajectory_pb,is_auto_state_);
    if (last_publishable_trajectory_ != nullptr) {
      last_publishable_trajectory_->SetBorrowRequest(
          ptr_trajectory_pb->borrow_request());
      // AINFO << "ptr_trajectory_pb->borrow_request() = "
      //       << ptr_trajectory_pb->borrow_request();
      last_publishable_trajectory_->SetPassStackerRequest(
          ptr_trajectory_pb->pass_stacker_request());
    }
    CheckNeedToShrinkCollisionBuffer(ptr_trajectory_pb);
    if (FLAGS_enable_reverse_trajectory) {
      CheckUseReverseTrajectory(stitching_trajectory.back(),
                                stitching_trajectory, ptr_trajectory_pb);
      if (local_view_.routing->routing_request().is_reverse_trajectory() &&
          !injector_->use_reverse_trajectory()) {
        GenerateStopTrajectory(ptr_trajectory_pb);
      }
    // only for reverse heading in simulation
    // auto reference_line_info = frame_->reference_line_info().begin();
    // while (reference_line_info != frame_->reference_line_info().end()) {
    // if (reference_line_info->NeedDiagonal()) {
    //   // AINFO<<"need diagonal";
    //   ptr_trajectory_pb->set_need_diagonal(true);
    //   // AINFO<<"need diagonal";
    //   auto adc_sl_boundary = reference_line_info->AdcSlBoundary();
    //   double center_s =
    //       (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
    //   // AINFO << "center_s = " << center_s;
    //   const auto& reference_point =
    //       reference_line_info->reference_line().GetReferencePoint(center_s);
    //   diagonal_heading = reference_point.heading();

    //   ptr_trajectory_pb->set_diagonal_heading(diagonal_heading);
    //   if(reference_line_info->IsInDiagonalRoad()){
    //     //need to change heading use diagonal road front and back lane heading.
    //     ptr_trajectory_pb->set_diagonal_heading(reference_line_info->DiagonalRoadHeading());
    //   }
    //   for (int i = 0; i < ptr_trajectory_pb->trajectory_point_size(); ++i) {
    //     // ptr_trajectory_pb->mutable_trajectory_point(i)
    //     //     ->mutable_path_point()
    //     //     ->set_theta(diagonal_heading);
    //   }
    //   break;
    // } else {
    //   // AINFO<<"no need diagonal";
    // }
    // ++reference_line_info;
  // }
    }
    
    CalcuHonkingStats(ptr_trajectory_pb);
    CalcuDisplayType(ptr_trajectory_pb);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);

 
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    // True to enable planning smoother among different planning cycles.
    // enable_planning_smoother, false
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);
    }
  }


  // reference line recovery only one frame
  bool complete_dead_end =
      frame_.get()->open_space_info().destination_reached();
  ADEBUG << "complete_dead_end is: " << complete_dead_end;

  if (complete_dead_end) {
    if (reference_line_provider_->IsStop()) {
      reference_line_provider_->Start();
    }
    AERROR << "complete_dead_end reuse reference line";
    wait_flag_ = false;
  }

  if (JudgeReachTargetPoint(ptr_trajectory_pb)) {
  } else {
    CheckTopBullRerouting();
  }

  if (local_view_.routing->routing_request().task_type() ==
      routing::TINY_ADJUSTMENT_STOP) {
    auto* destination = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_destination();
    ptr_trajectory_pb->set_has_reached_destination(true);
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_destination(true);
    destination->set_has_reached_station(true);
    AERROR << "arrived_station_immediately: GenerateStopTrajectory.";
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
    // GenerateStopTrajectory(ptr_trajectory_pb);
    ClearRoutingHeader();
  }
  // RemoteDecider: planning_error,traffic_light,near_traffic_light_stop_line.
  RemoteDecider(ptr_trajectory_pb);

  if (canbus::Chassis::GEAR_REVERSE == ptr_trajectory_pb->gear() &&
      ADCTrajectory::AD_PARKING != ptr_trajectory_pb->ad_behavior()) {
    ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_REVERSING);
  }

  {
    std::lock_guard<std::mutex> frame_lock(frame_mutex_);
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
  }
}

double OnLanePlanning::GetNewReverseDistance(
    planning::ReferenceLineInfo* best_ref_info) {
  double path_length =
      std::fabs(best_ref_info->path_data().discretized_path().back().s() -
                best_ref_info->path_data().discretized_path().front().s());
  auto* path_decision = best_ref_info->path_decision();
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double ego_length = vehicle_param.length() + kCollisionLonBuffer;
  double ego_width = vehicle_param.width() + kCollisionLateralBuffer;
  double shift_distance =
      ego_length * 0.5 - vehicle_param.back_edge_to_center();
  // AINFO << "ego_length:" << ego_length << "shift_distance:" << shift_distance;
  const auto& discretized_path = best_ref_info->path_data().discretized_path();
  size_t path_step = 1;
  if (discretized_path.empty()) {
    AINFO << "path point empty ,need stop.";
    return 0.0;
  }
  AINFO << "obstacle size = " << path_decision->obstacles().Items().size();
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (nullptr == ptr_obstacle) {
      // ADEBUG << "Current obstacle pointer is null.";
      continue;
    }
    AINFO << "ptr_obstacle_item = " << ptr_obstacle_item->Id();
    const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
    const auto& obs_type = ptr_obstacle_item->Perception().type();
    const auto& adc_sl = best_ref_info->AdcSlBoundary();
    // obs front adc 3m, no consider.
    if (obs_sl.start_s() > adc_sl.end_s() + kDistanceFrontAdc) {
      continue;
    }
      const auto& center_s =
      (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  const auto& reference_point =
      best_ref_info->reference_line().GetReferencePoint(center_s);
  double diagonal_heading = reference_point.heading();
  if(best_ref_info->IsInDiagonalRoad()){
    diagonal_heading = best_ref_info->DiagonalRoadHeading();
    // AINFO<<"diagonal_heading_==="<<diagonal_heading_;
  }
    for (size_t i = 0; i < discretized_path.size(); i += path_step) {
      const auto& point = discretized_path[i];
      auto path_point_theta = point.theta();
      // need control check is always diagonal.
      if(best_ref_info->NeedDiagonal()){
        path_point_theta = diagonal_heading;
      }
      Box2d ego_box({point.x(), point.y()}, path_point_theta, ego_length+kLonBufferForCollision,
                    ego_width+kLateralBufferForCollision);

      Vec2d shift_vec{shift_distance * std::cos(path_point_theta),
                      shift_distance * std::sin(path_point_theta)};
      ego_box.Shift(shift_vec);
      const auto& adc_polygon = Polygon2d(ego_box);
      if (ptr_obstacle->IsVirtual()) {
        continue;
      }
      // AINFO << "path_point.s() = " << point.s();
      if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type) {
        const auto& obstacle_polygon = ptr_obstacle_item->PerceptionPolygon();
        if (obstacle_polygon.HasOverlap(adc_polygon)) {
          AINFO << ": ["
                << "]"
                << "obstacle id " << ptr_obstacle_item->Id() << "     obs_type "
                << obs_type;
          path_length = std::min(path_length, std::fabs(point.s()));
          if (i > 0) {
            path_length =
                std::min(path_length, std::fabs(discretized_path[i - 1].s()));
          }
          return path_length;
        }
      } else {
        const auto& box = ptr_obstacle_item->PerceptionBoundingBox();
        if (ego_box.HasOverlap(box)) {
          AINFO << ": ["
                << "]"
                << "obstacle id " << ptr_obstacle_item->Id()
                << "      obs_type " << obs_type;
          path_length = std::min(path_length, std::fabs(point.s()));
          if (i > 0) {
            path_length =
                std::min(path_length, std::fabs(discretized_path[i - 1].s()));
          }
          return path_length;
        }

        // TODO(zongxingguo): consider dynamic obs.
      }
    }
  }
  AINFO << "path_length = " << path_length;
  return path_length;
}

void OnLanePlanning::SetIsAutoDrive(ADCTrajectory* const ptr_trajectory_pb,
                                    bool is_auto_drive) {
  if (ptr_trajectory_pb != nullptr) {
  if (!is_auto_drive) {
    ptr_trajectory_pb->set_enable_auto_drive(false);
      double trajectory_length = 0.0;
      if (!ptr_trajectory_pb->trajectory_point().empty()) {
        trajectory_length =
        ptr_trajectory_pb->trajectory_point()
            .at(ptr_trajectory_pb->trajectory_point().size() - 1)
            .path_point()
            .s() -
        ptr_trajectory_pb->trajectory_point().at(0).path_point().s();
      }
    // AINFO << "trajectory_length = " << trajectory_length;
    if (trajectory_length > kMinTrajectoryLength) {
      enable_auto_drive_count_++;
        if (enable_auto_drive_count_ > 20) {
      ptr_trajectory_pb->set_enable_auto_drive(true);
      }
      } else {
        enable_auto_drive_count_ = 0;
      }
    }
  }
}

void OnLanePlanning::CheckOutRouting(ADCTrajectory* const ptr_trajectory_pb) {
  planning::ReferenceLineInfo* best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == best_ref_info) {
    return;
  }
  const auto& adc_sl = best_ref_info->AdcSlBoundary();
  double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  common::SLPoint end_sl_point;
  const auto& route_end_waypoint_ref = best_ref_info->get_routing_end();
  AINFO << "route_end_waypoint_ref.pose() = "
        << route_end_waypoint_ref.pose().x() << "    "
        << route_end_waypoint_ref.pose().y();
  if (!best_ref_info->reference_line().XYToSL(route_end_waypoint_ref.pose(),
                                              &end_sl_point)) {
    return;
  }
  AINFO << "center_s = " << center_s;
  AINFO << "end_sl_point = " << end_sl_point.s();
  bool is_near_end_point = false;
  double adc_x = best_ref_info->vehicle_state().x();
  double adc_y = best_ref_info->vehicle_state().y();
  if (local_view_.routing != nullptr) {
    size_t waypoint_num =
        local_view_.routing->routing_request().waypoint().size();
    if (waypoint_num > 0) {
      auto route_first =
          local_view_.routing->routing_request().waypoint().at(0);
      double adc_to_first_point = std::sqrt(
          (adc_x - route_first.pose().x()) * (adc_x - route_first.pose().x()) +
          (adc_y - route_first.pose().y()) * (adc_y - route_first.pose().y()));
      double adc_to_end_point =
          std::sqrt((adc_x - route_end_waypoint_ref.pose().x()) *
                        (adc_x - route_end_waypoint_ref.pose().x()) +
                    (adc_y - route_end_waypoint_ref.pose().y()) *
                        (adc_y - route_end_waypoint_ref.pose().y()));
      is_near_end_point = adc_to_end_point < adc_to_first_point;
    }
  }
  if (!is_near_end_point) {
    AINFO << "no near end point, no consider out routing";
    return;
  }
AINFO<<"end_sl_point.l() = "<<end_sl_point.l();
  if (center_s > end_sl_point.s() &&
      std::fabs(center_l - end_sl_point.l()) <
                kOutRoutingConsiderLateralDistance) {
    const std::string msg = "need to stop for out routing.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp_, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);
    return;
  }
}
void OnLanePlanning::CheckIsNeedToReroutingForBlock() {
  planning::ReferenceLineInfo* best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  bool is_near_rerouting_point = false;
  if (best_ref_info != nullptr) {
    std::string D7_lane_id = "269";
    auto lane_info_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(D7_lane_id));
    if (lane_info_ptr != nullptr) {
      auto routing_end_pose = best_ref_info->get_routing_end().pose();
      double s = 0.0;
      double l = 0.0;
      lane_info_ptr->GetProjection({routing_end_pose.x(), routing_end_pose.y()},
                                   &s, &l);
      if (s > 0 && s < lane_info_ptr->total_length() && std::fabs(l) < 0.5) {
        AINFO << "routing end in target lane, no rerouting for blocked";
        injector_->is_need_rerouting_for_block_ = false;
        return;
      }
    }

    common::PointENU point;
    point.set_x(kReroutingPointX);
    point.set_y(kReroutingPointY);
    double adc_x = best_ref_info->vehicle_state().pose().position().x();
    double adc_y = best_ref_info->vehicle_state().pose().position().y();
    double distance_to_rerouting =
        std::sqrt((adc_x - point.x()) * (adc_x - point.x()) +
                  (adc_y - point.y()) * (adc_y - point.y()));
    if(distance_to_rerouting > 20.0){
      return;
    }
    auto adc_s = (best_ref_info->AdcSlBoundary().start_s() +
                  best_ref_info->AdcSlBoundary().end_s()) *
                 0.5;
    common::SLPoint rerouting_sl;
    if (best_ref_info->reference_line().XYToSL(point, &rerouting_sl)) {
      if(adc_s>rerouting_sl.s()-5.0 && adc_s< rerouting_sl.s()+10.0){
        is_near_rerouting_point = true;
        AINFO<<"adc in rerouting range";
      }
    }
  }
  if (injector_->is_need_rerouting_for_block_) {
    injector_->path_block_count_ = 0;
  }
  if (injector_->path_block_count_ > 100 &&
      injector_->is_need_rerouting_for_block_ == false && is_near_rerouting_point) {
        injector_->is_need_rerouting_for_block_ =true;
  } else {
  }
  AINFO<<"injector_->is_need_rerouting_for_block_ = "<<injector_->is_need_rerouting_for_block_;
}
void OnLanePlanning::CheckStopForWheelcrane(
    ADCTrajectory* const ptr_trajectory_pb) {
  if (ptr_trajectory_pb != nullptr) {
    bool is_short_trajectory = false;
    double trajectory_length = std::numeric_limits<double>::max();
    if (ptr_trajectory_pb != nullptr) {
      if (!ptr_trajectory_pb->trajectory_point().empty()) {
        trajectory_length =
            ptr_trajectory_pb->trajectory_point()
                .at(ptr_trajectory_pb->trajectory_point().size() - 1)
                .path_point()
                .s() -
            ptr_trajectory_pb->trajectory_point().at(0).path_point().s();
        if (trajectory_length < kMinTrajectoryLength) {
          is_short_trajectory = true;
        }
      } else {
        AINFO << "trajectory is null";
        is_short_trajectory = true;
      }
    }
    bool is_near_stop_wall = false;
    if (ptr_trajectory_pb->decision().main_decision().has_stop() &&
        ptr_trajectory_pb->decision().main_decision().stop().reason_code() ==
            StopReasonCode::STOP_REASON_WHEEL_CRANE) {
      const auto& stop_point =
          ptr_trajectory_pb->decision().main_decision().stop().stop_point();
      double distance_to_stop_point = std::sqrt(
          std::pow(stop_point.x() - injector_->vehicle_state()->x(), 2) +
          std::pow(stop_point.y() - injector_->vehicle_state()->y(), 2));
      if (distance_to_stop_point < kDistacneToWheelCraneStopWall) {
        is_near_stop_wall = true;
      }
    }
    if (ptr_trajectory_pb->decision().main_decision().stop().reason_code() ==
            StopReasonCode::STOP_REASON_WHEEL_CRANE &&
        is_short_trajectory && is_near_stop_wall) {
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::UNTREATED);
      borrow_response.set_block_obs_id("");
      borrow_response.set_has_response(false);
      injector_->set_borrow_response(borrow_response);
      injector_->borrow_response().clear_block_obs_id();
    }
  }
}
void OnLanePlanning::CheckEnableBorrow(ADCTrajectory* const ptr_trajectory_pb) {
  bool is_short_trajectory = false;
  double trajectory_length = std::numeric_limits<double>::max();
  if (ptr_trajectory_pb != nullptr) {
    if (!ptr_trajectory_pb->trajectory_point().empty()) {
      trajectory_length =
          ptr_trajectory_pb->trajectory_point()
              .at(ptr_trajectory_pb->trajectory_point().size() - 1)
              .path_point()
              .s() -
          ptr_trajectory_pb->trajectory_point().at(0).path_point().s();
      // AINFO << "trajectory_length = " << trajectory_length;
      if (trajectory_length < kMinTrajectoryLength) {
        is_short_trajectory = true;
      }
    } else {
      AINFO << "trajectory is null";
      is_short_trajectory = true;
    }
  }
  planning::ReferenceLineInfo* best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  bool is_stop_reason_obs = false;
  std::string block_obs_id = "";
  double stop_min_s = std::numeric_limits<double>::max();
  if (ptr_trajectory_pb->decision().main_decision().stop().reason_code() ==
      StopReasonCode::STOP_REASON_OBSTACLE) {
    is_stop_reason_obs = true;
    // AINFO << "STOP_REASON_OBSTACLE";
    const auto& object_decision =
        ptr_trajectory_pb->decision().object_decision();

    for (auto decision : object_decision.decision()) {
      for (const auto& object_decision_type : decision.object_decision()) {
        if (object_decision_type.has_stop()) {
          if (object_decision_type.stop().reason_code() ==
              StopReasonCode::STOP_REASON_OBSTACLE) {
            if (best_ref_info != nullptr) {
              common::SLPoint stop_sl;
              if (best_ref_info->reference_line().XYToSL(
                      object_decision_type.stop().stop_point(), &stop_sl)) {
                if (stop_sl.s() < stop_min_s) {
                  stop_min_s = stop_sl.s() ;
                  block_obs_id = std::to_string(decision.perception_id());
                  injector_->planning_context()
                      ->mutable_planning_status()
                      ->mutable_top_bull()
                      ->set_block_obs_id(block_obs_id);
                }
              }
            }
          }
        }
      }
    }
  }
  // if selfborrow faild , need to laneborrow request.
  bool is_stop_reason_boundary = false;
  std::string block_boundary_id = "";
  bool is_single_lane = false;


  double route_s = 0.0;

  hdmap::Id neighbor_lane_id;

  if (nullptr != best_ref_info) {
    for (const auto& seg : best_ref_info->Lanes()) {
      route_s += seg.end_s - seg.start_s;
    }
    for (double t_delta_s = 0; t_delta_s < route_s; t_delta_s += 5) {
      if (t_delta_s < best_ref_info->AdcSlBoundary().end_s()) {
        continue;
      }
      if (t_delta_s >
          best_ref_info->AdcSlBoundary().end_s() + kDistanceBeforeBorrow) {
        break;
      }
      if (!best_ref_info->HasNeighborLane(t_delta_s)) {
        is_single_lane = true;
        break;
      }
    }
  }

  if (!is_stop_reason_obs) {
    if (nullptr != best_ref_info) {
      auto* st_graph_data = best_ref_info->mutable_st_graph_data();
      double min_s = std::numeric_limits<double>::max();

      // get nearst block obs
      for (const STBoundary* boundary : st_graph_data->st_boundaries()) {
        // static obs
        if (std::fabs(boundary->obs_v()) > kStopSpeed) {
          // AINFO << "boundary->obs_v() = " << boundary->obs_v();
          continue;
        }
        // must cross all time.
        if (boundary->bottom_right_point().t() < kMaxPredictionTime) {
          continue;
        }
        double temp_min_s = boundary->bottom_left_point().s();
        if (temp_min_s < min_s) {
          min_s = temp_min_s;
          block_boundary_id = boundary->id();
        }
      }
      if (min_s < kMinTrajectoryLength) {
        is_stop_reason_boundary = true;
      }
    }
  }

  // AINFO << "is_stop_reason_boundary = " << is_stop_reason_boundary;
  bool is_stop_reason_stacker = false;
  if (ptr_trajectory_pb->decision().main_decision().stop().reason_code() ==
      StopReasonCode::STOP_REASON_STACKER) {
    is_stop_reason_stacker = true; 
      ADEBUG << "STOP_REASON_STACKER  "<<is_stop_reason_stacker;
  }
  double adc_velocity = std::fabs(injector_->vehicle_state()->linear_velocity());
  // Keep requesting without receiving a response.
  if (is_short_trajectory && is_stop_reason_obs &&
      !injector_->borrow_response().has_response() &&
      adc_velocity < kBorrowRequestSpeed) {
    // AINFO << "NEED REQUEST BORROW";
    ptr_trajectory_pb->set_borrow_request(true);
    ptr_trajectory_pb->set_block_obs_id(block_obs_id);
  } else if (is_short_trajectory && is_stop_reason_boundary &&
             !injector_->borrow_response().has_response() &&
             adc_velocity < kBorrowRequestSpeed) {
    // AINFO << "NEED REQUEST BORROW FOR BOUNDARY BLOCK.";
    ptr_trajectory_pb->set_borrow_request(true);
    ptr_trajectory_pb->set_block_obs_id(block_boundary_id);
  } else if (is_short_trajectory && is_stop_reason_obs  &&
      adc_velocity < kBorrowRequestSpeed) {
    ptr_trajectory_pb->set_block_obs_id(block_obs_id);
  } else if (is_short_trajectory && is_stop_reason_boundary &&
             adc_velocity < kBorrowRequestSpeed) {
    ptr_trajectory_pb->set_block_obs_id(block_boundary_id);
  }
  // if obs is igv
  if (best_ref_info != nullptr) {
    // if no block ,obs is null.
    const Obstacle* target_obs =
        best_ref_info->path_decision()->obstacles().Find(
            ptr_trajectory_pb->block_obs_id());
            if (target_obs != nullptr && injector_->borrow_response().has_response() &&
        injector_->borrow_response().block_obs_id() != target_obs->Id()) {
      if (target_obs->IsIgv() && (!injector_->planning_context()
                                      ->mutable_planning_status()
                                      ->has_top_bull() || (injector_->planning_context()
                                      ->mutable_planning_status()
                                      ->has_top_bull() && !injector_->planning_context()
                                      ->mutable_planning_status()
                                      ->top_bull().is_in_top_bull()))) {
        AINFO << "CLEAR REQUEST AND CLEAR STATES";
        ptr_trajectory_pb->set_borrow_request(true);
        ptr_trajectory_pb->set_block_obs_id(target_obs->Id());
        planning::BorrowResponse borrow_response;
        borrow_response.set_response_type(planning::ResponseType::UNTREATED);
        borrow_response.set_block_obs_id("");
        borrow_response.set_has_response(false);
        injector_->set_borrow_response(borrow_response);
        // need clear borrow state
        // initialize laneborrow state
        need_reset_borrow_state_ = true;
      }
    }
  }

   AINFO<<"ptr_trajectory_pb->borrow_request() = "<<ptr_trajectory_pb->borrow_request();
  // AINFO<<"is_need_backward_for_turn_blocking_ = "<<is_need_backward_for_turn_blocking_;
  if(best_ref_info!=nullptr){
    // trigger only one
    if(best_ref_info->IsNearTurn() && !is_need_backward_for_turn_blocking_ && FLAGS_enable_backward_in_turn){
      // if no block ,obs is null.
   const Obstacle* target_obs= best_ref_info->path_decision()->obstacles().Find(ptr_trajectory_pb->block_obs_id());
   //15m 
   if(target_obs != nullptr){
    auto obs_sl = target_obs->PerceptionSLBoundary();
    // AINFO<<"obs_sl .start_s = "<<obs_sl.start_s();
    auto adc_sl = best_ref_info->AdcSlBoundary();
        // AINFO<<"adc_sl.end_s = "<<adc_sl.end_s();
        // AINFO<<"is_need_backward_for_turn_blocking_ = "<<is_need_backward_for_turn_blocking_;
        common::SLPoint dest_sl;
        best_ref_info->reference_line().XYToSL(best_ref_info->get_routing_end().pose(),
                        &dest_sl);
        double distance_between_start_and_end = dest_sl.s() - adc_sl.end_s();
        // AINFO << "distance_between_start_and_end = "
        //       << distance_between_start_and_end;
        bool is_near_destinaton =
            (std::fabs(dest_sl.l()) < kLateralDistanceToDestination) &&
            (distance_between_start_and_end < kDestinationNoBorrowDistance) &&
            (distance_between_start_and_end > kTiny);
        if(((obs_sl.start_s()-adc_sl.end_s()) < kDistanceToBorrowObs )
        && ((obs_sl.start_s()-adc_sl.end_s()) >1.0) && !is_need_backward_for_turn_blocking_
        && !is_near_destinaton){
        //ptr_trajectory_pb->set_borrow_request(false);
        need_backward_count_++;
        if(need_backward_count_ > kBackwardCount){
          // AINFO<<"NEED TURN BACKWARD";
          is_need_backward_for_turn_blocking_ = true;
          if(!injector_->need_back_ward_ && is_need_backward_for_turn_blocking_ ){
            injector_->back_ward_distance_ = kDistanceToBorrowObs - (obs_sl.start_s()-adc_sl.end_s());
    }
   }
        }else{
          need_backward_count_ = 0;
        }
      }else{
        // AINFO<<"NO BLOCK OBS";
      }
    }
  }
  // if block obs is igv ,no request borrow.
  bool is_block_satcker_obs = false;
  if(best_ref_info!=nullptr){
    auto* path_decision = best_ref_info->path_decision();
    Obstacle* block_boundary = path_decision->Find(block_boundary_id);
    Obstacle* block_obs = path_decision->Find(block_obs_id);
    // AINFO<<"block_boundary_id = "<<block_boundary_id;
    // AINFO<<"block_obs_id = "<<block_obs_id;
    if(block_boundary != nullptr){
      const auto& obs_type = block_boundary->Perception().type();
      if (obs_type == perception::PerceptionObstacle::STACKER ||
          obs_type == perception::PerceptionObstacle::FORKLIFT_STACKER ||
          obs_type == perception::PerceptionObstacle::WHEELCRANE ||
          block_boundary_id.find("STACKER") != std::string::npos ||
          block_boundary_id.find("WHEEL") != std::string::npos) {
        is_block_satcker_obs = true;
      }
      if(block_boundary->IsIgv()){
        // ptr_trajectory_pb->set_borrow_request(false);
      }
    }
    if(block_obs != nullptr){
      const auto& obs_type = block_obs->Perception().type();
      if (obs_type == perception::PerceptionObstacle::STACKER ||
          obs_type == perception::PerceptionObstacle::FORKLIFT_STACKER ||
          obs_type == perception::PerceptionObstacle::WHEELCRANE ||
          block_boundary_id.find("STACKER") != std::string::npos ||
          block_boundary_id.find("WHEEL") != std::string::npos) {
        is_block_satcker_obs = true;
      }
      if(block_obs->IsIgv()){
        // ptr_trajectory_pb->set_borrow_request(false);
      }
    }

  }
  AINFO << "adc_velocity = " << adc_velocity;
  AINFO << "trajectory_length = " << trajectory_length;
  if (best_ref_info != nullptr && ptr_trajectory_pb->borrow_request() &&
      adc_velocity < kAutoBorrowConsiderSpeed &&
      trajectory_length < kAutoBorrowConsiderTrajectoryLength &&
      FLAGS_enable_auto_borrow_for_one_obs) {
    auto* path_decision = best_ref_info->path_decision();
    // when stop,need to get real obs.
    Obstacle* block_boundary = path_decision->Find(block_boundary_id);
    Obstacle* block_obs =
        path_decision->Find(ptr_trajectory_pb->block_obs_id());
    double min_s = std::numeric_limits<double>::max();
    double consider_min_l = 0.0;
    double consider_max_l = 0.0;
    double consider_min_s = 0.0;
    double consider_max_s = 0.0;
    bool is_stacker_obs_block = false;
    bool is_igv_block = false;
    bool is_dest = false;
    if (block_boundary != nullptr) {
      const auto& obs_type = block_boundary->Perception().type();
      if (perception::PerceptionObstacle::STACKER == obs_type ||
          perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type ||
          block_boundary->Id().find("STACKER") != std::string::npos ||
          block_boundary->Id().find("wheel_crane") != std::string::npos) {
        is_stacker_obs_block = true;
      }
      if (block_boundary->Id().find("DEST") != std::string::npos) {
        is_dest = true;
      }
      if(block_boundary->IsIgv()){
        is_igv_block = true;
      }
      AINFO << "block_boundary = " << block_boundary->Id();
      AINFO << "obs_type = " << obs_type;
      if (block_boundary->PerceptionSLBoundary().end_s() < min_s &&
          !is_stacker_obs_block&& !is_igv_block) {
        min_s = block_boundary->PerceptionSLBoundary().start_s();
        consider_min_l = block_boundary->PerceptionSLBoundary().start_l();
        consider_max_l = block_boundary->PerceptionSLBoundary().end_l();
        consider_min_s = block_boundary->PerceptionSLBoundary().start_s();
        consider_max_s = block_boundary->PerceptionSLBoundary().end_s();
      }
    }
    if (block_obs != nullptr) {
      if (block_obs->PerceptionSLBoundary().end_s() < min_s) {
        min_s = block_obs->PerceptionSLBoundary().start_s();
        consider_min_l = block_obs->PerceptionSLBoundary().start_l();
        consider_max_l = block_obs->PerceptionSLBoundary().end_l();
        consider_min_s = block_obs->PerceptionSLBoundary().start_s();
        consider_max_s = block_obs->PerceptionSLBoundary().end_s();
      }
      AINFO << "block_obs = " << block_obs->Id();

      const auto& obs_type = block_obs->Perception().type();
      AINFO << "obs_type = " << obs_type;
      if (perception::PerceptionObstacle::STACKER == obs_type ||
          perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type ||
          perception::PerceptionObstacle::WHEELCRANE == obs_type ||
          block_obs->Id().find("STACKER") != std::string::npos ||
          block_obs->Id().find("WHEEL_CRANE") != std::string::npos) {
        is_stacker_obs_block = true;
      }
      if (block_obs->IsIgv()) {
        is_igv_block = true;
      }
    }

    const double adc_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width();
    const auto& lane_width = best_ref_info->GetLaneWidthByS(min_s);
    double left_width = lane_width.first;
    double right_width = lane_width.second;
    double left_space = left_width;
    double right_space = right_width;
    const auto& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    double ego_width = vehicle_param.width();
    if (!best_ref_info->reference_line().GetRoadWidth(min_s, &left_space,
                                                      &right_space)) {
      AWARN << "Failed to get lane width at s = " << min_s;
    }
    AINFO << "left road space = " << left_space;
    AINFO << "right road space = " << right_space;

    double shrink_min_l = consider_min_l;
    double shrink_max_l = consider_max_l;
    double shrink_min_s = consider_min_s;
    double shrink_max_s = consider_max_s;
    consider_min_l = consider_min_l - adc_width - kAutoBorrowLateralBuffer;
    consider_max_l = consider_max_l + adc_width + kAutoBorrowLateralBuffer;
    consider_min_s = consider_min_s - kAutoBorrowLateralBuffer;
    consider_max_s = consider_min_s + 20.0 + kAutoBorrowLateralBuffer;

    AINFO << "consider_max_l = " << consider_max_l;
    if (min_s < best_ref_info->reference_line().Length()) {
      auto* path_decision = best_ref_info->path_decision();
      const auto& adc_sl = best_ref_info->AdcSlBoundary();
      bool is_front_blocking_obs = false;
      for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
        Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
        if (ptr_obstacle == nullptr) {
          continue;
        }
        if (ptr_obstacle->Id() == block_boundary_id ||
            ptr_obstacle->Id() == block_obs_id) {
          continue;
        }
        if (ptr_obstacle_item->IsVirtual()) {
          continue;
        }
        if (ptr_obstacle_item->speed() > 0.1) {
          continue;
        }
        const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
        bool obs_in_consider_box = obs_sl.start_l() > consider_min_l &&
                                   obs_sl.end_l() < consider_max_l &&
                                   obs_sl.start_s() > consider_min_s &&
                                   obs_sl.end_s() < consider_max_s;
        // AINFO << "ptr_obstacle_item = " << ptr_obstacle_item->Id();
        // AINFO << "obs_in_consider_box = " << obs_in_consider_box;
        if (obs_in_consider_box) {
          if (obs_sl.start_l() < shrink_min_l) {
            shrink_min_l = obs_sl.start_l();
          }
          if (obs_sl.end_l() > shrink_max_l) {
            shrink_max_l = obs_sl.end_l();
          }
          if (obs_sl.start_s() < shrink_min_s) {
            shrink_min_s = obs_sl.start_s();
          }
          if (obs_sl.end_s() > shrink_max_s) {
            shrink_max_s = obs_sl.end_s();
          }
        }
      }
      // get borrow box
      AINFO << "shrink_min_l = " << shrink_min_l;
      AINFO << "shrink_max_l = " << shrink_max_l;
      AINFO << "shrink_min_s = " << shrink_min_s;
      AINFO << "shrink_max_s = " << shrink_max_s;

      double min_s_left_space = left_width;
      double min_s_right_space = right_width;
      double max_s_left_space = left_width;
      double max_s_right_space = right_width;
      if (!best_ref_info->reference_line().GetRoadWidth(
              shrink_min_s, &min_s_left_space, &min_s_right_space)) {
        AWARN << "Failed to get lane width at s = " << shrink_min_s;
      }
      if (!best_ref_info->reference_line().GetRoadWidth(
              shrink_max_s, &max_s_left_space, &max_s_right_space)) {
        AWARN << "Failed to get lane width at s = " << shrink_max_s;
      }
      left_space = std::min(left_space, min_s_left_space);
      left_space = std::min(left_space, max_s_left_space);
      right_space = std::min(right_space, min_s_right_space);
      right_space = std::min(right_space, max_s_right_space);
      AINFO << "left_space = " << left_space;
      AINFO << "right space = " << right_space;
      if (left_space > ego_width + 1.5 || right_space > ego_width + 1.5) {
        AINFO << "has space to borrow";
        bool is_left_has_block_obs = false;
        bool is_right_has_block_obs = false;
        for (const auto* ptr_obstacle_item :
             path_decision->obstacles().Items()) {
          Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
          // const auto& obs_type = ptr_obstacle_item->Perception().type();
          if (ptr_obstacle == nullptr) {
            continue;
          }
          // AINFO<<"ptr_obstacle->Id() = "<<ptr_obstacle->Id();
          if (ptr_obstacle->Id() == block_boundary_id ||
              ptr_obstacle->Id() == block_obs_id) {
            // AINFO<<"CONTINUE";
            continue;
          }
          if (ptr_obstacle_item->IsVirtual()) {
            // AINFO<<"CONTINUE";
            continue;
          }
          if (std::fabs(ptr_obstacle_item->speed()) > kStopSpeed) {
            // AINFO<<"CONTINUE";
            continue;
          }
          const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
          if (obs_sl.start_s() - shrink_max_s > kDistanceToreturnDistance ||
              obs_sl.end_s() < adc_sl.start_s()) {
            // AINFO<<"CONTINUE";
            continue;
          }
          if (obs_sl.start_l() >= shrink_min_l &&
              obs_sl.end_l() <= shrink_max_l &&
              obs_sl.start_s() >= shrink_min_s &&
              obs_sl.end_s() <= shrink_max_s) {
            continue;
          }
          bool is_misplacement_with_need_borrow_obs =
              (obs_sl.start_l() > shrink_max_l ||
               obs_sl.end_l() < shrink_min_l);
          bool is_misplacement_with_adc = (obs_sl.start_l() > adc_sl.end_l() ||
                                           obs_sl.end_l() < adc_sl.start_l());
          AINFO << "is_misplacement_with_need_borrow_obs = "
                << is_misplacement_with_need_borrow_obs;
          AINFO << "is_misplacement_with_adc = " << is_misplacement_with_adc;
          if (is_misplacement_with_need_borrow_obs &&
              is_misplacement_with_adc) {
            // AINFO<<"CONTINUE";
            continue;
          }
          AINFO << "obs_sl.end_l() = " << obs_sl.end_l();
          AINFO << "obs_sl.start_l() = " << obs_sl.start_l();
          AINFO << "shrink_max_l = " << shrink_max_l;
          AINFO << "shrink_min_l = " << shrink_min_l;
          if (!is_misplacement_with_need_borrow_obs) {
            if (obs_sl.end_l() > shrink_max_l) {
              is_left_has_block_obs = true;
            }
            if (obs_sl.start_l() < shrink_min_l) {
              is_right_has_block_obs = true;
            }
            if (obs_sl.end_l() < shrink_max_l &&
                obs_sl.start_l() > shrink_min_l) {
              is_front_blocking_obs = true;
              AINFO << "has lateral placement obs = "
                    << ptr_obstacle_item->Id();
              break;
            }
            if (is_left_has_block_obs || is_right_has_block_obs) {
              if (obs_sl.start_s() > shrink_max_s) {
                is_front_blocking_obs = true;
                AINFO << "has front obs block  = " << ptr_obstacle_item->Id();
              }
            }
          }

          if (is_left_has_block_obs && is_right_has_block_obs) {
            is_front_blocking_obs = true;
            AINFO << "has lateral placement obs = " << ptr_obstacle_item->Id();
            break;
          }
          if (is_misplacement_with_adc) {
            is_front_blocking_obs = true;
            AINFO << "adc front has obs = " << ptr_obstacle_item->Id();
            break;
          }
        }

      } else {
        AINFO << "no space to borrow";
        is_front_blocking_obs = true;
      }

      AINFO << "is_front_blocking_obs = " << is_front_blocking_obs;
      AINFO << "is_stacker_obs_block = " << is_stacker_obs_block;
      AINFO <<"is_igv_block = "<<is_igv_block;
      if (is_front_blocking_obs || is_stacker_obs_block || is_igv_block||is_dest) {
        AINFO << "request for borrow";
        auto_borrow_count_ = 0;
        AINFO << "auto_borrow_count_ = " << auto_borrow_count_;
      } else {
        auto_borrow_count_++;
        auto_borrow_count_ = std::min(100, auto_borrow_count_);
        AINFO << "auto_borrow_count_ = " << auto_borrow_count_;
        if (auto_borrow_count_ > 10) {
          AINFO << "auto borrow for only one obs.";
          planning::BorrowResponse borrow_response;
          ptr_trajectory_pb->set_borrow_request(false);
          borrow_response.set_response_type(planning::ResponseType::ACCEPT);
          borrow_response.set_block_obs_id("auto borrow for only one obs");
          borrow_response.set_has_response(true);
          injector_->set_borrow_response(borrow_response);
        }
      }
    }
  } else {
    auto_borrow_count_ = 0;
  }
  AINFO << "auto_borrow_count_ = " << auto_borrow_count_;
  if ((ptr_trajectory_pb->block_obs_id() == FLAGS_destination_obstacle_id) ||
      (is_single_lane && !injector_->is_adc_in_gate_junction_ &&
       !is_block_satcker_obs) ||
      !is_auto_state_) {
    ptr_trajectory_pb->set_borrow_request(false);
  }
  if (nullptr != best_ref_info) {
    is_in_passing_ = false;
  if (best_ref_info->NeedNoticeStacker().request_for_pass_stacker()) {
    no_stacker_notice_count_ = 0;
  auto* notice_stacker = ptr_trajectory_pb->mutable_notice_stacker();
  notice_stacker->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
  std::string stacker_id = best_ref_info->NeedNoticeStacker().stacker_id();
  std::string prefix = "stacker_";
  if (best_ref_info->NeedNoticeStacker().stacker_id().find("stacker_") !=
      std::string::npos) {
    stacker_id = stacker_id.erase(0, prefix.length());
  }
  std::string notice_type = "";
  if (best_ref_info->NeedNoticeStacker().request_type() ==
      planning::PassStackerRequestType::PASSING) {
    is_in_passing_ = true;
    notice_type = "P";
  } else {
    notice_type = "R";
  }
  if (best_ref_info->NeedNoticeStacker().request_type() ==
      planning::PassStackerRequestType::PASSED) {
    stacker_id = injector_->pass_stacker_id_;
  }
  std::string message_id = stacker_id + "_" + notice_type;
  AINFO << "stacker_id = " << stacker_id;
  AINFO << "last_message_id_.first = " << last_message_id_.first;
  bool is_stacker_id_change = false;
  if (!last_message_id_.first.empty() &&
      last_message_id_.first.find(stacker_id) == std::string::npos) {
    stacker_id_change_count_++;
    is_stacker_id_change = true;
  } else {
    stacker_id_change_count_ = 0;
  }
  AINFO << "stacker_id_change_count_ = " << stacker_id_change_count_;
  if (message_id != last_message_id_.first && !is_stacker_id_change) {
    // if (message_id != last_message_id_.first) {
    last_message_id_.first = message_id;
    last_message_id_.second = last_message_id_.second + 1;
  } else {
  }
  if (message_id != last_message_id_.first && is_stacker_id_change &&
      stacker_id_change_count_ > 3) {
    last_message_id_.first = message_id;
    last_message_id_.second = last_message_id_.second + 1;
  }

  message_id =
      last_message_id_.first + "_" + std::to_string(last_message_id_.second);
  AINFO << "message_id = " << message_id;
  notice_stacker->set_message_id(message_id);
  notice_stacker->set_stacker_id(stacker_id);
  notice_stacker->set_request_type(best_ref_info->NeedNoticeStacker().request_type());
  notice_stacker->set_request_for_pass_stacker(true);
  } else {
    auto* notice_stacker = ptr_trajectory_pb->mutable_notice_stacker();
    notice_stacker->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    std::string stacker_id = injector_->pass_stacker_id_;
    std::string prefix = "stacker_";
    if (stacker_id.find("stacker_") != std::string::npos) {
      stacker_id = stacker_id.erase(0, prefix.length());
    }
    std::string notice_type = "";
    if (best_ref_info->NeedNoticeStacker().request_type() ==
        planning::PassStackerRequestType::PASSING) {
      notice_type = "P";
    } else {
      notice_type = "R";
    }
    if (!stacker_id.empty()) {
      std::string message_id = stacker_id + "_" + notice_type;
      // if(!message_id.empty() && !last_message_id_.first.empty()){
      if (message_id != last_message_id_.first) {
        last_message_id_.first = message_id;
        last_message_id_.second = last_message_id_.second + 1;
      }
      // }
      message_id = last_message_id_.first + "_" +
                   std::to_string(last_message_id_.second);
      AINFO << "message_id = " << message_id;
      notice_stacker->set_message_id(message_id);
      notice_stacker->set_stacker_id(stacker_id);
      notice_stacker->set_request_type(
          planning::PassStackerRequestType::PASS_DEFAULT);
      notice_stacker->set_request_for_pass_stacker(true);
    }else{
      no_stacker_notice_count_ = 0;
    }
  }
  // check need satcker notice
  bool no_stacker_notice = false;
  auto* notice_stacker = ptr_trajectory_pb->mutable_notice_stacker();
  if (notice_stacker->stacker_id() == "" ||
      !notice_stacker->has_request_for_pass_stacker()) {
    no_stacker_notice = true;
  }
   if(notice_stacker->has_request_for_pass_stacker()){
    if(notice_stacker->request_for_pass_stacker()){
      if(notice_stacker->request_type() == planning::PassStackerRequestType::PASS_DEFAULT){
        no_stacker_notice_count_++;
        no_stacker_notice_count_ = std::min(kLargeCountTimes,no_stacker_notice_count_);
      }
      if(no_stacker_notice_count_ > kNoWheelCraneCountTimes){
        no_stacker_notice = true;
      }
    }else{
      no_stacker_notice = true;
    }
  }
  // no stacker and need notice for wheelcrane
  if (no_stacker_notice) {
    if (injector_->wheelcrane_notice_.request_for_pass_stacker()) {
      // check need wheelcrane notice
      notice_stacker->mutable_header()->set_timestamp_sec(
          Clock::NowInSeconds());
      std::string wheelcrane_id = injector_->wheelcrane_notice_.stacker_id();
      std::string notice_type = "";
      if (injector_->wheelcrane_notice_.request_type() ==
          planning::PassStackerRequestType::PASSING) {
        notice_type = "P";
      } else {
        notice_type = "R";
      }
      if (!wheelcrane_id.empty()) {
        std::string message_id = wheelcrane_id + "_" + notice_type;
        if (message_id != last_wheelcrane_message_id_.first) {
          last_wheelcrane_message_id_.first = message_id;
          last_wheelcrane_message_id_.second =
              last_wheelcrane_message_id_.second + 1;
        }
        message_id = last_wheelcrane_message_id_.first + "_" +
                     std::to_string(last_wheelcrane_message_id_.second);
        notice_stacker->set_message_id(message_id);
        notice_stacker->set_stacker_id(wheelcrane_id);
        notice_stacker->set_request_type(
            injector_->wheelcrane_notice_.request_type());
        notice_stacker->set_request_for_pass_stacker(true);
      } else {
        last_wheelcrane_message_id_.first = "";
        last_wheelcrane_message_id_.second = 0;
      }
    } else {
      // no wheelcrane ,clear last wheelcrane message
      last_wheelcrane_message_id_.first = "";
      last_wheelcrane_message_id_.second = 0;
    }
  }
}
  if (nullptr != best_ref_info) {
  if (best_ref_info->HasPassStackerRequest().request_for_pass_stacker()) {
    // AINFO << "NEED PASS STACKER REQUEST";
  auto* stacker_request = ptr_trajectory_pb->mutable_pass_stacker_request();
  stacker_request->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
  std::string stacker_id = best_ref_info->HasPassStackerRequest().stacker_id();
  std::string prefix = "stacker_";
  if (best_ref_info->HasPassStackerRequest().stacker_id().find("stacker_") !=
      std::string::npos) {
    stacker_id = stacker_id.erase(0, prefix.length());
  }
  stacker_request->set_stacker_id(stacker_id);
  stacker_request->set_request_for_pass_stacker(true);
  }else{
    auto* pass_stacker_request = ptr_trajectory_pb->mutable_pass_stacker_request();
    pass_stacker_request->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    pass_stacker_request->set_stacker_id("");
    pass_stacker_request->set_request_for_pass_stacker(false);
  }}
  // Keep requesting without receiving a response.

  if(FLAGS_enable_check_times_for_borrow_request){
    if(ptr_trajectory_pb->borrow_request() && !last_trajectory_has_borrow_request_){
      enable_borrow_request_count_++;
      enable_borrow_request_count_ = std::min(10,enable_borrow_request_count_);
      if(enable_borrow_request_count_!=10){
        ptr_trajectory_pb->set_borrow_request(false);
        ptr_trajectory_pb->set_block_obs_id("");
      }
    }else{
      enable_borrow_request_count_--;
      enable_borrow_request_count_ =  std::max(0,enable_borrow_request_count_);
    }
     AINFO<<"enable_borrow_request_count_ = "<<enable_borrow_request_count_;


     AINFO<<"last_trajectory_has_borrow_request_ = "<<last_trajectory_has_borrow_request_;
     AINFO<<"ptr_trajectory_pb->borrow_request() = "<<ptr_trajectory_pb->borrow_request();
    if(!ptr_trajectory_pb->borrow_request() && 
    last_trajectory_has_borrow_request_ && 
    !injector_->pass_stacker_response().has_response()){
      no_borrow_request_count_++;
      no_borrow_request_count_ = std::min(10,no_borrow_request_count_);
      if(no_borrow_request_count_!=10){
        ptr_trajectory_pb->set_borrow_request(true);
        ptr_trajectory_pb->set_block_obs_id(last_block_obs_id_);
      }
    }else{
      no_borrow_request_count_--;
      no_borrow_request_count_ = std::max(0,no_borrow_request_count_);
    }
    // AINFO<<"no_borrow_request_count_ = "<<no_borrow_request_count_;
    // AINFO<<"ptr_trajectory_pb->borrow_request() = "<<ptr_trajectory_pb->borrow_request();
    if(ptr_trajectory_pb->borrow_request()){
      last_trajectory_has_borrow_request_ = true;
      last_block_obs_id_ = ptr_trajectory_pb->block_obs_id();
    }else{
      last_trajectory_has_borrow_request_ = false;
    }
    if (ptr_trajectory_pb->borrow_request() && FLAGS_enable_auto_lane_borrow) {
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::ACCEPT);
      borrow_response.set_block_obs_id(ptr_trajectory_pb->block_obs_id());
      borrow_response.set_has_response(true);
      injector_->set_borrow_response(borrow_response);
    }
  }

}

double OnLanePlanning::GetReverseTrajectoryDistance(
    const century::common::TrajectoryPoint& start_point,
    planning::ReferenceLineInfo* best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  CalRoutingRequestReversePoint();
  CalTaskStartReverseObstacleAvoidancePoint(best_ref_info);
      CalReverseObstacleAvoidancePoint(best_ref_info, ptr_trajectory_pb);

  if (!injector_->use_reverse_trajectory()) {
    return 0.0;
  }

  AINFO << "stop point = " << injector_->reverse_stop_point().x() << "  "
        << injector_->reverse_stop_point().y();
  AINFO << "start_point.path_point().x()  = " << start_point.path_point().x()
        << "   " << start_point.path_point().y();
  double distance_to_stop_point = std::sqrt(
      (start_point.path_point().x() - injector_->reverse_stop_point().x()) *
          (start_point.path_point().x() - injector_->reverse_stop_point().x()) +
      (start_point.path_point().y() - injector_->reverse_stop_point().y()) *
          (start_point.path_point().y() - injector_->reverse_stop_point().y()));
  AINFO << "distance_to_stop_point = " << distance_to_stop_point;
  if (distance_to_stop_point > kBackTrajectoryMaxDistance) {
    distance_to_stop_point = kBackTrajectoryMaxDistance;
  }
  if (injector_->use_reverse_type() == 
          ReverseTrajectoryType::TASK_POINT || 
        injector_->use_reverse_type() == 
          ReverseTrajectoryType::TASK_POINT_REVERSE_END) {
    if (distance_to_stop_point < kMinStopDistance) {
      AINFO << "=======TASK_POINT_REVERSE_END=====";
      injector_
            ->set_use_reverse_type(ReverseTrajectoryType::TASK_POINT_REVERSE_END);
      ptr_trajectory_pb->set_gear(vehicle_gear_);
      GenerateStopTrajectory(ptr_trajectory_pb);
    }
    // arrived, sent borrow request
    if (injector_->use_reverse_type() == 
        ReverseTrajectoryType::TASK_POINT_REVERSE_END && 
        (injector_->borrow_response().has_response() || 
            IsStackerSafeLonDistance(best_ref_info))) {
          AINFO << "===TASK_POINT_REVERSE_END" 
                << "-->injector_->borrow_response().has_response():"
                << injector_->borrow_response().has_response()
                << "====";
          injector_->set_use_reverse_trajectory(false);
          injector_
              ->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
          ptr_trajectory_pb->set_gear(vehicle_gear_);
    }
  } else {
    bool is_far_away_obs = IsObstacleFarAway(best_ref_info);
    AINFO << "distance_to_stop_point = " << distance_to_stop_point;
    AINFO << "is_far_away_obs = " << is_far_away_obs;
    if (injector_->need_back_ward_ || is_need_backward_for_turn_blocking_) {
      // overceed the end point,stop
      double adc_heading = injector_->vehicle_state()->heading();
      double adc_x = injector_->vehicle_state()->pose().position().x();
      double adc_y = injector_->vehicle_state()->pose().position().y();
      double end_x = injector_->reverse_stop_point().x();
      double end_y = injector_->reverse_stop_point().y();
      double dx = adc_x - end_x;
      double dy = adc_y - end_y;
      double heading_end_to_adc = std::atan2(dy, dx);  // calculate the heading
      AINFO << "heading_end_to_adc = " << heading_end_to_adc;
      AINFO << "adc_heading = " << adc_heading;
      double diff_theta_between_adc_and_end_point =
          common::math::NormalizeAngle(heading_end_to_adc - adc_heading);
      AINFO << "diff_theta_between_adc_and_end_point = "
            << diff_theta_between_adc_and_end_point;
      bool need_stop = false;
      if (std::fabs(diff_theta_between_adc_and_end_point) > M_PI_2) {
        need_stop = true;
      }
      if (distance_to_stop_point < kMinStopDistance || need_stop) {
        AINFO << "====finsh use reverse trajectory--->distance_to_stop_point:"
              << distance_to_stop_point << "=====";
        injector_->need_back_ward_ = false;
        is_need_backward_for_turn_blocking_ = false;
        injector_->back_ward_check_times_ = 0;
        injector_->set_use_reverse_trajectory(false);
        injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
        ptr_trajectory_pb->set_gear(vehicle_gear_);
        GenerateStopTrajectory(ptr_trajectory_pb);
    }
  } else {
      // overceed the end point,stop
      double adc_heading = injector_->vehicle_state()->heading();
      double adc_x = injector_->vehicle_state()->pose().position().x();
      double adc_y = injector_->vehicle_state()->pose().position().y();
      double end_x = injector_->reverse_stop_point().x();
      double end_y = injector_->reverse_stop_point().y();
      double dx = adc_x - end_x;
      double dy = adc_y - end_y;
      double heading_end_to_adc = std::atan2(dy, dx);  // calculate the heading
      double diff_theta_between_adc_and_end_point =
          common::math::NormalizeAngle(heading_end_to_adc - adc_heading);
      bool need_stop = false;
      if (std::fabs(diff_theta_between_adc_and_end_point) > M_PI_2) {
        need_stop = true;
      }
      if ((is_far_away_obs && ReverseTrajectoryType::OBSTACLE_AVOIDANCE ==
                                  injector_->use_reverse_type()) ||
          distance_to_stop_point < kMinStopDistance || need_stop) {
        injector_->set_use_reverse_trajectory(false);
        injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
      ptr_trajectory_pb->set_gear(vehicle_gear_);
      GenerateStopTrajectory(ptr_trajectory_pb);
      }
    }
  }
  return distance_to_stop_point;
}

void OnLanePlanning::GetReverseSpeedData(
    planning::ReferenceLineInfo* best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb,
    double distance_to_stop_point) {
  ref_v_list_.clear();
  const double adc_speed =
      std::fabs(injector_->vehicle_state()->linear_velocity());
  if (nullptr == best_ref_info) {
    AINFO << " no best info";
    return;
  }

  double total_length =
      std::fabs(best_ref_info->path_data().discretized_path().back().s() -
                best_ref_info->path_data().discretized_path().front().s());
  double ori_length =
      std::fabs(best_ref_info->path_data().discretized_path().back().s() -
                best_ref_info->path_data().discretized_path().front().s());
  AINFO << "total_length = " << total_length
        << " ori_length = " << ori_length;

  if (total_length < kEpsilon && distance_to_stop_point > 0.1) {
    if (injector_->use_reverse_type() == ReverseTrajectoryType::TASK_POINT ||
        injector_->use_reverse_type() ==
            ReverseTrajectoryType::TASK_POINT_REVERSE_END) {
      injector_->set_use_reverse_type(
          ReverseTrajectoryType::TASK_POINT_REVERSE_END);
      GenerateStopTrajectory(ptr_trajectory_pb);
      return;
    } else {
      AINFO << "set_use_reverse_trajectory is false";
      injector_->set_use_reverse_trajectory(false);
      // injector_->set_use_reverse_obstacle_avoidance(false);
      injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
      injector_->need_back_ward_ = false;
      is_need_backward_for_turn_blocking_ = false;
      injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
      GenerateStopTrajectory(ptr_trajectory_pb);
      return;
    }
  }
  // collision check ,get remind path length.
  total_length = GetNewReverseDistance(best_ref_info);
  AINFO << "total_length = " << total_length;
  total_length = std::max(total_length - 0.1, kEpsilon);
  if (std::fabs(total_length - ori_length) > 1.0) {
    total_length =
        std::max(total_length - kBackTrajectoryStopDistance, kEpsilon);
  }
  // total_length = std::max(total_length - 0.1, kEpsilon);
  bool is_short_length = false;
  if (total_length > kLowLowDistacne && total_length < kLowUpDistacne) {
    is_short_length = true;
  }
  AINFO << "PATH LENGTH = " << total_length;
  AINFO << "adc_speed = " << adc_speed;
  if (std::fabs(adc_speed) <= kReverseStopSpeed &&
      (total_length <= kMinStopDistance || is_short_length)) {
    AINFO << "STOP";
    GenerateStopTrajectory(ptr_trajectory_pb);
    if (injector_->use_reverse_type() == ReverseTrajectoryType::TASK_POINT) {
      injector_->set_use_reverse_type(
          ReverseTrajectoryType::TASK_POINT_REVERSE_END);
    } else if (std::fabs(total_length - ori_length) < 0.3) {
      AINFO << "STOP --> set_use_reverse_trajectory(false)";
      injector_->set_use_reverse_trajectory(false);
      injector_->need_back_ward_ = false;
      is_need_backward_for_turn_blocking_ = false;
      injector_->back_ward_check_times_ = 0;
      injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    }
    return;
  } else {
    // If there is still speed and the target length is already very small,
    // apply smooth braking.
    double target_v = kReverseTargetspeed;
    double begin_break_s;
    std::vector<std::pair<double, double>> ref_s_list;
    std::vector<std::pair<double, double>> ref_v_list;
    double comfort_decel = kComfortDecel;
    double comfort_acc = kComfortAcc;
    double comfort_decel_distance =
        adc_speed * adc_speed / (comfort_decel * 2.0);
    if (total_length < 0.001) {
      total_length = comfort_decel_distance;
    }
    AINFO << "comfort_decel_distance = " << comfort_decel_distance;
    AINFO << "total_length = " << total_length;
    if (comfort_decel_distance > total_length) {
      GetTotalTimeForNoComfortStop(total_length, adc_speed, target_v,
                                   &begin_break_s, &ref_s_list);
    } else {
      if (adc_speed > target_v) {
        // comfort decel to target speed ,and uniform speed ,and comfort stop.
        GetTotalTimeForUniformSpeedComfortStop(
            total_length, adc_speed, target_v, comfort_decel,
            comfort_decel_distance, &begin_break_s, &ref_s_list);
      } else {
        CHECK_NE(comfort_acc, 0.0);
        CHECK_NE(target_v, 0.0);
        CHECK_NE(comfort_decel, 0.0);
        double t_rampup = (target_v - adc_speed) / comfort_acc;
        double t_rampdown = (target_v - adc_speed) / comfort_decel;
        double s_ramp =
            (adc_speed + target_v) * (t_rampup + t_rampdown) * 0.5;

        double s_rest = total_length - s_ramp - comfort_decel_distance;
        if (s_rest > 0) {
          // comfort acc to target speed ,and uniform speed ,and comfort
          // stop.
          GetTotalTimeForACCAndUniformSpeedAndComfortStop(
              s_rest, t_rampup, total_length, adc_speed, target_v,
              comfort_decel, comfort_acc, comfort_decel_distance,
              &begin_break_s, &ref_s_list);
        } else {
          // acc to someone speed ,and comfort stop.
          GetTotalTimeForACCAndComfortStop(
              s_rest, t_rampup, total_length, adc_speed, target_v,
              comfort_decel, comfort_acc, comfort_decel_distance,
              &begin_break_s, &ref_s_list);
        }
      }
    }

    auto* speed_data = best_ref_info->mutable_speed_data();
    speed_data->clear();
    AINFO << "ref_v_list_ = " << ref_v_list_.size();
    AINFO << "ref_s_list = " << ref_s_list.size();
    for (size_t i = 0; i < ref_v_list_.size(); ++i) {
      speed_data->AppendSpeedPoint(ref_s_list[i].second, kDeltaT * i,
                                   ref_v_list_[i].second, 0.0, 0.0);
    }
  }
}

void OnLanePlanning::GenerateReversePathData(
    const century::common::TrajectoryPoint& start_point,
    planning::ReferenceLineInfo* best_ref_info,
    double* distance_to_stop_point) {
  AINFO << "============get reverse trajectory===============";
  AINFO << "use_reverse_type:" << injector_->use_reverse_type();
  if (FLAGS_enable_reverse_trajectory) {
    std::array<double, 3> end_d = {0.0, 0.0, 0.0};

    auto& reference_line = best_ref_info->reference_line();
    auto init_frenet_state = reference_line.ToFrenetFrame(start_point);
    if (*distance_to_stop_point < kDistanceToBackPoint) {
      end_d = init_frenet_state.second;
    }
    QuinticPolynomialCurve1d path_points = QuinticPolynomialCurve1d(
        end_d, init_frenet_state.second, *distance_to_stop_point);
    AINFO << "distance_to_stop_point = " << *distance_to_stop_point;
    if (*distance_to_stop_point > kLowDistacne &&
        *distance_to_stop_point < kCenterDistacne) {
      *distance_to_stop_point = kCenterDistacne;
    }
    if (*distance_to_stop_point > kCenterLowDistacne &&
        *distance_to_stop_point < kUpDistacne) {
      *distance_to_stop_point = kUpDistacne;
    }
    AINFO << "uinticPolynomialCurve1d OK!!!";
    std::vector<common::FrenetFramePoint> frenet_path_points;
    int i = 0;
    double kPathResolution = 0.1;
    common::math::Vec2d ego_xy = {injector_->vehicle_state()->vehicle_state().x(),
      injector_->vehicle_state()->vehicle_state().y()};
    common::SLPoint ego_sl_point;
    if (!best_ref_info->reference_line().XYToSL(ego_xy, &ego_sl_point)) {
      AERROR << "Failed to convert ego position to SL frame.";
      const auto& adc_sl_boundary = best_ref_info->AdcSlBoundary();
      ego_sl_point.set_s(adc_sl_boundary.start_s() +
                          common::VehicleConfigHelper::GetConfig().vehicle_param().back_edge_to_center());
      ego_sl_point.set_l((adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5);
    }
    // AINFO << "ego sl: " << ego_sl_point.s() << " " << ego_sl_point.l();
    while (i * kPathResolution < *distance_to_stop_point) {
      common::FrenetFramePoint frenet_point;
      frenet_point.set_s(ego_sl_point.s() + i * kPathResolution -
                          *distance_to_stop_point);
      frenet_point.set_l(ego_sl_point.l());
      frenet_point.set_dl(0.0);
      frenet_point.set_ddl(0.0);
      frenet_path_points.emplace_back(frenet_point);
      i += 1;
    }
    std::vector<common::FrenetFramePoint> frenet_path_points_result;
    for (int i = frenet_path_points.size() - 1; i > 0; --i) {
      // AINFO<<"frenet_path_points[i].s = "<<frenet_path_points[i].s();
      frenet_path_points_result.emplace_back(frenet_path_points[i]);
    }
    auto frenet_frame_path =
        FrenetFramePath(std::move(frenet_path_points_result));
    AINFO << "frenet_path_points.size = " << frenet_path_points.size()
          << ",frenet_frame_path.Length = " << frenet_frame_path.Length()
          << ",i = " << i;
    PathData path_data;
    path_data.SetReferenceLine(&reference_line);
    AINFO << "set frenet path";
    path_data.SetFrenetPath(std::move(frenet_frame_path));

    auto* ptr_display_path_reverse = best_ref_info->mutable_debug()
                                         ->mutable_planning_data()
                                         ->add_self_path();
    ptr_display_path_reverse->set_name("PIECEWISE_JERK_PATH_OPTIMIZER");
    ptr_display_path_reverse->mutable_path_point()->CopyFrom(
        {path_data.discretized_path().begin(),
         path_data.discretized_path().end()});
    std::vector<PathData> valid_path_data;
    valid_path_data.emplace_back(path_data);
    *(best_ref_info->mutable_path_data()) = valid_path_data.front();

    best_ref_info->SetCandidatePathData(std::move(valid_path_data));
    AINFO << "discretized_path.size = "
          << best_ref_info->path_data().discretized_path().size()
          << ",best_ref_info->path_data().discretized_path().back().s():"
          << best_ref_info->path_data().discretized_path().back().s()
          << ", best_ref_info->path_data().discretized_path().empty():"
          << best_ref_info->path_data().discretized_path().empty();
  }
}

bool OnLanePlanning::CalReverseObstacleAvoidancePoint(
              planning::ReferenceLineInfo* best_ref_info,
              ADCTrajectory* const ptr_trajectory_pb) {
  if (!FLAGS_enable_backward_for_obs_block) {
    return false;
  }
  if ((injector_->need_back_ward_ || (is_need_backward_for_turn_blocking_&&injector_->borrow_response().response_type() ==
                                            planning::ResponseType::ACCEPT))&& !injector_->use_reverse_trajectory() &&
      is_auto_state_) {

    AINFO << "best_ref_info->IsInNeedToBackward()";
    common::math::Vec2d reverse_stop_xy(injector_->vehicle_state()->x(),
                                        injector_->vehicle_state()->y());
    common::SLPoint reverse_stop_sl;
    auto adc_sl = best_ref_info->AdcSlBoundary();
    double center_l = (adc_sl.end_l() + adc_sl.start_l()) * 0.5;
    reverse_stop_sl.set_l(center_l);
    reverse_stop_sl.set_s((best_ref_info->AdcSlBoundary().start_s() +
                           best_ref_info->AdcSlBoundary().end_s()) *
                              0.5 -
                          injector_->back_ward_distance_ - 2.0);
    AINFO << "reverse_stop_sl.s():" << reverse_stop_sl.s()
          << ",best_ref_info->AdcSlBoundary().start_s():"
          << best_ref_info->AdcSlBoundary().start_s();
    if (best_ref_info->reference_line().SLToXY(reverse_stop_sl,
                                               &reverse_stop_xy)) {
      common::PointENU reverse_stop_point;
      reverse_stop_point.set_x(reverse_stop_xy.x());
      reverse_stop_point.set_y(reverse_stop_xy.y());
      injector_->set_reverse_stop_point(reverse_stop_point);
      injector_->set_use_reverse_trajectory(true);
      injector_->set_use_reverse_type(
          ReverseTrajectoryType::OBSTACLE_AVOIDANCE);
    }
    return true;
  } else if (((!injector_->need_back_ward_ || !is_need_backward_for_turn_blocking_)&&
        !injector_->use_reverse_trajectory()) || !is_auto_state_) {
    AINFO << "no need backward";
    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    injector_->need_back_ward_ = false;
    is_need_backward_for_turn_blocking_ = false;
  }
  std::string block_obs_id = "";
  bool is_close_to_obstacles = false;
  bool is_short_trajectory = false;
  if (nullptr != ptr_trajectory_pb &&
      !ptr_trajectory_pb->trajectory_point().empty()) {
    double trajectory_length =
        ptr_trajectory_pb->trajectory_point()
            .at(ptr_trajectory_pb->trajectory_point().size() - 1)
            .path_point()
            .s() -
        ptr_trajectory_pb->trajectory_point().at(0).path_point().s();
    if (trajectory_length < kMinTrajectoryLength) {
      is_short_trajectory = true;
    }
  }
  bool is_stop_reason_obs = false;
  double min_dis = std::numeric_limits<double>::max();
  if (ptr_trajectory_pb->decision().main_decision().stop().reason_code() ==
      StopReasonCode::STOP_REASON_OBSTACLE) {
    is_stop_reason_obs = true;
    AINFO << "STOP_REASON_OBSTACLE";
    const auto& object_decision =
                            ptr_trajectory_pb->decision().object_decision();
    for (auto decision : object_decision.decision()) {
      for (const auto& object_decision_type : decision.object_decision()) {
        if (!object_decision_type.has_stop() ||
            object_decision_type.stop().reason_code() !=
                StopReasonCode::STOP_REASON_OBSTACLE) {
          continue;
        }
        auto* path_decision = best_ref_info->path_decision();
        block_obs_id = std::to_string(decision.perception_id());
        Obstacle* ptr_obstacle =
                        path_decision->Find(block_obs_id);
        if (nullptr == ptr_obstacle) {
          continue;
        }
        const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
        const auto& adc_sl = best_ref_info->AdcSlBoundary();
        bool no_longitudinal_overlap =
                          adc_sl.end_s() + kCollisionLonBuffer < obs_sl.start_s() ||
                          adc_sl.start_s() - kCollisionLonBuffer > obs_sl.end_s();
        if (!no_longitudinal_overlap) {
          AINFO << "longitudinal_overlap, no use reverse obstacle avoidance";
          break;
        }
        double dis = obs_sl.start_s() - adc_sl.end_s() < kCollisionLonBuffer ?
                        0.0 : (obs_sl.start_s() - adc_sl.end_s());
        if (dis < min_dis) {
          min_dis = dis;
        }
      }
    }
    //
    if (min_dis < FLAGS_min_brorrow_length && min_dis > kCollisionLonBuffer) {
      is_close_to_obstacles = true;
    }
  }
  if (is_short_trajectory && is_stop_reason_obs &&
      !injector_->use_reverse_trajectory() &&
      injector_->vehicle_state()->linear_velocity() < kMinStopSpeed &&
      is_close_to_obstacles &&
      injector_->borrow_response().has_response() &&
      injector_->borrow_response().response_type() ==
          planning::ResponseType::ACCEPT) {
    auto* mutable_laneborrow = injector_->planning_context()
                                        ->mutable_planning_status()
                                        ->mutable_lane_borrow();
    if (mutable_laneborrow->lane_borrow_status() ==
                                          LaneborrowStatus::PREPARE) {
      AINFO << "---- use_reverse_obstacle_avoidance ---- ";
      //       << mutable_laneborrow->lane_borrow_status();
      common::math::Vec2d reverse_stop_xy(0.0, 0.0);
      common::SLPoint reverse_stop_sl;
      reverse_stop_sl.set_l(0.0);
      reverse_stop_sl.set_s(best_ref_info->AdcSlBoundary().end_s() -
                            (best_ref_info->AdcSlBoundary().end_s() -
                            best_ref_info->AdcSlBoundary().start_s()) * 0.5 -
                            (FLAGS_min_brorrow_length - min_dis));
      AINFO << "reverse_stop_sl.s():" << reverse_stop_sl.s()
                << ",best_ref_info->AdcSlBoundary().start_s():"
                << best_ref_info->AdcSlBoundary().start_s();
      if (best_ref_info->reference_line().SLToXY(reverse_stop_sl,
                                                  &reverse_stop_xy)) {
        common::PointENU reverse_stop_point;
        reverse_stop_point.set_x(reverse_stop_xy.x());
        reverse_stop_point.set_y(reverse_stop_xy.y());
        injector_->set_reverse_stop_point(reverse_stop_point);
        injector_->set_use_reverse_trajectory(true);
        injector_
          ->set_use_reverse_type(ReverseTrajectoryType::OBSTACLE_AVOIDANCE);
        return true;
      }
    }
  }
  return false;
}

bool OnLanePlanning::CalTaskStartReverseObstacleAvoidancePoint(
            planning::ReferenceLineInfo* best_ref_info) {
  if (FLAGS_enable_backward_for_task_start_blocking) {
    // AINFO << "FLAGS_enable_backward_for_task_start_blocking";
    if (!injector_->use_reverse_trajectory() &&
          IsTaskPointBlockingScenario(best_ref_info)) {
      AINFO << "-------------use_task_point_reverse_obstacle_avoidance---------- ";
      common::math::Vec2d reverse_stop_xy(0.0, 0.0);
      common::SLPoint reverse_stop_sl;
      reverse_stop_sl.set_l(0.0);
      reverse_stop_sl.set_s(best_ref_info->AdcSlBoundary().end_s() - 
                            (best_ref_info->AdcSlBoundary().end_s() - 
                            best_ref_info->AdcSlBoundary().start_s()) * 0.5 - 
                            FLAGS_task_point_reverse_length);

      AINFO << "reverse_stop_sl.s():" << reverse_stop_sl.s() 
                << ",best_ref_info->AdcSlBoundary().start_s():"
                << best_ref_info->AdcSlBoundary().start_s();
      if (best_ref_info->reference_line().SLToXY(reverse_stop_sl,
                                                &reverse_stop_xy)) {
        common::PointENU reverse_stop_point;
        reverse_stop_point.set_x(reverse_stop_xy.x());
        reverse_stop_point.set_y(reverse_stop_xy.y());
        injector_->set_reverse_stop_point(reverse_stop_point);
        // injector_->use_task_point_reverse_obstacle_avoidance(true);
        injector_->set_use_reverse_trajectory(true);
        injector_->set_use_reverse_type(ReverseTrajectoryType::TASK_POINT);
        return true;
      }
    }
  }
  return false;
}

bool OnLanePlanning::CalTopBullReversePoint(ADCTrajectory* const ptr_trajectory_pb) {
  auto planning_status = injector_->planning_context()->mutable_planning_status();
  if (nullptr == planning_status) {
    return false;
  }
  if (!planning_status->has_top_bull()) {
    is_first_enter_top_bull_ = false;
    return false;
  }

  const auto &top_bull = planning_status->top_bull();
    if (!top_bull.is_in_top_bull()){
    is_first_enter_top_bull_ = false;
  }
  if (!top_bull.is_in_top_bull() ||
      top_bull.action_type() != planning::TopBullStatus::REVERSE) {
    return false;
  }
if(top_bull.ego_complete_action()){
      const std::string msg = "top bull ireverse is completed now stop waiting.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(vehicle_gear_ ==
                                                  Chassis::GEAR_REVERSE);
    FillPlanningPb(start_timestamp_, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);

    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    return false;
  }
  double reverse_distance = top_bull.reverse_distance();
  if (reverse_distance <= 0.0) {
    return false;
  }
  planning::ReferenceLineInfo *best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto &reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() && reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == best_ref_info) {
    return false;
  }

  const auto &adc_sl = best_ref_info->AdcSlBoundary();
  double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  common::SLPoint reverse_stop_sl;
  reverse_stop_sl.set_s(center_s - reverse_distance);
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  reverse_stop_sl.set_l(center_l);

  common::math::Vec2d reverse_stop_xy;
  const auto &reference_line = best_ref_info->reference_line();
  if (!reference_line.SLToXY(reverse_stop_sl, &reverse_stop_xy)) {
    return false;
  }

  if(!injector_->use_reverse_trajectory()&&!is_first_enter_top_bull_){
  common::PointENU reverse_stop_point;
  reverse_stop_point.set_x(reverse_stop_xy.x());
  reverse_stop_point.set_y(reverse_stop_xy.y());
  injector_->set_reverse_stop_point(reverse_stop_point);
  injector_->set_use_reverse_trajectory(true);
  is_first_enter_top_bull_ = true;
  }

  double adc_x = injector_->vehicle_state()->pose().position().x();
  double adc_y = injector_->vehicle_state()->pose().position().y();
  double distance_to_stop_point = std::sqrt(
      (adc_x - injector_->reverse_stop_point().x()) * (adc_x - injector_->reverse_stop_point().x()) +
      (adc_y - injector_->reverse_stop_point().y()) * (adc_y - injector_->reverse_stop_point().y()));

  if (distance_to_stop_point < kTopBullArriveDistance &&
      std::fabs(injector_->vehicle_state()->linear_velocity()) < kTopBullArriveSpeed) {
    planning_status->mutable_top_bull()->set_ego_complete_action(true);
    const std::string msg = "top bull ireverse is completed now stop waiting.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    ptr_trajectory_pb->set_gear(vehicle_gear_);
    ptr_trajectory_pb->set_is_backward_trajectory(Chassis::GEAR_REVERSE ==
                                                  vehicle_gear_);
    FillPlanningPb(start_timestamp_, ptr_trajectory_pb);
    GenerateSlowBreakingTrajectory(ptr_trajectory_pb);

    injector_->set_use_reverse_trajectory(false);
    injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    return true;
  }

  common::SLPoint adc_center_sl;
  if (reference_line.XYToSL(common::math::Vec2d(adc_x, adc_y), &adc_center_sl)) {
    if (adc_center_sl.s() < reverse_stop_sl.s()) {
      planning_status->mutable_top_bull()->set_ego_complete_action(true);
      injector_->set_use_reverse_trajectory(false);
      injector_->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
    }
  }
  return true;
}
bool OnLanePlanning::CalRoutingRequestReversePoint() {
  if (FLAGS_enable_backward_for_routing_request) {
    // AINFO << "FLAGS_enable_backward_for_routing_request";
    // get reverse routing request.
    if (injector_->use_reverse_type() == 
            ReverseTrajectoryType::FORWARD_DRIVING ||
        injector_->use_reverse_type() == 
            ReverseTrajectoryType::ROUTING_REQUEST) { // && !FLAGS_enable_backward_for_obs_block) {
      if (local_view_.routing->routing_request().is_reverse_trajectory()) {
        AINFO << "need reverse trajectory for request";
        common::PointENU reverse_stop_point;
        // const auto& routing_pose = best_ref_info->get_routing_end().pose();
        const auto& routing_pose =
          local_view_.routing->routing_request().waypoint().at(0).pose();
        reverse_stop_point.set_x(routing_pose.x());
        reverse_stop_point.set_y(routing_pose.y());
        AINFO << "routing_pose.x() = " << routing_pose.x();
        AINFO << "routing_pose.y() = " << routing_pose.y();
        // test
        // reverse_stop_point.set_x(-3204.48);
        // reverse_stop_point.set_y(2345.22);
        // need check adc exceed end point.
        double adc_heading = injector_->vehicle_state()->heading();
        double adc_x = injector_->vehicle_state()->pose().position().x();
        double adc_y = injector_->vehicle_state()->pose().position().y();
        double end_x = reverse_stop_point.x();
        double end_y = reverse_stop_point.y();
        double dx = adc_x - end_x;
        double dy = adc_y - end_y;
        double heading_end_to_adc = std::atan2(dy, dx);  // calculate the heading
        AINFO << "heading_end_to_adc = " << heading_end_to_adc;
        AINFO << "adc_heading = " << adc_heading;
        double diff_theta_between_adc_and_end_point =
          common::math::NormalizeAngle(heading_end_to_adc - adc_heading);
        AINFO << "diff_theta_between_adc_and_end_point = "
            << diff_theta_between_adc_and_end_point;
        if (std::fabs(diff_theta_between_adc_and_end_point) < M_PI_2) {
          injector_->set_reverse_stop_point(reverse_stop_point);
          injector_->set_use_reverse_trajectory(true);
          injector_
            ->set_use_reverse_type(ReverseTrajectoryType::ROUTING_REQUEST);
          return true;
        } else {
          AINFO << "adc exceed end point,no reverse.";
          injector_->set_use_reverse_trajectory(false);
          injector_
            ->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
        }
      } else {
        injector_->set_use_reverse_trajectory(false);
        injector_
          ->set_use_reverse_type(ReverseTrajectoryType::FORWARD_DRIVING);
      }
    }
  }
  return false;
}

bool OnLanePlanning::IsObstacleFarAway(planning::ReferenceLineInfo* best_ref_info) {
  auto* mutable_laneborrow = injector_->planning_context()
                                        ->mutable_planning_status()
                                        ->mutable_lane_borrow();
  if (mutable_laneborrow->block_obstacle_id().empty() || 
      mutable_laneborrow->lane_borrow_status() == 
                                          LaneborrowStatus::DEFAULT ||
      mutable_laneborrow->lane_borrow_status() == 
                                          LaneborrowStatus::FINISH) {
    AINFO << "mutable_laneborrow->lane_borrow_status():" 
          << mutable_laneborrow->lane_borrow_status()
          << ",mutable_laneborrow->block_obstacle_id().size():"
          <<mutable_laneborrow->block_obstacle_id().size();
    return true;
  }
  auto* path_decision = best_ref_info->path_decision();
  bool result = true;
  for (const auto & obstacle_id : mutable_laneborrow->block_obstacle_id()) {
    // (TODO): all obs
    Obstacle* ptr_obstacle = path_decision->Find(obstacle_id);
    AINFO << "ptr_obstacle = " << obstacle_id;
    if (nullptr == ptr_obstacle) {
      continue;
    }
  
    const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
    const auto& adc_sl = best_ref_info->AdcSlBoundary();
    if (obs_sl.start_s() - adc_sl.end_s() > FLAGS_min_brorrow_length) {
      continue;
    } else {
      result = false;
    }
  }
 
  return result;
}

bool OnLanePlanning::IsTaskPointBlockingScenario(
                      planning::ReferenceLineInfo* const best_ref_info) {
  // type ?
  // near task point
  // AINFO << "waypoint_num = " << local_view_.routing->routing_request().waypoint().size()
  // << ",injector_->vehicle_state()->linear_velocity() = " << injector_->vehicle_state()->linear_velocity()
  // << ",injector_->is_new_routing() = " << injector_->is_new_routing();
  if (injector_->is_new_routing() && 
      injector_->vehicle_state()->linear_velocity() < kMinStopSpeed &&
      local_view_.routing->routing_request().waypoint().size() > 0) {
    const auto& routing_pose =
        local_view_.routing->routing_request().waypoint().at(0).pose();
    const auto& routing_heading =
        local_view_.routing->routing_request().waypoint().at(0).heading();
    double adc_heading = injector_->vehicle_state()->heading();
    double adc_x = injector_->vehicle_state()->pose().position().x();
    double adc_y = injector_->vehicle_state()->pose().position().y();
    double end_x = routing_pose.x();
    double end_y = routing_pose.y();
    double dx = adc_x - end_x;
    double dy = adc_y - end_y;
    double dheading = adc_heading - routing_heading;
    AINFO << "waypoint_num = " 
          << local_view_.routing->routing_request().waypoint().size()
          << ",routing_pose.x() = " << routing_pose.x() 
          << ",routing_pose.y() = " << routing_pose.y() 
          << ",routing_heading = " << routing_heading
          << ",dx = " << dx << ",dy = " << dy << ",dheading = " << dheading;
    if (dx < kMinStopDistance && dy < kMinStopDistance && 
                      std::fabs(dheading) < M_PI_2) {
      auto* path_decision = best_ref_info->path_decision();
      bool is_left_has_strack = false;
      bool is_front_blocking = false;
      for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
        Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
        if (nullptr == ptr_obstacle) {
          continue;
        }
        const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
        const auto& obs_type = ptr_obstacle_item->Perception().type();
        const auto& adc_sl = best_ref_info->AdcSlBoundary();
        if (obs_sl.start_s() > adc_sl.end_s() + FLAGS_min_brorrow_length ||
            obs_sl.end_s() < adc_sl.start_s()) {
          continue;
        }
        if (perception::PerceptionObstacle::STACKER == obs_type ||
            perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
          bool no_longitudinal_overlap =
                                  adc_sl.end_s() < obs_sl.start_s() ||
                                  adc_sl.start_s() > obs_sl.end_s();
          if (!no_longitudinal_overlap &&
                obs_sl.start_l() > adc_sl.end_l()) {
            is_left_has_strack = true;
            continue;
          }
        }
        bool no_lateral_overlap =
            adc_sl.end_l() + FLAGS_static_obstacle_nudge_l_buffer <
                obs_sl.start_l() ||
            adc_sl.start_l() - FLAGS_static_obstacle_nudge_l_buffer >
                obs_sl.end_l();
  
        if (!no_lateral_overlap) {
          is_front_blocking = true;
          continue;
        }
      }
      if (is_left_has_strack && is_front_blocking) {
        AINFO << "is_left_has_strack and is_front_blocking true;";
        return true;
      }
      return false;
    }
  }
  return false;
}

bool OnLanePlanning::IsStackerSafeLonDistance(
              planning::ReferenceLineInfo* const best_ref_info) {
  bool result = false;
  static uint8_t multi_frame_count = 0;
  auto* path_decision = best_ref_info->path_decision();
  // const auto& vehicle_param =
  //     common::VehicleConfigHelper::GetConfig().vehicle_param();
  // double ego_length = vehicle_param.length() + kCollisionLonBuffer;
  // double ego_width = vehicle_param.width() + kCollisionLateralBuffer;
  // double shift_distance =
  //     ego_length * 0.5 - vehicle_param.back_edge_to_center();
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (nullptr == ptr_obstacle) {
      continue;
    }
    const auto& obs_sl = ptr_obstacle->PerceptionSLBoundary();
    const auto& obs_type = ptr_obstacle_item->Perception().type();
    const auto& adc_sl = best_ref_info->AdcSlBoundary();
    if (obs_sl.start_s() - adc_sl.end_s() > FLAGS_min_brorrow_length) {
      continue;
    }
    if (perception::PerceptionObstacle::STACKER == obs_type ||
        perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
      multi_frame_count = 0;
      result = false;
    }
  }
  if (++multi_frame_count > kStackerLonSafeCount) {
    result = true;
    multi_frame_count = kStackerLonSafeCount;
  }
  return result;
}

void OnLanePlanning::CheckNeedToShrinkCollisionBuffer(ADCTrajectory* ptr_trajectory_pb) {
    planning::ReferenceLineInfo* best_ref_info = nullptr;
        double min_cost = std::numeric_limits<double>::infinity();
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == injector_ || nullptr == best_ref_info ||
      nullptr == ptr_trajectory_pb) {
    return;
  }
  AINFO << "injector_->enable_shrink_collision_buffer_ = "
          << injector_->enable_shrink_collision_buffer_;
  // stop reason st boundary
  bool is_stop_reason_boundary = false;
  std::string block_boundary_id = "";
  auto* st_graph_data = best_ref_info->mutable_st_graph_data();
  double min_s = std::numeric_limits<double>::max();

  // get nearst block obs
  PathDecision* const path_decision = best_ref_info->path_decision();

  for (const STBoundary* boundary : st_graph_data->st_boundaries()) {
    // static obs
    if (std::fabs(boundary->obs_v()) > kStopSpeed) {
      // AINFO << "boundary->obs_v() = " << boundary->obs_v();
      continue;
    }
    // must cross all time.
    if (boundary->bottom_right_point().t() < kMaxPredictionTime) {
      continue;
    }    auto* mutable_obstacle = path_decision->Find(boundary->id());
    if(mutable_obstacle->IsVirtual()){
      AINFO<<"virtual obstacle = "<<boundary->id();
      continue;
    }
    double temp_min_s = boundary->bottom_left_point().s();
    if (temp_min_s < min_s) {
      min_s = temp_min_s;
      block_boundary_id = boundary->id();
    }
  }

  // is stop trajectory
  if (min_s < kStopLength ||
      ptr_trajectory_pb->total_path_length() < kStopLength) {
    is_stop_reason_boundary = true;
  }


  AINFO<<"ptr_trajectory_pb->total_path_length() = "<<ptr_trajectory_pb->total_path_length();
  AINFO << "is_stop_reason_boundary = " << is_stop_reason_boundary;
  AINFO << "block_boundary_id = " << block_boundary_id;
  if (is_stop_reason_boundary) {
    // id change
    if (block_boundary_id == injector_->shrink_collision_id_) {
      injector_->adc_block_count_++;
      injector_->adc_block_count_ = std::min(MaxCount, injector_->adc_block_count_);
    } else {
      injector_->shrink_collision_id_ = block_boundary_id;
      injector_->adc_block_count_--;
      injector_->adc_block_count_ = std::max(injector_->adc_block_count_, 0);
    }
    disappear_boundary_obs_count_ = 0;
  } else if (!is_stop_reason_boundary) {
    // now no have cause stop obs
    // shrink buffer obs pass 2m or change id ,need recheck
    PathDecision* const path_decision = best_ref_info->path_decision();
    auto* mutable_obstacle =
        path_decision->Find(injector_->shrink_collision_id_);
    if (nullptr == mutable_obstacle) {
      injector_->adc_block_count_ = 0;
      disappear_boundary_obs_count_++;
      disappear_boundary_obs_count_ =
          std::min(disappear_boundary_obs_count_, DisappearBoundaryObsCount);
    } else {
      auto block_obs_sl = mutable_obstacle->PerceptionSLBoundary();
      auto adc_sl = best_ref_info->AdcSlBoundary();
      if (adc_sl.start_s() > block_obs_sl.end_s() + kPassObsDistance) {
        injector_->adc_block_count_ = 0;
        disappear_boundary_obs_count_++;
        disappear_boundary_obs_count_ =
            std::min(disappear_boundary_obs_count_, DisappearBoundaryObsCount);
      }
    }

    // obs 
    if (disappear_boundary_obs_count_ == DisappearBoundaryObsCount) {
      injector_->enable_shrink_collision_buffer_ = false;
      // injector_->adc_stop_count_ = 0;
      AINFO << "no block obs";
    }
  }
  AINFO<<"disappear_boundary_obs_count_ = "<<disappear_boundary_obs_count_;
  AINFO<<"injector_->adc_stop_count_ = "<<injector_->adc_stop_count_;
  AINFO<<"injector_->adc_block_count_ = "<<injector_->adc_block_count_;
  bool is_stop_long_time = injector_->adc_stop_count_ > MaxCount;
  bool is_adc_block_count_large = injector_->adc_block_count_ > AdcBlockCount;
   const auto &path_point  = best_ref_info->path_data().GetPathPointWithPathS(0.0);
  double diff_heading = std::fabs(common::math::NormalizeAngle(
      injector_->vehicle_state()->heading() - path_point.theta()));
  AINFO<<"diff_heading = "<<diff_heading;
  bool is_car_no_swing = best_ref_info->NeedDiagonal()|| diff_heading<0.01;
  AINFO<<"is_car_no_swing = "<<is_car_no_swing;
  AINFO<<"is_stop_long_time = "<<is_stop_long_time;
  AINFO<<"is_adc_block_count_large = "<<is_adc_block_count_large;
  AINFO<<"best_ref_info->IsPathStraight() = "<<best_ref_info->IsPathStraight();
  if (!injector_->enable_shrink_collision_buffer_) {
    if (is_stop_long_time && is_adc_block_count_large && is_car_no_swing &&
        best_ref_info->IsPathStraight()) {
      injector_->adc_stable_stop_count_++;
      injector_->adc_stable_stop_count_ =
          std::min(injector_->adc_stable_stop_count_, MaxCount);
    } else {
      injector_->adc_stable_stop_count_ = 0;
    }
    AINFO << "injector_->adc_stable_stop_count_ = "
          << injector_->adc_stable_stop_count_;
    if (injector_->adc_stable_stop_count_ > AdcStableStopCount) {
      AINFO << "ENTER SHRINK COLLISION";
      injector_->enable_shrink_collision_buffer_ = true;
    }
    AINFO << "injector_->enable_shrink_collision_buffer_ = "
          << injector_->enable_shrink_collision_buffer_;
  } else {
    if (!is_car_no_swing || !best_ref_info->IsPathStraight()) {
      injector_->enable_shrink_collision_buffer_ = false;
    }
  }
}

void OnLanePlanning::CheckUseReverseTrajectory(
    const century::common::TrajectoryPoint& start_point,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  planning::ReferenceLineInfo* best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == injector_ || nullptr == best_ref_info) {
    return;
  }
  double distance_to_stop_point = GetReverseTrajectoryDistance(
      start_point, best_ref_info, ptr_trajectory_pb);
  if (!injector_->use_reverse_trajectory()) {
    return;
  }
  if (injector_->use_reverse_trajectory() &&
      injector_->use_reverse_type() == ReverseTrajectoryType::TASK_POINT) {
    if (ptr_trajectory_pb->borrow_request()) {
      ptr_trajectory_pb->set_borrow_request(false);
      ptr_trajectory_pb->set_block_obs_id("");
      last_publishable_trajectory_->SetBorrowRequest(
          ptr_trajectory_pb->borrow_request());
    }
  }
  if (injector_->use_reverse_type() ==
      ReverseTrajectoryType::TASK_POINT_REVERSE_END) {
    GenerateStopTrajectory(ptr_trajectory_pb);
    AINFO << "============stop set_borrow_request===============";
    if (!injector_->borrow_response().has_response()) {
      ptr_trajectory_pb->set_borrow_request(true);
      ptr_trajectory_pb->set_block_obs_id("0");
      last_publishable_trajectory_->SetBorrowRequest(
          ptr_trajectory_pb->borrow_request());
    }
    return;
  }
  if (injector_->use_reverse_trajectory()) {
    GenerateReversePathData(start_point, best_ref_info, &distance_to_stop_point);
    GetReverseSpeedData(best_ref_info, ptr_trajectory_pb, distance_to_stop_point);
  }
  DiscretizedTrajectory trajectory;
  if (!best_ref_info->CombinePathAndSpeedProfile(
          0.0, start_point.path_point().s(), &trajectory)) {
    AINFO << "COMBINE TRAJECTORY FAIL";
    return;
  }
  best_ref_info->SetTrajectory(trajectory);
  std::lock_guard<std::mutex> frame_lock(frame_mutex_);
  {
    last_publishable_trajectory_.reset(new PublishableTrajectory(
        start_timestamp_, best_ref_info->trajectory()));

    ADEBUG << "current_time_stamp: " << start_time_;
    if (!FLAGS_enable_reverse_trajectory) {
      last_publishable_trajectory_->PrependTrajectoryPoints(
          std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                       stitching_trajectory.end() - 1));
    }
    // Update the trajectory of the previous frame first and fill in the
    // trajectory of the current frame.
    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  }

  // int num_of_points = 0;
  for (auto& p : *(ptr_trajectory_pb->mutable_trajectory_point())) {
    // AINFO << "p.relative_time() = " << p.relative_time();
    p.set_v(-p.v());
    auto* p_path = p.mutable_path_point();
    p_path->set_s(-p.path_point().s());
  }
  if (ptr_trajectory_pb->is_backward_trajectory()) {
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
  } else {
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_REVERSE);
  }
}

double OnLanePlanning::GetTotalTimeForNoComfortStop(
    double total_length, double adc_speed, double target_v,
    double* begin_break_s, std::vector<std::pair<double, double>>* ref_s_list) {
  double need_decel = (adc_speed * adc_speed) / (total_length * 2.0);
  double total_time = adc_speed / need_decel;
  *begin_break_s = total_length;
  double delta_t = kOptimizedDeltaT;
  AINFO << "GetTotalTimeForNoComfortStop";
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    ref_s.second =
        (adc_speed * adc_speed - (adc_speed - need_decel * ref_s.first) *
                                     (adc_speed - need_decel * ref_s.first)) /
        (2 * need_decel);
    ref_v.second = adc_speed - need_decel * ref_s.first;
    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double OnLanePlanning::GetTotalTimeForUniformSpeedComfortStop(
    double total_length, double adc_speed, double target_v,
    double comfort_decel, double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  CHECK_NE(target_v, 0.0);
  CHECK_NE(comfort_decel, 0.0);
  AINFO << "GetTotalTimeForUniformSpeedComfortStop";
  double t_cruise = (total_length - comfort_decel_distance) / target_v;
  double t_rampdown = (adc_speed - target_v) / comfort_decel;
  double t_dec = target_v / comfort_decel;
  double total_time = t_cruise + t_rampdown + t_dec;
  *begin_break_s = total_length - target_v * target_v / (2 * comfort_decel);
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  double start_break_t = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_rampdown) {
      ref_s.first = i * delta_t;
      ref_s.second = (adc_speed * adc_speed -
                      (adc_speed - comfort_decel * ref_s.first) *
                          (adc_speed - comfort_decel * ref_s.first)) /
                     (2 * comfort_decel);
      current_s = ref_s.second;
      ref_v.second = adc_speed - comfort_decel * ref_s.first;
      start_break_t = ref_s.first;
    } else if (ref_s.first > t_rampdown &&
               ref_s.first <= t_cruise + t_rampdown) {
      ref_s.first = i * delta_t;
      current_s = current_s + delta_t * target_v;
      ref_s.second = current_s;
      start_break_t = ref_s.first;
      ref_v.second = target_v;
    } else {
      ref_s.first = i * delta_t;
      double diff_t = ref_s.first - start_break_t;
      ref_s.second =
          current_s + target_v * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = target_v - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double OnLanePlanning::GetTotalTimeForACCAndUniformSpeedAndComfortStop(
    double s_rest, double t_rampup, double total_length, double adc_speed,
    double target_v, double comfort_decel, double comfort_acc,
    double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  AINFO << "GetTotalTimeForACCAndUniformSpeedAndComfortStop";
  double t_cruise = s_rest / target_v;
  double t_dec = target_v / comfort_decel;
  *begin_break_s = total_length - target_v * target_v / (2 * comfort_decel);
  double total_time = t_rampup + t_cruise + t_dec;
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  double start_break_t = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_rampup) {
      ref_s.first = i * delta_t;
      ref_s.second = ((adc_speed + comfort_acc * ref_s.first) *
                          (adc_speed + comfort_acc * ref_s.first) -
                      adc_speed * adc_speed) /
                     (2 * comfort_acc);
      current_s = ref_s.second;
      ref_v.second = adc_speed + comfort_acc * ref_v.first;
      start_break_t = ref_s.first;
    } else if (ref_s.first > t_rampup && ref_s.first <= t_cruise + t_rampup) {
      ref_s.first = i * delta_t;
      current_s = current_s + delta_t * target_v;
      ref_s.second = current_s;
      start_break_t = ref_s.first;
      ref_v.second = target_v;
    } else {
      ref_s.first = i * delta_t;
      double diff_t = ref_s.first - start_break_t;
      ref_s.second =
          current_s + target_v * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = target_v - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

double OnLanePlanning::GetTotalTimeForACCAndComfortStop(
    double s_rest, double t_rampup, double total_length, double adc_speed,
    double target_v, double comfort_decel, double comfort_acc,
    double comfort_decel_distance, double* begin_break_s,
    std::vector<std::pair<double, double>>* ref_s_list) {
  AINFO << "GetTotalTimeForACCAndComfortStop";
  double s_rampup_rampdown = total_length - comfort_decel_distance;
  double acc_use_s = s_rampup_rampdown * 0.5;
  double v_max =
      std::sqrt(adc_speed * adc_speed + 2.0 * comfort_acc * acc_use_s);
  // AINFO << "v_max = " << v_max;
  // AINFO << "adc_speed = " << adc_speed;
  double t_acc = (v_max - adc_speed) / comfort_acc;
  double t_dec = v_max / comfort_decel;
  // AINFO << "t_acc = " << t_acc;
  // AINFO << "t_dec = " << t_dec;
  *begin_break_s = total_length - v_max * v_max / (2 * comfort_decel);

  double total_time = t_acc + t_dec;
  // AINFO << "total_time = " << total_time;
  double delta_t = kOptimizedDeltaT;
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  double current_s = 0.0;
  for (int i = 0; i < num_of_knots; ++i) {
    std::pair<double, double> ref_s;
    std::pair<double, double> ref_v;
    ref_s.first = i * delta_t;
    ref_v.first = i * delta_t;
    if (ref_s.first <= t_acc) {
      ref_s.second = ((adc_speed + comfort_acc * ref_s.first) *
                          (adc_speed + comfort_acc * ref_s.first) -
                      adc_speed * adc_speed) /
                     (2 * comfort_acc);
      current_s = ref_s.second;
      ref_v.second = adc_speed + comfort_acc * ref_v.first;
    } else {
      double diff_t = ref_s.first - t_acc;
      ref_s.second =
          current_s + v_max * diff_t - 0.5 * comfort_decel * diff_t * diff_t;
      ref_v.second = v_max - comfort_decel * diff_t;
    }

    ref_s_list->emplace_back(ref_s);
    ref_v_list_.emplace_back(ref_v);
  }
  // TODO(zongxingguo): else also add last point..
  if (((num_of_knots - 1) * delta_t) < total_time &&
      FLAGS_enable_add_last_point_for_teb) {
    ref_s_list->emplace_back(total_time, total_length);
    ref_v_list_.emplace_back(total_time, 0.0);
  }
  return total_time;
}

// check and set whether the route destination is reached
bool OnLanePlanning::JudgeReachTargetPoint(
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
  } else if (!FLAGS_enable_use_pullover_mode ||
             (FLAGS_enable_use_pullover_mode && !injector_->pullover_using_ &&
              (!FLAGS_enable_TEB_thread || !injector_->is_in_play_street))) {
    // AINFO << "JudgeFinished: PRP";
    CheckPublicRoadReachTargetPoint(ptr_trajectory_pb);
    CheckReroutingInSpecificScenarios(ptr_trajectory_pb);
  }

  // set reached station if need arrived_station_immediately (Both TEB and PRP
  // need to be executed)
  if (destination->arrived_station_immediately()) {
    ptr_trajectory_pb->set_has_reached_destination(true);
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_destination(true);
    destination->set_has_reached_station(true);
    AERROR << "arrived_station_immediately: GenerateStopTrajectory.";
    GenerateStopTrajectory(ptr_trajectory_pb);
  }
  
  ClearRoutingHeader();

  return destination->has_reached_station();
}

bool OnLanePlanning::JudgeArrivedStationImmediately(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  destination->set_has_reached_destination(false);
  destination->set_has_reached_station(false);

  // set reached station if need arrived_station_immediately
  if (destination->arrived_station_immediately()) {
    ptr_trajectory_pb->set_has_reached_destination(true);
    ptr_trajectory_pb->set_has_reached_station(true);
    destination->set_has_reached_destination(true);
    destination->set_has_reached_station(true);
    AERROR << "arrived_station_immediately: GenerateStopTrajectory.";
  }
  ClearRoutingHeader();

  return destination->has_reached_station();
}

void OnLanePlanning::ClearRoutingHeader() {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();

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
    local_view_.routing->clear_header();
    has_reached_station_ = false;
  }
}

void OnLanePlanning::CheckOpenSpaceReachTargetPoint(
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
    AERROR << "OpenSpace_Finished!";
  }
  return;
}

void OnLanePlanning::CheckReroutingInSpecificScenarios(
    ADCTrajectory* const ptr_trajectory_pb) {
  size_t waypoint_num = local_view_.routing->routing_request().waypoint_size();
  if (waypoint_num > 1) {
    if (injector_->planning_context() != nullptr &&
        injector_->GetReroutinForHuman() &&
        !local_view_.routing->routing_request().rerouting_info().huaman_shaped().is_part_rerouting()) {
      AINFO << "ReroutinForDJZHuman.";
      if (!frame_->Rerouting(injector_->planning_context(),
                             ReroutingType::PART_HUMAN_SHAPED)) {
        AERROR << "Failed to send rerouting request";
      }
    }
    if (FLAGS_enable_rerouting_for_block &&
        injector_->planning_context() != nullptr &&
        local_view_.routing->routing_request()
                .rerouting_info()
                .block_rerouting()
                .rerouting_type() != routing::RE_ROUTING_D7 &&
        injector_->is_need_rerouting_for_block_) {
      AINFO << "ReroutinForNeedReroutingForBlock";
      injector_->is_need_rerouting_for_block_ = false;
      injector_->path_block_count_ = 0;
      if (!frame_->Rerouting(injector_->planning_context(),
                             ReroutingType::D7_BLOCK)) {
        AERROR << "Failed to send rerouting request";
      }
    }
    if (injector_->planning_context() != nullptr &&
        local_view_.routing->routing_request().rerouting_info().huaman_shaped().is_part_rerouting()) {
      // 1. check distance
      bool near_target_distance = false;
      common::PointENU target_point;
      size_t waypoint_num =
          local_view_.routing->routing_request().waypoint_size();
      const common::VehicleState& car_position = frame_->vehicle_state();
      target_point = frame_->local_view()
                         .routing->routing_request()
                         .waypoint()
                         .at(waypoint_num - 1)
                         .pose();
      double distance_to_vehicle =
          std::hypot(car_position.x() - target_point.x(),
                     car_position.y() - target_point.y());
      near_target_distance = FLAGS_enable_use_pullover_mode
                                 ? injector_->is_reach_goal_
                                 : (distance_to_vehicle <
                                    FLAGS_threshold_distance_for_destination);
      AINFO << "near_target_distance: " << near_target_distance;

      // 2. check angle
      bool near_target_angle = false;
      if (!local_view_.routing->routing_request()
               .parking_info()
               .parking_space_id()
               .empty()) {
        near_target_angle = true;
      } else if (frame_->DriveReferenceLineInfo() && waypoint_num > 0) {
        const ReferenceLine& reference_line =
            frame_->DriveReferenceLineInfo()->reference_line();
        ReferencePoint point = reference_line.GetReferencePoint(
            target_point.x(), target_point.y());
        const auto& target_heading = point.heading();
        const common::VehicleState& car_position = frame_->vehicle_state();
        const auto& adc_heading = car_position.heading();
        near_target_angle = std::fabs(century::common::math::AngleDiff(
                                target_heading, adc_heading)) < 0.5 * M_PI_4;

        if (near_target_distance && near_target_angle && is_stopped_) {
          AINFO << "ReroutinForDJZHuman--two routing.";
          if (!frame_->Rerouting(injector_->planning_context(),
                                 ReroutingType::SECOND_REROUTING)) {
            AERROR << "Failed to send rerouting request";
          }
        }
      }
    }
  }
}

void OnLanePlanning::CheckPublicRoadReachTargetPoint(
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  // 1. check distance
  bool near_target_distance = false;
  bool near_routing_point_distance = false;
  bool near_point_distance = false;
  common::PointENU target_point;
  size_t waypoint_num = local_view_.routing->routing_request().waypoint_size();
  if (waypoint_num > 0) {
    const common::VehicleState& car_position = frame_->vehicle_state();
    target_point = frame_->local_view()
                       .routing->routing_request()
                       .waypoint()
                       .at(waypoint_num - 1)
                       .pose();
    double distance_to_waypoint =
        std::hypot(car_position.x() - target_point.x(),
                   car_position.y() - target_point.y());
    if (local_view_.routing->routing_request().is_reverse_trajectory()) {
      target_point = frame_->local_view()
                         .routing->routing_request()
                         .waypoint()
                         .at(0)
                         .pose();
    }
    if (!local_view_.routing->routing_request()
             .parking_info()
             .parking_space_id()
             .empty()) {
      target_point =
          local_view_.routing->routing_request().parking_info().parking_point();
    }
    // AINFO << "target_point = " << target_point.x() << "    "
    //       << target_point.y();
    // use projection to cal distance.
    double distance_to_vehicle =
        std::hypot(car_position.x() - target_point.x(),
                   car_position.y() - target_point.y());
    near_routing_point_distance =
        (distance_to_waypoint < kReroutingMaxDistance);
    near_point_distance = (distance_to_waypoint < 25 * kReroutingMinDistance);
    // AINFO << "car_position = " << car_position.x() << "        "
    //       << car_position.y();
    if (frame_->DriveReferenceLineInfo()) {
      auto& reference_line = frame_->DriveReferenceLineInfo()->reference_line();
      const auto& reference_line_path = reference_line.GetMapPath();
      double left_width = reference_line_path.GetLaneLeftWidth(
          frame_->DriveReferenceLineInfo()->AdcSlBoundary().end_s());
      // AINFO<<"left_width = "<<left_width;
      if (distance_to_vehicle < left_width) {
        ReferencePoint point = reference_line.GetReferencePoint(
            target_point.x(), target_point.y());
        distance_to_vehicle = std::hypot(car_position.x() - point.x(),
                                         car_position.y() - point.y());
        ReferencePoint point_car = reference_line.GetReferencePoint(
            car_position.x(), car_position.y());

        distance_to_vehicle =
            std::hypot(point_car.x() - point.x(), point_car.y() - point.y());
        // AINFO << "point = " << point.x() << "    " << point.y();
        // AINFO << "point_car = " << point_car.x() << "    " << point_car.y();
      }
    }

    near_target_distance =
        FLAGS_enable_use_pullover_mode
            ? injector_->is_reach_goal_
            : (distance_to_vehicle < FLAGS_threshold_distance_for_destination);
    if (local_view_.routing->routing_request().task_type() ==
            routing::TINY_ADJUSTMENT_BACK ||
        local_view_.routing->routing_request().task_type() ==
            routing::TINY_ADJUSTMENT_FRONT ||
        (routing::TINY_ADJUSTMENT_RIGHT ==
         local_view_.routing->routing_request().task_type()) ||
        (routing::TINY_ADJUSTMENT_LEFT ==
         local_view_.routing->routing_request().task_type())) {
      near_target_distance = (distance_to_vehicle < kDistanceForDestination);
    }
    // AINFO << "distance_to_vehicle: " << distance_to_vehicle;
    // AINFO << "FLAGS_threshold_distance_for_destination: "
    //       << FLAGS_threshold_distance_for_destination;
    AINFO << "near_target_distance: " << near_target_distance;
  }

  // 2. check angle
  bool near_target_angle = false;
  if (!local_view_.routing->routing_request()
           .parking_info()
           .parking_space_id()
           .empty()) {
    near_target_angle = true;
  } else if (frame_->DriveReferenceLineInfo() && waypoint_num > 0) {
    const ReferenceLine& reference_line =
        frame_->DriveReferenceLineInfo()->reference_line();
    ReferencePoint point =
        reference_line.GetReferencePoint(target_point.x(), target_point.y());
    const auto& target_heading = point.heading();
    const common::VehicleState& car_position = frame_->vehicle_state();
    const auto& adc_heading = car_position.heading();
    near_target_angle = std::fabs(century::common::math::AngleDiff(
                            target_heading, adc_heading)) < 0.5 * M_PI_4;
    // AINFO << "target_heading: " << target_heading;
    // AINFO << "adc_heading: " << adc_heading;
    // AINFO << "near_target_angle: " << near_target_angle;
  }
  bool huaman_shaped_driver = CheckHumanShapedDriverRerouting(
      ptr_trajectory_pb, near_routing_point_distance, near_point_distance);
  CheckDestinationReachedRerouting(ptr_trajectory_pb, near_target_distance,
                                   near_target_angle, huaman_shaped_driver);
  CheckLoopRunningRerouting(ptr_trajectory_pb, near_routing_point_distance);
  bool reached_destination = destination->has_reached_destination();
  ptr_trajectory_pb->set_has_reached_destination(reached_destination);
  ptr_trajectory_pb->set_has_reached_station(reached_destination);
  destination->set_has_reached_station(reached_destination);
}

bool OnLanePlanning::CheckHumanShapedDriverRerouting(
    ADCTrajectory* const ptr_trajectory_pb,
    bool near_routing_point_distance,
    bool near_point_distance) {
  bool huaman_shaped_driver = false;
  if (!local_view_.routing->routing_request().rerouting_info().is_rerouting() ||
      local_view_.routing->routing_request().rerouting_info().huaman_shaped().level() >=
          kMaxHuamanShapedLevel) {
    cnt_obs_human_shaped_ = 0;
    return huaman_shaped_driver;
  }

  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (nullptr == injector_ || nullptr == best_ref_info) {
    cnt_obs_human_shaped_ = 0;
    return huaman_shaped_driver;
  }

  common::SLPoint routing_end_sl;
  bool is_endpoint_sl_usable = false;
  const auto routing_end = best_ref_info->get_routing_end();
  if (best_ref_info->reference_line().XYToSL(routing_end.pose(),
                                             &routing_end_sl)) {
    is_endpoint_sl_usable = true;
  }
  auto adc_sl_boundary = best_ref_info->AdcSlBoundary();
  double adc_center_s =
      (adc_sl_boundary.end_s() + adc_sl_boundary.start_s()) * 0.5;
  AINFO << "adc_center_s:" << adc_center_s
        << ",routing_end_sl:" << routing_end_sl.s()
        << ",is_endpoint_sl_usable:" << is_endpoint_sl_usable
        << ",near_point_distance:" << near_point_distance;
  bool is_end_steps_in_advance = false;
  bool is_end_all_steps_in_advance = false;
  bool is_huaman_shaped_level_0 =
      local_view_.routing->routing_request().rerouting_info().huaman_shaped().level() == 0;
  bool is_huaman_shaped_level_1 =
      local_view_.routing->routing_request().rerouting_info().huaman_shaped().level() == 1;
  if (is_huaman_shaped_level_0 || is_huaman_shaped_level_1) {
    auto reason_code =
        ptr_trajectory_pb->decision().main_decision().stop().reason_code();
    bool is_blocking_reason =
        reason_code == StopReasonCode::STOP_REASON_OBSTACLE ||
        reason_code == StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY;
    if (is_blocking_reason && near_routing_point_distance &&
        is_huaman_shaped_level_0 &&
        local_view_.routing->routing_request().local_routing_type() !=
            routing::ROUTING_J4_TO_T4T5_EAST &&
        routing_end_sl.s() > adc_sl_boundary.end_s() &&
        std::fabs(routing_end_sl.s() - adc_sl_boundary.end_s()) <
            kHumanReroutingMaxDistance) {
      cnt_block_human_shaped_ = 0;
      if (is_stopped_ && cnt_obs_human_shaped_++ > kObsBlockageCnt) {
        is_end_steps_in_advance = true;
        AINFO << "obstacles block the endpoint, end "
                 "(huaman_shaped_level==0) early";
      }
    } else {
      cnt_obs_human_shaped_ = 0;
    }
    if (is_blocking_reason && near_routing_point_distance &&
        is_huaman_shaped_level_1 &&
        local_view_.routing->routing_request().local_routing_type() !=
            routing::ROUTING_T4T5_TO_J4_WEST &&
        local_view_.routing->routing_request().local_routing_type() !=
            routing::ROUTING_J4_TO_T4T5_EAST &&
        local_view_.routing->routing_request().local_routing_type() !=
            routing::ROUTING_J4_TO_T4T5_EAST_NEAR &&
        routing_end_sl.s() > adc_sl_boundary.end_s() &&
        std::fabs(routing_end_sl.s() - adc_sl_boundary.end_s()) <
            FLAGS_rerouting_human_shape_distance) {
      cnt_obs_human_shaped_ = 0;
      if (is_stopped_ && cnt_block_human_shaped_++ > kBlockageCnt) {
        is_end_all_steps_in_advance = true;
        AINFO << "blockage near the endpoint, end "
                 "(huaman_shaped_level==1) early";
      }
    } else {
      cnt_block_human_shaped_ = 0;
    }
  } else {
    cnt_block_human_shaped_ = 0;
    cnt_obs_human_shaped_ = 0;
  }
  bool is_end_early_for_west = false;
  if (local_view_.routing->routing_request().local_routing_type() !=
          routing::ROUTING_T4T5_TO_J4_WEST &&
      local_view_.routing->routing_request().rerouting_info().huaman_shaped().level() == 2 &&
      routing_end_sl.s() - adc_center_s < 15 * kReroutingMinDistance) {
    is_end_early_for_west = true;
    AINFO << "not ROUTING_T4T5_TO_J4_WEST, end (huaman_shaped_level==2) "
             "early";
  } else if (std::fabs(routing_end_sl.s() - adc_center_s) <
             kReroutingMinDistance) {
    is_end_early_for_west = true;
  }

  if (injector_->planning_context() != nullptr && is_endpoint_sl_usable &&
      (is_end_steps_in_advance || is_end_early_for_west ||
       is_end_all_steps_in_advance)) {
    huaman_shaped_driver = true;
    AINFO << "ReroutingForHumanShaped:"
          << local_view_.routing->routing_request()
                 .rerouting_info()
                 .huaman_shaped()
                 .level()
          << "," << is_end_steps_in_advance << "," << is_end_early_for_west
          << "," << is_end_all_steps_in_advance;
    if (is_end_all_steps_in_advance) {
      AINFO << "is_end_all_steps_in_advance = ture";
      if (!frame_->Rerouting(injector_->planning_context(),
                             ReroutingType::SECOND_REROUTING)) {
        AERROR << "Failed to send rerouting request";
      }
    } else {
      if (!frame_->Rerouting(injector_->planning_context(),
                             ReroutingType::HUMAN_SHAPED)) {
        AERROR << "Failed to send rerouting request";
      }
    }
  }
  return huaman_shaped_driver;
}

void OnLanePlanning::CheckDestinationReachedRerouting(
    ADCTrajectory* const ptr_trajectory_pb,
    bool near_target_distance,
    bool near_target_angle,
    bool huaman_shaped_driver) {
  auto* destination = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_destination();
  if (!near_target_distance || !near_target_angle || !is_stopped_) {
    return;
  }

  destination->set_has_reached_destination(true);
    CheckAndCleanupTopBullReverseState(ptr_trajectory_pb);
  if (local_view_.routing != nullptr) {
    // specific junction rerouting.
    if (!huaman_shaped_driver) {
      if (local_view_.routing->routing_request().rerouting_info().is_rerouting() &&
          injector_ != nullptr && injector_->planning_context() != nullptr) {
        AINFO << "ReroutingForTwo";
        if (!frame_->Rerouting(injector_->planning_context(),
                               ReroutingType::SECOND_REROUTING)) {
          AERROR << "Failed to send rerouting request";
        }
      } else if (local_view_.routing->routing_request().is_loop_running() &&
                 injector_ != nullptr &&
                 injector_->planning_context() != nullptr) {
        AINFO << "ReroutingForLoopRuning";
        if (!frame_->Rerouting(injector_->planning_context(),
                               ReroutingType::LOOP_RUNING)) {
          AERROR << "Failed to send rerouting request";
        }
      }
    }
    if (local_view_.routing->routing_request().rerouting_info().dead_road().is_rerouting() &&
        !local_view_.routing->routing_request().rerouting_info().is_rerouting() &&
        !local_view_.routing->routing_request().rerouting_info().huaman_shaped().is_part_rerouting() &&
        injector_ != nullptr && injector_->planning_context() != nullptr) {
      AINFO << "ReroutingForDeadEnd";
      if (!frame_->Rerouting(injector_->planning_context(),
                             ReroutingType::DEAD_END_ROAD)) {
        AERROR << "Failed to send rerouting request";
      }
    }
  }

  AERROR << "lane follow has_reached_destination.";
}

void OnLanePlanning::CheckLoopRunningRerouting(
    ADCTrajectory* const ptr_trajectory_pb,
    bool near_routing_point_distance) {
  if (local_view_.routing->routing_request().is_one_click_loop_running()) {
    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
    double min_cost = std::numeric_limits<double>::infinity();
    for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
      if (reference_line_info.IsDrivable() &&
          reference_line_info.Cost() < min_cost) {
        best_ref_info = &reference_line_info;
        min_cost = reference_line_info.Cost();
      }
    }

    if (injector_ != nullptr && best_ref_info != nullptr) {
      common::SLPoint routing_end_sl;
      bool is_endpoint_sl_usable = false;
      const auto routing_end = best_ref_info->get_routing_end();
      if (best_ref_info->reference_line().XYToSL(routing_end.pose(),
                                                 &routing_end_sl)) {
        is_endpoint_sl_usable = true;
      }
      auto adc_sl_boundary = best_ref_info->AdcSlBoundary();
      AINFO << "adc_sl_boundary:" << adc_sl_boundary.end_s()
            << ",routing_end_sl:" << routing_end_sl.s();
      if (injector_->planning_context() != nullptr && is_endpoint_sl_usable &&
          near_routing_point_distance &&
          (routing_end_sl.s() > adc_sl_boundary.end_s() &&
           routing_end_sl.s() - adc_sl_boundary.end_s() <
               kReroutingMaxDistance)) {
        AINFO << "ReroutingForLoopRuning -- start running circle test.";
        if (!frame_->Rerouting(injector_->planning_context(),
                               ReroutingType::LOOP_RUNING)) {
          AERROR << "Failed to send rerouting request";
        }
      }
    }
  }
}

void OnLanePlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
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

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_openspace_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  AINFO << __func__ << ", is_in_openspace: "
        << frame_->open_space_info().is_on_open_space_trajectory();
  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    SetOpenSpaceTrajectory(ptr_trajectory_pb);
  } else {
    if (!frame_->FindDriveReferenceLineInfo()) {
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

void OnLanePlanning::SetOpenSpaceTrajectory(
    ADCTrajectory* const ptr_trajectory_pb) {
  frame_->mutable_open_space_info()->sync_debug_instance();
  const auto& publishable_trajectory =
      frame_->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame_->open_space_info().publishable_trajectory_data().second;
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
      frame_->open_space_info().latency_stats());

  if (FLAGS_enable_openspace_record_debug) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
    ADEBUG << "Open space debug information added!";
    ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                         ptr_debug);
  }
}

void OnLanePlanning::SetPublicRoadTrajectory(
    const double current_time_stamp,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();

  // Store current frame stitched path for possible speed fallback in next
  // frames
  DiscretizedPath current_frame_planned_path;
  for (const auto& trajectory_point : stitching_trajectory) {
    current_frame_planned_path.push_back(
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
  frame_->set_current_frame_planned_path(current_frame_planned_path);
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  ptr_debug->MergeFrom(best_ref_info->debug());
  if (FLAGS_export_chart) {
    ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
  } else {
    ExportReferenceLineDebug(ptr_debug);
    // Export additional ST-chart for failed lane-change speed planning
    const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
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

  if (is_railway_crossing_) {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::RAILWAY_CROSSING);
  } else if (is_dangerous_road_) {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::DANGEROUS_ROAD);
  } else if (best_ref_info->NoGreenInJunction()) {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::NO_GREEN_IN_JUNCTION);
  } else {
    ptr_trajectory_pb->set_warning_type(ADCTrajectory::WARNING_NONE);
  }
}

void OnLanePlanning::AddPlanningRecordDebug(
    const ReferenceLineInfo* best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (FLAGS_enable_record_debug) {
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

void OnLanePlanning::UpdateTrajectory(const ReferenceLineInfo* best_ref_info,
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

  if (ptr_trajectory_pb->is_backward_trajectory()) {
    if (common::VehicleSignal::TURN_RIGHT ==
        ptr_trajectory_pb->decision().vehicle_signal().turn_signal()) {
      ptr_trajectory_pb->mutable_decision()
          ->mutable_vehicle_signal()
          ->set_turn_signal(common::VehicleSignal::TURN_LEFT);
    } else if (common::VehicleSignal::TURN_LEFT ==
               ptr_trajectory_pb->decision().vehicle_signal().turn_signal()) {
      ptr_trajectory_pb->mutable_decision()
          ->mutable_vehicle_signal()
          ->set_turn_signal(common::VehicleSignal::TURN_RIGHT);
    }
  }

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

double OnLanePlanning::CalcRemainDistance(
    const ReferenceLineInfo* reference_line_info) {
  auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = reference_line_info->LocateLaneInfo(s);
  injector_->set_adc_lane_turn(locate_lane->lane().turn());
  
  // check dangerous and gatewaye (railway crossing) road
  is_dangerous_road_ = false;
  is_railway_crossing_ = false;
  if (nullptr != locate_lane) {
    if (locate_lane->lane().has_is_dangerous_road() &&
        locate_lane->lane().is_dangerous_road()) {
      is_dangerous_road_ = true;
    }
    if (locate_lane->lane().has_is_railway_crossing() &&
        locate_lane->lane().is_railway_crossing()) {
      is_railway_crossing_ = true;
    }
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
          CalcEgoPassageDistance(reference_line, i, adc_sl_boundary, ego_pose,
                                 remain_distance);
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

void OnLanePlanning::CalcEgoPassageDistance(const ReferenceLine& reference_line,
                                            const int& i,
                                            const SLBoundary& adc_sl_boundary,
                                            const common::PointENU& ego_pose,
                                            double& remain_distance) {
  const auto& lane_id =
      local_view_.routing->road(i).passage(0).segment().rbegin()->id();
  auto lane_info_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (nullptr == lane_info_ptr) {
    for (auto& lane_segment :
         local_view_.routing->road(i).passage(0).segment()) {
      remain_distance += (lane_segment.end_s() - lane_segment.start_s());
    }
  } else {
    const auto& road_end_pose = *(lane_info_ptr->points().rbegin());
    SLPoint road_end_sl;
    if (reference_line.XYToSL(road_end_pose, &road_end_sl)) {
      remain_distance += road_end_sl.s() - adc_sl_boundary.end_s();
    } else {
      remain_distance +=
          common::util::DistanceXY(ego_pose, *lane_info_ptr->points().rbegin());
    }
  }
}

bool OnLanePlanning::IsEgoPassage(const std::string& ego_lane_id,
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

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
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

void OnLanePlanning::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

void OnLanePlanning::ExportOnLaneChart(
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

void OnLanePlanning::ExportOpenSpaceChart(
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

void OnLanePlanning::PathPointNormalizing(double rotate_angle,
                                          const Vec2d& translate_origin,
                                          double* x, double* y, double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void OnLanePlanning::AddOpenSpaceOptimizerResult(
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

void OnLanePlanning::AddOpenSpaceObstacleDebugInfo(
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

void OnLanePlanning::AddOpenSpaceSmoothLineDebugInfo(
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

void OnLanePlanning::AddOpenSpaceWarmStartDebugInfo(
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

void OnLanePlanning::AddOpenSpaceStartEndPointDebugInfo(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  auto open_space_debug = debug_info.planning_data().open_space();
  auto chart = debug_chart->mutable_planning_data()->mutable_chart()->rbegin();
  // Show Start End Point
  auto* start_end_line = chart->add_line();
  start_end_line->set_label("StartEndPoint");
  auto* start_point_debug = start_end_line->add_point();
  start_point_debug->set_x(open_space_debug.hybrid_search_info()
                               .start_point()
                               .path_point()
                               .x());
  start_point_debug->set_y(open_space_debug.hybrid_search_info()
                               .start_point()
                               .path_point()
                               .y());
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

void OnLanePlanning::AddOpenSpaceROILineDebugInfo(
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

void OnLanePlanning::AddOpenSpaceEndHeadingLineDebugInfo(
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

void OnLanePlanning::AddPartitionedTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.hybrid_search_info().chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_->vehicle_state();
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
  if (frame_->dynamic_adc_position().x() > 0) {
    auto* dynamic_adc_shape = chart->add_car();
    dynamic_adc_shape->set_x(frame_->dynamic_adc_position().x());
    dynamic_adc_shape->set_y(frame_->dynamic_adc_position().y());
    dynamic_adc_shape->set_heading(frame_->dynamic_adc_position().z());
    dynamic_adc_shape->set_label("dynamic_ADV");
    dynamic_adc_shape->set_color("rgba(154, 162, 235, 1)");
  }
  const auto& obstacles = frame_->obstacles();
  double adc_x = injector_->vehicle_state()->x();
  double adc_y = injector_->vehicle_state()->y();
  for (const auto* obstacle : obstacles) {
    double obs_x = obstacle->Perception().position().x();
    double obs_y = obstacle->Perception().position().y();
    double distance_to_adc = std::sqrt((obs_x - adc_x) * (obs_x - adc_x) +
                                       (obs_y - adc_y) * (obs_y - adc_y));
    if (distance_to_adc > kDistanceofDisplayObs) {
      continue;
    }
    const auto& obs_polygon = obstacle->PerceptionPolygon();
    auto* obs_add_polygon = chart->add_polygon();
    obs_add_polygon->set_label(obstacle->Id());
    for (const auto& point : obs_polygon.points()) {
      auto* add_point = obs_add_polygon->add_point();
      add_point->set_x(point.x());
      add_point->set_y(point.y());
      ADEBUG << "point.x() = " << std::setprecision(9) << point.x()
             << "   point.y() = " << std::setprecision(9) << point.y();
    }
  }

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

void OnLanePlanning::DrawOpenSpaceChosenTrajectory(
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

void OnLanePlanning::DrawOpenSpaceTrajectoryStitchPoint(
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

void OnLanePlanning::DrawOpenSpaceFallbackTrajectory(
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

void OnLanePlanning::AddStitchSpeedProfile(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
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
  const auto& last_trajectory =
      injector_->frame_history()->Latest()->current_frame_planned_trajectory();
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

void OnLanePlanning::AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                                       planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
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

VehicleState OnLanePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                            const double curr_timestamp) const {
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

void OnLanePlanning::AddPublishedAcceleration(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
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

bool OnLanePlanning::CheckAndPubStopTrajectory(
    const double start_timestamp, ReferenceLineInfo* const reference_line_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(ptr_trajectory_pb);

  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (nullptr == obstacle) {
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
      ptr_trajectory_pb->set_gear(vehicle_gear_);
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
      GenerateStopTrajectory(ptr_trajectory_pb);

      return true;
    }
  }

  return false;
}

void OnLanePlanning::RemoteDecider(ADCTrajectory* const ptr_trajectory_pb) {
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
        !is_near_traffic_light_stop_line_ && 
        injector_->use_reverse_type() == 
                              ReverseTrajectoryType::FORWARD_DRIVING) {
      stop_long_time_request = true;
      AERROR << "stop long time request for normal stop.";
    }
    if (Clock::NowInSeconds() - stop_start_time_ > KStopLineStopTime &&
        (is_near_traffic_light_stop_line_ || 
          injector_->use_reverse_type() == 
                              ReverseTrajectoryType::OBSTACLE_AVOIDANCE)) {
      stop_long_time_request = true;
      AERROR << "stop long time request for near the traffic light stop line.";
    }
    if (Clock::NowInSeconds() - stop_start_time_ > KLaneBorrowFailTime &&
        lane_borrow_failed_) {
      lane_borrow_request = true;
      AERROR << "lane borrow fail request";
    }
    planning::ReferenceLineInfo* best_ref_info = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();
    if (nullptr == frame_) {
      AWARN << "RemoteDecider called before frame initialization.";
    } else {
      // find best reference line
      for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
        if (reference_line_info.IsDrivable() &&
            reference_line_info.Cost() < min_cost) {
          best_ref_info = &reference_line_info;
          min_cost = reference_line_info.Cost();
        }
      }
    }
    bool is_passing_gate = false;
    if (best_ref_info != nullptr) {
      is_passing_gate = best_ref_info->IsPassingGate();
    } 
    if (Clock::NowInSeconds() - stop_start_time_ > KGateBlockStopTime && is_passing_gate) {
      stop_for_gate_block_ = true;
      AERROR << "stop for gate block request.";
    }
  }

  // 3. teb stop long or teb more failed request
  if (nullptr != frame_ &&
      (Frame::FaultReport::TEB_STOP_LONG == frame_->fault_report_ ||
       Frame::FaultReport::TEB_MORE_FAILED == frame_->fault_report_)) {
    teb_failed_request = true;
    AERROR << "teb stop long or teb more failed request request";
  }
  if (planning_error_request || stop_long_time_request ||
      request_remote_for_traffic_light_ || lane_borrow_request ||
      teb_failed_request || stop_for_gate_block_) {
    // AINFO << "request_remote" << ",planning_error_request:" << planning_error_request
    //       << ",stop_long_time_request:" << stop_long_time_request
    //       << ",request_remote_for_traffic_light_:" << request_remote_for_traffic_light_
    //       << ",lane_borrow_request:" << lane_borrow_request
    //       << ",teb_failed_request:" << teb_failed_request;
    PubRemoteRequest(lane_borrow_request, ptr_trajectory_pb);
  }
}

void OnLanePlanning::PubRemoteRequest(const bool lane_borrow_request,
                                      ADCTrajectory* const ptr_trajectory_pb) {
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
  } else if (nullptr != frame_ &&
             Frame::FaultReport::TEB_STOP_LONG == frame_->fault_report_) {
    remote_scenario = ADCTrajectory::REMOTE_PUB_TEB_STOP_LONG;
  } else if (nullptr != frame_ &&
             Frame::FaultReport::TEB_MORE_FAILED == frame_->fault_report_) {
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
  }else if (stop_for_gate_block_) {
    remote_scenario = ADCTrajectory::REMOTE_GATE_BLOCK;
  } 
  else {
    remote_scenario = ADCTrajectory::REMOTE_PUB_STOP_LONG;
  }
  remote_status->set_remote_scenario(remote_scenario);

  if (ADCTrajectory::REMOTE_PUB_STOP_LONG == remote_scenario) {
    ptr_trajectory_pb->set_ad_behavior(ADCTrajectory::AD_PARKING);
  }
}

void OnLanePlanning::CancelRemoteRequest(
    ADCTrajectory* const ptr_trajectory_pb) {
  ADCTrajectory::RemotePub* remote_status =
      ptr_trajectory_pb->mutable_remote_status();
  remote_status->set_is_enabled(false);
  ADCTrajectory::RemoteScenario remote_scenario =
      ADCTrajectory::REMOTE_PUB_UNKNOWN;
  remote_status->set_remote_scenario(remote_scenario);
}

void OnLanePlanning::FallbackPlanningThread() {
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

    last_frenet_frame_points.clear();
    last_discretized_path_points.clear();
    path_obstacles.clear();
    obstacles.clear();

    SLBoundary adc_sl_boundary;
    SLPoint last_sl_point;
    SLPoint sl_point_of_match_point_in_last_frame;
    double last_time_stamp = -1.0;
    bool has_last_frame = false;

    //  get last frame and last trajectorypub
    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      if (nullptr == injector_->frame_history()->Latest()) {
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

bool OnLanePlanning::GetLastFramePoint(
    double* const last_time_stamp, SLPoint* const last_sl_point,
    SLPoint* const sl_point_of_match_point_in_last_frame) {
  const auto* last_frame = injector_->frame_history()->Latest();
  if (!last_frame->DriveReferenceLineInfo()) {
    AERROR << " last frame havn't reference_line_info";
    return false;
  }
  if (nullptr == last_publishable_trajectory_ ||
      nullptr == last_publishable_trajectory_.get()) {
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

void OnLanePlanning::GetLastFrameInfo(
    std::vector<FrenetFramePoint>* const last_frenet_frame_points,
    std::vector<common::PathPoint>* const last_discretized_path_points,
    std::vector<Obstacle>* const path_obstacles,
    std::vector<Obstacle>* const obstacles, SLBoundary* const adc_sl_boundary) {
  const auto* last_frame = injector_->frame_history()->Latest();
  for (auto point = last_frame->DriveReferenceLineInfo()
                        ->path_data()
                        .frenet_frame_path()
                        .begin();
       point != last_frame->DriveReferenceLineInfo()
                    ->path_data()
                    .frenet_frame_path()
                    .end();
       point++) {
    last_frenet_frame_points->push_back(std::move(*point));
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
    last_discretized_path_points->push_back(std::move(*point));
  }

  *adc_sl_boundary = last_frame->DriveReferenceLineInfo()->AdcSlBoundary();

  const auto& upstream_path_obstacles =
      last_frame->DriveReferenceLineInfo()->path_decision().obstacles().Items();
  for (uint i = 0; i < upstream_path_obstacles.size(); i++) {
    obstacles->push_back(std::move(*upstream_path_obstacles.at(i)));
    path_obstacles->push_back(std::move(*upstream_path_obstacles.at(i)));
  }
}

bool OnLanePlanning::GetReferenceTrajectoryFromLastFrame(
    const std::vector<FrenetFramePoint>& last_frenet_frame_points,
    const std::vector<common::PathPoint>& last_discretized_path_points,
    const std::vector<Obstacle>& path_obstacles,
    const std::vector<Obstacle>& obstacles, const SLBoundary& adc_sl_boundary,
    const double& last_time_stamp, const SLPoint& last_sl_point,
    const SLPoint& sl_point_of_match_point_in_last_frame,
    DiscretizedTrajectory* const reference_trajectory) {
  double veh_rel_time =
      Clock::NowInSeconds() - last_time_stamp + kFallbackOncetime;
  const auto* last_frame = injector_->frame_history()->Latest();
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

bool OnLanePlanning::CheckFallbackContinue() {
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

void OnLanePlanning::FallbackStop() {
  is_fallback_planning_thread_stop_ = true;
  if (fallback_planning_thread_ != nullptr &&
      fallback_planning_thread_->Joinable()) {
    fallback_planning_thread_->Join();
  }
}

void OnLanePlanning::GeneratePreferredLane() {
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
void OnLanePlanning::GenerateRouteLaneInfo() {
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
      if (nullptr == ptr_lane) {
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

void OnLanePlanning::AddPassageLaneInfo(
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

void OnLanePlanning::CheckOffLaneDeparture() {
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

void OnLanePlanning::CalcuHonkingStats(ADCTrajectory* const ptr_trajectory_pb) {
  bool reached_destination = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_destination()
                                 ->has_reached_destination();

  AINFO << "reached_destination: " << reached_destination;
  if (reached_destination) {
    ptr_trajectory_pb->mutable_honking_status()->set_need_honking(true);
    ptr_trajectory_pb->mutable_honking_status()->set_scenario(
        HonkingScenario::REACHE_DESTINATION);
  } else {
    ptr_trajectory_pb->mutable_honking_status()->set_need_honking(false);
    ptr_trajectory_pb->mutable_honking_status()->set_scenario(
        HonkingScenario::NO_HONKING);
  }
}

void OnLanePlanning::CalcuDisplayType(ADCTrajectory* const ptr_trajectory_pb) {
  planning::ReferenceLineInfo* best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();

  ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
  if (NeedDangerStayAwayDisplay(local_view_,
                                injector_->vehicle_state()->vehicle_state())) {
    ptr_trajectory_pb->set_display_type(DANGER_STAY_AWAY);
    return;
  }
  bool is_direct_turn = false, is_need_to_turn = false, at_left_turn = false,
       at_right_turn = false, path_turn_left = false, path_turn_right = false;

  // find best reference line
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == best_ref_info) {
    AERROR << "best_ref_info is nullptr.";
    return;
  }
  best_ref_info->GetTurnInfo(&is_direct_turn, &is_need_to_turn, &at_left_turn,
                             &at_right_turn, &path_turn_left, &path_turn_right);
  const auto t_turn_signal =
      ptr_trajectory_pb->decision().vehicle_signal().turn_signal();
  static int16_t obs_count = 0;
  double battery_soc = local_view_.chassis.get()->battery_soc();
  double tire_pressure_11 = local_view_.chassis.get()->tire_pressure_11();
  double tire_pressure_14 = local_view_.chassis.get()->tire_pressure_14();
  double tire_pressure_21 = local_view_.chassis.get()->tire_pressure_21();
  double tire_pressure_24 = local_view_.chassis.get()->tire_pressure_24();
  double tire_pressure_31 = local_view_.chassis.get()->tire_pressure_31();
  double tire_pressure_34 = local_view_.chassis.get()->tire_pressure_34();
  double tire_pressure_41 = local_view_.chassis.get()->tire_pressure_41();
  double tire_pressure_44 = local_view_.chassis.get()->tire_pressure_44();

  double max_tire_pressure = std::max(
      tire_pressure_11,
      std::max(
          tire_pressure_14,
          std::max(tire_pressure_21,
                   std::max(tire_pressure_24,
                            std::max(tire_pressure_31,
                                     std::max(tire_pressure_34,
                                              std::max(tire_pressure_41,
                                                       tire_pressure_44)))))));
  double min_tire_pressure = std::min(
      tire_pressure_11,
      std::min(
          tire_pressure_14,
          std::min(tire_pressure_21,
                   std::min(tire_pressure_24,
                            std::min(tire_pressure_31,
                                     std::min(tire_pressure_34,
                                              std::min(tire_pressure_41,
                                                       tire_pressure_44)))))));

  if (min_tire_pressure < kLowTirePressureThres) {
    ptr_trajectory_pb->set_display_type(LOW_TIRE_PRESSURE);
  } else if (max_tire_pressure > kHighTirePressureThres) {
    ptr_trajectory_pb->set_display_type(HIGH_TIRE_PRESSURE);
  } else if (battery_soc < kLowSOCThres) {
    ptr_trajectory_pb->set_display_type(LOW_SOC_WARNING);
  } else if (ptr_trajectory_pb->decision()
                 .main_decision()
                 .stop()
                 .reason_code() == StopReasonCode::STOP_REASON_OBSTACLE) {
    if (++obs_count >= KobsConfirmCnt) {
      CalcuObstaclesDisplay(best_ref_info, ptr_trajectory_pb);
      obs_count = KobsConfirmCnt;
    }
  } else if (injector_->planning_context()
                 ->mutable_planning_status()
                 ->mutable_destination()
                 ->has_reached_destination()) {
    ptr_trajectory_pb->set_display_type(REACHED_DESTINATION);
  }else if(injector_->use_reverse_type()){
    ptr_trajectory_pb->set_display_type(REVERCE_DRIVE);
  }
  else if (kMaxReroutingTimes >= local_view_.routing->routing_request().rerouting_info().huaman_shaped().level())
  {
    ptr_trajectory_pb->set_display_type(CROSS_DIAGNAL);
  }
   else if (best_ref_info->GetIsAtTurn() || is_need_to_turn) {
    // TODO: TEMPERARY JUDGEMENT
    if (local_view_.chassis.get()->driving_mode() !=
        Chassis::COMPLETE_AUTO_DRIVE) {
      double wheel_angle =
          (local_view_.chassis.get()->bridge_1_left_wheel_angle() +
           local_view_.chassis.get()->bridge_1_right_wheel_angle()) /
          2;
      if (wheel_angle > kTurnWheelAngleThres) {
        ptr_trajectory_pb->set_display_type(TURN_LEFT);
        if (ptr_trajectory_pb->is_backward_trajectory()) {
          ptr_trajectory_pb->set_display_type(TURN_RIGHT);
        }
      } else if (wheel_angle < -kTurnWheelAngleThres) {
        ptr_trajectory_pb->set_display_type(TURN_RIGHT);
        if (ptr_trajectory_pb->is_backward_trajectory()) {
          ptr_trajectory_pb->set_display_type(TURN_LEFT);
        }
      } else {
        ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
      }
    } else {
      if (ptr_trajectory_pb->need_diagonal()) {
        AINFO << "need_diagonal: " << ptr_trajectory_pb->need_diagonal();
        AINFO << "IsAtLeftTurn" << path_turn_left;
        AINFO << "IsAtRightTurn" << path_turn_right;
        if (path_turn_left) {
          ptr_trajectory_pb->set_display_type(DIAGNAL_LEFT);
        } else if (path_turn_right) {
          ptr_trajectory_pb->set_display_type(DIAGNAL_RIGHT);
        } else {
        }
      } else {
        if (common::VehicleSignal::TURN_LEFT == t_turn_signal) {
          ptr_trajectory_pb->set_display_type(TURN_LEFT);
          if (ptr_trajectory_pb->is_backward_trajectory()) {
            ptr_trajectory_pb->set_display_type(TURN_RIGHT);
          }
        } else if (common::VehicleSignal::TURN_RIGHT == t_turn_signal) {
          ptr_trajectory_pb->set_display_type(TURN_RIGHT);
          if (ptr_trajectory_pb->is_backward_trajectory()) {
            ptr_trajectory_pb->set_display_type(TURN_LEFT);
          }
        } else {
          ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
        }
      }
    }
  } else {
    if(--obs_count > -KobsConfirmCnt)
    {
      CalcuObstaclesDisplay(best_ref_info, ptr_trajectory_pb);
      obs_count = std::max(obs_count, (int16_t)(-6));
    }
    else{
      ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
    }
  }
  AINFO << "obs_count: "<< obs_count;
}

void OnLanePlanning::CalcuObstaclesDisplay(
    ReferenceLineInfo* const best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  const std::string& obs_id = ptr_trajectory_pb->block_obs_id();
  if (obs_id.empty()) {
    ptr_trajectory_pb->set_display_type(STOP_FOR_OBSTACLE);
    return;
  }

  auto* mutable_obstacle = best_ref_info->path_decision()->Find(obs_id);
  if (nullptr == mutable_obstacle) {
    ptr_trajectory_pb->set_display_type(STOP_FOR_OBSTACLE);
    return;
  }

  if (PerceptionObstacle::VEHICLE == mutable_obstacle->Perception().type()) {
    ptr_trajectory_pb->set_display_type(STOP_FOR_VEHICLE);
  } else if (PerceptionObstacle::PEDESTRIAN ==
             mutable_obstacle->Perception().type()) {
    ptr_trajectory_pb->set_display_type(STOP_FOR_PEDESTRIAN);
  } else {
    ptr_trajectory_pb->set_display_type(STOP_FOR_OBSTACLE);
  }
}

}  // namespace planning
}  // namespace century
