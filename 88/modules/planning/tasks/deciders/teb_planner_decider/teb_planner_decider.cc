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

#include "modules/planning/tasks/deciders/teb_planner_decider/teb_planner_decider.h"

#include <memory>
#include <utility>

#include "cyber/time/clock.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/math/discrete_points_math.h"

// #include "modules/prediction/proto/prediction_obstacle.pb.h"
namespace century {
namespace planning {

constexpr double kEp = 0.1;
// ------------Just For Rescue Scenario-------------------------
constexpr double kRescueLatCost = 20.0;
constexpr double kRescueEgoLCost = 10.0;
constexpr double kRescueLonCost = 4.0;
constexpr double kRescueObstacleCost = 10;
constexpr double kBaseCost = 500;
constexpr double kCollisionCost = 10086;
constexpr double kFrontBlockedCost = 200.0;
constexpr double kHisotryCost = 10.0;
constexpr double kEp2 = 1e-6;

constexpr double kPullOverSampleIntervalS = 0.5;
constexpr double kPullOverSampleIntervalL = 0.1;
constexpr double kPullOverLatCostWeight = 10.0;
constexpr double kPullOverLonCostWeight = 1.0;
constexpr double kPullOverObsCostWeight = 2.0;

// ------------Just For Rescue Scenario-------------------------

constexpr double kLargeDist = 100.0;
constexpr double kInParkingLot = 2.0;
constexpr double kStopSpeed = 0.05;
constexpr double kCheckStopTimeWindows = 1.7;       // second
constexpr double kCheckStopTimeMin = 1.2;           // second
constexpr double kStopLongTimeValidThr = 0.9;       // %
constexpr double kUturnCheckStopTimeWindows = 3.0;  // second
constexpr double kUturnCheckStopTimeMin = 2.9;      // second
constexpr double kCheckFallBackTimeWindows = 1.0;   // second
constexpr double kCheckFallBackTimeMin = 0.7;       // second
constexpr double kFallBackValidThr = 0.9;           // %
constexpr double kIngoreRoadBoundaryDist = 17.0;
constexpr int kMinLaneBoundaryPointSize = 2;
constexpr double kCheckDist = 10.0;
constexpr double kBoundaryInterval = 8.0;
constexpr double kBoundaryLargeInterval = 8.0;
constexpr double kDynamicObstacleSpeed = 2.0;
constexpr double kPreferL = 0.0;
constexpr double kBoundaryFrontLength = 27;
constexpr double kBoundaryBackLength = 7;
constexpr double kBoundaryWidth = 20;
constexpr double kWidthBuffer = 6.5;
constexpr double kSearchRadius = 20;
constexpr double kLongIntervalS = 0.5;
constexpr double kCheckStep = 1.0;
constexpr double kFrontSearchCheckDist = 2.0;

constexpr double kReplanDist = 3.0;
constexpr double kBackEndPoseCost = 20.0;
constexpr double kSearchExtraS = 10.0;
constexpr double kSearchMinInterval = 1.0;
constexpr double kLittleObjSize = 1.5;
constexpr double kMultiFrontObjBuffer = 2.0;
constexpr double kPulloverSearchExtraS = 5.0;
constexpr int kPolygonMinPointNum = 3;
constexpr double kCrossJunctionDist = 50.0;
constexpr double kPlayStreetJunctionDist = 2.0;
constexpr double kShiftAngle = M_PI / 12;

constexpr int kHisotryObsCountThreshold = 10;
constexpr double kObsNearHistoryObsThr = 0.5;

constexpr double kMinStopDisance = 0.5;
constexpr double kMaxStopDistance = 3.0;
constexpr double kMinStopDistanceCityRoad = 3.0;
constexpr double kMaxStopDistanceCityRoad = 7.0;
constexpr size_t kFailCount = 1;

constexpr double kMinBackDis = 0.5;
constexpr double kNearRoutingEndPointRoiDis = 6.0;
constexpr bool kTempProcessThreadFlages = false;
constexpr double kSlowSpeed = 1.0;
constexpr double kSafeBuffer = 0.5;

using century::common::ErrorCode;
using century::common::PointENU;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;

using century::common::PathPoint;
using century::common::TrajectoryPoint;
using century::common::VehicleConfigHelper;
using century::cyber::Clock;
using century::hdmap::HDMapUtil;
using century::hdmap::Id;
using century::hdmap::Junction;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::Lane;
using century::hdmap::LaneBoundary;
using century::hdmap::LaneBoundaryType;
using century::hdmap::LaneInfoConstPtr;
using century::hdmap::LaneSegment;
using century::hdmap::ParkingSpaceInfoConstPtr;
using century::hdmap::Path;
using century::routing::LaneWaypoint;
using century::routing::ParkingSpaceType;
using century::routing::RoutingRequest;

TEBPlannerDecider::TEBPlannerDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_params_ =
      century::common::VehicleConfigHelper::GetConfig().vehicle_param();
  rescue_end_point_.set_x(injector->vehicle_state()->x());
  rescue_end_point_.set_y(injector->vehicle_state()->y());
  rescue_status_ = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_rescue();
  injector_->set_rescue_replan(false);
  goals_vector_.clear();
  ready_to_replan_ = false;
  fallback_replan_count_ = 0;
  stop_long_count_ = 0;
}

void TEBPlannerDecider::SaveHistoryStaticObs(
    const Frame *frame, ThreadSafeIndexedObstacles *obstacles_by_frame_,
    std::deque<std::vector<century::planning::Obstacle>>
        *const history_obs_lists_ptr_) {
  current_obs_list_.clear();
  current_obs_list_.shrink_to_fit();
  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    if (obstacle->IsOutOfOpenSpaceROI()) {
      // AINFO << "IsOutOfOpenSpaceROI Obs_ID: " << obstacle->Id();
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    current_obs_list_.emplace_back(*obstacle);
  }
  if (history_obs_lists_ptr_->size() > kHisotryObsCountThreshold - 1) {
    history_obs_lists_ptr_->pop_front();
    history_obs_lists_ptr_->emplace_back(current_obs_list_);
  } else {
    history_obs_lists_ptr_->emplace_back(current_obs_list_);
  }
}
double TEBPlannerDecider::CalcBackDistance(const double lat_offset,
                                           const double lon_offset,
                                           const double min_stop_distance) {
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  AINFO << "min trun radius: " << min_turn_radius;
  AINFO << "++l: " << lat_offset;
  AINFO << "++s: " << lon_offset;
  if (is_in_city_road_) {
    return std::max(std::min(std::fabs(min_stop_distance - lon_offset),
                             kMaxStopDistanceCityRoad),
                    kMinStopDistanceCityRoad);
  } else {
    double res = 0.0;
    res = min_turn_radius * min_turn_radius -
          (min_turn_radius - std::fabs(lat_offset)) *
              (min_turn_radius - std::fabs(lat_offset));
    AINFO << res;
    if (res < kEp2) {
      return 0.0;
    }
    res = std::sqrt(res);
    return std::max(std::min(std::fabs(res - lon_offset), kMaxStopDistance),
                    kMinStopDisance);
  }
}

void TEBPlannerDecider::StableStaticObsWithHistory(
    const Frame *frame,
    std::deque<std::vector<century::planning::Obstacle>>
        *const history_obs_lists_ptr_,
    ThreadSafeIndexedObstacles *obstacles_by_frame_) {
  perception_polygon_list_.clear();
  perception_polygon_list_.shrink_to_fit();
  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    if (obstacle->IsOutOfOpenSpaceROI()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    std::vector<Vec2d> polygon_points;
    polygon_points.clear();
    polygon_points.shrink_to_fit();

    std::size_t history_obs_size = history_obs_lists_ptr_->size() > 1
                                       ? history_obs_lists_ptr_->size() - 1
                                       : 0;
    for (std::size_t i = 0; i < history_obs_size; ++i) {
      std::vector<century::planning::Obstacle> frame_stable_obs_list =
          (*history_obs_lists_ptr_)[i];
      // AINFO << "frame_stable_obs_list.size() : "
      // << frame_stable_obs_list.size();
      for (std::size_t j = 0; j < frame_stable_obs_list.size(); ++j) {
        // AINFO << "obstacle->Id() : " << obstacle->Id();
        if (!obstacle->Id().compare(frame_stable_obs_list[j].Id())) {
          Vec2d point = Vec2d(obstacle->Perception().position().x(),
                              obstacle->Perception().position().y());
          double dist = point.DistanceTo(
              Vec2d(frame_stable_obs_list[j].Perception().position().x(),
                    frame_stable_obs_list[j].Perception().position().y()));
          if (std::fabs(dist) < kObsNearHistoryObsThr) {
            // AINFO << "same id obs pos near";
            century::common::math::Polygon2d obs_polygon =
                frame_stable_obs_list[j].PerceptionPolygon();
            polygon_points.insert(polygon_points.end(),
                                  obs_polygon.points().begin(),
                                  obs_polygon.points().end());
          }
        }
      }
    }
    if (polygon_points.size() >= kPolygonMinPointNum) {
      common::math::Polygon2d perception_polygon;
      if (!common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                      &perception_polygon)) {
        AERROR << "ComputeConvexHull failed";
        continue;
      }

      perception_polygon_list_.emplace_back(perception_polygon);
      obstacles_by_frame_->Find(obstacle->Id())->no_const_PerceptionPolygon() =
          perception_polygon_list_.back();
    }
    polygon_points.clear();
    polygon_points.shrink_to_fit();
  }
}

void TEBPlannerDecider::DebugStaticObsWithHistory(
    const Frame *frame, ThreadSafeIndexedObstacles *obstacles_by_frame_) {
  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    if (obstacle->IsOutOfOpenSpaceROI()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    AINFO << "obstacle->Id() : " << obstacle->Id()
          << "obs_perception_polygon_size: "
          << obstacle->PerceptionPolygon().points().size();
  }
}

void TEBPlannerDecider::DeterMainJuncionInPlayStreet() {
  junction_in_play_street_ = false;
  is_in_junction_ = false;
  if (!FLAGS_disenable_play_street_common_junction) {
    AINFO << "FLAGS_disenable_play_street_common_junction 0";
    return;
  }

  // common junction
  if (injector_->is_in_common_junction_) {
    is_in_junction_ = true;
    AINFO << "vehicle in common_junction";
    // playstreet
    if (injector_->is_in_play_street) {
      junction_in_play_street_ = injector_->is_in_play_street;
    }
    AINFO << "junction_in_play_street_ " << junction_in_play_street_;
  }
  return;
}

Status TEBPlannerDecider::Process(Frame *frame) {
  if (nullptr == frame) {
    const std::string msg =
        "Invalid frame, fail to process the TEBPlannerDecider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  AINFO << "tar status: "
        << static_cast<int>(frame->open_space_info().tar_status());
  vehicle_state_ = frame->vehicle_state();
  obstacles_by_frame_ = frame->GetObstacleList();
  preset_long_buffer_ = FLAGS_astar_first_long_buffer;
  preset_lat_buffer_ = FLAGS_astar_first_lat_buffer;

  const auto &roi_type = config_.teb_roi_decider_config().roi_type();

  std::vector<std::vector<common::math::Vec2d>> roi_boundary;

  is_pullover_ready_ = injector_->is_in_near_goal_ &&
                       injector_->pullover_using_ &&
                       FLAGS_enable_use_pullover_mode;
  AERROR << "is_pullover_ready_: " << is_pullover_ready_
         << " is_in_near_goal_: " << injector_->is_in_near_goal_
         << " pullover_using_: " << injector_->pullover_using_;
  // use lane boundary for common junction in play street for rescue.
  DeterMainJuncionInPlayStreet();
  is_in_city_road_ =
      !injector_->is_in_play_street && FLAGS_enable_public_road_teb;
  frame->mutable_open_space_info()->set_is_rescue_mode(
      TEBRoiDeciderConfig::RESCUE == roi_type);
  SetCommonOriginPose(frame);
  AINFO << "is_in_city_road " << is_in_city_road_;

  // if (TEBRoiDeciderConfig::TRAFFIC_LIGHT == roi_type ||
  //     (injector_->use_teb_default_bound_ && is_in_junction_)) {
  //   const auto &nearby_path =
  //       frame->reference_line_info().front().reference_line().GetMapPath();
  //   if (!GetJunctionBoundary(frame, nearby_path, &roi_boundary)) {
  //     if (TEBRoiDeciderConfig::TRAFFIC_LIGHT != roi_type && is_in_junction_
  //     &&
  //         GenerateBoundary(frame, &roi_boundary)) {
  //       // try second
  //       AINFO << "GetJunctionBoundary failed ,use GenerateBoundary success";
  //     } else {
  //       const std::string msg = "Fail to get  teb boundary";
  //       AERROR << msg;
  //       return Status(ErrorCode::PLANNING_ERROR, msg);
  //     }
  //   }
  // } else {
  //   if (!GenerateBoundary(frame, &roi_boundary)) {
  //     const std::string msg = "Fail to get  teb boundary";
  //     AERROR << msg;
  //     return Status(ErrorCode::PLANNING_ERROR, msg);
  //   }
  // }

  AERROR << "IsAdcInDriveArea: "
         << frame->reference_line_info().front().IsAdcInDriveArea()
         << " is_in_play_street: " << injector_->is_in_play_street;
  if (FLAGS_enable_use_drive_area &&
      frame->reference_line_info().front().IsAdcInDriveArea() &&
      injector_->is_in_play_street && !injector_->is_in_common_junction_) {
    const double start_timestamp = Clock::NowInSeconds();

    if (!GetDriveAreaJunctionBoundary(frame, &roi_boundary)) {
      const std::string msg =
          "Teb_ROI: ERROR, in_junction, GetDriveAreaJunctionBoundary "
          "Fail";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    AERROR << "drive_area_time: " << time_diff_ms;
  } else {
    if (is_in_junction_) {
      if (junction_in_play_street_) {
        if (!GetPlayStreetJunctionBoundary(frame, &roi_boundary)) {
          const std::string msg =
              "Teb_ROI: ERROR, in_junction, GetPlayStreetJunctionBoundary Fail";
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
      } else if (TEBRoiDeciderConfig::TRAFFIC_LIGHT == roi_type) {
        if (!GetTrafficLightJunctionBoundary(frame, &roi_boundary)) {
          const std::string msg =
              "Teb_ROI: ERROR, in_junction, GetTrafficLightJunctionBoundary "
              "Fail";
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
      } else {
        const std::string msg = "Teb_ROI: ERROR, unknown_junction";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    } else {
      if (!GenerateBoundary(frame, &roi_boundary)) {
        const std::string msg =
            "Teb_ROI: ERROR, no_junction, GenerateBoundary Fail";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
  }

  // Set TrafficLight ROI for FilterOutObstacle
  if (is_in_junction_ && !junction_in_play_street_) {
    left_roi_l_ = config_.teb_roi_decider_config().frenet_boundary_left_l();
    right_roi_l_ = config_.teb_roi_decider_config().frenet_boundary_right_l();
    start_roi_s_ = config_.teb_roi_decider_config().frenet_boundary_start_s();
    end_roi_s_ = config_.teb_roi_decider_config().frenet_boundary_end_s();
  }
  AINFO << "roi_filter_obs l:" << left_roi_l_ << " r:" << right_roi_l_
        << " s:" << start_roi_s_ << " e:" << end_roi_s_;

  for (auto obstacle : obstacles_by_frame_->Items()) {
    if (FilterOutObstacle(*frame, *obstacle)) {
      obstacles_by_frame_->Find(obstacle->Id())->SetOutOfOpenSpaceROI(true);
      // AINFO << "obs_OutOfOpenSpaceROI:" << obstacle->IsOutOfOpenSpaceROI();
      continue;
    }
  }

  if (FLAGS_enable_use_costmap) {
    for (auto &polygon : frame_->filter_costmap_polygons()) {
      if (FilterOutCostMap(*frame, polygon)) {
        polygon.SetOutOfOpenSpaceROI(true);
        // AINFO << "polygon_OutOfOpenSpaceROI:";
        continue;
      }
    }
  }

  if (FLAGS_enable_stable_ststic_obs) {
    // AINFO << "StableStaticObsWithHistory_Start";
    // DebugStaticObsWithHistory(frame, obstacles_by_frame_);
    // AINFO << "obstacles_by_frame_->Items().size(): "
    //       << obstacles_by_frame_->Items().size();
    SaveHistoryStaticObs(frame, obstacles_by_frame_, history_obs_lists_ptr_);
    StableStaticObsWithHistory(frame, history_obs_lists_ptr_,
                               obstacles_by_frame_);
    // AINFO << "obstacles_by_frame_->Items().size(): "
    //       << obstacles_by_frame_->Items().size();
    // DebugStaticObsWithHistory(frame, obstacles_by_frame_);
    // AINFO << "StableStaticObsWithHistory_End";
  }

  if (!FormulateBoundaryConstraints(roi_boundary, frame)) {
    const std::string msg = "Fail to formulate boundary constraints";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (FLAGS_enable_not_use_map_as_boundary) {
    GenerateXYbounds(frame, &roi_boundary);
  }

  block_shift_l_ = 0.0;
  block_shift_s_ = 0.0;
  block_shift_calc_ = false;
  start_collision_ = false;
  if (TEBRoiDeciderConfig::RESCUE == roi_type) {
    if (!is_pullover_ready_) {
      CheckAdcIsBlocked(frame);
      if (start_collision_) {
        const std::string msg = "FAIL For Collsion within 0.1 buffer !!!";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
    if (!injector_->is_in_play_street && !is_in_city_road_) {
      goals_vector_.clear();
      const std::string msg =
          "lane_type != Lane::PLAY_STREET, cannot enter resuce at now ";
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // WYQ_Mark_Thread
    if (FLAGS_enable_teb_planner_thread && kTempProcessThreadFlages) {
      /*
      After this setting is set, the subsequent point selection cannot be
      changed, nor can the target point be cleared and set to the current frame
      */
      if (injector_->rescue_replan() && !goals_vector_.empty()) {
        /*
        Here, the target point remains unchanged. Although it is a relative
        coordinate, the origin point remains unchanged, so the absolute
        coordinates of the endpoint remain unchanged.
        */
        SetGoalToEndPose(frame);
        AINFO << "Need to plan and use historical target points";
      } else {
        injector_->set_rescue_replan(false);
        if (!rescue_status_->is_select_back_pose()) {
          AINFO << "forward select end pose.";
          double start = Clock::NowInSeconds();
          SetRescueEndPoseThread(frame);
          double diff = Clock::NowInSeconds() - start;
          AINFO << "end pose consumes: " << diff;
        } else {
          AINFO << "backward select end pose.";
          SetRescueBackEndPoseThread(frame);
        }
      }
    } else {
      if (is_in_city_road_) {
        SetRescueBackEndPose(frame);
      } else {
        if (!rescue_status_->is_select_back_pose()) {
          double start = Clock::NowInSeconds();
          SetRescueEndPose(frame);
          double diff = Clock::NowInSeconds() - start;
          ADEBUG << "end pose consumes: " << diff;
        } else {
          SetRescueBackEndPose(frame);
        }
      }
    }
  } else if (TEBRoiDeciderConfig::UTURN == roi_type) {
    // WYQ_Mark_Thread
    if (FLAGS_enable_teb_planner_thread && kTempProcessThreadFlages) {
      if (injector_->rescue_replan() && !goals_vector_.empty()) {
        SetGoalToEndPose(frame);
        AINFO << "UTurn mode. Need to plan and use historical target points";
      } else {
        injector_->set_rescue_replan(false);
        AINFO << "UTurn mode select end pose.";
        SetUturnEndPoseThread(frame);
      }
    } else {
      SetUturnEndPose(frame);
    }
  } else if (TEBRoiDeciderConfig::TRAFFIC_LIGHT == roi_type) {
    SetStopLineParkingEndPose(frame);
  } else {
    const std::string msg = "roi_type not defined";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (goals_vector_.empty() && TEBRoiDeciderConfig::TRAFFIC_LIGHT != roi_type &&
      TEBRoiDeciderConfig::UTURN != roi_type) {
    const std::string msg = "Fail to find a valid rescue end pose";
    AERROR << msg;
    // junction boundary is not large enough
    injector_->use_teb_default_bound_ = (is_in_junction_ && is_in_city_road_)
                                            ? !injector_->use_teb_default_bound_
                                            : true;
    // second enter, then select front
    if (fail_to_select_goal_ && is_in_city_road_ &&
        rescue_status_->is_select_back_pose()) {
      rescue_status_->set_is_select_back_pose(false);
    }
    fail_to_select_goal_ = true;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  fail_to_select_goal_ = false;
  injector_->set_rescue_crowded(injector_->is_personlike_blocked_);

  return Status::OK();
}

// set rescue origin pose
void TEBPlannerDecider::SetRescueOriginPose(Frame *const frame) {
  // get ADC box
  const auto rescue_status =
      injector_->planning_context()->planning_status().rescue();
  frame->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(rescue_status.adc_init_heading()));
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(
      rescue_status.adc_init_position().x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(
      rescue_status.adc_init_position().y());
}

// set common origin pose
void TEBPlannerDecider::SetCommonOriginPose(Frame *const frame) {
  // get ADC box
  const auto common_status =
      injector_->planning_context()->planning_status().teb_common();
  frame->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(common_status.adc_init_heading()));
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(
      common_status.adc_init_position().x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(
      common_status.adc_init_position().y());
}

void TEBPlannerDecider::OutRoadStartLogic(bool *const prefer_replan) {
  if (injector_->is_off_lane_depart_) {
    *prefer_replan = false;
    if (frame_->mutable_open_space_info()->destination_reached()) {
      injector_->is_off_lane_depart_ = false;
    }
  }
  return;
}

void TEBPlannerDecider::PullOverNearRoutingEndLogic(bool *const prefer_replan) {
  if (injector_->pullover_using_ && injector_->is_in_near_goal_) {
    *prefer_replan = false;
  }
  return;
}

// wyq_stop_long_time
bool TEBPlannerDecider::CheckVehicleStopLongTime(Frame *frame) {
  CheckData check_data;
  check_data.stamp = Clock::NowInSeconds();
  check_data.has_speed =
      std::fabs(frame->vehicle_state().linear_velocity()) > kStopSpeed;
  vehicle_speed_deque_.push_back(check_data);
  double time_diff = 0.0;
  while (true) {
    time_diff = (check_data.stamp - vehicle_speed_deque_.front().stamp);
    // AINFO << "time_diff" << time_diff;
    if (time_diff <= kCheckStopTimeWindows) {
      break;
    }
    vehicle_speed_deque_.pop_front();
  }

  bool long_time_stop = false;
  if (time_diff >= kCheckStopTimeMin && vehicle_speed_deque_.size() > 2) {
    int has_speed_count = 0;
    for (size_t i = 0; i < vehicle_speed_deque_.size(); ++i) {
      has_speed_count += vehicle_speed_deque_[i].has_speed ? 1 : 0;
    }
    long_time_stop = ((vehicle_speed_deque_.size() - has_speed_count) /
                      vehicle_speed_deque_.size()) > kStopLongTimeValidThr;
    if (long_time_stop) {
      vehicle_speed_deque_.clear();
      vehicle_speed_deque_.shrink_to_fit();
    }
  }
  return long_time_stop;
}

bool TEBPlannerDecider::CheckVehicleStopLongTime(
    Frame *frame, const double &stop_speed,
    const double &check_stop_time_windows, const double &check_stop_time_min,
    const double &stop_long_time_valid_thr) {
  CheckData check_data;
  check_data.stamp = Clock::NowInSeconds();
  check_data.has_speed =
      std::fabs(frame->vehicle_state().linear_velocity()) > stop_speed;
  // AINFO << "adc_speed: " << frame->vehicle_state().linear_velocity()
  //       << " time: " << check_data.stamp;
  vehicle_speed_deque_.push_back(check_data);
  double time_diff = 0.0;
  while (true) {
    time_diff = (check_data.stamp - vehicle_speed_deque_.front().stamp);
    // AINFO << "time_diff" << time_diff;
    if (time_diff <= check_stop_time_windows) {
      break;
    }
    vehicle_speed_deque_.pop_front();
  }

  bool long_time_stop = false;
  if (time_diff >= check_stop_time_min && vehicle_speed_deque_.size() > 2) {
    int has_speed_count = 0;
    for (size_t i = 0; i < vehicle_speed_deque_.size(); ++i) {
      has_speed_count += vehicle_speed_deque_[i].has_speed ? 1 : 0;
    }
    long_time_stop = ((vehicle_speed_deque_.size() - has_speed_count) /
                      vehicle_speed_deque_.size()) > stop_long_time_valid_thr;
  }
  return long_time_stop;
}

// wyq_rescue
// set rescue end pose
void TEBPlannerDecider::SetRescueEndPose(Frame *const frame) {
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();

  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "last frame is lane follow,clear goals";
    goals_vector_.clear();
  }

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();

  // WYQ_Mark_Replan Normal Rescue Forward
  bool prefer_replan = PreEndPoseReplayLogic(
      frame, previous_frame, is_pullover_ready_, is_in_city_road_);
  OutRoadStartLogic(&prefer_replan);
  PullOverNearRoutingEndLogic(&prefer_replan);
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  bool adc_slow_speed = std::fabs(adc_speed) < kSlowSpeed ? true : false;
  prefer_replan = prefer_replan && adc_slow_speed;

  bool fallback_replan = CalFallBackReplan();
  if (fallback_replan) {
    rescue_status_->set_is_first_init(true);
    goals_vector_.clear();
    AINFO << "Hard FallBackReplan, Init Teb ROI and End Point";
  }

  bool stop_long_time_replan = CheckVehicleStopLongTime(frame);
  stop_long_time_replan = false;
  // config_
  AINFO << "first_plan_failed:" << !is_first_plan_success
        << " goals_empty:" << goals_vector_.empty()
        << " prefer_replan:" << prefer_replan
        << " fallback_replan:" << fallback_replan
        << " stop_long_time_replan:" << stop_long_time_replan
        << " first_enable_pullover:" << injector_->first_into_pullover()
        << " first_enable_rescue:" << injector_->first_into_rescue();
  if (!is_first_plan_success || goals_vector_.empty() || prefer_replan ||
      injector_->first_into_rescue() || injector_->first_into_pullover()) {
    // get vehicle current location
    // get vehicle s,l info
    const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                              injector_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    const auto &reference_line_info = frame->reference_line_info().front();

    const auto &reference_line = reference_line_info.reference_line();
    reference_line.XYToSL(adc_position, &adc_position_sl);
    if (is_pullover_ready_) {
      GenerateSampleGoals(adc_position_sl, frame, reference_line);
    } else {
      if (injector_->shrink_end_s_) {
        AINFO << "shrink s for try second rescue";
        GenerateSampleGoals(
            adc_position_sl,
            config_.teb_roi_decider_config().goal_sample_start_s_second(),
            config_.teb_roi_decider_config().goal_sample_end_s_second(),
            config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
            reference_line);
      } else {
        GenerateSampleGoals(
            adc_position_sl,
            config_.teb_roi_decider_config().goal_sample_start_s(),
            config_.teb_roi_decider_config().goal_sample_end_s(),
            config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
            reference_line);
      }
    }
    injector_->set_rescue_replan(true);
  } else {
    AINFO << "no need refresh goals";
    if (stop_long_time_replan && FLAGS_enable_rescue_replan_reason1 &&
        !is_pullover_ready_) {
      goals_vector_.clear();
      rescue_status_->set_is_first_init(true);
      injector_->set_rescue_replan(true);
      AINFO << "stop too long ,refresh goals";
      return;
    }
  }

  // ADEBUG << "after goals_vector_ size " << goals_vector_.size();

  // not find
  NoFindGoalsProcess();

  SetGoalToEndPose(frame);
}

void TEBPlannerDecider::GenerateSampleGoals(
    const common::SLPoint &start_point, const Frame *frame,
    const ReferenceLine &reference_line) {
  static size_t fail_count = 0;
  fail_count = fail_to_select_goal_ ? fail_count + 1 : 0;

  const auto *previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool pre_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto &gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;
  bool pre_end_pose_check_valid =
      gear != canbus::Chassis::GEAR_REVERSE && pre_plan_success;
  const auto &end_pose =
      previous_frame->open_space_info().open_space_end_pose();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d end_pose_x_y(0.0, 0.0);

  if (pre_end_pose_check_valid && !end_pose.empty()) {
    end_pose_x_y.set_x(end_pose[0]);
    end_pose_x_y.set_y(end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;
  }

  goals_vector_.clear();

  const double half_adc_width =
      0.5 * (vehicle_params_.width() + FLAGS_astar_first_lat_buffer);
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  common::SLPoint adc_position_sl;
  common::SLPoint routing_end_position_sl;
  const auto &routing_end_point = frame_->local_view()
                                      .routing->routing_request()
                                      .waypoint()
                                      .rbegin()
                                      ->pose();
  reference_line.XYToSL(adc_position, &adc_position_sl);
  reference_line.XYToSL(routing_end_point, &routing_end_position_sl);

  double end_bound_s = 0.0;
  double start_bound_s = 0.0;
  end_bound_s = start_point.s() + routing_end_position_sl.s() -
                adc_position_sl.s() + (kPulloverSearchExtraS * 0.5);
  start_bound_s = start_point.s() + routing_end_position_sl.s() -
                  adc_position_sl.s() - 2.0 * kPulloverSearchExtraS - kEp;
  double sample_interval_s = kPullOverSampleIntervalS;
  double prefer_l = 0.0;
  double prefer_s = routing_end_position_sl.s();
  // expected that suppresing actions of adc.
  if (injector_->shrink_end_s_) {
    prefer_l = adc_position_sl.l();
  }

  PointENU ref_line_point;
  double ref_line_point_s = 0.0;
  double ref_line_point_l = 0.0;
  LaneInfoConstPtr ref_line_point_lane;
  double ref_lane_left_l = 0.0;
  double ref_lane_right_l = 0.0;
  for (double s = start_bound_s; s < end_bound_s; s = s + sample_interval_s) {
    if (!reference_line.GetLaneWidth(s, &ref_lane_left_l, &ref_lane_right_l)) {
      AERROR << "reference_line GetLaneWidth failed.";
      continue;
    }
    AINFO << "ref_lane_left_l:" << ref_lane_left_l
          << " ref_lane_right_l:" << ref_lane_right_l;
    double width = ref_lane_left_l + ref_lane_right_l;
    ref_lane_right_l = -ref_lane_right_l;

    ref_line_point.set_x(reference_line.GetReferencePoint(s).x());
    ref_line_point.set_y(reference_line.GetReferencePoint(s).y());
    hdmap_->GetNearestLane(ref_line_point, &ref_line_point_lane,
                           &ref_line_point_s, &ref_line_point_l);
    if (nullptr == ref_line_point_lane.get()) {
      AERROR << "get lane failed with pos x: " << ref_line_point.x()
             << " y: " << ref_line_point.y();
    } else {
      bool try_borrow = (!ref_line_point_lane->lane()
                              .left_neighbor_forward_lane_id()
                              .empty() ||
                         !ref_line_point_lane->lane()
                              .left_neighbor_reverse_lane_id()
                              .empty()) &&
                        ((is_in_city_road_ && fail_count > kFailCount));
      AINFO << "try_borrow " << try_borrow << "fail_count" << fail_count
            << "fail_to_select_goal_ " << fail_to_select_goal_;
      if (((!ref_line_point_lane->lane()
                 .left_neighbor_reverse_lane_id()
                 .empty() ||
            !ref_line_point_lane->lane()
                 .left_neighbor_forward_lane_id()
                 .empty()) &&
           !is_in_city_road_) ||
          try_borrow) {
        ref_lane_left_l += std::max(0.0, width);
      } else {
        AINFO << "no neighbor lane"
              << "ref_lane_id: " << ref_line_point_lane->lane().id().id();
      }
    }

    double left_bound_l = ref_lane_left_l - half_adc_width;
    double right_bound_r = ref_lane_right_l + half_adc_width;
    double sample_interval_l = kPullOverSampleIntervalL;

    prefer_l = right_bound_r;
    AINFO << "prefer_l: " << prefer_l << "prefer_s: " << prefer_s;
    for (double l = right_bound_r; l <= left_bound_l; l += sample_interval_l) {
      common::SLPoint sl_point;
      sl_point.set_s(s);
      sl_point.set_l(l);
      Vec2d sample_point;
      if (!reference_line.SLToXY(sl_point, &sample_point)) {
        AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
        continue;
      }
      double ref_heading = reference_line.GetReferencePoint(s).heading();
      if (!CheckGoalIsValid(sample_point, ref_heading)) {
        AINFO << "l= " << l;
        continue;
      }
      double lat_cost =
          (std::fabs(l - prefer_l) / std::fabs(left_bound_l - prefer_l));
      double lon_cost =
          (std::fabs(s - prefer_s) / std::fabs(start_bound_s - prefer_s));
      double obstacle_dist_cost = 0.0;
      double front_blocked_obstacle_dist_cost = 0.0;
      double history_cost = 0.0;

      double dist = GetObstacleDist(sample_point, ref_heading);
      ADEBUG << "dist " << dist;
      if (dist < FLAGS_rescue_hybird_ingore_safe_distance) {
        continue;
      } else if (dist > FLAGS_rescue_hybird_ingore_distance) {
        obstacle_dist_cost = 0.0;
      } else {
        obstacle_dist_cost =
            (1.0 - (dist / FLAGS_rescue_hybird_ingore_distance));
      }

      // it cost the most time
      if (FLAGS_enable_rescue_surround_cost) {
        bool is_sample_adc_blocked_with_front_obstacle =
            CheckADCIsBlockedWithSurroundObstacles(sample_point, ref_heading,
                                                   frame, kCheckDist,
                                                   preset_lat_buffer_, 0.0);
        ADEBUG << "is_sample_adc_blocked_with_front_obstacle: "
               << is_sample_adc_blocked_with_front_obstacle;
        if (is_sample_adc_blocked_with_front_obstacle) {
          front_blocked_obstacle_dist_cost = kFrontBlockedCost;
        }
      }

      if (pre_end_pose_check_valid) {
        double pose_distance = std::hypot(sample_point.x() - end_pose_x_y.x(),
                                          sample_point.y() - end_pose_x_y.y());
        history_cost = std::max(0.0, pose_distance * kHisotryCost);
        ADEBUG << "history_cost" << history_cost;
      }
      double total_cost = lat_cost * kPullOverLatCostWeight +
                          lon_cost * kPullOverLonCostWeight +
                          obstacle_dist_cost * kPullOverObsCostWeight;

      AINFO << "cost " << total_cost << " lat" << lat_cost << " lon" << lon_cost
            << " obs" << obstacle_dist_cost << " fobs"
            << front_blocked_obstacle_dist_cost << "same_history"
            << history_cost;

      PointWithCost temp_point = {sample_point, ref_heading, total_cost};
      goals_vector_.emplace_back(temp_point);
    }
  }
  std::sort(goals_vector_.begin(), goals_vector_.end(),
            [](const PointWithCost &ref_line_point_lane_left_l,
               const PointWithCost &ref_line_point_lane_right_l) {
              return ref_line_point_lane_left_l.cost >
                     ref_line_point_lane_right_l.cost;
            });
}

bool TEBPlannerDecider::PreEndPoseReplayLogic(
    Frame *const frame, const century::planning::Frame *previous_frame,
    const bool &is_pullover_ready, const bool &is_in_city_road) {
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  const auto &gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;

  bool pre_end_pose_check_valid =
      gear != canbus::Chassis::GEAR_REVERSE && is_first_plan_success;
  const auto &pre_end_pose =
      previous_frame->open_space_info().open_space_end_pose();
  Vec2d end_pose_x_y(0.0, 0.0);

  bool prefer_replan = false;
  if (pre_end_pose_check_valid && !pre_end_pose.empty() && !is_pullover_ready) {
    end_pose_x_y.set_x(pre_end_pose[0]);
    end_pose_x_y.set_y(pre_end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;

    double dx = end_pose_x_y.x() - adc_position.x();
    double dy = end_pose_x_y.y() - adc_position.y();
    double ds = std::hypot(dx, dy);
    AINFO << "ds = " << ds;

    double end_replan_dis =
        is_in_city_road_
            ? config_.teb_roi_decider_config().public_end_replan_dis()
            : config_.teb_roi_decider_config().park_end_replan_dis();
    prefer_replan = ds < end_replan_dis;
    AINFO << "PreEndPoseReplayLogic: prefer_replan " << prefer_replan;
  }
  return prefer_replan;
}

// wyq_fallback_replan
bool TEBPlannerDecider::CalFallBackReplan() {
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  CheckData check_data;
  check_data.stamp = Clock::NowInSeconds();
  check_data.has_fallback =
      (previous_frame->open_space_info().fallback_flag() &&
       !previous_frame->open_space_info().is_blocked_by_dynamic_obj());
  fallback_deque_.push_back(check_data);
  double time_diff = 0.0;
  while (true) {
    time_diff = (check_data.stamp - fallback_deque_.front().stamp);
    // AINFO << "time_diff" << time_diff;
    if (time_diff <= kCheckFallBackTimeWindows) {
      break;
    }
    fallback_deque_.pop_front();
  }

  bool fall_back_replan = false;
  if (time_diff >= kCheckFallBackTimeMin && fallback_deque_.size() > 2) {
    int has_fallback_count = 0;
    for (size_t i = 0; i < fallback_deque_.size(); ++i) {
      has_fallback_count += fallback_deque_[i].has_fallback ? 1 : 0;
    }
    fall_back_replan =
        (has_fallback_count / fallback_deque_.size()) > kFallBackValidThr;
    if (fall_back_replan) {
      vehicle_speed_deque_.clear();
      vehicle_speed_deque_.shrink_to_fit();
    }
  }
  return fall_back_replan;
}

void TEBPlannerDecider::NoFindGoalsProcess() {
  if (goals_vector_.empty()) {
    if (is_pullover_ready_) {
      AERROR << "find pull_over goal error";
    } else {
      AERROR << "find goal error";
    }
    injector_->use_teb_default_bound_ = true;
    return;
  }
  return;
}

bool TEBPlannerDecider::CheckFallbackReplan() {
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  // Only Fallback caused by non dynamic vehicle obstacles are counted
  if (previous_frame->open_space_info().fallback_flag() &&
      std::fabs(adc_speed) < kStopSpeed &&
      !previous_frame->open_space_info().is_blocked_by_dynamic_obj()) {
    ++fallback_replan_count_;
  } else {
    fallback_replan_count_ -= 2;
    fallback_replan_count_ = std::max(fallback_replan_count_, 0);
  }

  // Too many Fallbacks, clear target points, trigger replanning logic
  if (fallback_replan_count_ >
          config_.teb_roi_decider_config().replan_count() &&
      FLAGS_enable_rescue_replan_reason1) {
    fallback_replan_count_ = 0;
    AINFO
        << "Too many Fallbacks, clear target points, trigger replanning logic";
    goals_vector_.clear();
    return true;
  }
  return false;
}

bool TEBPlannerDecider::CheckPreferReplan(Frame *const frame) {
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  const auto &gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;

  /*
  In the case of being very close to the target point (within 3 meters) and
  in a forward gear, trigger replanning logic
  */
  bool pre_end_pose_check_valid =
      gear != canbus::Chassis::GEAR_REVERSE && is_first_plan_success;
  const auto &pre_end_pose =
      previous_frame->open_space_info().open_space_end_pose();

  bool prefer_replan = false;
  if (pre_end_pose_check_valid && !pre_end_pose.empty() &&
      !is_pullover_ready_) {
    // Convert to world coordinates
    Vec2d end_pose_x_y(0.0, 0.0);
    end_pose_x_y.set_x(pre_end_pose[0]);
    end_pose_x_y.set_y(pre_end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;

    double dx = end_pose_x_y.x() - adc_init_x;
    double dy = end_pose_x_y.y() - adc_init_y;
    double ds = std::hypot(dx, dy);
    prefer_replan = ds < kReplanDist;
    AINFO << "close to the target point (within 3 meters), dis is " << ds;
    AINFO << "CheckPreferReplan: prefer_replan " << prefer_replan;
  }

  return prefer_replan;
}

bool TEBPlannerDecider::CheckStopLongReplan() {
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  // Only Stop caused by non dynamic vehicle obstacles are counted
  if (std::fabs(adc_speed) < kStopSpeed &&
      !previous_frame->open_space_info().is_blocked_by_dynamic_obj()) {
    ++stop_long_count_;
  } else {
    stop_long_count_ = 0;
  }
  // Too many Stop long time, clear target points, trigger replanning logic
  if (stop_long_count_ > config_.teb_roi_decider_config().stop_long_count() &&
      FLAGS_enable_rescue_replan_reason1 && !is_pullover_ready_) {
    AINFO << "stop too long ,refresh goals. stop_long_count_ is "
          << stop_long_count_;
    goals_vector_.clear();
    stop_long_count_ = 0;
    // This will affect whether to choose forward or reverse
    rescue_status_->set_is_first_init(true);
    return true;
  }
  return false;
}

// set rescue end pose
void TEBPlannerDecider::SetRescueEndPoseThread(Frame *const frame) {
  // get vehicle current location
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  // get vehicle s,l info
  common::SLPoint adc_position_sl;
  // get nearest reference line
  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  AINFO << "fallback_replan_count_ " << fallback_replan_count_
        << " ,fallback_flag "
        << previous_frame->open_space_info().fallback_flag() << " ,adc_speed "
        << adc_speed;
  if (CheckFallbackReplan()) {
    return;
  }

  // Just come from lane_follow, clear target points
  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "last frame is lane follow, clear goals.";
    goals_vector_.clear();
  }

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  AINFO << "before goals_vector_ size " << goals_vector_.size()
        << " , and is_first_plan_success " << is_first_plan_success;

  // WYQ_Mark_Replan Thread Rescue Forward
  bool prefer_replan = CheckPreferReplan(frame);

  // config_
  AINFO << "first_plan_failed:" << !is_first_plan_success
        << " goals_empty:" << goals_vector_.empty()
        << " prefer_replan:" << prefer_replan
        << " first_enable_pullover:" << injector_->first_into_pullover()
        << " first_enable_rescue:" << injector_->first_into_rescue();
  /*
  Replanning conditions: Last planning failed (including just entering)
  or the target point is empty
  or near the target point
  */
  if (!is_first_plan_success || goals_vector_.empty() || prefer_replan ||
      injector_->first_into_rescue() || injector_->first_into_pullover()) {
    AINFO << "go to select end pose point...";
    if (injector_->shrink_end_s_) {
      AINFO << "shrink s for try second rescue";
      GenerateSampleGoals(
          adc_position_sl,
          config_.teb_roi_decider_config().goal_sample_start_s_second(),
          config_.teb_roi_decider_config().goal_sample_end_s_second(),
          config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
          reference_line);
    } else {
      GenerateSampleGoals(
          adc_position_sl,
          config_.teb_roi_decider_config().goal_sample_start_s(),
          config_.teb_roi_decider_config().goal_sample_end_s(),
          config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
          reference_line);
    }

    ready_to_replan_ = true;
  } else {
    AINFO << "no need refresh goals";
    if (CheckStopLongReplan()) {
      return;
    }
  }

  AINFO << "after goals_vector_ size " << goals_vector_.size();

  // not find
  if (goals_vector_.empty()) {
    if (is_pullover_ready_) {
      AERROR << "find pull_over goal error.";
    } else {
      AERROR << "find goal error.";
    }
    injector_->use_teb_default_bound_ = true;
    ready_to_replan_ = false;
    return;
  }

  if (ready_to_replan_) {
    ready_to_replan_ = false;
    injector_->set_rescue_replan(true);
    fallback_replan_count_ = 0;
    stop_long_count_ = 0;
    AINFO << "Now is need to replanning. goals size is "
          << goals_vector_.size();
  }

  SetGoalToEndPose(frame);
}

// wyq_uturn
// set uturn end pose
void TEBPlannerDecider::SetUturnEndPose(Frame *const frame) {
  // check is play_street
  if (!injector_->is_in_play_street) {
    goals_vector_.clear();
    AERROR << "lane_type != Lane::PLAY_STREET, cannot enter resuce at now ";
    return;
  }
  // get vehicle current location
  // get vehicle s,l info
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  common::SLPoint adc_position_sl;
  // get nearest reference line
  const auto &reference_line_info = frame->reference_line_info().front();

  // ADEBUG << "reference_line ID: " << reference_line_info.Lanes().Id();
  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();

  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "last frame is lane follow,clear goals";
    goals_vector_.clear();
  }

  bool fallback_replan = CalFallBackReplan();
  if (fallback_replan) {
    rescue_status_->set_is_first_init(true);
    AINFO << "Hard FallBackReplan, Init Teb ROI and End Point";
  }

  bool stop_long_time_replan =
      CheckVehicleStopLongTime(frame, kStopSpeed, kUturnCheckStopTimeWindows,
                               kUturnCheckStopTimeMin, kStopLongTimeValidThr);
  if (stop_long_time_replan) {
    vehicle_speed_deque_.clear();
    vehicle_speed_deque_.shrink_to_fit();
  }
  AINFO << "first_plan_failed:" << !is_first_plan_success
        << " goals_empty:" << goals_vector_.empty()
        << " fallback_replan:" << fallback_replan
        << " stop_long_time_replan:" << stop_long_time_replan
        << " first_enable_rescue:" << injector_->first_into_rescue();

  if (!is_first_plan_success || goals_vector_.empty() || fallback_replan) {
    GenerateSampleGoals(
        adc_position_sl, config_.teb_roi_decider_config().goal_sample_start_s(),
        config_.teb_roi_decider_config().goal_sample_end_s(),
        config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
        reference_line);
    injector_->set_rescue_replan(true);
  } else {
    AINFO << "no need refresh goals";
    if (stop_long_time_replan && FLAGS_enable_rescue_replan_reason1) {
      goals_vector_.clear();
      rescue_status_->set_is_first_init(true);
      injector_->set_rescue_replan(true);
      AINFO << "stop too long ,refresh goals";
      return;
    }
  }

  ADEBUG << "after goals_vector_ size " << goals_vector_.size();

  // not find
  if (goals_vector_.empty()) {
    AERROR << "find goal error";
    injector_->use_teb_default_bound_ = true;
    return;
  }

  SetGoalToEndPose(frame);
}

// set uturn end pose
void TEBPlannerDecider::SetUturnEndPoseThread(Frame *const frame) {
  // get vehicle current location
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  // get vehicle s,l info
  common::SLPoint adc_position_sl;
  // get nearest reference line
  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  bool is_city_drive = !injector_->is_in_play_street;
  if (is_city_drive) {
    goals_vector_.clear();
    AERROR << "lane_type != Lane::PLAY_STREET, cannot enter resuce at now ";
    return;
  }

  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  AINFO << "UTurn fallback_replan_count_ " << fallback_replan_count_
        << " ,fallback_flag "
        << previous_frame->open_space_info().fallback_flag() << " ,adc_speed "
        << adc_speed;
  if (CheckFallbackReplan()) {
    return;
  }

  // Just come from lane_follow, clear target points
  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "UTurn last frame is lane follow, clear goals";
    goals_vector_.clear();
  }

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  AINFO << "before goals_vector_ size " << goals_vector_.size()
        << " , and is_first_plan_success " << is_first_plan_success;

  /*
  Replanning conditions: Last planning failed (including just entering)
  or the target point is empty
  */
  if (!is_first_plan_success || goals_vector_.empty()) {
    AINFO << "UTurn go to select end pose point...";
    GenerateSampleGoals(
        adc_position_sl, config_.teb_roi_decider_config().goal_sample_start_s(),
        config_.teb_roi_decider_config().goal_sample_end_s(),
        config_.teb_roi_decider_config().goal_sample_interval_s(), frame,
        reference_line);
    // WYQ_Mark_Replan Thread Uturn
    ready_to_replan_ = true;
  } else {
    // Only Stop caused by non dynamic vehicle obstacles are counted
    AINFO << "UTurn no need refresh goals";
    if (std::fabs(adc_speed) < kStopSpeed &&
        !previous_frame->open_space_info().is_blocked_by_dynamic_obj()) {
      stop_long_count_++;
    } else {
      stop_long_count_ = 0;
    }
    if (stop_long_count_ > config_.teb_roi_decider_config().stop_long_count() &&
        FLAGS_enable_rescue_replan_reason1) {
      AERROR << "UTurn stop too long ,refresh goals. stop_long_count_ is "
             << stop_long_count_;
      goals_vector_.clear();
      stop_long_count_ = 0;
      return;
    }
  }

  AINFO << "after goals_vector_ size " << goals_vector_.size();

  // not find
  if (goals_vector_.empty()) {
    AERROR << "UTurn find goal error.";
    injector_->use_teb_default_bound_ = true;
    ready_to_replan_ = false;
    return;
  }

  if (ready_to_replan_) {
    ready_to_replan_ = false;
    injector_->set_rescue_replan(true);
    fallback_replan_count_ = 0;
    stop_long_count_ = 0;
    rescue_status_->set_is_select_back_pose(false);
    AINFO << "UTurn Now is need to replanning. goals size is "
          << goals_vector_.size();
  }

  SetGoalToEndPose(frame);
}

//---------------------------------------------------------------
void TEBPlannerDecider::SetRescueBackEndPose(Frame *const frame) {
  // get vehicle current location
  // get vehicle s,l info
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  common::SLPoint adc_position_sl;
  // get nearest reference line

  const auto &reference_line_info = frame->reference_line_info().front();

  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  const auto *previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  // Calculate the minimum turning radius
  double min_stop_distance = std::numeric_limits<double>::max();
  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    min_stop_distance =
        std::min(obstacle->MinRadiusStopDistance(vehicle_params_,
                                                 vehicle_params_.width() * 0.5),
                 min_stop_distance);
  }

  bool fallback_replan = CalFallBackReplan();
  if (fallback_replan) {
    rescue_status_->set_is_first_init(true);
    goals_vector_.clear();
    AINFO << "Hard FallBackReplan, Init Teb ROI and End Point";
  }

  bool stop_long_time_replan = CheckVehicleStopLongTime(frame);

  AINFO << "back first_plan_failed:" << !is_first_plan_success
        << " goals_vector_.empty():" << goals_vector_.empty()
        << " fallback_replan:" << fallback_replan
        << " stop_long_time_replan:" << stop_long_time_replan;
  if (!is_first_plan_success || goals_vector_.empty() || fallback_replan) {
    // double back_distance = rescue_status_->back_pose_distance();
    double back_distance =
        -1.0 * std::max(CalcBackDistance(block_shift_l_, block_shift_s_,
                                         min_stop_distance),
                        kMinBackDis);
    AINFO << "back_distance: " << back_distance;
    if (injector_->deal_start_block_) {
      back_distance = rescue_status_->back_pose_distance_little();
      AINFO << "back distance little" << back_distance;
    }
    GenerateBackSampleGoals(adc_position_sl, back_distance, frame,
                            reference_line);
  } else {
    if (stop_long_time_replan && FLAGS_enable_rescue_replan_reason1) {
      goals_vector_.clear();
      rescue_status_->set_is_first_init(true);
      AINFO << "back stop too long ,refresh goals";
      return;
    }
  }
  // not find
  if (goals_vector_.empty()) {
    return;
  }

  SetGoalToEndPose(frame);
}

void TEBPlannerDecider::SetRescueBackEndPoseThread(Frame *const frame) {
  // get vehicle current location
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  // get vehicle s,l info
  common::SLPoint adc_position_sl;
  // get nearest reference line
  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  const auto *previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  // Just come from lane_follow, clear target points
  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "Back last frame is lane follow, clear goals.";
    goals_vector_.clear();
  }
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  AINFO << "Back before goals_vector_ size " << goals_vector_.size()
        << " , and is_first_plan_success " << is_first_plan_success
        << " , adc_speed " << adc_speed;
  if (!is_first_plan_success || goals_vector_.empty()) {
    double back_distance = rescue_status_->back_pose_distance();
    if (injector_->deal_start_block_) {
      back_distance = rescue_status_->back_pose_distance_little();
      AINFO << "back distance little" << back_distance;
    }
    GenerateBackSampleGoals(adc_position_sl, back_distance, frame,
                            reference_line);
    stop_long_count_ = 0;
    // WYQ_Mark_Replan Thread Rescue Back
    ready_to_replan_ = true;
  } else {
    AINFO << "Back no need refresh goals.";
    // Only Stop caused by non dynamic vehicle obstacles are counted
    if (std::fabs(adc_speed) < kStopSpeed &&
        !previous_frame->open_space_info().is_blocked_by_dynamic_obj()) {
      ++stop_long_count_;
    } else {
      stop_long_count_ = 0;
    }
    // Too many Stop long time, clear target points, trigger replanning logic
    if (stop_long_count_ > config_.teb_roi_decider_config().stop_long_count() &&
        FLAGS_enable_rescue_replan_reason1) {
      AERROR << "Back stop too long ,refresh goals. stop_long_count_ is "
             << stop_long_count_;
      goals_vector_.clear();
      stop_long_count_ = 0;
      // This will affect whether to choose forward or reverse
      rescue_status_->set_is_first_init(true);
      return;
    }
  }

  // not find
  if (goals_vector_.empty()) {
    ready_to_replan_ = false;
    AERROR << "Back find goal error.";
    return;
  }

  if (ready_to_replan_) {
    ready_to_replan_ = false;
    injector_->set_rescue_replan(true);
    fallback_replan_count_ = 0;
    stop_long_count_ = 0;
    AINFO << "Back Now is need to replanning. goals size is "
          << goals_vector_.size();
  }

  SetGoalToEndPose(frame);
}

// delete
// void TEBPlannerDecider::GetRoadBoundary(
//     const hdmap::Path &nearby_path, const double center_line_s,
//     const common::math::Vec2d &origin_point, const double origin_heading,
//     std::vector<Vec2d> *left_lane_boundary,
//     std::vector<Vec2d> *right_lane_boundary,
//     std::vector<Vec2d> *center_lane_boundary_left,
//     std::vector<Vec2d> *center_lane_boundary_right,
//     std::vector<double> *center_lane_s_left,
//     std::vector<double> *center_lane_s_right,
//     std::vector<double> *left_lane_road_width,
//     std::vector<double> *right_lane_road_width) {
//   double start_s =
//       center_line_s -
//       config_.teb_roi_decider_config().roi_longitudinal_range_start();
//   double end_s = center_line_s +
//                  config_.teb_roi_decider_config().roi_longitudinal_range_end();

//   hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
//   double last_check_point_heading = start_point.heading();
//   double index = 0.0;
//   double check_point_s = start_s;

//   // For the road boundary, add key points to left/right side boundary
//   // separately. Iterate s_value to check key points at a step of
//   // roi_line_segment_length. Key points include: start_point, end_point,
//   // points where path curvature is large, points near left/right road-curb
//   // corners
//   while (check_point_s <= end_s) {
//     hdmap::MapPathPoint check_point =
//     nearby_path.GetSmoothPoint(check_point_s); double check_point_heading =
//     check_point.heading(); bool is_center_lane_heading_change =
//         std::abs(common::math::NormalizeAngle(check_point_heading -
//                                               last_check_point_heading)) >
//         config_.teb_roi_decider_config().roi_line_segment_min_angle();
//     last_check_point_heading = check_point_heading;

//     ADEBUG << "is is_center_lane_heading_change: "
//            << is_center_lane_heading_change;
//     // Check if the current center-lane checking-point is start point || end
//     // point || or point with larger curvature. If yes, mark it as an anchor
//     // point.
//     bool is_anchor_point = check_point_s == start_s || check_point_s == end_s
//     ||
//                            is_center_lane_heading_change;
//     // Add key points to the left-half boundary
//     AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
//                         is_anchor_point, true, center_lane_boundary_left,
//                         left_lane_boundary, center_lane_s_left,
//                         left_lane_road_width);
//     // Add key points to the right-half boundary
//     AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
//                         is_anchor_point, false, center_lane_boundary_right,
//                         right_lane_boundary, center_lane_s_right,
//                         right_lane_road_width);
//     ADEBUG << "is check_point_s: " << check_point_s
//            << " is start_s: " << start_s << " is end_s: " << end_s
//            << " left_lane_road_width " << left_lane_road_width->back()
//            << " right_lane_road_width " << right_lane_road_width->back();
//     if (check_point_s == end_s) {
//       break;
//     }
//     index += 1.0;
//     check_point_s =
//         start_s +
//         index * config_.teb_roi_decider_config().roi_line_segment_length();
//     check_point_s = check_point_s >= end_s ? end_s : check_point_s;
//   }

//   size_t left_point_size = left_lane_boundary->size();
//   size_t right_point_size = right_lane_boundary->size();
//   for (size_t i = 0; i < left_point_size; ++i) {
//     left_lane_boundary->at(i) -= origin_point;
//     left_lane_boundary->at(i).SelfRotate(-origin_heading);
//   }
//   for (size_t i = 0; i < right_point_size; ++i) {
//     right_lane_boundary->at(i) -= origin_point;
//     right_lane_boundary->at(i).SelfRotate(-origin_heading);
//   }
// }

// delete
// bool TEBPlannerDecider::GetRoadBoundaryFromMap(
//     const hdmap::Path &nearby_path, const double center_line_s,
//     const Vec2d &origin_point, const double origin_heading,
//     std::vector<Vec2d> *left_lane_boundary,
//     std::vector<Vec2d> *right_lane_boundary,
//     std::vector<Vec2d> *center_lane_boundary_left,
//     std::vector<Vec2d> *center_lane_boundary_right,
//     std::vector<double> *center_lane_s_left,
//     std::vector<double> *center_lane_s_right,
//     std::vector<double> *left_lane_road_width,
//     std::vector<double> *right_lane_road_width) {
//   // Longitudinal range can be asymmetric.
//   double start_s =
//       center_line_s -
//       config_.teb_roi_decider_config().roi_longitudinal_range_start();
//   double end_s = center_line_s +
//                  config_.teb_roi_decider_config().roi_longitudinal_range_end();

//   double check_dist = kIngoreRoadBoundaryDist;

//   double check_point_s = start_s;
//   hdmap::MapPathPoint center_point =
//   nearby_path.GetSmoothPoint(center_line_s); while (check_point_s <= end_s) {
//     hdmap::MapPathPoint check_point =
//     nearby_path.GetSmoothPoint(check_point_s);

//     // get road boundaries
//     double left_road_width = nearby_path.GetRoadLeftWidth(check_point_s);
//     double right_road_width = nearby_path.GetRoadRightWidth(check_point_s);

//     // use the max left/road width as radius to get the road boundary – the
//     // search radius
//     double current_road_width = std::max(left_road_width, right_road_width);

//     ADEBUG << "left_road_width: " << left_road_width
//            << " right_road_width: " << right_road_width
//            << " current_road_width: " << current_road_width;

//     // get road boundaries at current location
//     common::PointENU check_point_xy;
//     std::vector<hdmap::RoadRoiPtr> road_boundaries;
//     std::vector<hdmap::JunctionInfoConstPtr> junctions;
//     check_point_xy.set_x(check_point.x());
//     check_point_xy.set_y(check_point.y());
//     int result = 0;
//     if (lane_ids_.empty() || !FLAGS_enable_use_ref_lane_roadboundary) {
//       result = hdmap_->GetRoadBoundaries(check_point_xy, kInParkingLot,
//                                          &road_boundaries, &junctions);
//     } else {
//       result =
//           hdmap_->GetRoadBoundaries(check_point_xy, kInParkingLot, lane_ids_,
//                                     &road_boundaries, &junctions);
//     }

//     // TODO(all): need to ensure get road boundary ok.
//     if (0 != result) {
//       AINFO << "GetRoadBoundaryFromMap:hdmap_->GetRoadBoundaries failed, with
//       "
//                "result is "
//             << result;
//       return false;
//     }

//     if (road_boundaries.size() < 1) {
//       AINFO << "GetRoadBoundaryFromMap:road_boundaries size is empty!";
//       return false;
//     }

//     // TODO(weihuashen): just get the lane boundary directly.
//     for (size_t i = 0;
//          i < (*road_boundaries.at(0)).left_boundary.line_points.size(); ++i)
//          {
//       Vec2d point =
//           Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
//                 (*road_boundaries.at(0)).left_boundary.line_points[i].y());
//       double dist = point.DistanceTo(Vec2d(center_point.x(),
//       center_point.y())); if (dist > check_dist) {
//         continue;
//       }
//       left_lane_boundary->emplace_back(std::move(point));
//     }
//     for (size_t i = 0;
//          i < (*road_boundaries.at(0)).right_boundary.line_points.size(); ++i)
//          {
//       Vec2d point =
//           Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
//                 (*road_boundaries.at(0)).right_boundary.line_points[i].y());
//       double dist = point.DistanceTo(Vec2d(center_point.x(),
//       center_point.y())); if (dist > check_dist) {
//         continue;
//       }
//       right_lane_boundary->emplace_back(std::move(point));
//     }

//     center_lane_boundary_right->emplace_back(check_point);
//     center_lane_boundary_left->emplace_back(check_point);
//     center_lane_s_left->emplace_back(check_point_s);
//     center_lane_s_right->emplace_back(check_point_s);
//     left_lane_road_width->emplace_back(left_road_width);
//     right_lane_road_width->emplace_back(right_road_width);

//     double roi_line_segment_length_from_map =
//         config_.teb_roi_decider_config().roi_line_segment_length_from_map();

//     check_point_s = check_point_s + roi_line_segment_length_from_map;
//     ADEBUG << "roi_line_segment_length_from_map: "
//            << roi_line_segment_length_from_map
//            << " check_point_s: " << check_point_s;
//   }

//   size_t left_point_size = left_lane_boundary->size();
//   size_t right_point_size = right_lane_boundary->size();
//   ADEBUG << "right_road_boundary size: " << right_lane_boundary->size();
//   ADEBUG << "left_road_boundary size: " << left_lane_boundary->size();

//   if (left_point_size < 2 || right_point_size < 2) {
//     AERROR << "GetRoadBoundaryFromMap:left_point_size or right_point_size is
//     "
//               "< 2!";
//     return false;
//   }

//   // Convert coordinates to center at the origin
//   for (size_t i = 0; i < left_point_size; ++i) {
//     left_lane_boundary->at(i) -= origin_point;
//     left_lane_boundary->at(i).SelfRotate(-origin_heading);
//     ADEBUG << "left_road_boundary: [" << std::setprecision(9)
//            << left_lane_boundary->at(i).x() << ", "
//            << left_lane_boundary->at(i).y() << "]";
//   }
//   for (size_t i = 0; i < right_point_size; ++i) {
//     right_lane_boundary->at(i) -= origin_point;
//     right_lane_boundary->at(i).SelfRotate(-origin_heading);
//     ADEBUG << "right_road_boundary: [" << std::setprecision(9)
//            << right_lane_boundary->at(i).x() << ", "
//            << right_lane_boundary->at(i).y() << "]";
//   }
//   // Sort coordinate points
//   if (!left_lane_boundary->empty()) {
//     sort(left_lane_boundary->begin(), left_lane_boundary->end(),
//          [](const Vec2d &first_pt, const Vec2d &second_pt) {
//            return first_pt.x() < second_pt.x() ||
//                   (first_pt.x() == second_pt.x() &&
//                    first_pt.y() < second_pt.y());
//          });
//     auto unique_end =
//         std::unique(left_lane_boundary->begin(), left_lane_boundary->end());
//     left_lane_boundary->erase(unique_end, left_lane_boundary->end());
//   }
//   if (!right_lane_boundary->empty()) {
//     sort(right_lane_boundary->begin(), right_lane_boundary->end(),
//          [](const Vec2d &first_pt, const Vec2d &second_pt) {
//            return first_pt.x() < second_pt.x() ||
//                   (first_pt.x() == second_pt.x() &&
//                    first_pt.y() < second_pt.y());
//          });
//     auto unique_end =
//         std::unique(right_lane_boundary->begin(),
//         right_lane_boundary->end());
//     right_lane_boundary->erase(unique_end, right_lane_boundary->end());
//   }
//   InterpolateBoundary(
//       kBoundaryInterval,
//       config_.teb_roi_decider_config().roi_line_segment_min_angle(),
//       left_lane_boundary);
//   InterpolateBoundary(
//       kBoundaryInterval,
//       config_.teb_roi_decider_config().roi_line_segment_min_angle(),
//       right_lane_boundary);

//   return true;
// }

// add smooth point for better road boudary in curve
void TEBPlannerDecider::AddBoundaryKeyPoint(
    const hdmap::Path &nearby_path, const double check_point_s,
    const double start_s, const double end_s, const bool is_anchor_point,
    const bool is_left_curb, std::vector<Vec2d> *center_lane_boundary,
    std::vector<Vec2d> *curb_lane_boundary, std::vector<double> *center_lane_s,
    std::vector<double> *road_width) {
  // Check if current central-lane checking point's mapping on the left/right
  // road boundary is a key point. The road boundary point is a key point if
  // one of the following two confitions is satisfied:
  // 1. the current central-lane point is an anchor point: (a start/end point
  // or the point on path with large curvatures)
  // 2. the point on the left/right lane boundary is close to a curb corner
  // As indicated below:
  // (#) Key Point Type 1: Lane anchor points
  // (*) Key Point Type 2: Curb-corner points
  //                                                         #
  // Path Direction -->                                     /    /   #
  // Left Lane Boundary   #--------------------------------#    /   /
  //                                                           /   /
  // Center Lane          - - - - - - - - - - - - - - - - - - /   /
  //                                                             /
  // Right Lane Boundary  #--------*                 *----------#
  //                                \               /
  //                                 *-------------*

  // road width changes slightly at the turning point of a path
  // TODO(SHU): 1. consider distortion introduced by curvy road; 2. use both
  // round boundaries for single-track road; 3. longitudinal range may not be
  // symmetric
  CHECK_NE(check_point_s, start_s);
  CHECK_NE(check_point_s, end_s);
  const double previous_distance_s =
      std::min(config_.teb_roi_decider_config().roi_line_segment_length(),
               check_point_s - start_s);
  const double next_distance_s =
      std::min(config_.teb_roi_decider_config().roi_line_segment_length(),
               end_s - check_point_s);

  hdmap::MapPathPoint current_check_point =
      nearby_path.GetSmoothPoint(check_point_s);
  hdmap::MapPathPoint previous_check_point =
      nearby_path.GetSmoothPoint(check_point_s - previous_distance_s);
  hdmap::MapPathPoint next_check_point =
      nearby_path.GetSmoothPoint(check_point_s + next_distance_s);

  double current_check_point_heading = current_check_point.heading();
  double current_road_width =
      is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s)
                   : nearby_path.GetRoadRightWidth(check_point_s);
  ADEBUG << "current_road_width " << current_road_width;
  // If the current center-lane checking point is an anchor point, then add
  // current left/right curb boundary point as a key point
  if (is_anchor_point) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI * 0.5)
                     : std::cos(current_check_point_heading - M_PI * 0.5);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI * 0.5)
                     : std::sin(current_check_point_heading - M_PI * 0.5);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point = curb_lane_point + current_check_point;
    center_lane_boundary->push_back(current_check_point);
    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
    return;
  }
  double previous_road_width =
      is_left_curb
          ? nearby_path.GetRoadLeftWidth(check_point_s - previous_distance_s)
          : nearby_path.GetRoadRightWidth(check_point_s - previous_distance_s);
  double next_road_width =
      is_left_curb
          ? nearby_path.GetRoadLeftWidth(check_point_s + next_distance_s)
          : nearby_path.GetRoadRightWidth(check_point_s + next_distance_s);
  double previous_curb_segment_angle =
      (current_road_width - previous_road_width) / previous_distance_s;
  double next_segment_angle =
      (next_road_width - current_road_width) / next_distance_s;
  double current_curb_point_delta_theta =
      next_segment_angle - previous_curb_segment_angle;
  // If the delta angle between the previous curb segment and the next curb
  // segment is large (near a curb corner), then add current curb_lane_point
  // as a key point.
  if (std::abs(current_curb_point_delta_theta) >
      config_.teb_roi_decider_config()
          .curb_heading_tangent_change_upper_limit()) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI * 0.5)
                     : std::cos(current_check_point_heading - M_PI * 0.5);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI * 0.5)
                     : std::sin(current_check_point_heading - M_PI * 0.5);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point = curb_lane_point + current_check_point;
    center_lane_boundary->push_back(current_check_point);
    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
  }
}

bool TEBPlannerDecider::GetAcdOnLaneHalfWidth(
    common::SLPoint *const adc_position_sl,
    double *const current_road_left_line_l,
    double *const current_road_right_line_l) {
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_init_position, adc_position_sl);

  if (!reference_line.GetLaneWidth((*adc_position_sl).s(),
                                   current_road_left_line_l,
                                   current_road_right_line_l)) {
    AINFO << "adc_position_sl.s()" << (*adc_position_sl).s();
    AERROR << "reference_line GetLaneWidth failed.";
    return false;
  }
  AINFO << "current_road_left_line_l: " << (*current_road_left_line_l)
        << " | current_road_right_line_l: " << (*current_road_right_line_l);
  return true;
}

// WYQ_Maker
void TEBPlannerDecider::CalXYBoundary(
    const std::vector<common::math::Vec2d> &left_lane_boundary,
    const std::vector<common::math::Vec2d> &right_lane_boundary,
    Frame *const frame) {
  // Get xy boundary
  std::vector<Vec2d> boundary_points;
  std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
            std::back_inserter(boundary_points));
  std::copy(left_lane_boundary.begin(), left_lane_boundary.end(),
            std::back_inserter(boundary_points));

  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });

  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};

  ROI_xy_boundary_.clear();
  ROI_xy_boundary_.shrink_to_fit();
  ROI_xy_boundary_ = ROI_xy_boundary;

  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  AINFO << "ROI_boundary x_max:" << ROI_xy_boundary[0]
        << " x_min:" << ROI_xy_boundary[1] << " y_max:" << ROI_xy_boundary[2]
        << " y_min:" << ROI_xy_boundary[3];

  return;
}

void TEBPlannerDecider::CalOpenRoadTebRoiLeftAndRightPoint(
    Frame *const frame, const common::SLPoint &adc_position_sl,
    const double &current_road_left_line_l,
    const double &current_road_right_line_l,
    std::vector<common::math::Vec2d> *const left_lane_boundary,
    std::vector<common::math::Vec2d> *const right_lane_boundary) {
  start_roi_s_ = config_.teb_roi_decider_config().hd_map_boundary_start_s();
  end_roi_s_ = config_.teb_roi_decider_config().hd_map_boundary_end_s();
  double start_contract_s =
      config_.teb_roi_decider_config().hd_map_boundary_start_contract_s();
  double left_contract_ratio =
      config_.teb_roi_decider_config().hd_map_boundary_left_l_contract_ratio();
  double right_contract_ratio =
      config_.teb_roi_decider_config().hd_map_boundary_right_l_contract_ratio();

  left_roi_l_ = (2.0 - left_contract_ratio) * current_road_left_line_l +
                current_road_right_line_l;
  right_roi_l_ = -current_road_right_line_l;

  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::SLPoint left_sl_point = adc_position_sl;
  common::SLPoint right_sl_point = adc_position_sl;
  for (int i = start_roi_s_; i < end_roi_s_; ++i) {
    if (i > start_contract_s) {
      left_sl_point.set_l(left_roi_l_);
      right_sl_point.set_l(right_roi_l_ * (1.0 - right_contract_ratio));
    } else {
      left_sl_point.set_l(left_roi_l_);
      right_sl_point.set_l(right_roi_l_);
    }
    left_sl_point.set_s(adc_position_sl.s() + i);
    right_sl_point.set_s(adc_position_sl.s() + i);

    common::math::Vec2d left_point;
    common::math::Vec2d right_point;
    // AINFO << "left_sl_point: " << left_sl_point.DebugString();
    // AINFO << "right_sl_point: " << right_sl_point.DebugString();
    reference_line.SLToXY(left_sl_point, &left_point);
    reference_line.SLToXY(right_sl_point, &right_point);

    left_point -= origin_point;
    left_point.SelfRotate(-origin_heading);
    right_point -= origin_point;
    right_point.SelfRotate(-origin_heading);

    // AINFO << "left_point: " << left_point.DebugString();
    // AINFO << "right_point: " << right_point.DebugString();

    left_lane_boundary->emplace_back(left_point);
    right_lane_boundary->emplace_back(right_point);
  }
  return;
}

// wyq
bool TEBPlannerDecider::GenerateDefaultLaneBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  AINFO << "Teb_ROI: PubilcRoadHdMapFrenetBoundary";

  common::SLPoint adc_position_sl;
  double current_road_left_line_l = 0.0;
  double current_road_right_line_l = 0.0;
  if (!GetAcdOnLaneHalfWidth(&adc_position_sl, &current_road_left_line_l,
                             &current_road_right_line_l)) {
    return false;
  }

  std::vector<common::math::Vec2d> left_lane_boundary;
  std::vector<common::math::Vec2d> right_lane_boundary;
  CalOpenRoadTebRoiLeftAndRightPoint(
      frame, adc_position_sl, current_road_left_line_l,
      current_road_right_line_l, &left_lane_boundary, &right_lane_boundary);

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  CalXYBoundary(left_lane_boundary, right_lane_boundary, frame);

  return true;
}

// delete
// bool TEBPlannerDecider::GenerateNoHdMapBoundary(
//     Frame *const frame,
//     std::vector<std::vector<common::math::Vec2d>> *const
//     roi_parking_boundary) {
//   // get vehicle init position
//   const double adc_init_x = injector_->vehicle_state()->x();
//   const double adc_init_y = injector_->vehicle_state()->y();
//   common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};

//   const double roi_lon_forward_distance =
//       config_.teb_roi_decider_config().roi_lon_forward_distance();
//   const double roi_lon_backward_distance =
//       config_.teb_roi_decider_config().roi_lon_backward_distance();
//   const double roi_lat_left_distance =
//       config_.teb_roi_decider_config().roi_lat_left_distance();
//   const double roi_lat_right_distance =
//       config_.teb_roi_decider_config().roi_lat_right_distance();
//   const double boundary_width = roi_lat_left_distance +
//   roi_lat_right_distance; AINFO << "roi_lon_forward_distance " <<
//   roi_lon_forward_distance; AINFO << "roi_lon_backward_distance " <<
//   roi_lon_backward_distance; AINFO << "roi_lat_left_distance " <<
//   roi_lat_left_distance; AINFO << "roi_lat_right_distance " <<
//   roi_lat_right_distance;

//   AINFO << "boundary_width " << boundary_width;
//   AINFO << "adc_ref_heading_ " << adc_ref_heading_;

//   double adc_ref_heading = adc_ref_heading_;
//   const auto &roi_type = config_.teb_roi_decider_config().roi_type();

//   AINFO << "roi_type " << roi_type;

//   if (TEBRoiDeciderConfig::UTURN == roi_type) {
//     adc_ref_heading = common::math::NormalizeAngle(adc_ref_heading_ + M_PI);
//   }

//   AINFO << "adc_ref_heading 2" << adc_ref_heading;

//   // ADC box
//   Box2d box(adc_init_position, adc_ref_heading,
//             roi_lon_forward_distance + roi_lon_backward_distance,
//             boundary_width);
//   double lon_shift_distance =
//       (roi_lon_forward_distance - roi_lon_backward_distance) * 0.5;
//   double lat_shift_distance =
//       (roi_lat_left_distance - roi_lat_right_distance) * 0.5;
//   double lon_shift = lon_shift_distance * std::cos(adc_ref_heading) -
//                      lat_shift_distance * std::sin(adc_ref_heading);
//   double lat_shift = lon_shift_distance * std::sin(adc_ref_heading) +
//                      lat_shift_distance * std::cos(adc_ref_heading);
//   Vec2d shift_vec{lon_shift, lat_shift};
//   box.Shift(shift_vec);
//   // get vertices from ADC box
//   std::vector<common::math::Vec2d> corners;
//   box.GetAllCorners(&corners);
//   const auto &origin_heading = frame->open_space_info().origin_heading();
//   const auto &origin_point = frame->open_space_info().origin_point();

//   auto left_top = corners[3];
//   left_top -= origin_point;
//   left_top.SelfRotate(-origin_heading);
//   auto left_down = corners[0];
//   left_down -= origin_point;
//   left_down.SelfRotate(-origin_heading);
//   auto right_down = corners[1];
//   right_down -= origin_point;
//   right_down.SelfRotate(-origin_heading);
//   auto right_top = corners[2];
//   right_top -= origin_point;
//   right_top.SelfRotate(-origin_heading);

//   std::vector<Vec2d> left_lane_boundary;
//   std::vector<Vec2d> right_lane_boundary;
//   left_lane_boundary.emplace_back(left_top);
//   left_lane_boundary.emplace_back(left_down);
//   right_lane_boundary.emplace_back(right_down);
//   right_lane_boundary.emplace_back(right_top);

//   size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
//   for (size_t i = 0; i < left_lane_boundary_last_index; ++i) {
//     std::vector<Vec2d> segment{left_lane_boundary[i],
//                                left_lane_boundary[i + 1]};
//     roi_parking_boundary->emplace_back(segment);
//   }

//   if (TEBRoiDeciderConfig::UTURN == roi_type) {
//     std::vector<Vec2d> right_left_link_boundary;
//     right_left_link_boundary.emplace_back(left_down);
//     right_left_link_boundary.emplace_back(right_down);
//     size_t link_lane_boundary_last_index = right_left_link_boundary.size() -
//     1; for (size_t i = 0; i < link_lane_boundary_last_index; ++i) {
//       std::vector<Vec2d> segment{right_left_link_boundary[i],
//                                  right_left_link_boundary[i + 1]};
//       roi_parking_boundary->emplace_back(segment);
//     }
//   }

//   size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
//   for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
//     std::vector<Vec2d> segment{right_lane_boundary[i],
//                                right_lane_boundary[i + 1]};
//     roi_parking_boundary->emplace_back(segment);
//   }

//   return true;
// }

bool TEBPlannerDecider::GenerateXYbounds(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  const auto &open_space_info = frame->open_space_info();
  auto &obstacles_vertices_vec = open_space_info.obstacles_vertices_vec();

  size_t obstacles_vertices_vec_num = obstacles_vertices_vec.size();
  std::vector<Vec2d> vertices_points;
  for (size_t i = 0; i < obstacles_vertices_vec_num; ++i) {
    for (size_t j = 0; j < obstacles_vertices_vec[i].size(); ++j) {
      std::copy(obstacles_vertices_vec[i].begin(),
                obstacles_vertices_vec[i].end(),
                std::back_inserter(vertices_points));
    }
  }

  // Get xy boundary
  auto xminmax = std::minmax_element(
      vertices_points.begin(), vertices_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      vertices_points.begin(), vertices_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  AINFO << "ROI_xy_boundary x0 " << ROI_xy_boundary[0] << ", x1 "
        << ROI_xy_boundary[1] << ", y0 " << ROI_xy_boundary[2] << ", y1 "
        << ROI_xy_boundary[3];

  ROI_xy_boundary_.clear();
  ROI_xy_boundary_ = ROI_xy_boundary;

  return true;
}

// delete
// bool TEBPlannerDecider::GetRescueBoundary(
//     Frame *const frame, const hdmap::Path &nearby_path,
//     std::vector<std::vector<common::math::Vec2d>> *const
//     roi_parking_boundary) {
//   // get vehicle init position
//   const double adc_init_x = injector_->vehicle_state()->x();
//   const double adc_init_y = injector_->vehicle_state()->y();
//   const double adc_init_heading = injector_->vehicle_state()->heading();

//   common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
//   const double adc_length =
//       vehicle_params_.length() + FLAGS_astar_first_long_buffer;
//   const double adc_width =
//       vehicle_params_.width() + FLAGS_astar_first_lat_buffer;
//   // ADC box
//   Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
//   double shift_distance =
//       adc_length * 0.5 - vehicle_params_.back_edge_to_center();
//   Vec2d shift_vec{shift_distance * std::cos(adc_init_heading),
//                   shift_distance * std::sin(adc_init_heading)};
//   adc_box.Shift(shift_vec);
//   // get vertices from ADC box
//   std::vector<common::math::Vec2d> adc_corners;
//   adc_box.GetAllCorners(&adc_corners);
//   auto left_top = adc_corners[1];
//   auto right_top = adc_corners[0];

//   const auto &origin_point = frame->open_space_info().origin_point();
//   const auto &origin_heading = frame->open_space_info().origin_heading();

//   double left_top_s = 0.0;
//   double left_top_l = 0.0;
//   double right_top_s = 0.0;
//   double right_top_l = 0.0;
//   if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l) &&
//         nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
//     AERROR << "fail to get adc corners points' projections on reference
//     line"; return false;
//   }

//   std::vector<Vec2d> left_lane_boundary;
//   std::vector<Vec2d> right_lane_boundary;
//   const double center_line_s = (left_top_s + right_top_s) * 0.5;
//   std::vector<Vec2d> center_lane_boundary_left;
//   std::vector<Vec2d> center_lane_boundary_right;
//   std::vector<double> center_lane_s_left;
//   std::vector<double> center_lane_s_right;
//   std::vector<double> left_lane_road_width;
//   std::vector<double> right_lane_road_width;
//   ADEBUG << "center_line_s " << center_line_s;
//   // two method, road has no info at now
//   if (!FLAGS_enable_use_lane_as_boundary) {
//     if (FLAGS_use_road_boundary_from_map) {
//       if (!GetRoadBoundaryFromMap(
//               nearby_path, center_line_s, origin_point, origin_heading,
//               &left_lane_boundary, &right_lane_boundary,
//               &center_lane_boundary_left, &center_lane_boundary_right,
//               &center_lane_s_left, &center_lane_s_right,
//               &left_lane_road_width, &right_lane_road_width)) {
//         AERROR << "TEBPlannerDecider::GetRoadBoundaryFromMap Failed.";
//         return false;
//       }
//     } else {
//       GetRoadBoundary(nearby_path, center_line_s, origin_point,
//       origin_heading,
//                       &left_lane_boundary, &right_lane_boundary,
//                       &center_lane_boundary_left,
//                       &center_lane_boundary_right, &center_lane_s_left,
//                       &center_lane_s_right, &left_lane_road_width,
//                       &right_lane_road_width);
//     }
//   } else {
//     AINFO << "TEBPlannerDecider::GetRescueLaneBoundary";
//     // @brief Get rescue lane boundaries of both sides
//     GetRescueLaneBoundary(nearby_path, origin_point, origin_heading,
//                           &left_lane_boundary, &right_lane_boundary);
//   }

//   size_t left_point_size = left_lane_boundary.size();
//   size_t right_point_size = right_lane_boundary.size();
//   if (left_point_size < kMinLaneBoundaryPointSize ||
//       right_point_size < kMinLaneBoundaryPointSize) {
//     return false;
//   }

//   std::vector<Vec2d> boundary_points;
//   std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
//             std::back_inserter(boundary_points));
//   std::copy(left_lane_boundary.begin(), left_lane_boundary.end(),
//             std::back_inserter(boundary_points));

//   size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
//   for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
//     std::vector<Vec2d> segment{right_lane_boundary[i],
//                                right_lane_boundary[i + 1]};
//     ADEBUG << "right segment";
//     ADEBUG << "right_road_boundary: [" << std::setprecision(9)
//            << right_lane_boundary[i].x() << ", " <<
//            right_lane_boundary[i].y()
//            << "]";
//     ADEBUG << "right_road_boundary: [" << std::setprecision(9)
//            << right_lane_boundary[i + 1].x() << ", "
//            << right_lane_boundary[i + 1].y() << "]";
//     roi_parking_boundary->push_back(segment);
//   }

//   size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
//   for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
//     std::vector<Vec2d> segment{left_lane_boundary[i],
//                                left_lane_boundary[i - 1]};
//     roi_parking_boundary->push_back(segment);
//   }

//   ADEBUG << "before roi_parking_boundary size: ["
//          << roi_parking_boundary->size() << "]";

//   // Fuse line segments into convex contraints
//   if (!FuseLineSegments(roi_parking_boundary)) {
//     AERROR << "FuseLineSegments rescue boundary failed: ["
//            << roi_parking_boundary->size() << "]";
//     return false;
//   }

//   ADEBUG << "after roi_parking_boundary size: [" <<
//   roi_parking_boundary->size()
//          << "]";
//   // Get xy boundary
//   auto xminmax = std::minmax_element(
//       boundary_points.begin(), boundary_points.end(),
//       [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
//   auto yminmax = std::minmax_element(
//       boundary_points.begin(), boundary_points.end(),
//       [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
//   std::vector<double> ROI_xy_boundary{xminmax.first->x(),
//   xminmax.second->x(),
//                                       yminmax.first->y(),
//                                       yminmax.second->y()};
//   auto *xy_boundary =
//       frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
//   xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

//   ROI_xy_boundary_.clear();
//   ROI_xy_boundary_ = ROI_xy_boundary;

//   for (auto corner : adc_corners) {
//     corner -= origin_point;
//     corner.SelfRotate(-origin_heading);
//     if (corner.x() < ROI_xy_boundary_[0] || corner.x() > ROI_xy_boundary_[1]
//     ||
//         corner.y() < ROI_xy_boundary_[2] || corner.y() > ROI_xy_boundary_[3])
//         {
//       AERROR << "adc point is  outside of xy boundary of rescue ROI "
//              << "corner.x() " << corner.x() << " corner.y() " << corner.y()
//              << "x_min " << ROI_xy_boundary_[0] << " x_max "
//              << ROI_xy_boundary_[1] << " y_min " << ROI_xy_boundary_[2]
//              << " y_max " << ROI_xy_boundary_[3];
//       return false;
//     }
//   }
//   return true;
// }

bool TEBPlannerDecider::FuseLineSegments(
    std::vector<std::vector<common::math::Vec2d>> *line_segments_vec) {
  static constexpr double kEpsilon = 1.0e-8;
  auto cur_segment = line_segments_vec->begin();
  while (cur_segment != line_segments_vec->end() - 1) {
    auto next_segment = cur_segment + 1;
    auto cur_last_point = cur_segment->back();
    auto next_first_point = next_segment->front();
    // Check if they are the same points
    if (cur_last_point.DistanceTo(next_first_point) > kEpsilon) {
      ++cur_segment;
      continue;
    }
    if (cur_segment->size() < 2 || next_segment->size() < 2) {
      AERROR << "Single point line_segments vec not expected";
      return false;
    }
    size_t cur_segments_size = cur_segment->size();
    auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
    auto next_second_point = next_segment->at(1);
    // Straight or counterclockwise will not be erased
    if (CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) <
        0.0) {
      cur_segment->push_back(next_second_point);
      next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
      if (next_segment->empty()) {
        line_segments_vec->erase(next_segment);
      }
    } else {
      ++cur_segment;
    }
  }
  return true;
}

bool TEBPlannerDecider::FormulateBoundaryConstraints(
    const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  // Gather vertice needed by warm start and distance approach
  if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
    AERROR << "fail at LoadObstacleInVertices()";
    return false;
  } else {
    obstacles_linesegments_vec_.clear();

    const auto &obstacles_vertices_vec1 =
        frame->open_space_info().obstacles_vertices_vec();
    std::vector<std::vector<common::math::LineSegment2d>>
        obstacles_linesegments_vec;
    for (const auto &obstacle_vertices : obstacles_vertices_vec1) {
      size_t vertices_num = obstacle_vertices.size();
      std::vector<common::math::LineSegment2d> obstacle_linesegments;
      for (size_t i = 0; i < vertices_num - 1; ++i) {
        common::math::LineSegment2d line_segment = common::math::LineSegment2d(
            obstacle_vertices[i], obstacle_vertices[i + 1]);
        obstacle_linesegments.emplace_back(line_segment);
      }
      if (obstacle_linesegments.empty()) {
        continue;
      }
      obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  }
  // Transform vertices into the form of Ax>b
  if (!LoadObstacleInHyperPlanes(frame)) {
    AERROR << "fail at LoadObstacleInHyperPlanes()";
    return false;
  }
  return true;
}

bool TEBPlannerDecider::LoadObstacleInVertices(
    const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  auto *mutable_open_space_info = frame->mutable_open_space_info();
  auto *obstacles_vertices_vec =
      mutable_open_space_info->mutable_obstacles_vertices_vec();
  auto *pure_obstacles_vertices_vec =
      mutable_open_space_info->mutable_pure_obstacles_vertices_vec();
  auto *obstacles_edges_num_vec =
      mutable_open_space_info->mutable_obstacles_edges_num();

  // load vertices for parking boundary (not need to repeat the first
  // vertice to get close hull)
  size_t parking_boundaries_num = roi_parking_boundary.size();
  AINFO << "parking_boundaries_num " << parking_boundaries_num;
  size_t perception_obstacles_num = 0;

  size_t point_num = 0;
  std::vector<size_t> obstacle_points_num;
  obstacle_points_num.clear();

  auto *boundary_vertices_vec =
      mutable_open_space_info->mutable_boundary_vertices_vec();
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    obstacles_vertices_vec->emplace_back(roi_parking_boundary[i]);
    boundary_vertices_vec->emplace_back(roi_parking_boundary[i]);
  }

  Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num,
                                                         1);

  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    CHECK_GT(roi_parking_boundary[i].size(), 1U);
    parking_boundaries_obstacles_edges_num(i, 0) =
        static_cast<int>(roi_parking_boundary[i].size()) - 1;
  }

  bool enable_polygon_plan = FLAGS_enable_openspace_use_polygon_plan;
  ADEBUG << "config_.teb_roi_decider_config().enable_perception_obstacles() "
         << config_.teb_roi_decider_config().enable_perception_obstacles();
  obstacle_id_.clear();
  if (config_.teb_roi_decider_config().enable_perception_obstacles()) {
    // load vertices for perception obstacles(repeat the first vertice at
    // the last to form closed convex hull)

    CalculateFrameObstacles(frame, &perception_obstacles_num, &point_num,
                            obstacles_vertices_vec, pure_obstacles_vertices_vec,
                            &obstacle_points_num);

    if (FLAGS_enable_use_costmap) {
      EnableUseCostMap(frame, &point_num, &perception_obstacles_num,
                       &obstacle_points_num, obstacles_vertices_vec,
                       pure_obstacles_vertices_vec);
    }
    // obstacle boundary box is used, thus the edges are set to be 4
    // lwt: notice this

    Eigen::MatrixXi perception_obstacles_edges_num =
        Eigen::MatrixXi::Ones(perception_obstacles_num, 1);
    AINFO << "perception_obstacles_num " << perception_obstacles_num
          << "obstacle_points_num.size() " << obstacle_points_num.size()
          << "pure_obstacles_vertices_vec"
          << pure_obstacles_vertices_vec->size() << "obstacles_vertices_vec"
          << obstacles_vertices_vec->size();

    if (enable_polygon_plan) {
      ACHECK(obstacle_points_num.size() == perception_obstacles_num);
    }

    for (size_t i = 0; i < perception_obstacles_num; ++i) {
      perception_obstacles_edges_num(i, 0) =
          enable_polygon_plan ? obstacle_points_num[i] : 4;
    }

    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows() +
            perception_obstacles_edges_num.rows(),
        1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num,
        perception_obstacles_edges_num;

  } else {
    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows(), 1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num;
  }

  mutable_open_space_info->set_obstacles_num(parking_boundaries_num +
                                             perception_obstacles_num);
  return true;
}

void TEBPlannerDecider::DontUsePolygonPlan(
    century::planning::Frame *const frame, const Obstacle &obstacle,
    size_t *perception_obstacles_num,
    std::vector<std::vector<century::common::math::Vec2d>>
        *obstacles_vertices_vec,
    std::vector<std::vector<century::common::math::Vec2d>>
        *pure_obstacles_vertices_vec) {
  ++(*perception_obstacles_num);
  const auto &open_space_info = frame->open_space_info();
  const auto &origin_point = open_space_info.origin_point();
  const auto &origin_heading = open_space_info.origin_heading();
  Box2d original_box = obstacle.PerceptionBoundingBox();
  original_box.Shift(-1.0 * origin_point);

  if (!rescue_status_->is_select_back_pose()) {
    if (injector_->rescue_crowded()) {
      original_box.LongitudinalExtend(
          config_.teb_roi_decider_config()
              .crowded_perception_obstacle_buffer());
      original_box.LateralExtend(config_.teb_roi_decider_config()
                                     .crowded_perception_obstacle_buffer());
    } else {
      original_box.LongitudinalExtend(
          config_.teb_roi_decider_config().perception_obstacle_buffer());
      original_box.LateralExtend(
          config_.teb_roi_decider_config().perception_obstacle_buffer());
    }
  } else {
    original_box.LongitudinalExtend(
        rescue_status_->back_perception_obstacle_buffer());
    original_box.LateralExtend(
        rescue_status_->back_perception_obstacle_buffer());
  }

  // take vector from box
  std::vector<Vec2d> vertices_ccw = original_box.GetAllCorners();
  std::vector<Vec2d> vertices_cw;
  while (!vertices_ccw.empty()) {
    auto current_corner_pt = vertices_ccw.back();
    current_corner_pt.SelfRotate(-1.0 * origin_heading);
    vertices_cw.emplace_back(current_corner_pt);
    vertices_ccw.pop_back();
  }
  // As the perception obstacle is a closed convex set, the first
  // vertice is repeated at the end of the vector to help transform all
  // four edges to inequality constraint
  // 5 point means 4 line segement
  vertices_cw.emplace_back(vertices_cw.front());
  obstacles_vertices_vec->emplace_back(vertices_cw);
  pure_obstacles_vertices_vec->emplace_back(vertices_cw);
  return;
}

void TEBPlannerDecider::UsePolygonPlan(
    century::planning::Frame *const frame, const Obstacle &obstacle,
    size_t *perception_obstacles_num,
    std::vector<std::vector<century::common::math::Vec2d>>
        *obstacles_vertices_vec,
    std::vector<std::vector<century::common::math::Vec2d>>
        *pure_obstacles_vertices_vec,
    size_t *point_num, std::vector<size_t> *const obstacle_points_num) {
  const auto &open_space_info = frame->open_space_info();
  const auto &origin_point = open_space_info.origin_point();
  const auto &origin_heading = open_space_info.origin_heading();

  ++(*perception_obstacles_num);
  Polygon2d obstacle_polygon = obstacle.PerceptionPolygon();
  if (!rescue_status_->is_select_back_pose()) {
    if (injector_->rescue_crowded()) {
      obstacle_polygon = obstacle_polygon.ExpandByDistance(
          0.5 * config_.teb_roi_decider_config()
                    .crowded_perception_obstacle_buffer());
    } else {
      obstacle_polygon = obstacle_polygon.ExpandByDistance(
          0.5 * config_.teb_roi_decider_config().perception_obstacle_buffer());
    }
  } else {
    obstacle_polygon = obstacle_polygon.ExpandByDistance(
        0.5 * rescue_status_->back_perception_obstacle_buffer());
  }

  std::vector<Vec2d> vertices_cw;
  for (auto pt : obstacle_polygon.points()) {
    pt -= origin_point;
    pt.SelfRotate(-origin_heading);
    ++(*point_num);
    vertices_cw.emplace_back(pt);
  }
  if ((*point_num) <= 2) {
    AERROR << "there is a invalid obj point_num " << (*point_num);
  }
  obstacle_points_num->emplace_back((*point_num));
  (*point_num) = 0;
  // As the perception obstacle is a closed convex set, the first
  // vertice is repeated at the end of the vector to help transform all
  // four edges to inequality constraint
  // 5 point means 4 line segement
  vertices_cw.emplace_back(vertices_cw.front());
  obstacles_vertices_vec->emplace_back(vertices_cw);
  pure_obstacles_vertices_vec->emplace_back(vertices_cw);
  return;
}

void TEBPlannerDecider::EnableUseCostMap(
    Frame *const frame, size_t *point_num, size_t *perception_obstacles_num,
    std::vector<size_t> *obstacle_points_num,
    std::vector<std::vector<century::common::math::Vec2d>>
        *obstacles_vertices_vec,
    std::vector<std::vector<century::common::math::Vec2d>>
        *pure_obstacles_vertices_vec) {
  const auto &open_space_info = frame->open_space_info();
  const auto &origin_point = open_space_info.origin_point();
  const auto &origin_heading = open_space_info.origin_heading();
  // AINFO << "perception_obstacles_num before convert "
  //       << (*perception_obstacles_num);
  // AINFO << "costmap_polygons_size: " << frame_->costmap_polygons().size();
  for (const auto &polygon : frame_->costmap_polygons()) {
    if (polygon.IsOutOfOpenSpaceROI()) {
      // AINFO << "costmap IsOutOfOpenSpaceROI";
      continue;
    }
    // take vector from polygon
    Polygon2d obstacle_polygon = polygon;
    if (obstacle_polygon.points().size() < kPolygonMinPointNum) {
      continue;
    }

    ++(*perception_obstacles_num);
    if (injector_->rescue_crowded()) {
      obstacle_polygon = obstacle_polygon.ExpandByDistance(
          0.5 * config_.teb_roi_decider_config()
                    .crowded_perception_obstacle_buffer());
    } else {
      obstacle_polygon = obstacle_polygon.ExpandByDistance(
          0.5 * config_.teb_roi_decider_config().perception_obstacle_buffer());
    }
    std::vector<Vec2d> vertices_cw;
    for (auto pt : obstacle_polygon.points()) {
      pt -= origin_point;
      pt.SelfRotate(-origin_heading);
      ++(*point_num);
      vertices_cw.emplace_back(pt);
    }
    if ((*point_num) <= 2) {
      AERROR << "there is a invalid obj point_num " << (*point_num);
    }
    obstacle_points_num->emplace_back((*point_num));
    (*point_num) = 0;
    vertices_cw.emplace_back(vertices_cw.front());
    obstacles_vertices_vec->emplace_back(vertices_cw);
    pure_obstacles_vertices_vec->emplace_back(vertices_cw);
    obstacle_id_.emplace_back("0");
  }
}

double TEBPlannerDecider::GetObstacleDist(const common::math::Vec2d &point,
                                          const double &ref_heading) {
  double min_dist = kLargeDist;
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();

  // ADC box
  Box2d adc_box(point, ref_heading, adc_length + FLAGS_astar_first_long_buffer,
                adc_width + FLAGS_astar_first_lat_buffer);
  double shift_distance =
      adc_length * 0.5 - vehicle_config.vehicle_param().back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(ref_heading),
                  shift_distance * std::sin(ref_heading)};
  adc_box.Shift(shift_vec);

  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    if (obstacle->IsVirtual() || obstacle->speed() > kDynamicObstacleSpeed) {
      continue;
    }

    const auto &original_box = obstacle->PerceptionBoundingBox();
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();

    double dist = 0.0;
    const auto &obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        FLAGS_enable_openspace_use_polygon_plan) {
      dist = obstacle_polygon.DistanceTo(adc_box);
    } else {
      dist = original_box.DistanceTo(adc_box);
    }
    if (dist < min_dist) {
      min_dist = dist;
      if (min_dist < kEp) {
        ADEBUG << "dist too little , think block" << obstacle->Id();
        return -1.0;
      }
    }
  }
  return min_dist;
}

// WYQ_Maker If obs all point out roi return true
bool TEBPlannerDecider::FilterOutObstacle(const Frame &frame,
                                          const Obstacle &obstacle) {
  if (!obstacle.IsStatic()) {
    return false;
  }
  if (obstacle.IsVirtual()) {
    return true;
  }

  const auto &obstacle_polygon = obstacle.PerceptionPolygon();

  const auto &reference_line_info = frame.reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  common::SLPoint adc_position_sl;
  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_init_position, &adc_position_sl);

  common::SLPoint sl_point_temp;
  bool obs_point_has_in_roi = false;
  for (auto pt : obstacle_polygon.points()) {
    reference_line.XYToSL(pt, &sl_point_temp);
    // AINFO << "s:" << (sl_point_temp.s() - adc_position_sl.s())
    //       << " start:" << start_roi_s_ << " end:" << end_roi_s_
    //       << " l:" << sl_point_temp.l() << " left:" << left_roi_l_
    //       << " right:" << right_roi_l_;
    if (((sl_point_temp.s() - adc_position_sl.s()) >= start_roi_s_) &&
        ((sl_point_temp.s() - adc_position_sl.s()) <= end_roi_s_)) {
      obs_point_has_in_roi = (sl_point_temp.l() >= 0.0)
                                 ? (sl_point_temp.l() <= left_roi_l_)
                                 : (sl_point_temp.l() >= right_roi_l_);
      if (obs_point_has_in_roi) {
        // AINFO << "obs obj is in roi ";
        break;
      }
    }
  }

  if (obs_point_has_in_roi) {
    ADEBUG << "obj dont need filter return false";
    return false;
  } else {
    ADEBUG << "obj all points are out, need filter return true ";
    return true;
  }
  return false;
}

bool TEBPlannerDecider::FilterOutCostMap(
    const Frame &frame, const century::common::math::Polygon2d &polygon) {
  const auto &reference_line_info = frame.reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::SLPoint adc_position_sl;
  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_init_position, &adc_position_sl);

  common::SLPoint sl_point_temp;
  bool obs_point_has_in_roi = false;
  for (auto pt : polygon.points()) {
    reference_line.XYToSL(pt, &sl_point_temp);
    // AINFO << "s:" << (sl_point_temp.s() - adc_position_sl.s())
    //       << " start:" << start_roi_s_ << " end:" << end_roi_s_
    //       << " l:" << sl_point_temp.l() << " left:" << left_roi_l_
    //       << " right:" << right_roi_l_;
    if (((sl_point_temp.s() - adc_position_sl.s()) >= start_roi_s_) &&
        ((sl_point_temp.s() - adc_position_sl.s()) <= end_roi_s_)) {
      obs_point_has_in_roi = (sl_point_temp.l() >= 0.0)
                                 ? (sl_point_temp.l() <= left_roi_l_)
                                 : (sl_point_temp.l() >= right_roi_l_);
      if (obs_point_has_in_roi) {
        // AINFO << "costmap obj is in roi ";
        break;
      }
    }
  }

  if (obs_point_has_in_roi) {
    ADEBUG << "obj dont need filter return false";
    return false;
  } else {
    ADEBUG << "obj all points are out, need filter return true ";
    return true;
  }
  return false;
}

bool TEBPlannerDecider::FilterOutObstacleWithoutHdMap(
    const Frame &frame, const Obstacle &obstacle) {
  if (obstacle.IsVirtual() || !obstacle.IsStatic()) {
    return true;
  }
  const auto &obstacle_box = obstacle.PerceptionBoundingBox();
  // Get vehicle state
  Vec2d vehicle_x_y(vehicle_state_.x(), vehicle_state_.y());
  // Use vehicle position to filter out obstacle
  const double vehicle_center_to_obstacle =
      obstacle_box.DistanceTo(vehicle_x_y);
  const double filtering_distance =
      config_.teb_roi_decider_config().perception_obstacle_filtering_distance();
  AINFO << "vehicle_center_to_obstacle " << vehicle_center_to_obstacle;
  AINFO << "filtering_distance " << filtering_distance;
  if (vehicle_center_to_obstacle > filtering_distance) {
    return true;
  }
  return false;
}

// injector_->vehicle_state()->heading()
// y = ax + b
bool TEBPlannerDecider::LoadObstacleInHyperPlanes(Frame *const frame) {
  *(frame->mutable_open_space_info()->mutable_obstacles_A()) =
      Eigen::MatrixXd::Zero(
          frame->open_space_info().obstacles_edges_num().sum(), 2);
  *(frame->mutable_open_space_info()->mutable_obstacles_b()) =
      Eigen::MatrixXd::Zero(
          frame->open_space_info().obstacles_edges_num().sum(), 1);
  // vertices using H-representation
  if (!GetHyperPlanes(
          frame->open_space_info().obstacles_num(),
          frame->open_space_info().obstacles_edges_num(),
          frame->open_space_info().obstacles_vertices_vec(),
          frame->mutable_open_space_info()->mutable_obstacles_A(),
          frame->mutable_open_space_info()->mutable_obstacles_b())) {
    AERROR << "Fail to present obstacle in hyperplane";
    return false;
  }
  return true;
}

bool TEBPlannerDecider::GetHyperPlanes(
    const size_t &obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    AERROR << "obstacles_num != obstacles_vertices_vec.size()";
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (size_t i = 0; i < obstacles_num; ++i) {
    size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (size_t j = 0; j < current_vertice_num; ++j) {
      Vec2d v1 = obstacles_vertices_vec[i][j];
      Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += static_cast<int>(current_vertice_num);
  }
  return true;
}
bool TEBPlannerDecider::CheckADCIsBlockedTypeWithSurroundObstacles(
    const common::math::Vec2d adc_position, const double adc_heading,
    const Frame *frame, const double front_obstacle_buffer,
    const double shift_dist) {
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width =
      vehicle_config.vehicle_param().width() + FLAGS_astar_first_lat_buffer;
  // ADC box , 0.5 * adc_width as lat buffer
  Box2d adc_box(adc_position, adc_heading, adc_length + front_obstacle_buffer,
                adc_width);

  double buffer = (adc_length + front_obstacle_buffer) * 0.5;
  double shift_distance = shift_dist + buffer -
                          vehicle_config.vehicle_param().back_edge_to_center();

  Vec2d shift_vec{shift_distance * std::cos(adc_heading),
                  shift_distance * std::sin(adc_heading)};
  adc_box.Shift(shift_vec);
  const auto &adc_polygon = Polygon2d(adc_box);
  // obstacle boxes
  auto obstacles = frame->obstacles();
  for (const auto &obstacle : obstacles) {
    if (obstacle->IsOutOfOpenSpaceROI() || obstacle->IsVirtual()) {
      continue;
    }
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    const auto &obs_type = obstacle->Perception().type();
    if (adc_polygon.HasOverlap(obstacle_polygon)) {
      AINFO << "blocked obstacle: " << obstacle->Id()
            << ", front_obstacle_buffer: " << front_obstacle_buffer;
      if (perception::PerceptionObstacle::BICYCLE == obs_type ||
          perception::PerceptionObstacle::PEDESTRIAN == obs_type) {
        AINFO << "found little obj. ";
        return true;
      }
    }
  }
  // use_cost
  if (FLAGS_enable_use_costmap) {
    for (const auto polygon : frame_->costmap_polygons()) {
      if (polygon.IsOutOfOpenSpaceROI()) {
        continue;
      }
      if (polygon.HasOverlap(adc_polygon)) {
        AINFO << "HasOverlap  costmap_polygons_ ";
        // TODO(dzq): waiting costmap type
        const double dx = polygon.max_x() - polygon.min_x();
        const double dy = polygon.max_y() - polygon.min_y();
        if (dx < kLittleObjSize && dy < kLittleObjSize) {
          AINFO << "found little obj ";
          return true;
        }
      }
    }
  }
  return false;
}
// @brief Check if adc blocked with front obs
bool TEBPlannerDecider::CheckADCIsBlockedWithSurroundObstacles(
    const common::math::Vec2d adc_position, const double adc_heading,
    const Frame *frame, const double front_obstacle_buffer,
    const double lat_buffer, const double shift_dist,
    const bool preset_buffer) {
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width() + lat_buffer;
  // ADC box , 0.5 * adc_width as lat buffer
  Box2d adc_box(adc_position, adc_heading, adc_length + front_obstacle_buffer,
                adc_width);

  double buffer = (adc_length + front_obstacle_buffer) * 0.5;
  double shift_distance = shift_dist + buffer -
                          vehicle_config.vehicle_param().back_edge_to_center();

  Vec2d shift_vec{shift_distance * std::cos(adc_heading),
                  shift_distance * std::sin(adc_heading)};
  adc_box.Shift(shift_vec);
  bool block_shift_calc_inner = block_shift_calc_;
  const auto &adc_polygon = Polygon2d(adc_box);
  // obstacle boxes
  // WYQ_Maker
  auto obstacles = frame->obstacles();
  for (const auto &obstacle : obstacles) {
    if (obstacle->IsOutOfOpenSpaceROI() || obstacle->IsVirtual()) {
      continue;
    }
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (adc_polygon.HasOverlap(obstacle_polygon)) {
      AINFO << "blocked obstacle: " << obstacle->Id()
            << ", front_obstacle_buffer: " << front_obstacle_buffer;
      if (!block_shift_calc_inner) {
        if (!block_shift_calc_) {
          auto shift = CalcMinLateralDistance(adc_position, adc_heading,
                                              obstacle_polygon);
          block_shift_l_ = shift.l;
          block_shift_s_ = shift.s;
        } else {
          auto shift = CalcMinLateralDistance(adc_position, adc_heading,
                                              obstacle_polygon);
          if (std::fabs(shift.l) < std::fabs(block_shift_l_)) {
            // base on shift l
            block_shift_l_ = shift.l;
            block_shift_s_ = shift.s;
          }
        }
        AINFO << "++++++++block_shift_l_" << block_shift_l_;
        AINFO << "++++++++block_shift_s_" << block_shift_s_;
        block_shift_calc_ = true;
      }
      if (preset_buffer && !injector_->deal_start_block_) {
        AINFO << "injector_->deal_start_block_ 1";
        injector_->deal_start_block_ = true;
        // throngh here is no buffer, kown that obstacle vectices had buffer.
        if (DetermainADCIsBlockedWithFrontObstacles(
                obstacle_polygon, adc_position, adc_heading, 0.0, 0.0)) {
          start_collision_ = true;
          return true;
        }
      }
      return true;
    }
  }
  // use_cost
  if (FLAGS_enable_use_costmap) {
    if (CheckBlockedWithCostmap(adc_polygon, adc_position, adc_heading,
                                preset_buffer, block_shift_calc_inner))
      return true;
  }

  return false;
}

bool TEBPlannerDecider::CheckBlockedWithCostmap(
    const common::math::Polygon2d &adc_polygon,
    const common::math::Vec2d adc_position, const double adc_heading,
    bool preset_buffer, bool block_shift_calc_inner) {
  for (const auto polygon : frame_->costmap_polygons()) {
    if (polygon.IsOutOfOpenSpaceROI()) {
      continue;
    }
    // TODO(zhiqiang.ding): waiting for costmap dynamic info.
    if (polygon.HasOverlap(adc_polygon)) {
      AINFO << "HasOverlap  costmap_polygons_ ";
      if (!block_shift_calc_inner) {
        if (!block_shift_calc_) {
          auto shift =
              CalcMinLateralDistance(adc_position, adc_heading, polygon);
          block_shift_l_ = shift.l;
          block_shift_s_ = shift.s;
        } else {
          auto shift =
              CalcMinLateralDistance(adc_position, adc_heading, polygon);
          if (std::fabs(shift.l) < std::fabs(block_shift_l_)) {
            // base on shift l
            block_shift_l_ = shift.l;
            block_shift_s_ = shift.s;
          }
        }
        AINFO << "++++++++block_shift_l_" << block_shift_l_;
        AINFO << "++++++++block_shift_s_" << block_shift_s_;
        block_shift_calc_ = true;
      }
      if (preset_buffer && !injector_->deal_start_block_) {
        AINFO << "injector_->deal_start_block_ 1";
        injector_->deal_start_block_ = true;
        // throngh here is no buffer, kown that obstacle vectices had buffer.
        if (DetermainADCIsBlockedWithFrontObstacles(polygon, adc_position,
                                                    adc_heading, 0.0, 0.0)) {
          start_collision_ = true;
          return true;
        }
      }
      return true;
    }
  }
  return false;
}

bool TEBPlannerDecider::DetermainADCIsBlockedWithFrontObstacles(
    const common::math::Polygon2d &obj, const common::math::Vec2d adc_position,
    const double adc_heading, const double lon_buffer,
    const double lat_buffer) {
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width() + lat_buffer;
  Box2d adc_box(adc_position, adc_heading, adc_length + lon_buffer, adc_width);

  double shift_distance =
      adc_length * 0.5 - vehicle_config.vehicle_param().back_edge_to_center();

  Vec2d shift_vec{shift_distance * std::cos(adc_heading),
                  shift_distance * std::sin(adc_heading)};
  adc_box.Shift(shift_vec);
  const auto &adc_polygon = Polygon2d(adc_box);
  if (adc_polygon.HasOverlap(obj)) {
    AINFO << "remove buffer, blocked obstacle";
    return true;
  }
  return false;
}

// lwt: use sl dist may also ok (ref:GetRescueLaneBoundaryPoints),
// this  also ok, but some extra point , not much
void TEBPlannerDecider::GetLaneBoundaryPoints(
    LaneInfoConstPtr lane_info, const bool &is_left_bound,
    const hdmap::Path &nearby_path, std::vector<Vec2d> *const boundary_points) {
  const auto &boundary_segment =
      is_left_bound ? lane_info->lane().left_boundary().curve().segment()
                    : lane_info->lane().right_boundary().curve().segment();
  if (boundary_segment.size() > 1) {
    AINFO << "boundary_segment.size() " << boundary_segment.size();
  }
  size_t lane_points_num = boundary_segment[0].line_segment().point_size();

  Vec2d center_position = {
      0.5 * (injector_->vehicle_state()->x() + rescue_end_point_.x()),
      0.5 * (injector_->vehicle_state()->y() + rescue_end_point_.y())};

  const double search_r_sqrt = kSearchRadius * kSearchRadius;

  ADEBUG << "center_position.x() " << center_position.x()
         << "center_position.y() " << center_position.y() << " lane_points_num"
         << lane_points_num;

  ADEBUG << "lane_info->lane().id().id()== " << lane_info->lane().id().id()
         << "  rescue_end_point_.x()  " << rescue_end_point_.x();

  double last_sqrt_s = 0;
  for (size_t i = 0; i < lane_points_num; ++i) {
    const auto &point = boundary_segment[0].line_segment().point().at(i);
    Vec2d point_v = {point.x(), point.y()};

    double sqrt_s =
        (point.x() - center_position.x()) * (point.x() - center_position.x()) +
        (point.y() - center_position.y()) * (point.y() - center_position.y());

    ADEBUG << "point.x() " << point.x() << "point.y() " << point.y();
    ADEBUG << "sqrt_s " << sqrt_s << "   0  " << last_sqrt_s;
    if (sqrt_s < search_r_sqrt) {
      boundary_points->emplace_back(point_v);
    }

    last_sqrt_s = sqrt_s;
  }
}

// delete
// @brief Get rescue lane boundaries of both sides
// void TEBPlannerDecider::GetRescueLaneBoundary(
//     const hdmap::Path &nearby_path, const common::math::Vec2d &origin_point,
//     const double origin_heading, std::vector<Vec2d> *left_lane_boundary,
//     std::vector<Vec2d> *right_lane_boundary) {
//   //----------------------------------------------------------------------
//   ADEBUG << "---------Go Here use lane as rescue boundary----------";
//   double vehicle_lane_s = 0.0;
//   double vehicle_lane_l = 0.0;
//   hdmap::Id left_neighbor_lane_id;
//   hdmap::Id left_neighbor_forward_lane_id;
//   hdmap::Id left_neighbor_reverse_lane_id;
//   hdmap::Id right_neighbor_forward_lane_id;
//   LaneInfoConstPtr car_lane;
//   PointENU car_pose;
//   //----------------------------------------------------------------------
//   Vec2d target_position = {injector_->vehicle_state()->x(),
//                            injector_->vehicle_state()->y()};

//   car_pose.set_x(target_position.x());
//   car_pose.set_y(target_position.y());
//   //----------------------------------------------------------------------

//   hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s,
//   &vehicle_lane_l); if (nullptr == car_lane) {
//     AERROR << "hdmap_->GetNearestLane failed with pos x: " << car_pose.x()
//            << " y: " << car_pose.y();
//     return;
//   }

//   bool match_suc = false;
//   size_t cur_lane_i = 0;

//   for (size_t i = 0; i < lane_ids_.size(); ++i) {
//     if (car_lane->lane().id().id() == lane_ids_[i].id()) {
//       cur_lane_i = i;
//       match_suc = true;
//     }
//     ADEBUG << "ref lane_id is " << lane_ids_[i].id() << "i " << i
//            << "cur_lane_i" << cur_lane_i;
//   }
//   // match_suc = false;  // test
//   std::vector<century::hdmap::Id> use_lane_ids;
//   use_lane_ids.clear();
//   size_t pre_i = cur_lane_i >= 1 ? cur_lane_i - 1 : 0;
//   // +1 may not length enough , use next two lanes
//   size_t next_i = cur_lane_i + 2 >= lane_ids_.size() ? lane_ids_.size() - 1
//                                                      : cur_lane_i + 2;

//   const auto &debug_lane =
//       hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(lane_ids_.at(next_i));
//   double total_length = debug_lane->total_length();
//   ADEBUG << "total_length " << total_length;
//   if (match_suc) {
//     for (size_t i = pre_i; i < next_i + 1 && i < lane_ids_.size(); ++i) {
//       use_lane_ids.push_back(lane_ids_[i]);
//     }
//   } else {
//     use_lane_ids = lane_ids_;
//   }
//   ADEBUG << "car lane id  is " << car_lane->lane().id().id() << "cur_lane_i "
//          << cur_lane_i << "use_lane_ids " << use_lane_ids.size() << "pre_i "
//          << pre_i;

//   ADEBUG << "l_r " <<
//   car_lane->lane().left_neighbor_reverse_lane_id().empty()
//          << "l_f " <<
//          car_lane->lane().left_neighbor_forward_lane_id().empty();

//   for (const auto &id : use_lane_ids) {
//     const auto &use_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id);

//     // int lane_boundary_type = 0;
//     if (!use_lane->lane().left_neighbor_forward_lane_id().empty()) {
//       left_neighbor_forward_lane_id =
//           use_lane->lane().left_neighbor_forward_lane_id(0);

//       ADEBUG << "left_neighbor_forward_lane_id "
//              << left_neighbor_forward_lane_id.id() << " ,use_lane "
//              << use_lane->lane().id().id();

//       const auto &left_neighbor_forward_lane =
//           hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
//               left_neighbor_forward_lane_id);
//       if (nullptr == left_neighbor_forward_lane) {
//         AERROR << "Get left_neighbor_forward_lane failed, use current lane";
//         GetLaneBoundaryPoints(use_lane, true, nearby_path,
//         left_lane_boundary);
//       } else {
//         GetLaneBoundaryPoints(left_neighbor_forward_lane, true, nearby_path,
//                               left_lane_boundary);
//       }
//     } else if (!use_lane->lane().left_neighbor_reverse_lane_id().empty()) {
//       // If there are adjacent lanes in the reverse direction of this lane,
//       // the first lane on the left of the reverse direction is obtained
//       left_neighbor_reverse_lane_id =
//           use_lane->lane().left_neighbor_reverse_lane_id(0);

//       ADEBUG << "left_neighbor_reverse_lane_id "
//              << left_neighbor_reverse_lane_id.id() << " ,use_lane "
//              << use_lane->lane().id().id();

//       const auto &left_neighbor_reverse_lane =
//           hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
//               left_neighbor_reverse_lane_id);
//       if (nullptr == left_neighbor_reverse_lane) {
//         AERROR << "Get left_neighbor_reverse_lane failed, use current lane";
//         GetLaneBoundaryPoints(use_lane, true, nearby_path,
//         left_lane_boundary);
//       } else {
//         GetLaneBoundaryPoints(left_neighbor_reverse_lane, false, nearby_path,
//                               left_lane_boundary);
//       }
//     } else {
//       ADEBUG << "Get left_neighbor_forward_lane / left_neighbor_reverse_lane
//       "
//                 "failed, use current lane";
//       GetLaneBoundaryPoints(use_lane, true, nearby_path, left_lane_boundary);
//     }

//     // If the left boundary of the lane is obtained on the left, other lanes
//     on
//     // the right will be considered. Otherwise, obtain the right boundary of
//     // this lane.

//     // If this lane has adjacent lanes in the same direction, the first
//     // lane on the right is obtained
//     if (!use_lane->lane().right_neighbor_forward_lane_id().empty()) {
//       right_neighbor_forward_lane_id =
//           use_lane->lane().right_neighbor_forward_lane_id(0);

//       ADEBUG << "right_neighbor_forward_lane_id "
//              << right_neighbor_forward_lane_id.id() << ",use_lane "
//              << use_lane->lane().id().id();

//       const auto &right_neighbor_forward_lane =
//           hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
//               right_neighbor_forward_lane_id);
//       if (nullptr == right_neighbor_forward_lane) {
//         AERROR << "Get right_neighbor_forward_lane failed, use current "
//                   "right lane boundary.";
//         GetLaneBoundaryPoints(use_lane, false, nearby_path,
//                               right_lane_boundary);
//       } else {
//         GetLaneBoundaryPoints(right_neighbor_forward_lane, false,
//         nearby_path,
//                               right_lane_boundary);
//       }
//     } else {
//       ADEBUG << "Get right_neighbor_forward_lane failed, use current right "
//                 "lane boundary.";
//       GetLaneBoundaryPoints(use_lane, false, nearby_path,
//       right_lane_boundary);
//     }
//   }

//   //----------------------------------------------------------------------
//   // turn lane boundary to origin
//   size_t left_point_size = left_lane_boundary->size();
//   size_t right_point_size = right_lane_boundary->size();
//   for (size_t i = 0; i < left_point_size; ++i) {
//     ADEBUG << "left_road_boundary: [" << std::setprecision(9)
//            << left_lane_boundary->at(i).x() << ", "
//            << left_lane_boundary->at(i).y() << "]";
//     left_lane_boundary->at(i) -= origin_point;
//     left_lane_boundary->at(i).SelfRotate(-origin_heading);
//   }
//   for (size_t i = 0; i < right_point_size; ++i) {
//     right_lane_boundary->at(i) -= origin_point;
//     right_lane_boundary->at(i).SelfRotate(-origin_heading);
//   }

//   InterpolateBoundary(
//       kBoundaryInterval,
//       config_.teb_roi_decider_config().roi_line_segment_min_angle(),
//       left_lane_boundary);
//   InterpolateBoundary(
//       kBoundaryInterval,
//       config_.teb_roi_decider_config().roi_line_segment_min_angle(),
//       right_lane_boundary);
// }

void TEBPlannerDecider::CalPreLaneBoundary(
    hdmap::LaneInfoConstPtr lane_info, const bool &is_left_bound,
    const double *start_s, double *pre_start_s, double *pre_end_s,
    hdmap::Id *pre_lane_id, hdmap::LaneInfoConstPtr pre_lane_info,
    std::vector<common::math::Vec2d> *const boundary_points) {
  double point_s = 0.0;
  double point_l = 0.0;
  if ((*start_s) < 0) {
    if (!lane_info->lane().predecessor_id().empty()) {
      (*pre_lane_id) = lane_info->lane().predecessor_id(0);
      pre_lane_info =
          hdmap::HDMapUtil::BaseMapPtr()->GetLaneById((*pre_lane_id));
      if (nullptr == pre_lane_info) {
        AERROR << "Get pre_lane_info failed, lane id is "
               << (*pre_lane_id).id();
      } else {
        double pre_lane_total_length = pre_lane_info->total_length();
        (*pre_start_s) = pre_lane_total_length + (*start_s);
        (*pre_end_s) = pre_lane_total_length;
        // ----------------------------------------------------------------------------------
        // Add pre lane boundary
        const auto &pre_boundary_segment =
            is_left_bound
                ? pre_lane_info->lane().left_boundary().curve().segment()
                : pre_lane_info->lane().right_boundary().curve().segment();
        size_t pre_lane_points_num =
            pre_boundary_segment[0].line_segment().point_size();
        for (size_t i = 0; i < pre_lane_points_num; ++i) {
          const auto &point =
              pre_boundary_segment[0].line_segment().point().at(i);
          Vec2d point_v = {point.x(), point.y()};
          pre_lane_info->GetProjection(point_v, &point_s, &point_l);

          if (point_s > (*pre_start_s) && point_s < (*pre_end_s)) {
            boundary_points->emplace_back(point_v);
          }
        }
        // ----------------------------------------------------------------------------------
      }
    }
  }
  return;
}

// delete
// @brief just for GetRescueLaneBoundaryPoints
// void TEBPlannerDecider::GetRescueLaneBoundaryPoints(
//     hdmap::LaneInfoConstPtr lane_info, const bool &is_left_bound,
//     const hdmap::Path &nearby_path,
//     std::vector<common::math::Vec2d> *const boundary_points) {
//   //
//   ----------------------------------------------------------------------------------
//   double point_s = 0.0;
//   double point_l = 0.0;
//   double target_position_s, target_position_l;

//   // get the center point of start to end
//   const auto &origin_point = frame_->open_space_info().origin_point();
//   Vec2d origin_position = {origin_point.x(), origin_point.y()};
//   Vec2d end_position = {rescue_end_point_.x(), rescue_end_point_.y()};
//   Vec2d origin_to_end = end_position - origin_position;
//   Vec2d target_position = {origin_position.x() + origin_to_end.x() * 0.5,
//                            origin_position.y() + origin_to_end.y() * 0.5};

//   lane_info->GetProjection(target_position, &target_position_s,
//                            &target_position_l);

//   //
//   ----------------------------------------------------------------------------------
//   // Obtain the maximum s/length center curve length of the current lane
//   double total_length = lane_info->total_length();
//   //
//   ----------------------------------------------------------------------------------

//   // get lane boundary point from start_s to end_s
//   double start_s =
//       target_position_s -
//       config_.teb_roi_decider_config().roi_longitudinal_rescue_range();
//   double end_s =
//       target_position_s +
//       config_.teb_roi_decider_config().roi_longitudinal_rescue_range();
//   ADEBUG << "roi_longitudinal_rescue_range "
//          << config_.teb_roi_decider_config().roi_longitudinal_rescue_range();
//   ADEBUG << "start_s " << start_s << " end_s " << end_s;

//   //
//   ----------------------------------------------------------------------------------
//   // Add pre lane boundary
//   double pre_start_s = 0;
//   double pre_end_s = 0;
//   hdmap::LaneInfoConstPtr pre_lane_info = nullptr;
//   hdmap::Id pre_lane_id;
//   // If it is less than 0, the boundary of the previous Lane is required
//   CalPreLaneBoundary(lane_info, is_left_bound, &start_s, &pre_start_s,
//                      &pre_end_s, &pre_lane_id, pre_lane_info,
//                      boundary_points);
//   //
//   ----------------------------------------------------------------------------------

//   //
//   ----------------------------------------------------------------------------------
//   // Add current lane boundary
//   double current_start_s = start_s;
//   double current_end_s = end_s;
//   if (start_s < 0) {
//     current_start_s = 0;
//   }
//   if (end_s > total_length) {
//     current_end_s = total_length;
//   }

//   const auto &boundary_segment =
//       is_left_bound ? lane_info->lane().left_boundary().curve().segment()
//                     : lane_info->lane().right_boundary().curve().segment();
//   size_t lane_points_num = boundary_segment[0].line_segment().point_size();

//   for (size_t i = 0; i < lane_points_num; ++i) {
//     const auto &point = boundary_segment[0].line_segment().point().at(i);
//     Vec2d point_v = {point.x(), point.y()};
//     lane_info->GetProjection(point_v, &point_s, &point_l);

//     if (point_s > current_start_s && point_s < current_end_s) {
//       boundary_points->emplace_back(point_v);
//     }
//   }
//   //
//   ----------------------------------------------------------------------------------

//   //
//   ----------------------------------------------------------------------------------
//   // Add next lane boundary
//   double next_start_s = 0;
//   double next_end_s = 0;
//   hdmap::LaneInfoConstPtr next_lane_info = nullptr;
//   hdmap::Id next_lane_id;
//   // If greater than total_ Length, the boundary of the next Lane is required
//   if (end_s > total_length) {
//     if (!lane_info->lane().successor_id().empty()) {
//       next_lane_id = lane_info->lane().successor_id(0);
//       next_lane_info =
//           hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(next_lane_id);
//       if (nullptr == next_lane_info) {
//         AERROR << "Get next_lane_info failed, lane id is " <<
//         next_lane_id.id();
//       } else {
//         next_start_s = 0;
//         next_end_s = end_s - total_length;
//         //
//         ----------------------------------------------------------------------------------
//         // Add next lane boundary
//         const auto &next_boundary_segment =
//             is_left_bound
//                 ? next_lane_info->lane().left_boundary().curve().segment()
//                 : next_lane_info->lane().right_boundary().curve().segment();
//         size_t next_lane_points_num =
//             next_boundary_segment[0].line_segment().point_size();
//         for (size_t i = 0; i < next_lane_points_num; ++i) {
//           const auto &point =
//               next_boundary_segment[0].line_segment().point().at(i);
//           Vec2d point_v = {point.x(), point.y()};
//           next_lane_info->GetProjection(point_v, &point_s, &point_l);

//           if (point_s > next_start_s && point_s < next_end_s) {
//             boundary_points->emplace_back(point_v);
//           }
//         }
//         //
//         ----------------------------------------------------------------------------------
//       }
//     }
//   }
//   //
//   ----------------------------------------------------------------------------------
// }

void TEBPlannerDecider::InterpolateBoundary(
    const double &s_interval, const double &heading_interval,
    std::vector<common::math::Vec2d> *boundarys) {
  std::vector<Vec2d> boundary_points;
  std::copy(boundarys->begin(), boundarys->end(),
            std::back_inserter(boundary_points));
  ADEBUG << "heading_interval temp not use " << heading_interval;
  ADEBUG << "boundary_points size before Interpolate" << boundary_points.size();
  if (boundary_points.size() < 2) {
    return;
  }
  boundarys->clear();
  Vec2d start_point = boundary_points.front();
  boundarys->emplace_back(start_point);
  double last_heading =
      std::atan2(boundary_points[1].y() - boundary_points[0].y(),
                 boundary_points[1].x() - boundary_points[0].x());
  for (size_t i = 1; i < boundary_points.size(); ++i) {
    double heading =
        std::atan2(boundary_points[i].y() - boundary_points[i - 1].y(),
                   boundary_points[i].x() - boundary_points[i - 1].x());
    bool is_center_lane_heading_change =
        std::fabs(common::math::NormalizeAngle(heading - last_heading)) >
        heading_interval;
    ADEBUG << "delta heading"
           << std::fabs(common::math::NormalizeAngle(heading - last_heading));
    last_heading = heading;

    double dist = start_point.DistanceTo(boundary_points[i]);
    if (std::fabs(dist) > s_interval || boundary_points.size() == i + 1 ||
        is_center_lane_heading_change) {
      start_point = boundary_points[i];
      boundarys->emplace_back(start_point);
    }
  }
  ADEBUG << "boundarys size after Interpolate" << boundarys->size();
}
BlockShift TEBPlannerDecider::CalcMinLateralDistance(
    const common::math::Vec2d &adc_position, const double &adc_heading,
    const common::math::Polygon2d &obj) {
  // const double min_turn_radius =
  //     century::common::VehicleConfigHelper::MinSafeTurnRadius();
  double lateral_min_left = 0.0;
  double lateral_min_right = 0.0;
  double longitunial = std::numeric_limits<double>::max();
  // double nearest_dist = 10.0;
  for (const auto &pt : obj.points()) {
    double dx = pt.x() - adc_position.x();
    double dy = pt.y() - adc_position.y();
    common::math::Vec2d v1(dx, dy);
    common::math::Vec2d v2(std::cos(adc_heading), std::sin(adc_heading));
    // const double inner = v1.InnerProd(v2);
    // nearest_dist = std::min(nearest_dist, inner);
    const double corss = v2.CrossProd(v1);
    if (corss > 0.0) {
      //  on the left of adc
      lateral_min_left = std::max(lateral_min_left, corss);
    } else {
      // on the right of adc
      lateral_min_right = std::max(lateral_min_right, -corss);
    }
    longitunial = std::min(longitunial, std::fabs(v2.InnerProd(v1)));
  }
  AINFO << "lateral_min_left: " << lateral_min_left
        << ", lateral_min_right: " << lateral_min_right;
  // if (lateral_min_left < kEp && lateral_min_right < kEp) {
  //   return 0.0;
  // }
  BlockShift res;
  res.l = lateral_min_left < lateral_min_right ? lateral_min_left
                                               : -lateral_min_right;
  res.s = longitunial;
  return res;
}

bool TEBPlannerDecider::TryAddGoalSample(
    double s, double l,
    double prefer_l,
    double lateral_cost_ratio,
    const common::SLPoint& start_point,
    const common::SLPoint& adc_position_sl,
    const Vec2d& end_pose_x_y,
    bool pre_end_pose_check_valid,
    TEBTarStatus tar_status,
    bool is_rescue,
    bool is_pullover_ready,
    const Frame* frame,
    const ReferenceLine& reference_line) {
  common::SLPoint sl_point;
  sl_point.set_s(s);
  sl_point.set_l(l);

  Vec2d sample_point;
  if (!reference_line.SLToXY(sl_point, &sample_point)) {
    AERROR << "Failed to get xy from sl: " << sl_point.DebugString();
    return false;
  }

  double ref_heading = reference_line.GetReferencePoint(s).heading();
  if (!CheckGoalIsValid(sample_point, ref_heading)) {
    return false;
  }

  double lat_cost =
      lateral_cost_ratio * kRescueLatCost * std::fabs(l - prefer_l);
  double lon_cost =
      kBaseCost - kRescueLonCost * std::max(s - start_point.s(), 0.0);

  double obstacle_dist_cost = 0.0;
  double front_blocked_cost = 0.0;
  double history_cost = 0.0;
  double ego_cost = 0.0;

  if (is_rescue && !is_pullover_ready) {
    double dist = GetObstacleDist(sample_point, ref_heading);
    if (dist < FLAGS_rescue_hybird_ingore_safe_distance) {
      return false;
    } else if (dist <= FLAGS_rescue_hybird_ingore_distance) {
      obstacle_dist_cost =
          std::min(kCollisionCost, kRescueObstacleCost / dist);
    }

    if (FLAGS_enable_rescue_surround_cost) {
      if (CheckADCIsBlockedWithSurroundObstacles(
              sample_point, ref_heading, frame,
              kCheckDist, preset_lat_buffer_, 0.0)) {
        front_blocked_cost = kFrontBlockedCost;
      }
    }

    if (pre_end_pose_check_valid) {
      double pose_dist =
          std::hypot(sample_point.x() - end_pose_x_y.x(),
                     sample_point.y() - end_pose_x_y.y());
      history_cost = std::max(0.0, pose_dist * kHisotryCost);
    }

    if (FLAGS_use_ego_l_cost) {
      ego_cost = kRescueEgoLCost *
                 std::fabs(l - adc_position_sl.l());
    }
  }

  double total_cost = lat_cost + lon_cost + obstacle_dist_cost +
                      front_blocked_cost + history_cost + ego_cost;

  goals_vector_.emplace_back(
      PointWithCost{sample_point, ref_heading, total_cost});
  return true;
}

// lwt:the second sampling is more sparse in long direction;
// the second sampling start point is first sample end point;
void TEBPlannerDecider::GenerateSampleGoals(
    const common::SLPoint &start_point, const double &long_start_s,
    const double &long_end_s, const double &long_interval_s, const Frame *frame,
    const ReferenceLine &reference_line) {
  const auto *previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool pre_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto &gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;
  bool pre_end_pose_check_valid =
      gear != canbus::Chassis::GEAR_REVERSE && pre_plan_success;
  const auto &end_pose =
      previous_frame->open_space_info().open_space_end_pose();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d end_pose_x_y(0.0, 0.0);

  if (pre_end_pose_check_valid && !end_pose.empty()) {
    end_pose_x_y.set_x(end_pose[0]);
    end_pose_x_y.set_y(end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;
  }

  goals_vector_.clear();
  LaneInfoConstPtr car_lane;
  PointENU car_pose;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  const double half_adc_width =
      0.5 * (vehicle_params_.width() + FLAGS_astar_first_lat_buffer);
  double mutable_long_end_s = long_end_s;
  double mutable_long_start_s = long_start_s;
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  common::SLPoint adc_position_sl;
  common::SLPoint routing_end_position_sl;
  const auto &routing_end_point = frame_->local_view()
                                      .routing->routing_request()
                                      .waypoint()
                                      .rbegin()
                                      ->pose();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  reference_line.XYToSL(routing_end_point, &routing_end_position_sl);

  common::SLPoint check_position_sl = adc_position_sl;
  const double ref_heading =
      reference_line.GetReferencePoint(adc_position_sl.s()).heading();
  const auto &is_rescue = frame->open_space_info().is_rescue_mode();
  common::math::Vec2d check_position;
  double sample_interval = long_interval_s;
  TEBTarStatus tar_status = frame->open_space_info().tar_status();
  double prefer_l = kPreferL;
  const double l_shift = 0.5;

  // wyq same_lang
  if (is_pullover_ready_) {
    mutable_long_end_s = routing_end_position_sl.s() - adc_position_sl.s() +
                         (kPulloverSearchExtraS * 0.5);
    mutable_long_start_s = routing_end_position_sl.s() - adc_position_sl.s() -
                           2.0 * kPulloverSearchExtraS - kEp;
    sample_interval = kSearchMinInterval;
  } else if (!(injector_->use_teb_default_bound_ &&
               config_.teb_roi_decider_config().enable_costmap_boundary()) &&
             is_rescue) {
    if (!injector_->is_personlike_blocked_) {
      for (double check_s = kFrontSearchCheckDist; check_s <= long_start_s;
           check_s += kCheckStep) {
        check_position_sl.set_s(adc_position_sl.s() + check_s);
        reference_line.SLToXY(check_position_sl, &check_position);
        if (CheckADCIsBlockedWithSurroundObstacles(check_position, ref_heading,
                                                   frame, 0.0,
                                                   preset_lat_buffer_, 0.0)) {
          mutable_long_end_s = check_s + kEp + vehicle_params_.length();
          mutable_long_start_s = check_s + vehicle_params_.length();
          prefer_l = block_shift_l_ > 0.0 ? adc_position_sl.l() - l_shift

                                          : adc_position_sl.l() + l_shift;
          break;
        }
      }
    }
  }
  // expected that suppresing actions of adc.
  if (injector_->shrink_end_s_) {
    prefer_l = adc_position_sl.l();
  }
  double lateral_cost_ratio = 1.0;
  if (injector_->teb_adc_is_out_lane_ && injector_->is_in_play_street) {
    prefer_l = 0.0;
    lateral_cost_ratio = 2.0;
  }
  static size_t fail_count = 0;
  fail_count = fail_to_select_goal_ ? fail_count + 1 : 0;
  for (double s = start_point.s() + mutable_long_start_s;
       s < start_point.s() + mutable_long_end_s; s = s + sample_interval) {
    double left = 0.0;
    double right = 0.0;
    if (!reference_line.GetLaneWidth(s, &left, &right)) {
      AERROR << "reference_line GetLaneWidth failed.";
      continue;
    }
    // left not + ,for better carrying capacity
    double width = left + right;
    right = -right;
    car_pose.set_x(reference_line.GetReferencePoint(s).x());
    car_pose.set_y(reference_line.GetReferencePoint(s).y());
    hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s,
                           &vehicle_lane_l);

    if (nullptr == car_lane.get()) {
      AERROR << "hdmap_->GetNearestLane failed with pos x: " << car_pose.x()
             << " y: " << car_pose.y();
    } else {
      bool try_borrow =
          (!car_lane->lane().left_neighbor_forward_lane_id().empty() ||
           !car_lane->lane().left_neighbor_reverse_lane_id().empty()) &&
          ((is_in_city_road_ && fail_count > kFailCount));
      if (((!car_lane->lane().left_neighbor_reverse_lane_id().empty() ||
            !car_lane->lane().left_neighbor_forward_lane_id().empty()) &&
           !is_in_city_road_) ||
          try_borrow) {
        left += std::max(0.0, width);
      } else {
        AERROR << "notice! there is no neighbor lane"
               << "car_lane->lane() id: " << car_lane->lane().id().id();
      }
    }

    if (TEBRoiDeciderConfig::RESCUE ==
            config_.teb_roi_decider_config().roi_type() &&
        config_.teb_roi_decider_config().enable_costmap_boundary() &&
        config_.teb_roi_decider_config().enable_select_outside_road() &&
        !junction_in_play_street_ && !is_in_city_road_) {
      left += vehicle_params_.width() + FLAGS_astar_first_lat_buffer;
      right -= vehicle_params_.width() + FLAGS_astar_first_lat_buffer;
    }
    double right_bound = right + half_adc_width;
    double left_bound = left - half_adc_width;
    if (is_pullover_ready_) {
      left_bound = kEp;
      prefer_l = right_bound;
    }
    if (TEBTarStatus::CREEP == tar_status && !is_in_city_road_) {
      left_bound = kEp;
      prefer_l = right_bound - half_adc_width;
    }

    for (double l = right_bound; l <= left_bound;
         l += FLAGS_rescue_hybird_lat_sample_interval) {
      TryAddGoalSample(s, l, prefer_l, lateral_cost_ratio, start_point,
                       adc_position_sl, end_pose_x_y, pre_end_pose_check_valid,
                       tar_status, is_rescue, is_pullover_ready_, frame,
                       reference_line);
    }

    if (!is_pullover_ready_ && goals_vector_.empty() &&
        mutable_long_end_s < mutable_long_start_s + kSearchExtraS) {
      // complex case use small sample_interval , origin sample s 2~3
      sample_interval = kSearchMinInterval;
      mutable_long_end_s += sample_interval;
    }
  }
  std::sort(goals_vector_.begin(), goals_vector_.end(),
            [](const PointWithCost &left, const PointWithCost &right) {
              return left.cost > right.cost;
            });
}

void TEBPlannerDecider::GenerateBackSampleGoals(
    const common::SLPoint &start_point, const double &long_end_s,
    const Frame *frame, const ReferenceLine &reference_line) {
  goals_vector_.clear();

  double start_s = start_point.s() + long_end_s;
  double end_s = start_point.s() + 0.5 * long_end_s;
  double long_interval_s = kLongIntervalS;
  double left = 0.0;
  double right = 0.0;
  if (!CalLatSampleRange(reference_line, start_s, &left, &right)) {
    return;
  }
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_width =
      vehicle_config.vehicle_param().width() + FLAGS_astar_first_lat_buffer;
  const double half_adc_width = 0.5 * adc_width;
  auto obstacles = frame->obstacles();
  double perl = start_point.l();
  // const double l_shift = 0.5;  // = Rmin(1-cos(30))
  const double l_shift = 0.0;  // = Rmin(1-cos(30))
  perl = block_shift_l_ > 0.0 ? perl - l_shift : perl + l_shift;
  ADEBUG << "perl - ego l : " << perl - start_point.l();
  if (TEBTarStatus::YIELD == frame->open_space_info().tar_status()) {
    perl = right + half_adc_width + FLAGS_astar_first_lat_buffer * 0.5;
    // AINFO << "back perl reset: " << perl;
  }
  for (double s = start_s; s < end_s; s = s + long_interval_s) {
    common::SLPoint sl_point;
    sl_point.set_s(s);
    double lon_cost = kBackEndPoseCost * std::fabs(s - start_s);
    double ref_heading = reference_line.GetReferencePoint(s).heading();
    for (double l = right + half_adc_width; l + half_adc_width <= left;
         l += FLAGS_rescue_hybird_lat_sample_interval) {
      sl_point.set_l(l);
      Vec2d sample_point;
      if (!reference_line.SLToXY(sl_point, &sample_point)) {
        AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
        continue;
      }

      if (!CheckGoalIsValid(sample_point, ref_heading)) {
        continue;
      }

      double lat_cost = kBackEndPoseCost * std::fabs(l - perl);

      double total_cost = lon_cost + lat_cost;
      double heading = ref_heading;
      // const double shift_angle = kShiftAngle;
      const double shift_angle = 0.0;
      heading = block_shift_l_ > 0 ? ref_heading - shift_angle
                                   : ref_heading + shift_angle;
      goals_vector_.emplace_back(
          sample_point, common::math::NormalizeAngle(heading), total_cost);
    }
  }
  std::sort(goals_vector_.begin(), goals_vector_.end(),
            [](const PointWithCost &left, const PointWithCost &right) {
              return left.cost > right.cost;
            });
}

bool TEBPlannerDecider::CalLatSampleRange(const ReferenceLine &reference_line,
                                          const double &start_s,
                                          double *const left,
                                          double *const right) {
  PointENU car_pose;
  car_pose.set_x(vehicle_state_.x());
  car_pose.set_y(vehicle_state_.y());
  LaneInfoConstPtr car_lane;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;

  if (!reference_line.GetLaneWidth(start_s, left, right)) {
    AERROR << "reference_line GetLaneWidth failed.";
    return false;
  }
  (*right) = -(*right);
  const double lane_width = (*left) - (*right);
  hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s, &vehicle_lane_l);
  if (nullptr == car_lane) {
    AERROR << "hdmap_->GetNearestLane failed with pos x: " << car_pose.x()
           << " y: " << car_pose.y();
  } else {
    if ((!car_lane->lane().left_neighbor_reverse_lane_id().empty() ||
         !car_lane->lane().left_neighbor_forward_lane_id().empty()) &&
        (!is_in_city_road_ || fail_to_select_goal_)) {
      // back condition is simple use fail_to_select_goal
      // front is more difficult
      (*left) += lane_width;
    } else {
      // do nothing
    }
  }
  // const auto &vehicle_config =
  //     common::VehicleConfigHelper::Instance()->GetConfig();
  // const double adc_width =
  //     vehicle_config.vehicle_param().width() + FLAGS_astar_first_lat_buffer;
  // const double half_adc_width = 0.5 * adc_width;
  // auto obstacles = frame->obstacles();
  // double perl = start_point.l();
  // // const double l_shift = 0.5;  // = Rmin(1-cos(30))
  // const double l_shift = 0.0;  // = Rmin(1-cos(30))
  // perl = block_shift_l_ > 0.0 ? perl - l_shift : perl + l_shift;
  // ADEBUG << "perl - ego l : " << perl - start_point.l();
  // if (TEBTarStatus::YIELD == frame->open_space_info().tar_status()) {
  //   perl = right + half_adc_width + FLAGS_astar_first_lat_buffer * 0.5;
  //   // AINFO << "back perl reset: " << perl;
  // }

  return true;
}

bool TEBPlannerDecider::CheckGoalIsValid(const common::math::Vec2d &point,
                                         const double &ref_heading) {
  if (ROI_xy_boundary_.size() != 4) {
    AERROR << "ROI_xy_boundary_ size error, cannot planning size : "
           << ROI_xy_boundary_.size();
    return false;
  }
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length =
      vehicle_params_.length() + FLAGS_astar_first_long_buffer;
  const double adc_width =
      vehicle_params_.width() + FLAGS_astar_first_lat_buffer;

  // ADC box
  Box2d adc_box1(point, ref_heading, adc_length, adc_width);
  // lwt: adc_length*0.5 means buffer is all front
  // vehicle_params_.length() * 0.5 means buffer is half front and half back
  double shift_distance = vehicle_params_.length() * 0.5 -
                          vehicle_config.vehicle_param().back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(ref_heading),
                  shift_distance * std::sin(ref_heading)};
  adc_box1.Shift(shift_vec);
  const auto &corners = adc_box1.GetAllCorners();
  const auto &origin_point = frame_->open_space_info().origin_point();
  const auto &origin_heading = frame_->open_space_info().origin_heading();
  for (auto corner : corners) {
    corner -= origin_point;
    corner.SelfRotate(-origin_heading);
    if (corner.x() < ROI_xy_boundary_[0] || corner.x() > ROI_xy_boundary_[1] ||
        corner.y() < ROI_xy_boundary_[2] || corner.y() > ROI_xy_boundary_[3]) {
      AERROR << "sample point is  outside of xy boundary of rescue ROI "
             << "corner.x() " << corner.x() << " corner.y() " << corner.y()
             << "x_min " << ROI_xy_boundary_[0] << " x_max "
             << ROI_xy_boundary_[1] << " y_min " << ROI_xy_boundary_[2]
             << " y_max " << ROI_xy_boundary_[3];
      return false;
    }
  }

  Vec2d mutable_p(point.x(), point.y());
  mutable_p -= origin_point;
  mutable_p.SelfRotate(-origin_heading);
  double heading = common::math::NormalizeAngle(ref_heading - origin_heading);
  Box2d adc_box2(mutable_p, heading, adc_length, adc_width);
  Vec2d shift_vec2{shift_distance * std::cos(heading),
                   shift_distance * std::sin(heading)};
  adc_box2.Shift(shift_vec2);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t cnt = 0;
  size_t size_obj = obstacle_id_.size();
  for (const auto &obstacle_linesegments : obstacles_linesegments_vec_) {
    for (const common::math::LineSegment2d &linesegment :
         obstacle_linesegments) {
      if (adc_box2.HasOverlap(linesegment)) {
        if (cnt < size_obj) {
          ADEBUG << "obstacle_id: " << obstacle_id_[cnt];
        }
        return false;
      }
    }
    cnt++;
  }
  return true;
}

bool TEBPlannerDecider::JudgeInPlayStreet() {
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  auto lane_type = util::GetLaneTypeAt(reference_line, check_s);
  AINFO << "cal teb roi lane_type: " << lane_type;
  if (hdmap::Lane::PLAY_STREET == lane_type) {
    return true;
  } else {
    AINFO << "==============NO IN PLAY STREET=============";
    return false;
  }
  return false;
}

// WYQ_Maker same_lang
void TEBPlannerDecider::CalPlayStreetTebRoiLeftAndRightPoint(
    Frame *const frame, const common::SLPoint &adc_position_sl,
    const double &current_road_left_line_l,
    const double &current_road_right_line_l,
    std::vector<common::math::Vec2d> *const left_lane_boundary,
    std::vector<common::math::Vec2d> *const right_lane_boundary) {
  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  const auto &routing_end_point = frame->local_view()
                                      .routing->routing_request()
                                      .waypoint()
                                      .rbegin()
                                      ->pose();
  common::SLPoint routing_end_position_sl;
  reference_line.XYToSL(routing_end_point, &routing_end_position_sl);
  int pullover_start_s = routing_end_position_sl.s() - adc_position_sl.s() -
                         kPulloverSearchExtraS - kEp -
                         kNearRoutingEndPointRoiDis;
  int pullover_end_s = routing_end_position_sl.s() - adc_position_sl.s() +
                       kNearRoutingEndPointRoiDis;

  start_roi_s_ =
      !is_pullover_ready_
          ? config_.teb_roi_decider_config().frenet_boundary_start_s()
          : pullover_start_s;
  end_roi_s_ = !is_pullover_ready_
                   ? config_.teb_roi_decider_config().frenet_boundary_end_s()
                   : pullover_end_s;

  left_roi_l_ = current_road_left_line_l;
  right_roi_l_ = current_road_right_line_l;
  // AINFO << "left_roi_l_: " << left_roi_l_
  // << " right_roi_l_: " << right_roi_l_;

  common::SLPoint left_sl_point = adc_position_sl;
  common::SLPoint right_sl_point = adc_position_sl;
  for (int i = start_roi_s_; i < end_roi_s_; ++i) {
    left_sl_point.set_l(left_roi_l_);
    right_sl_point.set_l(right_roi_l_);

    left_sl_point.set_s(adc_position_sl.s() + i);
    right_sl_point.set_s(adc_position_sl.s() + i);

    common::math::Vec2d left_point;
    common::math::Vec2d right_point;
    // AINFO << "left_sl_point: " << left_sl_point.DebugString();
    // AINFO << "right_sl_point: " << right_sl_point.DebugString();
    reference_line.SLToXY(left_sl_point, &left_point);
    reference_line.SLToXY(right_sl_point, &right_point);

    left_point -= origin_point;
    left_point.SelfRotate(-origin_heading);
    right_point -= origin_point;
    right_point.SelfRotate(-origin_heading);

    // AINFO << "left_point: " << left_point.DebugString();
    // AINFO << "right_point: " << right_point.DebugString();

    left_lane_boundary->emplace_back(left_point);
    right_lane_boundary->emplace_back(right_point);
  }
  return;
}

void TEBPlannerDecider::RoiOffLaneRoutingLogic(
    double *const roi_left_line_l, double *const roi_right_line_l,
    const double &current_road_left_line_l,
    const double &current_road_right_line_l) {
  if (injector_->is_off_lane_depart_ &&
      TEBRoiDeciderConfig::UTURN ==
          config_.teb_roi_decider_config().roi_type()) {
    *roi_left_line_l =
        config_.teb_roi_decider_config().frenet_boundary_left_l();
    *roi_right_line_l = -current_road_right_line_l;
    return;
  }

  *roi_left_line_l = (injector_->is_off_lane_depart_ &&
                      TEBRoiDeciderConfig::UTURN !=
                          config_.teb_roi_decider_config().roi_type())
                         ? current_road_left_line_l
                         : *roi_left_line_l;
  *roi_right_line_l =
      (injector_->is_off_lane_depart_ &&
       TEBRoiDeciderConfig::UTURN !=
           config_.teb_roi_decider_config().roi_type())
          ? config_.teb_roi_decider_config().frenet_boundary_right_l()
          : -current_road_right_line_l;
  return;
}

void TEBPlannerDecider::ExtendCurrentLane(double *const roi_left_line_l,
                                          double *const roi_right_line_l,
                                          const double &extend_dis) {
  *roi_left_line_l = *roi_left_line_l + extend_dis;
  *roi_right_line_l = *roi_right_line_l - extend_dis;
  return;
}

void TEBPlannerDecider::UseConfigParamRoi(double *const roi_left_line_l,
                                          double *const roi_right_line_l) {
  *roi_left_line_l = config_.teb_roi_decider_config().frenet_boundary_left_l();
  *roi_right_line_l =
      config_.teb_roi_decider_config().frenet_boundary_right_l();
  return;
}

bool TEBPlannerDecider::GetTrafficLightCurrentLaneExtendBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  AINFO << "Teb_ROI: TrafficLightCurrentLaneExtendBoundary";

  common::SLPoint adc_position_sl;
  double current_road_left_line_l = 0.0;
  double current_road_right_line_l = 0.0;
  if (!GetAcdOnLaneHalfWidth(&adc_position_sl, &current_road_left_line_l,
                             &current_road_right_line_l)) {
    return false;
  }

  std::vector<common::math::Vec2d> left_lane_boundary;
  std::vector<common::math::Vec2d> right_lane_boundary;

  double roi_left_line_l = current_road_left_line_l;
  double roi_right_line_l = -current_road_right_line_l;

  if (config_.teb_roi_decider_config()
          .enable_traffic_light_extend_current_lane_roi()) {
    ExtendCurrentLane(
        &roi_left_line_l, &roi_right_line_l,
        config_.teb_roi_decider_config().traffic_light_extend_dis());
  }

  AINFO << " has_extend_current_lane"
        << config_.teb_roi_decider_config()
               .enable_traffic_light_extend_current_lane_roi()
        << " extend_dis"
        << config_.teb_roi_decider_config().traffic_light_extend_dis()
        << " roi_left_line_l" << roi_left_line_l << " roi_right_line_l"
        << roi_right_line_l;

  CalPlayStreetTebRoiLeftAndRightPoint(frame, adc_position_sl, roi_left_line_l,
                                       roi_right_line_l, &left_lane_boundary,
                                       &right_lane_boundary);

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }
  std::vector<Vec2d> segment{left_lane_boundary.back(),
                             right_lane_boundary.back()};
  roi_parking_boundary->emplace_back(segment);

  CalXYBoundary(left_lane_boundary, right_lane_boundary, frame);

  return true;
}

// WYQ_Maker
bool TEBPlannerDecider::GenerateNoHdMapFrenetBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  AINFO << "Teb_ROI: PlayStreetNoHdMapFrenetBoundary";

  common::SLPoint adc_position_sl;
  double current_road_left_line_l = 0.0;
  double current_road_right_line_l = 0.0;
  if (!GetAcdOnLaneHalfWidth(&adc_position_sl, &current_road_left_line_l,
                             &current_road_right_line_l)) {
    return false;
  }

  std::vector<common::math::Vec2d> left_lane_boundary;
  std::vector<common::math::Vec2d> right_lane_boundary;

  double left_neighbor_lane_width = 0.0;
  bool has_neighbor_lane =
      CalLeftWidthForNeighborLane(frame, &left_neighbor_lane_width);
  double roi_left_line_l =
      has_neighbor_lane ? current_road_left_line_l + left_neighbor_lane_width
                        : current_road_left_line_l;
  double roi_right_line_l = -current_road_right_line_l;

  if (config_.teb_roi_decider_config().enable_param_roi()) {
    UseConfigParamRoi(&roi_left_line_l, &roi_right_line_l);
  } else {
    RoiOffLaneRoutingLogic(&roi_left_line_l, &roi_right_line_l,
                           current_road_left_line_l, current_road_right_line_l);
  }

  AINFO << " has_neighbor_lane" << has_neighbor_lane
        << " left_neighbor_lane_width" << left_neighbor_lane_width
        << " roi_left_line_l" << roi_left_line_l << " roi_right_line_l"
        << roi_right_line_l;

  CalPlayStreetTebRoiLeftAndRightPoint(frame, adc_position_sl, roi_left_line_l,
                                       roi_right_line_l, &left_lane_boundary,
                                       &right_lane_boundary);

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }
  std::vector<Vec2d> segment{left_lane_boundary.back(),
                             right_lane_boundary.back()};
  roi_parking_boundary->emplace_back(segment);

  CalXYBoundary(left_lane_boundary, right_lane_boundary, frame);

  return true;
}

bool TEBPlannerDecider::GenerateBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_boundary) {
  // lane_ids_.clear();
  // const auto &reference_line_info = frame->reference_line_info().front();
  // for (const auto &id : reference_line_info.TargetLaneId()) {
  //   lane_ids_.push_back(id);
  // }

  // const auto &nearby_path =
  //     frame->reference_line_info().front().reference_line().GetMapPath();
  // common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
  //                                     injector_->vehicle_state()->y()};
  // double adc_s = 0;
  // double adc_l = 0;
  // if (nearby_path.GetProjection(adc_position, &adc_s, &adc_l)) {
  //   hdmap::MapPathPoint adc_point = nearby_path.GetSmoothPoint(adc_s);
  //   adc_ref_heading_ = adc_point.heading();
  // } else {
  //   AINFO << "GetProjection error , use vehicle_state_.heading(); ";
  //   adc_ref_heading_ = vehicle_state_.heading();
  // }

  if (JudgeInPlayStreet()) {
    if (config_.teb_roi_decider_config().enable_frenet_boundary()) {
      GenerateNoHdMapFrenetBoundary(frame, roi_boundary);
      roi_boundary_.clear();
      roi_boundary_ = *roi_boundary;
      AINFO << "use no_hd_map_frenet_roi_boundary " << roi_boundary_.size();
      return true;
    } else {
      GenerateDefaultLaneBoundary(frame, roi_boundary);
      roi_boundary_.clear();
      roi_boundary_ = *roi_boundary;
      AINFO << "use hd_map_road_roi_boundary " << roi_boundary_.size();
    }
  } else if (is_in_city_road_) {
    GenerateDefaultLaneBoundary(frame, roi_boundary);
    roi_boundary_.clear();
    roi_boundary_ = *roi_boundary;
    AINFO << "use hd_map_road_roi_boundary " << roi_boundary_.size();
    return true;
  } else {
    AINFO << "ERROR OpenSpace Not Supported Unknown Scene";
    return false;
  }
  return false;
}

// WYQ_Maker DelayTime TooLarge
void TEBPlannerDecider::CheckAdcIsBlocked(Frame *const frame) {
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  const auto speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  injector_->is_personlike_blocked_ = false;
  injector_->deal_start_block_ = false;
  // this lateral buffer to avoid
  const double later_block_init = 0.0;
  // 1. check collision when plan first starting for safe
  if (rescue_status_->is_first_init() && std::fabs(speed) < kStopSpeed) {
    CheckADCIsBlockedWithSurroundObstacles(
        adc_position, vehicle_state_.heading(), frame,
        preset_long_buffer_ * 0.5, later_block_init, 0.0, true);
  }
  // 2. check block when plan first starting for backwart
  // static int flag_cnt = 0;
  bool block_flag = false;
  rescue_status_->set_is_select_back_pose(false);
  if (rescue_status_->is_first_init() && std::fabs(speed) < kStopSpeed) {
    // deal with static obstacle.
    double front_obstacle_buffer =
        is_in_city_road_ ? rescue_status_->front_obstacle_buffer_city()
                         : rescue_status_->front_obstacle_buffer();
    AERROR << "front_obstacle_buffer: " << front_obstacle_buffer;
    block_flag = CheckADCIsBlockedWithSurroundObstacles(
        adc_position, vehicle_state_.heading(), frame, front_obstacle_buffer,
        later_block_init, 0.0);
    rescue_status_->set_is_near_front_obstacle(block_flag);

    AINFO << "is_sample_adc_blocked_with_front_static_obstacle---"
          << block_flag;
    // deal with dynamic obstacle.
    AINFO << "is_sample_adc_blocked_with_front_dynamic_obstacle: "
          << frame->open_space_info().is_yeild_flag();
    if (block_flag || frame->open_space_info().is_yeild_flag() ||
        TEBTarStatus::YIELD == frame->open_space_info().tar_status()) {
      if (TEBTarStatus::YIELD == frame->open_space_info().tar_status()) {
        AINFO << "frame->tar_status() == YIELD, select back end pose";
      }
      rescue_status_->set_is_select_back_pose(true);
      AINFO << "select backward to drive.";
    } else {
      rescue_status_->set_is_select_back_pose(false);
      AINFO << "select forward to drive.";
    }
    if (is_in_city_road_) {
      rescue_status_->set_is_select_back_pose(true);
    }
    if (last_select_back_pose_ != rescue_status_->is_select_back_pose()) {
      rescue_status_->set_is_switch_select_direction(true);
      AINFO << "switch select direction to drive.";
    }

    last_select_back_pose_ = rescue_status_->is_select_back_pose();

    // WYQ_Mark_Thread
    if (FLAGS_enable_teb_planner_thread && kTempProcessThreadFlages) {
      if (!injector_->rescue_replan()) {
        goals_vector_.clear();
      }
    } else {
      goals_vector_.clear();
    }
  } else {
    // bool is_sample_adc_blocked_with_front_obstacle =
    //     CheckADCIsBlockedWithSurroundObstacles(
    //         adc_position, vehicle_state_.heading(), frame,
    //         rescue_status_->far_front_obstacle_buffer(), 0.0);
    // rescue_status_->set_is_far_front_obstacle(
    //     !is_sample_adc_blocked_with_front_obstacle);
  }

  // 3. check little obj block for expand the kappa when rescue
  if (!injector_->is_personlike_blocked_) {
    injector_->is_personlike_blocked_ =
        CheckADCIsBlockedTypeWithSurroundObstacles(
            adc_position, vehicle_state_.heading(), frame,
            rescue_status_->far_front_obstacle_buffer() * kMultiFrontObjBuffer,
            0.0);
    AINFO << "is_personlike_blocked_: " << injector_->is_personlike_blocked_;
  }
}

void TEBPlannerDecider::SetGoalToEndPose(Frame *const frame) {
  if (goals_vector_.empty()) {
    AERROR << "input error";
    return;
  }

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &reference_line_info = frame->reference_line_info().front();

  const auto &reference_line = reference_line_info.reference_line();

  const auto &target_point = goals_vector_.back().adc_point;
  Vec2d target(target_point.x(), target_point.y());
  rescue_end_point_ = target;
  rescue_end_point_heading_ = goals_vector_.back().adc_heading;
  ADEBUG << "select rescue origin pose x: " << std::setprecision(9)
         << origin_point.x() << " y: " << origin_point.y()
         << " heading: " << origin_heading;
  ADEBUG << "select rescue end pose x: " << std::setprecision(9)
         << rescue_end_point_.x() << " y: " << rescue_end_point_.y()
         << " heading: " << rescue_end_point_heading_;

  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);

  target -= origin_point;
  target.SelfRotate(-origin_heading);

  common::SLPoint target_sl;

  reference_line.XYToSL(rescue_end_point_, &target_sl);
  double target_theta =
      reference_line.GetReferencePoint(target_sl.s()).heading();

  // set rescue end pose
  frame->mutable_open_space_info()->set_end_heading(
      common::math::NormalizeAngle(target_theta));
  ADEBUG << "rescue_end_point_heading_: " << rescue_end_point_heading_;

  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();

  end_pose->emplace_back(target.x());
  end_pose->emplace_back(target.y());
  end_pose->emplace_back(target_theta);

  AINFO << "target_sl.s() " << std::setprecision(9) << target_sl.s()
        << "target_sl.l() " << std::setprecision(9) << target_sl.l()
        << "adc_position_sl.s() " << std::setprecision(9) << adc_position_sl.s()
        << "adc_position_sl.l() " << std::setprecision(9) << adc_position_sl.l()
        << "target.x() " << target.x() << "target.y() " << target.y();
  // end pose velocity set to be speed limit
  end_pose->emplace_back(0.0);
  last_open_space_end_pose_.clear();
  last_open_space_end_pose_.emplace_back(target.x());
  last_open_space_end_pose_.emplace_back(target.y());
  last_open_space_end_pose_.emplace_back(target_theta);
  last_open_space_end_pose_.emplace_back(0.0);
}

void TEBPlannerDecider::SetStopLineParkingEndPose(Frame *const frame) {
  auto traffic_light_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_traffic_light();
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  common::SLPoint adc_position_sl;

  // get nearest reference line
  const auto &reference_line_list = frame->reference_line_info();
  ADEBUG << reference_line_list.size();
  const auto reference_line_info = std::min_element(
      reference_line_list.begin(), reference_line_list.end(),
      [&](const ReferenceLineInfo &ref_a, const ReferenceLineInfo &ref_b) {
        common::SLPoint adc_position_sl_a;
        common::SLPoint adc_position_sl_b;
        ref_a.reference_line().XYToSL(adc_position, &adc_position_sl_a);
        ref_b.reference_line().XYToSL(adc_position, &adc_position_sl_b);
        return std::fabs(adc_position_sl_a.l()) <
               std::fabs(adc_position_sl_b.l());
      });

  const auto &reference_line = reference_line_info->reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  // target_theta is calculted

  const double light_x =
      traffic_light_status->mutable_traffic_light_pose()->x();
  const double light_y =
      traffic_light_status->mutable_traffic_light_pose()->y();
  double target_theta = std::atan2(light_y - adc_init_y, light_x - adc_init_x);

  ADEBUG << "center.x(): " << std::setprecision(9) << light_x;
  ADEBUG << "center.y(): " << std::setprecision(9) << light_y;
  AINFO << "target_theta: " << std::setprecision(9) << target_theta;

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  ADEBUG << "origin_heading: " << std::setprecision(9) << origin_heading;
  Vec2d center(traffic_light_status->mutable_adc_adjust_end_pose()->x(),
               traffic_light_status->mutable_adc_adjust_end_pose()->y());
  AINFO << "center.x(): " << std::setprecision(9) << center.x();
  AINFO << "center.y(): " << std::setprecision(9) << center.y();
  center -= origin_point;
  center.SelfRotate(-origin_heading);
  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();

  end_pose->push_back(center.x());
  end_pose->push_back(center.y());
  end_pose->push_back(target_theta);
  end_pose->push_back(0.0);
}

bool TEBPlannerDecider::GetJunctionBoundary(
    Frame *const frame, const hdmap::Path &nearby_path,
    std::vector<std::vector<common::math::Vec2d>> *const roi_deadend_boundary) {
  const auto &point = common::util::PointFactory::ToPointENU(vehicle_state_);
  const hdmap::HDMap *base_map_ptr = HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  std::vector<Vec2d> point_boundary;
  if (base_map_ptr->GetJunctions(point, kCrossJunctionDist, &junctions) != 0) {
    const std::string msg = "Fail to get junctions from sim_map.";
    AERROR << msg;
    return false;
  }
  if (!SelectJunction(&junctions, point, &junction)) {
    const std::string msg = "interest junction not found";
    AERROR << msg;
    return false;
  }
  if (!GetDeadEndSpot(frame, &junction, &point_boundary)) {
    const std::string msg = "Fail to get junction from map";
    AERROR << msg;
    return false;
  }

  if (point_boundary.size() < kPolygonMinPointNum) {
    AERROR << "boundary size < 3, not a polygon";
    return false;
  }
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  for (size_t i = 0; i < point_boundary.size(); ++i) {
    point_boundary[i] -= origin_point;
    point_boundary[i].SelfRotate(-origin_heading);
  }

  for (size_t i = 0; i < point_boundary.size() - 1; ++i) {
    std::vector<Vec2d> segment{point_boundary[i], point_boundary[i + 1]};
    roi_deadend_boundary->push_back(segment);
  }
  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_deadend_boundary)) {
    AERROR << "FuseLineSegments failed in deadend ROI";
    return false;
  }
  // Get xy boundary
  auto xminmax = std::minmax_element(
      point_boundary.begin(), point_boundary.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      point_boundary.begin(), point_boundary.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);

  if (vehicle_xy.x() < ROI_xy_boundary[0] ||
      vehicle_xy.x() > ROI_xy_boundary[1] ||
      vehicle_xy.y() < ROI_xy_boundary[2] ||
      vehicle_xy.y() > ROI_xy_boundary[3]) {
    ADEBUG << "vehicle outside of xy boundary of deadend ROI";
    return false;
  }
  ROI_xy_boundary_.clear();
  ROI_xy_boundary_ = ROI_xy_boundary;
  return true;
}

bool TEBPlannerDecider::GetDeadEndSpot(Frame *const frame,
                                       JunctionInfoConstPtr *junction,
                                       std::vector<Vec2d> *dead_end_vertices) {
  if (nullptr == frame) {
    ADEBUG << "Invalid frame, fail to GetDeadEndSpotFromMap from frame. ";
    return false;
  }
  auto &junction_point = (*junction)->polygon().points();
  for (size_t i = 0; i < junction_point.size(); ++i) {
    (*dead_end_vertices).emplace_back(junction_point.at(i));
  }
  return true;
}

bool TEBPlannerDecider::SelectJunction(
    std::vector<JunctionInfoConstPtr> *junctions,
    const century::common::PointENU &point,
    JunctionInfoConstPtr *target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AINFO << "No junctions frim map";
    return false;
  }
  Vec2d target_point = {point.x(), point.y()};

  // first use CROSS_ROAD
  for (size_t i = 0; i < junction_num; ++i) {
    AINFO << "type " << junctions->at(i)->junction().type() << " i" << i;
    if (Junction::CROSS_ROAD == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point)) {
        *target_junction = junctions->at(i);
        ADEBUG << "car in the junction";
        return true;
      }
    }
  }

  // second  use COMMON_JUNCTION
  for (size_t i = 0; i < junction_num; ++i) {
    AINFO << "type " << junctions->at(i)->junction().type() << " i" << i;
    if (Junction::COMMON_JUNCTION == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point)) {
        *target_junction = junctions->at(i);
        ADEBUG << "car in the junction";
        return true;
      }
    }
  }
  return false;
}
bool TEBPlannerDecider::SelectDriveAreaJunction(
    Frame *const frame, std::vector<JunctionInfoConstPtr> *junctions,
    const century::common::PointENU &point,
    JunctionInfoConstPtr *target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AINFO << "No junctions frim map";
    return false;
  }
  // SLBoundary adc_sl_boundary;
  // const auto &vehicle_config =
  //     common::VehicleConfigHelper::Instance()->GetConfig();
  // Vec2d vec_to_center((vehicle_config.vehicle_param().front_edge_to_center()
  // -
  //                      vehicle_config.vehicle_param().back_edge_to_center())
  //                      *
  //                         0.5,
  //                     (vehicle_config.vehicle_param().left_edge_to_center() -
  //                      vehicle_config.vehicle_param().right_edge_to_center())
  //                      *
  //                         0.5);
  // // realtime vehicle position
  // Vec2d vehicle_position(frame->vehicle_state().x(),
  //                        frame->vehicle_state().y());
  // Vec2d vehicle_center(vehicle_position +
  //                      vec_to_center.rotate(frame->vehicle_state().heading()));
  // Box2d vehicle_box(vehicle_center, frame->vehicle_state().heading(),
  //                   vehicle_config.vehicle_param().length(),
  //                   vehicle_config.vehicle_param().width());
  // const auto &reference_line_info = frame->reference_line_info().front();
  // const auto &reference_line = reference_line_info.reference_line();
  // if (!reference_line.GetSLBoundary(vehicle_box, &adc_sl_boundary)) {
  //   AINFO << "GetSLBoundary  adc_sl_boundary field";
  //   return false;
  // }
  // // first use Drive Area
  // for (size_t i = 0; i < junction_num; ++i) {
  //   AINFO << "type " << junctions->at(i)->junction().type() << " i" << i;
  //   if (Junction::DRIVE_AREA == junctions->at(i)->junction().type()) {
  //     AERROR << "GetDriveAreaStartS: "
  //            << reference_line_info.GetDriveAreaStartS()
  //            << " adc_sl_boundary_start_s: " << adc_sl_boundary.start_s()
  //            << " GetDriveAreaEndS: " <<
  //            reference_line_info.GetDriveAreaEndS()
  //            << " adc_sl_boundary_end_s: " << adc_sl_boundary.end_s();
  //     if (reference_line_info.GetDriveAreaStartS() < adc_sl_boundary.end_s()
  //     &&
  //         reference_line_info.GetDriveAreaEndS() > adc_sl_boundary.start_s())
  //         {
  //       *target_junction = junctions->at(i);
  //       ADEBUG << "car in the junction";
  //       return true;
  //     }
  //   }
  // }
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  auto front_edge_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();
  auto back_edge_to_center =
      vehicle_config.vehicle_param().back_edge_to_center();
  auto current_heading = frame->vehicle_state().heading();
  Vec2d target_point = {point.x(), point.y()};
  Vec2d target_point_start = {
      point.x() + (front_edge_to_center + kSafeBuffer) * cos(current_heading),
      point.y() + (front_edge_to_center + kSafeBuffer) * sin(current_heading)};
  Vec2d target_point_end = {
      point.x() - (back_edge_to_center + kSafeBuffer) * cos(current_heading),
      point.y() - (back_edge_to_center + kSafeBuffer) * sin(current_heading)};

  for (size_t i = 0; i < junction_num; ++i) {
    AINFO << "type " << junctions->at(i)->junction().type() << " i" << i;
    if (Junction::DRIVE_AREA == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point) ||
          polygon.IsPointIn(target_point_start) ||
          polygon.IsPointIn(target_point_end)) {
        *target_junction = junctions->at(i);
        ADEBUG << "car in the junction";
        return true;
      }
    }
  }

  return false;
}

bool TEBPlannerDecider::GetTrafficLightJunctionBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_deadend_boundary) {
  AINFO << "Teb_ROI: TrafficLightJunction";
  const auto &point = common::util::PointFactory::ToPointENU(vehicle_state_);
  const hdmap::HDMap *base_map_ptr = HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  std::vector<Vec2d> point_boundary;
  if (base_map_ptr->GetJunctions(point, FLAGS_openspace_junction_search_radius,
                                 &junctions) != 0) {
    const std::string msg = "Fail to get junctions from sim_map.";
    AERROR << msg;
    return false;
  }
  if (!SelectJunction(&junctions, point, &junction)) {
    const std::string msg = "interest junction not found";
    AERROR << msg;
    return false;
  }
  if (!GetDeadEndSpot(frame, &junction, &point_boundary)) {
    const std::string msg = "Fail to get junction from map";
    AERROR << msg;
    return false;
  }

  if (point_boundary.size() < kPolygonMinPointNum) {
    AERROR << "boundary size < 3, not a polygon";
    return false;
  }
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  for (size_t i = 0; i < point_boundary.size(); ++i) {
    point_boundary[i] -= origin_point;
    point_boundary[i].SelfRotate(-origin_heading);
  }

  for (size_t i = 0; i < point_boundary.size() - 1; ++i) {
    std::vector<Vec2d> segment{point_boundary[i], point_boundary[i + 1]};
    roi_deadend_boundary->push_back(segment);
  }
  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_deadend_boundary)) {
    AERROR << "FuseLineSegments failed in deadend ROI";
    return false;
  }
  // Get xy boundary
  auto xminmax = std::minmax_element(
      point_boundary.begin(), point_boundary.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      point_boundary.begin(), point_boundary.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);

  if (vehicle_xy.x() < ROI_xy_boundary[0] ||
      vehicle_xy.x() > ROI_xy_boundary[1] ||
      vehicle_xy.y() < ROI_xy_boundary[2] ||
      vehicle_xy.y() > ROI_xy_boundary[3]) {
    ADEBUG << "vehicle outside of xy boundary of deadend ROI";
    return false;
  }
  ROI_xy_boundary_.clear();
  ROI_xy_boundary_ = ROI_xy_boundary;
  return true;
}

bool TEBPlannerDecider::GetDriveAreaJunctionBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_deadend_boundary) {
  AINFO << "Teb_ROI: DriveAreaJunction";
  const auto &point = common::util::PointFactory::ToPointENU(vehicle_state_);
  const hdmap::HDMap *base_map_ptr = HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  std::vector<Vec2d> point_boundary;
  std::vector<Vec2d> point_boundary_temp;
  std::vector<Vec2d> point_boundary_interpolate;
  std::vector<common::SLPoint> sl_point_boundary_interpolate;
  std::vector<common::SLPoint> sl_point_boundary;

  if (base_map_ptr->GetJunctions(point, kCrossJunctionDist, &junctions) != 0) {
    const std::string msg = "Fail to get junctions from sim_map.";
    AERROR << msg;
    return false;
  }
  if (!SelectDriveAreaJunction(frame, &junctions, point, &junction)) {
    const std::string msg = "interest drive area junction not found";
    AERROR << msg;
    return false;
  }
  if (!GetDeadEndSpot(frame, &junction, &point_boundary_temp)) {
    const std::string msg = "Fail to get junction from map";
    AERROR << msg;
    return false;
  }

  if (point_boundary_temp.size() < kPolygonMinPointNum) {
    AERROR << "boundary size < 3, not a polygon";
    return false;
  }
  point_boundary_temp.emplace_back(point_boundary_temp[0]);
  auto interpolate_step =
      config_.teb_roi_decider_config().drive_area_interpolation_step();
  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  AERROR << "point_boundary_temp_size: " << point_boundary_temp.size();
  ReducePointSpacing(point_boundary_temp, interpolate_step,
                     &point_boundary_interpolate);
  for (size_t i = 0; i < point_boundary_interpolate.size(); ++i) {
    common::SLPoint sl_point_temp;
    reference_line.XYToSL(point_boundary_interpolate[i], &sl_point_temp);
    // AERROR << "point_boundary_interpolate.x: " << std::setprecision(9)
    //        << point_boundary_interpolate[i].x()
    //        << " point_boundary_interpolate.y: " << std::setprecision(9)
    //        << point_boundary_interpolate[i].y();
    // AERROR << "sl_point_temp.s: " << sl_point_temp.s()
    //        << " sl_point_temp.l:" << sl_point_temp.l();
    sl_point_boundary_interpolate.emplace_back(sl_point_temp);
  }

  if (!CalDriveAreaSLBoundary(frame, sl_point_boundary_interpolate,
                              roi_deadend_boundary)) {
    AERROR << "can not Get Acd On Lane Half Width";
    return false;
  }

  return true;
}

bool TEBPlannerDecider::GetPlayStreetJunctionBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  AINFO << "Teb_ROI: PlayStreetJunction";

  common::SLPoint adc_position_sl;
  double current_road_left_line_l = 0.0;
  double current_road_right_line_l = 0.0;
  if (!GetAcdOnLaneHalfWidth(&adc_position_sl, &current_road_left_line_l,
                             &current_road_right_line_l)) {
    return false;
  }

  bool has_left_neighbor_lane = GetLeftNeighborLane(frame);

  std::vector<common::math::Vec2d> left_lane_boundary;
  std::vector<common::math::Vec2d> right_lane_boundary;
  CalPlayStreetJunctionTebRoiLeftAndRightPoint(
      frame, has_left_neighbor_lane, adc_position_sl, current_road_left_line_l,
      current_road_right_line_l, &left_lane_boundary, &right_lane_boundary);

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  CalXYBoundary(left_lane_boundary, right_lane_boundary, frame);

  return true;
}

void TEBPlannerDecider::CalPlayStreetJunctionTebRoiLeftAndRightPoint(
    Frame *const frame, const bool &has_left_neighbor_lane,
    const common::SLPoint &adc_position_sl,
    const double &current_road_left_line_l,
    const double &current_road_right_line_l,
    std::vector<common::math::Vec2d> *const left_lane_boundary,
    std::vector<common::math::Vec2d> *const right_lane_boundary) {
  start_roi_s_ = config_.teb_roi_decider_config().frenet_boundary_start_s();
  end_roi_s_ = config_.teb_roi_decider_config().frenet_boundary_end_s();

  double left_add_dis = config_.teb_roi_decider_config()
                            .hd_map_boundary_play_jun_left_l_add_dis();
  double right_add_dis = config_.teb_roi_decider_config()
                             .hd_map_boundary_play_jun_right_l_add_dis();
  left_roi_l_ = has_left_neighbor_lane
                    ? (current_road_left_line_l * 2 + current_road_right_line_l)
                    : (current_road_left_line_l + left_add_dis);
  right_roi_l_ = has_left_neighbor_lane
                     ? -current_road_right_line_l
                     : -current_road_right_line_l - right_add_dis;

  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::SLPoint left_sl_point = adc_position_sl;
  common::SLPoint right_sl_point = adc_position_sl;
  for (int i = start_roi_s_; i < end_roi_s_; ++i) {
    left_sl_point.set_l(left_roi_l_);
    right_sl_point.set_l(right_roi_l_);

    left_sl_point.set_s(adc_position_sl.s() + i);
    right_sl_point.set_s(adc_position_sl.s() + i);

    common::math::Vec2d left_point;
    common::math::Vec2d right_point;
    // AINFO << "left_sl_point: " << left_sl_point.DebugString();
    // AINFO << "right_sl_point: " << right_sl_point.DebugString();
    reference_line.SLToXY(left_sl_point, &left_point);
    reference_line.SLToXY(right_sl_point, &right_point);

    left_point -= origin_point;
    left_point.SelfRotate(-origin_heading);
    right_point -= origin_point;
    right_point.SelfRotate(-origin_heading);

    // AINFO << "left_point: " << left_point.DebugString();
    // AINFO << "right_point: " << right_point.DebugString();

    left_lane_boundary->emplace_back(left_point);
    right_lane_boundary->emplace_back(right_point);
  }
  return;
}

bool TEBPlannerDecider::GetLeftNeighborLane(Frame *const frame) {
  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};

  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_init_position, &adc_position_sl);
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line.GetLaneFromS(adc_position_sl.s(), &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return false;
  }
  if (!lanes.front()->lane().left_neighbor_forward_lane_id().empty() ||
      !lanes.front()->lane().left_neighbor_reverse_lane_id().empty()) {
    AINFO << "Teb_ROI: PlayStreetJunction current_lane_has_left_neighbor_lane";
    return true;
  }
  return false;
}

bool TEBPlannerDecider::CalLeftWidthForNeighborLane(
    Frame *const frame, double *const left_neighbor_lane_width) {
  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};

  const auto &reference_line_info = frame->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_init_position, &adc_position_sl);
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line.GetLaneFromS(adc_position_sl.s(), &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return false;
  }

  if (!lanes.front()->lane().left_neighbor_reverse_lane_id().empty()) {
    hdmap::Id left_neighbor_reverse_lane_id;
    left_neighbor_reverse_lane_id =
        lanes.front()->lane().left_neighbor_reverse_lane_id(0);
    const auto &left_neighbor_reverse_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
            left_neighbor_reverse_lane_id);
    if (nullptr != left_neighbor_reverse_lane) {
      *left_neighbor_lane_width =
          left_neighbor_reverse_lane->GetWidth(adc_position_sl.s());
      return true;
    }
  }

  if (!lanes.front()->lane().left_neighbor_forward_lane_id().empty()) {
    hdmap::Id left_neighbor_forward_lane_id;
    left_neighbor_forward_lane_id =
        lanes.front()->lane().left_neighbor_forward_lane_id(0);
    const auto &left_neighbor_forward_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
            left_neighbor_forward_lane_id);
    if (nullptr != left_neighbor_forward_lane) {
      *left_neighbor_lane_width =
          left_neighbor_forward_lane->GetWidth(adc_position_sl.s());
      return true;
    }
  }

  return false;
}

void TEBPlannerDecider::CalculateFrameObstacles(
    Frame *const frame, size_t *perception_obstacles_num, size_t *point_num,
    std::vector<std::vector<century::common::math::Vec2d>>
        *obstacles_vertices_vec,
    std::vector<std::vector<century::common::math::Vec2d>>
        *pure_obstacles_vertices_vec,
    std::vector<size_t> *const obstacle_points_num) {
  auto *interest_obstacles =
      frame->mutable_open_space_info()->mutable_interest_obstacles();
  for (const auto &obstacle : obstacles_by_frame_->Items()) {
    if (obstacle->IsOutOfOpenSpaceROI()) {
      // AINFO << "obs IsOutOfOpenSpaceROI id: " << obstacle->Id();
      continue;
    }
    if (!FLAGS_enable_use_origin_obstacle) {
      break;
    }
    // if (FLAGS_enable_teb_plan_ingore_dynamic_obs) {
    //   if (!obstacle->IsStatic()) {
    //     continue;
    //   }
    // }

    ADEBUG << "FLAGS_enable_not_use_map_as_boundary "
           << FLAGS_enable_not_use_map_as_boundary;

    if (FLAGS_enable_not_use_map_as_boundary) {
      if (FilterOutObstacleWithoutHdMap(*frame, *obstacle)) {
        continue;
      }
    }

    frame->AddOpenSpaceRoiObstacle(*obstacle);

    interest_obstacles->emplace_back(*obstacle);
    obstacle_id_.emplace_back(obstacle->Id());
    if (!FLAGS_enable_openspace_use_polygon_plan) {
      DontUsePolygonPlan(frame, *obstacle, perception_obstacles_num,
                         obstacles_vertices_vec, pure_obstacles_vertices_vec);
    } else {
      // take vector from polygon
      Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
      if (obstacle_polygon.points().size() < kPolygonMinPointNum) {
        continue;
      }
      UsePolygonPlan(frame, *obstacle, perception_obstacles_num,
                     obstacles_vertices_vec, pure_obstacles_vertices_vec,
                     point_num, obstacle_points_num);
    }
  }
}

// Reduce the dot spacing to below drive_area_interpolate_step
void TEBPlannerDecider::ReducePointSpacing(
    const std::vector<common::math::Vec2d> &point_boundary_temp,
    const double &interpolate_step,
    std::vector<common::math::Vec2d> *const point_boundary_interpolate) {
  for (size_t i = 0; i < point_boundary_temp.size() - 1; ++i) {
    point_boundary_interpolate->emplace_back(point_boundary_temp[i]);
    auto points_distance =
        point_boundary_temp[i].DistanceTo(point_boundary_temp[i + 1]);
    if ((points_distance + kEp) > interpolate_step) {
      int n = (points_distance + kEp) / interpolate_step;
      double step_x =
          (point_boundary_temp[i + 1].x() - point_boundary_temp[i].x()) /
          (n + 1);
      double step_y =
          (point_boundary_temp[i + 1].y() - point_boundary_temp[i].y()) /
          (n + 1);
      for (int j = 1; j <= n; ++j) {
        common::math::Vec2d point_interpolate;
        point_interpolate.set_x(point_boundary_temp[i].x() + j * step_x);
        point_interpolate.set_y(point_boundary_temp[i].y() + j * step_y);
        point_boundary_interpolate->emplace_back(point_interpolate);
      }
    }
  }
  point_boundary_interpolate->emplace_back(point_boundary_temp.back());
}

bool TEBPlannerDecider::CalDriveAreaSLBoundary(
    Frame *const frame,
    const std::vector<common::SLPoint> &sl_point_boundary_interpolate,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();

  const auto &routing_end_point = frame->local_view()
                                      .routing->routing_request()
                                      .waypoint()
                                      .rbegin()
                                      ->pose();
  common::SLPoint routing_end_position_sl, adc_position_sl;
  double current_road_left_line_l = 0.0, current_road_right_line_l = 0.0;
  std::vector<common::SLPoint> sl_point_left_boundary_filter,
      sl_point_right_boundary_filter;
  if (!GetAcdOnLaneHalfWidth(&adc_position_sl, &current_road_left_line_l,
                             &current_road_right_line_l)) {
    return false;
  }
  double left_neighbor_lane_width = 0.0;
  bool has_neighbor_lane =
      CalLeftWidthForNeighborLane(frame, &left_neighbor_lane_width);
  double roi_left_line_l =
      has_neighbor_lane ? current_road_left_line_l + left_neighbor_lane_width
                        : current_road_left_line_l;
  double roi_right_line_l = -current_road_right_line_l;
  RoiOffLaneRoutingLogic(&roi_left_line_l, &roi_right_line_l,
                         current_road_left_line_l, current_road_right_line_l);

  reference_line.XYToSL(routing_end_point, &routing_end_position_sl);
  int pullover_start_s = routing_end_position_sl.s() - adc_position_sl.s() -
                         kPulloverSearchExtraS - kEp -
                         kNearRoutingEndPointRoiDis;
  int pullover_end_s = routing_end_position_sl.s() - adc_position_sl.s() +
                       kNearRoutingEndPointRoiDis;
  start_roi_s_ =
      !is_pullover_ready_
          ? config_.teb_roi_decider_config().frenet_boundary_start_s()
          : pullover_start_s;
  end_roi_s_ = !is_pullover_ready_
                   ? config_.teb_roi_decider_config().frenet_boundary_end_s()
                   : pullover_end_s;
  AERROR << "sl_point_boundary_interpolate_size: "
         << sl_point_boundary_interpolate.size();
  for (size_t i = 0; i < sl_point_boundary_interpolate.size(); ++i) {
    // AERROR << "sl_point_boundary_interpolate_s: "
    //        << sl_point_boundary_interpolate[i].s()
    //        << " sl_point_boundary_interpolate_l: "
    //        << sl_point_boundary_interpolate[i].l();
    // AERROR << " start_roi_s_: " << (adc_position_sl.s() + start_roi_s_)
    //        << " end_roi_s_: " << (adc_position_sl.s() + end_roi_s_);
    if (sl_point_boundary_interpolate[i].s() >
            (adc_position_sl.s() + end_roi_s_) ||
        sl_point_boundary_interpolate[i].s() <
            (adc_position_sl.s() + start_roi_s_)) {
      continue;
    }
    if (sl_point_boundary_interpolate[i].l() > 0) {
      sl_point_left_boundary_filter.emplace_back(
          sl_point_boundary_interpolate[i]);
    } else {
      sl_point_right_boundary_filter.emplace_back(
          sl_point_boundary_interpolate[i]);
    }
  }

  std::sort(sl_point_left_boundary_filter.begin(),
            sl_point_left_boundary_filter.end(),
            [](const common::SLPoint &left, const common::SLPoint &right) {
              return left.s() > right.s();
            });
  std::sort(sl_point_right_boundary_filter.begin(),
            sl_point_right_boundary_filter.end(),
            [](const common::SLPoint &left, const common::SLPoint &right) {
              return left.s() > right.s();
            });
  CalDriveAreaSegmentBoundary(
      frame, sl_point_left_boundary_filter, sl_point_right_boundary_filter,
      adc_position_sl, roi_left_line_l, roi_right_line_l, roi_parking_boundary);
  return true;
}

void TEBPlannerDecider::CalDriveAreaSegmentBoundary(
    Frame *const frame,
    const std::vector<common::SLPoint> &sl_point_left_boundary_filter,
    const std::vector<common::SLPoint> &sl_point_right_boundary_filter,
    const common::SLPoint &adc_position_sl, const double &roi_left_line_l,
    const double &roi_right_line_l,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &reference_line_info = frame_->reference_line_info().front();
  const auto &reference_line = reference_line_info.reference_line();
  std::vector<century::common::math::Vec2d> left_drive_area_boundary,
      right_drive_area_boundary;
  common::SLPoint left_sl_point = adc_position_sl,
                  right_sl_point = adc_position_sl;
  // for (size_t i = 0; i < sl_point_right_boundary_filter.size(); ++i) {
  //   AERROR << "sl_point_right_boundary_filter.s: "
  //          << sl_point_right_boundary_filter[i].s()
  //          << " sl_point_right_boundary_filter.l: "
  //          << sl_point_right_boundary_filter[i].l();
  // }

  for (int i = start_roi_s_; i < end_roi_s_; ++i) {
    CalDriveAreaROIL(sl_point_left_boundary_filter,
                     sl_point_right_boundary_filter, adc_position_sl.s() + i,
                     roi_left_line_l, roi_right_line_l, &left_roi_l_,
                     &right_roi_l_);
    left_sl_point.set_l(left_roi_l_);
    right_sl_point.set_l(right_roi_l_);
    left_sl_point.set_s(adc_position_sl.s() + i);
    right_sl_point.set_s(adc_position_sl.s() + i);
    // AERROR << "left_sl_point.s: " << left_sl_point.s()
    //        << " left_sl_point.l: " << left_sl_point.l()
    //        << " right_sl_point.l: " << right_sl_point.l();

    common::math::Vec2d left_point, right_point;
    // AINFO << "left_sl_point: " << left_sl_point.DebugString();
    // AINFO << "right_sl_point: " << right_sl_point.DebugString();
    reference_line.SLToXY(left_sl_point, &left_point);
    reference_line.SLToXY(right_sl_point, &right_point);

    left_point -= origin_point;
    left_point.SelfRotate(-origin_heading);
    right_point -= origin_point;
    right_point.SelfRotate(-origin_heading);

    // AINFO << "left_point: " << left_point.DebugString();
    // AINFO << "right_point: " << right_point.DebugString();

    left_drive_area_boundary.emplace_back(left_point);
    right_drive_area_boundary.emplace_back(right_point);
  }
  size_t left_drive_area_boundary_last_index =
      left_drive_area_boundary.size() - 1;
  for (size_t i = left_drive_area_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_drive_area_boundary[i],
                               left_drive_area_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }
  std::vector<Vec2d> segment_start{left_drive_area_boundary[0],
                                   right_drive_area_boundary[0]};
  roi_parking_boundary->emplace_back(segment_start);

  size_t right_drive_area_boundary_last_index =
      right_drive_area_boundary.size() - 1;
  for (size_t i = 0; i < right_drive_area_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_drive_area_boundary[i],
                               right_drive_area_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }
  std::vector<Vec2d> segment_end{
      right_drive_area_boundary[right_drive_area_boundary_last_index],
      left_drive_area_boundary[left_drive_area_boundary_last_index]};
  roi_parking_boundary->emplace_back(segment_end);
  CalXYBoundary(left_drive_area_boundary, right_drive_area_boundary, frame);
  AERROR << "Drive Area finish";
  return;
}

void TEBPlannerDecider::CalDriveAreaROIL(
    const std::vector<common::SLPoint> &sl_point_left_boundary_filter,
    const std::vector<common::SLPoint> &sl_point_right_boundary_filter,
    const double &current_s, const double &roi_left_line_l,
    const double &roi_right_line_l, double *const left_roi_l_,
    double *const right_roi_l_) {
  *left_roi_l_ = roi_left_line_l;
  *right_roi_l_ = roi_right_line_l;
  auto interpolate_step =
      config_.teb_roi_decider_config().drive_area_interpolation_step();

  const auto frenet_boundary_left_l =
      config_.teb_roi_decider_config().frenet_boundary_left_l();
  const auto frenet_boundary_right_l =
      config_.teb_roi_decider_config().frenet_boundary_right_l();
  if (sl_point_left_boundary_filter.size() &&
      current_s >= (sl_point_left_boundary_filter.back().s()) &&
      current_s <= (sl_point_left_boundary_filter.front().s())) {
    auto left_roi_l_temp = frenet_boundary_left_l;
    for (size_t left_index = 0;
         left_index < sl_point_left_boundary_filter.size(); ++left_index) {
      if (sl_point_left_boundary_filter[left_index].s() >=
              (current_s - interpolate_step) &&
          sl_point_left_boundary_filter[left_index].s() <=
              (current_s + interpolate_step) &&
          sl_point_left_boundary_filter[left_index].l() < left_roi_l_temp) {
        left_roi_l_temp = sl_point_left_boundary_filter[left_index].l();
      }
    }
    *left_roi_l_ = std::max(roi_left_line_l, left_roi_l_temp);
  }
  if (sl_point_right_boundary_filter.size() &&
      current_s >= (sl_point_right_boundary_filter.back().s()) &&
      current_s <= (sl_point_right_boundary_filter.front().s())) {
    auto right_roi_l_temp = frenet_boundary_right_l;
    for (size_t right_index = 0;
         right_index < sl_point_right_boundary_filter.size(); ++right_index) {
      if (sl_point_right_boundary_filter[right_index].s() >=
              (current_s - interpolate_step) &&
          sl_point_right_boundary_filter[right_index].s() <=
              (current_s + interpolate_step) &&
          sl_point_right_boundary_filter[right_index].l() > right_roi_l_temp) {
        right_roi_l_temp = sl_point_right_boundary_filter[right_index].l();
      }
    }
    *right_roi_l_ = std::min(roi_right_line_l, right_roi_l_temp);
    // AERROR << " roi_right_line_l: " << roi_right_line_l
    //        << " right_roi_l_temp: " << right_roi_l_temp;
  }
}

}  // namespace planning
}  // namespace century
