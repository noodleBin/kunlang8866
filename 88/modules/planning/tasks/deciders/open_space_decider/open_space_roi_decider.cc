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

#include "modules/planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"

#include <float.h>
#include <memory>
#include <utility>

#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"

namespace century {
namespace planning {

namespace {
constexpr double kSpeedRatio = 0.2;  // after adjust speed is 10% of speed limit
constexpr double kAdcCheckBuffer = 0.3;
constexpr double kEp = 0.1;
// For Rescue Scenario
constexpr double kRescueLatCost = 10;
constexpr double kRescueLonCost = 2;
constexpr double kRescueObstacleCost = 200;
constexpr double kBaseCost = 500;
constexpr double kCollisionCost = 10086;
constexpr double kFrontBlockedCost = 200.0;
constexpr int kMinGoalNum = 2;
constexpr double kIngoreDist = 18;
constexpr double kLonSampleStartS = 5;
constexpr double kLonSampleEndS = 15;
constexpr double kLonSampleInterval = 1.5;
constexpr double kLatSampleInterval = 0.3;

constexpr double kLargeDist = 100.0;
constexpr double kInParkingLot = 2.0;
constexpr double kCrossJunctionDist = 50.0;
constexpr double kStopSpeed = 0.1;
constexpr int kReplanCount = 30;
constexpr int kBlockCount = 5;
constexpr double kIngoreRoadBoundaryDist = 12.0;
constexpr int kMinLaneBoundaryPointSize = 2;
constexpr double kShiftDist = 2.5;
constexpr double kCheckDist = 15.0;
constexpr double kBoundaryInterval = 12.0;
constexpr double kBoundaryLargeInterval = 12.0;
constexpr double kValidDistance = 100.0;
constexpr double kSecondStartS = 10.0;
constexpr double kSecondEndS = 17.0;
constexpr double kSecondSampleS = 2.0;

constexpr double kInValidDistance = 2.0;
constexpr double kDynamicObstacleSpeed = 2.0;
constexpr double kPreferL = 0.3;
constexpr int kStopTooLongCount = 400;
}  // namespace
using century::common::ErrorCode;
using century::common::PointENU;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
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

OpenSpaceRoiDecider::OpenSpaceRoiDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);

  vehicle_params_ =
      century::common::VehicleConfigHelper::GetConfig().vehicle_param();

  temp_state_.set_x(injector->vehicle_state()->x());
  temp_state_.set_y(injector->vehicle_state()->y());
  first_enter_park_and_go_scenario_ = true;
}

Status OpenSpaceRoiDecider::Process(Frame *frame) {
  if (nullptr == frame) {
    const std::string msg =
        "Invalid frame, fail to process the OpenSpaceRoiDecider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  vehicle_state_ = frame->vehicle_state();
  obstacles_by_frame_ = frame->GetObstacleList();
  // parking space four corner points
  std::array<Vec2d, 4> spot_vertices;
  // dead end polygon points
  std::vector<Vec2d> dead_end_vertices;

  Path nearby_path;

  // @brief vector of different obstacle consisting of vertice points.The
  // obstacle and the vertices order are in counter-clockwise order
  // std::vector<common::math::Vec2d> line_segment
  // std::vector<std::vector<common::math::Vec2d>> polygon
  std::vector<std::vector<Vec2d>> roi_boundary;
  const auto &roi_type = config_.open_space_roi_decider_config().roi_type();
  // initialize first_enter_park_and_go_scenario_
  std::string error_msg;
  if (OpenSpaceRoiDeciderConfig::PARK_AND_GO != roi_type) {
    first_enter_park_and_go_scenario_ = true;
    target_offset_ = config_.open_space_roi_decider_config().end_pose_s_distance();
  }
  if (OpenSpaceRoiDeciderConfig::PARKING == roi_type) {          // parking scenario
    if (Status::OK() != ProcessParkingScenario(spot_vertices, nearby_path,
                                               roi_boundary, error_msg)) {
      AERROR << error_msg;
      return Status(ErrorCode::PLANNING_ERROR, error_msg);
    }
  } else if (OpenSpaceRoiDeciderConfig::DEAD_END == roi_type) {
    // TODO
  } else if (OpenSpaceRoiDeciderConfig::PULL_OVER == roi_type) {
    // TODO
  } else if (OpenSpaceRoiDeciderConfig::PARK_AND_GO == roi_type) {  // park and go scenario
    nearby_path =
        frame_->reference_line_info().front().reference_line().GetMapPath();
    if (Status::OK() != ProcessParkAndGoScenario(spot_vertices, nearby_path,
                                                 roi_boundary, error_msg)) {
      AERROR << error_msg;
      return Status(ErrorCode::PLANNING_ERROR, error_msg);
    }
  } else if (OpenSpaceRoiDeciderConfig::TRAFFIC_LIGHT == roi_type) {
    // TODO
  } else if (OpenSpaceRoiDeciderConfig::RESCUE == roi_type) {     // rescue scenario
    nearby_path =
        frame_->reference_line_info().front().reference_line().GetMapPath();
    if (Status::OK() !=
        ProcessRescueScenario(nearby_path, roi_boundary, error_msg)) {
      AERROR << error_msg;
      return Status(ErrorCode::PLANNING_ERROR, error_msg);
    }
  } else {                                       // unsupport scenario
    const std::string msg =
        "chosen open space roi secenario type not implemented";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!FormulateBoundaryConstraints(roi_boundary, frame)) {
    const std::string msg = "Fail to formulate boundary constraints";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

Status OpenSpaceRoiDecider::ProcessParkAndGoScenario(
    std::array<Vec2d, 4>& spot_vertices, hdmap::Path& nearby_path,
    std::vector<std::vector<Vec2d>>& roi_boundary, std::string& msg) {
  if (first_enter_park_and_go_scenario_) {
    target_parking_spot_ = nullptr;
    target_parking_spot_id_.empty();
  }
  first_enter_park_and_go_scenario_ = false;
  if (!injector_->planning_context()
           ->planning_status()
           .park_and_go()
           .has_adc_init_position()) {
    msg = "ADC initial position is unavailable";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (frame_->reference_line_info().empty()) {
    msg = "reference line is empty";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SetOriginFromADC(frame_, nearby_path)) {
    msg = "failed to get origin pose";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SetParkAndGoEndPose(frame_)) {
    msg = "failed to get park_and_go end pose";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // check whether vehicle is in parking space
  bool is_in_parkinglot =
      IsInParkingLot(vehicle_state_.x(), vehicle_state_.y(),
                     vehicle_state_.heading(), nearby_path, &spot_vertices);
  if (is_in_parkinglot) {
    if (!GetParkingBoundary(frame_, spot_vertices, nearby_path,
                            &roi_boundary)) {
      msg = "Fail to get parking boundary from map2";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  } else {
    msg = "is not in parking lot";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status OpenSpaceRoiDecider::ProcessRescueScenario(
    const hdmap::Path& nearby_path,
    std::vector<std::vector<Vec2d>>& roi_boundary, std::string& msg) {
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  open_space_info_ptr->set_is_reverse(IsReverseDriving());
  // for trajectory partition, judge gear
  openspace_common_->set_is_reverse(IsReverseDriving());
  OpenspaceCommon::set_openspace_reason(injector_->openspace_reason());
  SetRescueOriginPose(frame_, nearby_path);
  SetRescueEndPose(frame_);
  if (goals_vector_.empty()) {
    if (is_try_to_second_plan_) {
      AERROR << "try second plan, reuse last info.";
      ReuseLastInfo(frame_);
      is_try_to_second_plan_ = false;
    } else {
      msg = "Fail to find a valid rescue end pose";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  if (is_try_to_second_plan_) {
    if (GetRescueBoundary(frame_, nearby_path, &roi_boundary) &&
        !is_already_second_plan_) {
      is_already_second_plan_ = true;
      frame_->mutable_open_space_info()->set_is_ready_to_second_plan(true);
    } else if (is_already_second_plan_) {
      ReuseLastInfo(frame_);
      frame_->mutable_open_space_info()->set_is_ready_to_second_plan(true);
    } else {
      AERROR << "Get Rescue Preview Boundary error";
    }
  } else if (!GetRescueBoundary(frame_, nearby_path, &roi_boundary)) {
    msg = "Fail to get rescue boundary from map";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status OpenSpaceRoiDecider::ProcessParkingScenario(
    std::array<Vec2d, 4>& spot_vertices, hdmap::Path& nearby_path,
    std::vector<std::vector<Vec2d>>& roi_boundary, std::string& msg) {
  target_parking_spot_ = nullptr;
  target_parking_spot_id_.empty();
  const auto& routing_request = frame_->local_view().routing->routing_request();
  if (routing_request.has_parking_info() &&
      routing_request.parking_info().has_parking_space_id()) {
    target_parking_spot_id_ = routing_request.parking_info().parking_space_id();
  } else {
    msg = "Failed to get parking space id from routing";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // get parking space corner points and path
  if (!GetParkingSpot(frame_, &spot_vertices, &nearby_path)) {
    msg = "Failed to get parking spot";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // spot_vertices (left_down, left_top, right_top, right_down)
  if (!SetOrigin(frame_, spot_vertices)) {
    msg = "Failed to get origin pose";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // set end pose in origin axis
  if (!SetParkingSpotEndPose(frame_, spot_vertices)) {
    msg = "Failed to get parking space end pose";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // generate roi_boundary
  if (!GetParkingBoundary(frame_, spot_vertices, nearby_path, &roi_boundary)) {
    msg = "Failed to get parking boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

bool OpenSpaceRoiDecider::SelectTargetDeadEndJunction(
    std::vector<JunctionInfoConstPtr> *junctions,
    const century::common::PointENU &dead_end_point,
    JunctionInfoConstPtr *target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AERROR << "No junctions from map";
    return false;
  }
  Vec2d target_point = {dead_end_point.x(), dead_end_point.y()};

  // junction may not  once  match successfully
  for (size_t i = 0; i < junction_num; ++i) {
    if (Junction::DEAD_END == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point)) {
        *target_junction = junctions->at(i);
        AERROR << "car in the deadend junction";
        return true;
      }
    }
  }
  return false;
}

bool OpenSpaceRoiDecider::SelectJunction(
    std::vector<JunctionInfoConstPtr> *junctions,
    const century::common::PointENU &point,
    JunctionInfoConstPtr *target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AERROR << "No junctions frim map";
    return false;
  }
  Vec2d target_point = {point.x(), point.y()};

  // first use CROSS_ROAD
  for (size_t i = 0; i < junction_num; ++i) {
    if (Junction::CROSS_ROAD == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point)) {
        *target_junction = junctions->at(i);
        AINFO << "car in the junction";
        return true;
      }
    }
  }

  // second  use COMMON_JUNCTION
  for (size_t i = 0; i < junction_num; ++i) {
    if (Junction::COMMON_JUNCTION == junctions->at(i)->junction().type()) {
      Polygon2d polygon = junctions->at(i)->polygon();
      if (polygon.IsPointIn(target_point)) {
        *target_junction = junctions->at(i);
        AINFO << "car in the junction";
        return true;
      }
    }
  }
  return false;
}

bool OpenSpaceRoiDecider::IsReverseDriving() {
  if (nullptr == injector_ || nullptr == injector_->vehicle_state() ||
      nullptr == frame_ ||
      nullptr == frame_->local_view().localization_estimate) {
    AERROR << __func__ << ", input is nullptr, return";
    return false;
  }
  if ((std::abs(injector_->vehicle_state()->heading()) -
       std::abs(frame_->local_view().localization_estimate->pose().heading())) >
      M_PI_2) {
    return true;
  }
  return false;
}

// set origin pose
bool OpenSpaceRoiDecider::SetOriginFromADC(Frame *const frame,
                                           const hdmap::Path &nearby_path) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return false;
  }

  const auto &park_and_go_status =
      injector_->planning_context()->planning_status().park_and_go();

  Vec2d adc_init_position = common::util::PointFactory().ToVec2d(
      park_and_go_status.adc_init_position());
  double init_s = 0.0, init_l = 0.0;
  nearby_path.GetNearestPoint(adc_init_position, &init_s, &init_l);
  hdmap::MapPathPoint smooth_point = nearby_path.GetSmoothPoint(init_s);

  frame->mutable_open_space_info()->set_origin_heading(smooth_point.heading());
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(smooth_point.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(smooth_point.y());
  return true;
}

void OpenSpaceRoiDecider::SetOriginFromAdcPose(Frame *const frame,
                                               const hdmap::Path &nearby_path) {
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const double adc_init_heading = injector_->vehicle_state()->heading();
  Vec2d adc_init_position = {adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  std::vector<Vec2d> adc_corners;
  adc_box.GetAllCorners(&adc_corners);
  for (size_t i = 0; i < adc_corners.size(); ++i) {
    ADEBUG << "ADC [" << i << "]x: " << std::setprecision(9)
           << adc_corners[i].x();
    ADEBUG << "ADC [" << i << "]y: " << std::setprecision(9)
           << adc_corners[i].y();
  }
  auto left_top = adc_corners[3];

  ADEBUG << "left_top x: " << std::setprecision(9) << left_top.x();
  ADEBUG << "left_top y: " << std::setprecision(9) << left_top.y();

  // rotate the points to have the lane to be horizontal to x axis positive
  // direction and scale them base on the origin point
  // heading angle
  double heading;
  if (!nearby_path.GetHeadingAlongPath(left_top, &heading)) {
    AERROR << "fail to get heading on reference line";
    return;
  }

  frame->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(heading));
  ADEBUG << "heading: " << heading;
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDecider::SetDeadEndOrigin(
    Frame *const frame, const std::vector<Vec2d> &dead_end_vertices) {
  auto last_point = dead_end_vertices.back();
  auto first_point = dead_end_vertices.front();
  Vec2d heading_vec = last_point - first_point;
  frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(
      first_point.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(
      first_point.y());
}

// set origin point and origin heading
bool OpenSpaceRoiDecider::SetOrigin(
    Frame *const frame, const std::array<Vec2d, 4> &vertices) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return false;
  }
  auto left_down = vertices[0];
  auto left_top = vertices[1];
  Vec2d heading_vec = left_top - left_down;
  frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_down.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_down.y());
  return true;
}

void OpenSpaceRoiDecider::SetDeadEndPose(
    Frame *const frame, const std::vector<Vec2d> &dead_end_vertices) {
  // the target point should be in the adjacent lanes based the map,
  // curret map is rectangle, vertice is  anti-clockwise
  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  const auto &target_point = frame->local_view()
                                 .routing->routing_request()
                                 .dead_end_info()
                                 .target_point();
  Vec2d end_point = {target_point.x(), target_point.y()};
  // coordinate transfer
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  auto first_point = dead_end_vertices[0];
  auto second_point = dead_end_vertices[1];
  end_point -= origin_point;
  end_point.SelfRotate(-origin_heading);
  first_point -= origin_point;
  first_point.SelfRotate(-origin_heading);
  second_point -= origin_point;
  second_point.SelfRotate(-origin_heading);
  double parking_spot_heading = (first_point - second_point).Angle();

  end_pose->emplace_back(end_point.x());
  end_pose->emplace_back(end_point.y());
  end_pose->emplace_back(parking_spot_heading);
  end_pose->emplace_back(0.0);
}

// set end pose in origin axis
bool OpenSpaceRoiDecider::SetParkingSpotEndPose(
    Frame *const frame, const std::array<Vec2d, 4> &vertices) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return false;
  }
  auto left_down = vertices[0];
  auto left_top = vertices[1];
  auto right_top = vertices[2];
  auto right_down = vertices[3];

  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  // End pose is set in normalized boundary
  // world ---> origin
  left_down -= origin_point;
  left_down.SelfRotate(-origin_heading);
  left_top -= origin_point;
  left_top.SelfRotate(-origin_heading);
  right_top -= origin_point;
  right_top.SelfRotate(-origin_heading);
  right_down -= origin_point;
  right_down.SelfRotate(-origin_heading);

  // parking space heading in origin axis
  double parking_spot_heading = (left_top - left_down).Angle();
  if ((right_down - left_down).Length() > (left_top - left_down).Length()) {
    if (ParkingSpacePosition::LEFT == parking_space_position_) {
      parking_spot_heading = (right_down - left_down).Angle();
    } else if (ParkingSpacePosition::RIGHT == parking_space_position_) {
      parking_spot_heading = (left_down - right_down).Angle();
    } else {
      parking_spot_heading = (left_down - right_down).Angle();
    }
  }

  const double parking_depth_buffer =
      config_.open_space_roi_decider_config().parking_depth_buffer();
  CHECK_GE(parking_depth_buffer, 0.0);
  bool parking_inwards =
      config_.open_space_roi_decider_config().parking_inwards();
  // end pose in origin axis
  double end_x = (left_down.x() + left_top.x() + right_top.x() + right_down.x()) * 0.25;
  double end_y = (left_down.y() + left_top.y() + right_top.y() + right_down.y()) * 0.25;
  double shift = vehicle_params_.length() * 0.5 - vehicle_params_.back_edge_to_center();
  if (parking_inwards) {
    parking_spot_heading = common::math::NormalizeAngle(parking_spot_heading + M_PI);
    end_x -= parking_depth_buffer * cos(parking_spot_heading);
    end_y -= parking_depth_buffer * sin(parking_spot_heading);
  } else {
    end_x += parking_depth_buffer * cos(parking_spot_heading);
    end_y += parking_depth_buffer * sin(parking_spot_heading);
  }
  // shift to planning center
  end_x -= shift * cos(parking_spot_heading);
  end_y -= shift * sin(parking_spot_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->emplace_back(end_x);
  end_pose->emplace_back(end_y);
  end_pose->emplace_back(parking_spot_heading);
  end_pose->emplace_back(0.0);
  return true;
}

void OpenSpaceRoiDecider::SetPullOverSpotEndPose(Frame *const frame) {
  const auto &pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const double pull_over_x = pull_over_status.position().x();
  const double pull_over_y = pull_over_status.position().y();
  double pull_over_theta = pull_over_status.theta();

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d center(pull_over_x, pull_over_y);
  center -= origin_point;
  center.SelfRotate(-origin_heading);
  pull_over_theta =
      common::math::NormalizeAngle(pull_over_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->emplace_back(center.x());
  end_pose->emplace_back(center.y());
  end_pose->emplace_back(pull_over_theta);
  end_pose->emplace_back(0.0);
}

// generate end pose
bool OpenSpaceRoiDecider::SetParkAndGoEndPose(Frame *const frame) {
  if (nullptr == frame) {
    AERROR << "frame is nullptr";
    return false;
  }

  auto park_and_go_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_park_and_go();

  const Vec2d adc_position = common::util::PointFactory::ToVec2d(
      park_and_go_status->adc_init_position());

  const auto &reference_line_list = frame->reference_line_info();
  if (reference_line_list.empty()) {
    AERROR << "reference line list is empty";
    return false;
  }
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
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);

  const double target_s = adc_position_sl.s() + target_offset_;
  auto *previous_frame = injector_->frame_history()->Latest();
  if (nullptr != previous_frame &&
      !previous_frame->open_space_info().open_space_provider_success()) {
    target_offset_ = std::fmax(0.0, target_offset_ - 2.0);
  }

  const auto reference_point = reference_line.GetReferencePoint(target_s);
  const double target_x = reference_point.x();
  const double target_y = reference_point.y();
  double target_theta = reference_point.heading();

  park_and_go_status->mutable_adc_adjust_end_pose()->set_x(target_x);
  park_and_go_status->mutable_adc_adjust_end_pose()->set_y(target_y);

  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  Vec2d center(target_x, target_y);
  // world --> origin
  center -= origin_point;
  center.SelfRotate(-origin_heading);
  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();

  // set end pose
  end_pose->emplace_back(center.x());
  end_pose->emplace_back(center.y());
  end_pose->emplace_back(target_theta);

  // end pose velocity set to be speed limit
  double target_speed = reference_line.GetSpeedLimitFromS(target_s);
  end_pose->emplace_back(kSpeedRatio * target_speed);

  // set end_pose based on previous frame
  if (nullptr != previous_frame &&
      previous_frame->open_space_info().is_on_open_space_trajectory() &&
      !previous_frame->open_space_info().open_space_end_pose().empty()) {
    AINFO << "set end pose based on previous frame";
    end_pose->clear();
    *end_pose = previous_frame->open_space_info().open_space_end_pose();
  }
  return true;
}

// set rescue origin pose
void OpenSpaceRoiDecider::SetRescueOriginPose(Frame *const frame,
                                              const hdmap::Path &nearby_path) {
  const auto &rescue_status =
      injector_->planning_context()->planning_status().rescue();

  const double adc_init_x = rescue_status.adc_init_position().x();
  const double adc_init_y = rescue_status.adc_init_position().y();
  double adc_init_heading = rescue_status.adc_init_heading();
  Vec2d adc_init_position = {adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  // ADC box
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  double shift_distance =
      adc_length * 0.5 - vehicle_params_.back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(adc_init_heading),
                  shift_distance * std::sin(adc_init_heading)};
  adc_box.Shift(shift_vec);
  // get vertices from ADC box
  std::vector<Vec2d> adc_corners;
  adc_box.GetAllCorners(&adc_corners);
  for (size_t i = 0; i < adc_corners.size(); ++i) {
    ADEBUG << "ADC [" << i << "]x: " << std::setprecision(9)
           << adc_corners[i].x();
    ADEBUG << "ADC [" << i << "]y: " << std::setprecision(9)
           << adc_corners[i].y();
  }

  auto left_top = adc_corners[3];
  ADEBUG << "left_top x: " << std::setprecision(9) << left_top.x();
  ADEBUG << "left_top y: " << std::setprecision(9) << left_top.y();

  // operation condition origin heading is special
  if (OpenspaceReason::OPERATION == injector_->openspace_reason()) {
    if (std::abs(common::math::NormalizeAngle(adc_init_heading)) < M_PI_2) {
      AERROR << "rotate adc heading toward west direction.";
      adc_init_heading = -adc_init_heading;
    }
  }
  frame->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(adc_init_heading));
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDecider::SetRescueEndPose(Frame *const frame) {
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();

  ADEBUG << "ADC position (x): " << std::setprecision(9) << adc_init_x;
  ADEBUG << "ADC position (y): " << std::setprecision(9) << adc_init_y;

  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  common::SLPoint adc_position_sl;
  const auto &reference_line_info = frame->reference_line_info().front();
  lane_ids_.clear();
  for (const auto &id : reference_line_info.TargetLaneId()) {
    lane_ids_.emplace_back(id);
  }

  const auto &reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  double neighbor_lane_width_for_expand;
  std::vector<century::hdmap::Id> expand_lane_ids;
  expand_lane_ids.clear();
  if (config_.open_space_roi_decider_config()
          .rescue_config()
          .enable_expand_neighbor_lane()) {
    openspace_common_->ExpandNeighborLaneIdForBoundary(
        injector_->vehicle_state(), frame_, hdmap_, adc_position_sl.s(),
        expand_lane_ids, neighbor_lane_ids_, &neighbor_lane_width_for_expand);
  }

  auto *previous_frame = injector_->frame_history()->Latest();
  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  static int count = 0;
  if (previous_frame->open_space_info().fallback_flag() &&
      std::fabs(adc_speed) < kStopSpeed) {
    ++count;
  } else {
    count -= 2;
    count = std::max(count, 0);
  }

  // to add replan logic
  if (count > kReplanCount) {
    count = 0;
    if (FLAGS_enable_rescue_replan_reason1) {
      goals_vector_.clear();
      is_already_second_plan_ = false;
      return;
    }
  }
  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AERROR << "last frame is lane follow,clear goals";
    goals_vector_.clear();
  }
  const auto &plan_start_point =
      previous_frame->open_space_info().rescue_second_end_point();
  const auto &second_start_pose =
      previous_frame->open_space_info().rescue_first_end_point();

  is_try_to_second_plan_ =
      FLAGS_enable_rescue_second_plan
          ? (previous_frame->open_space_info().open_space_provider_success() &&
             (!previous_frame->open_space_info()
                   .open_space_second_provider_success()))
          : false;
  // check first plan if finish,then need second plan
  if (previous_frame->open_space_info().open_space_second_provider_success() &&
      !is_try_to_second_plan_) {
    common::SLPoint end_position_sl;
    reference_line.XYToSL(second_start_pose, &end_position_sl);
    if (adc_position_sl.s() >
        end_position_sl.s() + config_.open_space_roi_decider_config()
                                  .rescue_config()
                                  .rescue_hybrid_lon_sample_interval()) {
      is_already_second_plan_ = false;
      // first enter second plan update the relative info
      frame->mutable_open_space_info()->set_open_space_second_provider_success(
          true);
    }
  }
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  static int second_count = 0;
  double dx = plan_start_point.x() - adc_init_x;
  double dy = plan_start_point.y() - adc_init_y;
  double dist = std::hypot(dx, dy);

  if (!is_first_plan_success) {
    GenerateSampleGoals(adc_position_sl,
                        config_.open_space_roi_decider_config()
                            .rescue_config()
                            .rescue_hybrid_start_distance(),
                        config_.open_space_roi_decider_config()
                            .rescue_config()
                            .rescue_hybrid_end_distance(),
                        config_.open_space_roi_decider_config()
                            .rescue_config()
                            .rescue_hybrid_lon_sample_interval(),
                        frame, reference_line);
    second_count = 0;
  } else if (is_try_to_second_plan_ &&
             (!is_already_second_plan_ || goals_vector_.empty())) {
    if (dist > kValidDistance) {
      AERROR << " dist= " << dist << " is invalid distance";
    } else if (dist < kInValidDistance && std::fabs(adc_speed) < kStopSpeed &&
               FLAGS_enable_rescue_replan_reason1) {
      is_try_to_second_plan_ = false;
      is_already_second_plan_ = false;
      goals_vector_.clear();
      return;
    }
    second_count = 0;
    common::SLPoint end_position_sl;
    reference_line.XYToSL(plan_start_point, &end_position_sl);
    GenerateSampleGoals(end_position_sl, kSecondStartS, kSecondEndS,
                        kSecondSampleS, frame, reference_line);
  } else {
    if (std::fabs(adc_speed) < kStopSpeed) {
      second_count++;
    } else {
      second_count = 0;
    }
    if (second_count > kStopTooLongCount &&
        FLAGS_enable_rescue_replan_reason1) {
      goals_vector_.clear();
      second_count = 0;
      AERROR << "stop too long ,refresh goals";
      return;
    }
  }
  if (goals_vector_.empty()) {
    AERROR << "find goal error, return";
    return;
  }
  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  const auto &target_point = goals_vector_.back().adc_point;
  Vec2d target(target_point.x(), target_point.y());
  // Rescue Scenario
  rescue_end_point_ = target;
  rescue_end_point_heading_ = goals_vector_.back().adc_heading;
  AINFO << "select rescue origin pose x: " << std::setprecision(9)
        << origin_point.x() << ", y: " << origin_point.y()
        << ", origin_heading_angle: " << origin_heading * common::util::RAD2ANG;
  AINFO << "select rescue end pose x: " << std::setprecision(9)
        << rescue_end_point_.x() << " y: " << rescue_end_point_.y()
        << ", end_heading_angle: "
        << rescue_end_point_heading_ * common::util::RAD2ANG;

  double dis_between_adc_and_end_pt =
      std::hypot(origin_point.x() - rescue_end_point_.x(),
                 origin_point.y() - rescue_end_point_.y());
  injector_->is_reach_search_end_point_ =
      dis_between_adc_and_end_pt < config_.open_space_roi_decider_config()
                                       .rescue_config()
                                       .reach_end_point_dis_threshold();
  frame->mutable_open_space_info()->mutable_end_point()->set_x(
      rescue_end_point_.x());
  frame->mutable_open_space_info()->mutable_end_point()->set_y(
      rescue_end_point_.y());
  target -= origin_point;
  target.SelfRotate(-origin_heading);
  common::SLPoint target_sl;
  common::SLPoint start_sl;
  reference_line.XYToSL(rescue_end_point_, &target_sl);
  reference_line.XYToSL(plan_start_point, &start_sl);
  double target_theta =
      reference_line.GetReferencePoint(target_sl.s()).heading();
  // set rescue end pose
  frame->mutable_open_space_info()->set_end_heading(
      common::math::NormalizeAngle(target_theta));
  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);
  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->emplace_back(target.x());
  end_pose->emplace_back(target.y());
  end_pose->emplace_back(target_theta);
  // end_pose info: x, y, theta, speed
  end_pose->emplace_back(0.0);
  if (end_pose->size() > 3) {
    Vec2d end_pose_to_world_frame((*end_pose)[0], (*end_pose)[1]);
    end_pose_to_world_frame.SelfRotate(
        frame_->open_space_info().origin_heading());
    end_pose_to_world_frame += frame_->open_space_info().origin_point();
    double end_theta_to_world_frame = (*end_pose)[2];
    end_theta_to_world_frame += frame_->open_space_info().origin_heading();
    AINFO << "end_x: " << end_pose_to_world_frame.x()
          << ", end_y: " << end_pose_to_world_frame.y() << ", end_theta_angle: "
          << end_theta_to_world_frame * common::util::RAD2ANG
          << ", end_speed: " << (*end_pose)[3];
  }
}

void OpenSpaceRoiDecider::SetStopLineParkingEndPose(Frame *const frame) {
  auto traffic_light_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_traffic_light();
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const Vec2d adc_position = {adc_init_x, adc_init_y};
  common::SLPoint adc_position_sl;

  const auto &reference_line_list = frame->reference_line_info();
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

  const double light_x =
      traffic_light_status->mutable_traffic_light_pose()->x();
  const double light_y =
      traffic_light_status->mutable_traffic_light_pose()->y();
  double target_theta = std::atan2(light_y - adc_init_y, light_x - adc_init_x);

  ADEBUG << "center.x(): " << std::setprecision(9) << light_x;
  ADEBUG << "center.y(): " << std::setprecision(9) << light_y;
  ADEBUG << "target_theta: " << std::setprecision(9) << target_theta;

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d center(traffic_light_status->mutable_adc_adjust_end_pose()->x(),
               traffic_light_status->mutable_adc_adjust_end_pose()->y());

  center -= origin_point;
  center.SelfRotate(-origin_heading);
  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();

  end_pose->emplace_back(center.x());
  end_pose->emplace_back(center.y());
  end_pose->emplace_back(target_theta);

  ADEBUG << "end_pose position (x): " << std::setprecision(9) << (*end_pose)[0];
  ADEBUG << "end_pose position (y): " << std::setprecision(9) << (*end_pose)[1];

  end_pose->emplace_back(0);
}

// get road boundary near parking space
void OpenSpaceRoiDecider::GetRoadBoundary(
    const hdmap::Path &nearby_path, const double center_line_s,
    const Vec2d &origin_point, const double origin_heading,
    std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<Vec2d> *center_lane_boundary_left,
    std::vector<Vec2d> *center_lane_boundary_right,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  double start_s =
      center_line_s -
      config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  double end_s =
      center_line_s +
      config_.open_space_roi_decider_config().roi_longitudinal_range_end();

  double vehicle_s = 0.0, vehicle_l = 0.0;
  nearby_path.GetProjection(Vec2d(injector_->vehicle_state()->x(),
                                  injector_->vehicle_state()->y()),
                            &vehicle_s, &vehicle_l);
  start_s = std::fmax(0.0,
      std::fmin(start_s, vehicle_s - vehicle_params_.length()));
  end_s = std::fmin(nearby_path.length(),
      std::fmax(end_s, vehicle_s + vehicle_params_.length()));
  ADEBUG << "ROI range " << start_s << '\t' << end_s;

  hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
  double last_check_point_heading = start_point.heading();
  double index = 0.0;                // sample index
  double check_point_s = start_s;    // sample s

  // For the road boundary, add key points to left/right side boundary
  // separately. Iterate s_value to check key points at a step of
  // roi_line_segment_length. Key points include: start_point, end_point,
  // points where path curvature is large, points near left/right road-curb
  // corners
  while (check_point_s <= end_s) {
    // path point
    hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);
    double check_point_heading = check_point.heading();
    // heading change flag
    bool is_center_lane_heading_change =
        std::abs(common::math::NormalizeAngle(check_point_heading -
                                              last_check_point_heading)) >
        config_.open_space_roi_decider_config().roi_line_segment_min_angle();
    // update last_check_point_heading
    last_check_point_heading = check_point_heading;

    // Check if the current center-lane checking-point is start point || end
    // point || or point with larger curvature. If yes, mark it as an anchor
    // point.
    // anchor point select
    bool is_anchor_point = check_point_s == start_s || check_point_s == end_s ||
                           is_center_lane_heading_change;
    /*
     * center_lane_boundary_left       sample point
     * left_lane_boundary              lane boundary point
     * center_lane_s_left              sample s
     * left_lane_road_width            left road width
     * center_lane_boundary_right      sample point
     * right_lane_boundary             lane boundary point
     * center_lane_s_right             sample s
     * right_lane_road_width           right road width
     */

    // Add key points to the left-half boundary
    AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
                        is_anchor_point, true, center_lane_boundary_left,
                        left_lane_boundary, center_lane_s_left,
                        left_lane_road_width);
    // Add key points to the right-half boundary
    AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
                        is_anchor_point, false, center_lane_boundary_right,
                        right_lane_boundary, center_lane_s_right,
                        right_lane_road_width);

    if (check_point_s == end_s) {
      break;
    }
    // roi_line_segment_length  step size
    index += 1.0;
    check_point_s =
        start_s +
        index *
            config_.open_space_roi_decider_config().roi_line_segment_length();
    check_point_s = check_point_s >= end_s ? end_s : check_point_s;
  }

  // world ---> origin
  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  for (size_t i = 0; i < left_point_size; ++i) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
  for (size_t i = 0; i < right_point_size; ++i) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
}

// ROI boundary from electric fence
bool OpenSpaceRoiDecider::GetRoiBoundaryFromElectricFence(
    const century::common::VehicleState &vehicle_state,
    std::vector<std::vector<Vec2d>> &electric_fences_boundary) {
  if (!config_.open_space_roi_decider_config().enable_electric_fence_roi()) {
    return true;
  }
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<Vec2d> ele_fence_points;
  std::vector<hdmap::ElectricFenceInfoConstPtr> electric_fences;
  ele_fence_points.clear();
  electric_fences.clear();
  bool is_in_roi_elec_fence_search_range = false;
  common::PointENU veh_point_enu;
  AINFO << __func__ << ", vehicle_state_x: " << vehicle_state.x()
        << ", y: " << vehicle_state.y();
  veh_point_enu.set_x(vehicle_state.x());
  veh_point_enu.set_y(vehicle_state.y());
  const double roi_electric_fence_search_radius =
      config_.open_space_roi_decider_config()
          .roi_electric_fence_search_radius();

  if (0 == base_map_ptr->GetElectricFences(veh_point_enu,
                                           roi_electric_fence_search_radius,
                                           &electric_fences)) {
    // electric fence loop
    for (const auto &fence : electric_fences) {
      const auto &ele_polygon_points = fence->polygon().points();
      if (fence->electric_fence().type() == hdmap::ElectricFence::DRIVABLE) {
        // ignore drivable electric fence area
        continue;
      }
      // get all not-drivable and distance < roi_electric_fence_search_radius electric fence area
      for (const auto &pt : ele_polygon_points) {
        if (pt.DistanceTo({vehicle_state.x(), vehicle_state.y()}) <=
            roi_electric_fence_search_radius) {
          is_in_roi_elec_fence_search_range = true;
        }
      }
      for (const auto &pt : ele_polygon_points) {
        if (is_in_roi_elec_fence_search_range) {
          ele_fence_points.emplace_back(pt);
        }
      }
      if (ele_fence_points.size() > 1) {
        // if electric fence size less 1, do not add to roi boundary
        electric_fences_boundary.emplace_back(ele_fence_points);
      }
    }
  } else {
    AERROR << "WARNING, No electric fence in map with search radius: "
           << roi_electric_fence_search_radius;
    return false;
  }
  return true;
}

// closed to fix large function.
bool OpenSpaceRoiDecider::GetRoadBoundaryFromMap(
    const hdmap::Path &nearby_path, const double center_line_s,
    const Vec2d &origin_point, const double origin_heading,
    std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<Vec2d> *center_lane_boundary_left,
    std::vector<Vec2d> *center_lane_boundary_right,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  // Longitudinal range can be asymmetric.
  double start_s =
      center_line_s -
      config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  double end_s =
      center_line_s +
      config_.open_space_roi_decider_config().roi_longitudinal_range_end();

  if (is_try_to_second_plan_) {
    end_s +=
        config_.open_space_roi_decider_config().roi_longitudinal_range_end();
  }

  double check_point_s = start_s;
  while (check_point_s <= end_s) {
    hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);

    double left_road_width = nearby_path.GetRoadLeftWidth(check_point_s);
    double right_road_width = nearby_path.GetRoadRightWidth(check_point_s);

    // get road boundaries at current location
    common::PointENU check_point_xy;
    std::vector<hdmap::RoadRoiPtr> road_boundaries;
    std::vector<hdmap::JunctionInfoConstPtr> junctions;
    check_point_xy.set_x(check_point.x());
    check_point_xy.set_y(check_point.y());
    int result = 0;
    if (lane_ids_.empty() || !FLAGS_enable_use_ref_lane_roadboundary) {
      result = hdmap_->GetRoadBoundaries(check_point_xy, kInParkingLot,
                                         &road_boundaries, &junctions);
    } else {
      result =
          hdmap_->GetRoadBoundaries(check_point_xy, kInParkingLot, lane_ids_,
                                    &road_boundaries, &junctions);
    }

    // TODO(all): need to ensure get road boundary ok.
    if (0 != result) {
      AERROR << "GetRoadBoundaryFromMap:hdmap_->GetRoadBoundaries "
                "failed, with result is: "
             << result;
      return false;
    }

    if (road_boundaries.size() < 1) {
      AERROR << "GetRoadBoundaryFromMap:road_boundaries size is empty!";
      return false;
    }

    // TODO(weihuashen): just get the lane boundary directly.
    for (size_t i = 0;
         i < (*road_boundaries.at(0)).left_boundary.line_points.size(); ++i) {
      Vec2d point =
          Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                (*road_boundaries.at(0)).left_boundary.line_points[i].y());
      double dist = point.DistanceTo(Vec2d(check_point.x(), check_point.y()));
      // TODO(weihuashen): should compare with s.
      if (dist > kIngoreRoadBoundaryDist && left_lane_boundary->size() > 1) {
        continue;
      }
      left_lane_boundary->emplace_back(std::move(point));
    }
    for (size_t i = 0;
         i < (*road_boundaries.at(0)).right_boundary.line_points.size(); ++i) {
      Vec2d point =
          Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                (*road_boundaries.at(0)).right_boundary.line_points[i].y());
      double dist = point.DistanceTo(Vec2d(check_point.x(), check_point.y()));
      // TODO(weihuashen): should compare with s.
      if (dist > kIngoreRoadBoundaryDist && right_lane_boundary->size() > 1) {
        continue;
      }
      right_lane_boundary->emplace_back(std::move(point));
    }

    center_lane_boundary_right->emplace_back(check_point);
    center_lane_boundary_left->emplace_back(check_point);
    center_lane_s_left->emplace_back(check_point_s);
    center_lane_s_right->emplace_back(check_point_s);
    left_lane_road_width->emplace_back(left_road_width);
    right_lane_road_width->emplace_back(right_road_width);

    double roi_line_segment_length_from_map =
        config_.open_space_roi_decider_config()
            .roi_line_segment_length_from_map();
    check_point_s = check_point_s + roi_line_segment_length_from_map;
  }

  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  if (left_point_size < 2 || right_point_size < 2) {
    AERROR << "GetRoadBoundaryFromMap:left_point_size or right_point_size is < 2!";
    return false;
  }

  // Convert coordinates
  for (size_t i = 0; i < left_point_size; ++i) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
  for (size_t i = 0; i < right_point_size; ++i) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).SelfRotate(-origin_heading);
  }

  // offset lane boundary according to turn type
  openspace_common_->OffsetBoundaryWithTurnType(
      frame_, injector_->vehicle_state()->vehicle_state(), *left_lane_boundary,
      *right_lane_boundary,
      config_.open_space_roi_decider_config()
          .rescue_config()
          .expand_boundary_offset_dis());

  // Sort coordinate points
  if (!left_lane_boundary->empty()) {
    RemoveSamePointsInBoundary(left_lane_boundary);
  }
  if (!right_lane_boundary->empty()) {
    RemoveSamePointsInBoundary(right_lane_boundary);
  }
  double s_intervals =
      is_try_to_second_plan_ ? kBoundaryLargeInterval : kBoundaryInterval;
  InterpolateBoundary(
      s_intervals,
      config_.open_space_roi_decider_config().roi_line_segment_min_angle(),
      left_lane_boundary);
  InterpolateBoundary(
      s_intervals,
      config_.open_space_roi_decider_config().roi_line_segment_min_angle(),
      right_lane_boundary);
  return true;
}

void OpenSpaceRoiDecider::RemoveSamePointsInBoundary(
    std::vector<century::common::math::Vec2d> *boundary) {
  if (boundary->empty()) {
    return;
  }
  std::unordered_set<Vec2d, Vec2dHash, Vec2dEqual> seen_points;
  std::vector<Vec2d> result;
  for (const auto &point : *boundary) {
    if (seen_points.find(point) == seen_points.end()) {
      seen_points.insert(point);
      result.emplace_back(point);
    }
  }
  *boundary = result;
}

// add smooth point for better road boudary in curve
void OpenSpaceRoiDecider::AddBoundaryKeyPoint(
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
  // start_s ~ check_point_s
  const double previous_distance_s = std::min(
      config_.open_space_roi_decider_config().roi_line_segment_length(),
      check_point_s - start_s);
  // check_point_s ~ end_s
  const double next_distance_s = std::min(
      config_.open_space_roi_decider_config().roi_line_segment_length(),
      end_s - check_point_s);

  hdmap::MapPathPoint current_check_point =
      nearby_path.GetSmoothPoint(check_point_s);
  // start point
  hdmap::MapPathPoint previous_check_point =
      nearby_path.GetSmoothPoint(check_point_s - previous_distance_s);
  // end point
  hdmap::MapPathPoint next_check_point =
      nearby_path.GetSmoothPoint(check_point_s + next_distance_s);

  // check point heading
  double current_check_point_heading = current_check_point.heading();
  // check point's road width
  double current_road_width =
      is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s)
                   : nearby_path.GetRoadRightWidth(check_point_s);
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
    center_lane_boundary->emplace_back(current_check_point);
    curb_lane_boundary->emplace_back(curb_lane_point);
    center_lane_s->emplace_back(check_point_s);
    road_width->emplace_back(current_road_width);
    return;
  }
  // start point's road width
  double previous_road_width =
      is_left_curb
          ? nearby_path.GetRoadLeftWidth(check_point_s - previous_distance_s)
          : nearby_path.GetRoadRightWidth(check_point_s - previous_distance_s);
  // end point's road width
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
      config_.open_space_roi_decider_config()
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
    center_lane_boundary->emplace_back(current_check_point);
    curb_lane_boundary->emplace_back(curb_lane_point);
    center_lane_s->emplace_back(check_point_s);
    road_width->emplace_back(current_road_width);
  }
}

// only one lane one lane has one segment
void OpenSpaceRoiDecider::GetInLaneEndPoint(LaneInfoConstPtr lane_info,
                                            PointENU *left_boundary_point,
                                            PointENU *right_boundary_point) {
  const auto &left_boundary_segment =
      lane_info->lane().left_boundary().curve().segment();
  const auto &right_boundary_segment =
      lane_info->lane().right_boundary().curve().segment();
  size_t lane_points_num = left_boundary_segment[0].line_segment().point_size();
  *left_boundary_point =
      left_boundary_segment[0].line_segment().point(lane_points_num - 1);
  *right_boundary_point =
      right_boundary_segment[0].line_segment().point(lane_points_num - 1);
}

void OpenSpaceRoiDecider::GetOutLaneStartPoint(LaneInfoConstPtr lane_info,
                                               PointENU *left_boundary_point,
                                               PointENU *right_boundary_point) {
  const auto &left_boundary_segment =
      lane_info->lane().left_boundary().curve().segment();
  const auto &right_boundary_segment =
      lane_info->lane().right_boundary().curve().segment();
  *left_boundary_point = left_boundary_segment[0].line_segment().point(0);
  *right_boundary_point = right_boundary_segment[0].line_segment().point(0);
}

void OpenSpaceRoiDecider::GetInLaneBoundaryPoints(
    LaneInfoConstPtr lane_info, const hdmap::Path &nearby_path,
    std::vector<PointENU> *const in_left_boundary_points,
    std::vector<PointENU> *const in_right_boundary_points) {
  return;
}

void OpenSpaceRoiDecider::GetOutLaneBoundaryPoints(
    LaneInfoConstPtr lane_info, const hdmap::Path &nearby_path,
    std::vector<PointENU> *const out_left_boundary_points,
    std::vector<PointENU> *const out_right_boundary_points) {
  const auto &left_boundary_segment =
      lane_info->lane().left_boundary().curve().segment();
  const auto &right_boundary_segment =
      lane_info->lane().right_boundary().curve().segment();
  size_t lane_points_num = left_boundary_segment[0].line_segment().point_size();
  int temp_record_left = 0;
  int temp_record_right = 0;
  double left_point_s = 0.0;
  double left_point_l = 0.0;
  double right_point_s = 0.0;
  double right_point_l = 0.0;
  std::vector<double> left_s, right_s, left_l, right_l;
  std::vector<PointENU> left_points, right_points;
  double target_point_s, target_point_l;
  Vec2d target_position = {routing_target_point_.x(),
                           routing_target_point_.y()};
  lane_info->GetProjection(target_position, &target_point_s, &target_point_l);
  for (size_t i = 0; i < lane_points_num; ++i) {
    const auto &left_point =
        left_boundary_segment[0].line_segment().point().at(i);
    const auto &right_point =
        right_boundary_segment[0].line_segment().point().at(i);
    left_points.emplace_back(left_point);
    right_points.emplace_back(right_point);
    Vec2d left_point_v = {
        left_boundary_segment[0].line_segment().point().at(i).x(),
        left_boundary_segment[0].line_segment().point().at(i).y()};
    Vec2d right_point_v = {
        right_boundary_segment[0].line_segment().point().at(i).x(),
        right_boundary_segment[0].line_segment().point().at(i).y()};
    lane_info->GetProjection(left_point_v, &left_point_s, &left_point_l);
    lane_info->GetProjection(right_point_v, &right_point_s, &right_point_l);
    if (left_point_s < target_point_s) {
      out_left_boundary_points->emplace_back(left_point);
      temp_record_left = i;
    }
    if (right_point_s < target_point_s) {
      out_right_boundary_points->emplace_back(right_point);
      temp_record_right = i;
    }
    left_s.emplace_back(left_point_s);
    left_l.emplace_back(left_point_l);
    right_s.emplace_back(right_point_s);
    right_l.emplace_back(right_point_l);
  }

  // Added a buffer to the boundary to ensure that the boundary exists.
  double temp_s = target_point_s + vehicle_params_.length();
  for (size_t i = temp_record_left; i < lane_points_num; ++i) {
    if (left_s[i] > temp_s) {
      out_left_boundary_points->emplace_back(left_points[i]);
      break;
    }
  }
  for (size_t i = temp_record_right; i < lane_points_num; ++i) {
    if (right_s[i] > temp_s) {
      out_right_boundary_points->emplace_back(right_points[i]);
      break;
    }
  }

  std::reverse(out_left_boundary_points->begin(),
               out_left_boundary_points->end());
}

// generate roi_parking_boundary
bool OpenSpaceRoiDecider::GetParkingBoundary(
    Frame *const frame, const std::array<Vec2d, 4> &vertices,
    const hdmap::Path &nearby_path,
    std::vector<std::vector<Vec2d>> *const roi_parking_boundary) {
  if (nullptr == frame || nullptr == roi_parking_boundary ||
      nullptr == target_parking_spot_) {
    AERROR << "Invalid input, cannot generate parking boundary";
    return false;
  }
  roi_parking_boundary->clear();
  const auto& origin_point = frame->open_space_info().origin_point();
  const auto& origin_heading = frame->open_space_info().origin_heading();

  std::vector<Vec2d> boundary_points;
  if (!BuildParkingBoundaryPoints(frame, vertices, nearby_path, origin_point,
                                  origin_heading, &boundary_points)) {
    return false;
  }

  // generate roi_parking_boundary
  for (size_t i = 0; i < boundary_points.size(); ++i) {
    size_t next_i = (i + 1) % boundary_points.size();
    std::vector<Vec2d> segment{boundary_points[i], boundary_points[next_i]};
    roi_parking_boundary->emplace_back(segment);
  }

  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_parking_boundary)) {
    AERROR << "FuseLineSegments failed in parking ROI";
    return false;
  }

  ADEBUG << "roi boundary in origin axis";
  for (auto line_segment : *roi_parking_boundary) {
    for (auto point : line_segment) {
      ADEBUG << point.DebugString();
    }
  }

  if (!SetRoiBoundaryAndCheckVehicle(frame, boundary_points, origin_point,
                                     origin_heading)) {
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::SetRoiBoundaryAndCheckVehicle(
    Frame* frame, const std::vector<Vec2d>& boundary_points,
    const Vec2d& origin_point, double origin_heading) {
  auto xminmax=std::minmax_element(boundary_points.begin(),boundary_points.end(),
      [](const Vec2d&a,const Vec2d&b){return a.x()<b.x();});

  auto yminmax=std::minmax_element(boundary_points.begin(),boundary_points.end(),
      [](const Vec2d&a,const Vec2d&b){return a.y()<b.y();});

  std::vector<double> ROI_xy_boundary{
      xminmax.first->x(),xminmax.second->x(),
      yminmax.first->y(),yminmax.second->y()};

  auto* xy_boundary=
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();

  xy_boundary->assign(ROI_xy_boundary.begin(),ROI_xy_boundary.end());

  Vec2d vehicle_xy(vehicle_state_.x(),vehicle_state_.y());
  vehicle_xy-=origin_point;
  vehicle_xy.SelfRotate(-origin_heading);

  if(vehicle_xy.x()<ROI_xy_boundary[0]||
     vehicle_xy.x()>ROI_xy_boundary[1]||
     vehicle_xy.y()<ROI_xy_boundary[2]||
     vehicle_xy.y()>ROI_xy_boundary[3]){

    AERROR<<"vehicle outside of xy boundary of parking ROI";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::BuildParkingBoundaryPoints(
    Frame* const frame, const std::array<Vec2d, 4>& vertices,
    const hdmap::Path& nearby_path, const Vec2d& origin_point,
    double origin_heading, std::vector<Vec2d>* boundary_points) {
  auto parking_lot =
      hdmap_->GetParkingLotById(target_parking_spot_->parking_lot_id());

  if (parking_lot != nullptr) {
    for (auto point : parking_lot->polygon().points()) {
      point -= origin_point;
      point.SelfRotate(-origin_heading);
      boundary_points->emplace_back(point);
    }
    return true;
  }
  auto left_down = vertices[0];
  auto left_top = vertices[1];
  auto right_top = vertices[2];
  auto right_down = vertices[3];

  double left_down_s = 0, left_down_l = 0;
  double left_top_s = 0, left_top_l = 0;
  double right_top_s = 0, right_top_l = 0;
  double right_down_s = 0, right_down_l = 0;

  if (!(nearby_path.GetProjection(left_down, &left_down_s, &left_down_l) &&
        nearby_path.GetProjection(left_top, &left_top_s, &left_top_l) &&
        nearby_path.GetProjection(right_top, &right_top_s, &right_top_l) &&
        nearby_path.GetProjection(right_down, &right_down_s, &right_down_l))) {
    AERROR << "fail to get parking spot projections";
    return false;
  }

  left_down -= origin_point;
  left_down.SelfRotate(-origin_heading);
  left_top -= origin_point;
  left_top.SelfRotate(-origin_heading);
  right_top -= origin_point;
  right_top.SelfRotate(-origin_heading);
  right_down -= origin_point;
  right_down.SelfRotate(-origin_heading);

  const double center_line_s =
      (left_down_s + left_top_s + right_top_s + right_down_s) * 0.25;

  std::vector<Vec2d> left_lane_boundary, right_lane_boundary;
  std::vector<Vec2d> center_lane_boundary_left, center_lane_boundary_right;
  std::vector<double> center_lane_s_left, center_lane_s_right;
  std::vector<double> left_lane_road_width, right_lane_road_width;

  GetRoadBoundary(nearby_path, center_line_s, origin_point, origin_heading,
                  &left_lane_boundary, &right_lane_boundary,
                  &center_lane_boundary_left, &center_lane_boundary_right,
                  &center_lane_s_left, &center_lane_s_right,
                  &left_lane_road_width, &right_lane_road_width);

  std::vector<std::tuple<double, double, Vec2d>> four_corner_points;
  std::vector<std::pair<Vec2d, Vec2d>> boundaries;

  size_t index = 0;

  if (ParkingSpacePosition::MIDDLE == parking_space_position_) {
    four_corner_points = {{left_down_s, left_down_l, left_down},
                          {left_top_s, left_top_l, left_top},
                          {right_top_s, right_top_l, right_top},
                          {right_down_s, right_down_l, right_down}};
    std::sort(four_corner_points.begin(), four_corner_points.end(),
              [](auto a, auto b) { return std::get<0>(a) < std::get<0>(b); });
  } else if (ParkingSpacePosition::LEFT == parking_space_position_) {
    four_corner_points = {{right_down_s, right_down_l, right_down},
                          {left_down_s, left_down_l, left_down},
                          {left_top_s, left_top_l, left_top},
                          {right_top_s, right_top_l, right_top}};
  } else {
    four_corner_points = {{left_down_s, left_down_l, left_down},
                          {right_down_s, right_down_l, right_down},
                          {right_top_s, right_top_l, right_top},
                          {left_top_s, left_top_l, left_top}};
  }

  for (size_t i = 0; i < right_lane_boundary.size(); ++i) {
    double s = center_lane_s_right[i];

    while (index < four_corner_points.size() &&
           std::get<0>(four_corner_points[index]) < s) {
      hdmap::MapPathPoint smooth_point =
          nearby_path.GetSmoothPoint(std::get<0>(four_corner_points[index]));

      double heading = smooth_point.heading();
      double left_width =
          nearby_path.GetRoadLeftWidth(std::get<0>(four_corner_points[index]));
      double right_width =
          nearby_path.GetRoadRightWidth(std::get<0>(four_corner_points[index]));

      Vec2d center_point(smooth_point.x(), smooth_point.y());

      if (ParkingSpacePosition::RIGHT == parking_space_position_)
        left_width += config_.open_space_roi_decider_config()
                          .road_borrow_width_for_parking();

      Vec2d left_pt =
          center_point + left_width * Vec2d(std::cos(heading + M_PI_2),
                                            std::sin(heading + M_PI_2));
      left_pt -= origin_point;
      left_pt.SelfRotate(-origin_heading);

      if (ParkingSpacePosition::LEFT == parking_space_position_)
        right_width += config_.open_space_roi_decider_config()
                           .road_borrow_width_for_parking();

      Vec2d right_pt =
          center_point + right_width * Vec2d(std::cos(heading - M_PI_2),
                                             std::sin(heading - M_PI_2));
      right_pt -= origin_point;
      right_pt.SelfRotate(-origin_heading);

      if (std::get<1>(four_corner_points[index]) > 0) {
        if (std::get<1>(four_corner_points[index]) > left_width)
          boundaries.emplace_back(std::get<2>(four_corner_points[index]),
                                  right_pt);
        else
          boundaries.emplace_back(left_pt, right_pt);
      } else {
        if (std::get<1>(four_corner_points[index]) < -right_width)
          boundaries.emplace_back(left_pt,
                                  std::get<2>(four_corner_points[index]));
        else
          boundaries.emplace_back(left_pt, right_pt);
      }
      ++index;
    }

    hdmap::MapPathPoint smooth_point =
        nearby_path.GetSmoothPoint(center_lane_s_right[i]);

    double heading = smooth_point.heading();
    double left_width = left_lane_road_width[i];
    if (ParkingSpacePosition::RIGHT == parking_space_position_)
      left_width += config_.open_space_roi_decider_config()
                        .road_borrow_width_for_parking();

    double right_width = right_lane_road_width[i];
    if (ParkingSpacePosition::LEFT == parking_space_position_)
      right_width += config_.open_space_roi_decider_config()
                         .road_borrow_width_for_parking();

    Vec2d center_point(smooth_point.x(), smooth_point.y());

    Vec2d left_pt =
        center_point + left_width * Vec2d(std::cos(heading + M_PI_2),
                                          std::sin(heading + M_PI_2));
    left_pt -= origin_point;
    left_pt.SelfRotate(-origin_heading);

    Vec2d right_pt =
        center_point + right_width * Vec2d(std::cos(heading - M_PI_2),
                                           std::sin(heading - M_PI_2));
    right_pt -= origin_point;
    right_pt.SelfRotate(-origin_heading);

    boundaries.emplace_back(left_pt, right_pt);
  }

  if (index < four_corner_points.size()) {
    AERROR << "should not happen";
    return false;
  }

  boundary_points->resize(boundaries.size() * 2);
  size_t num = boundary_points->size() - 1;
  for (size_t i = 0; i < boundaries.size(); ++i) {
    (*boundary_points)[i] = boundaries[i].first;
    (*boundary_points)[num - i] = boundaries[i].second;
  }

  return true;
}

bool OpenSpaceRoiDecider::GetPullOverBoundary(
    Frame *const frame, const std::array<Vec2d, 4> &vertices,
    const hdmap::Path &nearby_path,
    std::vector<std::vector<Vec2d>> *const roi_parking_boundary) {
  return true;
}

bool OpenSpaceRoiDecider::GetRescueBoundary(
    Frame *const frame, const hdmap::Path &nearby_path,
    std::vector<std::vector<Vec2d>> *const roi_parking_boundary) {
  const double adc_init_x = injector_->vehicle_state()->x();
  const double adc_init_y = injector_->vehicle_state()->y();
  const double adc_init_heading = injector_->vehicle_state()->heading();

  common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  double shift_distance =
      adc_length * 0.5 - vehicle_params_.back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(adc_init_heading),
                  shift_distance * std::sin(adc_init_heading)};
  adc_box.Shift(shift_vec);

  std::vector<common::math::Vec2d> adc_corners;
  adc_box.GetAllCorners(&adc_corners);
  auto left_top = adc_corners[1];
  auto right_top = adc_corners[0];

  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l) &&
        nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
    AERROR << "fail to get adc corners points' projections on reference line";
    return false;
  }

  std::vector<Vec2d> left_lane_boundary;
  std::vector<Vec2d> right_lane_boundary;
  std::vector<Vec2d> left_neighbor_lane_boundary;
  std::vector<Vec2d> right_neighbor_lane_boundary;
  const double center_line_s = (left_top_s + right_top_s) * 0.5;
  std::vector<Vec2d> center_lane_boundary_left;
  std::vector<Vec2d> center_lane_boundary_right;
  std::vector<double> center_lane_s_left;
  std::vector<double> center_lane_s_right;
  std::vector<double> left_lane_road_width;
  std::vector<double> right_lane_road_width;
  AINFO << "FLAGS_enable_use_lane_as_boundary: "
        << FLAGS_enable_use_lane_as_boundary
        << ", FLAGS_use_road_boundary_from_map: "
        << FLAGS_use_road_boundary_from_map;
  if (!FLAGS_enable_use_lane_as_boundary) {
    if (FLAGS_use_road_boundary_from_map) {
      if (!GetRoadBoundaryFromMap(
              nearby_path, center_line_s, origin_point, origin_heading,
              &left_lane_boundary, &right_lane_boundary,
              &center_lane_boundary_left, &center_lane_boundary_right,
              &center_lane_s_left, &center_lane_s_right, &left_lane_road_width,
              &right_lane_road_width)) {
        AERROR << "OpenSpaceRoiDecider::GetRoadBoundaryFromMap Failed.";
        return false;
      }
    } else {
      GetRoadBoundary(nearby_path, center_line_s, origin_point, origin_heading,
                      &left_lane_boundary, &right_lane_boundary,
                      &center_lane_boundary_left, &center_lane_boundary_right,
                      &center_lane_s_left, &center_lane_s_right,
                      &left_lane_road_width, &right_lane_road_width);
    }
  } else {
    // @brief Get rescue lane boundaries of both sides
    GetRescueLaneBoundary(nearby_path, origin_point, origin_heading,
                          &left_lane_boundary, &right_lane_boundary);
  }

  size_t left_point_size = left_lane_boundary.size();
  size_t right_point_size = right_lane_boundary.size();
  if (left_point_size < kMinLaneBoundaryPointSize ||
      right_point_size < kMinLaneBoundaryPointSize) {
    AERROR
        << "left or right lane boundary points size is too small, return false";
    return false;
  }

  std::vector<Vec2d> boundary_points;
  std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
            std::back_inserter(boundary_points));
  std::copy(left_lane_boundary.begin(), left_lane_boundary.end(),
            std::back_inserter(boundary_points));

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; --i) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->emplace_back(segment);
  }

  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_parking_boundary)) {
    AERROR << "FuseLineSegments rescue boundary failed: ["
           << roi_parking_boundary->size() << "]";
    return false;
  }

  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  std::vector<Vec2d> convex_hull = util::ConvexHull(boundary_points);
  if (!convex_hull.empty() &&
      config_.open_space_roi_decider_config().enable_use_convex_hull()) {
    ROI_xy_boundary.clear();
    float expansion_margin = 3.0;
    auto xminmax_hull = std::minmax_element(
        convex_hull.begin(), convex_hull.end(),
        [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
    auto yminmax_hull = std::minmax_element(
        convex_hull.begin(), convex_hull.end(),
        [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });

    ROI_xy_boundary = {
        xminmax_hull.first->x() - expansion_margin,
        xminmax_hull.second->x() + expansion_margin,
        yminmax_hull.first->y() - expansion_margin,
        yminmax_hull.second->y() + expansion_margin};
  }

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
    AERROR << "vehicle outside of xy boundary of rescue ROI "
           << ", vehicle_xy.x(): " << vehicle_xy.x()
           << ", vehicle_xy.y(): " << vehicle_xy.y()
           << ", x_min: " << ROI_xy_boundary[0]
           << ", x_max: " << ROI_xy_boundary[1]
           << ", y_min: " << ROI_xy_boundary[2]
           << ", y_max: " << ROI_xy_boundary[3];
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::GetDeadEndSpot(
    Frame *const frame, JunctionInfoConstPtr *junction,
    std::vector<Vec2d> *dead_end_vertices) {
  if (nullptr == frame) {
    AERROR << "Invalid frame, fail to GetDeadEndSpotFromMap from frame. ";
    return false;
  }
  auto &junction_point = (*junction)->polygon().points();
  for (size_t i = 0; i < junction_point.size(); ++i) {
    (*dead_end_vertices).emplace_back(junction_point.at(i));
  }
  return true;
}

// get parking space corner points and path
bool OpenSpaceRoiDecider::GetParkingSpot(Frame *const frame,
                                         std::array<Vec2d, 4> *vertices,
                                         Path *nearby_path) {
  if (nullptr == frame || nullptr == vertices) {
    AERROR << "Invalid Input, fail to GetParkingSpotFromMap from frame. ";
    return false;
  }

  auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
  Vec2d point_2d = common::util::PointFactory::ToVec2d(vehicle_state_);

  const auto &ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == frame) {
    AERROR << "Last frame failed, fail to GetParkingSpotfrom frame "
              "history.";
    return false;
  }

  LaneInfoConstPtr nearest_lane = nullptr;
  double vehicle_lane_s = 0.0, vehicle_lane_l = 0.0;
  const auto &previous_open_space_info = ptr_last_frame->open_space_info();
  if (previous_open_space_info.target_parking_lane() != nullptr &&
      previous_open_space_info.target_parking_spot_id() ==
          frame->open_space_info().target_parking_spot_id()) {
    nearest_lane = previous_open_space_info.target_parking_lane();
    nearest_lane->GetProjection(point_2d, &vehicle_lane_s, &vehicle_lane_l);
  } else {
    int status = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
                     point, 10.0, vehicle_state_.heading(), M_PI * 0.5,
                     &nearest_lane, &vehicle_lane_s, &vehicle_lane_l);
    if (0 != status) {
      AERROR << "Getlane failed at "
                 "OpenSpaceRoiDecider::GetParkingSpotFromMap()";
      return false;
    }
  }
  frame->mutable_open_space_info()->set_target_parking_lane(nearest_lane);
  LaneSegment nearest_lanesegment =
      LaneSegment(nearest_lane, nearest_lane->accumulate_s().front(),
                  nearest_lane->accumulate_s().back());

  // TODO: generate path (yjl)
  std::vector<LaneSegment> segments_vector;
  segments_vector.emplace_back(nearest_lanesegment);
  // reverse extend length
  double reverse_length =
      config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  if (reverse_length < vehicle_params_.length()) {
    reverse_length += vehicle_params_.length();
  }
  double reverse_extend_s = vehicle_lane_s - reverse_length;
  while (reverse_extend_s < 0.0) {
    // no predecessor lane
    if (0 == segments_vector.back().lane->lane().predecessor_id_size())
      break;
    auto prev_lane = hdmap_->GetLaneById(
        segments_vector.back().lane->lane().predecessor_id(0));
    // add predecessor lane segment
    segments_vector.emplace_back(prev_lane, prev_lane->accumulate_s().front(),
        prev_lane->accumulate_s().back());
    // update reverse_extend_s
    reverse_extend_s += prev_lane->total_length();
  }
  std::reverse(segments_vector.begin(), segments_vector.end());

  // forward extend
  std::deque<std::vector<LaneSegment>> extend_segments;
  extend_segments.emplace_back(segments_vector);
  while (!extend_segments.empty()) {
    auto element = extend_segments.front();
    extend_segments.pop_front();
    // check whether the target parking space is on the element.back().lane
    SearchTargetParkingSpotOnLane(element.back().lane, &target_parking_spot_);
    if (nullptr != target_parking_spot_) {        // find target parking space
      // extend next lane
      double extend_length =
          config_.open_space_roi_decider_config().roi_longitudinal_range_end();
      if (extend_length < vehicle_params_.length()) {
        extend_length += vehicle_params_.length();
      }
      while (0 != element.back().lane->lane().successor_id_size()) {
        auto next_lane = hdmap_->GetLaneById(
            element.back().lane->lane().successor_id(0));
        element.emplace_back(next_lane, next_lane->accumulate_s().front(),
            next_lane->accumulate_s().back());
        extend_length -= next_lane->total_length();
        if (extend_length <= 0.0) {
          break;
        }
      }
      // generate path
      *nearby_path = Path(element);
      break;
    }
    for (auto next_lane_id : element.back().lane->lane().successor_id()) {
      auto next_lane = hdmap_->GetLaneById(next_lane_id);
      std::vector<LaneSegment> temp_segments(element);
      temp_segments.emplace_back(next_lane, next_lane->accumulate_s().front(),
          next_lane->accumulate_s().back());
      extend_segments.emplace_back(temp_segments);
    }
  }

  // failed to find target_parking_spot
  if (nullptr == target_parking_spot_ || nullptr == nearby_path) {
    AERROR << "No such parking spot found after searching all path forward "
               "possible";
    return false;
  }

  // check distance to parking space is less than threshold
  if (!CheckDistanceToParkingSpot(frame, *nearby_path, target_parking_spot_,
                                  vertices)) {
    AERROR << "target parking spot found, but too far, distance larger than "
              "pre-defined distance";
    return false;
  }

  const auto &routing_request = frame->local_view().routing->routing_request();
  auto plot_type = routing_request.parking_info().parking_space_type();
  if (ParkingSpaceType::PARALLEL_PARKING == plot_type) {         // parallel to the lane
    // extend
    double extend_right_x_buffer =
        config_.open_space_roi_decider_config().extend_right_x_buffer();
    double extend_left_x_buffer =
        config_.open_space_roi_decider_config().extend_left_x_buffer();
    extend_right_x_buffer = 0.0;
    extend_left_x_buffer = 0.0;
    // left down
    vertices->at(0).set_x(vertices->at(0).x() + extend_left_x_buffer);
    // left top
    vertices->at(1).set_x(vertices->at(1).x() + extend_left_x_buffer);
    // right top
    vertices->at(2).set_x(vertices->at(2).x() - extend_right_x_buffer);
    // right down
    vertices->at(3).set_x(vertices->at(3).x() - extend_right_x_buffer);
  }
  return true;
}

bool OpenSpaceRoiDecider::GetParkingSpotTest(Frame *const frame,
                                             std::array<Vec2d, 4> *vertices,
                                             Path *nearby_path) {
  if (nullptr == frame) {
    AERROR << "Invalid frame, fail to GetParkingSpotFromMap from frame. ";
    return false;
  }

  auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
  LaneInfoConstPtr nearest_lane;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;

  int status = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
      point, 15.0, vehicle_state_.heading(), M_PI * 0.66, &nearest_lane,
      &vehicle_lane_s, &vehicle_lane_l);
  if (0 != status) {
    AERROR << "Getlane failed at OpenSpaceRoiDecider::GetParkingSpotFromMap()";
    return false;
  }

  // Find parking spot by getting nearestlane
  ParkingSpaceInfoConstPtr target_parking_spot = nullptr;
  LaneSegment nearest_lanesegment =
      LaneSegment(nearest_lane, nearest_lane->accumulate_s().front(),
                  nearest_lane->accumulate_s().back());
  std::vector<LaneSegment> segments_vector;
  int next_lanes_num = nearest_lane->lane().successor_id_size();
  if (next_lanes_num != 0) {
    for (int i = 0; i < next_lanes_num; ++i) {
      auto next_lane_id = nearest_lane->lane().successor_id(i);
      segments_vector.emplace_back(nearest_lanesegment);
      auto next_lane = hdmap_->GetLaneById(next_lane_id);
      LaneSegment next_lanesegment =
          LaneSegment(next_lane, next_lane->accumulate_s().front(),
                      next_lane->accumulate_s().back());
      segments_vector.emplace_back(next_lanesegment);
      size_t succeed_lanes_num = next_lane->lane().successor_id_size();
      if (succeed_lanes_num != 0) {
        for (size_t j = 0; j < succeed_lanes_num; ++j) {
          auto succeed_lane_id = next_lane->lane().successor_id(j);
          auto succeed_lane = hdmap_->GetLaneById(succeed_lane_id);
          LaneSegment succeed_lanesegment =
              LaneSegment(succeed_lane, succeed_lane->accumulate_s().front(),
                          succeed_lane->accumulate_s().back());
          segments_vector.emplace_back(succeed_lanesegment);
        }
      }
      *nearby_path = Path(segments_vector);
      SearchTargetParkingSpotOnPath(*nearby_path, &target_parking_spot);
      if (target_parking_spot != nullptr) {
        break;
      }
    }
  } else {
    segments_vector.emplace_back(nearest_lanesegment);
    *nearby_path = Path(segments_vector);
    SearchTargetParkingSpotOnPath(*nearby_path, &target_parking_spot);
  }

  if (nullptr == target_parking_spot) {
    AERROR << "No such parking spot found after searching all path forward "
              "possible";
    return false;
  }

  if (!CheckDistanceToParkingSpot(frame, *nearby_path, target_parking_spot,
                                  vertices)) {
    AERROR << "target parking spot found, but too far, distance larger than "
              "pre-defined distance";
    return false;
  }

  return true;
}

bool OpenSpaceRoiDecider::GetPullOverSpot(
    Frame *const frame, std::array<Vec2d, 4> *vertices,
    hdmap::Path *nearby_path) {
  const auto &pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (!pull_over_status.has_position() ||
      !pull_over_status.position().has_x() ||
      !pull_over_status.position().has_y() || !pull_over_status.has_theta()) {
    AERROR << "Pull over position not set in planning context";
    return false;
  }

  if (frame->reference_line_info().size() > 1) {
    AERROR << "Should not be in pull over when changing lane in open space "
              "planning";
    return false;
  }

  *nearby_path =
      frame->reference_line_info().front().reference_line().GetMapPath();

  // Construct left_top, left_down, right_down, right_top points
  double pull_over_x = pull_over_status.position().x();
  double pull_over_y = pull_over_status.position().y();
  const double pull_over_theta = pull_over_status.theta();
  const double pull_over_length_front = pull_over_status.length_front();
  const double pull_over_length_back = pull_over_status.length_back();
  const double pull_over_width_left = pull_over_status.width_left();
  const double pull_over_width_right = pull_over_status.width_right();

  Vec2d center_shift_vec((pull_over_length_front - pull_over_length_back) * 0.5,
                         (pull_over_width_left - pull_over_width_right) * 0.5);
  center_shift_vec.SelfRotate(pull_over_theta);
  pull_over_x += center_shift_vec.x();
  pull_over_y += center_shift_vec.y();

  const double half_length =
      (pull_over_length_front + pull_over_length_back) * 0.5;
  const double half_width =
      (pull_over_width_left + pull_over_width_right) * 0.5;

  const double cos_heading = std::cos(pull_over_theta);
  const double sin_heading = std::sin(pull_over_theta);

  const double dx1 = cos_heading * half_length;
  const double dy1 = sin_heading * half_length;
  const double dx2 = sin_heading * half_width;
  const double dy2 = -cos_heading * half_width;

  Vec2d left_top(pull_over_x - dx1 + dx2, pull_over_y - dy1 + dy2);
  Vec2d left_down(pull_over_x - dx1 - dx2, pull_over_y - dy1 - dy2);
  Vec2d right_down(pull_over_x + dx1 - dx2, pull_over_y + dy1 - dy2);
  Vec2d right_top(pull_over_x + dx1 + dx2, pull_over_y + dy1 + dy2);

  std::array<Vec2d, 4> pull_over_vertices{left_top, left_down, right_down,
                                          right_top};
  *vertices = std::move(pull_over_vertices);

  return true;
}

// check whether the target parking space is on the lane
void OpenSpaceRoiDecider::SearchTargetParkingSpotOnLane(
    const LaneInfoConstPtr &lane, ParkingSpaceInfoConstPtr *target_parking_spot) {
  // parking spaces list on the lane
  const auto &parking_space_overlaps = lane->parking_spaces();
  for (const auto &parking_overlap : parking_space_overlaps) {
    for (auto object : parking_overlap->overlap().object()) {
      if (target_parking_spot_id_ == object.id().id()) {
        // find target parking space
        *target_parking_spot = hdmap_->GetParkingSpaceById(object.id());
        return;
      }
    }
  }
}

// find target parking space in the nearby path
void OpenSpaceRoiDecider::SearchTargetParkingSpotOnPath(
    const hdmap::Path &nearby_path,
    ParkingSpaceInfoConstPtr *target_parking_spot) {
  // parking space and lane overlaps
  const auto &parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto &parking_overlap : parking_space_overlaps) {
    // find target parking space
    if (parking_overlap.object_id == target_parking_spot_id_) {
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      *target_parking_spot = hdmap_->GetParkingSpaceById(id);
    }
  }
}

// check distance to parking space is less than threshold
bool OpenSpaceRoiDecider::CheckDistanceToParkingSpot(
    Frame *const frame, const hdmap::Path &nearby_path,
    const hdmap::ParkingSpaceInfoConstPtr &target_parking_spot,
    std::array<Vec2d, 4> *vertices) {

  // check input
  if (nullptr == frame || nullptr == target_parking_spot ||
      nullptr == vertices ||
      target_parking_spot->polygon().points().size() != vertices->size()) {
    AERROR << "Invalid input, cannot compute distance";
    return false;
  }

  // sort corner points (left_down, left_top, right_top, right_down)
  // std::tuple<size_t, double, double> ---> std::tuple<index, s, l>
  bool is_left_lane = true, is_right_lane = true;
  std::vector<std::tuple<size_t, double, double>> sort_corner_points;
  // get parking space s range
  double min_s = DBL_MAX, max_s = -DBL_MAX;
  size_t index = 0;
  double tmp_s = 0.0, tmp_l = 0.0;
  for (auto corner_point : target_parking_spot->polygon().points()) {
    Vec2d point = common::util::PointFactory::ToVec2d(corner_point);
    nearby_path.GetNearestPoint(point, &tmp_s, &tmp_l);
    min_s = std::fmin(min_s, tmp_s);
    max_s = std::fmax(max_s, tmp_s);
    sort_corner_points.emplace_back(index++, tmp_s, tmp_l);
    is_left_lane = is_left_lane && (tmp_l > 0.0);
    is_right_lane = is_right_lane && (tmp_l < 0.0);
  }

  // get parking space position
  if (is_left_lane) {
    parking_space_position_ = ParkingSpacePosition::LEFT;
  } else if (is_right_lane) {
    parking_space_position_ = ParkingSpacePosition::RIGHT;
  } else {
    parking_space_position_ = ParkingSpacePosition::MIDDLE;
  }

  // sort by l
  std::sort(sort_corner_points.begin(), sort_corner_points.end(),
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<2>(a) > std::get<2>(b);
      });
  // sort by s
  std::sort(sort_corner_points.begin(), sort_corner_points.begin() + 2,
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<1>(a) < std::get<1>(b);
      });
  // sort by s
  std::sort(sort_corner_points.begin() + 2, sort_corner_points.end(),
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<1>(a) > std::get<1>(b);
      });

  // get vertices
  for (index = 0; index < sort_corner_points.size(); ++index) {
    auto corner_point = target_parking_spot->polygon().points()[
                            std::get<0>(sort_corner_points[index])];
    vertices->at(index) = common::util::PointFactory::ToVec2d(corner_point);
  }

  // vehicle state s
  Vec2d vehicle_vec = common::util::PointFactory::ToVec2d(vehicle_state_);
  double vehicle_point_s = 0.0, vehicle_point_l = 0.0;
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  // check distance between parking space and vehicle is less than parking_start_range
  if (std::abs((min_s + max_s) * 0.5 - vehicle_point_s) <
      config_.open_space_roi_decider_config().parking_start_range()) {
    return true;
  } else {
    return false;
  }
}

// delete duplicate points
bool OpenSpaceRoiDecider::FuseLineSegments(
    std::vector<std::vector<Vec2d>> *line_segments_vec) {
  static constexpr double kEpsilon = 1.0e-8;
  auto cur_segment = line_segments_vec->begin();
  while (cur_segment != line_segments_vec->end() - 1) {
    // next line segment
    auto next_segment = cur_segment + 1;
    
    // line segment must has more than two points
    if (cur_segment->size() < 2 || next_segment->size() < 2) {
      AERROR << "Single point line_segments vec not expected";
      return false;
    }
    auto cur_last_point = cur_segment->back();
    auto next_first_point = next_segment->front();
    if (cur_last_point.DistanceTo(next_first_point) > kEpsilon) {
      ++cur_segment;
      continue;
    }

    // erase duplicate point
    size_t cur_segments_size = cur_segment->size();
    auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
    auto next_second_point = next_segment->at(1);
    // Straight or counterclockwise will not be erased
    if (CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) <
        0.0) {
      cur_segment->emplace_back(next_second_point);
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

//
bool OpenSpaceRoiDecider::FormulateBoundaryConstraints(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  // Gather vertice needed by warm start and distance approach
  if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
    AERROR << "fail at LoadObstacleInVertices()";
    return false;
  }
  // Transform vertices into the form of Ax>b
  if (!LoadObstacleInHyperPlanes(frame)) {
    AERROR << "fail at LoadObstacleInHyperPlanes()";
    return false;
  }
  return true;
}

//
bool OpenSpaceRoiDecider::LoadObstacleInVertices(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  //
  auto *mutable_open_space_info = frame->mutable_open_space_info();
  const auto &open_space_info = frame->open_space_info();
  // obstacle list
  auto *obstacles_vertices_vec =
      mutable_open_space_info->mutable_obstacles_vertices_vec();
  auto *obstacles_edges_num_vec =
      mutable_open_space_info->mutable_obstacles_edges_num();

  // load vertices for parking boundary (not need to repeat the first
  // vertice to get close hull)
  size_t parking_boundaries_num = roi_parking_boundary.size();
  size_t perception_obstacles_num = 0;

  size_t point_num = 0;
  std::vector<size_t> obstacle_points_num;
  obstacle_points_num.clear();
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    obstacles_vertices_vec->emplace_back(roi_parking_boundary[i]);
  }

  Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num,
                                                     1);
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    CHECK_GT(roi_parking_boundary[i].size(), 1U);
    parking_boundaries_obstacles_edges_num(i, 0) =
        static_cast<int>(roi_parking_boundary[i].size()) - 1;
  }

  // get electirc fence polygon as roi boundary
  std::vector<std::vector<Vec2d>> electric_fences_boundary;
  GetRoiBoundaryFromElectricFence(vehicle_state_, electric_fences_boundary);
  size_t electric_boundaries_num = electric_fences_boundary.size();
  for (size_t i = 0; i < electric_boundaries_num; ++i) {
    if (electric_fences_boundary.at(i).empty()) {
      continue;
    }
    obstacles_vertices_vec->emplace_back(electric_fences_boundary.at(i));
  }
  Eigen::MatrixXi electric_boundaries_obstacles_edges_num(
      electric_boundaries_num, 1);
  for (size_t i = 0; i < electric_boundaries_num; ++i) {
    CHECK_GT(electric_fences_boundary.at(i).size(), 1U);
    electric_boundaries_obstacles_edges_num(i, 0) =
        static_cast<int>(electric_fences_boundary.at(i).size()) - 1;
  }

  bool enable_polygon_plan = FLAGS_enable_openspace_use_polygon_plan;
  std::vector<std::vector<common::math::Vec2d>> all_obstacle_points;
  if (config_.open_space_roi_decider_config().enable_perception_obstacles()) {
    // load vertices for perception obstacles(repeat the first vertice at
    // the last to form closed convex hull)
    const auto &origin_point = open_space_info.origin_point();
    const auto &origin_heading = open_space_info.origin_heading();
    for (const auto &obstacle : obstacles_by_frame_->Items()) {
      // filter obstacle
      if (FilterOutObstacle(*frame, *obstacle)) {
        continue;
      }
      if (!enable_polygon_plan) {
        ++perception_obstacles_num;

        // obstacle bounding box in origin axis
        Box2d original_box = obstacle->PerceptionBoundingBox();
        original_box.Shift(-1.0 * origin_point);
        original_box.LongitudinalExtend(config_.open_space_roi_decider_config()
                                            .perception_obstacle_buffer());
        original_box.LateralExtend(config_.open_space_roi_decider_config()
                                       .perception_obstacle_buffer());

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
        all_obstacle_points.emplace_back(vertices_cw);
        obstacles_vertices_vec->emplace_back(vertices_cw);
      } else {
        // take vector from polygon
        ++perception_obstacles_num;
        const Polygon2d &obstacle_polygon =
            obstacle->PerceptionPolygon().ExpandByDistance(
                0.5 * config_.open_space_roi_decider_config()
                          .perception_obstacle_buffer());
        std::vector<Vec2d> vertices_cw;
        for (auto pt : obstacle_polygon.points()) {
          pt -= origin_point;
          pt.SelfRotate(-origin_heading);
          ++point_num;
          vertices_cw.emplace_back(pt);
        }
        if (point_num <= 2) {
          AERROR << "there is a invalid obj point_num " << point_num;
        }
        obstacle_points_num.emplace_back(point_num);
        point_num = 0;
        // As the perception obstacle is a closed convex set, the first
        // vertice is repeated at the end of the vector to help transform all
        // four edges to inequality constraint
        // 5 point means 4 line segement
        vertices_cw.emplace_back(vertices_cw.front());
        all_obstacle_points.emplace_back(vertices_cw);
        obstacles_vertices_vec->emplace_back(vertices_cw);
      }
    }
    HybridAStarBidirectional::Build(all_obstacle_points);

    // obstacle boundary box is used, thus the edges are set to be 4
    // lwt: notice this

    Eigen::MatrixXi perception_obstacles_edges_num =
        Eigen::MatrixXi::Ones(perception_obstacles_num, 1);

    if (enable_polygon_plan) {
      ACHECK(obstacle_points_num.size() == perception_obstacles_num);
    }

    for (size_t i = 0; i < perception_obstacles_num; ++i) {
      perception_obstacles_edges_num(i, 0) =
          enable_polygon_plan ? obstacle_points_num[i] : 4;
    }

    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows() +
            perception_obstacles_edges_num.rows() +
            electric_boundaries_obstacles_edges_num.rows(),
        1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num,
        perception_obstacles_edges_num, electric_boundaries_obstacles_edges_num;

  } else {
    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows() +
            electric_boundaries_obstacles_edges_num.rows(),
        1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num,
        electric_boundaries_obstacles_edges_num;
  }

  mutable_open_space_info->set_obstacles_num(parking_boundaries_num +
                                             perception_obstacles_num +
                                             electric_boundaries_num);
  //
  return true;
}

double OpenSpaceRoiDecider::GetObstacleDist(const Vec2d &point,
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
  const auto &adc_polygon = Polygon2d(adc_box);

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
        AERROR << "dist too little , think block" << obstacle->Id();
        return -1.0;
      }
    }
  }
  return min_dist;
}

// filter obstacle
bool OpenSpaceRoiDecider::FilterOutObstacle(const Frame &frame,
                                            const Obstacle &obstacle) {
  if (obstacle.IsVirtual()) {
    ADEBUG << "virtual obstacle, need filter out, id: " << obstacle.Id();
    return true;
  }
  if (!obstacle.IsStatic()) {
    ADEBUG << "dynamic obstacle, need filter out, id: " << obstacle.Id();
    return true;
  }
  if (obstacle.IsCross()) {
    ADEBUG << "cross obstacle, need filter out, id: " << obstacle.Id();
    return true;
  }

  const auto &open_space_info = frame.open_space_info();
  const auto &origin_point = open_space_info.origin_point();
  const auto &origin_heading = open_space_info.origin_heading();
  const auto &obstacle_box = obstacle.PerceptionBoundingBox();
  const auto &obstacle_polygon = obstacle.PerceptionPolygon();

  // xy_boundary in xmin, xmax, ymin, ymax.
  const auto &roi_xy_boundary = open_space_info.ROI_xy_boundary();
  bool is_in_roi = false;
  for (auto pt : obstacle_polygon.points()) {
    if (roi_xy_boundary.empty()) {
      break;
    }
    // world ---> origin
    pt -= origin_point;
    pt.SelfRotate(-origin_heading);
    // check whether the obstacle polygon is in roi boundary
    is_in_roi = !(pt.x() < roi_xy_boundary[0] || pt.x() > roi_xy_boundary[1] ||
                  pt.y() < roi_xy_boundary[2] || pt.y() > roi_xy_boundary[3]);
    if (is_in_roi) {
      break;
    }
  }
  if (!is_in_roi) {
    return true;
  }

  // Translate the end pose back to world frame with endpose in x, y, phi, v
  const auto &end_pose = open_space_info.open_space_end_pose();
  // origin ---> world
  Vec2d end_pose_x_y(end_pose[0], end_pose[1]);
  end_pose_x_y.SelfRotate(origin_heading);
  end_pose_x_y += origin_point;

  // Get vehicle state
  Vec2d vehicle_x_y(vehicle_state_.x(), vehicle_state_.y());

  // Use vehicle position and end position to filter out obstacle
  double vehicle_center_to_obstacle = 0.0;
  double end_pose_center_to_obstacle = 0.0;
  const auto &obs_type = obstacle.Perception().type();
  if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type) {
    vehicle_center_to_obstacle = obstacle_polygon.DistanceTo(vehicle_x_y);
    end_pose_center_to_obstacle = obstacle_polygon.DistanceTo(end_pose_x_y);
  } else {
    vehicle_center_to_obstacle = obstacle_box.DistanceTo(vehicle_x_y);
    end_pose_center_to_obstacle = obstacle_box.DistanceTo(end_pose_x_y);
  }
  const double filtering_distance =
      config_.open_space_roi_decider_config()
          .perception_obstacle_filtering_distance();
  if (vehicle_center_to_obstacle > filtering_distance &&
      end_pose_center_to_obstacle > filtering_distance) {
    AERROR << "obj is too far away, return true.";
    return true;
  }
  return false;
}

// y = ax + b
bool OpenSpaceRoiDecider::LoadObstacleInHyperPlanes(Frame *const frame) {
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

bool OpenSpaceRoiDecider::GetHyperPlanes(
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

// check whether vehicle is in parking space
bool OpenSpaceRoiDecider::IsInParkingLot(
    const double adc_init_x, const double adc_init_y,
    const double adc_init_heading,
    const hdmap::Path &nearby_path, std::array<Vec2d, 4> *parking_space_vertices) {

  // target parking spot id is set
  if (!target_parking_spot_id_.empty()) {
    Id id;
    id.set_id(target_parking_spot_id_);
    auto parking_space = hdmap_->GetParkingSpaceById(id);
    return GetParkSpotFromMap(parking_space, nearby_path, parking_space_vertices);
  }

  // make sure there is only one parking lot in search range
  auto adc_parking_spot =
      common::util::PointFactory::ToPointENU(adc_init_x, adc_init_y, 0);

  // vehicle bounding box
  const Vec2d adc_init_position{adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  double shift_distance =
      adc_length * 0.5 - vehicle_params_.back_edge_to_center();
  const Vec2d shift_vec{shift_distance * std::cos(adc_init_heading),
                        shift_distance * std::sin(adc_init_heading)};
  adc_box.Shift(shift_vec);

  // get nearest parking spaces
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  hdmap_->GetParkingSpaces(adc_parking_spot, kInParkingLot, &parking_spaces);
  for (auto parking_space : parking_spaces) {
    Polygon2d parking_space_polygon = parking_space->polygon();
    if (parking_space_polygon.Contains(Polygon2d(adc_box))) {
      // get parking space corner points
      return GetParkSpotFromMap(parking_space, nearby_path, parking_space_vertices);
    }
  }
  return false;
}

// get parking space corner points
bool OpenSpaceRoiDecider::GetParkSpotFromMap(
    ParkingSpaceInfoConstPtr parking_space, const hdmap::Path &nearby_path,
    std::array<Vec2d, 4> *vertices) {

  // check input
  if (nullptr == parking_space ||
      parking_space->polygon().points().size() != vertices->size()) {
    return false;
  }

  // sort corner points (left_down, left_top, right_top, right_down)
  // std::tuple<size_t, double, double> ---> std::tuple<index, s, l>
  bool is_left_lane = true, is_right_lane = true;
  std::vector<std::tuple<size_t, double, double>> sort_corner_points;
  size_t index = 0;
  double tmp_s = 0.0, tmp_l = 0.0;
  for (auto corner_point : parking_space->polygon().points()) {
    Vec2d point = common::util::PointFactory::ToVec2d(corner_point);
    nearby_path.GetNearestPoint(point, &tmp_s, &tmp_l);
    sort_corner_points.emplace_back(index++, tmp_s, tmp_l);
    is_left_lane = is_left_lane && (tmp_l > 0.0);
    is_right_lane = is_right_lane && (tmp_l < 0.0);
  }

  // get parking space position
  if (is_left_lane) {
    parking_space_position_ = ParkingSpacePosition::LEFT;
  } else if (is_right_lane) {
    parking_space_position_ = ParkingSpacePosition::RIGHT;
  } else {
    parking_space_position_ = ParkingSpacePosition::MIDDLE;
  }

  // sort by l
  std::sort(sort_corner_points.begin(), sort_corner_points.end(),
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<2>(a) > std::get<2>(b);
      });
  // sort by s
  std::sort(sort_corner_points.begin(), sort_corner_points.begin() + 2,
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<1>(a) < std::get<1>(b);
      });
  // sort by s
  std::sort(sort_corner_points.begin() + 2, sort_corner_points.end(),
      [](std::tuple<size_t, double, double> a, std::tuple<size_t, double, double> b) {
        return std::get<1>(a) > std::get<1>(b);
      });

  // get vertices
  for (index = 0; index < sort_corner_points.size(); ++index) {
    auto corner_point = parking_space->polygon().points()[
                            std::get<0>(sort_corner_points[index])];
    vertices->at(index) = Vec2d(corner_point.x(), corner_point.y());
  }
  target_parking_spot_id_ = parking_space->id().id();
  target_parking_spot_ = parking_space;
  return true;
}

// @brief Check if adc blocked with front obs
bool OpenSpaceRoiDecider::CheckADCIsBlockedWithSurroundObstacles(
    const Vec2d adc_position, const double adc_heading,
    const Frame *frame, const double front_obstacle_buffer,
    const double shift_dist) {
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width();
  // ADC box , 0.5 * adc_width as lat buffer
  Box2d adc_box(adc_position, adc_heading, adc_length + front_obstacle_buffer,
                adc_width + 0.5 * adc_width);
  double shift_distance = shift_dist + adc_length * 0.5 -
                          vehicle_config.vehicle_param().back_edge_to_center();

  Vec2d shift_vec{shift_distance * std::cos(adc_heading),
                  shift_distance * std::sin(adc_heading)};
  adc_box.Shift(shift_vec);
  const auto &adc_polygon = Polygon2d(adc_box);
  auto obstacles = frame->obstacles();
  for (const auto &obstacle : obstacles) {
    if (obstacle->IsVirtual() || obstacle->speed() > kDynamicObstacleSpeed) {
      continue;
    }
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();

    if (adc_polygon.HasOverlap(obstacle_polygon)) {
      return true;
    }
  }
  return false;
}

void OpenSpaceRoiDecider::GetLaneBoundaryPoints(
    LaneInfoConstPtr lane_info, const bool &is_left_bound,
    const hdmap::Path &nearby_path, std::vector<Vec2d> *const boundary_points) {
  const auto &boundary_segment =
      is_left_bound ? lane_info->lane().left_boundary().curve().segment()
                    : lane_info->lane().right_boundary().curve().segment();
  size_t lane_points_num = boundary_segment[0].line_segment().point_size();
  double point_s = 0.0;
  double point_l = 0.0;
  double car_s, car_l;
  Vec2d car_position = {temp_state_.x(), temp_state_.y()};
  lane_info->GetProjection(car_position, &car_s, &car_l);

  double start_s =
      car_s -
      config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  double end_s =
      car_s +
      config_.open_space_roi_decider_config().roi_longitudinal_range_end();

  // TODO(wentao liu): one lane may not enough , add more
  ADEBUG << "start_s " << start_s << " end_s " << end_s;
  if (start_s < 0) {
    start_s = 0.0;
  }

  double top_s, top_l;
  for (size_t i = 0; i < lane_points_num; ++i) {
    const auto &point = boundary_segment[0].line_segment().point().at(i);
    Vec2d point_v = {point.x(), point.y()};
    lane_info->GetProjection(point_v, &point_s, &point_l);
    if (point_s > start_s && point_s < end_s) {
      nearby_path.GetProjection(point_v, &top_s, &top_l);
      boundary_points->emplace_back(point_v);
    }
  }
}

// @brief Get rescue lane boundaries of both sides
void OpenSpaceRoiDecider::GetRescueLaneBoundary(
    const hdmap::Path &nearby_path, const Vec2d &origin_point,
    const double origin_heading, std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary) {
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  hdmap::Id left_neighbor_lane_id;
  hdmap::Id left_neighbor_forward_lane_id;
  hdmap::Id left_neighbor_reverse_lane_id;
  hdmap::Id right_neighbor_forward_lane_id;
  LaneInfoConstPtr car_lane;
  PointENU car_pose;
  // get the center point of start to end
  Vec2d origin_position = {origin_point.x(), origin_point.y()};
  Vec2d end_position = {rescue_end_point_.x(), rescue_end_point_.y()};
  Vec2d origin_to_end = end_position - origin_position;
  Vec2d target_position = {origin_position.x() + origin_to_end.x() * 0.5,
                           origin_position.y() + origin_to_end.y() * 0.5};
  car_pose.set_x(target_position.x());
  car_pose.set_y(target_position.y());

  hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s, &vehicle_lane_l);
  if (nullptr == car_lane.get()) {
    AERROR << "hdmap_->GetNearestLane failed with pos x: " << car_pose.x()
           << " y: " << car_pose.y();
    return;
  }

  // Determine left boundary
  // 0  Left of this lane;  1  Left of the same direction lane;
  // 2  Right of the reverse direction lane
  int left_lane_boundary_type = 0;
  //  Get the left boundary type of this lane: solid line, dotted line
  LaneBoundaryType::Type lane_boundary_type = LaneBoundaryType::UNKNOWN;
  if (car_lane->lane().has_left_boundary()) {
    if (!car_lane->lane().left_boundary().boundary_type().empty()) {
      lane_boundary_type =
          car_lane->lane().left_boundary().boundary_type(0).types(0);
    }
  }

  // If it is a white dotted line/yellow dotted line, the boundary can be
  // extended to the left. Otherwise, use the left boundary of this lane.
  if (LaneBoundaryType::DOTTED_WHITE == lane_boundary_type ||
      LaneBoundaryType::DOTTED_YELLOW == lane_boundary_type) {
    //  If this lane has adjacent lanes in the same direction, the first lane
    //  on the left will be obtained
    if (!car_lane->lane().left_neighbor_forward_lane_id().empty()) {
      left_neighbor_forward_lane_id =
          car_lane->lane().left_neighbor_forward_lane_id(0);

      const auto &left_neighbor_forward_lane =
          hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
              left_neighbor_forward_lane_id);
      if (nullptr == left_neighbor_forward_lane) {
        AERROR << "Get left_neighbor_forward_lane failed, use current lane";
        GetRescueLaneBoundaryPoints(car_lane, true, nearby_path,
                                    left_lane_boundary);
      } else {
        left_lane_boundary_type = 1;  // 1 Left of the same direction lane;
        GetRescueLaneBoundaryPoints(left_neighbor_forward_lane, true,
                                    nearby_path, left_lane_boundary);
      }
    } else if (!car_lane->lane().left_neighbor_reverse_lane_id().empty()) {
      // If there are adjacent lanes in the reverse direction of this lane,
      // the first lane on the left of the reverse direction is obtained
      left_neighbor_reverse_lane_id =
          car_lane->lane().left_neighbor_reverse_lane_id(0);

      const auto &left_neighbor_reverse_lane =
          hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
              left_neighbor_reverse_lane_id);
      if (nullptr == left_neighbor_reverse_lane) {
        AERROR << "Get left_neighbor_reverse_lane failed, use current lane";
        GetRescueLaneBoundaryPoints(car_lane, true, nearby_path,
                                    left_lane_boundary);
      } else {
        left_lane_boundary_type = 2;  // 2 Right of the reverse direction lane
        GetRescueLaneBoundaryPoints(left_neighbor_reverse_lane, false,
                                    nearby_path, left_lane_boundary);
      }
    } else {
      AERROR << "Get left_neighbor_forward_lane / left_neighbor_reverse_lane "
                "failed, use current lane";
      GetRescueLaneBoundaryPoints(car_lane, true, nearby_path,
                                  left_lane_boundary);
    }
  } else {
    GetRescueLaneBoundaryPoints(car_lane, true, nearby_path,
                                left_lane_boundary);
  }
  AINFO << "left_lane_boundary_type is: " << left_lane_boundary_type;

  // Determine right boundary
  // 0  Right of this lane
  // 1  Right of the first lane in the same direction on the right
  int right_lane_boundary_type = 0;
  // Get the right boundary type of this lane: solid line, dotted line
  lane_boundary_type = LaneBoundaryType::UNKNOWN;
  if (car_lane->lane().has_right_boundary()) {
    if (!car_lane->lane().right_boundary().boundary_type().empty()) {
      lane_boundary_type =
          car_lane->lane().right_boundary().boundary_type(0).types(0);
    }
  }

  // If the left boundary of the lane is obtained on the left, other lanes on
  // the right will be considered. Otherwise, obtain the right boundary of
  // this lane.
  if (0 == left_lane_boundary_type) {
    // If it is a white dotted line/yellow dotted line, the boundary can be
    // extended to the right. Otherwise, use the right boundary of this lane.
    if (LaneBoundaryType::DOTTED_WHITE == lane_boundary_type ||
        LaneBoundaryType::DOTTED_YELLOW == lane_boundary_type) {
      // If this lane has adjacent lanes in the same direction, the first
      // lane on the right is obtained
      if (!car_lane->lane().right_neighbor_forward_lane_id().empty()) {
        right_neighbor_forward_lane_id =
            car_lane->lane().right_neighbor_forward_lane_id(0);

        const auto &right_neighbor_forward_lane =
            hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
                right_neighbor_forward_lane_id);
        if (nullptr == right_neighbor_forward_lane) {
          AERROR << "Get right_neighbor_forward_lane failed, use current "
                    "right lane boundary.";
          GetRescueLaneBoundaryPoints(car_lane, false, nearby_path,
                                      right_lane_boundary);
        } else {
          right_lane_boundary_type = 1;
          GetRescueLaneBoundaryPoints(right_neighbor_forward_lane, false,
                                      nearby_path, right_lane_boundary);
        }
      } else {
        AERROR << "Get right_neighbor_forward_lane failed, use current right "
                  "lane boundary.";
        GetRescueLaneBoundaryPoints(car_lane, false, nearby_path,
                                    right_lane_boundary);
      }
    } else {
      GetRescueLaneBoundaryPoints(car_lane, false, nearby_path,
                                  right_lane_boundary);
    }
  } else {
    GetRescueLaneBoundaryPoints(car_lane, false, nearby_path,
                                right_lane_boundary);
  }
  AINFO << "right_lane_boundary_type is: " << right_lane_boundary_type;

  // turn lane boundary to origin
  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  for (size_t i = 0; i < left_point_size; ++i) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
  for (size_t i = 0; i < right_point_size; ++i) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
}

// @brief just for GetRescueLaneBoundaryPoints
void OpenSpaceRoiDecider::GetRescueLaneBoundaryPoints(
    hdmap::LaneInfoConstPtr lane_info, const bool& is_left_bound,
    const hdmap::Path& nearby_path, std::vector<Vec2d>* const boundary_points) {
  return;
}

void OpenSpaceRoiDecider::InterpolateBoundary(
    const double &s_interval, const double &heading_interval,
    std::vector<Vec2d> *boundarys) {
  std::vector<Vec2d> boundary_points;
  std::copy(boundarys->begin(), boundarys->end(),
            std::back_inserter(boundary_points));
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
}

bool OpenSpaceRoiDecider::GetOperationGoalPoint(
    const Frame *const frame, std::vector<PointWithCost> &goal_points) {
  PointWithCost point;
  if (injector_->openspace_reason() != OpenspaceReason::OPERATION) {
    AERROR << "Not OPERATION conditon for rescue, return false.";
    return false;
  }
  if (nullptr == frame->local_view().routing ||
      !frame->local_view().routing->has_routing_request()) {
    AERROR
        << "Get Operation Goal Point, routing or routing request is empty.";
    return false;
  }
  if (frame->local_view().routing->routing_request().waypoint().empty()) {
    AERROR << "waypoints are empty, could not find goal point.";
    return false;
  }
  const auto& routing_request = frame->local_view().routing->routing_request();
  size_t waypoint_num = routing_request.waypoint().size();
  auto end_waypoint = frame->local_view()
                          .routing->routing_request()
                          .waypoint()
                          .at(waypoint_num - 1)
                          .pose();
  double end_waypoint_heading =
      routing_request.waypoint().at(waypoint_num - 1).heading();

  AINFO << "before_calculate_operation condition, goal point: ("
        << end_waypoint.x() << "," << end_waypoint.y() << ", "
        << end_waypoint.z() << ", " << end_waypoint_heading * 180 / M_PI << ")";

  common::VehicleState new_end_waypoint;
  new_end_waypoint.set_x(end_waypoint.x());
  new_end_waypoint.set_y(end_waypoint.y());
  new_end_waypoint.set_z(end_waypoint.z());
  century::common::PointENU routing_lane_point;
  if (!util::GetNearestPointFromRoutingLane(new_end_waypoint, routing_request,
                                            routing_lane_point,
                                            end_waypoint_heading)) {
    AERROR << "[error], Get Operation Goal Point, failed to get point in "
              "lane according to routing end point.";
    // fallback
    const auto& adc_heading = vehicle_state_.heading();
    double dx_offset = std::cos(adc_heading);
    double dy_offset = std::sin(adc_heading);
    std::pair<double, double> offset_reouting_lane_point = util::offsetPoint(
        end_waypoint.x(), end_waypoint.y(), dx_offset, dy_offset, 0.5,
        util::OffsetDirection::LEFT, util::CoordinateSystem::RFU);
    common::math::Vec2d target_point{offset_reouting_lane_point.first,
                                     offset_reouting_lane_point.second};
    point = {target_point, adc_heading, 0.0};
    goal_points.emplace_back(point);
    AERROR
        << "get rearest point from routing lane failed, fallback goal point: ("
        << end_waypoint.x() << "," << end_waypoint.y() << ", "
        << end_waypoint.z() << ", " << adc_heading * 180 / M_PI << ")";
    return true;
  }
  AINFO << "find_routing_lane_point, goal point: (" << routing_lane_point.x()
        << "," << routing_lane_point.y() << ", " << routing_lane_point.z()
        << ", " << end_waypoint_heading * 180 / M_PI << ")";

  double dx_offset = std::cos(end_waypoint_heading);
  double dy_offset = std::sin(end_waypoint_heading);
  std::pair<double, double> offset_reouting_lane_point = util::offsetPoint(
      routing_lane_point.x(), routing_lane_point.y(), dx_offset, dy_offset, 0.5,
      util::OffsetDirection::LEFT, util::CoordinateSystem::RFU);

  common::math::Vec2d target_point{offset_reouting_lane_point.first,
                                   offset_reouting_lane_point.second};
  point = {target_point, end_waypoint_heading, 0.0};
  goal_points.emplace_back(point);
  AINFO << "offset_operation condition, goal point: (" << target_point.x()
        << "," << target_point.y() << ", " << end_waypoint_heading * 180 / M_PI
        << ")";
  return true;
}

// lwt:the second sampling is more sparse in long direction;
// the second sampling start point is first sample end point;
void OpenSpaceRoiDecider::GenerateSampleGoals(
    const common::SLPoint &start_point, const double &long_start_s,
    const double &long_end_s, const double &long_interval_s, const Frame *frame,
    const ReferenceLine &reference_line) {
  goals_vector_.clear();
  LaneInfoConstPtr car_lane;
  PointENU car_pose;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  const double half_adc_width = 0.5 * vehicle_params_.width();

  if (GetOperationGoalPoint(frame, goals_vector_)) {
    AERROR << "Calculate operation condition goal point.";
    return;
  }

  for (double s = start_point.s() + long_start_s;
       s < start_point.s() + long_end_s; s = s + long_interval_s) {
    // get current s lane width
    double left = 0.0;
    double right = 0.0;
    if (!reference_line.GetLaneWidth(s, &left, &right)) {
      AERROR << "reference_line GetLaneWidth failed.";
      continue;
    }
    /*
      left ->  +
      right -> -
    */
    left -= FLAGS_rescue_hybird_lat_buffer;
    right = -right + FLAGS_rescue_hybird_lat_buffer;
    car_pose.set_x(reference_line.GetReferencePoint(s).x());
    car_pose.set_y(reference_line.GetReferencePoint(s).y());
    hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s,
                           &vehicle_lane_l);
    if (nullptr == car_lane.get()) {
      AERROR << "hdmap_->GetNearestLane failed with pos x: " << car_pose.x()
             << " y: " << car_pose.y();
    } else {
      if (!car_lane->lane().left_neighbor_reverse_lane_id().empty() ||
          !car_lane->lane().left_neighbor_forward_lane_id().empty()) {
        left += (left - right);
      } else {
        AERROR << "notice! there is no neighbor lane";
      }
    }

    for (double l =
             right + half_adc_width + FLAGS_rescue_hybird_lat_sample_interval;
         l + half_adc_width <= left;
         l += FLAGS_rescue_hybird_lat_sample_interval) {
      common::SLPoint sl_point;
      sl_point.set_s(s);
      sl_point.set_l(l);
      common::math::Vec2d sample_point;
      if (!reference_line.SLToXY(sl_point, &sample_point)) {
        AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
        continue;
      }

      double lat_cost = kRescueLatCost * (std::fabs(l - kPreferL));
      double lon_cost =
          kBaseCost - kRescueLonCost * std::max(s - start_point.s(), 0.0);
      double obstacle_dist_cost = 0.0;
      double ref_heading = reference_line.GetReferencePoint(s).heading();
      double dist = GetObstacleDist(sample_point, ref_heading);
      if (common::util::IsFloatEqual(dist, 0.0)) {
        AERROR << "the dist is zero! give another value: "
               << FLAGS_numerical_epsilon;
        dist = FLAGS_numerical_epsilon;
      }
      if (dist < 0.0) {
        if (goals_vector_.size() < kMinGoalNum) {
          obstacle_dist_cost = kCollisionCost;
        } else {
          continue;
        }
      } else if (dist > FLAGS_rescue_hybird_ingore_distance) {
        obstacle_dist_cost = 0.0;
      } else {
        obstacle_dist_cost = kRescueObstacleCost / dist;
      }

      double front_blocked_obstacle_dist_cost = 0.0;

      // it cost the most time
      if (FLAGS_enable_rescue_surround_cost) {
        bool is_sample_adc_blocked_with_front_obstacle =
            CheckADCIsBlockedWithSurroundObstacles(
                sample_point, ref_heading, frame, kCheckDist, kShiftDist);
        if (is_sample_adc_blocked_with_front_obstacle) {
          front_blocked_obstacle_dist_cost = kFrontBlockedCost;
        }
      }

      double total_cost = lat_cost + lon_cost + obstacle_dist_cost +
                          front_blocked_obstacle_dist_cost;
      PointWithCost temp_point = {sample_point, ref_heading, total_cost};
      goals_vector_.emplace_back(temp_point);
    }
  }
  AINFO << "goals_vector_size: " << goals_vector_.size();
  for (size_t i = 0; i + 1 < goals_vector_.size(); ++i) {
    for (size_t j = 0; j + 1 < goals_vector_.size() - i; ++j) {
      if (goals_vector_[j].cost < goals_vector_[j + 1].cost) {
        PointWithCost temp = goals_vector_[j + 1];
        goals_vector_[j + 1] = goals_vector_[j];
        goals_vector_[j] = temp;
      }
    }
  }
}

void OpenSpaceRoiDecider::ReuseLastInfo(Frame *const frame) {
  const auto &ptr_last_frame = injector_->frame_history()->Latest();

  *(frame->mutable_open_space_info()->mutable_open_space_end_pose()) =
      ptr_last_frame->open_space_info().open_space_end_pose();
}

}  // namespace planning
}  // namespace century
