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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_value.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory1d/constant_acc_to_target_v_trajectory1d.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::SpeedPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Box2d;
using century::perception::PerceptionObstacle;
using century::planning_internal::StGraphBoundaryDebug;
using century::planning_internal::STGraphDebug;
namespace {
constexpr uint32_t kFirstPathStep = 1;
constexpr double kSpeedLimitDistance = 5.0;
constexpr double kPredictionTime = 8.0;
constexpr double kMinSpeedLimitDistance = 10.0;
constexpr double kMaxSpeedLimitDistance = 80.0;
constexpr double kSpeedCoeff = 0.8;
constexpr double kLargeAeraBuffer = 2.0;
constexpr double kTurnScopeBuffer = 1.0;
constexpr double kDegrees = 90.0;
constexpr double kMinYieldDistance = 15.0;
constexpr size_t kStartUpAddCount = 10UL;
constexpr size_t kHyCapacity = 200UL;
constexpr double kMinDecel = -6.0;
constexpr double kMinSpeedCoeff = 1.2;
constexpr double kSecondSpeedCoeff = 1.5;
constexpr double kMaxSpeedCoeff = 2.0;
constexpr double kLonBuffer = 1.0;
constexpr double kEpison = 1e-6;
constexpr double kDiffHeading = 10.0;
constexpr double kQuadrantAngle = 90.0;
constexpr double kMaxReverseSpeed = -0.5;
constexpr double kMinKappa = 0.005;
constexpr double kConsiderLen = 10.0;
constexpr double kMaxTargetSpeed = 1e10;
constexpr size_t kTimeSteps = 71;
constexpr double kLatBuffer = 0.5;
constexpr double kStraightAngle = 10.0;
constexpr double kComfortDecel = 0.5;
constexpr double kMinReverseSpeed = -1.0;
constexpr double kLateralBuffer = 1.0;
constexpr double kStopBuffer = 2.0;
constexpr double kTargetLowspeed = 2.0;
constexpr double kLonSpeedDiff = 0.5;
constexpr double kMinLateralSpeed = 0.1;
constexpr double kMinSpeedDiff = 1.0;
constexpr double kLonStopBuffer = 3.0;
constexpr double kMaxSpeedDiff = 6.0;
constexpr double kSqrtTwo = std::sqrt(2.0);
constexpr double kLateralConsiderBuffer = 0.5;
constexpr double kMinDeltaTime = 0.01;
constexpr double kMaxDeltaTime = 10.0;
constexpr int kFeatureSize = 20;
constexpr double kLatBaseDisForNegative = 100.0;
constexpr uint32_t kLowNumObsoleteSeqNum = 5;
constexpr uint32_t kHighNumObsoleteSeqNum = 2;
}  // namespace

static ObstacleHistoryValue obstacles_history_value(50UL);
static ObstacleHistoryValue obstacles_history_value_for_speed_limit(50UL);
static ObstacleHistoryValue slow_breking_obstacles_history_value(50UL);
static ObstacleHistoryValue
    slow_breking_obstacles_history_value_loose_constraint(50UL);
static ObstacleHistoryDiffValue obstacle_history_diff_value(50UL);
static boost::filesystem::path folder_path =
    "/century/modules/planning/pipeline/data";
static const char *folder_path_feature_and_label =
    "/century/modules/planning/pipeline/feature_label/feature_and_label.txt";
static const char *folder_path_result =
    "/century/modules/planning/pipeline/result";
static const char *model_name =
    "/century/modules/planning/learning_based/start_up_learning_tool/"
    "start_up.xml";

SpeedBoundsDecider::DangerousObsContainer SpeedBoundsDecider::obs_container_;
SpeedBoundsDecider::DangerousObsContainer
    SpeedBoundsDecider::vehicle_container_;

SpeedBoundsDecider::DangerousObsForSpeedLimitContainer
    SpeedBoundsDecider::speed_limit_obs_container_;
size_t SpeedBoundsDecider::start_up_count_ = 0UL;
size_t SpeedBoundsDecider::start_up_extra_times_for_slower_ = 0UL;

SpeedBoundsDecider::SpeedBoundsDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  ACHECK(config.has_speed_bounds_decider_config());
  speed_bounds_config_ = config.speed_bounds_decider_config();
  speed_limit_config_ = speed_bounds_config_.speed_limit_config();
  if (FLAGS_enable_use_svm_model) {
    // TODO(zongxingguo): check file is empty.
    model_ = cv::ml::SVM::load(model_name);
    AINFO << "add model";
  }
}

bool SpeedBoundsDecider::GetStartUpStatus() {
  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  if (adc_speed < speed_bounds_config_.stoped_speed_for_start_up()) {
    start_up_count_ = speed_bounds_config_.stoped_delay_time_for_start_up();
    start_up_extra_times_for_slower_ = 0UL;
  } else if (start_up_count_ <
                 speed_bounds_config_.nearly_stoped_delay_time_for_start_up() &&
             adc_speed <
                 speed_bounds_config_.nearly_stoped_speed_for_start_up()) {
    start_up_count_ =
        speed_bounds_config_.nearly_stoped_delay_time_for_start_up();
  } else if (start_up_count_ > 0UL) {
    --start_up_count_;
  }
  if (start_up_extra_times_for_slower_ < 2UL && 1UL == start_up_count_ &&
      adc_speed < speed_bounds_config_.slower_speed_for_start_up()) {
    start_up_count_ += kStartUpAddCount;
    ++start_up_extra_times_for_slower_;
  }
  ADEBUG << "adc_speed = " << adc_speed;
  ADEBUG << "start_up_count_ = " << start_up_count_;
  bool is_start_up =
      start_up_count_ > 0UL &&
      adc_speed < speed_bounds_config_.max_speed_to_ignore_dangerous_start_up();
  return is_start_up;
}

bool SpeedBoundsDecider::GetAdcCornerPoint(
    const common::PathPoint &path_point,
    common::math::Vec2d *ptr_left_front) const {
  if (nullptr == ptr_left_front) {
    return false;
  }
  // Get ADC's center point from vehicle param.
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Convert reference point from center of rear axis to center of ADC.
  common::math::Vec2d ego_center_map_frame(
      (vehicle_param.front_edge_to_center() -
       vehicle_param.back_edge_to_center()) *
          0.5,
      (vehicle_param.left_edge_to_center() -
       vehicle_param.right_edge_to_center()) *
          0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  common::math::Vec2d adc_center(ego_center_map_frame.x() + path_point.x(),
                                 ego_center_map_frame.y() + path_point.y());
  // Calculate the ADC's left and right front corner points.
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  const double dx1 = cos_heading * vehicle_param.length() * 0.5;
  const double dy1 = sin_heading * vehicle_param.length() * 0.5;
  const double dx2 = sin_heading * vehicle_param.width() * 0.5;
  const double dy2 = -cos_heading * vehicle_param.width() * 0.5;
  common::math::Vec2d right_front(adc_center.x() + dx1 + dx2,
                                  adc_center.y() + dy1 + dy2);
  common::math::Vec2d left_front(adc_center.x() + dx1 - dx2,
                                 adc_center.y() + dy1 - dy2);
  // Log the ADC's left and right front corner points.
  ADEBUG << "right_front:(" << std::fixed << std::setprecision(2)
         << right_front.x() << ", " << right_front.y() << ")";
  ADEBUG << "left_front:(" << std::fixed << std::setprecision(2)
         << left_front.x() << ", " << left_front.y() << ")";
  // Set the left and right front corner points.
  *ptr_left_front = left_front;
  return true;
}

bool SpeedBoundsDecider::IsMergeArea(
    const double s,
    const hdmap::LaneInfoConstPtr &ptr_left_neighbor_lane) const {
  // get the lane segments of reference line
  const auto &lane_segments =
      reference_line_info_->reference_line().map_path().lane_segments();
  // get the index of adc_sucessor_lane
  int adc_sucessor_lane_index =
      reference_line_info_->reference_line().map_path().GetLaneIndexFromS(s).id;
  // if there is a successor lane, then the index of successor lane should be
  // one more than adc_sucessor_lane
  if (lane_segments.size() > static_cast<size_t>(adc_sucessor_lane_index + 1)) {
    ++adc_sucessor_lane_index;
  }
  // get the id of adc_sucessor_lane
  const std::string adc_sucessor_lane_id =
      lane_segments[adc_sucessor_lane_index].lane->lane().id().id();
  // if there is a left neighbor lane, then check if its successor lane is
  // the same as adc_sucessor_lane
  if (nullptr != ptr_left_neighbor_lane &&
      1 == ptr_left_neighbor_lane->lane().successor_id_size()) {
    const std::string left_neighbor_successor_lane_id =
        ptr_left_neighbor_lane->lane().successor_id(0).id();
    ADEBUG << "left_neighbor_successor_lane_id: "
           << left_neighbor_successor_lane_id;
    ADEBUG << "adc_sucessor_lane_id: " << adc_sucessor_lane_id;
    return left_neighbor_successor_lane_id == adc_sucessor_lane_id;
  }
  return false;
}

double SpeedBoundsDecider::FindStopS(
    const std::vector<common::PathPoint> &path_points,
    const hdmap::LaneInfoConstPtr &ptr_left_neighbor_lane, const double lat_buf,
    double *extend_l) {
  if (nullptr == ptr_left_neighbor_lane) {
    // If there is no left neighbor lane, we can't stop.
    return std::numeric_limits<double>::max();
  }
  static double driving_dis_in_adc_stop = 0.0;
  common::math::Vec2d left_front;
  double last_distance_to_bound = std::numeric_limits<double>::max();
  size_t index = 0UL;
  for (const auto &path_point : path_points) {
    GetAdcCornerPoint(path_point, &left_front);
    double neighbor_s = 0.0, neighbor_l = 0.0;
    if (!ptr_left_neighbor_lane->GetProjection(left_front, &neighbor_s,
                                               &neighbor_l)) {
      AWARN << "can't get neighbor lane SL info.";
      continue;
    }
    double right_width = 0.0;
    ptr_left_neighbor_lane->GetWidth(neighbor_s, nullptr, &right_width);
    double distance_to_bound = -neighbor_l - right_width;
    if (distance_to_bound < lat_buf) {
      *extend_l = lat_buf - distance_to_bound;
      // If the distance to bound is less than lat_buf, we can't stop.
      const auto &adc_state = reference_line_info_->vehicle_state();
      common::SLPoint adc_sl_point;
      reference_line_info_->reference_line().XYToSL(
          common::math::Vec2d(adc_state.x(), adc_state.y()), &adc_sl_point);
      common::SLPoint sl_point;
      reference_line_info_->reference_line().XYToSL(
          common::math::Vec2d(path_point.x(), path_point.y()), &sl_point);
      double ref_stop_s = 0.0;
      double adc_stop_s = 0.0;
      if (index >= 1UL) {
        // If the index is greater than 1, we have the last path point.
        common::PathPoint last_path_point = path_points[index - 1];
        common::SLPoint last_sl_point;
        reference_line_info_->reference_line().XYToSL(
            common::math::Vec2d(last_path_point.x(), last_path_point.y()),
            &last_sl_point);
        ref_stop_s = last_sl_point.s() +
                     (sl_point.s() - last_sl_point.s()) *
                         (lat_buf - last_distance_to_bound) /
                         (distance_to_bound - last_distance_to_bound);
        ADEBUG << "ref_stop_s: " << ref_stop_s;
        adc_stop_s = ref_stop_s - adc_sl_point.s();
        driving_dis_in_adc_stop = adc_stop_s + injector_->GetDrivingDistance();
      } else {
        // If the index is less than 1, we don't have the last path point.
        adc_stop_s = std::max(
            0.0, driving_dis_in_adc_stop - injector_->GetDrivingDistance());
      }
      ADEBUG << "adc_stop_s: " << adc_stop_s;
      return adc_stop_s;
    }
    ++index;
    last_distance_to_bound = distance_to_bound;
  }
  // If we can't find the stop point, we return max double value.
  return std::numeric_limits<double>::max();
}

bool SpeedBoundsDecider::IsAbleToStop(const double stop_s, double *extend_s) {
  // Construct the hysteresis interval for the stop distance.
  static HysteresisInterval adc_stop_dis_interval(0.0, 1.0, 5UL);
  // Set the anchor and limits for the stop distance.
  adc_stop_dis_interval.SetAnchorLimits(
      stop_s, speed_bounds_config_.lower_buffer_of_stop_dis(),
      speed_bounds_config_.upper_buffer_of_stop_dis());
  // Calculate the real stop distance.
  const double adc_v = reference_line_info_->vehicle_state().linear_velocity();
  const double real_stop_s =
      adc_v * adc_v / (2.0 * -speed_bounds_config_.max_aver_stop_accel());
  // Calculate the hysteresis value of the real stop distance.
  const double hy_real_stop_s =
      adc_stop_dis_interval.HyValue("stop_distance", real_stop_s);
  *extend_s = real_stop_s - stop_s;
  // Logging the values.
  ADEBUG << "real_stop_s: " << real_stop_s;
  ADEBUG << "hy_real_stop_s: " << hy_real_stop_s;
  ADEBUG << "stop_s: " << stop_s;
  // Return true if the hysteresis value of the real stop distance is less
  // than or equal to the stop distance.
  return hy_real_stop_s <= stop_s;
}

bool SpeedBoundsDecider::IsNearMergeAreaWithLaneAttributes(
    const double s, hdmap::LaneInfoConstPtr *ptr_left_neighbor_lane) const {
  // get the left neighbor lane id and its width.
  const auto locate_lane = reference_line_info_->LocateLaneInfo(s);
  if (0) {
    // get next lane of merge lane
    const auto next_lane_id = locate_lane->lane().successor_id(0);
    auto next_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(next_lane_id);
    using PreLaneInfo = std::pair<common::SLPoint, hdmap::LaneInfoConstPtr>;
    struct comp_pre_lane {
      bool operator()(const PreLaneInfo &lane_a, const PreLaneInfo &lane_b) {
        double a_l = lane_a.first.l();
        double b_l = lane_b.first.l();
        if (a_l < 0.0) {
          a_l = kLatBaseDisForNegative - a_l;
        }
        if (b_l < 0.0) {
          b_l = kLatBaseDisForNegative - b_l;
        }
        return a_l < b_l;
      }
    };

    std::priority_queue<PreLaneInfo, std::vector<PreLaneInfo>, comp_pre_lane>
        pre_lanes_info;
    for (const auto pre_id : next_lane->lane().predecessor_id()) {
      if (pre_id.id() == locate_lane->id().id()) {
        continue;
      }
      auto pre_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(pre_id);
      common::SLPoint front_point_sl;
      reference_line_info_->reference_line().XYToSL(pre_lane->points().front(),
                                                    &front_point_sl);
      pre_lanes_info.emplace(front_point_sl, pre_lane);
    }
    *ptr_left_neighbor_lane = pre_lanes_info.top().second;
    return true;
  }
  return false;
}

bool SpeedBoundsDecider::IsNearMergeAreaWithNeighborLane(
    const double s, hdmap::LaneInfoConstPtr *ptr_left_neighbor_lane) const {
  static double driving_dis_off_merge_area = 0.0;
  static hdmap::LaneInfoConstPtr ptr_left_lane_in_merge_area = nullptr;
  // get the left neighbor lane id and its width.
  hdmap::Id neighbor_lane_id;
  double left_neighbor_lane_width = 0.0;
  if (reference_line_info_->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward,
          s + speed_bounds_config_.presight_distance_to_merge_area(),
          &neighbor_lane_id, &left_neighbor_lane_width)) {
    // get the left neighbor lane.
    *ptr_left_neighbor_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
    ADEBUG << "Has left forward neighbor lane.";
  } else {
    ADEBUG << "There is no left neighbor lane.";
  }

  // check if the presight point is in merge area.
  bool is_near_merge_area = false;
  if (IsMergeArea(s + speed_bounds_config_.presight_distance_to_merge_area(),
                  *ptr_left_neighbor_lane)) {
    // if the presight point is in merge area.
    driving_dis_off_merge_area =
        injector_->GetDrivingDistance() +
        speed_bounds_config_.presight_distance_to_merge_area();
    is_near_merge_area = true;
    // store the left lane in merge area.
    ptr_left_lane_in_merge_area = *ptr_left_neighbor_lane;
  } else if (driving_dis_off_merge_area > injector_->GetDrivingDistance()) {
    // if the presight point is not in merge area, but ADC is not beyond the
    // last merge area point.
    *ptr_left_neighbor_lane = ptr_left_lane_in_merge_area;
    is_near_merge_area = true;
  }
  return is_near_merge_area;
}

void SpeedBoundsDecider::RadicalChangeLaneInMergeArea(
    PathDecision *const path_decision) {
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  hdmap::LaneInfoConstPtr ptr_left_neighbor_lane = nullptr;
  // Check if ADC is near merge area.
  is_near_merge_area_ = false;
  if (IsNearMergeAreaWithNeighborLane(adc_sl.end_s(),
                                      &ptr_left_neighbor_lane) ||
      IsNearMergeAreaWithLaneAttributes(adc_sl.end_s(),
                                        &ptr_left_neighbor_lane)) {
    AINFO << "IsNearMergeArea";
    is_near_merge_area_ = true;
    const PathData &path_data = reference_line_info_->path_data();
    double extend_l = std::numeric_limits<double>::lowest();
    const double stop_s = FindStopS(
        path_data.discretized_path(), ptr_left_neighbor_lane,
        speed_bounds_config_.lat_dis_off_lane_bound_to_stop(), &extend_l);
    // Check if ADC can stop before stop s.
    double extend_s = std::numeric_limits<double>::lowest();
    if (!IsAbleToStop(stop_s, &extend_s)) {
      ADEBUG << "can't stop before stop s.";
      // If ADC can't stop, ignore all the obstacles behind ADC rear at least
      // 5m.
      double ignore_buffer_behind_adc = InterpolationLookUp(
          extend_s,
          speed_bounds_config_.extend_s_for_max_ignore_distance() /*0.0*/,
          speed_bounds_config_.extend_s_for_min_ignore_distance() /*2.0*/,
          speed_bounds_config_.max_ignore_distance_behind_adc() /*5.0*/,
          speed_bounds_config_.min_ignore_distance_behind_adc() /*0.1*/);
      if (extend_l > 0.0) {
        // has exceed the left bound at first path point, so needn't stop.
        ignore_buffer_behind_adc =
            speed_bounds_config_.min_ignore_distance_behind_adc();
      }
      for (auto *obstacle : path_decision->obstacles().Items()) {
        const auto &obs_sl = obstacle->PerceptionSLBoundary();
        auto *mutable_obstacle = path_decision->Find(obstacle->Id());
        if (nullptr != mutable_obstacle &&
            adc_sl.start_s() - obs_sl.end_s() > ignore_buffer_behind_adc) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("merge area", ignore);
          reference_line_info_->AddIgnoreObstacle(mutable_obstacle->Id());
          AINFO << "merge area Ignore, obs ID: " << obstacle->Id();
        }
      }
    }
  }
}

bool SpeedBoundsDecider::FindStopSForLaneBorrow(double *stop_s,
                                                double *extend_l) {
  const auto &discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto &frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  if (discretized_path.size() < 2UL) {
    AERROR << "path points size is less than 2.";
    return false;
  }

  const auto &adc_state = reference_line_info_->vehicle_state();
  common::SLPoint adc_sl_point;
  reference_line_info_->reference_line().XYToSL(
      common::math::Vec2d(adc_state.x(), adc_state.y()), &adc_sl_point);
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  double lane_left_width = 0, lane_right_width = 0;
  for (size_t i = 0; i < discretized_path.size() - 1UL; ++i) {
    const double path_s = discretized_path[i].s();
    const double ref_s = frenet_path[i].s();
    if (path_s > speed_bounds_config_.presight_distance_to_changing_lane()) {
      break;
    }
    const auto &reference_point =
        reference_line_info_->reference_line().GetReferencePoint(ref_s);
    // get differ heading between path point and reference point.
    const double diff_heading = common::math::NormalizeAngle(
        discretized_path[i].theta() - reference_point.heading());
    const double front_left_corner_point_l =
        frenet_path[i].l() +
        vehicle_param.front_edge_to_center() * std::sin(diff_heading) +
        vehicle_param.left_edge_to_center() * std::cos(diff_heading);
    reference_line_info_->reference_line().GetLaneWidth(ref_s, &lane_left_width,
                                                        &lane_right_width);
    ADEBUG << "diff_heading: " << diff_heading * 180.0 / M_PI
           << " front_left_corner_point_l: " << front_left_corner_point_l
           << " lane_left_width: " << lane_left_width;
    // get extend l value.
    double extend_l_value =
        front_left_corner_point_l -
        (lane_left_width -
         speed_bounds_config_.lat_dis_off_lane_bound_to_stop());
    ADEBUG << "extend_l_value: " << extend_l_value;
    if (extend_l_value > 0) {
      *stop_s = ref_s - adc_sl_point.s();
      if (0UL == i) {
        // TODO(zhonghao): consider the situation ADC center point is behind the
        // first path point.
        *extend_l = extend_l_value;
      }
      ADEBUG << "stop_s: " << *stop_s;
      return true;
    }
  }
  return false;
}

bool SpeedBoundsDecider::IsChangingToLeftLaneByDiffHeading() {
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const double adc_s = 0.5 * (adc_sl.start_s() + adc_sl.end_s());
  const double adc_heading = injector_->vehicle_state()->heading();
  // get reference point at adc position.
  const auto &reference_point =
      reference_line_info_->reference_line().GetReferencePoint(adc_s);
  // get differ heading between adc and reference point.
  const double diff_heading =
      common::math::NormalizeAngle(adc_heading - reference_point.heading()) *
      180.0 / M_PI;
  // get lane width.
  double lane_left_width = 0.0, lane_right_width = 0.0;
  reference_line_info_->reference_line().GetLaneWidth(adc_s, &lane_left_width,
                                                      &lane_right_width);
  ADEBUG << "diff_heading: " << diff_heading
         << " lane_left_width: " << lane_left_width;
  ADEBUG << "adc_l:(" << adc_sl.start_l() << ", " << adc_sl.start_l() << ")";
  // check if diff_heading is big enough and adc is rading on left lane boundary
  // line.
  if (diff_heading >
          speed_bounds_config_.min_diff_heading_for_changing_lane() &&
      lane_left_width - kLatBuffer > adc_sl.start_l() &&
      lane_left_width < adc_sl.end_l()) {
    ADEBUG << "IsChangingToLeftLaneByDiffHeading";
    return true;
  }
  return false;
}

void SpeedBoundsDecider::RadicalChangeLaneInLaneBorrow(
    PathDecision *const path_decision) {
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  double stop_s = std::numeric_limits<double>::max();
  double extend_l = std::numeric_limits<double>::lowest();
  if (FindStopSForLaneBorrow(&stop_s, &extend_l)) {
    AINFO << "IsNearBorrowLane";
    double extend_s = std::numeric_limits<double>::lowest();
    // Check if ADC can stop before stop s.
    const bool is_able_to_stop = IsAbleToStop(stop_s, &extend_s);
    // extend_l is positive means: adc has exceed the left bound at first path
    // point.
    ADEBUG << "extend_l: " << extend_l;
    ADEBUG << "is_able_to_stop: " << is_able_to_stop;
    if ((extend_l <= 0.0 && !is_able_to_stop) ||
        (extend_l > 0.0 && IsChangingToLeftLaneByDiffHeading())) {
      ADEBUG << "can't stop before stop s.";
      // If ADC can't stop, ignore all the obstacles behind ADC rear at least
      // 5m.
      double ignore_buffer_behind_adc = InterpolationLookUp(
          extend_s,
          speed_bounds_config_.extend_s_for_max_ignore_distance() /*0.0*/,
          speed_bounds_config_.extend_s_for_min_ignore_distance() /*2.0*/,
          speed_bounds_config_.max_ignore_distance_behind_adc() /*5.0*/,
          speed_bounds_config_.min_ignore_distance_behind_adc() /*0.1*/);
      if (extend_l > 0.0) {
        // has exceed the left bound at first path point, so needn't stop.
        ignore_buffer_behind_adc =
            speed_bounds_config_.min_ignore_distance_behind_adc();
      }
      ADEBUG << "ignore_buffer_behind_adc: " << ignore_buffer_behind_adc;
      // Iterate all the obstacles and check if they are in the ignore zone
      // behind ADC.
      for (auto *obstacle : path_decision->obstacles().Items()) {
        const auto &obs_sl = obstacle->PerceptionSLBoundary();
        auto *mutable_obstacle = path_decision->Find(obstacle->Id());
        if (nullptr != mutable_obstacle &&
            adc_sl.start_s() - obs_sl.end_s() > ignore_buffer_behind_adc) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("borrow lane", ignore);
          reference_line_info_->AddIgnoreObstacle(mutable_obstacle->Id());
          AINFO << "borrow lane Ignore, obs ID: " << obstacle->Id();
        }
      }
    }
  }
}

size_t SpeedBoundsDecider::ClearObsoleteElements(
    DangerousObsContainer *container, size_t capacity) {
  uint32_t obsolete_seq_num = kLowNumObsoleteSeqNum;
  if (container->size() < capacity / 2) {
    ADEBUG << "the container capacity is enough, so deleting nothing.";
    return 0UL;
  } else if (container->size() < 3 * capacity / 4) {
    obsolete_seq_num = kLowNumObsoleteSeqNum;
  } else {
    obsolete_seq_num = kHighNumObsoleteSeqNum;
  }
  size_t erase_num = 0;

  std::vector<std::string> to_remove;
  for (const auto &item : *container) {
    if (injector_->GetSequenceNum() - item.second.last_seq_num >
        obsolete_seq_num) {
      to_remove.emplace_back(item.first);
    }
  }
  for (const auto &key : to_remove) {
    erase_num += container->erase(key);
  }
  return erase_num;
}

void SpeedBoundsDecider::MultiDecisionsBeforePrioriSpeedBounds(
    PathDecision *const path_decision) {
  is_adc_start_up_ = GetStartUpStatus();
  if (FLAGS_enable_make_dangerous_start_up) {
    MakeDangerousStartUpDecision(path_decision);
    ClearObsoleteElements(&obs_container_, 50UL);
  }
  if (FLAGS_enable_vehicle_start_up) {
    MakeVehicleStartUpDecision(path_decision);
    ClearObsoleteElements(&vehicle_container_, 50UL);
  }

  if (FLAGS_enable_radical_change_lane_in_merge_area) {
    RadicalChangeLaneInMergeArea(path_decision);
    RadicalChangeLaneInLaneBorrow(path_decision);
  }

  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  if (FLAGS_enable_speed_limit_for_approaching_obs &&
      adc_speed >
          speed_bounds_config_.min_adc_speed_for_approach_obs_speed_limit()) {
    SetSpeedLimitsForKeepApproachingObs(path_decision);
  } else {
    speed_limit_obs_container_.clear();
  }

  // obstacle sidepass decision
  if (FLAGS_enable_obstacle_sidepass_decision) {
    MakeObstacleSidepassDecision(path_decision);
  }

  // make slowly down decision for approaching obstacle
  if (FLAGS_enable_kinematic_speed_slowly_down) {
    KinematicSlowlyDownForApproachingObs(path_decision);
  }
}

void SpeedBoundsDecider::GetStaticObs(
    double diff_lateral, const SLBoundary &adc_sl_boundary,
    const SLBoundary &obs_sl, bool is_unknown, const Obstacle &obstacle,
    std::array<std::vector<std::string>, 20> *consider_obs) {
  std::vector<std::string> *one_level_unknown_obs = &((*consider_obs)[0]),
                           *one_level_typed_obs = &((*consider_obs)[1]),
                           *two_level_unknown_obs = &((*consider_obs)[2]),
                           *two_level_typed_obs = &((*consider_obs)[3]),
                           *three_level_unknown_obs = &((*consider_obs)[4]),
                           *three_level_typed_obs = &((*consider_obs)[5]),
                           *four_level_unknown_obs = &((*consider_obs)[6]),
                           *four_level_typed_obs = &((*consider_obs)[7]),
                           *five_level_unknown_obs = &((*consider_obs)[8]),
                           *five_level_typed_obs = &((*consider_obs)[9]);

  // AINFO << "diff_lateral = " << diff_lateral
  //       << "   obstacle.Id() = " << obstacle.Id();
  if (diff_lateral < speed_bounds_config_.svm_one_level_lateral_distance() &&
      obs_sl.start_s() - adc_sl_boundary.start_s() <
          speed_bounds_config_.svm_one_level_lon_distance()) {
    if (is_unknown) {
      one_level_unknown_obs->push_back(obstacle.Id());
    } else {
      one_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (diff_lateral <
                 speed_bounds_config_.svm_two_level_lateral_distance() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_.svm_two_level_lon_distance()) {
    if (is_unknown) {
      two_level_unknown_obs->push_back(obstacle.Id());
    } else {
      two_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (diff_lateral <
                 speed_bounds_config_.svm_three_level_lateral_distance() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_.svm_three_level_lon_distance()) {
    if (is_unknown) {
      three_level_unknown_obs->push_back(obstacle.Id());
    } else {
      three_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (diff_lateral <
                 speed_bounds_config_.svm_four_level_lateral_distance() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_.svm_four_level_lon_distance()) {
    if (is_unknown) {
      four_level_unknown_obs->push_back(obstacle.Id());
    } else {
      four_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (diff_lateral <
                 speed_bounds_config_.svm_five_level_lateral_distance() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_.svm_five_level_lon_distance()) {
    if (is_unknown) {
      five_level_unknown_obs->push_back(obstacle.Id());
    } else {
      five_level_typed_obs->push_back(obstacle.Id());
    }
  }
}

double SpeedBoundsDecider::GetReachTime(double diff_lateral,
                                        const SLBoundary &adc_sl_boundary,
                                        const SLBoundary &obs_sl,
                                        const Obstacle &obstacle) {
  double reach_time = std::numeric_limits<double>::max();
  if (diff_lateral < speed_bounds_config_.svm_one_level_lateral_distance()) {
    reach_time = 0.0;
  } else {
    double lateral_speed = obstacle.LateralSpeed();
    bool is_left_cross_obs =
        obs_sl.end_l() < adc_sl_boundary.start_l() &&
        lateral_speed > speed_bounds_config_.min_cross_speed();
    bool is_right_cross_obs =
        obs_sl.start_l() > adc_sl_boundary.end_l() &&
        lateral_speed < -speed_bounds_config_.min_cross_speed();
    bool is_face_to_path_obs = (is_left_cross_obs || is_right_cross_obs);

    // lateral no overlap ,use reach time
    // AINFO << "is_face_to_path_obs = " << is_face_to_path_obs;
    if (is_face_to_path_obs) {
      AINFO << "lateral speed obs = " << obstacle.Id();
      reach_time =
          std::max(std::max(adc_sl_boundary.start_l() - obs_sl.end_l(),
                            obs_sl.start_l() - adc_sl_boundary.end_l()) -
                       kLateralConsiderBuffer,
                   0.0) /
          std::fabs(lateral_speed);
      AINFO << "reach path time = " << reach_time;
    }
  }
  return reach_time;
}

void SpeedBoundsDecider::GetDynamicObs(
    double diff_lateral, const SLBoundary &adc_sl_boundary,
    const SLBoundary &obs_sl, bool is_unknown, const Obstacle &obstacle,
    std::array<std::vector<std::string>, 20> *consider_obs) {
  std::vector<std::string>
      *dynamic_one_level_unknown_obs = &((*consider_obs)[10]),
      *dynamic_one_level_typed_obs = &((*consider_obs)[11]),
      *dynamic_two_level_unknown_obs = &((*consider_obs)[12]),
      *dynamic_two_level_typed_obs = &((*consider_obs)[13]),
      *dynamic_three_level_unknown_obs = &((*consider_obs)[14]),
      *dynamic_three_level_typed_obs = &((*consider_obs)[15]),
      *dynamic_four_level_unknown_obs = &((*consider_obs)[16]),
      *dynamic_four_level_typed_obs = &((*consider_obs)[17]),
      *dynamic_five_level_unknown_obs = &((*consider_obs)[18]),
      *dynamic_five_level_typed_obs = &((*consider_obs)[19]);

  // lateral has overlap
  double reach_time =
      GetReachTime(diff_lateral, adc_sl_boundary, obs_sl, obstacle);

  // dynamic obs consider reach_time and lon distance.
  if (reach_time < speed_bounds_config_.svm_one_level_reach_time() &&
      obs_sl.start_s() - adc_sl_boundary.start_s() <
          speed_bounds_config_.svm_one_level_lon_distance_for_dynamic_obs()) {
    if (is_unknown) {
      dynamic_one_level_unknown_obs->push_back(obstacle.Id());
    } else {
      dynamic_one_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (reach_time < speed_bounds_config_.svm_two_level_reach_time() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_
                     .svm_two_level_lon_distance_for_dynamic_obs()) {
    if (is_unknown) {
      dynamic_two_level_unknown_obs->push_back(obstacle.Id());
    } else {
      dynamic_two_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (reach_time < speed_bounds_config_.svm_three_level_reach_time() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_
                     .svm_three_level_lon_distance_for_dynamic_obs()) {
    if (is_unknown) {
      dynamic_three_level_unknown_obs->push_back(obstacle.Id());
    } else {
      dynamic_three_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (reach_time < speed_bounds_config_.svm_four_level_reach_time() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_
                     .svm_four_level_lon_distance_for_dynamic_obs()) {
    if (is_unknown) {
      dynamic_four_level_unknown_obs->push_back(obstacle.Id());
    } else {
      dynamic_four_level_typed_obs->push_back(obstacle.Id());
    }
  } else if (reach_time < speed_bounds_config_.svm_five_level_reach_time() &&
             obs_sl.start_s() - adc_sl_boundary.start_s() <
                 speed_bounds_config_
                     .svm_five_level_lon_distance_for_dynamic_obs()) {
    if (is_unknown) {
      dynamic_five_level_unknown_obs->push_back(obstacle.Id());
    } else {
      dynamic_five_level_typed_obs->push_back(obstacle.Id());
    }
  }
}

void SpeedBoundsDecider::AddFeature(
    double time_stamp,
    const std::array<std::vector<std::string>, 20> &consider_obs) {
  const std::vector<std::string>
      &one_level_unknown_obs = consider_obs[0],
      &one_level_typed_obs = consider_obs[1],
      &two_level_unknown_obs = consider_obs[2],
      &two_level_typed_obs = consider_obs[3],
      &three_level_unknown_obs = consider_obs[4],
      &three_level_typed_obs = consider_obs[5],
      &four_level_unknown_obs = consider_obs[6],
      &four_level_typed_obs = consider_obs[7],
      &five_level_unknown_obs = consider_obs[8],
      &five_level_typed_obs = consider_obs[9],
      &dynamic_one_level_unknown_obs = consider_obs[10],
      &dynamic_one_level_typed_obs = consider_obs[11],
      &dynamic_two_level_unknown_obs = consider_obs[12],
      &dynamic_two_level_typed_obs = consider_obs[13],
      &dynamic_three_level_unknown_obs = consider_obs[14],
      &dynamic_three_level_typed_obs = consider_obs[15],
      &dynamic_four_level_unknown_obs = consider_obs[16],
      &dynamic_four_level_typed_obs = consider_obs[17],
      &dynamic_five_level_unknown_obs = consider_obs[18],
      &dynamic_five_level_typed_obs = consider_obs[19];
  // if (!one_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < one_level_unknown_obs.size(); ++i) {
  //     AINFO << "one_level_unknown_obs "
  //           << "[" << i << "]" << one_level_unknown_obs[i];
  //   }
  // }
  // if (!one_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < one_level_typed_obs.size(); ++i) {
  //     AINFO << "one_level_typed_obs "
  //           << "[" << i << "]" << one_level_typed_obs[i];
  //   }
  // }
  // if (!two_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < two_level_unknown_obs.size(); ++i) {
  //     AINFO << "two_level_unknown_obs "
  //           << "[" << i << "]" << two_level_unknown_obs[i];
  //   }
  // }
  // if (!two_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < two_level_typed_obs.size(); ++i) {
  //     AINFO << "two_level_typed_obs "
  //           << "[" << i << "]" << two_level_typed_obs[i];
  //   }
  // }
  // if (!three_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < three_level_unknown_obs.size(); ++i) {
  //     AINFO << "three_level_unknown_obs "
  //           << "[" << i << "]" << three_level_unknown_obs[i];
  //   }
  // }
  // if (!three_level_typed_obs.empty()) {
  //   AINFO << "three_level_typed_obs.size = " << three_level_typed_obs.size();
  //   for (size_t i = 0; i < three_level_typed_obs.size(); ++i) {
  //     AINFO << "three_level_typed_obs "
  //           << "[" << i << "]" << three_level_typed_obs[i];
  //   }
  // }
  // if (!four_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < four_level_unknown_obs.size(); ++i) {
  //     AINFO << "four_level_unknown_obs "
  //           << "[" << i << "]" << four_level_unknown_obs[i];
  //   }
  // }
  // if (!four_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < four_level_typed_obs.size(); ++i) {
  //     AINFO << "four_level_typed_obs "
  //           << "[" << i << "]" << four_level_typed_obs[i];
  //   }
  // }
  // if (!five_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < five_level_unknown_obs.size(); ++i) {
  //     AINFO << "five_level_unknown_obs "
  //           << "[" << i << "]" << five_level_unknown_obs[i];
  //   }
  // }
  // if (!five_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < five_level_typed_obs.size(); ++i) {
  //     AINFO << "five_level_typed_obs "
  //           << "[" << i << "]" << five_level_typed_obs[i];
  //   }
  // }

  // if (!dynamic_one_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_one_level_unknown_obs.size(); ++i) {
  //     AINFO << "dynamic_one_level_unknown_obs "
  //           << "[" << i << "]" << dynamic_one_level_unknown_obs[i];
  //   }
  // }
  // if (!dynamic_one_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_one_level_typed_obs.size(); ++i) {
  //     AINFO << "dynamic_one_level_typed_obs "
  //           << "[" << i << "]" << dynamic_one_level_typed_obs[i];
  //   }
  // }
  // if (!dynamic_two_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_two_level_unknown_obs.size(); ++i) {
  //     AINFO << "dynamic_two_level_unknown_obs "
  //           << "[" << i << "]" << dynamic_two_level_unknown_obs[i];
  //   }
  // }
  // if (!dynamic_two_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_two_level_typed_obs.size(); ++i) {
  //     AINFO << "dynamic_two_level_typed_obs "
  //           << "[" << i << "]" << dynamic_two_level_typed_obs[i];
  //   }
  // }
  // if (!dynamic_three_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_three_level_unknown_obs.size(); ++i) {
  //     AINFO << "dynamic_three_level_unknown_obs "
  //           << "[" << i << "]" << dynamic_three_level_unknown_obs[i];
  //   }
  // }
  // if (!dynamic_three_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_three_level_typed_obs.size(); ++i) {
  //     AINFO << "dynamic_three_level_typed_obs "
  //           << "[" << i << "]" << dynamic_three_level_typed_obs[i];
  //   }
  // }
  // if (!dynamic_four_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_four_level_unknown_obs.size(); ++i) {
  //     AINFO << "dynamic_four_level_unknown_obs "
  //           << "[" << i << "]" << dynamic_four_level_unknown_obs[i];
  //   }
  // }
  // if (!dynamic_four_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_four_level_typed_obs.size(); ++i) {
  //     AINFO << "dynamic_four_level_typed_obs "
  //           << "[" << i << "]" << dynamic_four_level_typed_obs[i];
  //   }
  // }
  // if (!dynamic_five_level_unknown_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_five_level_unknown_obs.size(); ++i) {
  //     AINFO << "dynamic_five_level_unknown_obs "
  //           << "[" << i << "]" << dynamic_five_level_unknown_obs[i];
  //   }
  // }
  // if (!dynamic_five_level_typed_obs.empty()) {
  //   for (size_t i = 0; i < dynamic_five_level_typed_obs.size(); ++i) {
  //     AINFO << "dynamic_five_level_typed_obs "
  //           << "[" << i << "]" << dynamic_five_level_typed_obs[i];
  //   }
  // }

  svm_feature_data_.clear_featrue();
  auto *feature_data = svm_feature_data_.add_featrue();
  auto *header = feature_data->mutable_header();
  header->set_timestamp_sec(time_stamp);
  feature_data->set_one_level_unknown_obs(one_level_unknown_obs.size());
  feature_data->set_one_level_typed_obs(one_level_typed_obs.size());
  feature_data->set_two_level_unknown_obs(two_level_unknown_obs.size());
  feature_data->set_two_level_typed_obs(two_level_typed_obs.size());
  feature_data->set_three_level_unknown_obs(three_level_unknown_obs.size());
  feature_data->set_three_level_typed_obs(three_level_typed_obs.size());
  feature_data->set_four_level_unknown_obs(four_level_unknown_obs.size());
  feature_data->set_four_level_typed_obs(four_level_typed_obs.size());
  feature_data->set_five_level_unknown_obs(five_level_unknown_obs.size());
  feature_data->set_five_level_typed_obs(five_level_typed_obs.size());
  feature_data->set_dynamic_one_level_unknown_obs(
      dynamic_one_level_unknown_obs.size());
  feature_data->set_dynamic_two_level_unknown_obs(
      dynamic_two_level_unknown_obs.size());
  feature_data->set_dynamic_three_level_unknown_obs(
      dynamic_three_level_unknown_obs.size());
  feature_data->set_dynamic_four_level_unknown_obs(
      dynamic_four_level_unknown_obs.size());
  feature_data->set_dynamic_five_level_unknown_obs(
      dynamic_five_level_unknown_obs.size());
  feature_data->set_dynamic_one_level_typed_obs(
      dynamic_one_level_typed_obs.size());
  feature_data->set_dynamic_two_level_typed_obs(
      dynamic_two_level_typed_obs.size());
  feature_data->set_dynamic_three_level_typed_obs(
      dynamic_three_level_typed_obs.size());
  feature_data->set_dynamic_four_level_typed_obs(
      dynamic_four_level_typed_obs.size());
  feature_data->set_dynamic_five_level_typed_obs(
      dynamic_five_level_typed_obs.size());

  feature_data->set_label(0);
  // AINFO << "ADD FEATURE";
}

void SpeedBoundsDecider::GetTrainingData() {
  if (FLAGS_enable_txt_to_bin_for_svm) {
    int file_count_tobin = 0;
    for (const auto &entry :
         boost::filesystem::directory_iterator(folder_path_result)) {
      if (boost::filesystem::is_regular_file(entry.path())) {
        file_count_tobin++;
      }
    }
    AINFO << "file_count_tobin = " << file_count_tobin;
    if (file_count_tobin == 0) {
      planning::SvmFeatureData svm_feature_data;
      bool load_success = cyber::common::GetProtoFromASCIIFile(
          folder_path_feature_and_label, &svm_feature_data);
      AINFO << "load_success = " << load_success;

      const std::string dest_file_1 =
          absl::StrCat(folder_path_result, "/", "result", ".bin");
      cyber::common::SetProtoToBinaryFile(svm_feature_data, dest_file_1);
    }
  }
}
void SpeedBoundsDecider::GetSVMPredictionData(Frame *const frame) {
  if (FLAGS_enable_use_svm_model) {
    // AINFO << "start get svm result";
    int feature_data[1][kFeatureSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!svm_feature_data_.featrue().empty()) {
      feature_data[0][0] = svm_feature_data_.featrue(0).one_level_unknown_obs();
      feature_data[0][1] = svm_feature_data_.featrue(0).one_level_typed_obs();
      feature_data[0][2] = svm_feature_data_.featrue(0).two_level_unknown_obs();
      feature_data[0][3] = svm_feature_data_.featrue(0).two_level_typed_obs();
      feature_data[0][4] =
          svm_feature_data_.featrue(0).three_level_unknown_obs();
      feature_data[0][5] = svm_feature_data_.featrue(0).three_level_typed_obs();
      feature_data[0][6] =
          svm_feature_data_.featrue(0).four_level_unknown_obs();
      feature_data[0][7] = svm_feature_data_.featrue(0).four_level_typed_obs();
      feature_data[0][8] =
          svm_feature_data_.featrue(0).five_level_unknown_obs();
      feature_data[0][9] = svm_feature_data_.featrue(0).five_level_typed_obs();
      feature_data[0][10] =
          svm_feature_data_.featrue(0).dynamic_one_level_unknown_obs();
      feature_data[0][11] =
          svm_feature_data_.featrue(0).dynamic_one_level_typed_obs();
      feature_data[0][12] =
          svm_feature_data_.featrue(0).dynamic_two_level_unknown_obs();
      feature_data[0][13] =
          svm_feature_data_.featrue(0).dynamic_two_level_typed_obs();
      feature_data[0][14] =
          svm_feature_data_.featrue(0).dynamic_three_level_unknown_obs();
      feature_data[0][15] =
          svm_feature_data_.featrue(0).dynamic_three_level_typed_obs();
      feature_data[0][16] =
          svm_feature_data_.featrue(0).dynamic_four_level_unknown_obs();
      feature_data[0][17] =
          svm_feature_data_.featrue(0).dynamic_four_level_typed_obs();
      feature_data[0][18] =
          svm_feature_data_.featrue(0).dynamic_five_level_unknown_obs();
      feature_data[0][19] =
          svm_feature_data_.featrue(0).dynamic_five_level_typed_obs();
    } else {
      AINFO << "use zero start";
    }
    cv::Mat feature_data_mat(1, kFeatureSize, CV_32SC1, feature_data);
    // AINFO << "type = " << feature_data_mat.type() << std::endl;
    feature_data_mat.convertTo(feature_data_mat, CV_32FC1);
    float response = model_->predict(feature_data_mat);
    // AINFO << "response = " << response;
    int acc_level = static_cast<int>(response);
    century::planning::Frame::AccLevel result_level =
        century::planning::Frame::AccLevel::ACC_NORMAL;
    if (acc_level < 0 ||
        acc_level > static_cast<int>(
                        century::planning::Frame::AccLevel::ACC_FOUR_LEVEL)) {
      result_level = static_cast<century::planning::Frame::AccLevel>(0);
    } else {
      result_level = static_cast<century::planning::Frame::AccLevel>(acc_level);
    }
    frame->acc_level_ = result_level;
  }
}

void SpeedBoundsDecider::GetLastTimeStamp(double *last_time_stamp) {
  if (injector_->frame_history() != nullptr) {
    if (injector_->frame_history()->Latest() != nullptr) {
      *last_time_stamp = injector_->frame_history()->Latest()->GetRecordTime();
    } else {
      AINFO << "NO get last time stamp1";
    }
  }
}

void SpeedBoundsDecider::RecordToLearningData(Frame *const frame) {
  if (FLAGS_svm_switch &&
      TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type()) {
    double time_stamp = frame->GetRecordTime();
    double last_time_stamp = time_stamp;
    GetLastTimeStamp(&last_time_stamp);

    bool open_new_file = false;
    if (!svm_feature_data_.featrue().empty() &&
        FLAGS_enable_record_to_learning_data_for_svm) {
      double first_feature_time_stamp =
          svm_feature_data_.featrue().begin()->header().timestamp_sec();
      // AINFO << "first_feature_time_stamp = " << std::setprecision(16)
      //       << first_feature_time_stamp;
      if (std::fabs(time_stamp - first_feature_time_stamp) > kMaxDeltaTime) {
        // AINFO << "exceed 10s,open now file to add feature.";
        open_new_file = true;
      } else if (std::fabs(time_stamp - last_time_stamp) < kMinDeltaTime) {
        // AINFO << "get obs time diff too small,no add feature.";
        return;
      }
    } else {
      // first frame
      open_new_file = true;
    }

    // AINFO << std::setprecision(16) << "obs_time_stamp = " << time_stamp;
    PathDecision *const path_decision = reference_line_info_->path_decision();
    const SLBoundary &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
    // AINFO << "adc_start_s = " << adc_sl_boundary.start_s()
    //       << " adc_end_s = " << adc_sl_boundary.end_s();
    // AINFO << "adc_start_l = " << adc_sl_boundary.start_l()
    //       << " adc_end_l = " << adc_sl_boundary.end_l();

    std::array<std::vector<std::string>, 20> consider_obs;
    for (auto *obstacle : path_decision->obstacles().Items()) {
      const auto &obs_sl = obstacle->PerceptionSLBoundary();
      // const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
      double diff_lateral =
          std::max(obs_sl.start_l() - adc_sl_boundary.end_l(),
                   adc_sl_boundary.start_l() - obs_sl.end_l());
      if (obstacle->IsVirtual()) {
        continue;
      }
      // no consider back obs
      if (obs_sl.end_s() < adc_sl_boundary.start_s()) {
        continue;
      }
      // static obs roi : lateral 3m ,lon 30.
      if (obstacle->IsStatic() &&
          (diff_lateral >
               speed_bounds_config_.svm_five_level_lateral_distance() ||
           obs_sl.start_s() - adc_sl_boundary.end_s() >
               speed_bounds_config_.svm_five_level_lon_distance())) {
        continue;
      }
      bool is_unknown = perception::PerceptionObstacle::UNKNOWN ==
                            obstacle->Perception().type() ||
                        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
                            obstacle->Perception().type() ||
                        perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
                            obstacle->Perception().type();
      if (obstacle->IsStatic()) {
        GetStaticObs(diff_lateral, adc_sl_boundary, obs_sl, is_unknown,
                     *obstacle, &consider_obs);
      } else {
        GetDynamicObs(diff_lateral, adc_sl_boundary, obs_sl, is_unknown,
                      *obstacle, &consider_obs);
      }
      // AINFO << "obs_start_s = " << obs_sl.start_s()
      //       << "  obs_end_s = " << obs_sl.end_s() << "  obs_s = " << obs_s;
    }
    AddFeature(time_stamp, consider_obs);

    // save features to file.
    if (FLAGS_enable_record_to_learning_data_for_svm) {
      int file_count = 0;
      // AINFO << "std::to_string(file_count) = " << std::to_string(file_count);
      for (const auto &entry :
           boost::filesystem::directory_iterator(folder_path)) {
        if (boost::filesystem::is_regular_file(entry.path())) {
          ++file_count;
        }
      }
      // AINFO << "file_count = " << file_count;

      // write the last file,else add new file.
      if (!open_new_file) {
        file_count--;
      }
      const std::string dest_file =
          "/century/modules/planning/pipeline/data/svm_feature_data_" +
          std::to_string(file_count) + ".bin.txt";
      cyber::common::AddProtoToASCIIFile(svm_feature_data_, dest_file);
    }
    // use model for prediction
    auto time1 = std::chrono::system_clock::now();
    GetSVMPredictionData(frame);
    auto time2 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = time2 - time1;
    // AINFO << "GET PREDICTION DATA TIME = " << diff.count() * 1000;
  }
}

Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const PathData &path_data = reference_line_info->path_data();
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  PathDecision *const path_decision = reference_line_info->path_decision();
  const SLBoundary &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  GetTrainingData();
  RecordToLearningData(frame);
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type()) {
    MultiDecisionsBeforePrioriSpeedBounds(path_decision);
  }
  reference_line_info->SetEnableShrinkCollisionBuffer(
      injector_->enable_shrink_collision_buffer_);
  // 1. Create speed limit along path
  GetSpeedLimit(reference_line_info);

  // 2. Map obstacles into st graph
  auto time1 = std::chrono::system_clock::now();
  STBoundaryMapper boundary_mapper(
      adc_sl_boundary, speed_bounds_config_, reference_line, path_data,
      reference_line_info_->speed_limit(),
      path_data.discretized_path().Length(), speed_bounds_config_.total_time(),
      injector_, reference_line_info_, frame);

  if (!FLAGS_use_st_drivable_boundary) {
    path_decision->EraseStBoundaries();
  }

  const auto map_ret = boundary_mapper.ComputeSTBoundary(path_decision);
  if (map_ret.code() == ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, map_ret.error_message());
  }
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";

  // SPEED_BOUNDS_PRIORI_DECIDER
  SlowBreakingOnIntersection();
  SlowBreakingReachTurn();
  SlowBreakingForAbnormalPrediction();
  SlowBreakingForApproachingObs();
  UpdateSlowBreakingInLowSpeed();
  SlowBreakingForInitCollisionInFastCutin();

  if (FLAGS_enable_no_emergency_break_reverse_obs) {
    CreatStopWallForReverseObs(frame);
  }

  // SPEED_BOUNDS_FINAL_DECIDER
  SlowBreakingForLargeTtc();
  //  When it is necessary to perform a slow braking, ensure that the decision
  //  wall of not colliding with other obstacles during the slow braking
  //  process.
  BuildStopWallForDecisionInSlowBreaking(frame);
  std::vector<const STBoundary *> boundaries;
  GetBoundaries(&boundaries);

  // only speed limit for stop decision.
  SpeedLimitDecider speed_limit_decider(
      speed_bounds_config_, reference_line_info, injector_->planning_context());
  if (!speed_limit_decider
           .UpdateSpeedLimitsForStopDecision(
               path_decision->obstacles(),
               reference_line_info_->mutable_speed_limit())
           .ok()) {
    const std::string msg = "Update speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          reference_line_info_->speed_limit(),
                          reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

void SpeedBoundsDecider::CreatStopWallForReverseObs(Frame *const frame) {
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type()) {
    PathDecision *const path_decision = reference_line_info_->path_decision();
    double min_stop_s = std::numeric_limits<double>::max();
    const auto &adc_sl = reference_line_info_->AdcSlBoundary();
    const auto &adc_v = reference_line_info_->vehicle_state().linear_velocity();
    for (auto *obstacle : path_decision->obstacles().Items()) {
      // AINFO << "obstacle Id: " << obstacle->Id();
      const auto &obs_sl = obstacle->PerceptionSLBoundary();
      const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;

      if (obstacle->path_st_boundary().IsEmpty()) {
        continue;
      }
      bool is_car_type =
          (PerceptionObstacle::VEHICLE == obstacle->Perception().type());
      double lateral_diff =
          std::max(adc_sl.start_l() - obs_sl.end_l() - kLateralBuffer,
                   obs_sl.start_l() - adc_sl.end_l() - kLateralBuffer);
      bool is_front_obs = obs_sl.start_s() > adc_sl.end_s();
      bool is_lateral_overlap = lateral_diff < 0.0;
      bool is_reverser_car = false;
      const auto &path_point =
          reference_line_info_->path_data().GetPathPointWithPathS(
              obs_s - adc_sl.start_s());
      const auto &obs_velocity = obstacle->Perception().velocity();
      double obs_lateral_speed =
          common::math::Vec2d::CreateUnitVec2d(path_point.theta())
              .CrossProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
      double obs_lon_speed =
          common::math::Vec2d::CreateUnitVec2d(path_point.theta())
              .InnerProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
      if (is_car_type && is_lateral_overlap && is_front_obs) {
        if (obs_lon_speed < kMinReverseSpeed &&
            std::fabs(obs_lon_speed) > std::fabs(obs_lateral_speed)) {
          is_reverser_car = true;
        }
      }
      if ((is_reverser_car || obstacle->IsReverse())) {
        double comfort_slow_down_length =
            std::fabs(adc_v * adc_v * 0.5 / kComfortDecel);
        double yield_distance =
            obs_sl.start_s() - adc_sl.end_s() - std::fabs(obs_lon_speed) * 2.0;
        // AINFO << "comfort_slow_down_length = " << comfort_slow_down_length;
        // AINFO << "yield_distance = " << yield_distance;
        if (comfort_slow_down_length > yield_distance) {
          auto *mutable_obs = path_decision->Find(obstacle->Id());
          if (nullptr == mutable_obs) {
            continue;
          }
          if (mutable_obs->path_st_boundary().IsEmpty()) {
            continue;
          }

          mutable_obs->SetSlowBreakingObstacle(true);
          mutable_obs->SetSlowBreakingTag("CreatStopWallForReverseObs");
          // mutable_obs->EraseStBoundary();
          const auto &vehicle_param =
              common::VehicleConfigHelper::GetConfig().vehicle_param();
          double stop_s = obs_sl.start_s() -
                          vehicle_param.front_edge_to_center() - kStopBuffer;
          if (stop_s < min_stop_s) {
            min_stop_s = stop_s;
          }
        }
      }
    }
    if (min_stop_s <
        reference_line_info_->path_data().frenet_frame_path().Length()) {
      const std::string stop_wall_id = "reverse_stop_wall";
      std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(stop_wall_id, min_stop_s, 0.0,
                              StopReasonCode::STOP_REASON_SLOW_BREAKING,
                              wait_for_obstacles, "StopReasonReverseObs", frame,
                              reference_line_info_);
      // no use qp
      if (FLAGS_enable_slow_breaking_for_reverse_obs) {
        // Obtain the corresponding deceleration based on distance.
        double stop_s = std::numeric_limits<double>::max(),
               min_decel = speed_bounds_config_.min_level_slow_breking_decel();
        if (min_stop_s - adc_sl.end_s() > 0.0) {
          min_decel = -adc_v * adc_v * 0.5 / (min_stop_s - adc_sl.end_s());
        }
        // AINFO << "min_decel = " << min_decel;
        reference_line_info_->SetIsNeedSlowBreaking(true, min_decel, stop_s);
      }
    }
  }
}

bool SpeedBoundsDecider::CheckObsOverlap(
    const common::PathPoint &path_point,
    const planning::SLBoundary &obs_sl_boundary,
    const common::math::Polygon2d &obs_polygon) const {
  // Convert reference point from center of rear axis to center of ADC.
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param.length(), vehicle_param.width());

  // collision detection based on SLBoundary.
  SLBoundary adc_sl_boundary = reference_line_info_->AdcSlBoundary();

  bool no_overlap =
      ((adc_sl_boundary.end_s() < obs_sl_boundary.start_s() ||
        adc_sl_boundary.start_s() > obs_sl_boundary.end_s()) ||  // longitudinal
       (adc_sl_boundary.end_l() < obs_sl_boundary.start_l() ||
        adc_sl_boundary.start_l() > obs_sl_boundary.end_l()));  // lateral
  if (no_overlap) {
    return false;
  }

  ADEBUG << "There is an overlap. adc_sl_boundary.start_s() = "
         << adc_sl_boundary.start_s()
         << ", adc_sl_boundary.end_s() = " << adc_sl_boundary.end_s()
         << ", obs_sl_boundary.start_s() = " << obs_sl_boundary.start_s()
         << ", obs_sl_boundary.end_s() = " << obs_sl_boundary.end_s()
         << ", adc_sl_boundary.start_l() = " << adc_sl_boundary.start_l()
         << ", adc_sl_boundary.end_l() = " << adc_sl_boundary.end_l()
         << ", obs_sl_boundary.start_l() = " << obs_sl_boundary.start_l()
         << ", obs_sl_boundary.end_l() = " << obs_sl_boundary.end_l();

  // collision detection based on Box2d.
  return obs_polygon.HasOverlap(common::math::Polygon2d(adc_box));
}

void SpeedBoundsDecider::GetScopeParamsForSidepass(const bool is_large_scope,
                                                   double *left_scope,
                                                   double *right_scope,
                                                   double *front_scope,
                                                   double *rear_scope) {
  // Set scope parameters for sidepass.
  *left_scope =
      speed_bounds_config_.scope_range_for_side_pass().left_scope_dis();
  *right_scope =
      speed_bounds_config_.scope_range_for_side_pass().right_scope_dis();
  *front_scope =
      speed_bounds_config_.scope_range_for_side_pass().front_scope_dis();
  *rear_scope =
      speed_bounds_config_.scope_range_for_side_pass().rear_scope_dis();
  // Add large area buffer for sidepass.
  if (is_large_scope) {
    *left_scope += kLargeAeraBuffer;
    *right_scope += kLargeAeraBuffer;
    *front_scope += kLargeAeraBuffer;
    *rear_scope += kLargeAeraBuffer;
  }
}

void SpeedBoundsDecider::GetScopeParams(const bool is_start_up,
                                        const bool is_unknown,
                                        const bool is_large_scope,
                                        double *left_scope, double *right_scope,
                                        double *front_scope,
                                        double *rear_scope) {
  if (!is_start_up) {
    *left_scope =
        speed_bounds_config_.scope_range_for_speed_limit().left_scope_dis();
    *right_scope =
        speed_bounds_config_.scope_range_for_speed_limit().right_scope_dis();
    *front_scope =
        speed_bounds_config_.scope_range_for_speed_limit().front_scope_dis();
    *rear_scope =
        speed_bounds_config_.scope_range_for_speed_limit().rear_scope_dis();
  } else {
    if (!is_unknown) {
      if (reference_line_info_->IsAdcInCommonJunction() || is_large_scope) {
        *left_scope =
            speed_bounds_config_.scope_range_in_junction_for_start_up()
                .left_scope_dis();
        *right_scope =
            speed_bounds_config_.scope_range_in_junction_for_start_up()
                .right_scope_dis();
        *front_scope =
            speed_bounds_config_.scope_range_in_junction_for_start_up()
                .front_scope_dis();
        *rear_scope =
            speed_bounds_config_.scope_range_in_junction_for_start_up()
                .rear_scope_dis();
      } else {
        *left_scope =
            speed_bounds_config_.scope_range_not_junction_for_start_up()
                .left_scope_dis();
        *right_scope =
            speed_bounds_config_.scope_range_not_junction_for_start_up()
                .right_scope_dis();
        *front_scope =
            speed_bounds_config_.scope_range_not_junction_for_start_up()
                .front_scope_dis();
        *rear_scope =
            speed_bounds_config_.scope_range_not_junction_for_start_up()
                .rear_scope_dis();
      }
    } else {
      if (reference_line_info_->IsAdcInCommonJunction() || is_large_scope) {
        *left_scope = speed_bounds_config_
                          .scope_range_in_junction_unknown_obs_for_start_up()
                          .left_scope_dis();
        *right_scope = speed_bounds_config_
                           .scope_range_in_junction_unknown_obs_for_start_up()
                           .right_scope_dis();
        *front_scope = speed_bounds_config_
                           .scope_range_in_junction_unknown_obs_for_start_up()
                           .front_scope_dis();
        *rear_scope = speed_bounds_config_
                          .scope_range_in_junction_unknown_obs_for_start_up()
                          .rear_scope_dis();
      } else {
        *left_scope = speed_bounds_config_
                          .scope_range_not_junction_unknown_obs_for_start_up()
                          .left_scope_dis();
        *right_scope = speed_bounds_config_
                           .scope_range_not_junction_unknown_obs_for_start_up()
                           .right_scope_dis();
        *front_scope = speed_bounds_config_
                           .scope_range_not_junction_unknown_obs_for_start_up()
                           .front_scope_dis();
        *rear_scope = speed_bounds_config_
                          .scope_range_not_junction_unknown_obs_for_start_up()
                          .rear_scope_dis();
      }
    }
  }
  if (is_large_scope) {
    *left_scope += kLargeAeraBuffer;
    *right_scope += kLargeAeraBuffer;
    *front_scope += kLargeAeraBuffer;
    *rear_scope += kLargeAeraBuffer;
  }
}

bool SpeedBoundsDecider::GetScopeState(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    double left_scope, double right_scope, double front_scope,
    double rear_scope) {
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto adc_lane = reference_line_info_->LocateLaneInfo(adc_sl.end_s());
  const bool is_left_obs =
      (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
  if (nullptr != adc_lane) {
    const auto &lane = adc_lane->lane();
    if (lane.has_turn()) {
      if (hdmap::Lane::LEFT_TURN == lane.turn() && is_left_obs) {
        rear_scope += kTurnScopeBuffer;
        ADEBUG << "LEFT_TURN, left obs, expand rear scope.";
      } else if (hdmap::Lane::RIGHT_TURN == lane.turn() && !is_left_obs) {
        rear_scope += kTurnScopeBuffer;
        ADEBUG << "RIGHT_TURN, right obs, expand rear scope.";
      } else {
        ADEBUG << "no turn";
      }
    }
  }

  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  bool is_lateral_in_scope =
      (obs_sl.start_l() < frenet_point.l() + adc_half_width + left_scope &&
       obs_sl.start_l() > frenet_point.l()) ||
      (obs_sl.end_l() > frenet_point.l() - adc_half_width - right_scope &&
       obs_sl.end_l() < frenet_point.l());
  bool is_longitudinal_in_scope =
      obs_sl.start_s() < adc_sl.end_s() + front_scope &&
      obs_sl.end_s() > adc_sl.start_s() - rear_scope;
  bool ret = false;
  if (is_lateral_in_scope && is_longitudinal_in_scope) {
    ret = true;
  }
  ADEBUG << "is_lateral_in_scope: " << is_lateral_in_scope;
  ADEBUG << "is_longitudinal_in_scope: " << is_longitudinal_in_scope;
  ADEBUG << "obs l: (" << obs_sl.start_l() << ", " << obs_sl.end_l() << ")";
  ADEBUG << "frenet point l : " << frenet_point.l();
  ADEBUG << "obs s: (" << obs_sl.start_s() << ", " << obs_sl.end_s() << ")";
  ADEBUG << "adc s: (" << adc_sl.start_s() << ", " << adc_sl.end_s() << ")";
  return ret;
}

bool SpeedBoundsDecider::GetScopeStateWithLatOverlap(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    double left_scope, double right_scope, double front_scope,
    double rear_scope) {
  // 1. Compute the scope range.
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto adc_lane = reference_line_info_->LocateLaneInfo(adc_sl.end_s());
  const bool is_left_obs =
      (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
  if (nullptr != adc_lane) {
    const auto &lane = adc_lane->lane();
    if (lane.has_turn()) {
      if (hdmap::Lane::LEFT_TURN == lane.turn() && is_left_obs) {
        rear_scope += kTurnScopeBuffer;
        ADEBUG << "LEFT_TURN, left obs, expand rear scope.";
      } else if (hdmap::Lane::RIGHT_TURN == lane.turn() && !is_left_obs) {
        rear_scope += kTurnScopeBuffer;
        ADEBUG << "RIGHT_TURN, right obs, expand rear scope.";
      } else {
        ADEBUG << "no turn";
      }
    }
  }

  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  // 2.1 Compute the scope state of lateral direction.
  bool is_lateral_in_scope =
      !(obs_sl.start_l() > frenet_point.l() + adc_half_width + left_scope ||
        obs_sl.end_l() < frenet_point.l() - adc_half_width - right_scope);
  // 2.2 Compute the scope state of longitudinal direction.
  bool is_longitudinal_in_scope =
      obs_sl.start_s() < adc_sl.end_s() + front_scope &&
      obs_sl.end_s() > adc_sl.start_s() - rear_scope;
  bool ret = false;
  if (is_lateral_in_scope && is_longitudinal_in_scope) {
    ret = true;
  }
  ADEBUG << "is_lateral_in_scope: " << is_lateral_in_scope;
  ADEBUG << "is_longitudinal_in_scope: " << is_longitudinal_in_scope;
  ADEBUG << "obs l: (" << obs_sl.start_l() << ", " << obs_sl.end_l() << ")";
  ADEBUG << "frenet point l : " << frenet_point.l();
  ADEBUG << "obs s: (" << obs_sl.start_s() << ", " << obs_sl.end_s() << ")";
  ADEBUG << "adc s: (" << adc_sl.start_s() << ", " << adc_sl.end_s() << ")";
  return ret;
}

bool SpeedBoundsDecider::IsObstacleInScopeForSidepass(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const bool is_large_scope) {
  bool ret = false;
  // Obstacle should be non-virtual.
  bool is_bicycle =
      perception::PerceptionObstacle::BICYCLE == obstacle.Perception().type();
  bool is_fast_pedestrian = perception::PerceptionObstacle::PEDESTRIAN ==
                                obstacle.Perception().type() &&
                            obstacle.speed() > 1.0;
  if (obstacle.IsVirtual() || !(is_bicycle || is_fast_pedestrian)) {
    ADEBUG << "Obstacle is not a bicycle or fast pedestrian.";
    return ret;
  }

  // Get scope parameters.
  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParamsForSidepass(is_large_scope, &left_scope, &right_scope,
                            &front_scope, &rear_scope);
  ADEBUG << "left_scope: " << left_scope << " right_scope: " << right_scope
         << " front_scope: " << front_scope << " rear_scope: " << rear_scope;
  // Get scope state with latitude overlap.
  ret = GetScopeStateWithLatOverlap(obstacle, frenet_point, left_scope,
                                    right_scope, front_scope, rear_scope);
  ADEBUG << "IsObstacleInScopeForSidepass: "
         << "is_large_scope(" << is_large_scope << ")" << ret;
  return ret;
}

bool SpeedBoundsDecider::IsObstacleInLargeScope(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const bool is_start_up) {
  bool ret = false;
  // Obstacle should be non-virtual.
  bool is_unknown =
      perception::PerceptionObstacle::UNKNOWN == obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
          obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
          obstacle.Perception().type();
  if (obstacle.IsVirtual() || is_unknown) {
    return ret;
  }

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(is_start_up, is_unknown, true, &left_scope, &right_scope,
                 &front_scope, &rear_scope);

  ret = GetScopeState(obstacle, frenet_point, left_scope, right_scope,
                      front_scope, rear_scope);
  ADEBUG << "IsObstacleInLargeScope: " << ret;
  return ret;
}

bool SpeedBoundsDecider::IsObstacleInScope(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const bool is_start_up) {
  bool ret = false;
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return ret;
  }
  bool is_unknown =
      perception::PerceptionObstacle::UNKNOWN == obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
          obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
          obstacle.Perception().type();
  if (is_unknown) {
    double aspect_ratio =
        obstacle.Perception().length() / obstacle.Perception().width();
    double area =
        obstacle.Perception().length() * obstacle.Perception().width();
    ADEBUG << "aspect_ratio: " << aspect_ratio;
    ADEBUG << "area: " << area;
    if (aspect_ratio >
            speed_bounds_config_.max_aspect_ratio_to_ignore_unknown() ||
        aspect_ratio <
            1 / speed_bounds_config_.max_aspect_ratio_to_ignore_unknown() ||
        area > speed_bounds_config_.max_area_size_to_ignore_unknown() ||
        area < speed_bounds_config_.min_area_size_to_ignore_unknown()) {
      return ret;
    }
  }
  ADEBUG << "is_unknown: " << is_unknown;
  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(is_start_up, is_unknown, false, &left_scope, &right_scope,
                 &front_scope, &rear_scope);

  ret = GetScopeState(obstacle, frenet_point, left_scope, right_scope,
                      front_scope, rear_scope);
  ADEBUG << "IsObstacleInScope: " << ret;
  return ret;
}

bool SpeedBoundsDecider::IsMoveTowardRearAdc(const Obstacle &obstacle) {
  bool ret = false;
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  bool is_longitudinal_no_overlap =
      obs_sl.start_s() > adc_sl.end_s() || obs_sl.end_s() < adc_sl.start_s();
  double diff_heading = 0.0;
  if (!is_longitudinal_no_overlap &&
      obstacle.speed() > FLAGS_min_dynamic_obstacle_speed) {
    const auto &obs_sl = obstacle.PerceptionSLBoundary();
    bool is_left_obs = (obs_sl.start_l() + obs_sl.end_l()) * 0.5 >
                       (adc_sl.end_l() + adc_sl.start_l()) * 0.5;
    double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
    diff_heading =
        -sign_obs_path_l *
        common::math::NormalizeAngle(obstacle.SpeedHeading() -
                                     injector_->vehicle_state()->heading()) *
        kDegrees / M_PI_2;
    ret = !(diff_heading > 0 && diff_heading < 90);
  }

  ADEBUG << "diff_heading = " << diff_heading << ", adc_heading = "
         << injector_->vehicle_state()->heading() * kDegrees / M_PI_2
         << ", obs_heading = " << obstacle.SpeedHeading() * kDegrees / M_PI_2
         << ", IsMoveTowardRearAdc = " << ret;
  return ret;
}

bool SpeedBoundsDecider::IsMoveNearToPathByObsHeading(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const double path_point_heading, const bool is_start_up) {
  static HysteresisInterval obs_interval(
      speed_bounds_config_.obs_cross_angle_degree_for_startup(),
      speed_bounds_config_.hy_buffer_of_cross_angle_for_startup(), 50UL);
  static HysteresisInterval obs_interval_for_speed_limit(
      speed_bounds_config_.obs_cross_angle_degree_for_speed_limit(),
      speed_bounds_config_.hy_buffer_of_cross_angle_for_speed_limit(), 50UL);
  double diff_heading = 0.0;
  double obs_heading = obstacle.SpeedHeading();
  if (obstacle.speed() > FLAGS_min_dynamic_obstacle_speed) {
    const auto &obs_sl = obstacle.PerceptionSLBoundary();
    bool is_left_obs =
        (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
    double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
    diff_heading =
        std::fabs(common::math::AngleDiff(
            path_point_heading, obs_heading + M_PI_2 * sign_obs_path_l)) *
        kDegrees / M_PI_2;
  } else {
    diff_heading = std::abs(std::abs(common::math::NormalizeAngle(
                                obs_heading - path_point_heading)) -
                            M_PI_2) *
                   kDegrees / M_PI_2;
  }
  bool obs_near_to_path = false;
  if (is_start_up) {
    obs_near_to_path =
        obs_interval.HyValue(obstacle, diff_heading) <
        speed_bounds_config_.obs_cross_angle_degree_for_startup();
  } else {
    obs_near_to_path =
        obs_interval_for_speed_limit.HyValue(obstacle, diff_heading) <
        speed_bounds_config_.obs_cross_angle_degree_for_speed_limit();
  }
  ADEBUG << "diff_heading = " << diff_heading
         << ", path_point_heading = " << path_point_heading * kDegrees / M_PI_2
         << ", obs_heading = " << obs_heading * kDegrees / M_PI_2
         << ", obs_near_to_path = " << obs_near_to_path;
  return obs_near_to_path;
}

bool SpeedBoundsDecider::IsMoveAwayFromPathByObsHeading(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const double path_point_heading) {
  // HysteresisInterval is used to determine if the obstacle is away from
  // the path by its heading.
  static HysteresisInterval away_obs_interval(
      speed_bounds_config_.obs_cross_angle_degree_for_away_obs(),
      speed_bounds_config_.hy_buffer_of_cross_angle_for_away_obs(), 50UL);
  // Check if the obstacle is in the path.
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  if (obs_sl.start_l() < frenet_point.l() + adc_half_width &&
      obs_sl.end_l() > frenet_point.l() - adc_half_width) {
    // The obstacle is in the path.
    return false;
  }
  // Calculate the heading difference between the obstacle and the path.
  double diff_heading = 0.0;
  double obs_heading = obstacle.SpeedHeading();
  if (obstacle.speed() > FLAGS_min_dynamic_obstacle_speed) {
    // Check if the obstacle is on the left side or right side of the path.
    bool is_left_obs =
        (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
    double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
    diff_heading =
        std::fabs(common::math::AngleDiff(
            path_point_heading, obs_heading + M_PI_2 * sign_obs_path_l)) *
        kDegrees / M_PI_2;
  }
  // Determine if the obstacle is away from the path by its heading.
  bool obs_away_from_path =
      away_obs_interval.HyValue(obstacle, diff_heading) >
      speed_bounds_config_.obs_cross_angle_degree_for_away_obs();
  ADEBUG << "diff_heading = " << diff_heading
         << ", path_point_heading = " << path_point_heading * kDegrees / M_PI_2
         << ", obs_heading = " << obs_heading * kDegrees / M_PI_2
         << ", obs_away_from_path = " << obs_away_from_path;
  return obs_away_from_path;
}

double SpeedBoundsDecider::GetStopDistanceForDangerousObs(
    const Obstacle &obstacle, const double path_point_heading) {
  const double diff_angle =
      std::abs(common::math::AngleDiff(path_point_heading - M_PI,
                                       obstacle.SpeedHeading())) *
      kDegrees / M_PI_2;
  double stop_distance = InterpolationLookUp(
      diff_angle, speed_bounds_config_.min_angle_for_dangerous_stop_distance(),
      speed_bounds_config_.max_angle_for_dangerous_stop_distance(),
      speed_bounds_config_.max_stop_distance_for_dangerous_startup(),
      speed_bounds_config_.min_stop_distance_for_dangerous_startup());
  return stop_distance;
}

void SpeedBoundsDecider::UpdateContainerItem(
    const Obstacle &obstacle, double frenet_point_l,
    const DangerousObsContainer::iterator &it) {
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  double obs_l = (obs_sl.start_l() + obs_sl.end_l()) * 0.5;
  common::math::Vec2d attention_xy(0.0, 0.0);
  common::SLPoint attention_sl;
  if (obs_l > frenet_point_l) {
    attention_sl.set_s(obs_sl.start_s());
    attention_sl.set_l(obs_sl.start_l());
  } else {
    attention_sl.set_s(obs_sl.start_s());
    attention_sl.set_l(obs_sl.end_l());
  }
  reference_line_info_->reference_line().SLToXY(attention_sl, &attention_xy);
  const auto &adc_pos = reference_line_info_->vehicle_state();
  common::math::Vec2d adc_xy(adc_pos.x(), adc_pos.y());
  double distance = adc_xy.DistanceTo(attention_xy);
  int delay_times_for_stop = InterpolationLookUp(
      distance, speed_bounds_config_.max_distance_for_delay_startup(),
      speed_bounds_config_.min_distance_for_delay_startup(),
      speed_bounds_config_.min_delay_frame_times_for_stop(),
      speed_bounds_config_.max_delay_frame_times_for_stop());
  if (obs_container_.end() == it) {
    obs_container_.emplace(
        std::to_string(obstacle.PerceptionId()),
        SpeedBoundsDecider::ObstacleInfo(
            delay_times_for_stop,
            speed_bounds_config_.delay_frame_times_for_limit_speed(),
            injector_->GetSequenceNum()));
  } else {
    it->second.delay_stop_times = delay_times_for_stop;
    it->second.delay_speed_limit_times =
        speed_bounds_config_.delay_frame_times_for_limit_speed();
    it->second.last_seq_num = injector_->GetSequenceNum();
  }
}

void SpeedBoundsDecider::UpdateVehicleContainerItem(
    const Obstacle &obstacle, double frenet_point_l,
    const DangerousObsContainer::iterator &it) {
  // const auto &obs_sl = obstacle.PerceptionSLBoundary();
  // double obs_l = (obs_sl.start_l() + obs_sl.end_l()) * 0.5;
  // common::math::Vec2d attention_xy(0.0, 0.0);
  // common::SLPoint attention_sl;
  // if (obs_l > frenet_point_l) {
  //   attention_sl.set_s(obs_sl.start_s());
  //   attention_sl.set_l(obs_sl.start_l());
  // } else {
  //   attention_sl.set_s(obs_sl.start_s());
  //   attention_sl.set_l(obs_sl.end_l());
  // }
  // reference_line_info_->reference_line().SLToXY(attention_sl, &attention_xy);
  // const auto &adc_pos = reference_line_info_->vehicle_state();
  // common::math::Vec2d adc_xy(adc_pos.x(), adc_pos.y());
  // double distance = adc_xy.DistanceTo(attention_xy);
  const auto adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y(),
      injector_->vehicle_state()->heading(), 0.0, 0.0);
  const auto adc_polygon = common::math::Polygon2d(adc_box);
  const auto obs_polygon = obstacle.PerceptionPolygon();
  double distance = adc_polygon.DistanceTo(obs_polygon);
  int delay_times_for_stop = InterpolationLookUp(
      distance, speed_bounds_config_.min_distance_for_delay_startup(),
      speed_bounds_config_.max_distance_for_delay_startup(),
      speed_bounds_config_.max_delay_frame_times_for_stop(),
      speed_bounds_config_.min_delay_frame_times_for_stop());
  AINFO << "delay_times_for_stop = " << delay_times_for_stop;
  if (vehicle_container_.end() == it) {
    vehicle_container_.emplace(
        std::to_string(obstacle.PerceptionId()),
        SpeedBoundsDecider::ObstacleInfo(
            delay_times_for_stop,
            speed_bounds_config_.delay_frame_times_for_limit_speed(),
            injector_->GetSequenceNum()));
  } else {
    it->second.delay_stop_times = delay_times_for_stop;
    it->second.delay_speed_limit_times =
        speed_bounds_config_.delay_frame_times_for_limit_speed();
    it->second.last_seq_num = injector_->GetSequenceNum();
    // if (std::to_string(obstacle.PerceptionId()) == "6607") {
    //   AINFO << "obstacle id = 6607, delay_times_for_stop = "
    //         << delay_times_for_stop;
    // }
  }
}

void SpeedBoundsDecider::UpdateSpeedLimitContainerItem(
    const Obstacle &obstacle,
    const DangerousObsForSpeedLimitContainer::iterator &it) {
  if (speed_limit_obs_container_.end() == it) {
    speed_limit_obs_container_.emplace(
        std::to_string(obstacle.PerceptionId()),
        speed_bounds_config_.delay_frame_times_of_approach_obs());
  } else {
    it->second = speed_bounds_config_.delay_frame_times_of_approach_obs();
  }
}

SpeedBoundsDecider::ObsLongiPos SpeedBoundsDecider::GetObstacleLongiPosition(
    const Obstacle &obstacle) {
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  const double adc_center_s = 0.5 * (adc_sl.end_s() + adc_sl.start_s());
  ObsLongiPos ret = UNKNOWN_POS;
  // Check the longitudinal position of the obstacle.
  if (obs_sl.start_s() > adc_sl.end_s()) {
    // The obstacle is in the front of the ADC totally.
    ret = FRONT_OF_ADC_TOTALLY;
  } else if (obs_sl.end_s() < adc_sl.start_s() - kLonBuffer) {
    // The obstacle is in the behind of the ADC totally.
    ret = BEHIND_ADC_TOTALLY;
  } else if (obs_sl.end_s() > adc_center_s) {
    // The obstacle is in the front of the ADC with overlap.
    ret = FRONT_OF_ADC_WITH_OVERLAP;
  } else {
    // The obstacle is in the behind of the ADC with overlap.
    ret = BEHIND_ADC_WITH_OVERLAP;
  }
  ADEBUG << "Obstacle longi position: " << ret;
  return ret;
}

bool SpeedBoundsDecider::IsObstacleNearPath(const Obstacle &obstacle) {
  static ObstacleHistoryValue obstacles_history_value_for_sidepass(50UL);
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const double obs_s = 0.5 * (obs_sl.end_s() + obs_sl.start_s());
  // Evaluate the frenet point by the obstacle s.
  const auto frenet_point =
      reference_line_info_->path_data().frenet_frame_path().EvaluateByS(obs_s);
  // Check if the obstacle is near to the path by history information.
  bool is_near_to_path =
      obstacles_history_value_for_sidepass.IsMoveNearToPathByDiffL(
          obstacle, frenet_point, true);
  ADEBUG << "Obstacle is near to path: " << is_near_to_path;
  return is_near_to_path;
}

// Is overlap on latitude
bool SpeedBoundsDecider::IsLatOverlapWithBuffer(const Obstacle &obstacle,
                                                const double lat_buffer) {
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  const double adc_l = 0.5 * (adc_sl.end_l() + adc_sl.start_l());
  const double obs_l = 0.5 * (obs_sl.end_l() + obs_sl.start_l());
  const bool is_obs_on_left_side = obs_l > adc_l;
  bool is_lat_overlap = is_obs_on_left_side
                            ? (obs_sl.start_l() - adc_sl.end_l() < lat_buffer)
                            : (adc_sl.start_l() - obs_sl.end_l() < lat_buffer);
  ADEBUG << "lat_buffer: " << lat_buffer;
  ADEBUG << "IsLatOverlapWithBuffer: " << is_lat_overlap;
  return is_lat_overlap;
}

// The obstacle is almost in the same direction as it is moving
bool SpeedBoundsDecider::IsSameDirectionWithNearPathPoint(
    const Obstacle &obstacle) {
  // Heading hysteresis interval.
  static HysteresisInterval heading_interval(
      speed_bounds_config_.max_diff_angle_for_same_orientation(),
      speed_bounds_config_.hy_buffer_lower_for_same_orientation(),
      speed_bounds_config_.hy_buffer_upper_for_same_orientation(), 50UL);

  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const double obs_s = 0.5 * (obs_sl.end_s() + obs_sl.start_s());
  // Get the path point on the reference line with the same s as the obstacle.
  const auto path_point =
      reference_line_info_->path_data().GetPathPointWithPathS(obs_s -
                                                              adc_sl.start_s());
  // Get the reference point on the reference line with the same s as the
  // obstacle.
  const auto refer_point =
      reference_line_info_->reference_line().GetNearestReferencePoint(obs_s);
  // Whether the obstacle is in the rear of ADC.
  bool is_rear_obs = obs_s < adc_sl.start_s();
  // Get the reference heading.
  const double refer_heading =
      is_rear_obs ? refer_point.heading() : path_point.theta();

  // Calculate the absolute value of the difference of the heading between
  // the obstacle and the reference point.
  double diff_heading_abs = std::abs(
      common::math::NormalizeAngle(obstacle.SpeedHeading() - refer_heading) *
      kDegrees / M_PI_2);
  // Convert the obstacle's id to a string.
  const std::string hy_obs_id = std::to_string(obstacle.PerceptionId());
  // Calculate the hysteresis value of the difference of the heading.
  const double hy_diff_heading =
      heading_interval.HyValue(hy_obs_id, diff_heading_abs);
  ADEBUG << "diff_heading = " << diff_heading_abs
         << ", hy_diff_heading = " << hy_diff_heading
         << ", obs_heading = " << obstacle.SpeedHeading() * kDegrees / M_PI_2
         << ", refer_heading = " << refer_heading * kDegrees / M_PI_2;
  // Return true if the difference of the heading is within the hysteresis
  // interval.
  return hy_diff_heading <
         speed_bounds_config_.max_diff_angle_for_same_orientation();
}

// Get the distance between the obstacle and ADC boundary
void SpeedBoundsDecider::LatDistanceAndSpeedOfNearAdc(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    double *lateral_dis, double *lateral_speed, double *longitude_speed) {
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  const double adc_heading = reference_line_info_->vehicle_state().heading();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  // Check if the obstacle is on the left side of ADC.
  bool is_left_obs =
      (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
  // Calculate the lateral distance between ADC and obstacle.
  *lateral_dis = is_left_obs
                     ? (obs_sl.start_l() - frenet_point.l() - adc_half_width)
                     : (frenet_point.l() - obs_sl.end_l() - adc_half_width);
  // The sign.
  double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
  // The velocity of obstacle.
  const auto &obs_velocity = obstacle.Perception().velocity();
  // Calculate the lateral speed between ADC and obstacle.
  *lateral_speed = -sign_obs_path_l *
                   common::math::Vec2d::CreateUnitVec2d(adc_heading)
                       .CrossProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
  // Calculate the longitude speed between ADC and obstacle.
  *longitude_speed = common::math::Vec2d::CreateUnitVec2d(adc_heading)
                         .InnerProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
  ADEBUG << "is_left_obs = " << is_left_obs;
  ADEBUG << "lateral_dis = " << *lateral_dis;
  ADEBUG << "lateral_speed = " << *lateral_speed;
  ADEBUG << "longitude_speed = " << *longitude_speed;
}

//  - FASTER_MOVING: Obstacle is moving faster than ADC.
//  - SLOWER_MOVING: Obstacle is moving slower than ADC.
//  - STABLE_MOVING: Obstacle is moving at a speed close to ADC.
ObstacleHistoryDiffValue::MovingState SpeedBoundsDecider::GetObstacleMoveState(
    const Obstacle &obstacle, double *relative_speed) {
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto moving_state =
      obstacle_history_diff_value.GetObstacleMoveStateByDiffS(obstacle, adc_sl,
                                                              relative_speed);
  ADEBUG << "moving_state = " << moving_state;
  return moving_state;
}

// Check if the obstacle is moving faster than ADC.
bool SpeedBoundsDecider::IsObstacleFasterMoving(
    const ObstacleHistoryDiffValue::MovingState &move_state) {
  return ObstacleHistoryDiffValue::FASTER_MOVING == move_state;
}

// Check if the obstacle is moving slower than ADC.
bool SpeedBoundsDecider::IsObstacleSlowerMoving(
    const ObstacleHistoryDiffValue::MovingState &move_state) {
  return ObstacleHistoryDiffValue::SLOWER_MOVING == move_state;
}

// Check if the obstacle is moving at a speed close to ADC.
bool SpeedBoundsDecider::IsObstacleStableMoving(
    const ObstacleHistoryDiffValue::MovingState &move_state) {
  return ObstacleHistoryDiffValue::STABLE_MOVING == move_state;
}

bool SpeedBoundsDecider::MakeIgnoreDecision(const std::string obs_id,
                                            const std::string decision_tag,
                                            PathDecision *const path_decision) {
  auto *mutable_obstacle = path_decision->Find(obs_id);
  if (nullptr == mutable_obstacle) {
    // no obstacle found
    return false;
  }
  // set the ignore decision
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  mutable_obstacle->AddLongitudinalDecision(decision_tag, ignore);
  reference_line_info_->AddIgnoreObstacle(obs_id);
  AINFO << "side pass Ignore, obs ID: " << obs_id
        << ", decision_tag: " << decision_tag;
  return true;
}

void SpeedBoundsDecider::MakeIgnoreDecisionForBehindOverlapObstacle(
    const Obstacle &obstacle,
    const ObstacleHistoryDiffValue::MovingState move_state,
    const bool is_move_away_obs, const double relative_speed,
    PathDecision *const path_decision, double *target_speed,
    double *target_accel) {
  if (!IsLatOverlapWithBuffer(
          obstacle, speed_bounds_config_.lat_overlap_small_buffer())) {
    std::string decision_tag = "sidepass/behind_adc_with_overlap";
    const double adc_speed =
        injector_->vehicle_state()->vehicle_state().linear_velocity();
    if (IsObstacleSlowerMoving(move_state) &&
        obstacle.speed() <=
            adc_speed *
                speed_bounds_config_.slower_speed_ratio_for_behind_obs()) {
      MakeIgnoreDecision(obstacle.Id(), decision_tag, path_decision);
    } else if (is_move_away_obs) {
      if (IsObstacleStableMoving(move_state) &&
          obstacle.speed() >
              adc_speed *
                  speed_bounds_config_.slower_speed_ratio_for_behind_obs() &&
          obstacle.speed() <
              adc_speed *
                  speed_bounds_config_.faster_speed_ratio_for_behind_obs()) {
        decision_tag = "sidepass/slower brake";
        // given the brake parameter
        double obs_target_speed = std::max(
            speed_bounds_config_.min_target_speed_ratio_for_sidepass_brake() *
                adc_speed,
            std::min(speed_bounds_config_.slower_speed_ratio_for_behind_obs() *
                         obstacle.speed(),
                     adc_speed + relative_speed -
                         speed_bounds_config_.faster_diff_speed()));
        *target_speed = std::min(*target_speed, obs_target_speed);
        *target_accel = std::min(
            *target_accel,
            speed_bounds_config_.obstacle_target_accel_for_sidepass_brake());
      }
      MakeIgnoreDecision(obstacle.Id(), decision_tag, path_decision);
    }
  }
}

void SpeedBoundsDecider::MakeIgnoreOrBrakeDecisionByObstacleState(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const ObstacleHistoryDiffValue::MovingState move_state,
    const double relative_speed, const bool is_obs_near_to_path,
    PathDecision *const path_decision, double *target_speed,
    double *target_accel) {
  // Check if the obstacle is moving in the same direction as ADC.
  bool is_move_away_obs = IsMoveAwayFromSegmentPathByObsHeading(
      obstacle, reference_line_info_->path_data());
  // bool is_same_direction = IsSameDirectionWithNearPathPoint(obstacle);
  // Get the obstacle's longitudinal position.
  ObsLongiPos obs_longi_pos = GetObstacleLongiPosition(obstacle);
  ADEBUG << "is_move_away_obs: " << is_move_away_obs
         << "\n\t\tobs_longi_pos: " << obs_longi_pos;
  // Get the obstacle's lateral distance, lateral speed and longitudinal
  // speed.
  double obs_lat_dis = 0.0, obs_lat_speed = 0.0, obs_lon_speed = 0.0;
  LatDistanceAndSpeedOfNearAdc(obstacle, frenet_point, &obs_lat_dis,
                               &obs_lat_speed, &obs_lon_speed);
  ADEBUG << "obstacle speed: " << obstacle.speed()
         << "\n\t\tobs near to path lat_dis: " << obs_lat_dis
         << "\n\t\tobs near to path lat_speed: " << obs_lat_speed
         << "\n\t\tobs near to path lon_speed: " << obs_lon_speed;
  // Calculate the time the obstacle will pass the ADC.
  double keep_approach_time = (obs_lat_speed > 0.0)
                                  ? (obs_lat_dis / obs_lat_speed)
                                  : std::numeric_limits<double>::max();
  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  const double adc_accel =
      injector_->vehicle_state()->vehicle_state().linear_acceleration();
  ADEBUG << "keep_approach_time: " << keep_approach_time
         << "\n\t\tadc speed: " << adc_speed;
  switch (obs_longi_pos) {
    case FRONT_OF_ADC_TOTALLY:
    case FRONT_OF_ADC_WITH_OVERLAP:
      if (is_move_away_obs && !is_obs_near_to_path &&
          keep_approach_time >
              speed_bounds_config_.keep_approach_time_buffer() &&
          !IsLatOverlapWithBuffer(
              obstacle, speed_bounds_config_.lat_overlap_large_buffer())) {
        MakeIgnoreDecision(obstacle.Id(), "sidepass/front_adc_slower_obs",
                           path_decision);
      } else if (FRONT_OF_ADC_TOTALLY == obs_longi_pos &&
                 obs_lon_speed >=
                     speed_bounds_config_.faster_speed_ratio_for_front_obs() *
                         adc_speed &&
                 IsObstacleFasterMoving(move_state)) {
        if (obs_lon_speed - adc_speed >
                speed_bounds_config_.very_faster_diff_speed() ||
            adc_accel < speed_bounds_config_.acceleration_buffer()) {
          MakeIgnoreDecision(obstacle.Id(), "sidepass/front_adc_faster_obs",
                             path_decision);
        } else {
          MakeIgnoreDecision(obstacle.Id(), "sidepass/front_adc_average_speed",
                             path_decision);
          // given the brake parameter
          *target_speed = std::min(*target_speed, adc_speed);
          *target_accel = std::min(*target_accel, 0.0);
        }
      }
      break;
    case BEHIND_ADC_TOTALLY:
      if (IsLatOverlapWithBuffer(
              obstacle, speed_bounds_config_.lat_overlap_large_buffer()) ||
          IsObstacleSlowerMoving(move_state) || is_move_away_obs) {
        MakeIgnoreDecision(obstacle.Id(), "sidepass/behind_adc", path_decision);
      }
      break;
    case BEHIND_ADC_WITH_OVERLAP:
      MakeIgnoreDecisionForBehindOverlapObstacle(
          obstacle, move_state, is_move_away_obs, relative_speed, path_decision,
          target_speed, target_accel);
      break;
    default:
      break;
  }
}

void SpeedBoundsDecider::MakeObstacleSidepassDecision(
    PathDecision *const path_decision) {
  ADEBUG << "MakeObstacleSidepassDecision!";
  // Initialize the target speed.
  double target_speed = kMaxTargetSpeed;
  double target_accel = kMaxTargetSpeed;
  // Check all the obstacles.
  for (auto *obstacle : path_decision->obstacles().Items()) {
    ADEBUG << "obstacle Id: " << obstacle->Id();
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    const auto frenet_point =
        reference_line_info_->path_data().frenet_frame_path().EvaluateByS(
            obs_s);
    // Check if the obstacle is in large scope for sidepass.
    if (!IsObstacleInScopeForSidepass(*obstacle, frenet_point, true)) {
      ADEBUG << "Not In LargeScope For Sidepass";
      continue;
    }
    // Check if the obstacle is near to the path.
    bool is_obs_near_to_path = IsObstacleNearPath(*obstacle);
    double relative_speed = std::numeric_limits<double>::max();
    // Get obstacle's moving state.
    const auto move_state = GetObstacleMoveState(*obstacle, &relative_speed);
    ADEBUG << "move_state: " << move_state
           << "\n\t\trelative_speed: " << relative_speed;
    // Check if the obstacle is in scope for sidepass.
    if (!IsObstacleInScopeForSidepass(*obstacle, frenet_point, false)) {
      ADEBUG << "Not In Scope For Sidepass";
      continue;
    }

    // Make ignore or brake decisions according to obstacle's state.
    MakeIgnoreOrBrakeDecisionByObstacleState(
        *obstacle, frenet_point, move_state, relative_speed,
        is_obs_near_to_path, path_decision, &target_speed, &target_accel);
  }
  // Set the target speed and acceleration.
  if (target_speed < kMaxTargetSpeed) {
    reference_line_info_->SetSlowerBreakingParam(target_speed, target_accel);
  }
  // Clear the obsolete elements.
  obstacle_history_diff_value.ClearObsoleteElements();
}

bool SpeedBoundsDecider::GetTargetSpeedForApproachingObs(
    const Obstacle *approaching_obstacle, const SLBoundary &adc_sl,
    double *obs_target_speed) {
  const SLBoundary &obs_sl = approaching_obstacle->PerceptionSLBoundary();
  const common::FrenetFramePoint frenet_point =
      reference_line_info_->path_data().frenet_frame_path().EvaluateByS(
          obs_sl.start_s());
  const common::FrenetFramePoint first_frenet_point =
      reference_line_info_->path_data().frenet_frame_path().front();
  const common::PathPoint path_point =
      reference_line_info_->path_data().discretized_path().Evaluate(
          obs_sl.start_s() - first_frenet_point.s());
  const double path_point_heading = path_point.theta();
  double obs_path_l = std::abs(obs_sl.end_l() - frenet_point.l()) <
                              std::abs(obs_sl.start_l() - frenet_point.l())
                          ? obs_sl.end_l() - frenet_point.l()
                          : obs_sl.start_l() - frenet_point.l();
  const auto &vehicle_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  double obs_path_abs_l = std::abs(obs_path_l) - vehicle_half_width;

  // if diff l is bigger than diff s, than ignore the obstacle(don't do speed
  // limit).
  ADEBUG << "obs_path_l: " << obs_path_l
         << " obs_path_abs_l: " << obs_path_abs_l
         << " vehicle_half_width: " << vehicle_half_width;
  ADEBUG << "obs_sl.end_s(): " << obs_sl.end_s()
         << " adc_sl.end_s(): " << adc_sl.end_s();
  if (obs_path_abs_l < 0.0 || obs_sl.end_s() < adc_sl.end_s() ||
      obs_path_abs_l > speed_limit_config_.aspect_range_ratio() *
                           (obs_sl.end_s() - adc_sl.end_s())) {
    return false;
  }

  const auto &velocity = approaching_obstacle->Perception().velocity();
  const double obs_lat_speed =
      std::abs(common::math::Vec2d::CreateUnitVec2d(path_point_heading)
                   .CrossProd(Vec2d(velocity.x(), velocity.y())));
  ADEBUG << "dis = " << obs_path_abs_l << ", obs_lat_speed = " << obs_lat_speed;
  if (obs_path_abs_l < 1.0) {
    *obs_target_speed =
        speed_limit_config_.min_speed_limit_ratio_for_approach_obs() *
        FLAGS_planning_upper_speed_limit;
  } else {
    double min_dis = speed_limit_config_.min_distance_for_fast_approach_obs();
    double max_dis = speed_limit_config_.max_distance_for_fast_approach_obs();
    double min_limit_v =
        speed_limit_config_.min_speed_limit_ratio_for_approach_obs() *
        FLAGS_planning_upper_speed_limit;
    double max_limit_v = speed_limit_config_.max_speed_limit_ratio() *
                         FLAGS_planning_upper_speed_limit;
    if (obs_lat_speed < speed_limit_config_.lower_approach_obs_speed()) {
      min_dis = speed_limit_config_.min_distance_lower_for_approach_obs();
      max_dis = speed_limit_config_.max_distance_lower_for_approach_obs();
    } else if (obs_lat_speed < speed_limit_config_.upper_approach_obs_speed()) {
      min_dis = speed_limit_config_.min_distance_middle_for_approach_obs();
      max_dis = speed_limit_config_.max_distance_middle_for_approach_obs();
    } else if (obs_lat_speed <
               speed_bounds_config_.speed_of_faster_move_obstacle()) {
      min_dis = speed_limit_config_.min_distance_upper_for_approach_obs();
      max_dis = speed_limit_config_.max_distance_upper_for_approach_obs();
    }
    *obs_target_speed = InterpolationLookUp(obs_path_abs_l, min_dis, max_dis,
                                            min_limit_v, max_limit_v);
  }
  return true;
}

void SpeedBoundsDecider::KinematicSlowlyDownForApproachingObs(
    PathDecision *const path_decision) {
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  double adc_v = injector_->vehicle_state()->linear_velocity();
  double target_accel = 0.0, target_speed = FLAGS_planning_upper_speed_limit;
  for (const auto obs_id :
       reference_line_info_->GetKeepApproachingObstacles()) {
    ADEBUG << "approaching obstacle id: " << obs_id;
    const auto *approaching_obstacle = path_decision->Find(obs_id);
    if (nullptr == approaching_obstacle) {
      continue;
    }
    double obs_target_speed = std::numeric_limits<double>::max();
    if (!GetTargetSpeedForApproachingObs(approaching_obstacle, adc_sl,
                                         &obs_target_speed)) {
      continue;
    }
    ADEBUG << "speed limit for appro obs(" << obs_id << ")" << obs_target_speed;
    const SLBoundary &obs_sl = approaching_obstacle->PerceptionSLBoundary();
    double obs_target_s = obs_sl.start_s() - adc_sl.end_s() -
                          speed_bounds_config_.advance_buffer();
    double obs_target_accel = 0.0;
    if (obs_target_speed < adc_v) {
      if (obs_target_s > 0.0) {
        obs_target_accel =
            std::max(speed_bounds_config_.min_deceleration_for_approach_obs(),
                     (obs_target_speed * obs_target_speed - adc_v * adc_v) /
                         (2.0 * obs_target_s));
      } else {
        obs_target_accel =
            speed_bounds_config_.min_deceleration_for_approach_obs();
      }
    }
    if (target_accel > obs_target_accel) {
      target_accel = obs_target_accel;
      target_speed = obs_target_speed;
    }
  }
  if (target_speed < FLAGS_planning_upper_speed_limit && target_accel < 0.0) {
    ADEBUG << "SetSlowlyBreakingParamForApproachObs:\n\t"
           << "target_speed: " << target_speed
           << ", target_accel: " << target_accel;
    reference_line_info_->SetSlowlyBreakingParamForApproachObs(target_speed,
                                                               target_accel);
  }
}

void SpeedBoundsDecider::SetSpeedLimitsForKeepApproachingObs(
    PathDecision *const path_decision) {
  ADEBUG << "SetSpeedLimitsForKeepApproachingObs!";
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
    const auto frenet_point =
        reference_line_info_->path_data().frenet_frame_path().EvaluateByS(
            obs_s);
    const auto &path_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            obs_s - adc_sl.start_s());
    const double path_point_heading = path_point.theta();
    const std::string id = std::to_string(obstacle->PerceptionId());
    ADEBUG << "obstacle Id: " << obstacle->Id();
    if (!IsObstacleInLargeScope(*obstacle, frenet_point, false)) {
      ADEBUG << "Not IsObstacleInLargeScope";
      continue;
    }
    auto it = speed_limit_obs_container_.find(id);
    if (IsMoveAwayFromPathByObsHeading(*obstacle, frenet_point,
                                       path_point_heading) ||
        IsMoveTowardRearAdc(*obstacle)) {
      if (speed_limit_obs_container_.end() != it) {
        it->second = 0UL;
      }
      continue;
    }
    bool added_keep_approach_obs = false;
    if (speed_limit_obs_container_.end() != it && it->second > 0UL) {
      ADEBUG << "KeepApproachingObs";
      reference_line_info_->AddKeepApproachingObstacle(obstacle->Id());
      added_keep_approach_obs = true;
      --it->second;
      ADEBUG << "left times: " << it->second;
    }
    bool is_left_obs =
        (obs_sl.start_l() + obs_sl.end_l()) * 0.5 > frenet_point.l();
    double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
    const auto &obs_velocity = obstacle->Perception().velocity();
    double obs_lat_speed =
        -sign_obs_path_l *
        common::math::Vec2d::CreateUnitVec2d(path_point_heading)
            .CrossProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
    ADEBUG << "is_left_obs: " << is_left_obs
           << ", obs_lat_speed: " << obs_lat_speed;
    // As long as the obstacle is in large range, it is judged whether it has
    // a tendency close to the path, so as to record the obstacle trend earlier.
    bool is_near_to_path =
        (obstacles_history_value_for_speed_limit.IsMoveNearToPathByDiffL(
             *obstacle, frenet_point,
             reference_line_info_->IsAdcInCommonJunction()) ||
         obs_lat_speed >
             speed_bounds_config_.speed_of_faster_move_obstacle()) &&
        IsMoveNearToPathByObsHeading(*obstacle, frenet_point,
                                     path_point_heading, false);
    if (!IsObstacleInScope(*obstacle, frenet_point, false)) {
      ADEBUG << "Not IsObstacleInScope";
      continue;
    }
    if (is_near_to_path) {
      ADEBUG << "enter KeepApproachingObs";
      if (!added_keep_approach_obs) {
        reference_line_info_->AddKeepApproachingObstacle(obstacle->Id());
      }
      UpdateSpeedLimitContainerItem(*obstacle, it);
    }
  }
}

void SpeedBoundsDecider::MakeDangerousStartUpDecision(
    PathDecision *const path_decision) {
  ADEBUG << "MakeDangerousStartUpDecision!";
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
    const auto frenet_point =
        reference_line_info_->path_data().frenet_frame_path().EvaluateByS(
            obs_s);
    const auto &path_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            obs_s - adc_sl.start_s());
    const double path_point_heading = path_point.theta();
    const std::string id = std::to_string(obstacle->PerceptionId());
    ADEBUG << "obstacle Id: " << obstacle->Id();
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->LongitudinalDecision().has_overtake()) {
      continue;
    }
    if (!IsObstacleInLargeScope(*obstacle, frenet_point, true)) {
      continue;
    }

    // As long as the obstacle is in large range, it is judged whether it has
    // a tendency close to the path, so as to record the obstacle trend earlier.
    bool is_near_to_path =
        obstacles_history_value.IsMoveNearToPathByDiffL(
            *obstacle, frenet_point,
            reference_line_info_->IsAdcInCommonJunction()) &&
        IsMoveNearToPathByObsHeading(*obstacle, frenet_point,
                                     path_point_heading, true);

    auto it = obs_container_.find(id);
    if (IsMoveAwayFromPathByObsHeading(*obstacle, frenet_point,
                                       path_point_heading) ||
        IsMoveTowardRearAdc(*obstacle) || obstacle->GetSidepassState()) {
      if (obs_container_.end() != it) {
        it->second.delay_stop_times = 0UL;
      }
      continue;
    }

    const double stop_distance =
        GetStopDistanceForDangerousObs(*obstacle, path_point_heading);
    bool has_build_stop_decision = false;
    if (is_adc_start_up_ && obs_container_.end() != it) {
      if (it->second.delay_stop_times > 0UL) {
        ADEBUG << "DangerousStartUp, Need Stop!";
        ObjectDecisionType object_decision;
        const std::string stop_tag = "DangerousStartUp/near_to_path";
        util::GenerateObjectStopDecision(*obstacle, stop_distance, stop_tag,
                                         *reference_line_info_,
                                         object_decision.mutable_stop());
        path_decision->AddLongitudinalDecision(stop_tag, obstacle->Id(),
                                               object_decision);
        has_build_stop_decision = true;
        it->second.delay_speed_limit_times =
            speed_bounds_config_.delay_frame_times_for_limit_speed();
        --it->second.delay_stop_times;
      } else if (it->second.delay_speed_limit_times > 0UL) {
        ADEBUG << "DangerousStartUp, Need Limit Speed!";
        auto *mutable_obstacle = path_decision->Find(obstacle->Id());
        if (nullptr != mutable_obstacle) {
          mutable_obstacle->SetIsCaution(true);
        }
        --it->second.delay_speed_limit_times;
      }
    }

    if (!IsObstacleInScope(*obstacle, frenet_point, true)) {
      continue;
    }
    if (is_near_to_path) {
      ADEBUG << "enter DangerousStartUp";
      if (is_adc_start_up_ && !has_build_stop_decision) {
        ObjectDecisionType object_decision;
        const std::string stop_tag = "DangerousStartUp/near_to_path";
        util::GenerateObjectStopDecision(*obstacle, stop_distance, stop_tag,
                                         *reference_line_info_,
                                         object_decision.mutable_stop());
        path_decision->AddLongitudinalDecision(stop_tag, obstacle->Id(),
                                               object_decision);
      }
      UpdateContainerItem(*obstacle, frenet_point.l(), it);
    }
  }
}

bool SpeedBoundsDecider::GetScopeStateForVehicleStartUp(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point,
    const double left_scope, const double right_scope, const double front_scope,
    const double rear_scope) {
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();

  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  bool is_lateral_in_scope =
      !(obs_sl.start_l() > frenet_point.l() + adc_half_width + left_scope ||
        obs_sl.end_l() < frenet_point.l() - adc_half_width - right_scope);
  bool is_longitudinal_in_scope =
      obs_sl.start_s() < adc_sl.end_s() + front_scope &&
      obs_sl.end_s() > adc_sl.start_s() - rear_scope;
  bool is_in_scope = false;
  bool without_lat_overlap =
      obs_sl.start_l() > (frenet_point.l() + adc_half_width) ||
      obs_sl.end_l() < (frenet_point.l() - adc_half_width);
  if (without_lat_overlap) {
    if (is_lateral_in_scope && is_longitudinal_in_scope) {
      is_in_scope = true;
    }

    double lon_diff_dis_without_overlap = obs_sl.end_s() - adc_sl.start_s();
    double lat_diff_dis_without_overlap =
        std::max(obs_sl.start_l() - (frenet_point.l() + adc_half_width),
                 (frenet_point.l() - adc_half_width) - obs_sl.end_l());

    if (lat_diff_dis_without_overlap >
        speed_bounds_config_.aspect_range_ratio_for_start_up() *
            lon_diff_dis_without_overlap) {
      is_in_scope = false;
    }
    ADEBUG << "lat_diff_dis_without_overlap: " << lat_diff_dis_without_overlap;
    ADEBUG << "lon_diff_dis_without_overlap: " << lon_diff_dis_without_overlap;
  }

  ADEBUG << "is_lateral_in_scope: " << is_lateral_in_scope;
  ADEBUG << "is_longitudinal_in_scope: " << is_longitudinal_in_scope;
  ADEBUG << "without_lat_overlap: " << without_lat_overlap;
  ADEBUG << "is_in_scope: " << is_in_scope;
  ADEBUG << "obs l: (" << obs_sl.start_l() << ", " << obs_sl.end_l() << ")";
  ADEBUG << "obs s: (" << obs_sl.start_s() << ", " << obs_sl.end_s() << ")";
  ADEBUG << "adc s: (" << adc_sl.start_s() << ", " << adc_sl.end_s() << ")";
  return is_in_scope;
}

bool SpeedBoundsDecider::IsObstacleInScopeForVehicleStartUp(
    const Obstacle &obstacle, const common::FrenetFramePoint &frenet_point) {
  bool ret = false;

  double left_scope =
      speed_bounds_config_.scope_range_not_junction_for_start_up()
          .left_scope_dis();
  double right_scope =
      speed_bounds_config_.scope_range_not_junction_for_start_up()
          .right_scope_dis();
  double front_scope =
      speed_bounds_config_.scope_range_not_junction_for_start_up()
          .front_scope_dis();
  double rear_scope =
      speed_bounds_config_.scope_range_not_junction_for_start_up()
          .rear_scope_dis();

  ret = GetScopeStateForVehicleStartUp(obstacle, frenet_point, left_scope,
                                       right_scope, front_scope, rear_scope);
  ADEBUG << "IsObstacleInScope: " << ret;
  return ret;
}

void SpeedBoundsDecider::MakeVehicleStartUpDecision(
    PathDecision *const path_decision) {
  ADEBUG << "MakeVehicleStartUpDecision!";
  for (auto *obstacle : path_decision->obstacles().Items()) {
    bool is_vehicle = perception::PerceptionObstacle::VEHICLE ==
                      obstacle->Perception().type();
    if (!is_vehicle ||
        obstacle->speed() > speed_bounds_config_.slower_vehicle_speed()) {
      continue;
    }
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
    const auto frenet_point =
        reference_line_info_->path_data().frenet_frame_path().EvaluateByS(
            obs_s);
    const auto &path_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            obs_s - adc_sl.start_s());
    const double path_point_heading = path_point.theta();
    const std::string id = std::to_string(obstacle->PerceptionId());
    ADEBUG << "obstacle Id: " << obstacle->Id();
    auto it = vehicle_container_.find(id);
    if (IsMoveAwayFromPathByObsHeading(*obstacle, frenet_point,
                                       path_point_heading) ||
        IsMoveTowardRearAdc(*obstacle) || obstacle->GetSidepassState()) {
      if (vehicle_container_.end() != it) {
        it->second.delay_stop_times = 0UL;
      }
      continue;
    }
    const double stop_distance =
        GetStopDistanceForDangerousObs(*obstacle, path_point_heading);
    bool has_build_stop_decision = false;
    if (vehicle_container_.end() != it) {
      // if (id == "6607") {
      //   AINFO << "6607 delay_stop_times = " << it->second.delay_stop_times;
      // }
      if (is_adc_start_up_ && it->second.delay_stop_times > 0UL) {
        ADEBUG << "VehicleStartUp, Need Stop!";
        ObjectDecisionType object_decision;
        const std::string stop_tag = "VehicleStartUp/near_to_path";
        util::GenerateObjectStopDecision(*obstacle, stop_distance, stop_tag,
                                         *reference_line_info_,
                                         object_decision.mutable_stop());
        path_decision->AddLongitudinalDecision(stop_tag, obstacle->Id(),
                                               object_decision);
        has_build_stop_decision = true;
        it->second.delay_speed_limit_times =
            speed_bounds_config_.delay_frame_times_for_limit_speed();
        --it->second.delay_stop_times;
      } else if (it->second.delay_speed_limit_times > 0UL) {
        ADEBUG << "VehicleStartUp, Need Limit Speed!";
        auto *mutable_obstacle = path_decision->Find(obstacle->Id());
        if (nullptr != mutable_obstacle) {
          mutable_obstacle->SetIsCaution(true);
        }
        --it->second.delay_speed_limit_times;
      }
    }
    if (!IsObstacleInScopeForVehicleStartUp(*obstacle, frenet_point)) {
      continue;
    }
    if (obstacle->GetStartUpState() && !obstacle->GetSidepassState()) {
      AINFO << "enter VehicleStartUp, obstacle Id: " << obstacle->Id();
      if (is_adc_start_up_ && !has_build_stop_decision) {
        ObjectDecisionType object_decision;
        const std::string stop_tag = "VehicleStartUp/near_to_path";
        util::GenerateObjectStopDecision(*obstacle, stop_distance, stop_tag,
                                         *reference_line_info_,
                                         object_decision.mutable_stop());
        path_decision->AddLongitudinalDecision(stop_tag, obstacle->Id(),
                                               object_decision);
      }
      UpdateVehicleContainerItem(*obstacle, frenet_point.l(), it);
    }
  }
}

Status SpeedBoundsDecider::GetSpeedLimit(
    ReferenceLineInfo *const reference_line_info) {
  SpeedLimitDecider speed_limit_decider(
      speed_bounds_config_, reference_line_info, injector_->planning_context());
  PathDecision *const path_decision = reference_line_info->path_decision();
  bool is_first_calculation =
      reference_line_info_->speed_limit().speed_limit_points().empty();
  // Is it the first calculation speed limits.s
  if (is_first_calculation) {
    auto start_time = std::chrono::system_clock::now();
    ADEBUG << "is_first_calculation = " << is_first_calculation;
    // high speed model
    if (FLAGS_enable_high_speed) {
      if (!speed_limit_decider
               .GetHighSpeedLimits(path_decision->obstacles(),
                                   reference_line_info_->mutable_speed_limit())
               .ok()) {
        const std::string msg = "Getting speed limits failed!";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    } else {
      if (!speed_limit_decider
               .GetSpeedLimits(path_decision->obstacles(),
                               reference_line_info_->mutable_speed_limit())
               .ok()) {
        const std::string msg = "Getting speed limits failed!";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    ADEBUG << "first calculation Time = " << diff.count() * 1000 << " msec.";
  } else {
    ADEBUG << "second get speedlimits";
    auto start_time = std::chrono::system_clock::now();
    // TODO(zongxingguo): in overtake model.
    if (!util::IsOvertake(injector_->planning_context()) &&
        !util::IsLaneChange(injector_->planning_context()) &&
        FLAGS_enable_high_speed) {
      if (!speed_limit_decider
               .UpdateSpeedLimits(path_decision->obstacles(),
                                  reference_line_info_->mutable_speed_limit())
               .ok()) {
        const std::string msg = "Update speed limits failed!";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    } else {
      if (!speed_limit_decider
               .UpdateSpeedLimitsForDecision(
                   path_decision->obstacles(),
                   reference_line_info_->mutable_speed_limit())
               .ok()) {
        const std::string msg = "Update speed limits for decisio failed!";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      auto end_time = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end_time - start_time;
      ADEBUG << "second calculation Time = " << diff.count() * 1000 << " msec.";
    }
  }
  return Status::OK();
}

bool SpeedBoundsDecider::SlowBreakingForLowRoadRightLane(
    double *min_stop_s, double *min_stop_decel) {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  if (nullptr == path_decision) {
    return false;
  }
  bool is_direct_turn = false, is_slow_breaking = false,
       is_need_to_turn = false, at_left_turn = false, at_right_turn = false,
       path_turn_left = false, path_turn_right = false;
  reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn,
                                    &at_left_turn, &at_right_turn,
                                    &path_turn_left, &path_turn_right);

  if (reference_line_info_->IsAdcInCommonJunction()) {
    bool is_adc_in_high_road_right_lane_on_intersection =
        reference_line_info_->IsInHighRoadRightLaneOnIntersection(
            &at_left_turn, &is_direct_turn);
    if (is_adc_in_high_road_right_lane_on_intersection) {
      return false;
    }
  } else {
    if (!reference_line_info_->IsReachTurnLane()) {
      return false;
    }
  }

  double min_decel = std::numeric_limits<double>::max(),
         second_stop_s = std::numeric_limits<double>::max();
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();

  for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle *ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (ptr_obstacle == nullptr) {
      // AINFO << "Current obstacle pointer is null.";
      continue;
    }
    const auto &ignore_obstacles = reference_line_info_->GetIgnoregObstacles();
    if (std::find(ignore_obstacles.begin(), ignore_obstacles.end(),
                  ptr_obstacle_item->Id()) != ignore_obstacles.end()) {
      // AINFO << "skip";
      continue;
    }

    // AINFO << "ptr_obstacle_id = " << ptr_obstacle->Id();
    if (ptr_obstacle->path_st_boundary().IsEmpty()) {
      // AINFO << "NO IN ST";
      continue;
    }
    const auto &obs_sl = ptr_obstacle->PerceptionSLBoundary();
    bool is_consider_side_obs = IsBackSideObs(adc_sl, obs_sl);
    // AINFO << "is_consider_side_obs = " << is_consider_side_obs;
    bool is_same_direction = IsSameDirection(
        *ptr_obstacle, reference_line_info_->vehicle_state().heading());
    // AINFO << "is_same_direction = " << is_same_direction;
    if (is_consider_side_obs) {
      // AINFO << "obs_id = " << ptr_obstacle->Id();
      double diff_latetal_distance = std::max(
          adc_sl.start_l() - obs_sl.end_l(), obs_sl.start_l() - adc_sl.end_l());
      // obs has overlap
      double decel = std::numeric_limits<double>::max(),
             obstacle_lon_speed = 0.0, obstacle_lateral_speed = 0.0;
      GetObsLateralAndLonSpeed(*ptr_obstacle, adc_sl, &obstacle_lon_speed,
                               &obstacle_lateral_speed);

      // cross path
      bool is_left_cross_obs = (obs_sl.end_l() < adc_sl.start_l()) &&
                               (obstacle_lateral_speed >
                                speed_bounds_config_.min_cross_speed()),
           is_right_cross_obs = (obs_sl.start_l() > adc_sl.end_l()) &&
                                (obstacle_lateral_speed <
                                 -speed_bounds_config_.min_cross_speed()),
           is_face_path = (is_left_cross_obs || is_right_cross_obs);
      bool is_need_to_slow_breaking = CheckSlowBreakingForObsInForLowRoadRight(
          obstacle_lon_speed, obstacle_lateral_speed, adc_sl, obs_sl,
          is_same_direction, is_face_path, diff_latetal_distance, ptr_obstacle,
          &decel);
      if (!is_need_to_slow_breaking) {
        continue;
      }

      //  is_direct_turn ,we need to consider left and right backside obs.
      //  >2s no consider,1-2s slow_breking,<1s fast breking.
      double stop_s = ptr_obstacle->path_st_boundary().bottom_left_point().s();
      if (obs_sl.end_s() < adc_sl.end_s() && is_direct_turn &&
          ptr_obstacle->path_st_boundary().min_t() <
              speed_bounds_config_.min_collision_time() &&
          (stop_s < second_stop_s)) {
        // AINFO << "NO ERASESTBOUNDARY";

        second_stop_s = stop_s;

      } else {
        ptr_obstacle->SetSlowBreakingObstacle(true);
        ptr_obstacle->SetSlowBreakingTag("SlowBreakingForLowRoadRightLane");
        // ptr_obstacle->EraseStBoundary();
      }

      if (decel < min_decel) {
        min_decel = decel;
      }
      // AINFO << "PSUH ptr_obstacle_item->Id() = " << ptr_obstacle_item->Id();
      slow_breaking_obs_ids_.push_back(ptr_obstacle_item->Id());
      is_slow_breaking = true;
    }
  }
  *min_stop_decel = min_decel;
  *min_stop_s = second_stop_s;
  // AINFO << "min_decel = " << min_decel;
  // AINFO << "is_slow_breaking= " << is_slow_breaking;
  return is_slow_breaking;
}

bool SpeedBoundsDecider::IsBackSideObs(const planning::SLBoundary &adc_sl,
                                       const planning::SLBoundary &obs_sl) {
  if ((obs_sl.start_l() > adc_sl.end_l() && obs_sl.end_s() < adc_sl.end_s()) ||
      (obs_sl.end_l() < adc_sl.start_l() && obs_sl.end_s() < adc_sl.end_s())) {
    return true;
  } else {
    return false;
  }
}

bool SpeedBoundsDecider::SlowBreaking(double *min_stop_s,
                                      std::string *slow_breking_id) {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  if (nullptr == path_decision) {
    // AERROR << "The path_decision pointer is null.";
    return false;
  }
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  bool is_slow_breaking = false, at_left_turn = false, is_direct_turn = false;
  bool is_adc_in_high_road_right_lane_on_intersection =
      reference_line_info_->IsInHighRoadRightLaneOnIntersection(
          &at_left_turn, &is_direct_turn);
  if (!is_adc_in_high_road_right_lane_on_intersection || is_direct_turn ||
      at_left_turn) {
    // ADEBUG << "no high right or no in junction";
    return false;
  }

  double first_stop_s = std::numeric_limits<double>::max();
  double second_stop_s = std::numeric_limits<double>::max();

  for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle *ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (ptr_obstacle == nullptr) {
      // ADEBUG << "Current obstacle pointer is null.";
      continue;
    }
    const auto &slow_breking_obs_sl = ptr_obstacle->PerceptionSLBoundary();
    double stop_s = std::numeric_limits<double>::max();
    bool has_st_boundary = !ptr_obstacle->path_st_boundary().IsEmpty();
    if (!has_st_boundary) {
      // ADEBUG << " no has_st_boundary";
      continue;
    }
    bool is_same_direction = IsSameDirection(
        *ptr_obstacle, reference_line_info_->vehicle_state().heading());
    bool is_face_path = false;

    const auto &velocity = ptr_obstacle_item->Perception().velocity();
    double obs_center_s = (ptr_obstacle_item->PerceptionSLBoundary().start_s() +
                           ptr_obstacle_item->PerceptionSLBoundary().end_s()) *
                          0.5;
    const auto &ref_point =
        reference_line_info_->path_data().GetPathPointWithPathS(
            obs_center_s - (adc_sl.start_s() + adc_sl.end_s()) * 0.5);
    double ref_heading = ref_point.theta();
    const double obstacle_lateral_speed =
        common::math::Vec2d::CreateUnitVec2d(ref_heading)
            .CrossProd(Vec2d(velocity.x(), velocity.y()));

    // cross path
    bool is_left_cross_obs =
        (slow_breking_obs_sl.end_l() < adc_sl.start_l()) &&
        (obstacle_lateral_speed > speed_bounds_config_.min_cross_speed());
    bool is_right_cross_obs =
        (slow_breking_obs_sl.start_l() > adc_sl.end_l()) &&
        (obstacle_lateral_speed < -speed_bounds_config_.min_cross_speed());
    is_face_path = (is_left_cross_obs || is_right_cross_obs);
    // ADEBUG << "is_face_path = " << is_face_path;

    // ADEBUG << "is_same_direction = " << is_same_direction;

    bool is_lon_overlap = (slow_breking_obs_sl.end_s() > adc_sl.start_s() &&
                           slow_breking_obs_sl.end_s() < adc_sl.end_s());
    // ADEBUG << "is_lon_overlap = " << is_lon_overlap;

    if ((is_same_direction || is_face_path) && is_lon_overlap) {
      // ADEBUG << "obs_id = " << ptr_obstacle->Id();
      stop_s = ptr_obstacle->path_st_boundary().bottom_left_point().s();
      // ADEBUG << "stop_s = " << stop_s;
      if (stop_s < first_stop_s) {
        first_stop_s = stop_s;
        *slow_breking_id = ptr_obstacle_item->Id();
      }
      double diff_latetal_distance =
          std::max(adc_sl.start_l() - slow_breking_obs_sl.end_l(),
                   slow_breking_obs_sl.start_l() - adc_sl.end_l());
      // if lateral distance is too close,we need to use stop now,use QP;
      // but if obs lateral distance close and cutin trajectory too long in
      // longitutal,have risk;
      // TODO(zongxingguo): no use QP?
      if (diff_latetal_distance < speed_bounds_config_.min_lateral_distance()) {
        // ADEBUG << "too close ,need stop.";
        return false;
      } else {
        // second check collision only for has decision,so no create again.
        ptr_obstacle->SetSlowBreakingObstacle(true);
        ptr_obstacle->SetSlowBreakingTag("SlowBreakingForHighRoadRightLane");
        // ptr_obstacle->EraseStBoundary();
        // AINFO << "PSUH ptr_obstacle_item->Id() = " <<
        // ptr_obstacle_item->Id();
        slow_breaking_obs_ids_.push_back(ptr_obstacle_item->Id());
        is_slow_breaking = true;
      }
    } else {
      // get no broadside obs (front obs) 's collision point;
      stop_s = ptr_obstacle->path_st_boundary().bottom_left_point().s();
      if (stop_s < second_stop_s) {
        second_stop_s = stop_s;
      }
    }
  }
  // ADEBUG << "first_stop_s = " << first_stop_s;
  // ADEBUG << "second_stop_s = " << second_stop_s;

  *min_stop_s = second_stop_s;
  return is_slow_breaking;
}

bool SpeedBoundsDecider::IsMoveAwayFromSegmentPathByObsHeading(
    const Obstacle &obstacle, const PathData &path_data) {
  // HysteresisInterval is used to determine if the obstacle is away from
  // the path by its heading.
  static HysteresisInterval away_path_obs_interval(
      speed_bounds_config_.obs_cross_angle_degree_for_away_obs(),
      speed_bounds_config_.hy_buffer_of_cross_angle_for_away_obs(), 50UL);
  // Check if the obstacle is in the path.
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  if (obs_sl.start_l() < adc_sl.end_l() && obs_sl.end_l() > adc_sl.start_l()) {
    // The obstacle is in the path.
    return false;
  }
  // Calculate the heading difference between the obstacle and the path.
  double min_diff_heading = 0.0;
  double obs_heading = obstacle.SpeedHeading();
  if (obstacle.speed() > FLAGS_min_dynamic_obstacle_speed) {
    // Check if the obstacle is on the left side or right side of the path.
    bool is_left_obs = 0.5 * (obs_sl.start_l() + obs_sl.end_l()) >
                       0.5 * (adc_sl.start_l() + adc_sl.end_l());
    double sign_obs_path_l = is_left_obs ? 1.0 : -1.0;
    const double adc_speed =
        injector_->vehicle_state()->vehicle_state().linear_velocity();
    const double presight_lon_dis =
        std::max(adc_speed * speed_bounds_config_.presight_time_for_move_away(),
                 speed_bounds_config_.min_presight_lon_dis_for_move_away());
    min_diff_heading = std::numeric_limits<double>::max();
    for (const auto &path_point : path_data.discretized_path()) {
      if (path_point.s() > presight_lon_dis) {
        break;
      }
      double diff_heading =
          std::fabs(common::math::AngleDiff(
              path_point.theta(), obs_heading + M_PI_2 * sign_obs_path_l)) *
          kDegrees / M_PI_2;
      min_diff_heading = std::min(min_diff_heading, diff_heading);
      ADEBUG << "diff_heading ( " << path_point.s() << "): " << diff_heading;
    }
    if (std::numeric_limits<double>::max() == min_diff_heading) {
      min_diff_heading = 0.0;
    }
  }

  // Determine if the obstacle is away from the path by its heading.
  bool obs_away_from_path =
      away_path_obs_interval.HyValue(obstacle, min_diff_heading) >
      speed_bounds_config_.obs_cross_angle_degree_for_away_obs();
  ADEBUG << "min_diff_heading = " << min_diff_heading
         << ", path_point_heading = "
         << path_data.discretized_path().front().theta() * kDegrees / M_PI_2
         << ", obs_heading = " << obs_heading * kDegrees / M_PI_2
         << ", obs_away_from_path = " << obs_away_from_path;
  return obs_away_from_path;
}

bool SpeedBoundsDecider::IsSameDirection(const Obstacle &obstacle,
                                         double adc_theta) const {
  static HysteresisInterval adc_obstacles_heading_interval(
      speed_bounds_config_.max_diff_angle_for_same_orientation(),
      speed_bounds_config_.hy_buffer_lower_for_same_orientation(),
      speed_bounds_config_.hy_buffer_upper_for_same_orientation(), kHyCapacity);
  std::string hy_obs_id = std::to_string(obstacle.PerceptionId());
  ADEBUG << "bstacle->SpeedHeading() = " << obstacle.SpeedHeading();
  const double diff_heading =
      common::math::NormalizeAngle(obstacle.SpeedHeading() - adc_theta) *
      kDegrees / M_PI_2;
  double hy_diff_heading = diff_heading;
  if (diff_heading > 0.0) {
    hy_obs_id += "_e";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, diff_heading);
  } else {
    hy_obs_id += "_f";
    hy_diff_heading =
        adc_obstacles_heading_interval.HyValue(hy_obs_id, -diff_heading);
  }
  // hy_diff_heading is not negative value.
  ADEBUG << "hy_obs_id: " << hy_obs_id << ", diff_heading: " << diff_heading
         << " degree, hy_diff_heading: " << hy_diff_heading << " degree";
  const bool is_same_orientation =
      hy_diff_heading <
      speed_bounds_config_.max_diff_angle_for_same_orientation();

  ADEBUG << "is_same_orientation = " << is_same_orientation;
  return is_same_orientation;
}

double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();

  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

bool SpeedBoundsDecider::CheckSlowBreakingForObsInForLowRoadRight(
    double obstacle_lon_speed, double obstacle_lateral_speed,
    const planning::SLBoundary &adc_sl, const planning::SLBoundary &obs_sl,
    bool is_same_direction, bool is_face_path,
    const double diff_latetal_distance, Obstacle *ptr_obstacle, double *decel) {
  bool is_direct_turn = false, is_need_to_turn = false, at_left_turn = false,
       at_right_turn = false, path_turn_left = false, path_turn_right = false;
  reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn,
                                    &at_left_turn, &at_right_turn,
                                    &path_turn_left, &path_turn_right);
  bool is_in_commonjunction = reference_line_info_->IsAdcInCommonJunction();
  double adc_v = injector_->vehicle_state()->linear_velocity();

  // AINFO << "is_direct_turn = " << is_direct_turn;
  if (IsOverlapDangerousObs(adc_sl, obs_sl, is_same_direction, is_face_path)) {
    GetSlowBreakingAccBasedLateralDistanceForLowRoadRight(diff_latetal_distance,
                                                          decel);
    // obs in back or side ,fully oriented towards adc.
    if (obstacle_lon_speed < 0.5 * adc_v &&
        std::fabs(obstacle_lateral_speed) > 2 * std::fabs(obstacle_lon_speed)) {
      // AINFO << "no slow breaking for side or back crossobs";
      *decel = 0.0;
    }
    if (!is_direct_turn && is_in_commonjunction && is_same_direction) {
      if (at_left_turn &&
          obs_sl.end_l() <
              adc_sl.start_l() - speed_bounds_config_.lateral_monitor_range()) {
        // AINFO << "in junction left merage area, skip right back obs which has
        // "
        //          "large lateral distance in same direction.";
        *decel = std::numeric_limits<double>::max();
        ptr_obstacle->SetSlowBreakingObstacle(true);
        ptr_obstacle->SetSlowBreakingTag(
            "CheckSlowBreakingForObsInForLowRoadRight");
        // ptr_obstacle->EraseStBoundary();
        return false;
      }
      if (at_right_turn &&
          obs_sl.start_l() >
              adc_sl.end_l() + speed_bounds_config_.lateral_monitor_range()) {
        // AINFO << "in junction right merage area, skip left back obs which has
        // "
        //          "large lateral distance in same direction.";
        *decel = std::numeric_limits<double>::max();
        ptr_obstacle->SetSlowBreakingObstacle(true);
        ptr_obstacle->SetSlowBreakingTag(
            "CheckSlowBreakingForObsInForLowRoadRight");
        // ptr_obstacle->EraseStBoundary();
        return false;
      }
    }
  } else if (IsOverlapAndAwayFromPath(is_same_direction, is_face_path, obs_sl,
                                      adc_sl)) {
    // longitude overlap no face to path,need to skip,so EraseStBoundary.
  } else if (obs_sl.end_s() < adc_sl.start_s()) {
    // back obs
    // ADEBUG << "back obs ";
    *decel = speed_bounds_config_.first_level_decel();
    // in junction left merage area, skip right back obs.
    if (!IsSlowBreakingForBackObs(is_direct_turn, is_in_commonjunction,
                                  at_left_turn, at_right_turn, obs_sl, adc_sl,
                                  ptr_obstacle, decel)) {
      return false;
    }

    // AINFO << "is_in_commonjunction " << is_in_commonjunction;
    // AINFO << "is_same_direction = " << is_same_direction;
    // AINFO << "at_left_turn = " << at_left_turn;
    // AINFO << "!is_face_path = " << !is_face_path;
    // no slow breaking
    bool is_right_obs = adc_sl.start_l() - obs_sl.end_l() > 0.0;
    if (!IsSlowBreakingObs(is_in_commonjunction, is_same_direction,
                           at_left_turn, is_face_path, is_right_obs)) {
      return false;
    }
  } else {
    // obs in front adc
    // AINFO << "in front adc no slow breaking";
    return false;
  }
  return true;
}

bool SpeedBoundsDecider::IsSlowBreakingForBackObs(
    const bool is_direct_turn, const bool is_in_commonjunction,
    const bool at_left_turn, const bool at_right_turn,
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl,
    Obstacle *ptr_obstacle, double *decel) {
  if (!is_direct_turn && is_in_commonjunction) {
    if (at_left_turn && obs_sl.end_l() < adc_sl.start_l()) {
      // AINFO << "in junction left merage area, skip right back obs.";
      ptr_obstacle->SetSlowBreakingTag(
          "in junction left merage area, skip right back obs");
      return false;
    }
    if (at_right_turn && obs_sl.start_l() > adc_sl.end_l()) {
      // AINFO << "in junction right merage area, skip left back obs.";
      ptr_obstacle->SetSlowBreakingTag(
          "in junction right merage area, skip left back obs");
      return false;
    }
  }
  if (is_near_merge_area_ && !is_in_commonjunction) {
    ptr_obstacle->SetSlowBreakingTag("no slow breaking for back merage obs");
    // AINFO << "no slow breaking for back merage obs";
    return false;
  }
  return true;
}

bool SpeedBoundsDecider::IsSlowBreakingObs(bool is_in_commonjunction,
                                           bool is_same_direction,
                                           bool at_left_turn, bool is_face_path,
                                           bool is_right_obs) {
  if (is_in_commonjunction && is_same_direction && at_left_turn &&
      !is_face_path && is_right_obs) {
    return false;
  }
  return true;
}

bool SpeedBoundsDecider::IsOverlapAndAwayFromPath(
    bool is_same_direction, bool is_face_path,
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl) {
  if ((obs_sl.end_s() > adc_sl.start_s() && obs_sl.end_s() < adc_sl.end_s()) &&
      !(is_same_direction || is_face_path)) {
    return true;
  } else {
    return false;
  }
}

bool SpeedBoundsDecider::IsOverlapDangerousObs(
    const planning::SLBoundary &adc_sl, const planning::SLBoundary &obs_sl,
    bool is_same_direction, bool is_face_path) {
  if ((obs_sl.end_s() > adc_sl.start_s() && obs_sl.end_s() < adc_sl.end_s()) &&
      (is_same_direction || is_face_path)) {
    return true;
  } else {
    return false;
  }
}

void SpeedBoundsDecider::GetObsLateralAndLonSpeed(
    const Obstacle &obstacle, const planning::SLBoundary &adc_sl,
    double *obstacle_lon_speed, double *obstacle_lateral_speed) {
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  const auto &velocity = obstacle.Perception().velocity();
  double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const auto &ref_point =
      reference_line_info_->path_data().GetPathPointWithPathS(
          obs_center_s - (adc_sl.start_s() + adc_sl.end_s()) * 0.5);
  double ref_heading = ref_point.theta();
  *obstacle_lon_speed = common::math::Vec2d::CreateUnitVec2d(ref_heading)
                            .InnerProd(Vec2d(velocity.x(), velocity.y()));
  // AINFO << "obstacle_lon_speed = " << obstacle_lon_speed;
  *obstacle_lateral_speed = common::math::Vec2d::CreateUnitVec2d(ref_heading)
                                .CrossProd(Vec2d(velocity.x(), velocity.y()));
}

void SpeedBoundsDecider::GetSlowBreakingAccBasedLateralDistanceForLowRoadRight(
    const double diff_latetal_distance, double *decel) {
  if (diff_latetal_distance < speed_bounds_config_.min_lateral_distance()) {
    *decel = speed_bounds_config_.four_level_decel();
  } else if (diff_latetal_distance >=
                 speed_bounds_config_.min_lateral_distance() &&
             diff_latetal_distance <
                 speed_bounds_config_.second_lateral_distance()) {
    *decel = speed_bounds_config_.three_level_decel();
  } else if (diff_latetal_distance >=
                 speed_bounds_config_.second_lateral_distance() &&
             diff_latetal_distance <
                 speed_bounds_config_.max_lateral_distance()) {
    *decel = InterpolationLookUp(diff_latetal_distance,
                                 speed_bounds_config_.second_lateral_distance(),
                                 speed_bounds_config_.max_lateral_distance(),
                                 speed_bounds_config_.second_level_decel(),
                                 speed_bounds_config_.first_level_decel());
    // AINFO << "*decel = " << *decel;
  } else {
    // lateral_distance > 2m
    *decel = std::min(speed_bounds_config_.first_level_decel() /
                          (diff_latetal_distance /
                           speed_bounds_config_.max_lateral_distance()),
                      speed_bounds_config_.first_level_decel());
  }
}

void SpeedBoundsDecider::GetSlowBreakingAccBasedSpeedDiff(
    bool is_front_obs, double obstacle_lon_speed, double adc_speed,
    double *decel_temp) {
  *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
  if (obstacle_lon_speed >= kMinSpeedCoeff * adc_speed &&
      obstacle_lon_speed < kMaxSpeedCoeff * adc_speed) {
    *decel_temp = speed_bounds_config_.second_level_slow_breking_decel();
  }
  if (obstacle_lon_speed >= kMaxSpeedCoeff * adc_speed) {
    *decel_temp = speed_bounds_config_.first_level_slow_breking_decel();
  }
  // fast obs all front adc ,no low decel.
  if (is_front_obs) {
    *decel_temp = speed_bounds_config_.first_level_slow_breking_decel();
  }
  // AINFO << "obs high speed,DECEL  = " << *decel_temp;
  // polygon and real vehicle no match，so make collision to stop.
  //  const auto &obs_polygon = mutable_obstacle->PerceptionPolygon();
  //  if (CheckObsOverlap(reference_line_info_->path_data()
  //                          .discretized_path()
  //                          .front(),
  //                      obs_sl, obs_polygon)) {
  //    AINFO << "init_collision";
  //    decel_temp =
  //    speed_bounds_config_.min_level_slow_breking_decel();
  //  }
}

void SpeedBoundsDecider::GetSlowBreakingAccBasedSpeedDiffNew(
    const Obstacle &obstacle, bool is_front_obs, double obstacle_lon_speed,
    double adc_speed, double *decel_temp) {
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  double obstacle_lon_acc = obstacle.LongitudianlAcc();
  // AINFO << "id_" << obstacle.Id()
  //       << "  obstacle_lon_acc = " << obstacle_lon_acc;
  // adc got straight ,so noconsider back obs.
  if (obstacle_lon_speed >= kMinSpeedCoeff * adc_speed &&
      obstacle_lon_speed < kMaxSpeedCoeff * adc_speed) {
    *decel_temp = speed_bounds_config_.second_level_cutin_decel();
  }
  if (obstacle_lon_speed >= kMaxSpeedCoeff * adc_speed) {
    *decel_temp = speed_bounds_config_.first_level_cutin_decel();
  }

  double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                 obs_sl.start_l() - adc_sl.end_l());
  bool is_front_side_obs =
      obs_sl.end_s() > adc_sl.end_s() && obs_sl.start_s() <= adc_sl.end_s();
  double speed_diff = std::fabs(obstacle_lon_speed - adc_speed);
  double obs_lon_acc = obstacle.LongitudianlAcc();
  if (is_front_obs) {
    // AINFO << "is_front_obs = " << is_front_obs;
    GetSlowBreakingDecelForFrontObs(
        obstacle, obs_sl, adc_sl, obstacle_lon_speed, lateral_diff, decel_temp);
    *decel_temp = std::min(obstacle_lon_acc, *decel_temp);
  } else if (is_front_side_obs) {
    // obs in front side,we need to check has collision risk. if obs is too long
    // AINFO << "is_front_side_obs = " << is_front_side_obs;
    // AINFO << "obs.acc = " << obstacle.acceleration();
    // AINFO << "lateral_diff = " << lateral_diff;
    bool is_lateral_overlap = lateral_diff < 0.0;
    // AINFO << "is_lateral_overlap = " << is_lateral_overlap;
    if (is_lateral_overlap) {
      double overlap_lon_distance =
          adc_sl.end_s() - obs_sl.start_s() + kLonBuffer;
      // delta_lon_speed no lowst
      double delta_lon_speed =
          std::max(obstacle_lon_speed - adc_speed, kLonSpeedDiff);
      double delta_overtake_t = overlap_lon_distance / delta_lon_speed;
      // AINFO << "delta_lon_speed = " << delta_lon_speed;
      // AINFO << "overlap_lon_distance = " << overlap_lon_distance;
      // AINFO << "delta_overtake_t = " << delta_overtake_t;
      if (delta_overtake_t > speed_bounds_config_.cutin_comfort_time()) {
        *decel_temp = speed_bounds_config_.min_level_cutin_decel();
      } else {
        *decel_temp = InterpolationLookUp(
            delta_overtake_t, 0.0, speed_bounds_config_.cutin_comfort_time(),
            speed_bounds_config_.first_level_cutin_decel(),
            speed_bounds_config_.min_level_cutin_decel());
      }
      // AINFO << "*decel_temp = " << *decel_temp;
    }

    // front side obs，obs breaking ，diff_speed<3.0
    if (is_lateral_overlap &&
        obs_lon_acc < speed_bounds_config_.obs_breaking_decel() &&
        speed_diff < speed_bounds_config_.min_speed_diff()) {
      // AINFO << "obs cutin adc obs breaking";
      double low_decel = std::min(
          InterpolationLookUp(
              speed_diff, 0.0, kMaxSpeedDiff,
              speed_bounds_config_.five_level_apporaching_decel(), 0.0),
          std::trunc(obs_lon_acc));
      *decel_temp = std::min(*decel_temp, low_decel);
    }
  }

  // AINFO << "obs high speed,DECEL  = " << *decel_temp;
  // polygon and real vehicle no match，so make collision to stop.
  //  const auto &obs_polygon = mutable_obstacle->PerceptionPolygon();
  //  if (CheckObsOverlap(reference_line_info_->path_data()
  //                          .discretized_path()
  //                          .front(),
  //                      obs_sl, obs_polygon)) {
  //    AINFO << "init_collision";
  //    decel_temp =
  //    speed_bounds_config_.min_level_slow_breking_decel();
  //  }
}

void SpeedBoundsDecider::
    GetSlowBreakingAccBasedLateralDistanceForApproachingObs(
        const planning::SLBoundary &adc_sl, const planning::SLBoundary &obs_sl,
        double obstacle_lon_speed, double adc_speed, double *decel_temp) {
  double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                 obs_sl.start_l() - adc_sl.end_l());
  // AINFO << "lateral_diff = " << lateral_diff;
  if (obs_sl.end_s() - adc_sl.start_s() <= 0.0) {
    // only in low right lane(merage or turn)
    // AINFO << "obs in back side";
    if (obstacle_lon_speed > 0.5 * adc_speed) {
      *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
    } else {
      // can no slow breaking
      *decel_temp = 0.0;
    }

    // AINFO << "decel_temp = " << decel_temp;
  } else if (obs_sl.end_s() - adc_sl.start_s() > 0.0 &&
             obs_sl.start_s() - adc_sl.end_s() <= 0.0) {
    // TODO(zhonghao): add hysteresis.
    // AINFO << "obs with adc has lon overlap";
    if (lateral_diff <= 0.0) {
      // AINFO << "lon and lat has overlap,in equal speed ,-4.";
      *decel_temp = speed_bounds_config_.five_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff < speed_bounds_config_.first_lateral_distance() &&
               lateral_diff > 0.0) {
      // lateral_distance:0.0-0.3 decel:-6.0
      *decel_temp = speed_bounds_config_.min_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff >= speed_bounds_config_.first_lateral_distance() &&
               lateral_diff <= speed_bounds_config_
                                   .second_lateral_distance_for_approaching()) {
      // lateral_distance:0.3-0.6 decel:-4.0
      *decel_temp = speed_bounds_config_.five_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff >= speed_bounds_config_
                                   .second_lateral_distance_for_approaching() &&
               lateral_diff <= speed_bounds_config_.three_lateral_distance()) {
      // lateral_distance:0.6-1.0 decel:-2.0
      *decel_temp = speed_bounds_config_.four_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff > speed_bounds_config_.three_lateral_distance()) {
      // AINFO << "lateral_diff = " << lateral_diff;
      // lateral_distance:1.0-4.0 decel:-1.5 ----> -0.5
      *decel_temp = InterpolationLookUp(
          lateral_diff, speed_bounds_config_.three_lateral_distance(),
          speed_bounds_config_.consider_lateral_distance(),
          speed_bounds_config_.second_level_decel(),
          speed_bounds_config_.first_level_decel());
      // *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
      // AINFO << "decel_temp = " << *decel_temp;
    }
  } else {
    GetSlowBreakingDecel(obs_sl, adc_sl, obstacle_lon_speed, lateral_diff,
                         decel_temp);
  }
}

void SpeedBoundsDecider::
    GetSlowBreakingAccBasedLateralDistanceForApproachingObsNew(
        const Obstacle &obstacle, const planning::SLBoundary &adc_sl,
        const planning::SLBoundary &obs_sl, double obstacle_lon_speed,
        double adc_speed, double *decel_temp) {
  double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                 obs_sl.start_l() - adc_sl.end_l());
  double obstacle_lat_speed = obstacle.LateralSpeed();
  // AINFO << "obstacle_lat_speed = " << obstacle_lat_speed;
  // AINFO << "lateral_diff = " << lateral_diff;

  // back side obs
  if (obs_sl.end_s() - adc_sl.start_s() <= 0.0) {
    // AINFO << "============obs back  adc only consider in turn lane.";
    // only in low right lane(merage or turn)
    if (obstacle_lon_speed > 0.5 * adc_speed) {
      *decel_temp = speed_bounds_config_.three_level_apporaching_decel();
    } else {
      // can no slow breaking for back low speed obs
      *decel_temp = 0.0;
    }
  } else if (obs_sl.end_s() > adc_sl.start_s() &&
             obs_sl.start_s() <= adc_sl.end_s()) {
    // AINFO << "obs with adc has lon overlap";

    if (lateral_diff < speed_bounds_config_.first_lateral_distance() &&
        lateral_diff > 0.0) {
      // lateral_distance:0.0-0.3 decel:-6.0
      *decel_temp = speed_bounds_config_.min_level_apporaching_decel();
      // AINFO << "decel_temp = " << *decel_temp;
    } else if (lateral_diff >= speed_bounds_config_.first_lateral_distance() &&
               lateral_diff <= speed_bounds_config_
                                   .second_lateral_distance_for_approaching()) {
      // lateral_distance:0.3-0.6 decel:-4.0
      *decel_temp = speed_bounds_config_.five_level_apporaching_decel();
      // AINFO << "decel_temp = " << *decel_temp;
    } else if (lateral_diff >= speed_bounds_config_
                                   .second_lateral_distance_for_approaching() &&
               lateral_diff <= speed_bounds_config_.three_lateral_distance()) {
      // lateral_distance:0.6-1.0 decel:-2.0
      *decel_temp = speed_bounds_config_.four_level_apporaching_decel();
      // AINFO << "decel_temp = " << *decel_temp;
    } else if (lateral_diff > speed_bounds_config_.three_lateral_distance()) {
      // AINFO << "lateral_diff = " << lateral_diff;
      // lateral_distance:1.0-4.0 decel:-1.5 ----> -0.5
      *decel_temp = InterpolationLookUp(
          lateral_diff, speed_bounds_config_.three_lateral_distance(),
          speed_bounds_config_.consider_lateral_distance(),
          speed_bounds_config_.second_level_apporaching_decel(),
          speed_bounds_config_.first_level_apporaching_decel());
      // *decel_temp =
      // speed_bounds_config_.three_level_slow_breking_decel();
      // AINFO << "decel_temp = " << *decel_temp;
    }

    if (std::fabs(obstacle_lat_speed) > kMinLateralSpeed &&
        (obs_sl.end_s() > adc_sl.end_s() &&
         obs_sl.start_s() <= adc_sl.end_s())) {
      // AINFO << "============obs front side  adc";
      if (lateral_diff > 0.0 && adc_speed > obstacle_lon_speed) {
        double collision_time = lateral_diff / std::fabs(obstacle_lat_speed);
        // AINFO << "collision_time = " << collision_time;
        double decel_according_time =
            -(adc_speed - obstacle_lon_speed) / collision_time;

        // AINFO << "according time decel_temp = " << decel_according_time;
        *decel_temp = std::min(decel_according_time, *decel_temp);
      }
      // front side obs，obs breaking ，diff_speed<3.0
      double speed_diff = std::fabs(obstacle_lon_speed - adc_speed);
      double obs_lon_acc = obstacle.LongitudianlAcc();
      if (lateral_diff <= 0.0 &&
          obs_lon_acc < speed_bounds_config_.obs_breaking_decel() &&
          speed_diff < speed_bounds_config_.min_speed_diff()) {
        double low_decel = std::min(
            InterpolationLookUp(
                speed_diff, 0.0, kMaxSpeedDiff,
                speed_bounds_config_.five_level_apporaching_decel(), 0.0),
            std::trunc(obs_lon_acc));
        *decel_temp = std::min(*decel_temp, low_decel);
      }
    }

  } else {
    // AINFO << "============obs in front adc";
    GetSlowBreakingDecelForFrontObs(
        obstacle, obs_sl, adc_sl, obstacle_lon_speed, lateral_diff, decel_temp);
  }
}

void SpeedBoundsDecider::GetSlowBreakingDecel(
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl,
    double obstacle_lon_speed, double lateral_diff, double *decel_temp) {
  // AINFO << "obs all front adc";
  double lon_diff = obs_sl.start_s() - adc_sl.end_s();
  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  if (obstacle_lon_speed > kMinSpeedCoeff * adc_speed) {
    *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
    if (obstacle_lon_speed >= kMinSpeedCoeff * adc_speed &&
        obstacle_lon_speed < kMaxSpeedCoeff * adc_speed) {
      *decel_temp = speed_bounds_config_.second_level_slow_breking_decel();
    }
    if (obstacle_lon_speed >= kMaxSpeedCoeff * adc_speed) {
      *decel_temp = speed_bounds_config_.first_level_slow_breking_decel();
    }
  } else {
    if (lateral_diff <= 0.0) {
      // because front obs ,so lon_diff > 0.0
      double breaking_distance = lon_diff - kLonBuffer;
      if (breaking_distance > 0.0) {
        *decel_temp =
            adc_speed > obstacle_lon_speed
                ? -(obstacle_lon_speed - adc_speed) *
                      (obstacle_lon_speed - adc_speed) * 0.5 /
                      (breaking_distance)
                : speed_bounds_config_.three_level_slow_breking_decel();

        // AINFO << "decel_temp = " << *decel_temp;

      } else {
        *decel_temp = speed_bounds_config_.min_level_slow_breking_decel();
      }

    } else if (lateral_diff < speed_bounds_config_.first_lateral_distance() &&
               lateral_diff > 0.0) {
      *decel_temp = speed_bounds_config_.min_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff >= speed_bounds_config_.first_lateral_distance() &&
               lateral_diff <= speed_bounds_config_.three_lateral_distance()) {
      *decel_temp = speed_bounds_config_.four_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    } else if (lateral_diff > speed_bounds_config_.three_lateral_distance()) {
      *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
      // AINFO << "decel_temp = " << decel_temp;
    }
  }
}

void SpeedBoundsDecider::GetSlowBreakingDecelForFrontObs(
    const Obstacle &obstacle, const planning::SLBoundary &obs_sl,
    const planning::SLBoundary &adc_sl, double obstacle_lon_speed,
    double lateral_diff, double *decel_temp) {
  // AINFO << "obs all front adc";
  double lon_diff = obs_sl.start_s() - adc_sl.end_s();
  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  double obstacle_lat_speed = obstacle.LateralSpeed();
  double speed_diff = std::fabs(obstacle_lon_speed - adc_speed);
  // AINFO << "obstacle_lat_speed = " << obstacle_lat_speed;
  // AINFO << "obstacle_lon_speed = " << obstacle_lon_speed
  //       << " adc_speed = " << adc_speed;
  double obs_lon_acc = obstacle.LongitudianlAcc();
  // AINFO << "obs_lon_acc = " << obs_lon_acc;
  // obs_lon_v > adc(1-1.2;1.2-2;>2)
  if (obstacle_lon_speed >= adc_speed) {
    if (obstacle_lon_speed < kSecondSpeedCoeff * adc_speed) {
      *decel_temp = speed_bounds_config_.three_level_apporaching_decel();
    } else if (obstacle_lon_speed >= kSecondSpeedCoeff * adc_speed &&
               obstacle_lon_speed < kMaxSpeedCoeff * adc_speed) {
      *decel_temp = speed_bounds_config_.second_level_apporaching_decel();
    } else {
      *decel_temp = speed_bounds_config_.first_level_apporaching_decel();
    }
    if (lon_diff < kLonStopBuffer) {
      // AINFO << "lon_diff < kLonStopBuffer";
      // AINFO << "lateral_diff = " << lateral_diff;
      if (lateral_diff <= 0.0 &&
          obs_lon_acc < speed_bounds_config_.obs_breaking_decel()) {
        double low_decel = std::min(
            InterpolationLookUp(
                speed_diff, 0.0, kMaxSpeedDiff,
                speed_bounds_config_.five_level_apporaching_decel(), 0.0),
            std::trunc(obs_lon_acc));
        *decel_temp = std::min(*decel_temp, low_decel);
      }
    }
    // AINFO << "*decel_temp = " << *decel_temp;
  } else {
    // front, has lateral overlap ,adc_v > obs_v,get decel according delta_v and
    // lon_distance.
    if (lateral_diff <= 0.0) {
      // because front obs ,so lon_diff > 0.0
      double breaking_distance = lon_diff - kLonStopBuffer;
      // AINFO << "lon_diff = " << lon_diff;
      // AINFO << "breaking_distance = " << breaking_distance;
      if (breaking_distance > 0.0) {
        *decel_temp = -(adc_speed - obstacle_lon_speed) *
                      (adc_speed - obstacle_lon_speed) * 0.5 /
                      (breaking_distance);
        // front obs ,obs breaking
        // AINFO << "according speed *decel_temp = " << *decel_temp;
        if (obs_lon_acc < speed_bounds_config_.obs_breaking_decel()) {
          double obs_breaking_distance =
              std::fabs(obstacle_lon_speed * obstacle_lon_speed * 0.5 /
                        std::trunc(obs_lon_acc));
          // AINFO << "breaking_distance = " << breaking_distance;
          double adc_breaking_distance =
              obs_breaking_distance + breaking_distance;
          // AINFO << "adc_breaking_distance = " << adc_breaking_distance;
          if (adc_breaking_distance > 0.0) {
            *decel_temp =
                -adc_speed * adc_speed * 0.5 / (adc_breaking_distance);
          } else {
            *decel_temp = speed_bounds_config_.min_level_apporaching_decel();
          }
        }
      } else {
        *decel_temp = speed_bounds_config_.min_level_apporaching_decel();
      }
      // AINFO << "decel_temp = " << *decel_temp;
    } else {
      // obs_lon_v < adc_v ,front obs ,has no lateral overlap.
      if (std::fabs(obstacle_lat_speed) > kMinLateralSpeed) {
        // AINFO << "has  lateral speed";
        double collision_time = lateral_diff / std::fabs(obstacle_lat_speed);
        // AINFO << "collision_time = " << collision_time;
        double decel_according_time =
            -(adc_speed - obstacle_lon_speed) / collision_time;

        // AINFO << "according time decel_temp = " << decel_according_time;
        *decel_temp = std::min(decel_according_time, *decel_temp);
      } else {
        // AINFO << "has no lateral speed";
        if (lateral_diff < speed_bounds_config_.first_lateral_distance() &&
            lateral_diff > 0.0) {
          *decel_temp = speed_bounds_config_.min_level_apporaching_decel();
          // AINFO << "decel_temp = " << decel_temp;
        } else if (lateral_diff >=
                       speed_bounds_config_.first_lateral_distance() &&
                   lateral_diff <=
                       speed_bounds_config_.three_lateral_distance()) {
          *decel_temp = speed_bounds_config_.four_level_apporaching_decel();
          // AINFO << "decel_temp = " << decel_temp;
        } else {
          *decel_temp = speed_bounds_config_.three_level_slow_breking_decel();
          // AINFO << "decel_temp = " << decel_temp;
        }
      }
      // AINFO << "*decel_temp = " << *decel_temp;
    }
  }
}

bool SpeedBoundsDecider::IsInLongitudinalRange(
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl) {
  bool in_longitudinal_range =
      obs_sl.end_s() >
          adc_sl.start_s() - speed_bounds_config_.lon_back_monitor_range() &&
      obs_sl.start_s() <
          adc_sl.end_s() + speed_bounds_config_.lon_front_monitor_range();
  if (is_near_merge_area_ && !reference_line_info_->IsAdcInCommonJunction()) {
    in_longitudinal_range =
        obs_sl.end_s() > adc_sl.start_s() &&
        obs_sl.start_s() <
            adc_sl.end_s() + speed_bounds_config_.lon_front_monitor_range();
  }
  return in_longitudinal_range;
}

void SpeedBoundsDecider::GetMonitorObstacles(
    std::vector<std::string> *range_obs_ids) {
  bool is_direct_turn = false, is_need_to_turn = false, at_left_turn = false,
       at_right_turn = false, path_turn_left = false, path_turn_right = false;
  reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn,
                                    &at_left_turn, &at_right_turn,
                                    &path_turn_left, &path_turn_right);
  PathDecision *const path_decision = reference_line_info_->path_decision();
  if (nullptr == path_decision) {
    AERROR << "The path_decision pointer is null.";
    return;
  }
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle *ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    if (ptr_obstacle == nullptr) {
      // AINFO << "Current obstacle pointer is null.";
      continue;
    }
    if (ptr_obstacle->IsStatic()) {
      continue;
    }
    if (PerceptionObstacle::PEDESTRIAN == ptr_obstacle->Perception().type()) {
      continue;
    }
    // AINFO << "ptr_obstacle_item->Id() = " << ptr_obstacle_item->Id();

    bool is_unknown = perception::PerceptionObstacle::UNKNOWN ==
                          ptr_obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
                          ptr_obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
                          ptr_obstacle->Perception().type();
    if (is_unknown && ptr_obstacle->path_st_boundary().IsEmpty()) {
      // AINFO << "No slow breaking for no unknown no in st";
      continue;
    }
    const auto &obs_sl = ptr_obstacle->PerceptionSLBoundary();
    bool is_in_monitor_range = false;

    double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                   obs_sl.start_l() - adc_sl.end_l());
    bool is_within_lateral_range =
        lateral_diff > 0.0 &&
        lateral_diff <
            speed_bounds_config_.lateral_monitor_range_for_approaching_obs();

    bool is_lateral_overlap = lateral_diff < 0.0;
    bool in_front_range = obs_sl.end_s() > adc_sl.end_s();

    bool in_lateral_range =
        is_within_lateral_range || (is_lateral_overlap && in_front_range);
    bool in_longitudinal_range = IsInLongitudinalRange(obs_sl, adc_sl);
    if (in_lateral_range && in_longitudinal_range) {
      is_in_monitor_range = true;
    }
    // AINFO << "is_in_monitor_range = " << is_in_monitor_range;
    if (!is_in_monitor_range) {
      continue;
    }

    //
    bool has_st_boundary = !ptr_obstacle->path_st_boundary().IsEmpty();

    if (has_st_boundary && is_need_to_turn && is_direct_turn) {
      // AINFO << "in ST graph and no high right,and is_direct_turn ";
      continue;
    }

    // no only consider face to path,because the speed_heading maby no accurate.
    //  const auto &velocity = ptr_obstacle_item->Perception().velocity();
    //  double obs_center_s =
    //  (ptr_obstacle_item->PerceptionSLBoundary().start_s() +
    //                         ptr_obstacle_item->PerceptionSLBoundary().end_s())
    //                         *
    //                        0.5;
    //  const auto &ref_point =
    //      reference_line_info_->path_data().GetPathPointWithPathS(
    //          obs_center_s - (adc_sl.start_s() + adc_sl.end_s()) * 0.5);
    //  double ref_heading = ref_point.theta();
    //  const double obstacle_lateral_speed =
    //      common::math::Vec2d::CreateUnitVec2d(ref_heading)
    //          .CrossProd(Vec2d(velocity.x(), velocity.y()));
    //  bool is_left_cross_obs =
    //      (obs_sl.end_l() < adc_sl.start_l()) &&
    //      (obstacle_lateral_speed > speed_bounds_config_.min_cross_speed());
    //  bool is_right_cross_obs =
    //      (obs_sl.start_l() > adc_sl.end_l()) &&
    //      (obstacle_lateral_speed < -speed_bounds_config_.min_cross_speed());
    //  bool is_face_path = (is_left_cross_obs || is_right_cross_obs);
    //  if (!is_face_path) {
    //    continue;
    //  }

    // bool is_same_direction = IsSameDirection(
    //     *ptr_obstacle, reference_line_info_->vehicle_state().heading());
    // if (!is_same_direction && !is_need_to_turn) {
    //   // AINFO << "no same direction ,maby cross";
    //   continue;
    // }
    // // merage or laneborrow
    // if (is_need_to_turn && !is_direct_turn) {
    //   // AINFO << "same direction and no direct turn";
    //   continue;
    // }
    range_obs_ids->emplace_back(ptr_obstacle->Id());
  }
}

void SpeedBoundsDecider::GetBoundaries(
    std::vector<const STBoundary *> *boundaries) {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries->push_back(&st_boundary);
    }
  }
}

void SpeedBoundsDecider::BuildStopWallForDecisionInSlowBreaking(
    Frame *const frame) {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  if (TaskConfig::SPEED_BOUNDS_FINAL_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking) {
    double min_stop_decel = kMinDecel, min_stop_s = 0.0,
           stop_s = reference_line_info_->reference_line().Length(),
           adc_speed = injector_->vehicle_state()->linear_velocity();
    bool is_need_slow_breaking = reference_line_info_->IsNeedSlowBreaking(
             &min_stop_decel, &min_stop_s),
         need_to_build_stop_decision = false;
    // ADEBUG << "is_need_slow_breaking = " << is_need_slow_breaking;
    if (is_need_slow_breaking) {
      for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
        Obstacle *ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
        if (ptr_obstacle == nullptr) {
          // ADEBUG << "Current obstacle pointer is null.";
          continue;
        }
        if (ptr_obstacle->IsSlowBreakingObstacle()) {
          continue;
        }
        // ADEBUG << "ptr_obstacle_id = " << ptr_obstacle->Id();
        bool has_st_boundary = !ptr_obstacle->path_st_boundary().IsEmpty();
        if (!has_st_boundary) {
          // ADEBUG << "NO IN ST";
          continue;
        }

        if (ptr_obstacle->HasLongitudinalDecision()) {
          const auto &decision = ptr_obstacle->LongitudinalDecision();
          double temp_s = std::numeric_limits<double>::max();
          if (decision.has_follow()) {
            // ADEBUG << "FOLLOW DECISION";
            common::SLPoint follow_sl_point;
            reference_line_info_->reference_line().XYToSL(
                common::math::Vec2d(decision.follow().fence_point().x(),
                                    decision.follow().fence_point().y()),
                &follow_sl_point);
            temp_s = follow_sl_point.s() - decision.follow().distance_s() - 1.0;
          } else if (decision.has_stop()) {
            common::SLPoint stop_sl_point;
            reference_line_info_->reference_line().XYToSL(
                common::math::Vec2d(decision.stop().stop_point().x(),
                                    decision.stop().stop_point().y()),
                &stop_sl_point);
            if (ptr_obstacle->IsVirtual()) {
              temp_s = stop_sl_point.s();
            } else {
              temp_s = stop_sl_point.s() - decision.stop().distance_s() - 1.0;
            }
          } else if (decision.has_yield()) {
            // ADEBUG << "YIELD DECISION";
            common::SLPoint yield_sl_point;
            reference_line_info_->reference_line().XYToSL(
                common::math::Vec2d(decision.yield().fence_point().x(),
                                    decision.yield().fence_point().y()),
                &yield_sl_point);
            temp_s = yield_sl_point.s() - decision.yield().distance_s() - 1.0;
          } else if (decision.has_overtake()) {
            // AINFO << "OVERTAKE DECISION";
            double overtake_s =
                       ptr_obstacle->path_st_boundary().upper_left_point().s(),
                   overtake_t =
                       ptr_obstacle->path_st_boundary().upper_left_point().t();

            if (adc_speed * overtake_t < overtake_s) {
              const auto &adc_sl = reference_line_info_->AdcSlBoundary();
              const auto &vehicle_param =
                  common::VehicleConfigHelper::GetConfig().vehicle_param();
              const double adc_center_s =
                  adc_sl.end_s() - vehicle_param.front_edge_to_center();
              temp_s =
                  adc_center_s +
                  ptr_obstacle->path_st_boundary().bottom_left_point().s() -
                  1.0;
            }
          }

          if (temp_s < stop_s) {
            stop_s = temp_s;
            need_to_build_stop_decision = true;
          }
        }
      }
      if (need_to_build_stop_decision) {
        const std::string stop_wall_id = "slow_breaking_wall";
        std::vector<std::string> wait_for_obstacles;
        // ADEBUG << "build stop wall";
        util::BuildStopDecision(stop_wall_id, stop_s, 0.0,
                                StopReasonCode::STOP_REASON_SLOW_BREAKING,
                                wait_for_obstacles, "StopReasonSlowBreaking",
                                frame, reference_line_info_);
        reference_line_info_->SetIsNeedSlowBreaking(
            is_need_slow_breaking, min_stop_decel,
            stop_s - reference_line_info_->AdcSlBoundary().end_s());
      }
    }
  }
}

void SpeedBoundsDecider::SlowBreakingForInitCollisionInFastCutin() {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  const SLBoundary &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  bool is_low_speed = adc_speed < speed_bounds_config_.min_adc_speed();
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      !is_low_speed) {
    // AINFO << "boundaries.size = " << boundaries_all.size();
    for (auto *obstacle : path_decision->obstacles().Items()) {
      const auto &boundary = obstacle->path_st_boundary();
      if (!boundary.IsEmpty()) {
        if (boundary.IsPointInBoundary({0.01, 0.01}) ||
            (std::fabs(boundary.min_t()) < 0.01 &&
             std::fabs(boundary.min_s()) < 0.01)) {
          // AINFO << "FIRST POINT COLLISION";
          // AINFO << "boundary->min_t() = " << boundary.min_t();
          // AINFO << "boundary->min_s() = " << boundary.min_s();
          // AINFO << "init collision id = " << boundary.id();
          Obstacle *collision_obstacle = path_decision->Find(boundary.id());
          const auto &collision_obstacle_sl =
              collision_obstacle->PerceptionSLBoundary();
          // AINFO << "collision_obstacle_sl_start_s = "
          //       << collision_obstacle_sl.start_s();
          // AINFO << "collision_obstacle_sl_end_s = "
          //       << collision_obstacle_sl.end_s();
          // AINFO << "collision_obstacle_sl_start_l = "
          //       << collision_obstacle_sl.start_l();
          // AINFO << "collision_obstacle_sl_end_l = "
          //       << collision_obstacle_sl.end_l();
          // AINFO << "adc_sl_start_s = " << adc_sl_boundary.start_s();
          // AINFO << "adc_sl_end_s = " << adc_sl_boundary.end_s();
          // AINFO << "adc_sl_start_l = " << adc_sl_boundary.start_l();
          // AINFO << "adc_sl_end_l = " << adc_sl_boundary.end_l();

          double l_distance = std::max(
              collision_obstacle_sl.start_l() - adc_sl_boundary.end_l(),
              adc_sl_boundary.start_l() - collision_obstacle_sl.end_l());
          ADEBUG << "l_distance = " << l_distance;

          double adc_heading = injector_->vehicle_state()->heading();
          const auto &obs_velocity =
              collision_obstacle->Perception().velocity();
          const double obs_longitude_speed =
              common::math::Vec2d::CreateUnitVec2d(adc_heading)
                  .InnerProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
          // AINFO << "obs_longitude_speed = " << obs_longitude_speed;
          const double obs_lateral_speed =
              common::math::Vec2d::CreateUnitVec2d(adc_heading)
                  .CrossProd(Vec2d(obs_velocity.x(), obs_velocity.y()));
          ADEBUG << "obs_lateral_speed = " << obs_lateral_speed;

          const double adc_speed =
              injector_->vehicle_state()->linear_velocity();

          // AINFO << "adc_speed = " << adc_speed;

          // is_recreat_boundary = true;
          const auto &obs_polygon = collision_obstacle->PerceptionPolygon();
          if (CheckObsOverlap(
                  reference_line_info_->path_data().discretized_path().front(),
                  collision_obstacle_sl, obs_polygon)) {
            AINFO << "init collision";
            break;
          }

          if (obs_longitude_speed > adc_speed * kMinSpeedCoeff) {
            // AINFO << "=========cutin obs need slow breaking============";
            double min_stop_s = std::numeric_limits<double>::max();
            double min_stop_decel = std::numeric_limits<double>::max();
            bool is_in_slow_breaking = reference_line_info_->IsNeedSlowBreaking(
                &min_stop_decel, &min_stop_s);
            ADEBUG << "is_in_slow_breaking = " << is_in_slow_breaking;
            reference_line_info_->SetIsNeedSlowBreaking(
                true,
                std::min(speed_bounds_config_.three_level_slow_breking_decel(),
                         min_stop_decel),
                min_stop_s);
            const auto &id = obstacle->Id();
            auto *mutable_obs = path_decision->Find(id);
            mutable_obs->SetSlowBreakingObstacle(true);
            mutable_obs->SetSlowBreakingTag(
                "SlowBreakingForInitCollisionInFastCutin");
            // mutable_obs->EraseStBoundary();
          }
        }
      }
    }
  }
}

bool SpeedBoundsDecider::EnableSlowBreakingForCutin() {
  bool is_in_lanechange = util::IsLaneChange(injector_->planning_context());
  bool is_over_take = util::IsOvertake(injector_->planning_context());
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking_for_cutin && !is_in_lanechange &&
      !is_over_take) {
    return true;
  }
  return false;
}

void SpeedBoundsDecider::SlowBreakingForApproachingObs() {
  bool is_direct_turn = false, is_need_to_turn = false, at_left_turn = false,
       at_right_turn = false, path_turn_left = false, path_turn_right = false,
       is_need_to_slow_breaking = false;
  reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn,
                                    &at_left_turn, &at_right_turn,
                                    &path_turn_left, &path_turn_right);
  // is_in_lanechange = false;
  double adc_speed = reference_line_info_->vehicle_state().linear_velocity();

  if (EnableSlowBreakingForCutin()) {
    double decel = std::numeric_limits<double>::max();
    PathDecision *const path_decision = reference_line_info_->path_decision();
    if (nullptr == path_decision) {
      // AERROR << "The path_decision pointer is null.";
      return;
    }
    const auto &adc_sl = reference_line_info_->AdcSlBoundary();
    std::vector<std::string> range_obs_ids;
    GetMonitorObstacles(&range_obs_ids);

    if (!range_obs_ids.empty()) {
      // AINFO << "====check is face to path move=====";

      for (const auto &obs_id : range_obs_ids) {
        const auto &ignore_obstacles =
            reference_line_info_->GetIgnoregObstacles();
        if (std::find(ignore_obstacles.begin(), ignore_obstacles.end(),
                      obs_id) != ignore_obstacles.end()) {
          // AINFO << "skip";
          continue;
        }
        auto *mutable_obstacle = path_decision->Find(obs_id);
        const auto &obs_sl = mutable_obstacle->PerceptionSLBoundary();
        // AINFO << "obs_id = " << obs_id;
        // AINFO << "obs_start_l = " << obs_sl.start_l()
        //       << "   obs_end_l = " << obs_sl.end_l();

        double obstacle_lon_speed = 0.0, obstacle_lateral_speed = 0.0,
               lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                       obs_sl.start_l() - adc_sl.end_l());
        GetObsLateralAndLonSpeed(*mutable_obstacle, adc_sl, &obstacle_lon_speed,
                                 &obstacle_lateral_speed);

        // AINFO << "lateral_diff = " << lateral_diff;

        if (IsNearToSlowBreaking(lateral_diff, obstacle_lateral_speed, obs_sl,
                                 adc_sl, is_need_to_turn, mutable_obstacle)) {
          // AINFO << "NEED SLO BREAKING FOR TOO CLOSE";
          double decel_temp = std::numeric_limits<double>::max();
          // obs high speed cutin in straight lane.
          bool is_go_straight = IsGoStraight();
          // AINFO << "is_go_straight = " << is_go_straight;
          // AINFO << "obstacle_lon_speed = " << obstacle_lon_speed;
          // reverse obs in front no slowbreaking,side or back no consider in
          // st_boundary;
          // AINFO << "obstacle_lateral_speed = " << obstacle_lateral_speed;
          // AINFO << "adc_speed = " << adc_speed;

          if (obstacle_lon_speed < 0.0) {
            // AINFO << "no consider reverse obs";
            continue;
          }
          // AINFO << "adc_speed = " << adc_speed;
          if (obstacle_lon_speed > kMinSpeedCoeff * adc_speed &&
              is_go_straight) {
            bool is_front_obs = obs_sl.start_s() > adc_sl.end_s();
            if (FLAGS_enable_use_new_method_to_get_decel_for_large_speed_obs) {
              GetSlowBreakingAccBasedSpeedDiffNew(
                  *mutable_obstacle, is_front_obs, obstacle_lon_speed,
                  adc_speed, &decel_temp);
            } else {
              GetSlowBreakingAccBasedSpeedDiff(is_front_obs, obstacle_lon_speed,
                                               adc_speed, &decel_temp);
            }

          } else {
            if (FLAGS_enable_use_new_method_to_get_decel_for_approaching_obs) {
              GetSlowBreakingAccBasedLateralDistanceForApproachingObsNew(
                  *mutable_obstacle, adc_sl, obs_sl, obstacle_lon_speed,
                  adc_speed, &decel_temp);
            } else {
              GetSlowBreakingAccBasedLateralDistanceForApproachingObs(
                  adc_sl, obs_sl, obstacle_lon_speed, adc_speed, &decel_temp);
            }
          }
          if (decel_temp < decel) {
            decel = decel_temp;
          }
          mutable_obstacle->SetSlowBreakingObstacle(true);
          mutable_obstacle->SetSlowBreakingTag("SlowBreakingForApproachingObs");
          // mutable_obstacle->EraseStBoundary();
          is_need_to_slow_breaking = true;
          if (std::find(slow_breaking_obs_ids_.begin(),
                        slow_breaking_obs_ids_.end(),
                        obs_id) != slow_breaking_obs_ids_.end()) {
          } else {
            slow_breaking_obs_ids_.push_back(obs_id);
          }
        }
      }
      // AINFO << "is_need_to_slow_breaking = " << is_need_to_slow_breaking;
      // AINFO << "decel = " << decel;
      if (is_need_to_slow_breaking) {
        UpdateDecel(decel);
      }
    }
  }
}

void SpeedBoundsDecider::UpdateDecel(double decel) {
  double min_stop_decel = std::numeric_limits<double>::max(),
         min_stop_s = std::numeric_limits<double>::max();
  bool is_in_slow_breaking =
      reference_line_info_->IsNeedSlowBreaking(&min_stop_decel, &min_stop_s);
  // AINFO << "is_in_slow_breaking = " << is_in_slow_breaking;
  if ((is_in_slow_breaking && decel < min_stop_decel) ||
      (!is_in_slow_breaking)) {
    // AINFO << "approach min_stop_decel = " << min_stop_decel;
    reference_line_info_->SetIsNeedSlowBreaking(true, decel, min_stop_s);
  }
}

bool SpeedBoundsDecider::IsAdcExceedBackObs(const Obstacle &obstacle,
                                            double obstacle_lateral_speed) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_heading = injector_->vehicle_state()->heading(),
         obs_heading = obstacle.SpeedHeading();
  double backward_distance = vehicle_param.front_edge_to_center();

  double adc_front_x = injector_->vehicle_state()->x() +
                       backward_distance * std::cos(adc_heading),
         adc_front_y = injector_->vehicle_state()->y() +
                       backward_distance * std::sin(adc_heading);
  double obs_x_first = obstacle.Perception().position().x(),
         obs_y_first = obstacle.Perception().position().y();
  double obs_x_second =
             obstacle.Perception().position().x() + 1.0 * std::cos(obs_heading),
         obs_y_second =
             obstacle.Perception().position().y() + 1.0 * std::sin(obs_heading);

  common::math::Vec2d obs_first_point, obs_second_point, adc_front_point;
  obs_first_point.set_x(obs_x_first);
  obs_first_point.set_y(obs_y_first);
  obs_second_point.set_x(obs_x_second);
  obs_second_point.set_y(obs_y_second);
  adc_front_point.set_x(adc_front_x);
  adc_front_point.set_y(adc_front_y);
  double obs_side = common::math::CrossProd(adc_front_point, obs_first_point,
                                            obs_second_point);

  bool is_adc_exceed_cross_obs = false;
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  // cross path
  bool is_left_cross_obs =
           obs_sl.end_l() < adc_sl_boundary.start_l() &&
           obstacle_lateral_speed > speed_bounds_config_.min_cross_speed(),
       is_right_cross_obs =
           obs_sl.start_l() > adc_sl_boundary.end_l() &&
           obstacle_lateral_speed < -speed_bounds_config_.min_cross_speed();
  // AINFO << "obs_side = " << obs_side;
  // AINFO << "is_left_cross_obs = " << is_left_cross_obs;
  // AINFO << "is_right_cross_obs = " << is_right_cross_obs;
  if (is_left_cross_obs) {
    is_adc_exceed_cross_obs = obs_side < 0.0;
  }
  if (is_right_cross_obs) {
    is_adc_exceed_cross_obs = obs_side > 0.0;
  }
  return is_adc_exceed_cross_obs;
}

bool SpeedBoundsDecider::IsNearToSlowBreaking(
    double lateral_diff, double obstacle_lateral_speed,
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl,
    bool is_need_to_turn, Obstacle *mutable_obstacle) {
  bool is_in_slow_breaking_scope = false;
  if (mutable_obstacle->path_st_boundary().IsEmpty()) {
    is_in_slow_breaking_scope =
        ((lateral_diff <
          speed_bounds_config_
              .lateral_enter_slow_breaking_range_for_approaching_obs()) &&
         (obs_sl.end_s() > adc_sl.start_s()) && !is_need_to_turn) ||
        ((lateral_diff <
          speed_bounds_config_
              .lateral_enter_slow_breaking_range_for_approaching_obs()) &&
         (obs_sl.end_s() >
          adc_sl.start_s() -
              speed_bounds_config_.pre_slow_breaking_distance()) &&
         is_need_to_turn);
  } else {
    is_in_slow_breaking_scope =
        ((lateral_diff < speed_bounds_config_.consider_lateral_distance()) &&
         (obs_sl.end_s() > adc_sl.start_s()) && !is_need_to_turn) ||
        ((lateral_diff < speed_bounds_config_.consider_lateral_distance()) &&
         (obs_sl.end_s() >
          adc_sl.start_s() -
              speed_bounds_config_.pre_slow_breaking_distance()) &&
         is_need_to_turn);
  }
  bool is_back_obs = (obs_sl.end_s() < adc_sl.start_s());
  if (is_back_obs) {
    bool is_can_skip =
        IsAdcExceedBackObs(*mutable_obstacle, obstacle_lateral_speed);
    if (is_can_skip) {
      is_in_slow_breaking_scope = false;
      // AINFO << "exceed back obs = " << mutable_obstacle->Id();
    }
  }
  bool is_too_close =
      (lateral_diff < speed_bounds_config_.close_lateral_distance()) &&
      (std::fabs(obstacle_lateral_speed) >
       speed_bounds_config_.min_lateral_speed()) &&
      (obs_sl.end_s() > adc_sl.start_s());
  bool is_face_to_path =
      IsFaceToPath(*mutable_obstacle, obstacle_lateral_speed, obs_sl, adc_sl);
  // AINFO << "obstacle_lateral_speed = " << obstacle_lateral_speed;
  // AINFO << "is_face_to_path = " << is_face_to_path;
  const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const auto frenet_point =
      reference_line_info_->path_data().frenet_frame_path().EvaluateByS(obs_s);
  bool is_near_to_path =
      slow_breking_obstacles_history_value.IsMoveNearToPathByDiffL(
          false, *mutable_obstacle, frenet_point);
  bool is_near_to_path_loose_constraint =
      slow_breking_obstacles_history_value_loose_constraint
          .IsMoveNearToPathByDiffL(true, *mutable_obstacle, frenet_point);
  // AINFO << "is_too_close = " << is_too_close;
  // AINFO << "is_near_to_path_loose_constraint = "
  // << is_near_to_path_loose_constraint;
  // left turn no consider right no face path's obs.
  IsConsiderAwayFromPathObs(obs_sl, adc_sl, obstacle_lateral_speed,
                            &is_near_to_path);
  // AINFO << "is_near_to_path = " << is_near_to_path;
  if (IsSlowBreakingApproachingObs(
          is_too_close, is_face_to_path, is_near_to_path_loose_constraint,
          is_in_slow_breaking_scope, is_near_to_path)) {
    return true;
  } else {
    return false;
  }
}

void SpeedBoundsDecider::IsConsiderAwayFromPathObs(
    const planning::SLBoundary &obs_sl, const planning::SLBoundary &adc_sl,
    const double obstacle_lateral_speed, bool *is_near_to_path) {
  bool is_need_to_turn_for_path = false;
  bool at_left_turn = false;
  bool at_right_turn = false;
  bool path_turn_left = false;
  bool path_turn_right = false;
  bool is_direct_turn = false;
  reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn_for_path,
                                    &at_left_turn, &at_right_turn,
                                    &path_turn_left, &path_turn_right);
  if (at_left_turn && obs_sl.end_s() < adc_sl.start_s() &&
      adc_sl.start_l() > obs_sl.end_l() && *is_near_to_path) {
    if (obstacle_lateral_speed < 0.0) {
      // AINFO << "LEFT TURN ,RIGHT OBS, NO FACE ADC";
      *is_near_to_path = false;
    }
  }
}

bool SpeedBoundsDecider::IsSlowBreakingApproachingObs(
    const bool is_too_close, const bool is_face_to_path,
    const bool is_near_to_path_loose_constraint,
    const bool is_in_slow_breaking_scope, const bool is_near_to_path) {
  if ((is_too_close && (is_face_to_path || is_near_to_path_loose_constraint)) ||
      (is_in_slow_breaking_scope && is_near_to_path)) {
    return true;
  } else {
    return false;
  }
}

bool SpeedBoundsDecider::IsFaceToPath(const Obstacle &obstacle,
                                      double obstacle_lateral_speed,
                                      const planning::SLBoundary &obs_sl,
                                      const planning::SLBoundary &adc_sl) {
  double adc_theta = injector_->vehicle_state()->heading();
  // AINFO << "adc_theta = " << adc_theta;
  double adc_x = injector_->vehicle_state()->pose().position().x();
  double adc_y = injector_->vehicle_state()->pose().position().y();
  double obs_x = obstacle.Perception().position().x();
  double obs_y = obstacle.Perception().position().y();
  double dx = obs_x - adc_x;
  double dy = obs_y - adc_y;
  double heading_adc_to_obs = std::atan2(dy, dx);  // calculate the heading
  // AINFO << "heading_adc_to_obs = " << heading_adc_to_obs;
  const double diff_heading =
      common::math::NormalizeAngle(adc_theta - heading_adc_to_obs);
  // AINFO << "diff_heading = " << diff_heading;
  bool is_face_to_path = false;
  bool is_left_side_obs = false;
  bool is_right_side_obs = false;
  if (diff_heading < 0.0) {
    is_left_side_obs = true;
    ADEBUG << "LEFT SIDE OBS";
  } else {
    is_right_side_obs = true;
    ADEBUG << "RIGHT SIDE OBS";
  }
  // Classify whether all obstacles are on the left or right side of the ADC
  if (is_right_side_obs &&
      obstacle_lateral_speed > speed_bounds_config_.min_lateral_speed()) {
    ADEBUG << "is_right_side_obs, and face to left.";
    is_face_to_path = true;
  }
  if (is_left_side_obs &&
      obstacle_lateral_speed < -speed_bounds_config_.min_lateral_speed()) {
    ADEBUG << "is_left_side_obs , and face to right.";
    is_face_to_path = true;
  }
  return is_face_to_path;
}

bool SpeedBoundsDecider::IsGoStraight() {
  const auto &discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto &frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  for (size_t i = 0; i < discretized_path.size(); ++i) {
    const auto &frenet_point = frenet_path[i];
    const auto &reference_point =
        reference_line_info_->reference_line().GetReferencePoint(
            frenet_point.s());
    // ADEBUG << "reference_point.kappa = " << reference_point.kappa();
    double ref_point_kappa = reference_point.kappa();
    // AINFO << "ref_point_kappa = " << ref_point_kappa;

    // check path first turn
    if (ref_point_kappa > kMinKappa) {
      return false;
    } else if (ref_point_kappa < -kMinKappa) {
      return false;
    } else {
      // AINFO << "NO_TURN";
    }
    if (discretized_path[i].s() > kConsiderLen) {
      break;
    }
  }
  return true;
}

bool SpeedBoundsDecider::IsNeedToUpdateSlowBreakingInLowSpeed() {
  bool is_adc_in_low_speed = injector_->vehicle_state()->linear_velocity() <
                             speed_bounds_config_.min_adc_speed();
  if (!slow_breaking_obs_ids_.empty() && is_adc_in_low_speed) {
    return true;
  } else {
    return false;
  }
}

void SpeedBoundsDecider::GetDecelInLowSpeedUseLateralDiff(
    const double lateral_diff, double *tem_decel) {
  // AINFO << "obs with adc has lon overlap";
  if (lateral_diff <= 0.0) {
    // AINFO << "lon and lat has overlap,in equal speed ,-4.";
    *tem_decel = speed_bounds_config_.five_level_slow_breking_decel();
    // AINFO << "decel_temp = " << tem_decel;
  } else if (lateral_diff < speed_bounds_config_.first_lateral_distance() &&
             lateral_diff > 0.0) {
    *tem_decel = speed_bounds_config_.min_level_slow_breking_decel();
    // AINFO << "decel_temp = " << tem_decel;
  } else if (lateral_diff >= speed_bounds_config_.first_lateral_distance() &&
             lateral_diff <= speed_bounds_config_.three_lateral_distance()) {
    *tem_decel = speed_bounds_config_.three_level_slow_breking_decel();
    // AINFO << "decel_temp = " << tem_decel;
  } else if (lateral_diff > speed_bounds_config_.three_lateral_distance() &&
             lateral_diff <= speed_bounds_config_.consider_lateral_distance()) {
    *tem_decel = speed_bounds_config_.first_level_slow_breking_decel();
    // AINFO << "decel_temp = " << tem_decel;
  } else {
    *tem_decel = speed_bounds_config_.first_level_slow_breking_decel();
  }
}

void SpeedBoundsDecider::UpdateSlowBreakingInLowSpeed() {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  const SLBoundary &adc_sl = reference_line_info_->AdcSlBoundary();
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking) {
    // double min_lateral_diff = std::numeric_limits<double>::max();
    double decel = std::numeric_limits<double>::max();

    if (IsNeedToUpdateSlowBreakingInLowSpeed()) {
      double min_stop_decel = kMinDecel;
      double min_stop_s = 0.0;
      bool is_need_slow_breaking = reference_line_info_->IsNeedSlowBreaking(
          &min_stop_decel, &min_stop_s);
      std::string main_slow_breaking_tag = "";
      for (const auto &obs_id : slow_breaking_obs_ids_) {
        double tem_decel = std::numeric_limits<double>::max();
        Obstacle *ptr_obstacle = path_decision->Find(obs_id);
        const auto &obs_sl = ptr_obstacle->PerceptionSLBoundary();
        double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
                                       obs_sl.start_l() - adc_sl.end_l());
        if (obs_sl.end_s() - adc_sl.start_s() <= 0.0) {
          // AINFO << "obs in back side";
          tem_decel = speed_bounds_config_.first_level_slow_breking_decel();
          // AINFO << "decel_temp = " << tem_decel;
        } else if (obs_sl.end_s() - adc_sl.start_s() > 0.0 &&
                   obs_sl.start_s() - adc_sl.end_s() <= 0.0) {
          // // AINFO << "obs with adc has lon overlap";
          GetDecelInLowSpeedUseLateralDiff(lateral_diff, &tem_decel);
        } else {
          // AINFO << "obs all front adc";

          double lon_diff = obs_sl.start_s() - adc_sl.end_s();
          double obstacle_lon_speed = 0.0, obstacle_lateral_speed = 0.0;
          GetObsLateralAndLonSpeed(*ptr_obstacle, adc_sl, &obstacle_lon_speed,
                                   &obstacle_lateral_speed);
          double adc_speed =
              reference_line_info_->vehicle_state().linear_velocity();
          // adc_v < 2.0&& obs_v>1.2adc_v&& delta_v > 1.0 &&   delta_s > 3m
          // AINFO << "adc_speed = " << adc_speed
          // << "  obstacle_lon_speed = " << obstacle_lon_speed;
          if (obstacle_lon_speed > kMinSpeedCoeff * adc_speed &&
              ((obstacle_lon_speed - adc_speed) > kMinSpeedDiff) &&
              lon_diff > kLonStopBuffer) {
            tem_decel = 0.0;
          } else if (obstacle_lon_speed >
                         speed_bounds_config_.min_obs_speed() &&
                     lon_diff < kLonStopBuffer) {
            // adc low speed ,obs high speed ,small lon distances
            tem_decel = speed_bounds_config_.second_level_apporaching_decel();
          } else {
            tem_decel = speed_bounds_config_.min_level_apporaching_decel();
          }
        }

        if (tem_decel < decel) {
          decel = tem_decel;
          main_slow_breaking_tag = ptr_obstacle->SlowBreakingTag();
        }
      }

      // if (decel >= speed_bounds_config_.first_level_slow_breking_decel() ) {
      //  is_need_slow_breaking = false;
      // }
      if (decel >= speed_bounds_config_.first_level_slow_breking_decel() &&
          "SlowBreakingForLowRoadRightLane" == main_slow_breaking_tag) {
        is_need_slow_breaking = false;
      }
      AINFO << "low speed no slow breaking";
      AINFO << "update low speed decel = " << decel;
      double stop_s = std::numeric_limits<double>::max();
      reference_line_info_->SetIsNeedSlowBreaking(is_need_slow_breaking, decel,
                                                  stop_s);
    }
  }
}

void SpeedBoundsDecider::SlowBreakingReachTurn() {
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking) {
    bool is_adc_in_traffic_junction =
        reference_line_info_->IsAdcInCommonJunction();
    bool is_need_slow_breaking = false;
    if (!is_adc_in_traffic_junction &&
        reference_line_info_->IsReachTurnLane()) {
      // no intersection and turn direct lane, slow breaking.
      double decel = std::numeric_limits<double>::lowest();
      double stop_s = std::numeric_limits<double>::max();
      is_need_slow_breaking = SlowBreakingForLowRoadRightLane(&stop_s, &decel);
      // AINFO << "is_need_slow_breaking = " << is_need_slow_breaking;
      if (is_need_slow_breaking) {
        reference_line_info_->SetIsNeedSlowBreaking(is_need_slow_breaking,
                                                    decel, stop_s);
        // AINFO << "stop_s = " << stop_s;
      }
    }
  }
}

void SpeedBoundsDecider::SlowBreakingOnIntersection() {
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking) {
    // First time use requires clearing
    slow_breaking_obs_ids_.clear();
    bool is_need_to_turn = false;
    bool at_left_turn = false;
    bool at_right_turn = false;
    bool path_turn_left = false;
    bool path_turn_right = false;
    bool is_direct_turn = false;
    reference_line_info_->GetTurnInfo(&is_direct_turn, &is_need_to_turn,
                                      &at_left_turn, &at_right_turn,
                                      &path_turn_left, &path_turn_right);
    bool is_need_slow_breaking = false;
    bool is_adc_in_traffic_junction =
        reference_line_info_->IsAdcInCommonJunction();
    // internection
    if (is_adc_in_traffic_junction) {
      bool is_adc_in_high_road_right_lane_on_intersection =
          reference_line_info_->IsInHighRoadRightLaneOnIntersection(
              &at_left_turn, &is_direct_turn);
      // AINFO << "is_adc_in_high_road_right_lane_on_intersection = "
      //       << is_adc_in_high_road_right_lane_on_intersection;
      //  1.common junction;2.straight lane;3.broadside obs;4.in st_graph.
      if (is_adc_in_high_road_right_lane_on_intersection) {
        double stop_s = std::numeric_limits<double>::max();
        std::string slow_breking_id = "";
        is_need_slow_breaking = SlowBreaking(&stop_s, &slow_breking_id);
        // ADEBUG << "is_need_slow_breaking= " << is_need_slow_breaking;
        if (is_need_slow_breaking) {
          reference_line_info_->SetIsNeedSlowBreaking(
              is_need_slow_breaking, speed_bounds_config_.min_level_decel(),
              stop_s);
        }
        // ADEBUG << "slow_breking_id = " << slow_breking_id;
      } else {
        // 1.common junction;2.left merage lane;3.(left back and broadside obs)
        // or (right broadside obs);4.in st_graph. 1.common junction;2.left turn
        // lane;3.all back and broadside obs which first in st_graph exceed 2s
        // ;4.in st_graph.
        double decel = std::numeric_limits<double>::lowest();
        double stop_s = std::numeric_limits<double>::max();
        std::string slow_breking_id = "";
        is_need_slow_breaking =
            SlowBreakingForLowRoadRightLane(&stop_s, &decel);
        // AINFO << "is_need_slow_breaking = " << is_need_slow_breaking;
        if (is_need_slow_breaking) {
          reference_line_info_->SetIsNeedSlowBreaking(is_need_slow_breaking,
                                                      decel, stop_s);
          // AINFO << "stop_s = " << stop_s;
        }
      }
    }
  }
}

void SpeedBoundsDecider::SlowBreakingForLargeTtc() {
  if (TaskConfig::SPEED_BOUNDS_FINAL_DECIDER == config_.task_type() &&
      FLAGS_enable_slow_breaking_for_large_ttc) {
    PathDecision *const path_decision = reference_line_info_->path_decision();
    const SLBoundary &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
    double adc_v = injector_->vehicle_state()->linear_velocity(),
           min_stop_decel = std::numeric_limits<double>::max();
    bool need_slow_breaking = false, is_vulnerable_groups = false;
    for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
      Obstacle *ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
      if (ptr_obstacle == nullptr) {
        // ADEBUG << "Current obstacle pointer is null.";
        continue;
      }
      // ADEBUG << "ptr_obstacle_id = " << ptr_obstacle->Id();
      bool has_st_boundary = !ptr_obstacle->path_st_boundary().IsEmpty();
      if (!has_st_boundary) {
        continue;
      }

      if (ptr_obstacle->HasLongitudinalDecision()) {
        const auto &decision = ptr_obstacle->LongitudinalDecision();
        if (!decision.has_yield()) {
          continue;
        }
        double min_stop_decel = kMinDecel, min_stop_s = 0.0;
        bool is_need_slow_breaking = reference_line_info_->IsNeedSlowBreaking(
            &min_stop_decel, &min_stop_s);
        if (ptr_obstacle->IsSlowBreakingObstacle() && is_need_slow_breaking) {
          continue;
        }
        // AINFO << " YIELD DECISION";
        double min_s = ptr_obstacle->path_st_boundary().min_s();

        double fast_breaking_distance =
            adc_v * adc_v * 0.5 /
            std::fabs(speed_bounds_config_.three_level_slow_breking_decel());
        bool is_can_slow_breking = fast_breaking_distance > min_s;
        // AINFO << "fast_breaking_distance = " << fast_breaking_distance;
        // AINFO << "min_s = " << min_s;
        const auto &obs_sl = ptr_obstacle->PerceptionSLBoundary();
        bool is_front_obs = obs_sl.end_s() > adc_sl_boundary.end_s();
        double tmp_decel = std::numeric_limits<double>::max();
        // front obs; yield obs; collision time > 2.5; low speed or has high
        // decel;
        if (is_front_obs && (is_can_slow_breking) &&
            ptr_obstacle->path_st_boundary().min_t() >
                speed_bounds_config_.yield_collision_begin_time()) {
          // time: 2.5-5.5s ===> decel: -1.5 - -0.5
          tmp_decel = InterpolationLookUp(
              ptr_obstacle->path_st_boundary().min_t(),
              speed_bounds_config_.yield_collision_begin_time(),
              speed_bounds_config_.yield_collision_end_time(),
              speed_bounds_config_.three_level_slow_breking_decel(),
              speed_bounds_config_.first_level_slow_breking_decel());
          ptr_obstacle->SetSlowBreakingObstacle(true);
          ptr_obstacle->SetSlowBreakingTag("SlowBreakingForLargeTtc");
          // ptr_obstacle->EraseStBoundary();
          ObjectDecisionType ignore_decision;
          ignore_decision.mutable_ignore();
          ptr_obstacle->clear_yield_decision();
          ptr_obstacle->AddLongitudinalDecision("latge_ttc_slow_breaking",
                                                ignore_decision);

          if (tmp_decel < min_stop_decel) {
            min_stop_decel = tmp_decel;
            need_slow_breaking = true;
            is_vulnerable_groups = IsVulnerableGroups(*ptr_obstacle);
          }
        }
      }
    }

    if (need_slow_breaking) {
      // in low speed we can start up or keep speed.
      double target_low_speed = is_vulnerable_groups
                                    ? kTargetLowspeed
                                    : speed_bounds_config_.target_low_speed();
      if (adc_v < target_low_speed) {
        UpdateDpDataInlowSpeed(min_stop_decel, adc_v);
        return;
      }

      // in high speed we slow breaking.
      double last_stop_decel = std::numeric_limits<double>::max(),
             min_stop_s = std::numeric_limits<double>::max();
      bool is_in_slow_breaking = reference_line_info_->IsNeedSlowBreaking(
          &last_stop_decel, &min_stop_s);
      // AINFO << "is_in_slow_breaking = " << is_in_slow_breaking;
      // AINFO << "large ttc min_stop_decel = " << min_stop_decel;
      if ((is_in_slow_breaking && min_stop_decel < last_stop_decel) ||
          (!is_in_slow_breaking)) {
        reference_line_info_->SetIsNeedSlowBreaking(true, min_stop_decel,
                                                    min_stop_s);
      }
    }
  }
}

bool SpeedBoundsDecider::IsVulnerableGroups(const Obstacle &ptr_obstacle) {
  if (PerceptionObstacle::PEDESTRIAN == ptr_obstacle.Perception().type() ||
      PerceptionObstacle::BICYCLE == ptr_obstacle.Perception().type()) {
    return true;
  }
  return false;
}

void SpeedBoundsDecider::UpdateDpDataInlowSpeed(const double &min_stop_decel,
                                                const double &adc_v) {
  // AINFO << "min_stop_decel = " << min_stop_decel;
  double start_acc = 0.0;
  const auto &adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  // within 4s and has 1.5m/s，keep speed no speed up.
  if (min_stop_decel < speed_bounds_config_.second_level_slow_breking_decel() &&
      adc_v > speed_bounds_config_.target_low_speed() * 0.5) {
    start_acc = 0.0;
  } else {
    start_acc = speed_bounds_config_.start_up_target_acc();
  }

  // AINFO << "low speed ,large ttc, no consider";
  double init_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  ConstantAccToTargetVTrajectory1d target_trajectory(
      init_s, adc_v, speed_bounds_config_.target_low_speed(), start_acc);
  auto *speed_data = reference_line_info_->mutable_speed_data();
  std::vector<SpeedPoint> speed_profile;
  for (size_t i = 0; i < kTimeSteps; ++i) {
    SpeedPoint speed_point;
    speed_point.set_t(i * 0.1);
    speed_point.set_s(target_trajectory.Evaluate(0, i * 0.1) - init_s);
    // AINFO << "speed_point_t = " << speed_point.t()
    //       << "    speed_point.s = " << speed_point.s();
    speed_profile.emplace_back(speed_point);
  }
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    // AINFO << "DP rough v = " << v;
    speed_profile[i].set_v(v);
  }
  (*speed_data).clear();
  *speed_data = SpeedData(speed_profile);
}

bool SpeedBoundsDecider::IsCanSlowBreakingForAbnormalPrediction() {
  if (TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER == config_.task_type() &&
      !reference_line_info_->IsAdcInCommonJunction() &&
      FLAGS_enable_slow_breaking_for_abnormal_prediction) {
    return true;
  } else {
    return false;
  }
}

bool SpeedBoundsDecider::NoConsiderObs(const planning::SLBoundary &obs_sl,
                                       const planning::SLBoundary &adc_sl,
                                       Obstacle *mutable_obstacle) {
  if (mutable_obstacle->path_st_boundary().IsEmpty()) {
    // AINFO << "no st boundary";
    return true;
  }
  const auto &ignore_obstacles = reference_line_info_->GetIgnoregObstacles();
  if (std::find(ignore_obstacles.begin(), ignore_obstacles.end(),
                mutable_obstacle->Id()) != ignore_obstacles.end()) {
    // AINFO << "skip";
    return true;
  }
  if (!(PerceptionObstacle::VEHICLE == mutable_obstacle->Perception().type())) {
    // AINFO << "no car maby across ,so continue";
    return true;
  }

  if (!(obs_sl.start_s() > adc_sl.end_s())) {
    // AINFO << "no front obs";
    return true;
  }
  return false;
}

void SpeedBoundsDecider::SlowBreakingForAbnormalPrediction() {
  PathDecision *const path_decision = reference_line_info_->path_decision();
  if (IsCanSlowBreakingForAbnormalPrediction()) {
    const auto &adc_sl = reference_line_info_->AdcSlBoundary();

    for (const auto *ptr_obstacle_item : path_decision->obstacles().Items()) {
      Obstacle *mutable_obstacle = path_decision->Find(ptr_obstacle_item->Id());
      // AINFO << "obs_id = " << mutable_obstacle->Id();
      const auto &obs_sl = mutable_obstacle->PerceptionSLBoundary();
      if (NoConsiderObs(obs_sl, adc_sl, mutable_obstacle)) {
        continue;
      }
      // if (mutable_obstacle->path_st_boundary().IsEmpty()) {
      //   // AINFO << "no st boundary";
      //   continue;
      // }

      // if (!(PerceptionObstacle::VEHICLE ==
      //       ptr_obstacle_item->Perception().type())) {
      //   // AINFO << "no car maby across ,so continue";
      //   continue;
      // }

      // if (!(obs_sl.start_s() > adc_sl.end_s())) {
      //   // AINFO << "no front obs";
      //   continue;
      // }

      // AINFO << "obs_start_l = " << obs_sl.start_l()
      //       << "   obs_end_l = " << obs_sl.end_l();
      bool is_left_obs = obs_sl.start_l() - adc_sl.end_l() > 0.0;
      bool is_right_obs = adc_sl.start_l() - obs_sl.end_l() > 0.0;
      double adc_center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
      double curr_s =
          adc_center_s +
          ptr_obstacle_item->path_st_boundary().bottom_left_point().s();
      double curr_road_left_width = 0, curr_road_right_width = 0,
             curr_lane_left_width = 0, curr_lane_right_width = 0;
      reference_line_info_->reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      reference_line_info_->reference_line().GetLaneWidth(
          curr_s, &curr_lane_left_width, &curr_lane_right_width);
      // AINFO << "curr_road_left_width = " << curr_road_left_width;
      // AINFO << "curr_road_right_width = " << curr_road_right_width;
      // AINFO << "curr_lane_left_width = " << curr_lane_left_width;
      // AINFO << "curr_lane_right_width = " << curr_lane_right_width;
      // double lateral_diff = std::max(adc_sl.start_l() - obs_sl.end_l(),
      //                                obs_sl.start_l() - adc_sl.end_l());
      // AINFO << "lateral_diff = " << lateral_diff;
      std::vector<hdmap::LaneInfoConstPtr> lanes;
      reference_line_info_->reference_line().GetLaneFromS(curr_s, &lanes);
      if (!lanes.empty() && lanes.front() != nullptr) {
        double lane_front_heading = lanes.front()->headings().front();
        double lane_back_heading = lanes.front()->headings().back();
        if (std::fabs(common::math::NormalizeAngle(lane_back_heading -
                                                   lane_front_heading)) *
                kQuadrantAngle / M_PI_2 >
            kDiffHeading) {
          // AINFO << "collision in turn";
          continue;
        }
      }
      if (is_left_obs && obs_sl.start_l() - adc_sl.end_l() <
                             curr_road_left_width + curr_lane_left_width) {
        // AINFO << "left has no solid line,can across";
        continue;
      }
      if (is_right_obs && adc_sl.start_l() - obs_sl.end_l() <
                              (curr_road_right_width + curr_lane_right_width)) {
        // AINFO << "right has no solid line,can across";
        continue;
      }
      if (!is_left_obs && !is_right_obs) {
        continue;
      }
      // AINFO << "HAS SOLID LINE ,CAN SLOW BREAKING";
      const auto &velocity = mutable_obstacle->Perception().velocity();
      double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
      const auto &ref_point =
          reference_line_info_->path_data().GetPathPointWithPathS(
              obs_center_s - (adc_sl.start_s() + adc_sl.end_s()) * 0.5);
      double ref_heading = ref_point.theta();
      const double obstacle_lon_speed =
          common::math::Vec2d::CreateUnitVec2d(ref_heading)
              .InnerProd(Vec2d(velocity.x(), velocity.y()));
      // AINFO << "obstacle_lon_speed = " << obstacle_lon_speed;
      const double obstacle_lateral_speed =
          common::math::Vec2d::CreateUnitVec2d(ref_heading)
              .CrossProd(Vec2d(velocity.x(), velocity.y()));
      // AINFO << "obstacle_lateral_speed = " << obstacle_lateral_speed;
      // AINFO << "lon_diff = " << obs_sl.start_s() - adc_sl.end_s();
      bool is_left_cross_obs =
          (obs_sl.end_l() < adc_sl.start_l()) &&
          (obstacle_lateral_speed > speed_bounds_config_.min_cross_speed());
      bool is_right_cross_obs =
          (obs_sl.start_l() > adc_sl.end_l()) &&
          (obstacle_lateral_speed < -speed_bounds_config_.min_cross_speed());
      bool is_face_path = (is_left_cross_obs || is_right_cross_obs);
      if (!is_face_path || (obstacle_lon_speed > kMaxReverseSpeed)) {
        continue;
      }
      double adc_v = injector_->vehicle_state()->linear_velocity();
      double decel = adc_v * adc_v * 0.5 /
                     (mutable_obstacle->path_st_boundary().min_s() + kEpison);
      // no fast breaking ,no slow breaking.
      if (-decel > speed_bounds_config_.three_level_slow_breking_decel()) {
        continue;
      }
      // AINFO << "SLOW BREAKING ABNORMAL PREDICTION";
      double stop_s = std::numeric_limits<double>::max();
      mutable_obstacle->SetSlowBreakingObstacle(true);
      mutable_obstacle->SetSlowBreakingTag("SlowBreakingForAbnormalPrediction");
      // mutable_obstacle->EraseStBoundary();
      reference_line_info_->SetIsNeedSlowBreaking(
          true, speed_bounds_config_.second_level_slow_breking_decel(), stop_s);
    }
  }
}

void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto &boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint *speed_point = st_graph_debug->add_speed_limit();
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

}  // namespace planning
}  // namespace century
