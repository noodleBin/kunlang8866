/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/top_bull_decider/top_bull_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "modules/map/proto/map_electric_fence.pb.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

using century::common::VehicleState;
using century::common::math::Polygon2d;
using century::hdmap::HDMapUtil;
using century::hdmap::Junction;
using century::hdmap::JunctionInfo;
using century::hdmap::JunctionInfoConstPtr;

namespace century {
namespace planning {
namespace {
constexpr double kStopSpeed = 0.1;
constexpr double kMinTheta = M_PI / 8;
constexpr double kEpsilon = 1e-6;
constexpr double kOverlapDistanceThreshold = 0.4;
constexpr double kTimeoutThreshold = 300;  // seconds
constexpr double kLateralBuffer = 0.6;
constexpr double kBlockingIgvLength = 15.0;
constexpr double kBlockingIgvWidth = 3.2;
constexpr double kDefaultReverseDistance = 15.0;
constexpr double kDefaultBorrowReverseDistance = 10.0;
constexpr int kMinMatchNum = 1;
}  // namespace
using century::common::ErrorCode;
using century::common::Status;
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;

TopBullDecider::TopBullDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status TopBullDecider::Process(Frame* frame,
                               ReferenceLineInfo* reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  frame_ = frame;
  reference_line_info_ = reference_line_info;

  // skip top_bull_decider if not enable
  if (!Decider::config_.top_bull_decider_config().enable_top_bull()) {
    return Status::OK();
  }

  MakeTopBullDecision(frame, reference_line_info);

  return Status::OK();
}

void TopBullDecider::MakeTopBullDecision(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  auto* top_bull = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_top_bull();
  bool is_in_top_bull_scenario = false;
  if (top_bull->has_is_in_top_bull() && true == top_bull->is_in_top_bull()) {
    is_in_top_bull_scenario = true;
  }

  std::string top_bull_msg;
  top_bull_msg.clear();

  if (!is_in_top_bull_scenario) {
    // Check whether it is in the head-to-head scenario
    std::string blocking_igv_vehicle_id;
    VehicleInfo blocking_igv_info;
    if (!CheckTopBullConditions(top_bull_msg, blocking_igv_vehicle_id,
                                blocking_igv_info, top_bull)) {
      return;
    }

    // Get adc path info
    std::vector<planning::PathInfo> adc_path_info;
    const auto& adc_path_points =
        reference_line_info->path_data().discretized_path();
    for (size_t i = 0; i < adc_path_points.size(); ++i) {
      planning::PathInfo path_info;
      path_info.set_x(adc_path_points[i].x());
      path_info.set_y(adc_path_points[i].y());
      path_info.set_theta(adc_path_points[i].theta());
      adc_path_info.emplace_back(path_info);
    }

    // Get blocking igv path info
    std::vector<planning::PathInfo> blocking_igv_path_info;
    for (size_t i = 0; i < blocking_igv_info.path_info_size(); ++i) {
      blocking_igv_path_info.emplace_back(blocking_igv_info.path_info(i));
    }

    // Analyze path relation
    auto path_relation_type =
        AnalyzePathRelation(adc_path_info, blocking_igv_path_info);
    if (PathRelationType::NONE_RELATION == path_relation_type ||
        PathRelationType::SAME_DIRECTION_OVERLAP == path_relation_type) {
      top_bull_msg =
          "Adc and blocking igv is not in top bull, path relation type: " +
          std::to_string(path_relation_type);
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }

    // Adc is in top bull
    top_bull->set_is_in_top_bull(true);
    top_bull->set_blocking_igv_id(blocking_igv_vehicle_id);

    MakeTopBullAction(blocking_igv_info, path_relation_type, top_bull_msg,
                      top_bull);
  } else {
    // Check if it is possible to exit the head-to-head scenario
    CheckExitTopBull(top_bull_msg, top_bull);
  }
}

bool TopBullDecider::CheckTopBullConditions(
    std::string& top_bull_msg, std::string& blocking_igv_vehicle_id,
    VehicleInfo& blocking_igv_info, TopBullStatus* top_bull) {
  // 1. check adc is auto drive
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE !=
      frame_->local_view().chassis->driving_mode()) {
    top_bull_msg = "Driving mode is not complete auto drive mode";
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  // 2. check adc is stop
  bool is_adc_stop =
      std::fabs(reference_line_info_->vehicle_state().linear_velocity()) <
      kStopSpeed;
  if (!is_adc_stop) {
    top_bull_msg =
        "Adc is not stop, adc speed: " +
        std::to_string(reference_line_info_->vehicle_state().linear_velocity());
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  // 3. check blocking obstacle is igv
  if (top_bull->block_obs_id().empty()) {
    top_bull_msg = "Block obs id is empty";
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  const Obstacle* blocking_obstacle =
      reference_line_info_->path_decision()->obstacles().Find(
          top_bull->block_obs_id());
  if (nullptr == blocking_obstacle) {
    top_bull_msg =
        "No blocking obstacle, block obs id: " + top_bull->block_obs_id();
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  if (!blocking_obstacle->IsIgv()) {
    top_bull_msg = "Blocking obstacle is not igv, block obs id: " +
                   blocking_obstacle->Id();
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  // 4. check v2x info
  blocking_igv_vehicle_id = blocking_obstacle->GetIgvVehicleId();
  if (blocking_igv_vehicle_id.empty()) {
    top_bull_msg = "Blocking igv vehicle id is empty, block obs id: " +
                   blocking_obstacle->Id();
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  auto v2x_info = injector_->v2x_info();
  bool find_blocking_igv_info = false;
  for (const auto& vehicle_info : v2x_info.vehicle_info()) {
    if (vehicle_info.id() == blocking_igv_vehicle_id) {
      blocking_igv_info.CopyFrom(vehicle_info);
      find_blocking_igv_info = true;
      break;
    }
  }

  if (!find_blocking_igv_info) {
    top_bull_msg = "Can not find blocking igv info, vehicle id: " +
                   blocking_igv_vehicle_id +
                   ", block obs id: " + blocking_obstacle->Id();
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  // 5. check blocking igv speed
  bool is_blocking_igv_stop = std::fabs(blocking_igv_info.speed()) < kStopSpeed;
  if (!is_blocking_igv_stop) {
    top_bull_msg = "Blocking igv is not stop, blocking_igv speed: " +
                   std::to_string(blocking_igv_info.speed()) +
                   ", vehicle id: " + blocking_igv_vehicle_id +
                   ", block obs id: " + blocking_obstacle->Id();
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  // 6. check path info
  if (0 == blocking_igv_info.path_info_size() ||
      reference_line_info_->path_data().Empty()) {
    top_bull_msg = "Adc path info or blocking igv path info is empty";
    top_bull->set_top_bull_msg(top_bull_msg);
    return false;
  }

  return true;
}

void TopBullDecider::MakeTopBullAction(
    const VehicleInfo& blocking_igv_info,
    const PathRelationType& path_relation_type, std::string& top_bull_msg,
    TopBullStatus* top_bull) {
  // Check light-loaded IGV makes way for the heavy-loaded IGV
  bool adc_is_load =
      frame_->local_view().routing->routing_request().is_loading();
  bool blocking_igv_is_load = blocking_igv_info.is_loading();

  if (adc_is_load && !blocking_igv_is_load) {
    top_bull->set_action_type(TopBullStatus::WAITING);
    top_bull_msg = "Adc is load, blocking igv is not load, adc is waiting";
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  }

  if (!adc_is_load && blocking_igv_is_load) {
    if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::BORROW);
      top_bull->set_reverse_distance(kDefaultBorrowReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg = "Adc is not load, blocking igv is load, adc is borrow";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }

    if (PathRelationType::CROSSING_RELATION == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::REVERSE);
      top_bull->set_reverse_distance(kDefaultReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg = "Adc is not load, blocking igv is load, adc is reverse";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }
  }

  // Turn to let go straight, right turn to let the left turn get adc lane
  // turn type
  auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = reference_line_info_->LocateLaneInfo(s);
  if (nullptr == locate_lane) {
    top_bull->set_is_in_top_bull(false);
    top_bull_msg = "Can not get adc lane";
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  }
  auto adc_turn_type = locate_lane->lane().turn();
  // blocking igv no turn, but adc turn
  if ((hdmap::Lane::LEFT_TURN == adc_turn_type ||
       hdmap::Lane::RIGHT_TURN == adc_turn_type) &&
      planning::NO_TURN == blocking_igv_info.turn_type()) {
    if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::BORROW);
      top_bull->set_reverse_distance(kDefaultBorrowReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg = "Adc is not turn, blocking igv is turn, adc is borrow";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    } else if (PathRelationType::CROSSING_RELATION == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::REVERSE);
      top_bull->set_reverse_distance(kDefaultReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg = "Adc is not turn, blocking igv is turn, adc is reverse";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }
  }

  // adc no turn, but blocking igv turn
  if (hdmap::Lane::NO_TURN == adc_turn_type &&
      (planning::LEFT_TURN == blocking_igv_info.turn_type() ||
       planning::RIGHT_TURN == blocking_igv_info.turn_type())) {
    top_bull->set_action_type(TopBullStatus::WAITING);
    top_bull_msg = "Adc is turn, blocking igv is not turn, adc is waiting";
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  }

  // adc left turn, blocking igv right turn
  if (hdmap::Lane::LEFT_TURN == adc_turn_type &&
      planning::RIGHT_TURN == blocking_igv_info.turn_type()) {
    top_bull->set_action_type(TopBullStatus::WAITING);
    top_bull_msg =
        "Adc is left turn, blocking igv is right turn, adc is waiting";
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  }

  // adc right turn, blocking igv left turn
  if (hdmap::Lane::RIGHT_TURN == adc_turn_type &&
      planning::LEFT_TURN == blocking_igv_info.turn_type()) {
    if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::BORROW);
      top_bull->set_reverse_distance(kDefaultBorrowReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg =
          "Adc is right turn, blocking igv is left turn, adc is borrow";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    } else if (PathRelationType::CROSSING_RELATION == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::REVERSE);
      top_bull->set_reverse_distance(kDefaultReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg =
          "Adc is right turn, blocking igv is left turn, adc is reverse";
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }
  }

  // Check random number
  if (!top_bull->has_random_number() || 0 == top_bull->random_number()) {
    top_bull->set_random_number(GenerateRandomNumber());
  }

  if (0 == blocking_igv_info.random_number()) {
    top_bull->set_is_in_top_bull(false);
    top_bull_msg = "Blocking igv random number is zero";
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  }

  if (top_bull->random_number() > blocking_igv_info.random_number()) {
    top_bull->set_action_type(TopBullStatus::WAITING);
    top_bull_msg =
        "Adc random number is bigger than blocking igv random number, adc is "
        "waiting, adc random number: " +
        std::to_string(top_bull->random_number()) +
        ", blocking igv random number: " +
        std::to_string(blocking_igv_info.random_number());
    top_bull->set_top_bull_msg(top_bull_msg);
    return;
  } else {
    if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::BORROW);
      top_bull->set_reverse_distance(kDefaultBorrowReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg =
          "Adc random number is smaller than blocking igv random number, adc "
          "is borrow, adc random number: " +
          std::to_string(top_bull->random_number()) +
          ", blocking igv random number: " +
          std::to_string(blocking_igv_info.random_number());
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    } else if (PathRelationType::CROSSING_RELATION == path_relation_type) {
      top_bull->set_action_type(TopBullStatus::REVERSE);
      top_bull->set_reverse_distance(kDefaultReverseDistance);
      top_bull->set_ego_start_action_time(Clock::NowInSeconds());
      top_bull_msg =
          "Adc random number is smaller than blocking igv random number, adc "
          "is reverse, adc random number: " +
          std::to_string(top_bull->random_number()) +
          ", blocking igv random number: " +
          std::to_string(blocking_igv_info.random_number());
      top_bull->set_top_bull_msg(top_bull_msg);
      return;
    }
  }
}

void TopBullDecider::CheckExitTopBull(std::string& top_bull_msg,
                                      TopBullStatus* top_bull) {
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE !=
      frame_->local_view().chassis->driving_mode()) {
    ClearTopBullStatus(top_bull);
    top_bull_msg = "Adc is not in auto drive, exit top bull";
  } else if (TopBullStatus::WAITING == top_bull->action_type()) {
    if (NoPathCollision(top_bull->blocking_igv_id())) {
      ClearTopBullStatus(top_bull);
      top_bull_msg = "Adc is waiting, but no collision, exit top bull";
    }
  } else if (top_bull->ego_complete_action()) {
    if (NoPathCollision(top_bull->blocking_igv_id())) {
      ClearTopBullStatus(top_bull);
      top_bull_msg = "Adc is complete action, but no collision, exit top bull";
    }
  } else if (0 != top_bull->ego_start_action_time() &&
             Clock::NowInSeconds() - top_bull->ego_start_action_time() >
                 kTimeoutThreshold) {
    ClearTopBullStatus(top_bull);
    top_bull_msg = "Adc is timeout, exit top bull";
  }
  top_bull->set_top_bull_msg(top_bull_msg);
}

void TopBullDecider::ClearTopBullStatus(TopBullStatus* top_bull) {
  top_bull->set_is_in_top_bull(false);
  top_bull->set_action_type(TopBullStatus::NONE);
  top_bull->set_block_obs_id("");
  top_bull->set_random_number(0);
  top_bull->set_reverse_distance(0);
  top_bull->set_blocking_igv_id("");
  top_bull->set_ego_start_action_time(0);
  top_bull->set_ego_complete_action(false);
}

void TopBullDecider::CalculateMeanAndVar(const std::vector<int>& data,
                                         double& mean, double& variance) {
  if (data.empty()) {
    mean = 0;
    variance = 0;
    return;
  }
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  mean = sum / data.size();

  double sq_sum = 0.0;
  for (int val : data) {
    sq_sum += (val - mean) * (val - mean);
  }
  variance = sq_sum / data.size();
}

PathRelationType TopBullDecider::AnalyzePathRelation(
    const std::vector<planning::PathInfo>& path_a,
    const std::vector<planning::PathInfo>& path_b, double dist_threshold,
    double angle_threshold) {
  if (path_a.empty() || path_b.empty()) {
    AERROR << "TopBull: Empty path, no relation.";
    return NONE_RELATION;
  }

  size_t len_a = path_a.size();
  size_t len_b = path_b.size();

  // Record the index difference values of valid matching points
  std::vector<int> indexOffsets;
  // Record the orientation difference values of valid matching points
  std::vector<double> angleDiffs;

  // 1. find matching points
  // Utilizing sliding window optimization, assuming that vehicle speeds are
  // similar, the index changes will not be too drastic
  const int SEARCH_WINDOW = 10;

  for (size_t i = 0; i < len_a; ++i) {
    double minDist = std::numeric_limits<double>::max();
    int bestIdx = -1;

    int startJ = 0;
    int endJ = len_b - 1;

    if (i > 0 && !indexOffsets.empty()) {
      // Search based on the previous matching location
      // The index of the last matched B is i-1 + indexOffsets.back()
      int lastMatchJ = (int)(i - 1) + indexOffsets.back();
      startJ = std::max(0, lastMatchJ - SEARCH_WINDOW);
      endJ = std::min((int)len_b - 1, lastMatchJ + SEARCH_WINDOW);
    }

    for (int j = startJ; j <= endJ; ++j) {
      double d = common::util::DistanceXY(path_a[i], path_b[j]);
      if (d < minDist) {
        minDist = d;
        bestIdx = j;
      }
    }

    // 2. Verify the validity of the match
    if (bestIdx != -1 && minDist < dist_threshold) {
      int offset = bestIdx - (int)i;
      indexOffsets.push_back(offset);

      // Calculation of orientation difference
      double angleDiff = std::abs(common::math::NormalizeAngle(
          path_a[i].theta() - path_b[bestIdx].theta()));
      angleDiffs.push_back(angleDiff);
    }
  }

  // If there are too few matching points, it is determined as a crossover
  // Set the threshold to 20% of the path length (adjustable according to
  // requirements)
  ADEBUG << "TopBull: Number of matching points: " << indexOffsets.size();
  if (indexOffsets.size() < kMinMatchNum) {
    ADEBUG << "TopBull: Too few matching points, determine as a none.";
    return NONE_RELATION;
  } else if (indexOffsets.size() < std::max(len_a, len_b) * 0.2) {
    ADEBUG << "TopBull: Too few matching points, determine as a crossover.";
    return CROSSING_RELATION;
  }

  // 3. Analyze the statistical characteristics of index offsets
  double meanOffset, varOffset;
  CalculateMeanAndVar(indexOffsets, meanOffset, varOffset);

  // Calculate the average orientation difference
  double avgAngleDiff =
      std::accumulate(angleDiffs.begin(), angleDiffs.end(), 0.0) /
      angleDiffs.size();

  // Judgment logic:
  // A. Coincidence in the same direction: small offset variance and small
  // average orientation difference, B. Non-coincidence in direction: offset
  // decreases monotonically, or the average orientation difference is close to
  // PI, C. Cross: Other situations

  // Determining the same direction: small variance and consistent orientation
  // The variance threshold is set to 4.0 (2 standard deviations), allowing for
  // some bending and noise
  ADEBUG << "TopBull: Mean offset: " << meanOffset
         << ", variance: " << varOffset << ", avg angle diff: " << avgAngleDiff;
  if (varOffset < 4.0 && avgAngleDiff < angle_threshold) {
    ADEBUG << "TopBull: Same direction overlap.";
    return SAME_DIRECTION_OVERLAP;
  }

  // Judgment reversal：
  // Scenario 1: Clearly opposite orientations (average angle difference > 90
  // degrees)
  if (avgAngleDiff > angle_threshold) {
    return OPPOSITE_DIRECTION_OVERLAP;
  }

  // Scenario 2: The index offset exhibits a monotonically decreasing trend
  // (even in the presence of noisy data)
  bool isMonotonicallyDecreasing = true;
  for (size_t k = 1; k < indexOffsets.size(); ++k) {
    // Occasional non-decrease is allowed, but the overall trend must be
    // decreasing
    if (indexOffsets[k] > indexOffsets[k - 1] + 1) {
      isMonotonicallyDecreasing = false;
      break;
    }
  }

  if (isMonotonicallyDecreasing) {
    ADEBUG << "TopBull: Opposite direction overlap.";
    return OPPOSITE_DIRECTION_OVERLAP;
  }

  // If it is neither in the same direction nor in the opposite direction, it is
  // determined as a crossing
  ADEBUG << "TopBull: Crossing.";
  return CROSSING_RELATION;
}

bool TopBullDecider::NoPathCollision(
    const std::string& blocking_igv_vehicle_id) {
  // Generate the blocking igv bounding box
  auto v2x_info = injector_->v2x_info();
  bool find_blocking_igv_info = false;
  planning::VehicleInfo blocking_igv_info;
  for (const auto& vehicle_info : v2x_info.vehicle_info()) {
    if (vehicle_info.id() == blocking_igv_vehicle_id) {
      blocking_igv_info.CopyFrom(vehicle_info);
      find_blocking_igv_info = true;
      break;
    }
  }

  if (!find_blocking_igv_info) {
    AERROR << "TopBull: Failed to find blocking igv : "
           << blocking_igv_vehicle_id;
    return false;
  }

  common::math::Vec2d blocking_igv_point =
      Vec2d(blocking_igv_info.x(), blocking_igv_info.y());
  double blocking_igv_heading = blocking_igv_info.heading();
  // Compute the blocking igv bounding box.
  Box2d blocking_igv_box({blocking_igv_point.x(), blocking_igv_point.y()},
                         blocking_igv_heading, kBlockingIgvLength,
                         kBlockingIgvWidth);

  common::math::Polygon2d blocking_igv_polygon =
      common::math::Polygon2d(blocking_igv_box);

  // Check adc path collision
  auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double center_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  const auto& reference_point =
      reference_line_info_->reference_line().GetReferencePoint(center_s);
  double heading = reference_line_info_->vehicle_state().heading();
  if (reference_line_info_->NeedDiagonal()) {
    if (reference_line_info_->IsInDiagonalRoad()) {
      heading = reference_line_info_->DiagonalRoadHeading();
    } else {
      heading = reference_point.heading();
    }
  }

  for (const auto& path_point :
       reference_line_info_->path_data().discretized_path()) {
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if (reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s)) {
        heading = path_point.theta();
      }
    }
    common::math::Box2d adc_box;
    if (reference_line_info_->NeedDiagonal()) {
      adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
          path_point.x(), path_point.y(), heading, 0.0, kLateralBuffer);
    } else {
      adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
          path_point, kLateralBuffer);
    }
    if (blocking_igv_polygon.HasOverlap(common::math::Polygon2d(adc_box))) {
      return false;
    }
  }

  return true;
}

uint64_t TopBullDecider::GenerateRandomNumber() {
  // Generate random number
  std::random_device rd;
  std::mt19937_64 generator(rd());
  std::uniform_int_distribution<uint64_t> distribution;

  return distribution(generator);
}

}  // namespace planning
}  // namespace century