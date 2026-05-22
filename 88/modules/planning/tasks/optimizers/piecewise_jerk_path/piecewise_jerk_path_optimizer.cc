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
 * @file piecewise_jerk_path_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include "cyber/time/clock.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {
using century::canbus::Chassis;
using century::perception::PerceptionObstacle;
namespace {
constexpr double kLateralDistanceToDestination = 5.0;
constexpr double kTiny = 0.001;
constexpr double kLateralBuffer = 0.50;
constexpr double kPlusLateralDistance = 0.5;
constexpr double kReduceLateralDistance = -0.5;
constexpr double kMinDeltaL = 0.2;
constexpr double kMaxLateralBuffer = 1.0;
constexpr double kMinLateralBuffer = 0.5;
constexpr double kPedstrianLateralBuffer = 1.0;
constexpr double kSameDirectionThr = M_PI * 0.2;
constexpr double kSafeTimeOnSameDirection = 2.0;
constexpr double kSafeTimeOnOppositeDirection = 3.0;
constexpr int kMaxCount = 10;
constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
constexpr double kBackwardMinSafeDistanceOnSameDirection = 5.0;
constexpr double kForwardMinSafeDistanceOnOppositeDirection = 10.0;
constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
constexpr double kBusLength = 7.0;
constexpr double kAddBuffer = 0.4;
constexpr double kDistanceReverseDistance = 15.0;
constexpr double kDistanceFront = 10.0;
constexpr int kMaxOptimizeIter = 4000;
constexpr double kMinPreVelocity = 1.0;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kEndStateShiftThreshold = 0.02;
constexpr double kEpsilon = 0.001;
constexpr double kBlockBorrowObsLonBuffer = 2.0;
constexpr double kBlockBorrowObsLatBuffer = 0.8;
constexpr double kAdcStopVelocityThreshold = 0.05;
constexpr double min_turn_radius = 9.0;
constexpr double kMaxSpeedForBorrow = 3.0;
constexpr double kStopSpeed = 0.1;
constexpr double kMinLateralError = 0.05;
constexpr double kDistanceExtraExtend = 0.5;
constexpr double kDistanceToBlockobs = 15.0;
constexpr double kDistanceBuffer = 0.5;
constexpr double kMinSafeReactionDistance = 1.0;
constexpr double kMinDiffL = 0.3;
constexpr double kMinSpeed = 0.1;
constexpr double kMinLDiff = 0.1;
constexpr double kConsiderBlockBuffer = 1.0;
constexpr double kMinConsiderRanger = 0.5;
constexpr double kNoDiagonalSpeed = 4.0;
constexpr double kDestinationNoBorrowDistance = 40.0;
constexpr double kDestinationConsiderLateralDistance = 3.0;
constexpr double kBackConsiterDistance = 2.0;
constexpr double kFrontConsierDistance = 15.0;
constexpr double kDistanceToTurn = 25.0;
constexpr double kAddBufferInTurn = 1.5;
constexpr double kInitLateralSpeed = 0.3;
constexpr double kConsiderLateralBuffer = 1.0;
constexpr double kBufferToBoundary = 1.5;
constexpr double kBackwardDistanceBuffer = 3.0;
constexpr double kMaxSteerAngle = 0.4363;
constexpr double kMinKappa = 0.01;
constexpr double kMinLateralDistance = 0.2;
constexpr double kStackerLateralBuffer = 2.5;
constexpr double kMinL = 0.1;
constexpr double kTireStackerHeight = 6.0;
constexpr double kEndStateStep = 0.5;
constexpr double kOutBoundaryLateralBuffer = 0.3;
constexpr double kMinSteerPercentage = 1.0;
constexpr double kPrintFeasibleSpaceForwardDistance = 10.0;
constexpr double kMinXRefWeight = 1.0;
constexpr double kLowXRefWeight = 50.0;
constexpr double kHighXRefWeight = 1000.0;
constexpr double kBoundaryDropProtectionStartDdlBuffer = 0.03;
constexpr double kBoundaryDropProtectionTightDddlBoundRatio = 0.5;

struct BoundaryDropSmoothProtectionState {
  bool triggered = false;
  bool near_left_boundary = false;
  bool near_right_boundary = false;
  double min_left_clearance = std::numeric_limits<double>::infinity();
  double min_right_clearance = std::numeric_limits<double>::infinity();
  double max_left_inward_jump = 0.0;
  double max_right_inward_jump = 0.0;
};

double LinearBlend(const double start_value, const double end_value,
                   const double ratio) {
  return start_value + (end_value - start_value) * ratio;
}

void PrintFeasibleSpaceNearInitPoint(
    const PathBoundary& path_boundary,
    const std::pair<std::array<double, 3>, std::array<double, 3>>&
        init_frenet_state) {
  const double init_s = init_frenet_state.first[0];
  const double init_l = init_frenet_state.second[0];
  const double start_s = path_boundary.start_s();
  const double delta_s = path_boundary.delta_s();
  const auto& boundary = path_boundary.boundary();

  ADEBUG << "[FeasibleSpace] label=" << path_boundary.label()
        << ", init_s=" << init_s << ", init_l=" << init_l
        << ", print_forward_distance=" << kPrintFeasibleSpaceForwardDistance;

  bool has_point_to_print = false;
  for (size_t i = 0; i < boundary.size(); ++i) {
    const double sample_s = start_s + static_cast<double>(i) * delta_s;
    const double relative_s = sample_s - init_s;
    if (relative_s < -kEpsilon) {
      continue;
    }
    if (relative_s > kPrintFeasibleSpaceForwardDistance) {
      break;
    }
    // PathBoundary stores each point as [l_min, l_max].
    const double l_min = boundary.at(i).first;
    const double l_max = boundary.at(i).second;
    const double right_feasible_space = init_l - l_min;
    const double left_feasible_space = l_max - init_l;

    ADEBUG << "[FeasibleSpace] label=" << path_boundary.label()
          << ", s_rel=" << relative_s << ", s=" << sample_s
          << ", l_min=" << l_min << ", l_max=" << l_max
          << ", right_space_from_init_l=" << right_feasible_space
          << ", left_space_from_init_l=" << left_feasible_space
          << ", total_width=" << (l_max - l_min);
    has_point_to_print = true;
  }

  if (!has_point_to_print) {
    ADEBUG << "[FeasibleSpace] label=" << path_boundary.label()
          << ", no boundary samples found within "
          << kPrintFeasibleSpaceForwardDistance
          << "m ahead of init point.";
  }
}

BoundaryDropSmoothProtectionState EvaluateBoundaryDropSmoothProtection(
    const PiecewiseJerkPathOptimizerConfig& config,
    const PathBoundary& path_boundary,
    const std::pair<std::array<double, 3>, std::array<double, 3>>&
        init_frenet_state) {
  BoundaryDropSmoothProtectionState state;
  if (!config.enable_boundary_drop_smooth_protection()) {
    return state;
  }

  const double init_s = init_frenet_state.first[0];
  const double init_l = init_frenet_state.second[0];
  const double half_width =
      century::common::VehicleConfigHelper::GetConfig().vehicle_param().width() *
      0.5;
  const auto& boundary = path_boundary.boundary();
  const double start_s = path_boundary.start_s();
  const double delta_s = path_boundary.delta_s();

  bool has_prev_point = false;
  double prev_l_min = 0.0;
  double prev_l_max = 0.0;
  size_t valid_point_count = 0;
  for (size_t i = 0; i < boundary.size(); ++i) {
    const double sample_s = start_s + static_cast<double>(i) * delta_s;
    const double relative_s = sample_s - init_s;
    if (relative_s < -kEpsilon) {
      continue;
    }
    if (relative_s > config.boundary_drop_protection_forward_distance()) {
      break;
    }

    const double l_min = boundary.at(i).first;
    const double l_max = boundary.at(i).second;
    const double left_clearance = l_max - (init_l + half_width);
    const double right_clearance = (init_l - half_width) - l_min;
    state.min_left_clearance =
        std::min(state.min_left_clearance, left_clearance);
    state.min_right_clearance =
        std::min(state.min_right_clearance, right_clearance);

    if (has_prev_point) {
      state.max_left_inward_jump =
          std::max(state.max_left_inward_jump, prev_l_max - l_max);
      state.max_right_inward_jump =
          std::max(state.max_right_inward_jump, l_min - prev_l_min);
    }
    prev_l_min = l_min;
    prev_l_max = l_max;
    has_prev_point = true;
    ++valid_point_count;
  }

  if (valid_point_count < 2) {
    return state;
  }

  state.near_left_boundary =
      state.min_left_clearance <
      config.boundary_drop_protection_near_boundary_clearance();
  state.near_right_boundary =
      state.min_right_clearance <
      config.boundary_drop_protection_near_boundary_clearance();
  const double large_drop_threshold =
      config.boundary_drop_protection_large_drop_threshold();
  state.triggered =
      (state.near_left_boundary &&
       state.max_left_inward_jump > large_drop_threshold) ||
      (state.near_right_boundary &&
       state.max_right_inward_jump > large_drop_threshold);
  return state;
}

void ApplyBoundaryDropProtectionToDdlBounds(
    const PiecewiseJerkPathOptimizerConfig& config,
    const PathBoundary& path_boundary, const double init_ddl,
    std::vector<std::pair<double, double>>* const ddl_bounds) {
  if (ddl_bounds == nullptr || ddl_bounds->empty()) {
    return;
  }

  const double hold_length =
      std::max(0.0, config.boundary_drop_protection_hold_length());
  const double release_length =
      std::max(0.0, config.boundary_drop_protection_release_length());

  for (size_t i = 0; i < ddl_bounds->size(); ++i) {
    const double relative_s = static_cast<double>(i) * path_boundary.delta_s();
    if (relative_s > hold_length + release_length + kEpsilon) {
      break;
    }

    const double original_min = ddl_bounds->at(i).first;
    const double original_max = ddl_bounds->at(i).second;
    const double limited_min =
        std::max(original_min, init_ddl - kBoundaryDropProtectionStartDdlBuffer);
    const double limited_max =
        std::min(original_max, init_ddl + kBoundaryDropProtectionStartDdlBuffer);
    if (limited_min > limited_max) {
      continue;
    }

    if (relative_s <= hold_length + kEpsilon) {
      ddl_bounds->at(i) = std::make_pair(limited_min, limited_max);
      continue;
    }
    if (release_length <= kEpsilon) {
      continue;
    }

      const double ratio =
          std::max(0.0, std::min(1.0, (relative_s - hold_length) /
                                          std::max(release_length, kEpsilon)));
    ddl_bounds->at(i) = std::make_pair(
        LinearBlend(limited_min, original_min, ratio),
        LinearBlend(limited_max, original_max, ratio));
    }
  }

}  // namespace
using century::common::ErrorCode;
using century::common::Status;
using century::common::VehicleConfigHelper;
using century::common::math::Clamp;
using century::common::math::Gaussian;
using century::cyber::Clock;

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector) {
  ACHECK(config_.has_piecewise_jerk_path_optimizer_config());
}

bool PiecewiseJerkPathOptimizer::CheckIsDiagonalRoad() {
  if(!FLAGS_enable_diagonal_road_check){
    return false;
  }
  bool is_in_diagonal = false;
  bool no_use_locate_lane_heading = false;

  auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane =
      reference_line_info_->LocateLaneInfo(adc_s);
  double lane_length = locate_lane->lane().length();
  // AINFO << "lane_length = " << lane_length;
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double adc_half_length = veh_param.length() * 0.5;
  // AINFO << "adc_half_length = " << adc_half_length;
  if (nullptr != locate_lane) {
    double diagonal_lane_heading =
        locate_lane->Heading(locate_lane->lane().length() * 0.5);
    if (locate_lane->lane().has_is_diagonal_road() &&
        locate_lane->lane().is_diagonal_road()) {
      is_in_diagonal = true;
      no_use_locate_lane_heading = true;
      // AINFO << "adc all in diagonal road";
    }
    double accumulate_s = 0.0;
    double lateral = 0.0;
    common::math::Vec2d point;
    point.set_x(reference_line_info_->vehicle_state().pose().position().x());
    point.set_y(reference_line_info_->vehicle_state().pose().position().y());
    bool get_neast_point =
        locate_lane->GetProjection(point, &accumulate_s, &lateral);
    // AINFO << "accumulate_s = " << accumulate_s;
    if (!get_neast_point) {
      // AINFO << "get_neast_point = false";
      return false;
    }

    if (accumulate_s - adc_half_length < kBackConsiterDistance) {
      hdmap::LaneInfoConstPtr pressor_lane =
          reference_line_info_->LocateLaneInfo(adc_s - kBackConsiterDistance -
                                               adc_half_length);

      if (nullptr != pressor_lane) {
        if (pressor_lane->lane().has_is_diagonal_road() &&
            pressor_lane->lane().is_diagonal_road()) {
          // AINFO << "adc over in diagonal road";
          is_in_diagonal = true;
        }
      }
    }

    if (lane_length - accumulate_s - adc_half_length < kFrontConsierDistance) {
      hdmap::LaneInfoConstPtr successor_lane =
          reference_line_info_->LocateLaneInfo(adc_s + kFrontConsierDistance +
                                               adc_half_length);

      if (nullptr != successor_lane) {
        // AINFO << "successor_lane = " << successor_lane->lane().id().id();
        if (successor_lane->lane().has_is_diagonal_road() &&
            successor_lane->lane().is_diagonal_road()) {
          // AINFO << "NEED  DIAGONAL";
          is_in_diagonal = true;
          // AINFO << "adc reach in diagonal road";
        }
      }
    }
    if (no_use_locate_lane_heading) {
      double precessor_lane_heading =
          locate_lane->Heading(locate_lane->lane().length() * 0.5);
      double successor_lane_heading =
          locate_lane->Heading(locate_lane->lane().length() * 0.5);
      hdmap::LaneInfoConstPtr pressor_lane =
          reference_line_info_->LocateLaneInfo(adc_s - accumulate_s - 1.0);
      bool has_preceor = true;
      if (pressor_lane != nullptr) {
        precessor_lane_heading =
            pressor_lane->Heading(pressor_lane->lane().length() * 0.5);
        // AINFO << "pressor_lane = " << pressor_lane->lane().id().id();
        if (pressor_lane->lane().id().id() == locate_lane->lane().id().id()) {
          has_preceor = false;
        }
        // AINFO << "precessor_lane_heading = " << precessor_lane_heading;
      }
      hdmap::LaneInfoConstPtr successor_lane =
          reference_line_info_->LocateLaneInfo(
              adc_s - accumulate_s + locate_lane->lane().length() + 1.0);
      bool has_successor = true;
      if (nullptr != successor_lane) {
        successor_lane_heading =
            successor_lane->Heading(successor_lane->lane().length() * 0.5);
        // AINFO << "successor_lane = " << successor_lane->lane().id().id();
        if (successor_lane->lane().id().id() == locate_lane->lane().id().id()) {
          has_successor = false;
        }
        // AINFO << "successor_lane_heading = " << successor_lane_heading;
      }
      if (has_successor && has_preceor) {
        diagonal_lane_heading =
            (precessor_lane_heading + successor_lane_heading) * 0.5;
      } else if (!has_successor && has_preceor) {
        diagonal_lane_heading = precessor_lane_heading;
      } else if (has_successor && !has_preceor) {
        diagonal_lane_heading = successor_lane_heading;
      } else {
        diagonal_lane_heading = reference_line_info_->vehicle_state().heading();
        AERROR << "no get preceor or succeor lane";
      }
    }

    // AINFO << "diagonal_lane_heading = " << diagonal_lane_heading;
    // AINFO << "vehicle heading = "
    // << reference_line_info_->vehicle_state().heading();
    reference_line_info_->SetDiagonalRoadHeading(diagonal_lane_heading);
  }

  return is_in_diagonal;
}

common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const bool path_reusable,
    PathData* const final_path_data) {
  // skip piecewise_jerk_path_optimizer if reused path
  if (FLAGS_enable_skip_path_tasks && path_reusable) {
    return Status::OK();
  }
  // AINFO<<"count_need_use_large_lateral_speed_ =
  // "<<count_need_use_large_lateral_speed_;
  ADEBUG << "Plan at the starting point: x = " << init_point.path_point().x()
         << ", y = " << init_point.path_point().y()
         << ", and angle = " << init_point.path_point().theta();

  CheckBorrowInfo();
  is_diagonal_road_ = CheckIsDiagonalRoad();
  std::vector<PathData> candidate_path_data;
  is_can_exit_ = true;
  GetAllCandidatePathData(reference_line, init_point, *final_path_data,
                          &candidate_path_data);

  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

void PiecewiseJerkPathOptimizer::CheckBorrowInfo() {
  has_left_neighbor_lane_ = false;
  has_right_neighbor_lane_ = false;
  accept_to_left_borrow_ = false;
  accept_to_right_borrow_ = false;

  // check has neighbor lane
  // double get_left_neighbor_lane_width = 0.0;
  // double get_right_neighbor_lane_width = 0.0;
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  hdmap::Id get_neighbor_lane_id;
  if (reference_line_info_->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, adc_sl.end_s(),
          &get_neighbor_lane_id, &get_left_neighbor_lane_width_) ||
      reference_line_info_->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftReverse, adc_sl.end_s(),
          &get_neighbor_lane_id, &get_left_neighbor_lane_width_)) {
    has_left_neighbor_lane_ = true;
    // AINFO << "has_left_neighbor_lane";
  }
  if (reference_line_info_->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, adc_sl.end_s(),
          &get_neighbor_lane_id, &get_right_neighbor_lane_width_) ||
      reference_line_info_->GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightReverse, adc_sl.end_s(),
          &get_neighbor_lane_id, &get_right_neighbor_lane_width_)) {
    has_right_neighbor_lane_ = true;
    // AINFO << "has_right_neighbor_lane";
  }
  const auto& lane_width = reference_line_info_->GetLaneWidthBaseOnAdcCenter();
  double curr_lane_left_width = lane_width.first;
  double curr_lane_right_width = lane_width.second;
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double past_road_left_width = curr_lane_left_width;
  double past_road_right_width = curr_lane_right_width;
  if (!reference_line_info_->reference_line().GetRoadWidth(
          adc_sl.end_s(), &curr_road_left_width, &curr_road_right_width)) {
    // AWARN << "Failed to get lane width at s = " << curr_s;
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  get_left_neighbor_lane_width_ = curr_road_left_width - curr_lane_left_width;
  get_right_neighbor_lane_width_ =
      curr_road_right_width - curr_lane_right_width;
  // AINFO << "curr_road_left_width = " << curr_road_left_width;
  // AINFO << "get_left_neighbor_lane_width_ = " <<
  // get_left_neighbor_lane_width_;
  // AINFO << "curr_road_right_width = " << curr_road_right_width;
  // AINFO << "get_right_neighbor_lane_width_ = "
  //       << get_right_neighbor_lane_width_;
  // check can left or right borrow
  if (has_left_neighbor_lane_ &&
      (injector_->borrow_response().response_type() ==
       planning::ResponseType::ACCEPT)) {
    accept_to_left_borrow_ = true;
    //AINFO << "accept_to_left_borrow";
  }
  if (has_right_neighbor_lane_ &&
      (injector_->borrow_response().response_type() ==
       planning::ResponseType::ACCEPT)) {
    accept_to_right_borrow_ = true;
    //AINFO << "accept_to_right_borrow";
  }
}

void PiecewiseJerkPathOptimizer::GetAllCandidatePathData(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const PathData& final_path_data,
    std::vector<PathData>* const candidate_path_data) {
  auto planning_start_point =
      FLAGS_use_front_axe_center_in_path_planning
          ? InferFrontAxeCenterFromRearAxeCenter(init_point)
          : init_point;
  auto init_frenet_state = reference_line.ToFrenetFrame(planning_start_point);
  ADEBUG << "[init_frenet_state] s_condition: " << init_frenet_state.first[0]
         << ", " << init_frenet_state.first[1] << ", "
         << init_frenet_state.first[2]
         << "; l_condition: " << init_frenet_state.second[0] << ", "
         << init_frenet_state.second[1] << ", " << init_frenet_state.second[2];

  // AINFO << "init_frenet_state.second[1]  = " << init_frenet_state.second[1];
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  AINFO << "[Path] There are " << path_boundaries.size() << " path boundaries.";

  bool has_path_ok = false;
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    AINFO << "[Path] path label is: " << path_boundary.label();
    PrintFeasibleSpaceNearInitPoint(path_boundary, init_frenet_state);
    // for (size_t i = 0; i < path_boundary_size; ++i) {
    //   AWARN << "idx " << i
    //         << "; l_min = " << path_boundary.boundary().at(i).first
    //         << "; l_max = " << path_boundary.boundary().at(i).second;
    // }
    if (path_boundary_size < 2) {
      continue;
    }
    CHECK_GT(path_boundary_size, 1U);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;
    PathData path_data = final_path_data;

    bool res_opt =
        OptimizePath(reference_line, path_boundary, init_frenet_state, &opt_l,
                     &opt_dl, &opt_ddl, &path_data);
    if (!has_path_ok &&
        std::string::npos == path_boundary.label().find("fallback")) {
      has_path_ok = res_opt;
    }
    SaveLastSuccessPath(path_boundary, res_opt, opt_l, opt_dl, opt_ddl);

    if (IsValidOptimizeCondition(reference_line, path_boundary, res_opt, &opt_l,
                                 &opt_dl, &opt_ddl)) {
      SetValidPathData(reference_line, path_boundary, opt_l, opt_dl, opt_ddl,
                       &path_data, candidate_path_data);
    }
  }
  if (FLAGS_enable_replan_for_diagobal_change_to_normal &&
      !reference_line_info_->IsChangeLanePath()) {
    if(reference_line_info_->NeedDiagonal()){
      injector_->SetLastFrameNeedDiagonal(true);
      // AINFO<<"now need diagonal";

    }else{
      // AINFO<<"now no need diagonal";
      // AINFO<<"injector_->LastFrameNeedDiagonal() = "<<injector_->LastFrameNeedDiagonal();
      if(injector_->LastFrameNeedDiagonal()){
        injector_->SetLastFrameNeedDiagonal(false);
        reference_line_info_->SetNeedDiagonal(true);
        // AINFO<<"injector_->LastFrameNeedDiagonal() = "<<injector_->LastFrameNeedDiagonal();
        // AINFO<<"need to replan";
        injector_->SetNeedToReplan(true);
      }else{

      }
    }
  }
  bool is_need_backward_diagonal = IsNeedBackWardTodiagonal();
  //  AINFO << "is_need_backward_diagonal = " << is_need_backward_diagonal;
  reference_line_info_->SetIsInNeedToBackward(is_need_backward_diagonal);
  if (reference_line_info_->lane_change_path_reusable()) {
    candidate_path_data->push_back(reference_line_info_->path_data());
  }
  UpdateLoosenPathConstrainState(has_path_ok);
}

bool PiecewiseJerkPathOptimizer::IsNeedBackWardTodiagonal() {
  if(!FLAGS_enable_backward_path){
    // no open.
    injector_->need_back_ward_ = false;
    return false;
  }
  if (!reference_line_info_->NeedDiagonal()) {
    injector_->need_back_ward_ = false;
    return false;
  }
  if (injector_->borrow_response().response_type() !=
      planning::ResponseType::ACCEPT) {
    injector_->need_back_ward_ = false;
    return false;
  }
  if (!injector_->is_auto_state_) {
    injector_->need_back_ward_ = false;
    return false;
  }
  if (injector_->need_back_ward_) {
    return true;
  }

  double self_borrow_end_l = injector_->selfborrow_endstate;
  double fallback_end_l = injector_->fallback_endstate;
  double borrow_end_l = request_end_l_;
  auto adc_sl = reference_line_info_->AdcSlBoundary();
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;

  std::vector<int> diff_l = {std::fabs(self_borrow_end_l - center_l),
                             std::fabs(fallback_end_l - center_l),
                             std::fabs(borrow_end_l - center_l)};
  std::sort(diff_l.begin(), diff_l.end());
  double max_diff_l = diff_l.back();
  // AINFO << "max_diff_l = " << max_diff_l;
  // 25-0.4363 / 20-0.3491 15-0.2618
  double safe_s_distance =
      max_diff_l / std::tan(kMaxSteerAngle) + kBackwardDistanceBuffer;
  // AINFO << "safe_s_distance = " << safe_s_distance;

  const auto& path_decision = reference_line_info_->path_decision();
  const auto& indexed_obstacles = path_decision->obstacles();
  auto obstacle_list = indexed_obstacles.Items();
  double adc_v = std::fabs(injector_->vehicle_state()->linear_velocity());
  bool is_adc_stop = std::fabs(adc_v) < kMinSpeed;
  bool has_near_obs = false;
  double min_s = std::numeric_limits<double>::max();
  double min_l = std::min(center_l, request_end_l_);
  double max_l = std::max(center_l, request_end_l_);
  for (const auto* obstacle : obstacle_list) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    auto obs_sl = obstacle->PerceptionSLBoundary();
    // with quest_l have overlap.
    bool no_lat_overlap = (obs_sl.start_l() > max_l || obs_sl.end_l() < min_l);
    if (no_lat_overlap) {
      continue;
    }
    double lon_distance = obs_sl.start_s() - adc_sl.end_s();
    // AINFO << "obstacle = " << obstacle->Id()
    //       << "   lon_distance = " << lon_distance;
    // no conider back ob
    if (adc_sl.start_s() > obs_sl.end_s()) {
      continue;
    }
    if (lon_distance > safe_s_distance) {
      continue;
    } else {
      has_near_obs = true;
      if (lon_distance < min_s) {
        min_s = lon_distance;
      }
    }
  }

  // AINFO << "is_adc_stop -= " << is_adc_stop;
  // AINFO << "has_near_obs = " << has_near_obs;
  if (has_near_obs && is_adc_stop && injector_->back_ward_check_times_ >= 10) {
    injector_->need_back_ward_ = true;
    // AINFO << "injector_->need_back_ward_  = " << injector_->need_back_ward_;
    injector_->back_ward_distance_ = safe_s_distance - min_s;
    return true;
  }
  if (has_near_obs && is_adc_stop && injector_->back_ward_check_times_ < 10) {
    injector_->back_ward_check_times_++;
    injector_->back_ward_check_times_ =
        std::min(10, injector_->back_ward_check_times_);
  }

  if (injector_->need_back_ward_ && !has_near_obs) {
    injector_->back_ward_check_times_--;
    injector_->back_ward_check_times_ =
        std::max(0, injector_->back_ward_check_times_);
  }
  if (injector_->back_ward_check_times_ == 0) {
    injector_->need_back_ward_ = false;
  }
  // AINFO << "injector_->back_ward_distance_ = "
  //       << injector_->back_ward_distance_;
  // AINFO << "injector_->back_ward_check_times_ = "
  //       << injector_->back_ward_check_times_;
  // AINFO << "injector_->need_back_ward_  = " << injector_->need_back_ward_;
  return false;
}

bool PiecewiseJerkPathOptimizer::OptimizePath(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary,
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state,
    std::vector<double>* x, std::vector<double>* dx, std::vector<double>* ddx,
    PathData* const path_data) {
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  auto start_time = std::chrono::system_clock::now();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  const size_t kNumKnots = path_boundary.boundary().size();
  auto init_state_change = init_state;
  if (std::string::npos != path_boundary.label().find("left") ||
      std::string::npos != path_boundary.label().find("right")) {
    // todo:if routing backward.
    if (reference_line_info_->NeedDiagonal() &&
        !injector_->need_back_ward_) {
      auto adc_sl = reference_line_info_->AdcSlBoundary();
      double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
      double request_end_l = request_end_l_;
      if(util::IsLaneChange(injector_->planning_context())){
      request_end_l =  
        reference_line_info_->GetIsClearToChangeLane() ? 0.0 : center_l;
      }
      if (request_end_l - center_l > kConsiderLateralBuffer &&
          std::string::npos != path_boundary.label().find("left")) {
        count_need_use_large_lateral_speed_++;
        if (count_need_use_large_lateral_speed_ > 5) {
          init_state_change.second[1] =
              std::max(kInitLateralSpeed, init_state.second[1]);
        }

      } else if (request_end_l - center_l <= kConsiderLateralBuffer &&
                 std::string::npos != path_boundary.label().find("left")) {
        count_need_use_large_lateral_speed_ = 0;
      }
      if (center_l - request_end_l > kConsiderLateralBuffer &&
          std::string::npos != path_boundary.label().find("right")) {
        count_need_use_large_lateral_speed_++;
        if (count_need_use_large_lateral_speed_ > 5) {
          init_state_change.second[1] =
              std::min(-kInitLateralSpeed, init_state.second[1]);
        }

      } else if (center_l - request_end_l <= kConsiderLateralBuffer &&
                 std::string::npos != path_boundary.label().find("right")) {
        count_need_use_large_lateral_speed_ = 0;
      }
    }
  }
  const auto smooth_protection = EvaluateBoundaryDropSmoothProtection(
      config, path_boundary, init_state_change);
  const double path_x_ref_pre_smooth_time = config.path_x_ref_pre_smooth_time();
  const double lateral_derivative_bound =
      config.lateral_derivative_bound_default();
  if (smooth_protection.triggered) {
    ADEBUG << "[BoundaryDropSmoothProtection] ddl/dddx protection trigger, label="
           << path_boundary.label()
           << ", near_left=" << smooth_protection.near_left_boundary
           << ", near_right=" << smooth_protection.near_right_boundary
           << ", min_left_clearance="
           << smooth_protection.min_left_clearance
           << ", min_right_clearance="
           << smooth_protection.min_right_clearance
           << ", max_left_inward_jump="
           << smooth_protection.max_left_inward_jump
           << ", max_right_inward_jump="
           << smooth_protection.max_right_inward_jump;
  }
  PiecewiseJerkPathProblem piecewise_jerk_problem(
      kNumKnots, path_boundary.delta_s(), init_state_change.second);
  if (IsValidPathReference(reference_line, path_boundary,
                           &piecewise_jerk_problem, &end_state)) {
    path_data->set_is_optimized_towards_trajectory_reference(true);
  } else {
    end_state[0] =
        ComputeEndStateL(reference_line, path_boundary, init_state_change);
    // laneborrow selfborrow lanechange or adc no in center line.
    if ((injector_->borrow_response().response_type() ==
             planning::ResponseType::ACCEPT ||
         injector_->enable_self_borrow_ ||
         (util::IsLaneChange(injector_->planning_context())) || !AdcInLane())) {
      reference_line_info_->SetNeedDiagonal(true);
    }
      // only adc back to adc front+10.0 in turn lane, no enter diagonal state
      if(reference_line_info_->IsNearTurn()){
        reference_line_info_->SetNeedDiagonal(false);
      }
      auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
      double center_s =
          (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
      const auto& reference_point =
          reference_line_info_->reference_line().GetReferencePoint(center_s);
      double diff_heading = std::fabs(common::math::NormalizeAngle(
          (reference_point.heading() -
           reference_line_info_->vehicle_state().heading())));
      // if adc heading with ref heading has large diff,no diagonal.
      if (diff_heading > M_PI * 0.1) {
        reference_line_info_->SetNeedDiagonal(false);
      }
    
    if (is_diagonal_road_) {
      reference_line_info_->SetNeedDiagonal(true);
      reference_line_info_->SetIsInDiagonalRoad(true);
    }
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    SetSmoothXRef(path_boundary, &x_ref, path_x_ref_pre_smooth_time);

    double x_ref_weight = kMinXRefWeight;
    if(!reference_line_info_->NeedDiagonal()){
      x_ref_weight = kMinXRefWeight;
    }else{
      x_ref_weight = kLowXRefWeight;
    }
    if (util::IsLaneChange(injector_->planning_context()) &&
        (!(std::fabs(request_end_l_) > kMinL))) {
      x_ref_weight = kLowXRefWeight;
    }
    if (util::IsLaneChange(injector_->planning_context()) &&
        (reference_line_info_->IsNearTurn() ||
         !reference_line_info_->NeedDiagonal())) {
      x_ref_weight = kMinXRefWeight;
    }
    if (FLAGS_allow_narrow_pass && injector_->is_adc_in_gate_junction_ &&
        !util::IsLaneChange(injector_->planning_context()) &&
        (!(std::fabs(request_end_l_) > kMinL))) {
      x_ref_weight = kHighXRefWeight;
    }
    AINFO << "x_ref_weight = " << x_ref_weight;
      piecewise_jerk_problem.set_x_ref(x_ref_weight, std::move(x_ref));
  }
  if(!reference_line_info_->NeedDiagonal()){
  piecewise_jerk_problem.set_end_state_ref({10.0, 0.0, 0.0}, end_state);
  }else{
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  }
  auto w = GetPathOptimizeWeight(init_state_change);
  if (smooth_protection.triggered) {
    w[2] *= config.boundary_drop_protection_ddl_weight_factor();
    w[3] = std::min(w[3] * config.boundary_drop_protection_dddl_weight_factor(),
                    config.dddl_weight_max());
  }
  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
  piecewise_jerk_problem.set_x_bounds(path_boundary.boundary());
  std::vector<std::pair<double, double>> dx_bounds(
      kNumKnots,
      std::make_pair(-lateral_derivative_bound, lateral_derivative_bound));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  std::vector<std::pair<double, double>> ddl_bounds;
  ComputeddlBounds(path_boundary, &ddl_bounds);
  if (smooth_protection.triggered) {
    ApplyBoundaryDropProtectionToDdlBounds(
        config, path_boundary, init_state_change.second[2], &ddl_bounds);
  }
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddl_bounds));
  double dddx_bound = ComputeJerkBound(init_state_change);
  if (smooth_protection.triggered) {
    dddx_bound *= kBoundaryDropProtectionTightDddlBoundRatio;
    ADEBUG << "[BoundaryDropSmoothProtection] tighten dddx bound to "
           << dddx_bound;
  }
  piecewise_jerk_problem.set_dddx_bound(dddx_bound);

  const Status success = piecewise_jerk_problem.Optimize(kMaxOptimizeIter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success.ok()) {
    AERROR << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();
  return true;
}

bool PiecewiseJerkPathOptimizer::AdcInLane() {
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  const auto& reference_point =
      reference_line_info_->reference_line().GetReferencePoint(center_s);
  double ref_heading = reference_point.heading();
  double adc_heading = reference_line_info_->vehicle_state().heading();
double steer_percentage = reference_line_info_->vehicle_state().steering_percentage();
  double diff_heading =
      std::fabs(common::math::NormalizeAngle(ref_heading - adc_heading));
//  double diff_heading_between_adc_and_startpoint =
//      std::fabs(common::math::NormalizeAngle(
//          reference_line_info_->InitPointHeading() - adc_heading));
  // AINFO << " reference_line_info_->InitPointHeading() = "
  //       << reference_line_info_->InitPointHeading();
   AINFO << "center_l  = " << center_l;
  // AINFO << "diff_heading = " << diff_heading;
  // AINFO << "diff_heading_between_adc_and_startpoint = "
        // << diff_heading_between_adc_and_startpoint;
  double diff_center_l = 0.0;
  const auto& reference_line = reference_line_info_->reference_line();
  double adc_x = reference_line_info_->vehicle_state().x();
  double adc_y = reference_line_info_->vehicle_state().y();
  common::SLPoint path_reference_sl;
  if (reference_line.XYToSL(
          common::util::PointFactory::ToPointENU(adc_x, adc_y),
          &path_reference_sl)) {
    diff_center_l = path_reference_sl.l();
  }
  
  double consider_l = FLAGS_lateral_error*0.5;
  if (!util::IsLaneChange(injector_->planning_context()) &&
      !injector_->LastFrameNeedDiagonal() &&
      std::fabs(diff_center_l) > consider_l) {
        consider_l = FLAGS_lateral_error;
  }
  AINFO << "consider_l = " << consider_l;
  AINFO<<"injector_->LastFrameNeedDiagonal() = "<<injector_->LastFrameNeedDiagonal();
  bool is_in_ref_line = injector_->LastFrameNeedDiagonal() ? false : true;
  AINFO<<"is_in_ref_line = "<<is_in_ref_line;
  AINFO<<"diff_center_l = "<<diff_center_l;
  AINFO<<"steer_percentage = "<<steer_percentage;
  AINFO<<"diff_heading = "<<diff_heading;
  if(injector_->LastFrameNeedDiagonal()){
    is_in_ref_line = std::fabs(diff_center_l) < consider_l &&
                     std::fabs(steer_percentage) < kMinSteerPercentage &&
                     diff_heading < FLAGS_same_heading;
  }else{
    is_in_ref_line =
        std::fabs(diff_center_l) < consider_l || diff_heading > 0.03 * M_PI;
  }

  //    diff_heading_between_adc_and_startpoint < FLAGS_same_heading;
   AINFO << "is_in_ref_line = " << is_in_ref_line;
  if (is_in_ref_line) {
    return true;
  }

  return false;
}

void PiecewiseJerkPathOptimizer::SetSmoothXRef(
    const PathBoundary& path_boundary, std::vector<double>* const x_ref,
    const double path_x_ref_pre_smooth_time) {
  if (nullptr == x_ref || (*x_ref).empty()) {
    return;
  }
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  auto* merge_lane_lateral_l =
      injector_->planning_context()->mutable_merge_lane_lateral_l();
        const auto& adc_sl = reference_line_info_->AdcSlBoundary();
        if (std::string::npos != path_boundary.label().find("left") ||
            std::string::npos != path_boundary.label().find("right") ||
            std::string::npos != path_boundary.label().find("lanechange")) {
          double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
          double distance_l = adc_center_l - (*x_ref).at(0);
          double disstance_s = std::fabs(2.0 * distance_l);
          if (std::fabs(distance_l) > 2.0) {
            //  AINFO << "disstance_s = " << disstance_s;
            for (size_t j = 0; j < (*x_ref).size(); ++j) {
              if (j * path_boundary.delta_s() > disstance_s) {
                break;
              }
              if (distance_l > 0.0) {
                (*x_ref)[j] =
                   adc_center_l- j * path_boundary.delta_s() * 0.5 
                    ;
              } else {
                (*x_ref)[j] =
                    adc_center_l+ j * path_boundary.delta_s() * 0.5 
                    ;
              }
            }
            // for (size_t j = 0; j < (*x_ref).size(); ++j) {
            //   AINFO << "x_ref[" << j << "] = " << (*x_ref)[j];
            // }
          }
        }
  if (config.enable_merge_lane_shift_laneborrow() &&
      path_decider_status.will_pass_merge_lane_area() &&
      merge_lane_lateral_l->size() > 0) {
    size_t index = 0;
    for (size_t j = 0; j < (*x_ref).size(); ++j) {
      if ((j * path_boundary.delta_s()) <
          path_decider_status.merge_lane_remain_dis()) {
        continue;
      }

      if (index < merge_lane_lateral_l->size()) {
        (*x_ref)[j] = (*merge_lane_lateral_l)[index];
        ++index;
      } else {
        (*x_ref)[j] =
            (*merge_lane_lateral_l)[0] > 0.0
                ? std::fmin(-config.merge_lane_fix_end_state_shift_buffer(),
                            (*x_ref)[j])
                : std::fmax(config.merge_lane_fix_end_state_shift_buffer(),
                            (*x_ref)[j]);
      }
    }
    return;
  }

  double init_v = std::fmax(
      reference_line_info_->vehicle_state().linear_velocity(), kMinPreVelocity);
  double smooth_length = init_v * path_x_ref_pre_smooth_time;
  for (size_t i = 0;
       i < (*x_ref).size() && (i * path_boundary.delta_s()) < (smooth_length);
       ++i) {
    (*x_ref)[i] = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  }
}

bool PiecewiseJerkPathOptimizer::IsValidPathReference(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary,
    PiecewiseJerkPathProblem* const piecewise_jerk_problem,
    std::array<double, 3>* const end_state) {
  const auto& reference_path_data = reference_line_info_->path_data();
  const auto& config = config_.piecewise_jerk_path_optimizer_config();

  if (std::string::npos != path_boundary.label().find("regular") &&
      reference_path_data.is_valid_path_reference()) {
    size_t path_reference_size = reference_path_data.path_reference().size();
    size_t kNumKnots = path_boundary.boundary().size();
    std::vector<double> path_reference_l(kNumKnots, 0.0);
    // when path reference is ready
    for (size_t i = 0; i < path_reference_size && i < kNumKnots; ++i) {
      common::SLPoint path_reference_sl;
      reference_line.XYToSL(common::util::PointFactory::ToPointENU(
                                reference_path_data.path_reference().at(i).x(),
                                reference_path_data.path_reference().at(i).y()),
                            &path_reference_sl);
      path_reference_l[i] = path_reference_sl.l();
    }
    (*end_state)[0] = path_reference_l.back();
    // for non-path-reference part
    // weight_x_ref is set to default value, where
    // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
    std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
    // increase l weight for path reference part only
    const double peak_value = config.path_reference_l_weight();
    const double peak_value_x = 0.5 * static_cast<double>(path_reference_size) *
                                path_boundary.delta_s();
    for (size_t i = 0; i < path_reference_size; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * path_boundary.delta_s();
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
      ADEBUG << "i: " << i << ", weight: " << weight_x_ref_vec.at(i);
    }
    piecewise_jerk_problem->set_x_ref(std::move(weight_x_ref_vec),
                                      std::move(path_reference_l));
    return true;
  }
  return false;
}

double PiecewiseJerkPathOptimizer::ComputeEndStateL(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary,
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state) {
  double end_state_l = 0.0;
  if (0 && !FLAGS_enable_force_pull_over_open_space_parking_test) {
    end_state_l = ComputePullOverEndStatel(reference_line, path_boundary);
  }

  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  const auto& config = config_.piecewise_jerk_path_optimizer_config();

  if ((reference_line_info_->IsChangeLanePath() &&
       util::IsLaneChange(injector_->planning_context())) ||
      util::IsOvertake(injector_->planning_context()) ||
      util::IsNewRouting(injector_->planning_context()) ||
      injector_->is_adc_in_gate_junction_) {
    end_state_l =
        reference_line_info_->GetIsClearToChangeLane() ? 0.0 : center_l;
    ADEBUG << "end_state_l = " << end_state_l;
    self_borrow_l_ =
        util::IsNewRouting(injector_->planning_context()) ? center_l : 0.0;
    if (injector_->is_adc_in_gate_junction_) {
      self_borrow_l_ = 0.0;
    }
    real_self_borrow_l_ = 0.0;
    injector_->selfborrow_endstate = 0.0;
    injector_->left_laneborrow_endstate = 0.0;
    injector_->right_laneborrow_endstate = 0.0;
    injector_->fallback_endstate = 0.0;
    injector_->last_using_lateral_ = self_borrow_l_;
    if (injector_->is_adc_in_gate_junction_) {
      end_state_l = 0.0;
    }
    if (is_diagonal_road_) {
      end_state_l = 0.0;
    }
    AINFO<<"end_state_l = "<<end_state_l;
    if (std::string::npos != path_boundary.label().find("fallback")) {
      end_state_l =
          std::fabs(reference_line_info_->GetAdcHeadingDiffWithRefLine()) <
                  config.smooth_l_heading_diff_threshold()
              ? center_l
              : 0.0;


      injector_->fallback_endstate = end_state_l;
    }
  } else {
    if (std::string::npos != path_boundary.label().find("self") ||
        (!reference_line_info_->IsChangeLanePath() &&
         util::IsLaneChange(injector_->planning_context()))) {
      end_state_l = GetSelfPathEndStateL(path_boundary, init_state);
      AINFO << "<<self>> end state l: " << end_state_l;
    } else if (std::string::npos != path_boundary.label().find("fallback")) {
      if (reference_line_info_->GetLaneType() != hdmap::Lane::PLAY_STREET) {
        // no consider selfborrow
        if (util::IsLaneBorrow(injector_->planning_context()) ||
            !reference_line_info_->IsAdcOnReferenceLine()) {
          end_state_l = reference_line_info_->GetIsClearToChangeLane()
                            ? center_l * config.fallback_smooth_ratio()
                            : center_l;
        } else {
          if (injector_->last_using_lateral_ < 0.0) {
            end_state_l =
                std::min(injector_->last_using_lateral_ + kMinDeltaL, 0.0);
          } else {
            end_state_l =
                std::max(injector_->last_using_lateral_ - kMinDeltaL, 0.0);
          }
        }
         AINFO << "end_state_l = = " << end_state_l;
      } else {
        double width =
            common::VehicleConfigHelper::GetConfig().vehicle_param().width();
        double curr_lane_left_width =
            reference_line_info_->GetLaneWidthBaseOnAdcCenter().first;
        // use road width
        end_state_l = std::min(
            get_left_neighbor_lane_width_ + curr_lane_left_width - width * 0.5,
            self_borrow_l_);
      }
      if(is_diagonal_road_){
        end_state_l = 0.0;
      }
      injector_->fallback_endstate = end_state_l;
      AINFO << "<<fallback>> end state l: " << end_state_l;
    }
  }

  // --------for laneborrow end_state_l-----------
  AINFO << "end_state_l = " << end_state_l;
  GetLaneBorrowEndState(path_boundary, &end_state_l);
  return end_state_l;
}

double PiecewiseJerkPathOptimizer::ComputePullOverEndStatel(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary) {
  // pull over scenario
  // set end lateral to be at the desired pull over destination
  double pull_over_end_state_l = 0.0;
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y() &&
      std::string::npos != path_boundary.label().find("pullover")) {
    common::SLPoint pull_over_sl;
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    pull_over_end_state_l = pull_over_sl.l();
  }
  return pull_over_end_state_l;
}

std::array<double, 5> PiecewiseJerkPathOptimizer::GetPathOptimizeWeight(
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state) {
  // Choose lane_change_path_config for lane-change cases
  // Otherwise, choose default_path_config for normal path planning
  const auto* config = config_.mutable_piecewise_jerk_path_optimizer_config()
                           ->mutable_default_path_config();
  bool use_common_config = false;
  if (reference_line_info_->IsChangeLanePath()) {
    config = config_.mutable_piecewise_jerk_path_optimizer_config()
                 ->mutable_lane_change_path_config();
  } else {
    use_common_config = true;
  }

  std::array<double, 5> weight = {
      config->l_weight(),
      config->dl_weight() *
          std::fmax(init_state.first[1] * init_state.first[1], 5.0),
      config->ddl_weight(), config->dddl_weight(), 0.0};
  hdmap::Lane_LaneType lane_type = reference_line_info_->GetLaneType();
  if (hdmap::Lane::PLAY_STREET != lane_type) {
    weight[3] = config->dddl_weight_for_cityroad();
    const auto& qp_config = config_.piecewise_jerk_path_optimizer_config();
    double init_v =
        std::max(reference_line_info_->vehicle_state().linear_velocity(), 0.0);
    if (use_common_config && IsSpecialSceneForSelfBorrow() &&
        init_v < qp_config.adc_slow_drive_threshold()) {
      weight[3] = qp_config.dddl_weight_min();
    }
  }
  return weight;
}

void PiecewiseJerkPathOptimizer::ComputeddlBounds(
    const PathBoundary& path_boundary,
    std::vector<std::pair<double, double>>* const ddl_bounds) {
  double max_curve_base_radius = 1.0 / min_turn_radius;
  const double lat_acc_bound = max_curve_base_radius;
  for (size_t i = 0; i < path_boundary.boundary().size(); ++i) {
    double s = static_cast<double>(i) * path_boundary.delta_s() +
               path_boundary.start_s();
    double kappa = reference_line_info_->reference_line()
                       .GetNearestReferencePoint(s)
                       .kappa();
    //  AINFO<<"kappa_min = "<<-lat_acc_bound - kappa<<"      kappa_max = "<<lat_acc_bound - kappa;
    ddl_bounds->emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
  }
}

double PiecewiseJerkPathOptimizer::ComputeJerkBound(
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state) {
  double init_v =
      std::max(reference_line_info_->vehicle_state().linear_velocity(), 0.0);
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  double max_yaw_rate_coeff = common::math::lerp(
      config.yaw_rate_max_coeff(), config.adc_min_speed(),
      config.yaw_rate_min_coeff(), config.adc_max_speed(), init_v);
  max_yaw_rate_coeff =
      std::fmin(config.yaw_rate_max_coeff(), max_yaw_rate_coeff);
  max_yaw_rate_coeff =
      std::fmax(config.yaw_rate_min_coeff(), max_yaw_rate_coeff);
  if (util::IsNeedLoosenPathConstrains(injector_->planning_context())) {
    max_yaw_rate_coeff = config.yaw_rate_max_coeff();
  }
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base() * 0.5;
  const double max_yaw_rate = veh_param.max_steer_angle_rate() /
                              veh_param.steer_ratio();
  // add coeff may cause boudnary small.
  ADEBUG<<"max_yaw_rate_coeff = "<<max_yaw_rate_coeff;
  
  // The greater the lateral velocity, the smaller the boundary of the
  // acceleration. So the same as the speed, the greater the angle between the
  // heading and the reference line, the greater the lateral velocity, the
  // smaller the boundary of the lateral acceleration;    the
  // max_lateral_speed is 1 m/s maybe need to change for fixing the century
  // AINFO<<"EstimateJerkBoundary(std::fmax(init_state.second[1], 0.5)="<<EstimateJerkBoundary(std::fmax(init_state.second[1], 1.0),
  //                             axis_distance, max_yaw_rate);
  // AINFO<<"max_yaw_rate = "<<max_yaw_rate;   
  // AINFO<<"std::fmax(init_state.second[1], 1.0) = "<<std::fmax(init_state.second[1], 0.5);     
  // AINFO<<"axis_distance = "<<axis_distance;
  // AINFO<<"JERK = "<<max_yaw_rate / axis_distance / (std::fmax(init_state.second[1], 0.5));
  // use lateral speed ,no use lon speed
  return EstimateJerkBoundary(std::fmax(init_state.second[1], 0.5),
                              axis_distance, max_yaw_rate);
}

void PiecewiseJerkPathOptimizer::SaveLastSuccessPath(
    const PathBoundary& path_boundary, const bool res_opt,
    const std::vector<double>& opt_l, const std::vector<double>& opt_dl,
    const std::vector<double>& opt_ddl) {
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  if (!res_opt || !config.reuse_last_optimized_path()) {
    return;
  }

  auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  if (std::string::npos != path_boundary.label().find("self")) {
    last_self_success_point_ =
        common::math::Vec2d(vehicle_state.x(), vehicle_state.y());
    last_self_success_opt_l_ = opt_l;
    last_self_success_opt_dl_ = opt_dl;
    last_self_success_opt_ddl_ = opt_ddl;
    ADEBUG << "[SaveLastSuccessPath] SELF path";
  } else if (std::string::npos != path_boundary.label().find("lanechange")) {
    last_lanechange_success_point_ =
        common::math::Vec2d(vehicle_state.x(), vehicle_state.y());
    last_lanechange_success_opt_l_ = opt_l;
    last_lanechange_success_opt_dl_ = opt_dl;
    last_lanechange_success_opt_ddl_ = opt_ddl;
    ADEBUG << "[SaveLastSuccessPath] LaneChange path";

    auto* lane_change_status = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_change_lane();
    if (!lane_change_status->is_success_change_lane_path()) {
      lane_change_status->set_is_success_change_lane_path(
          lane_change_status->is_clear_to_change_lane() &&
          !lane_change_status->exist_lane_change_start_position());
    }
  }
}

bool PiecewiseJerkPathOptimizer::IsValidOptimizeCondition(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary,
    const bool res_opt, std::vector<double>* const opt_l,
    std::vector<double>* const opt_dl, std::vector<double>* const opt_ddl) {
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  if (res_opt || !config.reuse_last_optimized_path()) {
    return res_opt;
  }
  if (std::string::npos != path_boundary.label().find("self") &&
      reference_line_info_->IsAdcCenterInLane() &&
      !injector_->GetReplanState() &&
      VaildPathLable::SELF == injector_->last_path_label_ &&
      !(util::IsLaneChange(injector_->planning_context()))) {
    AINFO << "[check reuse] last self path";
    ReuseLastSuccessPathData(
        reference_line, path_boundary.delta_s(), last_self_success_point_,
        last_self_success_opt_l_, last_self_success_opt_dl_,
        last_self_success_opt_ddl_, opt_l, opt_dl, opt_ddl);
  }
  if (std::string::npos != path_boundary.label().find("lanechange") &&
      VaildPathLable::LANE_CHANGE == injector_->last_path_label_) {
    AINFO << "[check reuse] last lanechange path";
    ReuseLastSuccessPathData(
        reference_line, path_boundary.delta_s(), last_lanechange_success_point_,
        last_lanechange_success_opt_l_, last_lanechange_success_opt_dl_,
        last_lanechange_success_opt_ddl_, opt_l, opt_dl, opt_ddl);
  }

  return (!opt_l->empty() && !opt_dl->empty() && !opt_ddl->empty());
}

void PiecewiseJerkPathOptimizer::ReuseLastSuccessPathData(
    const ReferenceLine& reference_line, const double path_delta_s,
    const common::math::Vec2d& last_success_point,
    const std::vector<double>& last_success_opt_l,
    const std::vector<double>& last_success_opt_dl,
    const std::vector<double>& last_success_opt_ddl,
    std::vector<double>* const opt_l, std::vector<double>* const opt_dl,
    std::vector<double>* const opt_ddl) {
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  double last_success_s = 0.0;
  double last_success_l = 0.0;
  double current_ADC_s = 0.0;
  double current_ADC_l = 0.0;

  common::math::Vec2d current_ADC_point(vehicle_state.x(), vehicle_state.y());
  const auto& map_path = reference_line.map_path();
  map_path.GetNearestPoint(last_success_point, &last_success_s,
                           &last_success_l);
  map_path.GetNearestPoint(current_ADC_point, &current_ADC_s, &current_ADC_l);
  double delta_s = current_ADC_s - last_success_s;
  size_t idx =
      static_cast<size_t>(round(delta_s / std::fmax(path_delta_s, kEpsilon)));

  if (delta_s < config.max_reuse_last_path_length() &&
      last_success_opt_l.size() > idx && last_success_opt_dl.size() > idx &&
      last_success_opt_ddl.size() > idx) {
    auto start_opt_l = last_success_opt_l.begin();
    auto start_opt_dl = last_success_opt_dl.begin();
    auto start_opt_ddl = last_success_opt_ddl.begin();
    std::advance(start_opt_l, idx);
    std::advance(start_opt_dl, idx);
    std::advance(start_opt_ddl, idx);
    opt_l->assign(start_opt_l, last_success_opt_l.end());
    opt_dl->assign(start_opt_dl, last_success_opt_dl.end());
    opt_ddl->assign(start_opt_ddl, last_success_opt_ddl.end());
    AINFO << "[Reuse last Path], delta_s: " << delta_s << ", diff_idx: " << idx
          << ", opt_l size: " << opt_l->size();
  }
  return;
}

void PiecewiseJerkPathOptimizer::SetValidPathData(
    const ReferenceLine& reference_line, const PathBoundary& path_boundary,
    const std::vector<double>& opt_l, const std::vector<double>& opt_dl,
    const std::vector<double>& opt_ddl, PathData* const path_data,
    std::vector<PathData>* const candidate_path_data) {
  auto frenet_frame_path = ToPiecewiseJerkPath(
      opt_l, opt_dl, opt_ddl, path_boundary.delta_s(), path_boundary.start_s());

  path_data->SetReferenceLine(&reference_line);
  path_data->SetFrenetPath(std::move(frenet_frame_path));
  if (FLAGS_use_front_axe_center_in_path_planning) {
    auto discretized_path =
        DiscretizedPath(ConvertPathPointRefFromFrontAxeToRearAxe(*path_data));
    path_data->SetDiscretizedPath(std::move(discretized_path));
  }
  path_data->set_path_label(path_boundary.label());
  path_data->set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
  candidate_path_data->push_back(std::move(*path_data));
}

FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());
  CHECK_NE(delta_s, 0.0);

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

common::TrajectoryPoint
PiecewiseJerkPathOptimizer::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base() * 0.5;
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

std::vector<common::PathPoint>
PiecewiseJerkPathOptimizer::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base() * 0.5;
  for (auto path_point : path_data.discretized_path()) {
    common::PathPoint new_path_point = path_point;
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));
    ret.push_back(new_path_point);
  }
  return ret;
}

double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  CHECK_NE(axis_distance, 0.0);
  CHECK_NE(vehicle_speed, 0.0);
  return max_yaw_rate / axis_distance / vehicle_speed;
}

double PiecewiseJerkPathOptimizer::GaussianWeighting(
    const double x, const double peak_weighting,
    const double peak_weighting_x) const {
  CHECK_NE(peak_weighting, 0.0);
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  ADEBUG << peak_weighting *
                exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x));
  ADEBUG << Gaussian(u, std, x_updated);
  return Gaussian(u, std, x_updated);
}
bool PiecewiseJerkPathOptimizer::IsSafeToLaneborrow(
    const bool is_turn_left) const {
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double ego_start_s = adc_sl_boundary.start_s();
  double ego_end_s = adc_sl_boundary.end_s();
  double ego_v = std::abs(injector_->vehicle_state()->linear_velocity());

  // get neighbor lane width
  double neighbor_lane_width = 0;

  // get self lane width
  const auto& lane_width = reference_line_info_->GetLaneWidthBaseOnAdcCenter();
  double left_width_self = lane_width.first;
  double right_width_self = lane_width.second;

  for (const auto* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }
    if (obstacle->IsVirtual()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    // skip static obstacle
    if (obstacle->IsStatic()) {
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (is_turn_left) {  // check the left lane obstacle
      if (obstacle_sl.start_l() > (left_width_self + neighbor_lane_width) ||
          obstacle_sl.end_l() < left_width_self) {
        continue;
      }
    } else {  // check the right lane obstacle
      if (obstacle_sl.end_l() < -right_width_self ||
          obstacle_sl.start_l() > right_width_self) {
        continue;
      }
    }

    // judge the directiion of the obstacle.
    bool same_direction = true;
    double obstacle_moving_direction = 0.0;
    if (!obstacle->HasTrajectory()) {
      obstacle_moving_direction = obstacle->Perception().theta();
    } else {
      obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
    }
    double vehicle_moving_direction = injector_->vehicle_state()->heading();
    double heading_difference = std::abs(common::math::NormalizeAngle(
        obstacle_moving_direction - vehicle_moving_direction));
    same_direction = heading_difference < kSameDirectionThr;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    // set a safe distance based on direction
    if (same_direction) {
      // Same direction, calculate the safe distance in front of ADC:
      // 1. ADC speed is higher, the ForwardSafeDistance is the maximum value in
      //    a constant and relative moving distance.
      // 2. ADC speed is lower, the ForwardSafeDistance is a constant.
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * (kSafeTimeOnSameDirection));
      // Same direction, calculate the safe distance behind ADC:
      // 1. ADC speed is higher, the BackwardSafeDistance is a constant.
      // 2. ADC speed is lower, the BackwardSafeDistance is the maximum value in
      //    a constant and relative moving distance.
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * (kSafeTimeOnSameDirection));
    } else {
      // Opposite direction, calculate the safe distance in front of ADC:
      // the ForwardSafeDistance is the maximum value in a constant and relative
      // moving distance.
      kForwardSafeDistance = std::fmax(
          kForwardMinSafeDistanceOnOppositeDirection,
          (ego_v + obstacle->speed()) * (kSafeTimeOnOppositeDirection));
      // Opposite direction, calculate the safe distance behind ADC:
      // the BackwardSafeDistance is a constant value.
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    if ((ego_start_s - obstacle_sl.end_s() < kBackwardSafeDistance) &&
        (obstacle_sl.start_s() - ego_end_s) < kForwardSafeDistance) {
      return false;
    }
  }
  return true;
}

bool PiecewiseJerkPathOptimizer::SafeReturnCheck(
    const double request_end_state_l) const {
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_center_l =
      (adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5;
  double ego_start_s = adc_sl_boundary.start_s();
  // double ego_end_s = adc_sl_boundary.end_s();
  // double ego_v =
  //     std::fabs(reference_line_info_->vehicle_state().linear_velocity());
  // const auto& veh_param =
  //     common::VehicleConfigHelper::GetConfig().vehicle_param();
  // double adc_half_width = veh_param.width() * 0.5;
  if (std::fabs(request_end_state_l - adc_center_l) < kMinDiffL) {
    // return true;
  }
  bool is_left_self_borrow_or_right_return =
      (request_end_state_l > adc_center_l);
  bool is_right_self_borrow_or_left_return =
      (request_end_state_l < adc_center_l);
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  bool is_in_left_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_LEFTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
  bool is_in_right_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_RIGHTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
  // AINFO << "is_in_right_borrow = " << is_in_right_borrow;
  bool is_left_return =
      is_in_left_borrow && is_right_self_borrow_or_left_return;
  bool is_right_return =
      is_in_right_borrow && is_left_self_borrow_or_right_return;
  // AINFO << "is_left_return = " << is_left_return;
  // AINFO << "is_right_return = " << is_right_return;
  bool is_borrow_obs_change = false;
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  if (is_left_return || is_right_return) {
    for (const auto* obstacle :
         reference_line_info_->path_decision()->obstacles().Items()) {
      if (!obstacle) {
        AERROR << "input obstacle is nullptr!";
        continue;
      }
      // AINFO << "obstacle->Id() = " << obstacle->Id();
      const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
      // AINFO << "obstacle_sl.end_l() = " << obstacle_sl.end_l();
      // AINFO << "mutable_path_decider_status->front_static_obstacle_id() = "
      //       << mutable_path_decider_status->front_static_obstacle_id();
      if(obstacle_sl.end_s()+ FLAGS_distance_borrow_return <adc_sl.start_s()){
        // AINFO << "obstacle is behind adc!,no conider";
        continue;
      }
      if (obstacle->Id().find(injector_->borro_obs_name_) !=
              std::string::npos ||
          injector_->borro_obs_name_ == obstacle->Id() ||
          mutable_path_decider_status->front_static_obstacle_id() ==
              obstacle->Id()) {
        if (injector_->borro_obs_name_ !=
            mutable_path_decider_status->front_static_obstacle_id()) {
          injector_->borro_obs_name_ =
              mutable_path_decider_status->front_static_obstacle_id();
        }
        bool is_right_obs = obstacle_sl.end_l() + 0.5 < adc_sl.start_l();
        bool is_left_obs = obstacle_sl.start_l() - 0.5 > adc_sl.end_l();
        bool is_no_overlap_with_adc = (is_right_obs || is_left_obs);
        if (obstacle_sl.start_s() - adc_sl.end_s() < 40.0 &&
            obstacle_sl.start_s() - adc_sl.end_s() > 0.0 &&
            !is_no_overlap_with_adc) {
          AINFO << "fornt has obs,need return!";
          is_borrow_obs_change = false;
          break;
        }
        if(injector_->borro_obs_name_.empty()){
          continue;
        }
        if (!obstacle->IsStatic()) {
          const auto refer_point =
              reference_line_info_->reference_line().GetNearestReferencePoint(
                  obstacle_sl.end_s());
          double adc_ref_heading = refer_point.heading();

          // AINFO << "adc_ref_heading = " << adc_ref_heading;
          // AINFO << "obstacle->SpeedHeading() = " << obstacle->SpeedHeading();
          double diff_heading = common::math::NormalizeAngle(
              obstacle->SpeedHeading() - adc_ref_heading);
          double diff_angle = std::fabs(diff_heading);
          if (((is_left_return && is_right_obs) ||
               (is_right_return && is_left_obs)) &&
              diff_angle < 0.785) {
            // AINFO << "same direction obstacle!";
            continue;
          }
        }
        if (ego_start_s - obstacle_sl.end_s() > FLAGS_distance_borrow_return) {
          // AINFO << "passed the borrow obs!";
          continue;
        }
        if (is_left_return &&
            injector_->borrow_obs_consider_l_ - obstacle_sl.end_l() > 0.5) {
          is_borrow_obs_change = true;
          break;
        }
        if (is_right_return &&
            obstacle_sl.start_l() - injector_->borrow_obs_consider_l_ > 0.5) {
          is_borrow_obs_change = true;
          break;
        }
      }
    }
  }
  // AINFO << "injector_->borrow_obs_consider_l_ = "
  //       << injector_->borrow_obs_consider_l_;
  // AINFO << "is_borrow_obs_change = " << is_borrow_obs_change;
  // AINFO << "injector_->borro_obs_name_ = " << injector_->borro_obs_name_;
  if (is_borrow_obs_change) {
    return false;
  }

  return true;
}

bool PiecewiseJerkPathOptimizer::SelfBorrowSafeCheck(
    const double request_end_state_l) const {
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_center_l =
      (adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5;
  double ego_start_s = adc_sl_boundary.start_s();
  double ego_end_s = adc_sl_boundary.end_s();
  double ego_v =
      std::fabs(reference_line_info_->vehicle_state().linear_velocity());
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double adc_half_width = veh_param.width() * 0.5;
  if (std::fabs(request_end_state_l - adc_center_l) < kMinDiffL) {
    return true;
  }
  bool is_left_self_borrow_or_right_return =
      (request_end_state_l > adc_center_l);
  bool is_right_self_borrow_or_left_return =
      (request_end_state_l < adc_center_l);
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  bool is_in_left_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_LEFTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
  bool is_in_right_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_RIGHTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  for (const auto* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (!obstacle) {
      AERROR << "check lane change safe, but input obstacle is nullptr!";
      continue;
    }
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle->IsStatic() || obstacle->speed() < kMinSpeed) {
      if (adc_sl_boundary.start_s() - FLAGS_distance_borrow_return <
              obstacle_sl.end_s() &&
          adc_sl_boundary.end_s() > obstacle_sl.end_s()) {
        double min_lat = request_end_state_l - adc_half_width;
        double max_lat = request_end_state_l + adc_half_width;
        bool no_lateral_overlap =
            obstacle_sl.end_l() < min_lat || obstacle_sl.start_l() > max_lat;
        if (!no_lateral_overlap && is_in_left_borrow &&
            is_right_self_borrow_or_left_return) {
          AINFO << "NO SAFE RETURN";
          return false;
        }
        if (!no_lateral_overlap && is_in_right_borrow &&
            is_left_self_borrow_or_right_return) {
          AINFO << "NO SAFE RETURN";
          return false;
        }
      }
    }
    // no consider static obs
    if (obstacle->IsVirtual() || obstacle->IsStatic() ||
        obstacle->speed() < kMinSpeed) {
      continue;
    }

    if (is_left_self_borrow_or_right_return) {
      // only consider left side obs
      if (obstacle_sl.start_l() - kLateralBuffer >
              request_end_state_l + adc_half_width ||
          obstacle_sl.end_l() < adc_center_l) {
        continue;
      }
    }
    // AINFO << "is_right_self_borrow_or_left_return = "
    //       << is_right_self_borrow_or_left_return;
    if (is_right_self_borrow_or_left_return) {
      // only consider right side obs
      if (obstacle_sl.start_l() > adc_center_l ||
          obstacle_sl.end_l() + kLateralBuffer <
              request_end_state_l - adc_half_width) {
        continue;
      }
    }

    // check range obs.

    // Raw estimation on whether same direction with ADC or not based on
    // prediction trajectory
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction = obstacle->SpeedHeading();
      // obstacle->Trajectory().trajectory_point(0).path_point().theta();
      //
      double vehicle_moving_direction =
          reference_line_info_->vehicle_state().heading();
      // TODO(zongxingguo): only consider in reverse type ,no in backward type.
      // reverse type no consider lanechange.
      // if (vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
      //   vehicle_moving_direction =
      //       common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      // }
      double heading_difference = std::fabs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < kSameDirectionThr;
    }

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    // AINFO << "same_direction = " << same_direction;
    //  set a safe distance based on direction
    if (same_direction) {
      // Same direction, calculate the safe distance in front of ADC:
      // 1. ADC speed is higher, the ForwardSafeDistance is the maximum value in
      //    a constant and relative moving distance.
      // 2. ADC speed is lower, the ForwardSafeDistance is a constant.
      kForwardSafeDistance = std::fmax(
          kForwardMinSafeDistanceOnSameDirection,
          (ego_v - obstacle->speed()) *
              (kSafeTimeOnSameDirection + FLAGS_lane_change_total_time));
      // Same direction, calculate the safe distance behind ADC:
      // 1. ADC speed is higher, the BackwardSafeDistance is a constant.
      // 2. ADC speed is lower, the BackwardSafeDistance is the maximum value in
      //    a constant and relative moving distance.
      kBackwardSafeDistance = std::fmax(
          kBackwardMinSafeDistanceOnSameDirection,
          (obstacle->speed() - ego_v) *
              (kSafeTimeOnSameDirection + FLAGS_lane_change_total_time));
    } else {
      // Opposite direction, calculate the safe distance in front of ADC:
      // the ForwardSafeDistance is the maximum value in a constant and relative
      // moving distance.
      kForwardSafeDistance = std::fmax(
          kForwardMinSafeDistanceOnOppositeDirection,
          (ego_v + obstacle->speed()) *
              (kSafeTimeOnOppositeDirection + FLAGS_lane_change_total_time));
      // Opposite direction, calculate the safe distance behind ADC:
      // the BackwardSafeDistance is a constant value.
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }
    // AINFO << "kForwardSafeDistance = " << kForwardSafeDistance;
    // AINFO << "kBackwardSafeDistance = " << kBackwardSafeDistance;
    // Equation of motion: v^2 / 2a
    if (obstacle->speed() < FLAGS_static_unknown_obstacle_speed_threshold) {
      kForwardSafeDistance =
          std::fmax(kForwardSafeDistance,
                    std::fabs((ego_v * ego_v) /
                              (FLAGS_slowdown_profile_deceleration * 2.0)) +
                        kMinSafeReactionDistance);
      kBackwardSafeDistance =
          std::fmax(0.0, kMinSafeReactionDistance -
                             ego_v * LaneChangeLateralTTC(*obstacle));
    }

    if (HysteresisFilter(ego_start_s - obstacle_sl.end_s(),
                         kBackwardSafeDistance, kDistanceBuffer) &&
        HysteresisFilter(obstacle_sl.start_s() - ego_end_s,
                         kForwardSafeDistance, kDistanceBuffer)) {
      // AINFO << "back dis: " << kBackwardSafeDistance
      //       << ", front_dis: " << kForwardSafeDistance
      //       << ", is static: " << obstacle->IsStatic()
      //       << ", same direction: " << same_direction << ", ego_v: " << ego_v
      //       << ", obs_v: " << obstacle->speed()
      //       << ", obs id: " << obstacle->Id()
      //       << ", obs_v: " << obstacle->speed()
      //       << ", obs sl: [s: " << obstacle_sl.start_s() << "~"
      //       << obstacle_sl.end_s() << "] [l:" << obstacle_sl.start_l() << "~"
      //       << obstacle_sl.end_l() << "]";
      return false;
    }
  }
  return true;
}

bool PiecewiseJerkPathOptimizer::HysteresisFilter(
    const double obstacle_distance, const double safe_distance,
    const double distance_buffer) const {
  return obstacle_distance < safe_distance + distance_buffer;
}
double PiecewiseJerkPathOptimizer::LaneChangeLateralTTC(
    const Obstacle& obstacle) const {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double lateral_diff = std::fmax(adc_sl_boundary.start_l() - obs_sl.end_l(),
                                  obs_sl.start_l() - adc_sl_boundary.end_l());
  if (lateral_diff <= FLAGS_obstacle_max_lat_buffer_public_road) {
    return 0.0;
  }
  return (lateral_diff - FLAGS_obstacle_max_lat_buffer_public_road) /
         std::fmax(kEpsilon,
                   std::fabs(reference_line_info_->GetAdcLateralVelocity()));
}

bool PiecewiseJerkPathOptimizer::IsSafeToLeftborrow(
    const SLBoundary leftborrow_obs) const {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double adc_v = std::fabs(injector_->vehicle_state()->linear_velocity());

  for (const auto* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }
    if (obstacle->IsVirtual()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle->IsStatic() ||
        obstacle->speed() < FLAGS_borrow_slow_obstacle_velocity_threshold) {
      if (reference_line_info_->IsAdcLocatedInLane() &&
          util::WithLeftObsSideBySide(obstacle_sl, adc_sl_boundary) &&
          adc_v > kAdcStopVelocityThreshold) {
        AINFO << "has left sidebyside obs, unsafe left borrow, obs id: "
              << obstacle->Id();
        return false;
      }
    } else if ((obstacle_sl.start_s() < leftborrow_obs.start_s() &&
                obstacle_sl.end_s() > leftborrow_obs.start_s() -
                                          veh_param.length() -
                                          kBlockBorrowObsLonBuffer) ||
               (obstacle_sl.end_s() > leftborrow_obs.end_s() &&
                obstacle_sl.start_s() < leftborrow_obs.end_s() +
                                            veh_param.length() +
                                            kBlockBorrowObsLonBuffer)) {
      // AINFO << "leftborrow_obs.start_s()  = " << leftborrow_obs.start_s();
      // AINFO << " obstacle_sl.end_s() = " << obstacle_sl.end_s();
      // AINFO << "obstacle->LongitudinalSpeed() = "
      //       << obstacle->LongitudinalSpeed() << "         " <<
      //       obstacle->Id();
      if (obstacle_sl.end_l() > leftborrow_obs.end_l() &&
          (obstacle_sl.start_l() - leftborrow_obs.end_l() <
               veh_param.width() + kBlockBorrowObsLatBuffer * 2.0 &&
           obstacle->LongitudinalSpeed() > 0.0)) {
        AINFO << "Barriers for containment, obstacle id: " << obstacle->Id();
        // if static obs front has dynamic may cause no borrow.
        //return false;
      }
    }
  }
  return true;
}

// determine whether the left and right directions of s are passable.
void PiecewiseJerkPathOptimizer::IsSCanExit(
    const ReferenceLineInfo& reference_line_info, bool* left_neighbor_exitable,
    bool* right_neighbor_exitable, const double s) const {
  *right_neighbor_exitable = true;
  *left_neighbor_exitable = true;
  if (reference_line_info.GetLaneType() == hdmap::Lane::PLAY_STREET) {
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(s);

  if (ref_point.lane_waypoints().empty()) {
    *right_neighbor_exitable = false;
    *left_neighbor_exitable = false;
    return;
  }

  const auto& waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type left_lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  hdmap::LaneBoundaryType::Type right_lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;

  left_lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  right_lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  bool is_gap = false;
  is_gap = waypoint.lane->lane().right_boundary().gap();
  if ((hdmap::LaneBoundaryType::CURB == left_lane_boundary_type ||
       hdmap::LaneBoundaryType::UNKNOWN == left_lane_boundary_type) ||
      (hdmap::LaneBoundaryType::SOLID_YELLOW == left_lane_boundary_type ||
       hdmap::LaneBoundaryType::SOLID_WHITE == left_lane_boundary_type) ||
      hdmap::LaneBoundaryType::DOUBLE_YELLOW == left_lane_boundary_type) {
    *left_neighbor_exitable = false;
  }
  if ((hdmap::LaneBoundaryType::CURB == right_lane_boundary_type ||
       hdmap::LaneBoundaryType::UNKNOWN == right_lane_boundary_type) ||
      (hdmap::LaneBoundaryType::SOLID_YELLOW == right_lane_boundary_type ||
       hdmap::LaneBoundaryType::SOLID_WHITE == right_lane_boundary_type) ||
      hdmap::LaneBoundaryType::DOUBLE_YELLOW == right_lane_boundary_type ||
      is_gap) {
    *right_neighbor_exitable = false;
  }
  return;
}

double PiecewiseJerkPathOptimizer::GetEndStateLForHasConvex(
    double request_end_l) {
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  double max_s = 0.0;
  for (const auto& path_boundary_candiate : path_boundaries) {
    double s =
        static_cast<double>(path_boundary_candiate.boundary().size() - 1) *
            path_boundary_candiate.delta_s() +
        path_boundary_candiate.start_s();
    if (s > max_s) {
      max_s = s;
    }
  }
  auto init_path_boundary = path_boundaries.at(0);
  for (const auto& path_boundary_candiate : path_boundaries) {
    if (std::string::npos != path_boundary_candiate.label().find("left") ||
        std::string::npos != path_boundary_candiate.label().find("right")) {
      init_path_boundary = path_boundary_candiate;
      break;
    } else if (std::string::npos !=
               path_boundary_candiate.label().find("self")) {
      init_path_boundary = path_boundary_candiate;
    }
  }
  double use_end_l = request_end_l;
  bool to_right = false;
  bool to_left = false;
  if (request_end_l < -kMinDiffL) {
    to_right = true;
  } else if (request_end_l > kMinDiffL) {
    to_left = true;
  }
  for (double s = init_path_boundary.start_s(); s < max_s;) {
    if (!to_right && !to_left) {
      break;
    }
    double delta_s = s - init_path_boundary.start_s();
    size_t relative_size = std::min(
        static_cast<size_t>(delta_s /
                            std::max(kEpsilon, init_path_boundary.delta_s())),
        init_path_boundary.boundary().size() - 1);
    auto relative_boudanry = init_path_boundary.boundary().at(relative_size);
    double min_l = relative_boudanry.first;
    double max_l = relative_boudanry.second;
    if ((request_end_l < min_l || use_end_l < min_l) && to_right) {
      use_end_l = min_l;
    }
    if ((request_end_l > max_l || use_end_l > max_l) && to_left) {
      use_end_l = max_l;
    }
    s = s + init_path_boundary.delta_s();
  }
  return use_end_l;
}

double PiecewiseJerkPathOptimizer::GetSelfPathBlockS(
    const PathBoundary& path_boundary, double curr_lane_right_width,
    double curr_lane_left_width, double* center_l) {
  need_to_use_narrow_center_l_ = false;
  // get block s
  double self_borrow_block_s = std::numeric_limits<double>::max();
  auto ref_obs_ids = reference_line_info_->GetRefObstacleIds();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_width = veh_param.width() * 0.5;
  // if has neighbor lane,need extend width.
  double curr_lane_right_sapce = curr_lane_right_width;
  double curr_lane_left_space = curr_lane_left_width;
  // same with path_boundary extend ,half idth + 0.8
  if (has_left_neighbor_lane_) {
    curr_lane_left_space = curr_lane_left_space + half_width +
                           FLAGS_distance_self_borrow_extend_boudary;
  }
  if (has_right_neighbor_lane_) {
    curr_lane_right_sapce = curr_lane_right_sapce + half_width +
                            FLAGS_distance_self_borrow_extend_boudary;
  }

  //  if accept to laneborrow, check is block need extend boudnary.
  if (accept_to_left_borrow_ || accept_to_right_borrow_) {
    if (has_left_neighbor_lane_) {
        curr_lane_left_space =
            curr_lane_left_width + get_left_neighbor_lane_width_;
      }
      if (has_right_neighbor_lane_) {
        curr_lane_right_sapce =
            curr_lane_right_width + get_right_neighbor_lane_width_;
      }
    }
    double min_l = -curr_lane_right_sapce;
    double max_l = curr_lane_left_space;
    // path_length
    const auto& path_boundaries =
        reference_line_info_->GetCandidatePathBoundaries();
  double max_s = 0.0;
  // double center_l = 0.0;
  double min_space = std::numeric_limits<double>::max();
  for (const auto& path_boundary_candiate : path_boundaries) {
    //  AINFO << "[Path] path label is: " << path_boundary_candiate.label();
    double s =
        static_cast<double>(path_boundary_candiate.boundary().size() - 1) *
            path_boundary_candiate.delta_s() +
        path_boundary_candiate.start_s();
    if (s > max_s) {
      max_s = s;
    }
  }
  // AINFO << "max_s = " << max_s;
  bool has_center_l = false;
    auto init_path_boundary = path_boundaries.at(0);
  for (const auto& path_boundary_candiate : path_boundaries) {
    //  AINFO << "[Path] path label is: " << path_boundary_candiate.label();
    if (std::string::npos != path_boundary_candiate.label().find("left") ||
        std::string::npos != path_boundary_candiate.label().find("right")) {
      init_path_boundary = path_boundary_candiate;
      // AINFO<<"USE BORROW BOUNDARY";
      break;
    } else if (std::string::npos !=
               path_boundary_candiate.label().find("self")) {
      // AINFO<<"USE SELF BOUNDARY";
      init_path_boundary = path_boundary_candiate;
    } else {
      // AINFO<<"USE FALLBACK BOUNDARY";
    }
  }
  *center_l = 0.0;
  // AINFO<<"max_s = "<<max_s;
  for (double s = path_boundary.start_s(); s < max_s;) {
    // AINFO<<"S = "<<s;
    // AINFO <<"  s =  " << s;
    double delta_s = s - path_boundary.start_s();
    size_t relative_size = std::min(
        static_cast<size_t>(delta_s /
                            std::max(kEpsilon, init_path_boundary.delta_s())),
        init_path_boundary.boundary().size() - 1);
    auto relative_boudanry = init_path_boundary.boundary().at(relative_size);
     min_l = relative_boudanry.first;
     max_l = relative_boudanry.second;
    //  AINFO<<"relative_boudanry.first = "<<relative_boudanry.first;
    //  AINFO<<"relative_boudanry.second = "<<relative_boudanry.second;
    bool has_obs = false;
    for (auto blocking_obstacle :
         reference_line_info_->path_decision()->obstacles().Items()) {
      // for (int j = 0; j < ref_obs_ids.size(); ++j) {
      // const Obstacle* blocking_obstacle =
      //     reference_line_info_->path_decision()->obstacles().Find(
      //         ref_obs_ids.at(j));
      if (blocking_obstacle == nullptr) {
        AERROR << "Blocking obstacle is null.";
        continue;
      }
      if (blocking_obstacle->IsVirtual()) {
        continue;
      }
      // if(blocking_obstacle->speed()>0.1 || !blocking_obstacle->IsStatic()){
      //   continue;
      // }
      const auto& obs_sl = blocking_obstacle->PerceptionSLBoundary();
      if (obs_sl.end_l() < -curr_lane_right_sapce ||
          obs_sl.start_l() > curr_lane_left_space || obs_sl.start_s() > max_s ||
          obs_sl.end_s() < reference_line_info_->AdcSlBoundary().start_s()) {
        // AINFO<<"CONTINUE";
        continue;
      }
      // if lateral has large distance, no use center l
      if (obs_sl.end_l() < std::min((min_l + half_width), 0.0) ||
          obs_sl.start_l() > std::max((max_l + half_width), 0.0)) {
        continue;
      }
      if (obs_sl.start_s() <= s && obs_sl.end_s() >= s) {
        has_obs = true;
      } else {
      }
    }

    if (max_l - min_l < kMinLateralDistance) {
      if (accept_to_left_borrow_ || accept_to_right_borrow_) {
         // AINFO << "lane borrow block s = " << s;
      }
      // block obs back 15.0m no consider move end_state.
      self_borrow_block_s = std::max(0.0, s - kDistanceToBlockobs);
      break;
    } else {
      // only has obs and in narrow need to compute center_l
      // only use first cause narrow obs
      if (max_l - min_l < min_space && has_obs && !has_center_l &&
          max_l - min_l < kMinConsiderRanger) {
        min_space = max_l - min_l;
        *center_l = (max_l + min_l) * 0.5;
        // AINFO << "center_l = " << *center_l;
        // AINFO << "curr_lane_left_space = " << curr_lane_left_space;
        // AINFO << "curr_lane_right_sapce = " << curr_lane_right_sapce;
        if ((std::fabs(max_l - curr_lane_left_space) < kMinLDiff) &&
            std::fabs(min_l - (-curr_lane_right_sapce)) > kMinLDiff) {
          *center_l = (max_l + min_l + kConsiderBlockBuffer + half_width) * 0.5;
          has_center_l = true;
        }
        if ((std::fabs(max_l - curr_lane_left_space) > kMinLDiff) &&
            std::fabs(min_l - (-curr_lane_right_sapce)) < kMinLDiff) {
          *center_l = (max_l + min_l - kConsiderBlockBuffer - half_width) * 0.5;
          has_center_l = true;
        }
        need_to_use_narrow_center_l_ = true;
        // AINFO << "center_l = " << *center_l;
      }
      //
      // if((max_l +min_l)* 0.5 <*center_l){
      //   *center_l = (max_l +min_l)* 0.5;
      // }
    }
    s = s + path_boundary.delta_s();
  }
  return self_borrow_block_s;
}

double PiecewiseJerkPathOptimizer::GetSelfPathEndStateL(
    const PathBoundary& path_boundary,
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state) {
  double end_state_l = 0.0;
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_width = veh_param.width() * 0.5;
  const auto& reference_line = reference_line_info_->reference_line();
  double max_offset = 0.0;
  const auto& lane_width = reference_line_info_->GetLaneWidthBaseOnAdcCenter();
  double curr_lane_left_width = lane_width.first;
  double curr_lane_right_width = lane_width.second;
  // AINFO << "curr_lane_right_width = " << curr_lane_right_width;
  // AINFO << "curr_lane_left_width = " << curr_lane_left_width;
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();

  double left_lane_width =
      reference_line_info_->GetLeftNeighborLaneWidth(adc_sl.end_s());
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double past_road_left_width = curr_lane_left_width;
  double past_road_right_width = curr_lane_right_width;
  if (!reference_line_info_->reference_line().GetRoadWidth(
          adc_sl.end_s(), &curr_road_left_width, &curr_road_right_width)) {
    // AWARN << "Failed to get lane width at s = " << curr_s;
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  left_lane_width = curr_road_left_width - curr_lane_left_width;

  // AINFO << "left_lane_width = " << left_lane_width;
  const auto& config = config_.piecewise_jerk_path_optimizer_config();

  // Linear increase in lateral distance buffer based on self driving speed.
  double kMinLateralDiff =
      (injector_->is_auxiliary_road_ || injector_->is_in_play_street) &&
              !util::IsLaneBorrow(injector_->planning_context())
          ? config.single_lane_selfborrow_buffer()
          : config.min_end_state_l_lateral_buffer();
  // AINFO << "kMinLateralDiff = " << kMinLateralDiff;
  double kEndstateOffset = kMinLateralDiff;
  double adc_velocity =
      std::fabs(injector_->vehicle_state()->linear_velocity());
  ADEBUG << "adc_velocity=" << adc_velocity;
  // TODO(zongxingguo): need to change max speed.
  if (!injector_->is_in_play_street) {
    kEndstateOffset =
        common::math::lerp(kMinLateralDiff, config.low_adc_speed_end_state_l(),
                           config.max_end_state_l_lateral_buffer(),
                           config.high_adc_speed_end_state_l(), adc_velocity);
    // AINFO << "kEndstateOffset===" << kEndstateOffset;
  }
  if (IsSpecialSceneForSelfBorrow()) {
    // AINFO << "IsSpecialSceneForSelfBorrow";
    //  kEndstateOffset = config.max_end_state_l_lateral_buffer();
  }

  const auto& path_decision = reference_line_info_->path_decision();
  const auto& indexed_obstacles = path_decision->obstacles();
  auto obstacle_list = indexed_obstacles.Items();
  for (const auto& disappear_obs : injector_->GetDisappearObstacles()) {
    obstacle_list.emplace_back(&disappear_obs.second);
  }
  for (const auto& moving_obs : injector_->GetStartMovingObstacles()) {
    obstacle_list.emplace_back(&moving_obs.second.second);
  }
  double max_obs_l = std::numeric_limits<double>::lowest();
  double min_obs_l = std::numeric_limits<double>::max();
  double max_offset_for_ped = std::numeric_limits<double>::lowest();
  std::string blocked_id = "";
  std::string pedestrian_blocked_id = "";
  bool need_caution_pedestrain = false;
  bool is_need_to_lane_borrow = false;
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  bool is_near_junction = FLAGS_enable_near_junction_laneborrow &&
                          (path_decider_status.is_adc_near_junction() ||
                           path_decider_status.is_obs_near_junction());
  bool is_in_junction = FLAGS_enable_near_junction_laneborrow &&
                        injector_->adc_in_junction_info_.first;
  SLBoundary leftborrow_obs;
  std::vector<std::pair<double, double>> end_state_set;
  bool is_adc_in_merge_lane = reference_line_info_->IsADCLocatedInMergeLane();
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  mutable_laneborrow->clear_block_obstacle_id();
  double check_ignore_distance =
      (util::IsMixedTraffic(injector_->planning_context()) ||
       injector_->is_in_play_street)
          ? config.ignore_end_state_l_distance_threshold_mixed_traffic()
          : config.ignore_end_state_l_distance_threshold();

  // get path block s
  double center_l = 0.0;
  double self_borrow_block_s = GetSelfPathBlockS(
      path_boundary, curr_lane_right_width, curr_lane_left_width, &center_l);
  // AINFO<<"center_l = "<<center_l;
  // AINFO << "self_borrow_block_s = " << self_borrow_block_s;
  // check can't left borrow.
  bool is_right_borrow = false;
  // TODO(zongxingguo): use obs lane width.
  // double left_neighbor_lane_width =
  //     reference_line_info_->GetLeftNeighborLaneWidth(adc_sl.end_s());
  // ;
  double right_neighbor_lane_width =
      curr_road_right_width - curr_lane_right_width;
  common::SLPoint dest_sl;
  reference_line.XYToSL(reference_line_info_->get_routing_end().pose(),
                        &dest_sl);
  double distance_between_start_and_end = dest_sl.s() - init_state.first[0];
  // AINFO << "distance_between_start_and_end = "
  //       << distance_between_start_and_end;
  bool is_near_destinaton =
      (std::fabs(dest_sl.l()) < kLateralDistanceToDestination) &&
      (distance_between_start_and_end < kDestinationNoBorrowDistance) &&
      (distance_between_start_and_end > kTiny);
  bool is_consider_destinaton =
      (std::fabs(dest_sl.l()) < kLateralDistanceToDestination) &&
      (distance_between_start_and_end < 2 * kDestinationNoBorrowDistance) &&
      (distance_between_start_and_end > kTiny);
  bool need_left_self_borrow = false;
  bool need_right_self_borrow = false;
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  // AINFO << "[Path] There are " << path_boundaries.size() << " path
  // boundaries.";
  auto init_path_boundary = path_boundaries.at(0);
  for (const auto& path_boundary_candiate : path_boundaries) {
    // AINFO << "[Path] path label is: " << path_boundary_candiate.label();
    if (std::string::npos != path_boundary_candiate.label().find("left") ||
        std::string::npos != path_boundary_candiate.label().find("right")) {
      init_path_boundary = path_boundary_candiate;
      break;
    } else if (std::string::npos !=
               path_boundary_candiate.label().find("self")) {
      init_path_boundary = path_boundary_candiate;
    } else {
    }
  }
  double get_max_l = std::numeric_limits<double>::max();
  double get_min_l = std::numeric_limits<double>::lowest();
  // check can borrow igv
  bool can_ignore_igv = true;
  const Obstacle* target_obs =
      reference_line_info_->path_decision()->obstacles().Find(
          injector_->borrow_response().block_obs_id());
  if (target_obs != nullptr && injector_->borrow_response().has_response()) {
    if (target_obs->IsIgv()) {
      can_ignore_igv = false;
    }
  }
  auto planning_status =
      injector_->planning_context()->mutable_planning_status();
  if (nullptr == planning_status) {
    return false;
  }
  const auto& top_bull = planning_status->top_bull();
  if ((injector_->borrow_response().has_response() &&
       injector_->borrow_response().response_type() ==
           planning::ResponseType::ACCEPT &&
       injector_->borrow_response().block_obs_id() ==
           top_bull.blocking_igv_id())) {
    can_ignore_igv = false;
  }
  for (const auto* obstacle : obstacle_list) {
    // get obs in path_boudnary
    bool is_left_self_borrow = false;
    bool is_right_self_borrow = false;
    // AINFO << "obstacle = " << obstacle->Id();
    if (adc_velocity > kMaxSpeedForBorrow) {
      // continue;
    }

    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    if (obstacle->IsVirtual()) {
      continue;
    }

    if(obstacle->IsHigherObs()){
      AINFO<<"no borrow for higher obs  "<<obstacle->Id();
      continue;
    }
    // skip obstacles behind ADC.
    if (obs_sl.end_s() < adc_sl.start_s()) {
      continue;
    }
    if (obstacle->IsIgv() && can_ignore_igv ) {
      continue;
    }

    bool is_tire_lifter =
            obstacle->Perception().type() ==
                perception::PerceptionObstacle::WHEELCRANE;
    // AINFO << "is_tire_lifter = " << is_tire_lifter;
    double relative_s = (obs_sl.end_s() + obs_sl.start_s()) * 0.5 -
                        (adc_sl.end_s() + adc_sl.start_s()) * 0.5;
    size_t relative_size = std::min(
        static_cast<size_t>(relative_s /
                            std::max(kEpsilon, init_path_boundary.delta_s())),
        init_path_boundary.boundary().size() - 1);
    auto relative_boudanry = init_path_boundary.boundary().at(relative_size);
    double min_l = relative_boudanry.first;
    double max_l = relative_boudanry.second;
    // AINFO << "min_l = " << min_l;
    // AINFO << "max_l = " << max_l;
    bool is_in_left_borrow =
        LaneborrowStatus_Status::LaneborrowStatus_Status_LEFTBORROW ==
        mutable_laneborrow->lane_borrow_status();
    // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
    bool is_in_right_borrow =
        LaneborrowStatus_Status::LaneborrowStatus_Status_RIGHTBORROW ==
        mutable_laneborrow->lane_borrow_status();
    if (max_l < get_max_l) {
      get_max_l = max_l;
    }
    if (min_l > get_min_l) {
      get_min_l = min_l;
    }

    // skip far obs when allow to ignore
    if (is_tire_lifter && (is_in_left_borrow || is_in_right_borrow)) {
      if ((obs_sl.start_s() - adc_sl.end_s()) >
          FLAGS_wheelcrane_consider_distance) {
        AINFO << "skip far obs, obs_id: " << obstacle->Id()
              << ", distance: " << obs_sl.start_s() - adc_sl.end_s();
        continue;
      }
      // wheel crane block left space
      if (is_in_left_borrow && obs_sl.end_l() > max_l) {
        continue;
      }
      // right is block
      if (is_in_right_borrow && obs_sl.start_l() < min_l) {
        continue;
      }
    } else {
      if ((obs_sl.start_s() - adc_sl.end_s()) > check_ignore_distance) {
        ADEBUG << "skip far obs, obs_id: " << obstacle->Id()
               << ", distance: " << obs_sl.start_s() - adc_sl.end_s();
        continue;
      }
    }

    if (is_consider_destinaton) {
      // 100m consider reach destination.no borrow 50 insaide obs.
      // u turn routing
      if (dest_sl.s() - kDestinationNoBorrowDistance < obs_sl.start_s() &&
          std::fabs((obs_sl.start_l() + obs_sl.end_l()) * 0.5) <
              kDestinationConsiderLateralDistance) {
        // AINFO << "no move obs end_state ";
        continue;
      }
    }
    double obs_center_l = (obs_sl.end_l() + obs_sl.start_l()) * 0.5;
    // TODO(zongxingguo): judge pedestrian direction.
    if (!FLAGS_enable_efficiency_shift_borrow &&
        PerceptionObstacle::PEDESTRIAN == obstacle->Perception().type() &&
        !path_decider_status.is_in_efficiency_bypass_state()) {
      // Filter non static or non forward pedestrian.
      util::MovingObstacleType moving_obstacle_type =
          util::GetMovingObstacleType(obstacle,
                                      reference_line_info_->vehicle_state(),
                                      reference_line_info_->reference_line());
      if (util::STRAIGHT_FORWARD != moving_obstacle_type &&
          util::NO_MOVING != moving_obstacle_type) {
        continue;
      }
      // pedestrians on the left side of the adc do not carry out
      // laneborrow,in city_road.
      if ((obs_center_l < adc_sl.end_l() + kPedstrianLateralBuffer &&
           obs_center_l > adc_sl.start_l() - kPedstrianLateralBuffer &&
           obs_sl.start_s() <
               adc_sl.end_s() +
                   config.max_caution_pedestrain_front_dis_threshold()) ||
          obs_sl.start_s() <
              adc_sl.end_s() +
                  config.min_caution_pedestrain_front_dis_threshold()) {
        need_caution_pedestrain = true;
      }

      bool right_can_pass = obs_sl.start_l() + curr_lane_right_width >
                            veh_param.width() + kMinLateralDiff;
      if (obs_sl.start_l() > adc_sl.end_l() && right_can_pass) {
        // AINFO<<"continue";
        continue;
      }

      double offset_for_ped =
          obs_sl.end_l() + half_width + kPedstrianLateralBuffer;
      if (obs_center_l < adc_sl.start_l() + kMinDeltaL) {
        if (offset_for_ped > 0.0) {
          end_state_set.emplace_back(std::make_pair(
              offset_for_ped, util::GetObstacleConfidence(obs_sl.start_s())));
          ADEBUG << "[end_state_set] add l: " << offset_for_ped
                 << ", ped obs: " << pedestrian_blocked_id;
        }
        if (offset_for_ped > max_offset_for_ped) {
          max_offset_for_ped = offset_for_ped;
          pedestrian_blocked_id = obstacle->Id();
        }
        // AINFO<<"continue";
        continue;
      }
      if (right_can_pass) {
        AINFO << "can sidepass ped: " << obstacle->Id() << " from right";
        continue;
      }
    }

    // skip obstacle which out self path length or out perception distance(40m)
    if (!obstacle->IsCanPass() && !obstacle->IsNeedShift() &&
        !obstacle->IsSlowCanPass()) {
      // AINFO << "obstacle->IsCanPass()  = " << obstacle->IsCanPass();
      // AINFO << "obstacle->IsNeedShift() = " << obstacle->IsNeedShift();
      // AINFO << "obstacle->IsSlowCanPass() = " << obstacle->IsSlowCanPass();

      // can be ignore no pedestrian.
      //  << "obstacle: " << obstacle->Id() << " can neither pass nor shift.";
      continue;
    }
    // skip dynamic obstacle except which just change from static to dynamic
    // check is static no rely prediction IsStatic() flags
    bool is_static =
        (obstacle->IsStatic() ||
         obstacle->speed() < FLAGS_borrow_slow_obstacle_velocity_threshold);

    if (!is_static && !obstacle->IsStaticToDynamic() &&
        !obstacle->IsSlowCanPass()) {
      AINFO << "CAN PASS";
      continue;
    }
    // bus obstacle add buffer
    double addbuffer = 0.0;
    if (reference_line_info_->GetLaneType() != hdmap::Lane::PLAY_STREET) {
      if (((obs_sl.end_s() - obs_sl.start_s() > kBusLength) &&
           obstacle->Perception().type() == PerceptionObstacle::VEHICLE)) {
        addbuffer = kAddBuffer;
        }
    // add lateral buffer for stacker.
      if((obstacle->Perception().type() == PerceptionObstacle::STACKER ||
            obstacle->Perception().type() == PerceptionObstacle::FORKLIFT_STACKER
          )){
      if(injector_->pass_stacker_response().pass_stacker_response_type() !=
      planning::PassStackerResponseType::PASS && FLAGS_enable_use_pass_stacker){
        continue;
      }
        addbuffer = FLAGS_stacker_borrow_lateral_distance;
      }
      if (is_tire_lifter && (is_in_left_borrow || is_in_right_borrow)) {
        addbuffer = FLAGS_stacker_borrow_lateral_distance*2;
      }
    }
    // in turn we need to add lateral buffer,because adc_boundary in turn may
    // large.
    if (reference_line_info_->GetRemainDistanceToTurnLane() < kDistanceToTurn) {
      addbuffer = kAddBufferInTurn;
    }
    // AINFO << "add lat buffer for large obs = " << addbuffer;

    // 1.The obstacle protrudes to the lane centerline
    // 2.There is no space left to drive on the right side of the obstacle
    // 3.The left side of the obstacle needs to be reserved for driving

    // AINFO << "half_width = " << half_width;
    // AINFO << "adc start l : " << adc_sl.start_l()
    //       << "    adc end l : " << adc_sl.end_l();
    // AINFO << "obs_sl.start_l() = " << obs_sl.start_l()
    //       << "     obs_sl.end_l() = " << obs_sl.end_l();
    // obs cause adc no drive in ref line center.
    if ((obs_sl.end_l() + kMinLateralDiff + addbuffer + half_width < 0.0) ||
        (obs_sl.start_l() - kMinLateralDiff - addbuffer - half_width > 0.0)) {
      // AINFO << "obs no in lateral range.continue";
      continue;
    }
    // has neighbor and accept the request for laneborrow
    // AINFO << "accept_to_left_borrow_ = " << accept_to_left_borrow_;
    if(!is_in_right_borrow){
    if (accept_to_left_borrow_ ) {
      // no right space,and has left space.if has right space ,use right self
      // borrow.
      is_need_to_lane_borrow =
          (obs_sl.start_l() - 2 * (kMinLateralDiff + half_width) <
           -curr_lane_right_width) &&
          (obs_sl.end_l() + kMinLateralDiff + half_width * 2 <
           curr_lane_left_width + left_lane_width) &&
          obs_sl.end_s() < self_borrow_block_s;
      // we need to use obs in road width.
      // AINFO << "curr_lane_left_width = " << curr_lane_left_width;
      // AINFO << "left_lane_width = " << left_lane_width;
      // AINFO << "self_borrow_block_s = " << self_borrow_block_s;
      // AINFO << "kMinLateralDiff = " << kMinLateralDiff;
      // AINFO << "has left space to left borrow = " << is_need_to_lane_borrow;
      // left borrow ,right side unknown obs ,no need large buffer.
      if (obstacle->Perception().type() == PerceptionObstacle::UNKNOWN &&
          is_need_to_lane_borrow &&!is_tire_lifter) {
        // AINFO << "NO VEHICLE";
        is_need_to_lane_borrow =
            -half_width - FLAGS_slef_borrow_buffer_unknown < obs_sl.end_l();
        // AINFO << "accept left laneborrow, check unknown is in "
        //          "laneis_need_to_lane_borrow = "
        //       << is_need_to_lane_borrow;
      }
      // need to left borrow ,no need to consider s overlap and no lateral
      // overlap left obs.
      if (is_need_to_lane_borrow) {
        bool is_s_overlap = (obs_sl.start_s() > adc_sl.start_s() &&
                             obs_sl.start_s() < adc_sl.end_s()) ||
                            (obs_sl.end_s() > adc_sl.start_s() &&
                             obs_sl.end_s() < adc_sl.end_s());
        bool is_left_obs = obs_sl.start_l() > adc_sl.end_l();
        if (is_s_overlap && is_left_obs) {
          is_need_to_lane_borrow = false;
          // AINFO << "no need to left borrow for left obs";
        }
      }
    } else {
      // no accept left borrow request.
      // AINFO << "has_left_neighbor_lane_ = " << has_left_neighbor_lane_;
      // AINFO << "curr_lane_right_width = " << curr_lane_right_width;
      // AINFO << "curr_lane_left_width = " << curr_lane_left_width;
      if (has_left_neighbor_lane_) {
        // check has left neighbor lane,to use larger buffer for pass.1.2
        is_need_to_lane_borrow =
            (obs_sl.start_l() - (FLAGS_slef_borrow_buffer + 2 * half_width) <
             -curr_lane_right_width) &&
            (obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width <
             curr_lane_left_width +
                 FLAGS_distance_self_borrow_extend_boudary) &&
            obs_sl.end_s() < self_borrow_block_s;
      } else {
        // check has left neighbor lane,to use larger buffer for pass.0.8
        is_need_to_lane_borrow =
            (obs_sl.start_l() -
                 (FLAGS_slef_borrow_buffer_min + 2 * half_width) <
             -curr_lane_right_width) &&
            (obs_sl.end_l() + FLAGS_slef_borrow_buffer_min + half_width * 2 <
             curr_lane_left_width) &&
            obs_sl.end_s() < self_borrow_block_s;
      }
      // If it is already in the self border, the space should be larger,
      // otherwise there is a problem of expansion near obstacles.
      // Because the corresponding maximum width will be limited later, it's
      // okay to increase it here.
      if (injector_->enable_self_borrow_) {
        is_need_to_lane_borrow =
            (obs_sl.start_l() - (FLAGS_slef_borrow_buffer + 2 * half_width) <
             -curr_lane_right_width) &&
            (obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width <
             curr_lane_left_width + FLAGS_distance_self_borrow_extend_boudary +
                 kDistanceExtraExtend) &&
            obs_sl.end_s() < self_borrow_block_s;
      }
      if (is_need_to_lane_borrow) {
        // AINFO << "has left space to self borrow = " <<
        // is_need_to_lane_borrow; If there is enough space for self seeding,
        // then determine whether self seeding is necessary. Negligible
        // long-distance obstacles are divided into vehicle types and non
        // vehicle types.
        is_need_to_lane_borrow =
            -half_width - FLAGS_slef_borrow_buffer < obs_sl.end_l();
        if (obstacle->Perception().type() == PerceptionObstacle::UNKNOWN &&
            is_need_to_lane_borrow&&!is_tire_lifter) {
          // AINFO << "NO VEHICLE";
          is_need_to_lane_borrow =
              -half_width - FLAGS_slef_borrow_buffer_unknown < obs_sl.end_l();
        }
        if (is_need_to_lane_borrow) {
          // AINFO << "obs with adc has lateral overlap,need left self borrow";
          is_left_self_borrow = true;
          need_left_self_borrow = true;

        } else {
          // AINFO
          //     << "obs with adc has lateral no overlap,no need left self
          //     borrow";
        }
      } else {
        // AINFO << "no has left space to left self borrow";
        // AINFO << "obs_sl.end_s() < self_borrow_block_s = "
        //       << (obs_sl.end_s() < self_borrow_block_s);
        // AINFO << "(obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width "
        //          "<curr_lane_left_width) = "
        //       << (obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width <
        //           curr_lane_left_width + 1.6);
        // AINFO << "(obs_sl.start_l() - (FLAGS_slef_borrow_buffer + 2 * "
        //          "half_width) <-curr_lane_right_width) = "
        //       << (obs_sl.start_l() -
        //               (FLAGS_slef_borrow_buffer + 2 * half_width) <
        //           -curr_lane_right_width);
      }
      if (is_need_to_lane_borrow) {
        // if has turn ,no self borrow
        if (reference_line_info_->GetRemainDistanceToTurnLane() <
                2 * FLAGS_distance_to_turnlane &&
            !injector_->enable_self_borrow_) {
          // AINFO << "no self borrow for near turnlane.";
          is_need_to_lane_borrow = false;
          is_left_self_borrow = false;
          need_left_self_borrow = false;
        }
      }
    }}
    // if left lane no remaining place, check right lane.
    // has right neighbor lane ,and accept borrow.
    // AINFO << "accept_to_right_borrow_ = " << accept_to_right_borrow_;
     //only no left/self borrow,use right
     if(!is_need_to_lane_borrow){
    if (accept_to_right_borrow_) {
      // AINFO << "reference lane id: "
      //       << reference_line_info_->LocateLaneInfo(adc_sl.end_s())
      //              ->lane()
      //              .id()
      //              .id()
      //       << " has no left neighbor lane,consider right lane.";

      is_need_to_lane_borrow =
          (obs_sl.start_l() - (kMinLateralDiff + 2 * half_width) >
           -curr_lane_right_width - right_neighbor_lane_width) &&
          (obs_sl.end_l() + kMinLateralDiff + half_width * 2 >
           curr_lane_left_width) &&
          obs_sl.end_s() < self_borrow_block_s;
      is_right_borrow = true;
      // AINFO << "has right space to right borrow = " << is_need_to_lane_borrow;
      // right borrow ,left side unknown obs ,no need
      // large buffer.
      if (obstacle->Perception().type() == PerceptionObstacle::UNKNOWN &&
          is_need_to_lane_borrow&&!is_tire_lifter) {
        is_need_to_lane_borrow =
            half_width + FLAGS_slef_borrow_buffer_unknown > obs_sl.start_l();
        // AINFO << "accept right laneborrow, check unknown is in lane "
        //          "is_need_to_lane_borrow = "
        //       << is_need_to_lane_borrow;
        if (!is_need_to_lane_borrow) {
          is_right_borrow = false;
        }
      }
      if (is_need_to_lane_borrow) {
        // need to right borrow ,no need to consider s overlap and no lateral
        // overlap right obs.
        bool is_s_overlap = (obs_sl.start_s() > adc_sl.start_s() &&
                             obs_sl.start_s() < adc_sl.end_s()) ||
                            (obs_sl.end_s() > adc_sl.start_s() &&
                             obs_sl.end_s() < adc_sl.end_s());
        bool is_right_obs = obs_sl.end_l() < adc_sl.start_l();
        if (is_s_overlap && is_right_obs) {
          is_need_to_lane_borrow = false;
          // AINFO << "no need to right borrow for right obs";
        }
      }
    } else {
      if (!is_need_to_lane_borrow) {
        // AINFO << "no left borrow and no left self borrow and no right borrow
        // "
        //          ",consider right self borrow";
        if (has_right_neighbor_lane_) {
          is_need_to_lane_borrow =
              (obs_sl.start_l() - (FLAGS_slef_borrow_buffer + half_width) >
               -curr_lane_right_width -
                   FLAGS_distance_self_borrow_extend_boudary) &&
              (obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width * 2 >
               curr_lane_left_width) &&
              obs_sl.end_s() < self_borrow_block_s;
        } else {
          is_need_to_lane_borrow =
              (obs_sl.start_l() -
                   (FLAGS_slef_borrow_buffer_min + half_width * 2) >
               -curr_lane_right_width) &&
              (obs_sl.end_l() + FLAGS_slef_borrow_buffer_min + half_width * 2 >
               curr_lane_left_width) &&
              obs_sl.end_s() < self_borrow_block_s;
        }

        if (injector_->enable_self_borrow_) {
          is_need_to_lane_borrow =
              (obs_sl.start_l() - (FLAGS_slef_borrow_buffer + half_width) >
               -curr_lane_right_width -
                   FLAGS_distance_self_borrow_extend_boudary -
                   kDistanceExtraExtend) &&
              (obs_sl.end_l() + FLAGS_slef_borrow_buffer + half_width * 2 >
               curr_lane_left_width) &&
              obs_sl.end_s() < self_borrow_block_s;
        }
        if (is_need_to_lane_borrow) {
          is_need_to_lane_borrow =
              half_width + FLAGS_slef_borrow_buffer > obs_sl.start_l();
        }
        // when near out lane box, no need self borrow.
        if (obstacle->Perception().type() == PerceptionObstacle::UNKNOWN &&
            is_need_to_lane_borrow&&!is_tire_lifter) {
          is_need_to_lane_borrow =
              half_width + FLAGS_slef_borrow_buffer_unknown > obs_sl.start_l();
        }
        if (is_need_to_lane_borrow) {
          // AINFO << "has right space to right selft_borrow.";
          is_right_borrow = true;
          is_right_self_borrow = true;
          need_right_self_borrow = true;
        }
        if (is_need_to_lane_borrow &&
            reference_line_info_->GetRemainDistanceToTurnLane() <
                2 * FLAGS_distance_to_turnlane &&
            !injector_->enable_self_borrow_) {
          // AINFO << "no self borrow for near turnlane.";
          is_need_to_lane_borrow = false;
          is_right_borrow = false;
          is_right_self_borrow = false;
          need_right_self_borrow = false;
        }
      }
    }
     }

    if (is_need_to_lane_borrow ||
        (path_decider_status.is_in_path_lane_borrow_scenario() &&
         !reference_line_info_->IsAdcPostureStraight() &&
         !is_adc_in_merge_lane)) {
      // check is need to selfborrow.(10m before and after obstacles)
      // Filter out all obstacles 10 meters before and after the solid line,
      // without making any final state shift.
      //! injector_->is_single_lane_ if right borrow no set this ,so no
      //! consider,pre consider.
      if (reference_line_info_->GetLaneType() != hdmap::Lane::PLAY_STREET &&
          !is_near_junction) {
        double obs_sl_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
        bool left_exitable;
        bool right_exitable;
        IsSCanExit(*reference_line_info_, &left_exitable, &right_exitable,
                   obs_sl_s);
        if (!left_exitable && !right_exitable) {
          continue;
        }
        IsSCanExit(*reference_line_info_, &left_exitable, &right_exitable,
                   obs_sl.start_s() - kDistanceFront);
        if (!left_exitable && !right_exitable) {
          continue;
        }
        IsSCanExit(*reference_line_info_, &left_exitable, &right_exitable,
                   obs_sl.end_s() + kDistanceFront);
        if (!left_exitable && !right_exitable) {
          continue;
        }
      }
      // use large obstacle
      // AINFO << "obs_sl.end_l() = " << obs_sl.end_l();
      // AINFO << "addbuffer = " << addbuffer;
      // AINFO << "kEndstateOffset = " << kEndstateOffset;
      // AINFO << "is_need_to_lane_borrow = " << is_need_to_lane_borrow;
      // AINFO << "half_width = " << half_width;
      double obs_end_state_l =
          obs_sl.end_l() + half_width + addbuffer + kEndstateOffset;
      // AINFO << "obs_end_state_l = " << obs_end_state_l;
      // no need to lane borrow, use 0.0
      if (!is_need_to_lane_borrow) {
        // obs_end_state_l = 0.0;
        continue;
      }
      if (is_right_borrow) {
        // AINFO << "is_right_borrow";
        obs_end_state_l =
            obs_sl.start_l() - half_width - addbuffer - kEndstateOffset;
      }
      if (is_left_self_borrow) {
        // AINFO << "is_left_self_borrow";
        obs_end_state_l =
            obs_sl.end_l() + half_width + FLAGS_slef_borrow_buffer;
      }
      if (is_right_self_borrow) {
        // AINFO << "is_right_self_borrow";
        obs_end_state_l =
            obs_sl.start_l() - half_width - FLAGS_slef_borrow_buffer;
      }
       AINFO << "obs_end_state_l = " << obs_end_state_l;
       // no reach turn
      if (std::fabs(obs_end_state_l) > std::fabs(center_l) &&
          need_to_use_narrow_center_l_) {
        obs_end_state_l = center_l;
        AINFO << "use center l for narrow road";
      }
      // check end_state_l in boundary.modify obs_end_state_l
      if (obs_end_state_l < min_l || obs_end_state_l > max_l) {
        // need obs sl
        AINFO << "obs_end_state_l OUT BOUDNARY";
        // AINFO<<"accept_to_right_borrow_ = "<<accept_to_right_borrow_;
        // AINFO<<"accept_to_left_borrow_ = "<<accept_to_left_borrow_;
        if (is_in_right_borrow) {
          if (is_tire_lifter) {
            obs_end_state_l =
                std::max(obs_end_state_l, min_l + kOutBoundaryLateralBuffer);
            AINFO << "obs_end_state_l = " << obs_end_state_l;
          } else {
            obs_end_state_l = std::max(max_l - kBufferToBoundary,
                                       min_l + kOutBoundaryLateralBuffer);
          }
        } else if (is_in_left_borrow) {
          if (is_tire_lifter) {
            obs_end_state_l =
                std::min(obs_end_state_l, max_l - kOutBoundaryLateralBuffer);
            AINFO << "obs_end_state_l = " << obs_end_state_l;
          } else {
            obs_end_state_l = std::min(min_l + kBufferToBoundary,
                                       max_l - kOutBoundaryLateralBuffer);
          }
        } else {
          if (accept_to_right_borrow_) {
            obs_end_state_l = std::max(max_l - kBufferToBoundary,
                                       min_l + kOutBoundaryLateralBuffer);
          }
          if (accept_to_left_borrow_) {
            obs_end_state_l = std::min(min_l + kBufferToBoundary,
                                       max_l - kOutBoundaryLateralBuffer);
          }
        }
      }

      AINFO << "obs_end_state_l = " << obs_end_state_l;
      if (obs_end_state_l < curr_lane_left_width + left_lane_width) {
        end_state_set.emplace_back(std::make_pair(
            obs_end_state_l, util::GetObstacleConfidence(obs_sl.start_s())));
        mutable_laneborrow->add_block_obstacle_id(obstacle->Id());
        ADEBUG << "[end_state_set] add l: " << obs_end_state_l
               << ", obs: " << obstacle->Id();
      }
      if (is_right_borrow) {
        if ((obs_end_state_l < min_obs_l) &&
            (obs_end_state_l > min_l && obs_end_state_l < max_l)) {
          min_obs_l = obs_end_state_l;
          end_state_l = obs_end_state_l;
          leftborrow_obs = obs_sl;
          blocked_id = obstacle->Id();
          // AINFO << "obstacle need_to_lane_borrow: " << blocked_id
          //       << ", obs_sl.end_l()=" << obs_sl.end_l()
          //       << ", end_state_l: " << end_state_l;
          bool left_exitable;
          bool right_exitable;
          IsSCanExit(*reference_line_info_, &left_exitable, &right_exitable,
                     obs_sl.end_s() + 2 * kDistanceFront);
          if (!left_exitable && !right_exitable && !is_near_junction) {
            is_can_exit_ = false;
          }
        } else {
          continue;
        }
      }
      if ((obs_end_state_l > max_obs_l) &&
          (obs_end_state_l > min_l && obs_end_state_l < max_l)) {
        max_obs_l = obs_end_state_l;
        end_state_l = obs_end_state_l;
        leftborrow_obs = obs_sl;
        blocked_id = obstacle->Id();
        // AINFO << "obstacle need_to_lane_borrow: " << blocked_id
        //       << ", obs_sl.end_l()=" << obs_sl.end_l()
        //       << ", end_state_l: " << end_state_l;
        bool left_exitable;
        bool right_exitable;
        IsSCanExit(*reference_line_info_, &left_exitable, &right_exitable,
                   obs_sl.end_s() + 2 * kDistanceFront);
        if (!left_exitable && !right_exitable && !is_near_junction) {
          is_can_exit_ = false;
        }
      }
    }
  }
  // AINFO << "end_state_l = " << end_state_l;
  if (accept_to_right_borrow_ && end_state_l < 0.5 && end_state_l > get_max_l) {
    end_state_l = std::max(get_max_l - kBufferToBoundary, get_min_l);
  }
  if (accept_to_left_borrow_ && end_state_l > 0.5 && end_state_l < get_min_l) {
    end_state_l = std::min(get_min_l + kBufferToBoundary, get_max_l);
  }
  // adc for request borrow
  end_state_l = GetEndStateLForHasConvex(end_state_l);
  AINFO << "end_state_l = " << end_state_l;
  UpdatePedestrainCautionState(need_caution_pedestrain);
  if (max_offset_for_ped > end_state_l) {
    end_state_l = max_offset_for_ped;
    blocked_id = pedestrian_blocked_id;
  }

  max_offset = end_state_l;

  // near trafficlight stopline(<35m),end_state 0.0
  if (!FLAGS_enable_near_junction_laneborrow && injector_->near_traffic_line_ &&
      reference_line_info_->IsAdcCenterInLane()) {
    end_state_l = 0.0;
    end_state_set.emplace_back(std::make_pair(0.0, 1.0));
    ADEBUG << "[end_state_set] near_traffic_line.";
  }

  if ((!is_in_junction && !injector_->is_auxiliary_road_ &&
       util::IsMixedTraffic(injector_->planning_context())) ||
      is_adc_in_merge_lane) {
    end_state_l = 0.0;
    end_state_set.emplace_back(std::make_pair(0.0, 1.0));
    // AINFO << "[end_state_set] is_adc_in_merge_lane: " << is_adc_in_merge_lane;
  }
  // AINFO << "end_state_l = " << end_state_l;
  bool need_borrow_obs = false;
  if (IsNeedWaitToBorrowStraightly()) {
    // if no need to laneborrow or no in ref_lane,go back to ref_lane?
    // end_state_l = 0.0;
    // need_borrow_obs = true;
    end_state_set.clear();
    double confidence =
        path_decider_status.is_in_reverse_avoid_state() ? 0.0 : 1.0;
    end_state_set.emplace_back(std::make_pair(0.0, confidence));
    // AINFO << "[end_state_set] need wait to straight.";
    if (FLAGS_enable_diagonal_path) {
      // AINFO << "reference_line_info_->vehicle_state().linear_velocity() = "
      //       << reference_line_info_->vehicle_state().linear_velocity();
      // AINFO << "end_state_l  = " << end_state_l;
      // adc stop and has obs to selfborrow
      if (std::fabs(reference_line_info_->vehicle_state().linear_velocity()) <
              kStopSpeed &&
          (need_left_self_borrow || need_right_self_borrow)) {
        // TODO(zongxingguo):  There is a speed limit added here, and there will
        // be a critical value where collision detection can pass within the
        // range of 0.3, so there will be no stopping.
        injector_->self_borrow_check_times_++;
        injector_->self_borrow_check_times_ =
            std::min(injector_->self_borrow_check_times_, kMaxCount);
        // obs need live 10 times,
        if (injector_->self_borrow_check_times_ == kMaxCount) {
          injector_->enable_self_borrow_ = true;
        }
      } else {
        if (!injector_->enable_self_borrow_) {
          injector_->self_borrow_check_times_--;
          injector_->self_borrow_check_times_ =
              std::max(injector_->self_borrow_check_times_, 0);
        }
      }
      // AINFO<<"need_left_self_borrow = "<<need_left_self_borrow;
      // AINFO<<"need_right_self_borrow = "<<need_right_self_borrow;
      // AINFO<<"injector_->enable_self_borrow_ = "<<injector_->enable_self_borrow_;
      // lateral_error < 0.1m, heading diff < 0.02rad
      // need add replan check
      double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
      auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
      double center_s =
          (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
      // AINFO << "center_s = " << center_s;
      const auto& reference_point =
          reference_line_info_->reference_line().GetReferencePoint(center_s);
      double ref_heading = reference_point.heading();
      double adc_heading = reference_line_info_->vehicle_state().heading();
      double diff_heading =
          std::fabs(common::math::NormalizeAngle(ref_heading - adc_heading));
      double diff_heading_between_adc_and_startpoint =
          std::fabs(common::math::NormalizeAngle(
              reference_line_info_->InitPointHeading() - adc_heading));
      // AINFO << "injector_->self_borrow_check_times_ = "
      //       << injector_->self_borrow_check_times_;
      // AINFO << "center_l = " << center_l;
      // AINFO << "diff_heading  = " << diff_heading;
      // adc go back to ref line and min heading error and min lateral error
      // and no need to borrow
      if (injector_->enable_self_borrow_ &&
          std::fabs(end_state_l) < kMinLateralError &&
          std::fabs(center_l) < FLAGS_lateral_error &&
          diff_heading < FLAGS_same_heading &&
          diff_heading_between_adc_and_startpoint < FLAGS_same_heading) {
        injector_->self_borrow_check_times_--;
        injector_->self_borrow_check_times_ =
            std::max(injector_->self_borrow_check_times_, 0);

        if (injector_->self_borrow_check_times_ == 0) {
          injector_->enable_self_borrow_ = false;
        }
      }
      if (injector_->enable_self_borrow_) {
        // AINFO << "need_borrow_obs = true";
        need_borrow_obs = true;
        reference_line_info_->SetNeedDiagonal(true);
      } else {
        // AINFO << "need_borrow_obs = false";
        need_borrow_obs = false;
        end_state_l = 0.0;
      }
    } else {
      // AINFO << "NO DIAGONAL";
      end_state_l = 0.0;
      need_borrow_obs = true;
      injector_->enable_self_borrow_ = false;
    }
  }
  // AINFO << "is_right_borrow = " << is_right_borrow;
  if (is_right_borrow) {
    need_borrow_obs = true;
  }
  // AINFO << "need_borrow_obs = " << need_borrow_obs;
  // AINFO << "end_state_l = " << end_state_l;
  if (mutable_laneborrow->need_keep_self_path_straight() && !is_right_borrow) {
    // end_state_l = 0.0;
    end_state_set.emplace_back(std::make_pair(0.0, 1.0));
    // AINFO << "[end_state_set] need_keep_self_path_straight.";
  }
  // AINFO << "end_state_l = " << end_state_l;
  if (path_decider_status.is_in_reverse_avoid_state() && !is_right_borrow) {
    end_state_set.emplace_back(reference_line_info_->GetRiskShiftResult());
    ADEBUG << "[end state] in risk shift state, value: "
           << reference_line_info_->GetRiskShiftResult().first;
  }

  // keep right 's end_state.
  if (injector_->is_need_to_keep_right_) {
    end_state_l =
        std::max(reference_line_info_->get_end_state_l(),
                 -curr_lane_right_width + kLateralBuffer + half_width);
    end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    // AINFO << "[end_state_set] is_need_to_keep_right add l: " << end_state_l;
  }
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // near junction to stop line and junction too close,no borrow
  if (0) {
    double to_solid_line_distance =
        std::fmin(reference_line_info_->GetRemainDistanceToSolidLine(),
                  reference_line_info_->GetRemainDistanceToJunction());
    double heading_threshold =
        config.adc_goback_heading_threshold() * kDegreesToRadians;
    bool obs_in_ref_line = path_decider_status.has_obs_in_reference_lane() &&
                           adc_sl.start_l() > curr_lane_left_width;
    bool not_enough_to_goback =
        to_solid_line_distance <
            config.adc_to_solid_line_distance_threshold() &&
        std::fabs(0.5 * (adc_sl.start_l() + adc_sl.end_l()) -
                  (curr_lane_left_width + 0.5 * left_lane_width)) <
            config.lateral_buffer_to_center();
    bool straight_in_left_solid_line =
        reference_line_info_->GetAdcHeadingDiffWithRefLine() >
            -heading_threshold &&
        reference_line_info_->IsInLeftNeighborSolidLine(adc_sl);
    if (path_decider_status.is_need_go_left_lane() ||
        (path_decider_status.is_in_path_lane_borrow_scenario() &&
         !path_decider_status.is_need_goback_reference_lane() &&
         (obs_in_ref_line || not_enough_to_goback ||
          straight_in_left_solid_line))) {
      end_state_l = curr_lane_left_width + 0.5 * left_lane_width;
      end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    }
    // AINFO << "[near junction] laneborrow end state l: " << end_state_l
    //       << ", curr_lane_left_width: " << curr_lane_left_width
    //       << ", half left_lane_width: " << 0.5 * left_lane_width;
  }

  // whole in junction
  if (is_in_junction && path_decider_status.is_in_path_lane_borrow_scenario() &&
      !path_decider_status.is_need_goback_reference_lane() &&
      !is_adc_in_merge_lane) {
    end_state_l =
        std::fmax(curr_lane_left_width + 0.5 * left_lane_width, end_state_l);
    end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    // AINFO << "[in junction] laneborrow end state l: " << end_state_l;
  }

  // in crowd traffic
  if (path_decider_status.is_in_crowd_traffic()) {
    bool safe_to_goback = reference_line_info_->GetIsClearToChangeLane();
    double heading_threshold =
        config.adc_goback_heading_threshold() * kDegreesToRadians;
    if (util::IsLaneBorrow(injector_->planning_context()) &&
        !reference_line_info_->IsAdcCenterInLane() &&
        (reference_line_info_->GetAdcHeadingDiffWithRefLine() >
             -heading_threshold ||
         !safe_to_goback)) {
      end_state_l = curr_lane_left_width + 0.5 * left_lane_width;
    }
    end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    // AINFO << "[CrowdTraffic] laneborrow end state l: " << end_state_l;
  }

  // in auxiliary road
  if (injector_->is_auxiliary_road_) {
    bool adc_in_auxiliary_road_junction =
        reference_line_info_->AdcIsOnOverlapJunction() &&
        !reference_line_info_->IsADCInCurbArea();
    double distance_limit =
        adc_in_auxiliary_road_junction
            ? config.auxiliary_road_in_junction_distance_to_curb_limit()
            : config.auxiliary_road_distance_to_curb_limit();
    double distance_to_curb =
        reference_line_info_->GetRemainDistanceToCurbArea();
    bool need_exit_laneborrow =
        mutable_laneborrow->need_exit_laneborrow_in_auxiliary_road();
    if (distance_to_curb < distance_limit && !need_exit_laneborrow) {
      end_state_l = Clamp(end_state_l, -curr_lane_right_width + half_width,
                          curr_lane_left_width - half_width);
      AINFO << "near curb, end state l: " << end_state_l
            << ", distance_to_curb: " << distance_to_curb;
      end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    }
    if (need_exit_laneborrow &&
        path_decider_status.is_in_path_lane_borrow_scenario()) {
      end_state_l = 0.0;
      AINFO << "need_exit_laneborrow_in_auxiliary_road, end state l: "
            << end_state_l;
      end_state_set.emplace_back(std::make_pair(end_state_l, 1.0));
    }
    AINFO << "[AuxiliaryRoad] laneborrow end state l: " << end_state_l;
  }

  bool safe_to_goback_lane = true;
  bool keep_lane_borrow = false;

  // If you are in the park or not in the state of lanebrow, you do not need
  // to consider the obstacles in the left rear or the obstacles in the left
  // front
  // 2.Outside the park, after the obstacle is determined to be bypassed, the
  // laneborrow is not needed for the time being, and the continuous end state
  // is 0.0
  if (injector_->is_in_play_street) {
    safe_to_goback_lane = true;
    keep_lane_borrow = false;
  } else if (!injector_->is_in_play_street &&
             std::fabs(end_state_l) < curr_lane_left_width &&
             injector_->planning_context()
                         ->mutable_planning_status()
                         ->last_lateral_distance() -
                     end_state_l >
                 kMinLateralBuffer) {
    safe_to_goback_lane = reference_line_info_->GetIsClearToChangeLane();
    reference_line_info_->SetNeedTurnRight();
  }
  // AINFO << "safe_to_goback_lane = " << safe_to_goback_lane;
  // AINFO << "end_state_l = " << end_state_l;
  if (!safe_to_goback_lane) {
    keep_lane_borrow = true;
  }
  // Is there also an obstacle on the left side of the obstacle that needs to be
  // laneborrow? If so, it indicates blockage and laneborrow is not needed
  bool safe_to_left_lane = true;
  if (((!injector_->is_in_play_street && !injector_->is_auxiliary_road_) ||
       (injector_->is_in_play_street &&
        !util::IsLaneBorrow(injector_->planning_context()))) &&
      end_state_l - injector_->planning_context()
                        ->mutable_planning_status()
                        ->last_lateral_distance() >
          kMinLateralBuffer) {
    if (blocked_id == "") {
      // AINFO << " no borrow obs";
      safe_to_left_lane = true;
    } else {
      // in borrow action check.
      safe_to_left_lane = IsSafeToLeftborrow(leftborrow_obs);
    }

    // AINFO << "safe_to_left_lane=" << safe_to_left_lane;
  }
  // Keep following the reference line during containment, and do not carry
  // out laneborrow
  if (!safe_to_left_lane) {
    keep_lane_borrow = true;
    end_state_l = 0.0;
    end_state_set.emplace_back(std::make_pair(0.0, 1.0));
    // AINFO << "[end_state_set] unsafe_to_left_lane.";
  }

  if (!keep_lane_borrow) {
    mutable_path_decider_status->set_keep_lane_borrow_end_state_l(
        0.5 * (adc_sl.start_l() + adc_sl.end_l()));
  }

  // end_state_l_test = GetMaxValidEndState(end_state_set).first;
  // AINFO << "end_state_l_test = " << end_state_l_test;
  AINFO << "[valid] end state l: " << end_state_l;

  double origin_state_l = end_state_l;
  if (keep_lane_borrow) {
    end_state_l = path_decider_status.keep_lane_borrow_end_state_l();
    // AINFO << "keep_lane_borrow is true, end state l: " << end_state_l;
    if (!safe_to_left_lane && injector_->is_in_play_street) {
      end_state_l = 0.0;
    }
  }
  // near destination
  if (is_near_destinaton) {
    // no pullover for destination
    //  end_state_l = dest_sl.l();
    AINFO << "is_near_destinaton = " << is_near_destinaton;
    end_state_l = 0.0;
    end_state_set.emplace_back(std::make_pair(dest_sl.l(), 1.0));
    // AINFO << "[end_state_set] is_near_destinaton add l: " << dest_sl.l();
  }
  AINFO << "end_state_l = " << end_state_l;
  reference_line_info_->SetRequestEndL(end_state_l);
  bool safe_to_self_borrow = SelfBorrowSafeCheck(end_state_l);
  if (!safe_to_self_borrow) {
    AINFO << "self borrow unsafe, use last end state l";
    end_state_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  }
  if(!SafeReturnCheck(end_state_l)){
    AINFO << "return unsafe, use last end state l";
    end_state_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  }

  // 1.In an offset state,need to laneborrow, add blocked obstacle.
  // 2.No in offset state,add selfborrow count.(>6 clear laneborrrow decision)
  if (std::fabs(end_state_l) > config.end_state_center_threshold() ||
      need_borrow_obs) {
    int front_static_obstacle_cycle_counter = std::max(
        mutable_path_decider_status->front_static_obstacle_cycle_counter(), 0);
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(kMaxCount, ++front_static_obstacle_cycle_counter));
    // AINFO << "BLOCK COUNT = " << front_static_obstacle_cycle_counter;
    if (mutable_path_decider_status->front_static_obstacle_cycle_counter() >=
        kMaxCount) {
      // AINFO << "set block id = " << blocked_id;
      // set block id
      mutable_path_decider_status->set_front_static_obstacle_id(blocked_id);
    } else {
      mutable_path_decider_status->set_front_static_obstacle_id("");
    }
    AINFO << "set_able_to_use_self_lane_counter";
    int able_to_use_self_lane_counter =
        mutable_path_decider_status->able_to_use_self_lane_counter();
    mutable_path_decider_status->set_able_to_use_self_lane_counter(
        std::max(0, --able_to_use_self_lane_counter));

  } else if (reference_line_info_->IsAdcPostureStraight() &&
             !is_adc_in_merge_lane) {
    int able_to_use_self_lane_counter = std::max(
        mutable_path_decider_status->able_to_use_self_lane_counter(), 0);

    mutable_path_decider_status->set_able_to_use_self_lane_counter(
        std::min(able_to_use_self_lane_counter + 1, kMaxCount));
    if (mutable_path_decider_status->will_pass_merge_lane_area() &&
        last_adc_is_in_merge_lane_) {
      mutable_path_decider_status->set_able_to_use_self_lane_counter(kMaxCount);
    }
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    // AINFO << "BLOCK COUNT = " << front_static_obstacle_cycle_counter;
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(0, --front_static_obstacle_cycle_counter));
    mutable_path_decider_status->set_front_static_obstacle_id("");
  }
  last_adc_is_in_merge_lane_ = is_adc_in_merge_lane;

  double last_lateral_l = injector_->planning_context()
                              ->mutable_planning_status()
                              ->last_lateral_distance();
  double delta_l = end_state_l - last_lateral_l;
  AINFO << "request end_state_l = " << end_state_l;
  request_end_l_ = end_state_l;
  AINFO << "last_lateral_l = " << last_lateral_l;
  if (std::fabs(delta_l) > 0.3) {
    injector_->end_state_shake_check_times_++;
    if (injector_->end_state_shake_check_times_ <= 3) {
      // AINFO << "use last end state l";
      end_state_l = last_lateral_l;
    }
    // AINFO << "injector_->end_state_shake_check_times_ = "
    //       << injector_->end_state_shake_check_times_;
  } else {
    injector_->end_state_shake_check_times_ = 0;
  }
    // face ref line center
  if (std::fabs(end_state_l) < kEpsilon) {
    injector_->end_state_return_check_times_++;
  } else {
    injector_->end_state_return_check_times_ = 0;
  }
  if (std::fabs(delta_l) <= 0.3) {
    // is disappear
    if (std::fabs(end_state_l) < kEpsilon &&
        injector_->end_state_return_check_times_ < 4) {
      // AINFO << "use last end state l for check obs dispaer";
      end_state_l = last_lateral_l;
    }
    // no face ref lene center
    if (std::fabs(end_state_l) > kEpsilon) {
      // AINFO << "use last end state l for no large change";
      end_state_l = last_lateral_l;
    }
  }

  // AINFO << "now end_state_l = " << end_state_l;
  delta_l = end_state_l - last_lateral_l;
  AINFO << "delta_l = " << delta_l;
  int is_plus = delta_l > 0.0 ? 1 : -1;
  // check is left borrow
  bool is_in_left_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_LEFTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
  bool is_in_right_borrow =
      LaneborrowStatus_Status::LaneborrowStatus_Status_RIGHTBORROW ==
      mutable_laneborrow->lane_borrow_status();
  // AINFO << "is_right_borrow = " << is_right_borrow;
  if (is_right_borrow) {
    if (std::fabs(delta_l) < FLAGS_min_step_end_state_l) {
      end_state_l = (last_lateral_l + end_state_l) * 0.5;
      // AINFO << "use last end state: " << end_state_l;
    } else {
      double step_l = common::math::lerp(
          FLAGS_max_step_end_state_l, FLAGS_min_speed_step_end_state_l,
          FLAGS_min_step_end_state_l, FLAGS_max_speed_step_end_state_l,
          adc_velocity);
      step_l =
          Clamp(step_l, FLAGS_max_step_end_state_l, FLAGS_min_step_end_state_l);
      AINFO << "adc_velocity = " << adc_velocity;
      // only in borrow state
      if (std::fabs(end_state_l) > std::fabs(center_l) &&
          std::fabs(delta_l) > kEndStateStep) {
        step_l = kEndStateStep;
      }
      if (is_in_right_borrow && is_plus * is_plus > 0 && 0) {
        // no left return
        end_state_l = last_lateral_l;
      } else {
        end_state_l = last_lateral_l + is_plus * step_l;
      }
      AINFO << "step_l: " << step_l << ", last_lateral_l: " << last_lateral_l
             << ", end_state_l: " << end_state_l;
    }
  } else {
    AINFO << "LEFT BORROW";
    if (std::fabs(delta_l) < FLAGS_min_step_end_state_l) {
      end_state_l = (last_lateral_l + end_state_l) * 0.5;
      // AINFO << "use last end state: " << end_state_l;
    } else {
      double step_l = common::math::lerp(
          FLAGS_max_step_end_state_l, FLAGS_min_speed_step_end_state_l,
          FLAGS_min_step_end_state_l, FLAGS_max_speed_step_end_state_l,
          adc_velocity);
      step_l =
          Clamp(step_l, FLAGS_max_step_end_state_l, FLAGS_min_step_end_state_l);
      // AINFO << "is_in_left_borrow = " << is_in_left_borrow;
      // AINFO << "step_l = " << step_l;
      // AINFO << "is_plus = " << is_plus;
      if (std::fabs(end_state_l) > std::fabs(center_l) &&
          std::fabs(delta_l) > kEndStateStep) {
        step_l = kEndStateStep;
      }
      if (is_in_left_borrow && ((is_plus * step_l) < 0) && 0) {
        end_state_l = last_lateral_l;
        // AINFO << "now can return ,but state in borrow station, no return";
      } else {
        end_state_l = last_lateral_l + is_plus * step_l;
      }
      // end_state_l = last_lateral_l + is_plus * step_l;
      AINFO << "step_l: " << step_l << ", last_lateral_l: " << last_lateral_l
            << ", end_state_l: " << end_state_l;
    }
  }

  injector_->planning_context()
      ->mutable_planning_status()
      ->set_last_lateral_distance(end_state_l);
  // lane borrow end state l
  self_borrow_l_ = end_state_l;

  // Selfborrow's boundary limit cannot exceed the lane width
  // If obs's end state l is too large and neighbor lane is not borrowable,
  // adc go straight in current lane.
  // self borrow in boudary.
  if ((origin_state_l > curr_lane_left_width - half_width ||
       max_offset > curr_lane_left_width - half_width)) {
    // AINFO << "curr_lane_left_width = " << curr_lane_left_width;

    if (has_left_neighbor_lane_) {
      end_state_l =
          reference_line_info_->IsAdcInGateArea()
              ? 0.0
              : std::min(curr_lane_left_width +
                             FLAGS_distance_self_borrow_extend_boudary,
                         end_state_l);
    } else {
      end_state_l = reference_line_info_->IsAdcInGateArea()
                        ? 0.0
                        : std::min(curr_lane_left_width, end_state_l);
    }
  }

  real_self_borrow_l_ = end_state_l;
  injector_->selfborrow_endstate = end_state_l;
  // AINFO << "selfborrow_endstate = " << end_state_l;

  return end_state_l;
}

void PiecewiseJerkPathOptimizer::GetLaneBorrowEndState(
    const PathBoundary& path_boundary, double* const end_state_l) {
  if (std::string::npos != path_boundary.label().find("left")) {
    *end_state_l = self_borrow_l_;
    if (!is_can_exit_) {
      *end_state_l = real_self_borrow_l_;
      AINFO << "can NOT exit, NOT use self_borrow_l: " << self_borrow_l_;
    }
    AINFO << "<<left borrow>> end state l: " << *end_state_l;
    injector_->left_laneborrow_endstate = *end_state_l;
  } else if (std::string::npos != path_boundary.label().find("right")) {
    const auto& lat_boundaries = path_boundary.boundary();
    double max_start_l = std::numeric_limits<double>::lowest();
    double min_end_l = std::numeric_limits<double>::max();
    for (size_t i = 0; i < lat_boundaries.size(); ++i) {
      max_start_l = std::max(max_start_l, lat_boundaries[i].first);
      min_end_l = std::min(min_end_l, lat_boundaries[i].second);
    }
    *end_state_l = std::min(0.0, std::max((min_end_l + max_start_l) * 0.5,
                                          min_end_l - kMaxLateralBuffer));
    // for right borrow end state l
    *end_state_l = self_borrow_l_;
    AINFO << "<<right borrow>> end state l: " << *end_state_l;
    injector_->right_laneborrow_endstate = *end_state_l;
  }
  return;
}

void PiecewiseJerkPathOptimizer::UpdatePedestrainCautionState(
    const bool need_caution_pedestrain) {
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  auto count = mutable_path_decider_status->caution_pedestrain_count_times();
  count = need_caution_pedestrain ? (count + 1) : (count - 1);

  if (count >= config.caution_pedestrain_check_times_threshold()) {
    injector_->can_borrow_pedestrian_ = true;
  } else if (count <= 0) {
    injector_->can_borrow_pedestrian_ = false;
  }
  mutable_path_decider_status->set_caution_pedestrain_count_times(
      Clamp(count, 0, config.caution_pedestrain_check_times_threshold()));
  // AINFO << "need_caution_pedestrain: " << need_caution_pedestrain
  //       << ", caution_ped_count_times: "
  //       << mutable_path_decider_status->caution_pedestrain_count_times()
  //       << ", can_Borrow: " << injector_->can_borrow_pedestrian_;
}

std::pair<double, double> PiecewiseJerkPathOptimizer::GetMaxValidEndState(
    const std::vector<std::pair<double, double>>& end_state_set) {
  const auto last_end_state_l =
      injector_->planning_context()->planning_status().last_lateral_distance();
  double valid_delta_l = 0.0;
  std::pair<double, double> valid_stable_end_state = {0.0, 1.0};
  bool need_right = false;
  for (const auto end_state : end_state_set) {
    double confidence_delta_l =
        (end_state.first - last_end_state_l) * end_state.second +
        last_end_state_l;
    if (end_state.first < -kEndStateShiftThreshold ||
        injector_->planning_context()
            ->planning_status()
            .path_decider()
            .left_reverse_unsafe()) {
      need_right = true;
      if (confidence_delta_l < valid_delta_l) {
        valid_delta_l = confidence_delta_l;
        valid_stable_end_state = {valid_delta_l, end_state.second};
        ADEBUG << "[right shift], update min valid_delta_l: " << valid_delta_l;
      }
    } else if (!need_right) {
      if (confidence_delta_l > valid_delta_l) {
        valid_delta_l = confidence_delta_l;
        valid_stable_end_state = {valid_delta_l, end_state.second};
        ADEBUG << "[left shift], update max valid_delta_l: " << valid_delta_l;
      }
    } else {
      ADEBUG << "discard end state l: " << end_state.first;
    }

    ADEBUG << "end_state.first: " << end_state.first
           << ", second: " << end_state.second
           << "; confidence_delta_l: " << confidence_delta_l
           << ", valid_delta_l: " << valid_delta_l;
  }
  return valid_stable_end_state;
}

void PiecewiseJerkPathOptimizer::UpdateLoosenPathConstrainState(
    const bool has_path_ok) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const auto& config = config_.piecewise_jerk_path_optimizer_config();
  auto failure_count = mutable_path_decider_status->path_failure_times();

  if (!has_path_ok) {
    failure_count =
        Clamp(failure_count, 0, config.max_allow_path_failure_time());
    mutable_path_decider_status->set_path_failure_times(++failure_count);
  } else {
    failure_count =
        Clamp(failure_count, -config.max_allow_path_failure_time(), 0);
    mutable_path_decider_status->set_path_failure_times(--failure_count);
  }

  if (failure_count >= config.path_failure_times_threshold()) {
    mutable_path_decider_status->set_need_loosen_path_constrain(true);
  } else if (failure_count < -config.max_allow_path_failure_time()) {
    mutable_path_decider_status->set_need_loosen_path_constrain(false);
  }
  return;
}

bool PiecewiseJerkPathOptimizer::IsSpecialSceneForSelfBorrow() {
  return util::IsNeedLoosenPathConstrains(injector_->planning_context());
}

bool PiecewiseJerkPathOptimizer::IsNeedWaitToBorrowStraightly() {
  // const auto& path_decider_status =
  //     injector_->planning_context()->planning_status().path_decider();
  // bool is_near_junction = FLAGS_enable_near_junction_laneborrow &&
  //                         (path_decider_status.is_adc_near_junction() ||
  //                          path_decider_status.is_obs_near_junction());
  // bool has_truck_near_junction =
  // path_decider_status.has_truck_near_junction();

  // junction or truck
  if (reference_line_info_->IsAdcCenterInLane() &&
      !util::IsLaneBorrow(injector_->planning_context())) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace century
