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

#include "modules/planning/tasks/deciders/path_assessment_decider/path_assessment_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::perception::PerceptionObstacle;

namespace {
// PointDecision contains (s, PathPointType, distance to closest obstacle).
using PathPointDecision = std::tuple<double, PathData::PathPointType, double>;
constexpr double kMinObstacleArea = 1e-4;
static constexpr double kSelfPathLengthComparisonTolerance = 15.0;
static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
static constexpr double kAdcPositionBuffer = 1.0;
static constexpr double kOffReferenceLineThreshold = 20.0;
static constexpr double kOffRoadThreshold = 10.0;
// filter out sidepass stop fence
static constexpr double kLookForwardBuffer = 5.0;
static constexpr double kSDelta = 0.3;

static constexpr double kNearTrafficLightDistance = 30.0;
static constexpr double kFarAwayTrafficLightBuffer = 5.0;
constexpr double kMaxScanDistance = 30.0;
constexpr double kPathTpyeBuffer = 1.0;
constexpr size_t kMinAppearTimes = 5UL;
constexpr double kLateralDistanceBuffer = 0.3;
constexpr double kMinKappa = 0.01;
constexpr double kConsiderPathLength = 30.0;
constexpr size_t kPathStep = 4UL;
}  // namespace

static HysteresisInterval obstacles_interval(
    0.0, FLAGS_block_obstacle_lat_dis_hysteresis_width, 20UL);
PathAssessmentDecider::PathAssessmentDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathAssessmentDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  // skip path_assessment_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  const auto& candidate_path_data = reference_line_info->GetCandidatePathData();

  if (candidate_path_data.empty()) {
    ADEBUG << "Candidate path data is empty.";
  } else {
    ADEBUG << "There are " << candidate_path_data.size() << " candidate paths";
  }
  const auto& end_time0 = std::chrono::system_clock::now();

  // 1. Remove invalid path.
  std::vector<PathData> valid_path_data;
  RemoveInvalidPath(reference_line_info, candidate_path_data, &valid_path_data);

  const auto& end_time1 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time1 - end_time0;
  ADEBUG << "Time for path validity checking: " << diff.count() * 1000
         << " msec.";

  // 2. Analyze and add important info for speed decider to use
  const Obstacle* blocking_obstacle_on_selflane = nullptr;
  AnalyzePath(reference_line_info, blocking_obstacle_on_selflane,
              &valid_path_data);
  // If there is no valid path_data, exit.
  if (valid_path_data.empty()) {
    const std::string msg = "Neither regular nor fallback path is valid.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  AINFO << "There are " << valid_path_data.size() << " valid path data.";

  const auto& end_time2 = std::chrono::system_clock::now();
  diff = end_time2 - end_time1;
  ADEBUG << "Time for path info labeling: " << diff.count() * 1000 << " msec.";

  // 3. Pick the optimal path.
  std::sort(
      valid_path_data.begin(), valid_path_data.end(),
      std::bind(ComparePathData, std::placeholders::_1, std::placeholders::_2,
                blocking_obstacle_on_selflane, reference_line_info));
  PickOptimalPath(reference_line_info, blocking_obstacle_on_selflane,
                  valid_path_data);

  const auto& end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for optimal path selection: " << diff.count() * 1000
         << " msec.";

  reference_line_info->SetCandidatePathData(std::move(valid_path_data));

  // 4. Update necessary info for lane-borrow decider's future uses.
  // Update front static obstacle's info.
  UpdateNecessaryInfoForLaneBorrow(reference_line_info);

  const auto& end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for FSM state updating: " << diff.count() * 1000 << " msec.";

  // Plot the path in simulator for debug purpose.
  RecordDebugInfo(reference_line_info->path_data(), "Planning PathData",
                  reference_line_info);
  return Status::OK();
}

void PathAssessmentDecider::RemoveInvalidPath(
    ReferenceLineInfo* const reference_line_info,
    const std::vector<PathData>& candidate_path_data,
    std::vector<PathData>* const valid_path_data) {
  for (const auto& curr_path_data : candidate_path_data) {
    // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
    //                 reference_line_info);
    if (curr_path_data.path_label().find("fallback") != std::string::npos) {
      if (IsValidFallbackPath(*reference_line_info, curr_path_data)) {
        valid_path_data->push_back(curr_path_data);
      }
    } else {
      if (IsValidRegularPath(*reference_line_info, curr_path_data)) {
        valid_path_data->push_back(curr_path_data);
      }
    }
  }
}

void PathAssessmentDecider::AnalyzePath(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle* blocking_obstacle_on_selflane,
    std::vector<PathData>* const valid_path_data) {
  size_t cnt = 0;
  for (size_t i = 0; i != valid_path_data->size(); ++i) {
    auto& curr_path_data = (*valid_path_data)[i];
    if (curr_path_data.path_label().find("fallback") != std::string::npos) {
      // remove empty path_data.
      if (!curr_path_data.Empty()) {
        if (cnt != i) {
          (*valid_path_data)[cnt] = curr_path_data;
        }
        ++cnt;
      }
      continue;
    }
    SetPathInfo(*reference_line_info, &curr_path_data);
    // Trim all the lane-borrowing paths so that it ends with an in-lane
    // position.
    if (curr_path_data.path_label().find("pullover") == std::string::npos) {
      TrimTailingOutLanePoints(&curr_path_data);
    }

    // find blocking_obstacle_on_selflane, to be used for lane selection later
    if (curr_path_data.path_label().find("self") != std::string::npos) {
      const auto blocking_obstacle_id = curr_path_data.blocking_obstacle_id();
      blocking_obstacle_on_selflane =
          reference_line_info->path_decision()->Find(blocking_obstacle_id);
    }

    // remove empty path_data.
    if (!curr_path_data.Empty()) {
      if (cnt != i) {
        (*valid_path_data)[cnt] = curr_path_data;
      }
      ++cnt;
    }

    // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
    //                 reference_line_info);
    ADEBUG << "For " << curr_path_data.path_label() << ", "
           << "path length = " << curr_path_data.frenet_frame_path().size();
  }
  valid_path_data->resize(cnt);
}

void PathAssessmentDecider::PickOptimalPath(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle* blocking_obstacle_on_selflane,
    const std::vector<PathData>& valid_path_data) {
  AINFO << "Using '" << valid_path_data.front().path_label() << "' path out of "
        << valid_path_data.size() << " path(s)";
  // Obtain the endstate used in the previous frame.
  injector_->last_using_lateral_ = 0.0;
  injector_->last_path_label_ = VaildPathLable::DEFAULT;
  auto* laneborrow_debug = reference_line_info->mutable_debug()
                               ->mutable_planning_data()
                               ->mutable_valid_path_info();
  if (valid_path_data.front().path_label().find("fallback") !=
      std::string::npos) {
    FLAGS_static_obstacle_nudge_l_buffer = 0.8;
    injector_->last_using_lateral_ = injector_->fallback_endstate;
    ADEBUG << "use fallback";
    laneborrow_debug->set_pick_path_label(FLAGS_path_label_is_fallback);
    injector_->last_path_label_ = VaildPathLable::FALLBACK;
  }
  if (valid_path_data.front().path_label().find("self") != std::string::npos) {
    ADEBUG << "use self";
    injector_->last_using_lateral_ = injector_->selfborrow_endstate;
    auto* path_debug = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path();
    path_debug->MergeFrom(
        reference_line_info->debug().planning_data().self_path());
    laneborrow_debug->set_pick_path_label(FLAGS_path_label_is_self);
    injector_->last_path_label_ = VaildPathLable::SELF;
    if (FLAGS_enable_check_self_path_has_turn) {
      const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
      double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
      auto using_path = valid_path_data.front();
      const auto& discretized_path = using_path.discretized_path();
      size_t path_step = 4;
      bool path_has_turn = false;
      bool ref_line_has_turn = false;
      for (size_t i = 0; i < discretized_path.size(); i += path_step) {
        auto kappa = discretized_path[i].kappa();
        //  AINFO << "kappa = " << kappa;
        if (std::fabs(kappa) > kMinKappa) {
          path_has_turn = true;
        }
        //  AINFO << "discretized_path[i].s() = " << discretized_path[i].s();
        hdmap::LaneInfoConstPtr locate_lane =
            reference_line_info_->LocateLaneInfo(s + discretized_path[i].s());

        if (nullptr != locate_lane) {
          // AINFO<<"1";
          common::SLPoint sl;
          sl.set_l(0);
          sl.set_s(s + discretized_path[i].s());
          common::math::Vec2d xy;
          if (!reference_line_info_->reference_line().SLToXY(sl, &xy)) {
            //  AINFO<<"CONTINUE";
            continue;
          }
          double s_projection = s + discretized_path[i].s();
          const auto& reference_point = reference_line_info_->reference_line().GetNearestReferencePoint(s_projection);
          // AINFO<<"reference_point kappa = "<<reference_point.kappa();
          double minddle_kappa = reference_point.kappa();
          // AINFO << "minddle_kappa = " << minddle_kappa;
          if (std::fabs(minddle_kappa) > kMinKappa ||
              locate_lane->lane().turn() != hdmap::Lane::NO_TURN) {
            // if (std::fabs(minddle_kappa) > kMinKappa) {
            ref_line_has_turn = true;
          }
        }
      }
      if (!ref_line_has_turn && path_has_turn) {
        reference_line_info_->SetNeedDiagonal(true);
        // AINFO << "need diagonal";
      }
      // AINFO << "ref_line_has_turn = " << ref_line_has_turn;
      if (ref_line_has_turn) {
        // reference_line_info_->SetNeedDiagonal(false);
      }
    }   
  }
  if (valid_path_data.front().path_label().find("left") != std::string::npos) {
    ADEBUG << "use left laneborrow";
    injector_->last_using_lateral_ = injector_->left_laneborrow_endstate;
    auto* mutable_laneborrow = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_lane_borrow();
    mutable_laneborrow->set_laneborrow_direction(LaneborrowStatus::LEFT_BORROW);
    auto* path_debug = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path();
    path_debug->MergeFrom(
        reference_line_info->debug().planning_data().left_borrow_path());
    laneborrow_debug->set_pick_path_label(FLAGS_path_label_is_left_borrow);
    injector_->last_path_label_ = VaildPathLable::LEFT_BORROW;
  }
  if (valid_path_data.front().path_label().find("right") != std::string::npos) {
    ADEBUG << "use right laneborrow";
    auto* mutable_laneborrow = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_lane_borrow();
    mutable_laneborrow->set_laneborrow_direction(
        LaneborrowStatus::RIGHT_BORROW);
    auto* path_debug = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path();
    path_debug->MergeFrom(
        reference_line_info->debug().planning_data().right_borrow_path());
    laneborrow_debug->set_pick_path_label(FLAGS_path_label_is_right_borrow);
    injector_->last_path_label_ = VaildPathLable::RIGHT_BORROW;
  }
  if (valid_path_data.front().path_label().find("lanechange") !=
      std::string::npos) {
    auto* path_debug = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path();
    path_debug->MergeFrom(
        reference_line_info->debug().planning_data().lanechange_path());
    laneborrow_debug->set_pick_path_label(FLAGS_path_label_is_lane_change);
    injector_->last_path_label_ = VaildPathLable::LANE_CHANGE;
  }
  AINFO << "injector_->last_using_lateral_=" << injector_->last_using_lateral_;
  laneborrow_debug->set_last_using_lateral(injector_->last_using_lateral_);

  *(reference_line_info->mutable_path_data()) = valid_path_data.front();
  reference_line_info->SetBlockingObstacle(
      valid_path_data.front().blocking_obstacle_id());
  auto using_path = valid_path_data.front();
  const auto& discretized_path = using_path.discretized_path();
  size_t path_step = kPathStep;
  bool is_path_straight = true;
  for (size_t i = 0; i < discretized_path.size(); i += path_step) {
    double s= discretized_path[i].s();
    if (s > kConsiderPathLength) {
      break;
    }
    // AINFO<<"consider_s = "<<s;
    auto kappa = discretized_path[i].kappa();
    if(std::fabs(kappa) > kMinKappa){
      is_path_straight = false;
      break;
    }
  }
  if (is_path_straight) {
    reference_line_info_->SetIsPathStraight(true);
  } else {
    reference_line_info_->SetIsPathStraight(false);
  }
  AINFO << "is_path_straight = " << reference_line_info_->IsPathStraight();
}

void PathAssessmentDecider::UpdateNecessaryInfoForLaneBorrow(
    ReferenceLineInfo* const reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // Update side-pass direction.
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    bool left_borrow = false;
    bool right_borrow = false;
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW &&
          reference_line_info->path_data().path_label().find("left") !=
              std::string::npos) {
        left_borrow = true;
      }
      if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW &&
          reference_line_info->path_data().path_label().find("right") !=
              std::string::npos) {
        right_borrow = true;
      }
    }

    if (!left_borrow && !right_borrow) {
      auto fail_time =
          mutable_path_decider_status->lane_borrow_path_fail_times();
      mutable_path_decider_status->set_lane_borrow_path_fail_times(++fail_time);
    } else {
      mutable_path_decider_status->clear_decided_side_pass_direction();

      if (right_borrow) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::RIGHT_BORROW);
      }
      if (left_borrow) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::LEFT_BORROW);
      }
      mutable_path_decider_status->set_lane_borrow_path_fail_times(0);
    }
    if (mutable_path_decider_status->lane_borrow_path_fail_times() >
        config_.path_assessment_decider_config()
            .assess_lane_borrow_exit_times()) {
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      mutable_path_decider_status->clear_decided_side_pass_direction();
      mutable_path_decider_status->set_lane_borrow_path_fail_times(0);
    }
  }
  PathType path_typ = STRAIGHT_PATH;
  double path_typ_end_s = 0.0;
  const auto& frenet_path =
      reference_line_info->path_data().frenet_frame_path();
  // Get the type of path (left LaneBorrow or right LaneBorrow) and the path end
  // s value of that type
  GetPathTpye(frenet_path, &path_typ, &path_typ_end_s);
  injector_->ClearBorrowObstacles();
  UpdateBorrowObstacles(path_typ, reference_line_info, frenet_path,
                        path_typ_end_s);
}

void PathAssessmentDecider::UpdateBorrowObstacles(
    const PathType& path_typ, ReferenceLineInfo* const reference_line_info,
    const FrenetFramePath& frenet_path, const double& path_typ_end_s) {
  if (STRAIGHT_PATH != path_typ) {
    // Initialize the obstacle list (including perceived obstacles and
    // disappearing obstacles) and the number of consecutive disappearances (the
    // number of consecutive disappearances of perceived obstacles is 0)
    auto obstacles_list =
        reference_line_info->path_decision()->obstacles().Items();
    std::vector<int> disappear_counts(obstacles_list.size(), 0);
    for (const auto& disappear_obs : injector_->GetDisappearObstacles()) {
      disappear_counts.emplace_back(disappear_obs.first);
      obstacles_list.emplace_back(&disappear_obs.second);
    }

    size_t idx = 0UL;
    for (const auto* obstacle : obstacles_list) {
      // Filter out unrelated obstacles.
      if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
        ++idx;
        ADEBUG << "Ignore unrelated obstacle ID: " << obstacle->Id();
        continue;
      }
      // Ignore too small obstacles and too large obstacles.
      if (ObstacleSizeUnNormal(*obstacle)) {
        ++idx;
        continue;
      }

      if (IgnoreUnstableObstacle(*obstacle)) {
        ++idx;
        continue;
      }

      ADEBUG << "consider obstacle ID: " << obstacle->Id();

      // Depending on the type of path, record the obstacles being borrowed by
      // ADC
      switch (path_typ) {
        case LEFT_PATH:
          if (LeftPathCanBorrow(reference_line_info, frenet_path, obstacle,
                                path_typ_end_s)) {
            injector_->AddBorrowObstacle(
                BorrowObstacle(disappear_counts[idx], *obstacle));
          }
          break;
        case RIGHT_PATH:
          if (RightPathCanBorrow(reference_line_info, frenet_path, obstacle,
                                 path_typ_end_s)) {
            injector_->AddBorrowObstacle(
                BorrowObstacle(disappear_counts[idx], *obstacle));
          }
          break;
        default:
          AERROR << "path type is not LEFT_PATH or RIGHT_PATH.";
          break;
      }
      ++idx;
    }
  }
}

bool PathAssessmentDecider::IgnoreUnstableObstacle(const Obstacle& obstacle) {
  auto it = injector_->GetAppearedObstacles().find(obstacle.Id());
  if (injector_->GetAppearedObstacles().end() != it) {
    // Ignore the obstacle which type always is UNKNOWN
    // Ignore obstacles that appear in a few consecutive times
    if ((false == std::get<2>(it->second)) ||
        (std::get<0>(it->second) < kMinAppearTimes)) {
      return true;
    }
  }
  return false;
}

bool PathAssessmentDecider::LeftPathCanBorrow(
    ReferenceLineInfo* const reference_line_info,
    const FrenetFramePath& frenet_path, const Obstacle* obstacle,
    const double& path_typ_end_s) {
  double curr_lane_right_width = FLAGS_default_reference_line_width * 0.5;
  auto* find_obs = reference_line_info->path_decision()->Find(obstacle->Id());
  if (find_obs) {
    const auto& lane_width = find_obs->GetLaneWidthBaseOnCenter(
        reference_line_info->reference_line());
    curr_lane_right_width = lane_width.second;
  }
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
  auto obs_frenet_point =
      reference_line_info->path_data().frenet_frame_path().EvaluateByS(
          (obstacle_sl.start_s() + obstacle_sl.end_s()) * 0.5);

  if (adc_sl_boundary.start_s() < obstacle_sl.end_s() &&
      obstacle_sl.end_s() < kMaxScanDistance + adc_sl_boundary.end_s() &&
      obstacle_sl.start_s() < path_typ_end_s &&
      obstacle_sl.start_s() < frenet_path.back().s() &&
      -curr_lane_right_width < obstacle_sl.end_l() &&
      obstacle_sl.end_l() < obs_frenet_point.l()) {
    return true;
  }
  return false;
}

bool PathAssessmentDecider::RightPathCanBorrow(
    ReferenceLineInfo* const reference_line_info,
    const FrenetFramePath& frenet_path, const Obstacle* obstacle,
    const double& path_typ_end_s) {
  double curr_lane_left_width = FLAGS_default_reference_line_width * 0.5;
  auto* find_obs = reference_line_info->path_decision()->Find(obstacle->Id());
  if (find_obs) {
    const auto& lane_width = find_obs->GetLaneWidthBaseOnCenter(
        reference_line_info->reference_line());
    curr_lane_left_width = lane_width.first;
  }
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
  auto obs_frenet_point =
      reference_line_info->path_data().frenet_frame_path().EvaluateByS(
          (obstacle_sl.start_s() + obstacle_sl.end_s()) * 0.5);

  if (adc_sl_boundary.start_s() < obstacle_sl.end_s() &&
      obstacle_sl.end_s() < kMaxScanDistance + adc_sl_boundary.end_s() &&
      obstacle_sl.start_s() < path_typ_end_s &&
      obstacle_sl.start_s() < frenet_path.back().s() &&
      obstacle_sl.start_l() < curr_lane_left_width &&
      obs_frenet_point.l() < obstacle_sl.start_l()) {
    return true;
  }
  return false;
}

void GetPathTpye(const FrenetFramePath& frenet_path,
                 PathType* const ptr_path_typ,
                 double* const ptr_path_typ_end_s) {
  *ptr_path_typ = STRAIGHT_PATH;
  *ptr_path_typ_end_s = 0.0;

  bool found_path_typ = false;
  auto item = frenet_path.begin();
  // Within 30 meters or the entire length of the path, search for the nearest
  // path type
  for (; (item->s() - frenet_path.front().s() < kMaxScanDistance) &&
         (item != frenet_path.end());
       ++item) {
    if (item->l() > kPathTpyeBuffer && RIGHT_PATH != *ptr_path_typ) {
      *ptr_path_typ = LEFT_PATH;
      found_path_typ = true;
    } else if (item->l() < -kPathTpyeBuffer && LEFT_PATH != *ptr_path_typ) {
      *ptr_path_typ = RIGHT_PATH;
      found_path_typ = true;
    } else if (found_path_typ) {
      break;
    }
  }
  std::advance(item, -1);
  *ptr_path_typ_end_s = item->s();
}

bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle,
                     const ReferenceLineInfo* reference_line_info) {
  ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();
  // Empty path_data is never the larger one.
  if (lhs.Empty()) {
    ADEBUG << "LHS is empty.";
    return false;
  }
  if (rhs.Empty()) {
    ADEBUG << "RHS is empty.";
    return true;
  }

  // Regular path goes before fallback path.
  bool lhs_is_regular = lhs.path_label().find("regular") != std::string::npos;
  bool rhs_is_regular = rhs.path_label().find("regular") != std::string::npos;
  if (lhs_is_regular != rhs_is_regular) {
    return lhs_is_regular;
  }

  bool lhs_is_lanechange =
      lhs.path_label().find("lanechange") != std::string::npos;
  bool rhs_is_lanechange =
      rhs.path_label().find("lanechange") != std::string::npos;
  if (lhs_is_lanechange != rhs_is_lanechange) {
    return lhs_is_lanechange;
  }
  // Select longer path.
  // If roughly same length, then select laneborrow path.
  bool lhs_on_selflane = lhs.path_label().find("self") != std::string::npos;
  bool rhs_on_selflane = rhs.path_label().find("self") != std::string::npos;
  double lhs_path_length = lhs.frenet_frame_path().back().s();
  double rhs_path_length = rhs.frenet_frame_path().back().s();
  // prefer laneborrow to selfborrow.
  if (lhs_on_selflane || rhs_on_selflane) {
    if (std::fabs(lhs_path_length - rhs_path_length) >
        kSelfPathLengthComparisonTolerance) {
      return lhs_path_length > rhs_path_length;
    } else {
      return !lhs_on_selflane;
    }
  } else {
    bool lhs_on_rightlane = lhs.path_label().find("right") != std::string::npos;
    bool rhs_on_rightlane = rhs.path_label().find("right") != std::string::npos;
    ACHECK(lhs_on_rightlane || rhs_on_rightlane);
    // LeftBorrow path will be choiced, when the difference between the lengths
    // of the two paths is less than 25m, or LeftBorrow path is longer than
    // RightBorrow.
    if ((std::fabs(lhs_path_length - rhs_path_length) <=
         kSelfPathLengthComparisonTolerance) ||
        (lhs_on_rightlane ^ (lhs_path_length > rhs_path_length))) {
      return !lhs_on_rightlane;
    }
  }

  return SelectPathData(lhs, rhs, blocking_obstacle);
}

bool UseCloseToOverlapStrategy(const ReferenceLineInfo* reference_line_info,
                               const PathData& lhs, const PathData& rhs,
                               bool* const overlap_strategy) {
  // used fallback path before the traffic light to avoid SelfBorrow.
  const auto& indexed_obstacle =
      reference_line_info->path_decision().obstacles();
  const auto& lane_width = reference_line_info->GetLaneWidthBaseOnAdcCenter();
  double lane_left_width = lane_width.first;
  double lane_right_width = lane_width.second;
  const auto adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  const auto& first_encountered_overlaps =
      reference_line_info->FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    // TODO(zhong hao): Consider the CROSSWALK type
    if (ReferenceLineInfo::SIGNAL != overlap.first) {
      continue;
    }
    ADEBUG << "overlap.second.start_s: " << overlap.second.start_s;
    ADEBUG << "adc_end_s : " << adc_end_s;
    auto distance = overlap.second.start_s - adc_end_s;
    // when ADC is near the traffic light.
    if (distance < kNearTrafficLightDistance &&
        distance > -kFarAwayTrafficLightBuffer) {
      for (const auto& obstacle : indexed_obstacle.Items()) {
        const auto& obstacle_type = obstacle->Perception().type();
        double obstacle_end_s = obstacle->PerceptionSLBoundary().end_s();
        double obstacle_to_stop_sign_distance =
            overlap.second.start_s - obstacle_end_s;
        double obstacle_start_l = obstacle->PerceptionSLBoundary().start_l();
        double obstacle_end_l = obstacle->PerceptionSLBoundary().end_l();
        if (PerceptionObstacle::VEHICLE == obstacle_type &&
            obstacle_to_stop_sign_distance < distance &&
            obstacle_to_stop_sign_distance > 0 &&
            obstacle_end_l <= -lane_right_width + kLateralDistanceBuffer &&
            obstacle_end_l >= -lane_right_width) {
          bool lhs_is_self = lhs.path_label().find("self") != std::string::npos;
          bool rhs_is_self = rhs.path_label().find("self") != std::string::npos;
          if (lhs_is_self || rhs_is_self) {
            *overlap_strategy = lhs_is_self;
            return true;
          }
        } else if (PerceptionObstacle::BICYCLE == obstacle_type &&
                   obstacle_to_stop_sign_distance < distance &&
                   obstacle_to_stop_sign_distance > 0 &&
                   obstacle_start_l >= -lane_right_width &&
                   obstacle_end_l <= lane_left_width) {
          bool lhs_is_fallback =
              lhs.path_label().find("fallback") != std::string::npos;
          bool rhs_is_fallback =
              rhs.path_label().find("fallback") != std::string::npos;
          if (lhs_is_fallback || rhs_is_fallback) {
            *overlap_strategy = lhs_is_fallback;
            return true;
          }
        }
      }
      break;
    }
  }
  return false;
}

bool SelectPathData(const PathData& lhs, const PathData& rhs,
                    const Obstacle* blocking_obstacle) {
  // If roughly same length, and must borrow neighbor lane,
  // then prefer to borrow forward lane rather than reverse lane.
  int lhs_on_reverse =
      ContainsOutOnReverseLane(lhs.path_point_decision_guide());
  int rhs_on_reverse =
      ContainsOutOnReverseLane(rhs.path_point_decision_guide());
  // TODO(jiacheng): make this a flag.
  if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
    return lhs_on_reverse < rhs_on_reverse;
  }

  // For two lane-borrow directions, based on ADC's position,
  // select the more convenient one.
  if ((lhs.path_label().find("left") != std::string::npos &&
       rhs.path_label().find("right") != std::string::npos) ||
      (lhs.path_label().find("right") != std::string::npos &&
       rhs.path_label().find("left") != std::string::npos)) {
    if (blocking_obstacle) {
      // select left/right path based on blocking_obstacle's position
      const double obstacle_l =
          (blocking_obstacle->PerceptionSLBoundary().start_l() +
           blocking_obstacle->PerceptionSLBoundary().end_l()) *
          0.5;
      double hy_obs_l =
          obstacles_interval.HyValue(*blocking_obstacle, obstacle_l);
      ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l[" << obstacle_l
             << "] hy_l[" << hy_obs_l << "]";
      return (hy_obs_l > 0.0
                  ? (lhs.path_label().find("right") != std::string::npos)
                  : (lhs.path_label().find("left") != std::string::npos));
    } else {
      // select left/right path based on ADC's position
      double adc_l = lhs.frenet_frame_path().front().l();
      if (adc_l < -kAdcPositionBuffer) {
        return lhs.path_label().find("right") != std::string::npos;
      } else if (adc_l > kAdcPositionBuffer) {
        return lhs.path_label().find("left") != std::string::npos;
      }
    }
  }

  // If same length, both neighbor lane are forward,
  // then select the one that returns to in-lane earlier.
  int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
  int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
  double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
  double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
  if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
    return lhs_back_idx < rhs_back_idx;
  }

  // If same length, both forward, back to inlane at same time,
  // select the left one to side-pass.
  bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
  bool rhs_on_leftlane = rhs.path_label().find("left") != std::string::npos;
  if (lhs_on_leftlane != rhs_on_leftlane) {
    ADEBUG << "Select " << (lhs_on_leftlane ? "left" : "right") << " lane over "
           << (!lhs_on_leftlane ? "left" : "right") << " lane.";
    return lhs_on_leftlane;
  }

  // Otherwise, they are the same path, lhs is not < rhs.
  return false;
}

bool PathAssessmentDecider::IsValidRegularPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Basic sanity checks.
  if (path_data.Empty()) {
    ADEBUG << path_data.path_label() << ": path data is empty.";
    return false;
  }
  // Check if the path is greatly off the reference line.
  if (IsGreatlyOffReferenceLine(path_data)) {
    ADEBUG << path_data.path_label() << ": ADC is greatly off reference line.";
    return false;
  }
  // Check if the path is greatly off the road.
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    ADEBUG << path_data.path_label() << ": ADC is greatly off road.";
    return false;
  }
  // Check if there is any collision.
  // When using asymptotically convex boundaries, the path intersecting the
  // obstacle cannot be removed
  if (config_.path_assessment_decider_config()
          .enable_path_assessment_check_collision() &&
      IsCollidingWithStaticObstacles(reference_line_info, path_data)) {
    AINFO << path_data.path_label() << ": ADC has collision.";
    return false;
  }
  // This is currently of no use to us (the introduced end state offset may
  // cause conflicts)
  // if (IsStopOnReverseNeighborLane(reference_line_info, path_data)) {
  //   ADEBUG << path_data.path_label() << ": stop at reverse neighbor lane";
  //   return false;
  // }

  return true;
}

bool PathAssessmentDecider::IsValidFallbackPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Basic sanity checks.
  if (path_data.Empty()) {
    ADEBUG << "Fallback Path: path data is empty.";
    return false;
  }
  // Check if the path is greatly off the reference line.
  if (IsGreatlyOffReferenceLine(path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off reference line.";
    return false;
  }
  // Check if the path is greatly off the road.
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off road.";
    return false;
  }
  return true;
}

void PathAssessmentDecider::SetPathInfo(
    const ReferenceLineInfo& reference_line_info, PathData* const path_data) {
  // Go through every path_point, and label its:
  //  - in-lane/out-of-lane info (side-pass or lane-change)
  //  - distance to the closest obstacle.
  std::vector<PathPointDecision> path_decision;

  // 0. Initialize the path info.
  InitPathPointDecision(*path_data, &path_decision);

  // 1. Label caution types, differently for side-pass or lane-change.
  if (reference_line_info.IsChangeLanePath()) {
    // If lane-change, then label the lane-changing part to
    // be out-on-forward lane.
    SetPathPointType(reference_line_info, *path_data, true, &path_decision);
  } else {
    // Otherwise, only do the label for borrow-lane generated paths.
    // if (path_data->path_label().find("fallback") == std::string::npos &&
    //     path_data->path_label().find("self") == std::string::npos) {
    SetPathPointType(reference_line_info, *path_data, false, &path_decision);
    // }
  }

  // SetObstacleDistance(reference_line_info, *path_data, &path_decision);
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

void PathAssessmentDecider::TrimTailingOutLanePoints(
    PathData* const path_data) {
  // Don't trim regular-lane path or fallback path.
  if (path_data->path_label().find("fallback") != std::string::npos ||
      path_data->path_label().find("regular") != std::string::npos) {
    return;
  }

  // Trim.
  ADEBUG << "Trimming " << path_data->path_label();
  auto frenet_path = path_data->frenet_frame_path();
  auto path_point_decision = path_data->path_point_decision_guide();
  CHECK_EQ(frenet_path.size(), path_point_decision.size());
  while (!path_point_decision.empty() &&
         std::get<1>(path_point_decision.back()) !=
             PathData::PathPointType::IN_LANE) {
    if (std::get<1>(path_point_decision.back()) ==
        PathData::PathPointType::OUT_ON_FORWARD_LANE) {
      ADEBUG << "Trimming out forward lane point";
    } else if (std::get<1>(path_point_decision.back()) ==
               PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      ADEBUG << "Trimming out reverse lane point";
    } else {
      ADEBUG << "Trimming unknown lane point";
    }
    frenet_path.pop_back();
    path_point_decision.pop_back();
  }
  path_data->SetFrenetPath(std::move(frenet_path));
  path_data->SetPathPointDecisionGuide(std::move(path_point_decision));
}

bool PathAssessmentDecider::IsGreatlyOffReferenceLine(
    const PathData& path_data) {
  const auto& frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    if (std::fabs(frenet_path_point.l()) > kOffReferenceLineThreshold) {
      ADEBUG << "Greatly off reference line at s = " << frenet_path_point.s()
             << ", with l = " << frenet_path_point.l();
      return true;
    }
  }
  return false;
}

bool PathAssessmentDecider::IsGreatlyOffRoad(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  const auto& frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    if (reference_line_info.reference_line().GetRoadWidth(
            frenet_path_point.s(), &road_left_width, &road_right_width)) {
      if (frenet_path_point.l() > road_left_width + kOffRoadThreshold ||
          frenet_path_point.l() < -road_right_width - kOffRoadThreshold) {
        ADEBUG << "Greatly off-road at s = " << frenet_path_point.s()
               << ", with l = " << frenet_path_point.l();
        return true;
      }
    }
  }
  return false;
}

bool PathAssessmentDecider::IsCollidingWithStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Get all obstacles and convert them into frenet-frame polygons.
  std::vector<Polygon2d> obstacle_polygons;
  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Filter out unrelated obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    if (!obstacle->IsCanPass()) {
      ADEBUG << "obstacle: " << obstacle->Id() << " can not static pass";
      continue;
    }
    // Ignore too small obstacles.
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
            (obstacle_sl.end_l() - obstacle_sl.start_l()) <
        kMinObstacleArea) {
      continue;
    }
    // Convert into polygon and save it.
    obstacle_polygons.push_back(
        Polygon2d({Vec2d(obstacle_sl.start_s(), obstacle_sl.start_l()),
                   Vec2d(obstacle_sl.start_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.start_l())}));
  }

  // Go through all the four corner points at every path pt, check collision.
  for (size_t i = 0; i < path_data.discretized_path().size(); ++i) {
    if (path_data.frenet_frame_path().back().s() -
            path_data.frenet_frame_path()[i].s() <
        (kNumExtraTailBoundPoint + 1) * kPathBoundsDeciderResolution) {
      break;
    }
    const auto& path_point = path_data.discretized_path()[i];
    // Get the four corner points ABCD of ADC at every path point.
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();
    for (const auto& corner_point : ABCDpoints) {
      // For each corner point, project it onto reference_line
      common::SLPoint curr_point_sl;
      if (!reference_line_info.reference_line().XYToSL(corner_point,
                                                       &curr_point_sl)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return true;
      }
      auto curr_point = Vec2d(curr_point_sl.s(), curr_point_sl.l());
      // Check if it's in any polygon of other static obstacles.
      for (const auto& obstacle_polygon : obstacle_polygons) {
        if (obstacle_polygon.IsPointIn(curr_point)) {
          ADEBUG << "ADC is colliding with obstacle at path s = "
                 << path_point.s();
          return true;
        }
      }
    }
  }

  return false;
}

bool PathAssessmentDecider::IsStopOnReverseNeighborLane(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  if (path_data.path_label().find("left") == std::string::npos &&
      path_data.path_label().find("right") == std::string::npos) {
    return false;
  }

  std::vector<common::SLPoint> all_stop_point_sl =
      reference_line_info.GetAllStopDecisionSLPoint();
  if (all_stop_point_sl.empty()) {
    return false;
  }

  double check_s = 0.0;
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  for (const auto& stop_point_sl : all_stop_point_sl) {
    if (stop_point_sl.s() - adc_end_s < kLookForwardBuffer) {
      continue;
    }
    check_s = stop_point_sl.s();
    break;
  }
  if (check_s <= 0.0) {
    return false;
  }

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line_info.reference_line().GetLaneWidth(
          check_s, &lane_left_width, &lane_right_width)) {
    return false;
  }

  common::SLPoint path_point_sl;
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    if (std::fabs(frenet_path_point.s() - check_s) < kSDelta) {
      path_point_sl.set_s(frenet_path_point.s());
      path_point_sl.set_l(frenet_path_point.l());
    }
  }
  ADEBUG << "path_point_sl[" << path_point_sl.s() << ", " << path_point_sl.l()
         << "] lane_left_width[" << lane_left_width << "] lane_right_width["
         << lane_right_width << "]";

  hdmap::Id neighbor_lane_id;
  double neighbor_lane_width = 0.0;
  if (path_data.path_label().find("left") != std::string::npos &&
      path_point_sl.l() > lane_left_width) {
    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::LeftReverse, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      ADEBUG << "stop path point at LeftReverse neighbor lane["
             << neighbor_lane_id.id() << "]";
      return true;
    }
  } else if (path_data.path_label().find("right") != std::string::npos &&
             path_point_sl.l() < -lane_right_width) {
    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::RightReverse, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      ADEBUG << "stop path point at RightReverse neighbor lane["
             << neighbor_lane_id.id() << "]";
      return true;
    }
  }
  return false;
}

void PathAssessmentDecider::InitPathPointDecision(
    const PathData& path_data,
    std::vector<PathPointDecision>* const path_point_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_point_decision);
  path_point_decision->clear();

  // Go through every path point in path data, and initialize a
  // corresponding path point decision.
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    path_point_decision->emplace_back(frenet_path_point.s(),
                                      PathData::PathPointType::UNKNOWN,
                                      std::numeric_limits<double>::max());
  }
}

void PathAssessmentDecider::SetPathPointType(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data,
    const bool is_lane_change_path,
    std::vector<PathPointDecision>* const path_point_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_point_decision);

  // Go through every path_point, and add in-lane/out-of-lane info.
  const auto& discrete_path = path_data.discretized_path();
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_length = vehicle_config.vehicle_param().length();
  const double ego_width = vehicle_config.vehicle_param().width();
  const double ego_back_to_center =
      vehicle_config.vehicle_param().back_edge_to_center();
  const double ego_center_shift_distance =
      ego_length * 0.5 - ego_back_to_center;

  bool is_prev_point_out_lane = false;
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    const auto& rear_center_path_point = discrete_path[i];
    const double ego_theta = rear_center_path_point.theta();
    Box2d ego_box({rear_center_path_point.x(), rear_center_path_point.y()},
                  ego_theta, ego_length, ego_width);
    Vec2d shift_vec{ego_center_shift_distance * std::cos(ego_theta),
                    ego_center_shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    SLBoundary ego_sl_boundary;
    if (!reference_line_info.reference_line().GetSLBoundary(ego_box,
                                                            &ego_sl_boundary)) {
      ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
      continue;
    }
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s =
        (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) * 0.5;
    if (reference_line_info.reference_line().GetLaneWidth(
            middle_s, &lane_left_width, &lane_right_width)) {
      // Rough sl boundary estimate using single point lane width
      double back_to_inlane_extra_buffer = 0.2;
      double in_and_out_lane_hysteresis_buffer =
          is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;

      // Check for lane-change and lane-borrow differently:
      if (is_lane_change_path) {
        // For lane-change path, only transitioning part is labeled as
        // out-of-lane.
        if (ego_sl_boundary.start_l() > lane_left_width ||
            ego_sl_boundary.end_l() < -lane_right_width) {
          // This means that ADC hasn't started lane-change yet.
          std::get<1>((*path_point_decision)[i]) =
              PathData::PathPointType::IN_LANE;
        } else if (ego_sl_boundary.start_l() >
                       -lane_right_width + back_to_inlane_extra_buffer &&
                   ego_sl_boundary.end_l() <
                       lane_left_width - back_to_inlane_extra_buffer) {
          // This means that ADC has safely completed lane-change with margin.
          std::get<1>((*path_point_decision)[i]) =
              PathData::PathPointType::IN_LANE;
        } else {
          // ADC is right across two lanes.
          std::get<1>((*path_point_decision)[i]) =
              PathData::PathPointType::OUT_ON_FORWARD_LANE;
        }
      } else {
        // For lane-borrow path, as long as ADC is not on the lane of
        // reference-line, it is out on other lanes. It might even be
        // on reverse lane!
        if (ego_sl_boundary.end_l() >
                lane_left_width + in_and_out_lane_hysteresis_buffer ||
            ego_sl_boundary.start_l() <
                -lane_right_width - in_and_out_lane_hysteresis_buffer) {
          if (path_data.path_label().find("reverse") != std::string::npos) {
            std::get<1>((*path_point_decision)[i]) =
                PathData::PathPointType::OUT_ON_REVERSE_LANE;
          } else if (path_data.path_label().find("forward") !=
                     std::string::npos) {
            std::get<1>((*path_point_decision)[i]) =
                PathData::PathPointType::OUT_ON_FORWARD_LANE;
          } else {
            std::get<1>((*path_point_decision)[i]) =
                PathData::PathPointType::UNKNOWN;
          }
          if (!is_prev_point_out_lane) {
            if (ego_sl_boundary.end_l() >
                    lane_left_width + back_to_inlane_extra_buffer ||
                ego_sl_boundary.start_l() <
                    -lane_right_width - back_to_inlane_extra_buffer) {
              is_prev_point_out_lane = true;
            }
          }
        } else {
          // The path point is within the reference_line's lane.
          std::get<1>((*path_point_decision)[i]) =
              PathData::PathPointType::IN_LANE;
          if (is_prev_point_out_lane) {
            is_prev_point_out_lane = false;
          }
        }
      }
    } else {
      AERROR << "reference line not ready when setting path point guide";
      return;
    }
  }
}

void PathAssessmentDecider::SetObstacleDistance(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data,
    std::vector<PathPointDecision>* const path_point_decision) {
  // Sanity checks
  CHECK_NOTNULL(path_point_decision);

  // Get all obstacles and convert them into frenet-frame polygons.
  std::vector<Polygon2d> obstacle_polygons;
  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Filter out unrelated obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Convert into polygon and save it.
    const auto& obstacle_box = obstacle->PerceptionBoundingBox();
    if (obstacle_box.area() < kMinObstacleArea) {
      continue;
    }

    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type) {
      const auto& obs_polygon = obstacle->PerceptionPolygon();
      obstacle_polygons.emplace_back(obs_polygon);
    } else {
      obstacle_polygons.emplace_back(obstacle_box);
    }
  }

  // Go through every path point, update closest obstacle info.
  const auto& discrete_path = path_data.discretized_path();
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    const auto& path_point = discrete_path[i];
    // Get the bounding box of the vehicle at that point.
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    // Go through all the obstacle polygons, and update the min distance.
    double min_distance_to_obstacles = std::numeric_limits<double>::max();
    for (const auto& obstacle_polygon : obstacle_polygons) {
      double distance_to_vehicle = obstacle_polygon.DistanceTo(vehicle_box);
      min_distance_to_obstacles =
          std::min(min_distance_to_obstacles, distance_to_vehicle);
    }
    std::get<2>((*path_point_decision)[i]) = min_distance_to_obstacles;
  }
}

void PathAssessmentDecider::RecordDebugInfo(
    const PathData& path_data, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_optimized_path->set_name(debug_name);
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

int ContainsOutOnReverseLane(
    const std::vector<PathPointDecision>& path_point_decision) {
  int ret = 0;
  for (const auto& curr_decision : path_point_decision) {
    if (std::get<1>(curr_decision) ==
        PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      ++ret;
    }
  }
  return ret;
}

int GetBackToInLaneIndex(
    const std::vector<PathPointDecision>& path_point_decision) {
  // ACHECK(!path_point_decision.empty());
  // ACHECK(std::get<1>(path_point_decision.back()) ==
  //       PathData::PathPointType::IN_LANE);

  for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
    if (std::get<1>(path_point_decision[i]) !=
        PathData::PathPointType::IN_LANE) {
      return i;
    }
  }
  return 0;
}

}  // namespace planning
}  // namespace century
