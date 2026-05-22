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
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/kinematic_speed_optimizer/constant_jerk_speed_optimizer.h"
#include "modules/planning/common/kinematic_speed_optimizer/kinematic_deceleration_speed_optimizer.h"
#include "modules/planning/common/kinematic_speed_optimizer/two_piecewise_jerk_speed_optimizer.h"
#include "modules/planning/common/kinematic_speed_optimizer/two_piecewise_stationary_jerk_speed_optimizer.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace century {
namespace planning {
namespace {
constexpr double kAccWeight = 30.0;
constexpr double kJerkWeight = 90.0;
constexpr double kForwardToSignalOverlap = 20.0;
constexpr double kBackwardToSignalOverlap = 3.0;
constexpr double kMaxVelocitySatrt = 4.0;
constexpr double kMaxAcc = 6.0;
constexpr double kComfortableAcc = 3.0;
constexpr double kTimes = 1.1;
constexpr double kMaxCoeff = 10000.0;
constexpr double kSecondCoeff = 0.1;
constexpr double kMinCoeff = 0.01;
constexpr double kSpeedCoeff = 1.2;
constexpr double kComfotableAcc = 1.0;
constexpr double kMinOffset = 0.1;
constexpr double kStopDistanceBuffer = 0.5;
constexpr double kMinVelocity = 1e-8;
constexpr double kDeltaT = 0.1;
constexpr double kMinDecel = -6.0;
constexpr double kMinJerk = -4.0;
constexpr double kMinAccForSlowerDecel = -4.0;
constexpr double kMaxAccForSlowerDecel = 0.0;
constexpr size_t kFrontPathPointNum = 4UL;
constexpr size_t kHyCapacity = 50UL;
constexpr double kDegrees = 90.0;
constexpr double kEpison = 1e-5;
constexpr double kMinKappa = 0.005;
constexpr double kLowStartUpAcc = 0.01;
constexpr double kLowStartUpTargetAcc = 1000.0;
constexpr double kWeightOneLevelAcc = 10000.0;
constexpr double kWeightTwoLevelAcc = 20000.0;
constexpr double kWeightThreeLevelAcc = 40000.0;
constexpr double kWeightFourLevelAcc = 90000.0;
constexpr double kMinSpeed = 0.1;
constexpr double kStopSpeed = 0.03;
constexpr double kSartUpSpeed = 0.3;
constexpr double kMinAcc = -0.001;
constexpr int kMaxStartUpCount = 5;
}  // namespace

using century::common::ErrorCode;
using century::common::PathPoint;
using century::common::SpeedPoint;
using century::common::Status;
using century::common::TrajectoryPoint;

static HysteresisInterval adc_obstacles_heading_interval(
    FLAGS_max_diff_angle_for_stoped_collision_check,
    FLAGS_hy_buffer_lower_for_stoped_collision_check,
    FLAGS_hy_buffer_upper_for_stoped_collision_check, kHyCapacity);

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : SpeedOptimizer(config, injector) {
  ACHECK(config_.has_piecewise_jerk_speed_optimizer_config());
}

const Obstacle* PiecewiseJerkSpeedOptimizer::GetCloseObstacle(
    double* close_key_point_s) {
  std::string close_obs_id = "";
  const auto& indexed_obstacles =
      reference_line_info_->path_decision()->obstacles();
  for (const auto* ptr_obs : indexed_obstacles.Items()) {
    if (!ptr_obs->HasLongitudinalDecision() ||
        !ptr_obs->is_path_st_boundary_initialized()) {
      continue;
    }
    if (ptr_obs->LongitudinalDecision().has_follow() ||
        ptr_obs->LongitudinalDecision().has_yield() ||
        ptr_obs->LongitudinalDecision().has_stop()) {
      double key_point_s = ptr_obs->path_st_boundary().min_s();
      if (*close_key_point_s > key_point_s) {
        *close_key_point_s = key_point_s;
        close_obs_id = ptr_obs->Id();
      }
    }
  }

  const Obstacle* close_obs = nullptr;
  if (close_obs_id.find("TL") != std::string::npos ||
      close_obs_id.find("SS") != std::string::npos ||
      close_obs_id.find("DEST") != std::string::npos) {
    close_obs = indexed_obstacles.Find(close_obs_id);
  }
  return close_obs;
}

bool PiecewiseJerkSpeedOptimizer::KinematicSpeedOptimize(
    double s_0, const std::array<double, 3>& init_s,
    SpeedData* const speed_data) {
  bool ret = false;
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  double close_key_point_s = std::numeric_limits<double>::max();
  const Obstacle* close_obs = GetCloseObstacle(&close_key_point_s);
  if ((nullptr != close_obs &&
       STBoundary::BoundaryType::STOP ==
           close_obs->path_st_boundary().boundary_type() &&
       close_key_point_s <
           config.max_dis_to_enable_kinematic_speed_optimizer()) ||
      (s_0 > kEpison)) {
    ADEBUG << "the stop obstacle id:(" << close_obs->Id() << "), min_s:("
           << close_obs->path_st_boundary().min_s() << ")";

    double s0 = close_key_point_s - config.stop_distance_buffer_for_kinematic();
    // ADEBUG << "BEFORE s0 = " << s0;
    double v0 = init_s[1];
    double a0 = init_s[2];
    if (s_0 > kEpison) {
      s0 = s_0;
    }
    // ADEBUG << "AFTER s0 = " << s0;
    std::unique_ptr<KinematicSpeedBaseOptimizer> kinematic_speed_opt = nullptr;
    switch (config.kinematic_speed_optimizer_type()) {
      case KinematicSpeedOptimizerType::CONSTANT_JERK:
        kinematic_speed_opt.reset(new ConstantJerkSpeedOptimizer(
            static_cast<size_t>(num_of_knots_), kDeltaT, config));
        ADEBUG
            << "select the ConstantJerkSpeedOptimizer as the SpeedOptimizer.";
        break;
      case KinematicSpeedOptimizerType::TWO_PIECEWISE_JERK:
        kinematic_speed_opt.reset(new TwoPiecewiseJerkSpeedOptimizer(
            static_cast<size_t>(num_of_knots_), kDeltaT, config));
        ADEBUG << "select the TwoPiecewiseJerkSpeedOptimizer as the "
                  "SpeedOptimizer.";
        break;
      case KinematicSpeedOptimizerType::TWO_PIECEWISE_STATIONARY_JERK:
        kinematic_speed_opt.reset(new TwoPiecewiseStationaryJerkSpeedOptimizer(
            static_cast<size_t>(num_of_knots_), kDeltaT, config));
        ADEBUG << "select the TwoPiecewiseStationaryJerkSpeedOptimizer as "
                  "the SpeedOptimizer.";
        break;
      default:
        kinematic_speed_opt.reset(new TwoPiecewiseStationaryJerkSpeedOptimizer(
            static_cast<size_t>(num_of_knots_), kDeltaT, config));
        ADEBUG << "select the TwoPiecewiseStationaryJerkSpeedOptimizer as "
                  "the SpeedOptimizer.";
        break;
    }
    kinematic_speed_opt->Init(s0, v0, a0);
    speed_data->clear();
    bool ret_kinematic_speed_opt =
        kinematic_speed_opt->GenerateSpeedDataByStopDecision(speed_data);
    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    for (auto sp : *speed_data) {
      ADEBUG << "KinematicSpeed For t[" << sp.t() << "], s = " << sp.s()
             << ", v = " << sp.v() << ", a = " << sp.a();
    }
    if (ret_kinematic_speed_opt) {
      ret = true;
    } else {
      ADEBUG << "Need continue use PiecewiseJerkSpeedOptimizer.";
    }
  }
  ADEBUG << "select the PiecewiseJerkSpeedOptimizer as the SpeedOptimizer.";
  return ret;
}

double PiecewiseJerkSpeedOptimizer::GetTargetAcc(
    const SpeedData& reference_speed_data, const double init_v) {
  SpeedPoint sp_end;
  reference_speed_data.EvaluateByTime((num_of_knots_ - 1) * kDeltaT, &sp_end);

  bool has_target_v = false;
  double min_v_tem = init_v;
  double min_s_tem = sp_end.s();
  bool has_min_v = false;
  double target_v = init_v;
  double target_s = sp_end.s();

  // if slow down
  for (size_t i = 1; i < reference_speed_data.size() - 1; ++i) {
    ADEBUG << "reference_speed_data[i].v() = " << reference_speed_data[i].v();
    if (reference_speed_data[i].v() + kMinOffset <
        reference_speed_data[i - 1].v()) {
      ADEBUG << "dp min v = " << reference_speed_data[i].v()
             << ", reference_speed_data[i].s=" << reference_speed_data[i].s();
      min_v_tem = std::min(min_v_tem, reference_speed_data[i].v());
      min_s_tem = reference_speed_data[i].s();
      has_min_v = true;
      continue;
    }
    if (reference_speed_data[i].v() + kMinOffset <
        reference_speed_data[i + 1].v()) {
      if (has_min_v) {
        target_s = min_s_tem;
        target_v = min_v_tem;
        has_target_v = true;
      }
      break;
    }
  }
  ADEBUG << "has_min_v = " << has_min_v << ", has_target_v = " << has_target_v;

  double max_v_tem = init_v;
  double max_s_tem = sp_end.s();
  bool has_max_v = false;
  // if speed up
  if (!has_target_v && !has_min_v) {
    for (size_t i = 1; i < reference_speed_data.size() - 1; ++i) {
      if (reference_speed_data[i].v() - kMinOffset >
          reference_speed_data[i - 1].v()) {
        ADEBUG << "dp max v = " << reference_speed_data[i].v()
               << ", reference_speed_data[i].s=" << reference_speed_data[i].s();
        max_v_tem = std::max(max_v_tem, reference_speed_data[i].v());
        max_s_tem = reference_speed_data[i].s();
        has_max_v = true;
        continue;
      }
      if (reference_speed_data[i].v() - kMinOffset >
          reference_speed_data[i + 1].v()) {
        if (has_max_v) {
          target_s = max_s_tem;
          target_v = max_v_tem;
          has_target_v = true;
        }
        break;
      }
    }
  }
  ADEBUG << "has_max_v = " << has_max_v << ", has_target_v = " << has_target_v;

  if (!has_target_v) {
    target_v = reference_speed_data[reference_speed_data.size() - 2].v();
    target_s = sp_end.s();
    ADEBUG << " monotonic or uniform";
  }
  ADEBUG << "target_v = " << target_v << ", target_s = " << target_s;

  // get dp target acc
  double target_a =
      (target_v * target_v - init_v * init_v) / (2.0 * std::max(target_s, 0.1));
  ADEBUG << "target_a = " << target_a;
  return target_a;
}

void PiecewiseJerkSpeedOptimizer::GetWightCoeffs(
    const double target_a, const bool use_ref_v,
    std::array<double, 4U>* weight_coeffs) {
  // dp target slow down ,but not comforttacle acc
  if (target_a < -kComfotableAcc) {
    (*weight_coeffs)[2] = kSecondCoeff;
    (*weight_coeffs)[3] = kSecondCoeff;
    // (*weight_coeffs)[0] = kMinCoeff;
  }
  if (use_ref_v) {
    // no trust DP result, use reference_v
    // (*weight_coeffs)[2] = kSecondCoeff;
    // (*weight_coeffs)[3] = kSecondCoeff;
    // (*weight_coeffs)[0] = kMinCoeff;
    (*weight_coeffs)[1] = kMaxCoeff;
  }
  ADEBUG << "coeff_acc = " << (*weight_coeffs)[2]
         << ", coeff_jerk =" << (*weight_coeffs)[3]
         << ", coeff_ref_s = " << (*weight_coeffs)[0]
         << ", coeff_ref_v = " << (*weight_coeffs)[1];
}

void PiecewiseJerkSpeedOptimizer::GetWights(
    const double target_a, const std::array<double, 4U>& weight_coeffs,
    const std::array<double, 3U>& init_s, std::array<double, 4U>* weights) {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  if (FLAGS_enable_dynamic_modify_piecewise_jerk_weight) {
    (*weights)[2] = config.acc_weight_dynamic() * weight_coeffs[2];
    (*weights)[3] = config.jerk_weight_dynamic() * weight_coeffs[3];
    (*weights)[0] = config.ref_s_weight_dynamic() * weight_coeffs[0];
    (*weights)[1] = config.ref_v_weight_dynamic() * weight_coeffs[1];
  }

  const double init_v = init_s[1];
  const double init_a = init_s[2];
  if (FLAGS_enable_acc_start) {
    const auto& reference_line = reference_line_info_->reference_line();
    common::SLPoint last_signal_overlap_end_sl;
    if (reference_line.XYToSL(injector_->last_signal_overlap_end_xy_,
                              &last_signal_overlap_end_sl)) {
      // 1.less 4m/s;2.signal overlap front and rear -3m and 20m;3.start;
      if (init_v < kMaxVelocitySatrt && init_a > 0.0 &&
          reference_line_info_->AdcSlBoundary().end_s() >
              last_signal_overlap_end_sl.s() - kBackwardToSignalOverlap &&
          reference_line_info_->AdcSlBoundary().start_s() -
                  last_signal_overlap_end_sl.s() <
              kForwardToSignalOverlap &&
          reference_line_info_->GetLaneType() != hdmap::Lane::PLAY_STREET) {
        (*weights)[2] = config.acc_weight_for_trafficlight();
        (*weights)[3] = config.jerk_weight_for_trafficlight();
      }
    }
  }
  if (reference_line_info_->IsLowStartUp() && init_a > 0.0) {
    (*weights)[2] = config.acc_weight();
    (*weights)[3] = config.jerk_weight();
  }

  // AINFO << "target_a = " << target_a;
  // AINFO << "init_v = " << init_v;
  // AINFO << "init_a = " << init_a;
  // target_a > 0.0 DP want to speed up
  if (FLAGS_enable_change_acc_weight_for_svm && target_a > kLowStartUpAcc &&
      init_v < config_.piecewise_jerk_speed_optimizer_config()
                   .low_start_up_speed() &&
      !util::IsOvertake(injector_->planning_context()) &&
      !util::IsLaneChange(injector_->planning_context())) {
    if (century::planning::Frame::AccLevel::ACC_NORMAL == frame_->acc_level_) {
      (*weights)[2] = std::max((*weights)[2], kLowStartUpTargetAcc);
    } else if (century::planning::Frame::AccLevel::ACC_ONE_LEVEL ==
               frame_->acc_level_) {
      (*weights)[2] = std::max((*weights)[2], kWeightOneLevelAcc);
    } else if (century::planning::Frame::AccLevel::ACC_TWO_LEVEL ==
               frame_->acc_level_) {
      (*weights)[2] = std::max((*weights)[2], kWeightTwoLevelAcc);
    } else if (century::planning::Frame::AccLevel::ACC_THREE_LEVEL ==
               frame_->acc_level_) {
      (*weights)[2] = std::max((*weights)[2], kWeightThreeLevelAcc);
    } else {
      (*weights)[2] = std::max((*weights)[2], kWeightFourLevelAcc);
    }
    AINFO << "acc_weight = " << (*weights)[2];
  }

  ADEBUG << "acc_weight = " << (*weights)[2];
  ADEBUG << "jerk_weight = " << (*weights)[3];
  ADEBUG << "ref_s_weight = " << (*weights)[0];
  ADEBUG << "ref_v_weight = " << (*weights)[1];
}

bool PiecewiseJerkSpeedOptimizer::IsNeedSlowerAccelerationByDkappa(
    const DiscretizedPath& path_points) {
  if (path_points.size() < kFrontPathPointNum) {
    return false;
  }
  double max_abs_kappa_of_front_path = 0.0;
  double max_abs_dkappa_of_front_path = 0.0;
  for (size_t i = 1; i < kFrontPathPointNum; ++i) {
    double abs_kappa = std::abs(path_points[i].kappa());
    double abs_dkappa = std::abs(path_points[i].dkappa());
    if (abs_kappa > max_abs_kappa_of_front_path) {
      max_abs_kappa_of_front_path = abs_kappa;
    }
    if (abs_dkappa > max_abs_dkappa_of_front_path) {
      max_abs_dkappa_of_front_path = abs_dkappa;
    }
  }
  ADEBUG << "max_abs_kappa_of_front_path = " << max_abs_kappa_of_front_path;
  ADEBUG << "max_abs_dkappa_of_front_path = " << max_abs_dkappa_of_front_path;
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  return max_abs_kappa_of_front_path >
             config.kappa_bound_for_slower_acceleration() ||
         max_abs_dkappa_of_front_path >
             config.dkappa_bound_for_slower_acceleration();
}

Status PiecewiseJerkSpeedOptimizer::SetProblemStatus(
    const std::array<double, 4U>& weights, const PathData& path_data,
    const std::array<double, 3U>& init_s, const SpeedLimit& speed_limit,
    const bool use_ref_v, const double total_length,
    SpeedData* const speed_data,
    PiecewiseJerkSpeedProblem* piecewise_jerk_problem) {
  piecewise_jerk_problem->set_weight_ddx(weights[2]);
  piecewise_jerk_problem->set_weight_dddx(weights[3]);
  // piecewise_jerk_problem->set_weight_ddx(config.acc_weight());
  // piecewise_jerk_problem->set_weight_dddx(config.jerk_weight());

  piecewise_jerk_problem->set_x_bounds(0.0, total_length);
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  piecewise_jerk_problem->set_dx_bounds(
      0.0, std::fmax(FLAGS_planning_upper_speed_limit, init_s[1]));
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  double max_acc = veh_param.max_acceleration();
  if (FLAGS_enable_longitudinal_accel_and_jerk_constraint &&
      IsNeedSlowerAccelerationByDkappa(path_data.discretized_path())) {
    piecewise_jerk_problem->set_ddx_bounds(
        veh_param.max_deceleration(),
        config.max_acceleration_for_slowly_start());
    piecewise_jerk_problem->set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                           config.max_jerk_for_slowly_start());
    max_acc = config.max_acceleration_for_slowly_start();
  } else {
    piecewise_jerk_problem->set_ddx_bounds(veh_param.max_deceleration(),
                                           veh_param.max_acceleration());
    piecewise_jerk_problem->set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                           FLAGS_longitudinal_jerk_upper_bound);
  }
  // AINFO << "target_a_ = " << target_a_;
  if (init_s[1] < config_.piecewise_jerk_speed_optimizer_config()
                      .low_start_up_speed() &&
      FLAGS_enable_use_svm_model &&
      !util::IsOvertake(injector_->planning_context()) &&
      !util::IsLaneChange(injector_->planning_context())) {
    double target_acc = SetTargetAcc();
    if (reference_line_info_->IsAdcInCommonJunction()) {
      // AINFO << "IN CONMONJUNCTION SET TARGET ACC";
      target_acc = SetTargetAccAtIntersection();
      // AINFO << "target_acc = " << target_acc;
    }

    // AINFO << " target_acc before  = " << target_acc;
    target_acc = std::max(std::min(max_acc, target_acc), init_s[2]);
    // AINFO << " target_acc after  = " << target_acc;
    piecewise_jerk_problem->set_ddx_bounds(veh_param.max_deceleration(),
                                           target_acc);
    piecewise_jerk_problem->set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                           config.max_jerk_for_slowly_start());
  }
  if (use_ref_v) {
    std::vector<double> ref_v;
    for (size_t i = 0; i < num_of_knots_; ++i) {
      double curr_t = i * kDeltaT;
      SpeedPoint sp;
      speed_data->EvaluateByTime(curr_t, &sp);
      const double path_s = sp.s();
      // get v_upper_bound
      double v_upper_bound = FLAGS_planning_upper_speed_limit;
      v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
      ref_v.emplace_back(v_upper_bound);
    }
    piecewise_jerk_problem->set_dx_ref(weights[1], ref_v);
  } else {
    piecewise_jerk_problem->set_dx_ref(weights[1],
                                       reference_line_info_->GetCruiseSpeed());
  }
  // Update STBoundary
  std::vector<std::pair<double, double>> s_bounds;

  if (!GetSTBoundary(&s_bounds)) {
    speed_data->clear();
    const std::string msg =
        "s_lower_bound larger than s_upper_bound on STGraph";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  ReInitStartPointByAccel(s_bounds, init_s[2]);

  piecewise_jerk_problem->set_x_bounds(std::move(s_bounds));
  piecewise_jerk_problem->set_scale_factor({1.0, 10.0, 100.0});
  // Update SpeedBoundary and ref_s
  std::vector<double> x_ref;
  std::vector<double> penalty_dx;
  std::vector<std::pair<double, double>> s_dot_bounds;
  GetSpeedBoundaryAndRefS(path_data, *speed_data, init_s, &s_dot_bounds,
                          &penalty_dx, &x_ref);

  // GetSpeedBoundaryAndRefS(path_data, *speed_data, &s_dot_bounds, &penalty_dx,
  //                         &x_ref, &min_v_upper_bound);

  piecewise_jerk_problem->set_x_ref(weights[0], std::move(x_ref));
  piecewise_jerk_problem->set_penalty_dx(penalty_dx);
  piecewise_jerk_problem->set_dx_bounds(std::move(s_dot_bounds));
  return Status::OK();
}

double PiecewiseJerkSpeedOptimizer::SetTargetAccAtIntersection() {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double target_acc = veh_param.max_acceleration();
  switch (frame_->acc_level_) {
    case century::planning::Frame::AccLevel::ACC_ONE_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_one_level_acc_at_intersection();
      break;
    case century::planning::Frame::AccLevel::ACC_TWO_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_two_level_acc_at_intersection();
      break;
    case century::planning::Frame::AccLevel::ACC_THREE_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_three_level_acc_at_intersection();
      break;
    case century::planning::Frame::AccLevel::ACC_FOUR_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_four_level_acc_at_intersection();
      break;
    default:
      target_acc = veh_param.max_acceleration();
      break;
  }
  return target_acc;
}

double PiecewiseJerkSpeedOptimizer::SetTargetAcc() {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double target_acc = veh_param.max_acceleration();
  switch (frame_->acc_level_) {
    case century::planning::Frame::AccLevel::ACC_ONE_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_one_level_acc();
      break;
    case century::planning::Frame::AccLevel::ACC_TWO_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_two_level_acc();
      break;
    case century::planning::Frame::AccLevel::ACC_THREE_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_three_level_acc();
      break;
    case century::planning::Frame::AccLevel::ACC_FOUR_LEVEL:
      target_acc = config_.piecewise_jerk_speed_optimizer_config()
                       .low_start_up_four_level_acc();
      break;
    default:
      target_acc = veh_param.max_acceleration();
      break;
  }
  return target_acc;
}

bool PiecewiseJerkSpeedOptimizer::UpdateDecel(double* min_stop_decel) {
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  double adc_speed = reference_line_info_->vehicle_state().linear_velocity();
  // AINFO << "adc_speed = " << adc_speed;
  bool is_need_update = false;
  for (size_t i = 0; i < discretized_path.size(); ++i) {
    const auto& frenet_point = frenet_path[i];
    const auto& reference_point =
        reference_line_info_->reference_line().GetReferencePoint(
            frenet_point.s());
    double ref_point_kappa = reference_point.kappa();
    // AINFO << " frenet_point.s() = " << frenet_point.s();
    // AINFO << "ref_point_kappa = " << ref_point_kappa;
    double speed_limit_from_centripetal_acc =
        std::sqrt(FLAGS_max_centric_acceleration_limit /
                  std::fmax(std::fabs(ref_point_kappa), kEpison));
    // AINFO << "speed_limit_from_centripetal_acc = "
    //       << speed_limit_from_centripetal_acc;
    if (adc_speed > speed_limit_from_centripetal_acc) {
      double delta_distance = std::fabs(
          frenet_point.s() - reference_line_info_->AdcSlBoundary().end_s());
      // AINFO << "delta_distance = " << delta_distance;
      double delta_v =
          (speed_limit_from_centripetal_acc * speed_limit_from_centripetal_acc -
           adc_speed * adc_speed);
      double target_decel = std::numeric_limits<double>::max();
      if (delta_distance > 0.0 && delta_v < 0.0) {
        target_decel = delta_v * 0.5 / delta_distance;
        // AINFO << "target_decel = " << target_decel;
        if (target_decel < *min_stop_decel) {
          *min_stop_decel = std::max(target_decel, kMinAccForSlowerDecel);
          is_need_update = true;
        }
      }
    }
  }
  return is_need_update;
}

bool PiecewiseJerkSpeedOptimizer::ExportSlowerBrakeSpeed(
    const std::array<double, 3U>& init_s, SpeedData* const speed_data) {
  double min_stop_decel = kMinDecel;
  double min_stop_s = 0.0;
  bool is_need_slow_breaking =
      reference_line_info_->IsNeedSlowBreaking(&min_stop_decel, &min_stop_s);
  if (min_stop_decel > kEpison) {
    min_stop_decel = kMinDecel;
  }
  // AINFO << "is_need_slow_breaking = " << is_need_slow_breaking;
  // AINFO << "min_stop_decel = " << min_stop_decel;
  if (is_need_slow_breaking) {
    const auto& config = config_.piecewise_jerk_speed_optimizer_config();
    // ADEBUG << "st_graph_data.init_point().v() = "
    //        << st_graph_data.init_point().v();
    // ADEBUG << "st_graph_data.init_point().a() = "
    //        << st_graph_data.init_point().a();
    // std::unique_ptr<KinematicDecelerationSpeedOptimizer>
    //     kinematic_deceleration_speed_opt = nullptr;
    // kinematic_deceleration_speed_opt.reset(
    //     new KinematicDecelerationSpeedOptimizer(
    //         static_cast<size_t>(num_of_knots_), kDeltaT, config));
    // kinematic_deceleration_speed_opt->Init(
    //     st_graph_data.init_point().v(), st_graph_data.init_point().a(),
    //     KinematicDecelerationSpeedOptimizer::RESTRICT_UPPER_LIMIT);
    // speed_data->clear();
    // double stop_s =
    //     kinematic_deceleration_speed_opt->GenerateSpeedDataByConstantJerk(
    //         min_stop_jerk, speed_data);
    // AINFO << "min_stop_s = " << min_stop_s;
    // ADEBUG << "stop_s = " << stop_s;

    bool is_need_update_decel_for_kappa = UpdateDecel(&min_stop_decel);
    ADEBUG << "is_need_update_decel_for_kappa = "
           << is_need_update_decel_for_kappa;

    KinematicDecelerationSpeedOptimizer decel_speed_optimizer(
        static_cast<size_t>(num_of_knots_), kDeltaT, config);

    if (injector_->GetSequenceNum() -
            injector_->GetLatestSeqNumForSlowerDecel() >
        1U) {
      const double adc_speed =
          std::max(0.0, injector_->vehicle_state()->linear_velocity());
      const double adc_acc = injector_->vehicle_state()->linear_acceleration();
      decel_speed_optimizer.Init(
          adc_speed, adc_acc,
          KinematicDecelerationSpeedOptimizer::RESTRICT_LOWER_AND_UPPER_LIMIT);
    } else {
      decel_speed_optimizer.Init(init_s[1], init_s[2]);
    }
    decel_speed_optimizer.GenerateSpeedDataByConstantAccel(min_stop_decel,
                                                           speed_data);
    injector_->SetLatestSeqNumForSlowerDecel();
    double stop_s = speed_data->back().s();
    // AINFO << "speed_data.back = " << stop_s;

    // second boundary wall
    if (min_stop_s < stop_s) {
      // AINFO << "use second wall";
      if (KinematicSpeedOptimize(min_stop_s, init_s, speed_data)) {
        return true;
      } else {
        // AINFO << "use  qp ";
        return false;
      }
    }
    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    return true;
  }
  return false;
}

bool PiecewiseJerkSpeedOptimizer::MakeSlowerBrakeAfterQPFailureByPassableArea(
    const std::array<double, 3U>& init_s, SpeedData* const speed_data) {
  const double adc_speed =
      std::max(0.0, injector_->vehicle_state()->linear_velocity());
  const double adc_acc = injector_->vehicle_state()->linear_acceleration();
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  const double max_presight_time =
      std::min(config.max_presight_time_for_passable_area(),
               std::sqrt(2.0 * adc_speed /
                         -(config.max_jerk_for_decel() +
                           config.jerk_buffer_for_presight_time())));
  std::array<double, 2U> accel_bounds{std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max()};
  const double accel_lower_bound = config.accel_for_slower_decel() -
                                   config.accel_low_buffer_for_slower_decel();
  const double accel_upper_bound = config.accel_for_slower_decel() +
                                   config.accel_up_buffer_for_slower_decel();
  const auto has_passable_area =
      HasPassableArea(accel_lower_bound, accel_upper_bound, max_presight_time,
                      false, &accel_bounds);

  bool need_slower_deceleration = false;
  double deceleration_value = 0.0;
  switch (has_passable_area) {
    case PASSABLE_AREA:
      need_slower_deceleration = true;
      deceleration_value = config.accel_for_slower_decel();
      break;
    case BELOW_PASSABLE_AREA:
    case BOTH_SIDES_PASSABLE_AREA: {
      const auto accel_value =
          accel_bounds[0] - config.accel_buffer_for_passable_area();
      if (accel_value > std::max(kMinAccForSlowerDecel, accel_lower_bound)) {
        need_slower_deceleration = true;
        deceleration_value = accel_value;
      }
      break;
    }
    case ABOVE_PASSABLE_AREA:
    case MIDDLE_PASSABLE_AREA: {
      const auto accel_value1 =
          accel_bounds[0] + config.accel_buffer_for_passable_area();
      const auto accel_value2 =
          accel_bounds[1] - config.accel_buffer_for_passable_area();
      if (accel_value1 < std::min(kMaxAccForSlowerDecel, accel_upper_bound) &&
          accel_value2 > accel_value1) {
        need_slower_deceleration = true;
        deceleration_value = accel_value1;
      }
      break;
    }
    default:
      need_slower_deceleration = false;
      break;
  }
  if (need_slower_deceleration) {
    KinematicDecelerationSpeedOptimizer decel_speed_optimizer(
        static_cast<size_t>(num_of_knots_), kDeltaT, config);
    if (injector_->GetSequenceNum() -
            injector_->GetLatestSeqNumForSlowerDecel() >
        1U) {
      decel_speed_optimizer.Init(
          adc_speed, adc_acc,
          KinematicDecelerationSpeedOptimizer::RESTRICT_TO_SPECIFIED_VALUE);
    } else {
      decel_speed_optimizer.Init(init_s[1], init_s[2]);
    }
    decel_speed_optimizer.GenerateSpeedDataByConstantAccel(deceleration_value,
                                                           speed_data);
    injector_->SetLatestSeqNumForSlowerDecel();
    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    return true;
  }
  return false;
}

bool PiecewiseJerkSpeedOptimizer::MakeSlowerBrakeByParamAndPassableArea(
    const std::array<double, 3U>& init_s, SpeedData* const speed_data) {
  double target_speed = 0.0, target_accel = 0.0;
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (reference_line_info_->GetSlowerBreakingParam(&target_speed,
                                                   &target_accel) &&
      (PASSABLE_AREA ==
       HasPassableArea(veh_param.max_deceleration(), 0.0,
                       config.max_presight_time_for_passable_area(), true))) {
    ADEBUG << "slower breaking to target_speed(" << target_speed
           << ") with target_accel(" << target_accel << ")";
    KinematicDecelerationSpeedOptimizer decel_speed_optimizer(
        static_cast<size_t>(num_of_knots_), kDeltaT, config);
    if (injector_->GetSequenceNum() -
            injector_->GetLatestSeqNumForTargetSpeedSlowerDecel() >
        1U) {
      const double adc_speed =
          std::max(0.0, injector_->vehicle_state()->linear_velocity());
      const double adc_acc = injector_->vehicle_state()->linear_acceleration();
      decel_speed_optimizer.Init(
          adc_speed, adc_acc,
          KinematicDecelerationSpeedOptimizer::RESTRICT_TO_SPECIFIED_VALUE);
    } else {
      decel_speed_optimizer.Init(init_s[1], init_s[2]);
    }
    decel_speed_optimizer.GenerateSpeedDataByConstantAccelAndSpeed(
        target_accel, target_speed, speed_data);
    injector_->SetLatestSeqNumForTargetSpeedSlowerDecel();
    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    return true;
  }
  return false;
}

bool PiecewiseJerkSpeedOptimizer::PreProcessForSpeedProblem(
    const SpeedData& reference_speed_data, const std::array<double, 3U>& init_s,
    const SpeedLimit& speed_limit, std::array<double, 4U>* ptr_weights) {
  SpeedPoint sp_first;
  reference_speed_data.EvaluateByTime(0.0, &sp_first);
  double init_v_upper_bound = speed_limit.GetSpeedLimitByS(sp_first.s());
  bool use_ref_v = false;
  if (init_v_upper_bound * kSpeedCoeff < init_s[1]) {
    // no trust DP result, use reference_v
    use_ref_v = true;
  }
  std::array<double, 4U> weight_coeffs{1.0, 1.0, 1.0, 1.0};
  double target_a = GetTargetAcc(reference_speed_data, init_s[1]);
  target_a_ = target_a;
  GetWightCoeffs(target_a, use_ref_v, &weight_coeffs);
  GetWights(target_a, weight_coeffs, init_s, ptr_weights);
  return use_ref_v;
}

Status PiecewiseJerkSpeedOptimizer::CheckValidityOfSpeedData(
    SpeedData* const speed_data) {
  for (const auto& sp : *speed_data) {
    if (std::isnan(sp.s()) || std::isinf(sp.s()) || std::isnan(sp.v()) ||
        std::isinf(sp.v()) || std::isnan(sp.a()) || std::isinf(sp.a())) {
      AERROR << "Invalid speed data: t[" << sp.t() << "], s = " << sp.s()
             << ", v = " << sp.v() << ", a = " << sp.a();
      speed_data->clear();
      return Status(ErrorCode::PLANNING_ERROR, "Invalid speed data");
    }
  }
  return Status::OK();
}

void PiecewiseJerkSpeedOptimizer::KinematicSlowerBreakingForApproachObs(
    double target_speed, double target_accel,
    const std::array<double, 3U>& init_s, SpeedData* const speed_data) {
  ADEBUG << "using kinematic slowly breaking for approach obs";
  KinematicDecelerationSpeedOptimizer decel_speed_optimizer(
      static_cast<size_t>(num_of_knots_), kDeltaT,
      config_.piecewise_jerk_speed_optimizer_config());
  if (injector_->GetSequenceNum() -
          injector_->GetLatestSeqNumForApproachObsSlowerDecel() >
      1U) {
    const double adc_speed =
        std::max(0.0, injector_->vehicle_state()->linear_velocity());
    const double adc_acc = injector_->vehicle_state()->linear_acceleration();
    decel_speed_optimizer.Init(
        adc_speed, adc_acc,
        KinematicDecelerationSpeedOptimizer::RESTRICT_TO_SPECIFIED_VALUE);
  } else {
    decel_speed_optimizer.Init(init_s[1], init_s[2]);
  }
  SpeedData slower_decel_speed_data;
  decel_speed_optimizer.GenerateSpeedDataByConstantAccelAndSpeed(
      target_accel, target_speed, &slower_decel_speed_data);
  injector_->SetLatestSeqNumForApproachObsSlowerDecel();
  if (slower_decel_speed_data[10].v() < (*speed_data)[10].v()) {
    ADEBUG << "select kinematic slowly breaking for approach obs";
    for (auto sp : *speed_data) {
      ADEBUG << "QP For t[" << sp.t() << "], s = " << sp.s()
             << ", v = " << sp.v() << ", a = " << sp.a();
    }
    speed_data->clear();
    *speed_data = std::move(slower_decel_speed_data);
    for (auto sp : *speed_data) {
      ADEBUG << "KinematicSpeed For t[" << sp.t() << "], s = " << sp.s()
             << ", v = " << sp.v() << ", a = " << sp.a();
    }
  }
}

Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {
    return CheckValidityOfSpeedData(speed_data);
  }

  ACHECK(speed_data != nullptr);
  SpeedData reference_speed_data = *speed_data;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  num_of_knots_ =
      static_cast<size_t>(st_graph_data.total_time_by_conf() / kDeltaT) + 1;

  const auto& config = config_.piecewise_jerk_speed_optimizer_config();

  std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                  st_graph_data.init_point().a()};
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (injector_->GetReplanState() &&
      init_s[2] < config.decel_buffer_of_init_point() &&
      PASSABLE_AREA ==
          HasPassableArea(veh_param.max_deceleration(),
                          veh_param.max_acceleration(),
                          config.max_presight_time_for_passable_area(), true)) {
    ADEBUG << "Modify accel, init acceleration" << init_s[2];
    init_s[2] = config.decel_buffer_of_init_point();
  }

  if (ExportSlowerBrakeSpeed(init_s, speed_data)) {
    return CheckValidityOfSpeedData(speed_data);
  }

  if (FLAGS_enable_kinematic_speed_optimizer &&
      KinematicSpeedOptimize(0.0, init_s, speed_data)) {
    return CheckValidityOfSpeedData(speed_data);
  }

  if (MakeSlowerBrakeByParamAndPassableArea(init_s, speed_data)) {
    return CheckValidityOfSpeedData(speed_data);
  }

  // double acc_weight = config.acc_weight();
  // double jerk_weight = config.jerk_weight();
  // double ref_s_weight = config.ref_s_weight();
  // double ref_v_weight = config.ref_v_weight();
  std::array<double, 4U> weights{config.ref_s_weight(), config.ref_v_weight(),
                                 config.acc_weight(), config.jerk_weight()};
  bool use_ref_v = PreProcessForSpeedProblem(
      reference_speed_data, init_s, st_graph_data.speed_limit(), &weights);

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, kDeltaT,
                                                   init_s);
  double total_length = st_graph_data.path_length();
  Status ret = SetProblemStatus(
      weights, path_data, init_s, st_graph_data.speed_limit(), use_ref_v,
      total_length, speed_data, &piecewise_jerk_problem);
  // piecewise_jerk_problem.InfoProblemParam();
  if (!ret.ok()) {
    return ret;
  }
  // Solve the problem
  ADEBUG << "SPEED OPTIMIZER : "
         << ", init_speed: " << init_s[1] << ", init_acc: " << init_s[2];
  ret = piecewise_jerk_problem.Optimize();
  if (ret.ok()) {
    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
    ExtractSpeedPoints(s, ds, dds, speed_data);
    double target_speed = 0.0, target_accel = 0.0;
    if (reference_line_info_->GetSlowlyBreakingParamForApproachObs(
            &target_speed, &target_accel)) {
      KinematicSlowerBreakingForApproachObs(target_speed, target_accel, init_s,
                                            speed_data);
    }
    SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  } else {
    if (FLAGS_enable_slower_deceleration_after_qp_failure &&
        MakeSlowerBrakeAfterQPFailureByPassableArea(init_s, speed_data)) {
      return CheckValidityOfSpeedData(speed_data);
    }
    const std::string msg = "Piecewise jerk speed optimizer failed!";
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
  return CheckValidityOfSpeedData(speed_data);
}

void PiecewiseJerkSpeedOptimizer::ExtractSpeedPoints(
    const std::vector<double>& s, const std::vector<double>& ds,
    const std::vector<double>& dds, SpeedData* const speed_data) {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  for (size_t i = 0; i < num_of_knots_; ++i) {
    ADEBUG << "For t[" << i * kDeltaT << "], s = " << s[i] << ", v = " << ds[i]
           << ", a = " << dds[i];
  }
  speed_data->clear();
  speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  bool is_need_to_start_up = false;
  for (size_t i = 1; i < num_of_knots_; ++i) {
    // Avoid the very last points when already stopped
    if (ds[i] >= config.start_up_min_v()) {
      is_need_to_start_up = true;
      break;
    }
    if (kDeltaT * i > config.look_up_relative_time()) {
      break;
    }
  }
  for (size_t i = 1; i < num_of_knots_; ++i) {
    // Avoid the very last points when already stopped
    double v = ds[i];
    if (ds[i] <= 0.0 && !is_need_to_start_up) {
      break;
    } else if (ds[i] <= 0.0 && is_need_to_start_up) {
      v = kMinVelocity;
    }

    speed_data->AppendSpeedPoint(s[i], kDeltaT * i, v, dds[i],
                                 (dds[i] - dds[i - 1]) / kDeltaT);
  }
}

void PiecewiseJerkSpeedOptimizer::DebugSpeedLimit(
    const std::vector<double>& x_ref,
    const std::vector<std::pair<double, double>>& s_dot_bounds) {
  SpeedData speed_data_limit;
  for (size_t i = 0; i < s_dot_bounds.size(); ++i) {
    speed_data_limit.AppendSpeedPoint(x_ref[i], i * kDeltaT,
                                      s_dot_bounds[i].second, 0.0, 0.0);
  }
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name("limit");
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data_limit.begin(), speed_data_limit.end()});
}

void PiecewiseJerkSpeedOptimizer::GetSpeedBoundaryAndRefS(
    const PathData& path_data, const SpeedData& reference_speed_data,
    const std::array<double, 3U>& init_s,
    std::vector<std::pair<double, double>>* s_dot_bounds,
    std::vector<double>* penalty_dx, std::vector<double>* x_ref) {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  double min_v_upper_bound = std::numeric_limits<double>::max();
  double last_path_s = 0.0;
  for (size_t i = 0; i < num_of_knots_; ++i) {
    double curr_t = i * kDeltaT;
    // get path_s
    SpeedPoint sp;
    reference_speed_data.EvaluateByTime(curr_t, &sp);
    const double path_s = std::max(sp.s(), last_path_s);
    last_path_s = path_s;
    x_ref->emplace_back(path_s);
    // get curvature
    PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
    penalty_dx->emplace_back(std::fabs(path_point.kappa()) *
                             config.kappa_penalty_weight());
    // get v_upper_bound
    const double v_lower_bound = 0.0;
    double v_upper_bound = FLAGS_planning_upper_speed_limit;
    v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
    min_v_upper_bound =
        min_v_upper_bound < v_upper_bound ? min_v_upper_bound : v_upper_bound;
    s_dot_bounds->emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
  }
  const double init_speed = init_s[1];
  const double init_acc = init_s[2];
  ADEBUG << "init_speed: " << init_speed << ", init_acc: " << init_acc;
  ADEBUG << "------speed_limit before modify------";
  size_t idx = 0UL;
  for (const auto& item : *s_dot_bounds) {
    ADEBUG << "before modify sp (" << idx * kDeltaT << "): " << item.second;
    ++idx;
  }
  ReconstructSpeedLimitByKinematic(min_v_upper_bound, init_speed, init_acc,
                                   s_dot_bounds);
  ADEBUG << "------speed_limit after modify------";
  idx = 0UL;
  for (const auto& item : *s_dot_bounds) {
    ADEBUG << "after modify sp (" << idx * kDeltaT << "): " << item.second;
    ++idx;
  }
  DebugSpeedLimit(*x_ref, *s_dot_bounds);
}

void PiecewiseJerkSpeedOptimizer::ReInitStartPointByAccel(
    const std::vector<std::pair<double, double>>& s_bounds,
    const double accel) {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double adc_acc = injector_->vehicle_state()->linear_acceleration();
  if (injector_->GetDiffSeqNumFromLastReinit() >
          config.min_sequence_num_interval_for_reinit() &&
      adc_speed < config.lower_speed_for_reinit() &&
      accel < config.decel_buffer_of_init_point() &&
      adc_acc - accel > config.diff_acc_buffer_for_reinit()) {
    bool can_accelerate = true;
    size_t presight_knots_num =
        static_cast<size_t>(config.presight_time_for_reinit() / kDeltaT + 1);
    for (size_t i = 0; i < presight_knots_num; ++i) {
      double curr_t = i * kDeltaT;
      if (s_bounds[i].second <
          curr_t * adc_speed +
              0.5 * config.accel_buffer_for_reinit() * curr_t * curr_t) {
        can_accelerate = false;
      }
    }
    if (can_accelerate) {
      ADEBUG << "init acceleration" << accel;
      injector_->SetReinitStartPoint();
    }
  }
}

bool PiecewiseJerkSpeedOptimizer::GetPassableStateAtAccel(
    const IndexedObstacles& indexed_obstacles, const double max_presight_time,
    const double s, const double curr_time, const double move_time,
    const bool need_check_after_stop) const {
  bool can_passable_at_accel = true;
  for (const auto* ptr_obs : indexed_obstacles.Items()) {
    if (!ptr_obs->is_path_st_boundary_initialized()) {
      continue;
    }
    const auto& st_boundary = ptr_obs->path_st_boundary();
    const double min_t = st_boundary.min_t();
    const double max_t = std::min(st_boundary.max_t(), max_presight_time);
    const double diff_heading =
        std::abs(common::math::NormalizeAngle(
            ptr_obs->SpeedHeading() - injector_->vehicle_state()->heading())) *
        kDegrees / M_PI_2;
    const double hy_diff_heading = adc_obstacles_heading_interval.HyValue(
        std::to_string(ptr_obs->PerceptionId()), diff_heading);
    bool is_same_direction_bicycle =
        perception::PerceptionObstacle::BICYCLE ==
            ptr_obs->Perception().type() &&
        hy_diff_heading < FLAGS_max_diff_angle_for_stoped_collision_check;
    bool is_need_consider = !ptr_obs->IsCross() && !ptr_obs->IsReverse();
    // AINFO << "is_need_consider = " << is_need_consider;
    if (!need_check_after_stop &&
        (is_same_direction_bicycle || is_need_consider)) {
      if (curr_time > max_t || move_time < min_t) {
        continue;
      }
    } else {
      if (curr_time > max_t || curr_time < min_t) {
        continue;
      }
    }

    double s_upper = 0.0, s_lower = 0.0;
    if (st_boundary.GetBoundarySRange(curr_time, &s_upper, &s_lower)) {
      if (s > s_lower && s < s_upper) {
        can_passable_at_accel = false;
        break;
      }
    }
  }
  return can_passable_at_accel;
}

PiecewiseJerkSpeedOptimizer::PassableAreaType
PiecewiseJerkSpeedOptimizer::HasPassableArea(
    const double min_accel, const double max_accel,
    const double max_presight_time, const bool need_check_after_stop,
    std::array<double, 2U>* accel_bounds) const {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  const double adc_speed =
      std::min(FLAGS_planning_upper_speed_limit,
               std::max(0.0, injector_->vehicle_state()->linear_velocity()));
  const auto& indexed_obstacles =
      reference_line_info_->path_decision()->obstacles();
  std::vector<std::pair<bool, double>> passable_states_all_accel;
  passable_states_all_accel.clear();
  for (double accel = min_accel; accel <= max_accel;
       accel += config.accel_step_for_passable_area()) {
    bool can_passable_at_accel = true;
    for (double curr_t = 0.0; curr_t <= max_presight_time; curr_t += kDeltaT) {
      double accel_t = curr_t, constant_t = 0.0, constant_v = 0.0;
      if (accel < 0.0) {
        accel_t = std::min(curr_t, adc_speed / -accel);
        constant_t = 0.0;
        constant_v = 0.0;
      } else if (accel > 0.0) {
        const double reach_speed_limit_t =
            (FLAGS_planning_upper_speed_limit - adc_speed) / accel;
        accel_t = std::min(curr_t, reach_speed_limit_t);
        constant_t = std::max(0.0, curr_t - reach_speed_limit_t);
        constant_v = FLAGS_planning_upper_speed_limit;
      }
      const double s = adc_speed * accel_t + 0.5 * accel * accel_t * accel_t +
                       constant_v * constant_t;
      double move_time = curr_t;
      move_time = accel_t + constant_t;
      ADEBUG << "curr_t: " << curr_t << ", move time: " << move_time;
      ADEBUG << "accel: " << accel << "s: " << s << ", accel_t: " << accel_t
             << ", constant_t: " << constant_t
             << ", constant_v: " << constant_v;
      can_passable_at_accel =
          GetPassableStateAtAccel(indexed_obstacles, max_presight_time, s,
                                  curr_t, move_time, need_check_after_stop);
      if (!can_passable_at_accel) {
        break;
      }
    }
    if (passable_states_all_accel.empty()) {
      passable_states_all_accel.emplace_back(can_passable_at_accel, accel);
    } else if (can_passable_at_accel !=
               passable_states_all_accel.back().first) {
      if (can_passable_at_accel) {
        passable_states_all_accel.emplace_back(can_passable_at_accel, accel);
      } else {
        passable_states_all_accel.emplace_back(
            can_passable_at_accel,
            accel - config.accel_step_for_passable_area());
      }
    }
    if (3UL == passable_states_all_accel.size()) {
      break;
    }
  }
  PassableAreaType has_passable_area =
      GetPassableState(passable_states_all_accel, accel_bounds);

  ADEBUG << "has_passable_area: " << has_passable_area;
  if (nullptr != accel_bounds) {
    ADEBUG << "accel_bounds: " << (*accel_bounds)[0] << ", "
           << (*accel_bounds)[1];
  }
  return has_passable_area;
}

PiecewiseJerkSpeedOptimizer::PassableAreaType
PiecewiseJerkSpeedOptimizer::GetPassableState(
    const std::vector<std::pair<bool, double>>& passable_states_all_accel,
    std::array<double, 2U>* accel_bounds) const {
  PassableAreaType has_passable_area = PASSABLE_AREA;
  switch (passable_states_all_accel.size()) {
    case 1U:
      if (passable_states_all_accel.back().first) {
        has_passable_area = PASSABLE_AREA;
      } else {
        has_passable_area = NONE_PASSABLE_AREA;
      }
      break;

    case 2U:
      ADEBUG << "passable_states_all_accel 2 = ("
             << passable_states_all_accel[0].second << ") , ("
             << passable_states_all_accel[1].second << ")";
      if (passable_states_all_accel.back().first) {
        has_passable_area = ABOVE_PASSABLE_AREA;
      } else {
        has_passable_area = BELOW_PASSABLE_AREA;
      }
      if (nullptr != accel_bounds) {
        (*accel_bounds)[0] = passable_states_all_accel.back().second;
      }
      break;

    case 3U:
      ADEBUG << "passable_states_all_accel 3 = ("
             << passable_states_all_accel[0].second << ") , ("
             << passable_states_all_accel[1].second << ") , ("
             << passable_states_all_accel[2].second << ")";
      if (passable_states_all_accel.back().first) {
        has_passable_area = BOTH_SIDES_PASSABLE_AREA;
      } else {
        has_passable_area = MIDDLE_PASSABLE_AREA;
      }
      if (nullptr != accel_bounds) {
        (*accel_bounds)[0] = passable_states_all_accel[1].second;
        (*accel_bounds)[1] = passable_states_all_accel[2].second;
      }
      break;
    default:
      break;
  }
  return has_passable_area;
}

bool PiecewiseJerkSpeedOptimizer::GetSTBoundary(
    std::vector<std::pair<double, double>>* ptr_s_bounds) {
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  double adc_speed = reference_line_info_->vehicle_state().linear_velocity();

  double init_a = st_graph_data.init_point().a();

  bool is_need_start_up_check = false;
  if (!reference_line_info_->IsChangeLanePath()) {
    bool is_stop = std::fabs(adc_speed) < kMinSpeed;
    // continue 5 frame to start up
    if (((init_a > kMinAcc) || (target_a_ > kMinAcc)) && is_stop) {
      injector_->start_up_safe_check_count_++;
      is_need_start_up_check = true;
      start_up_count_ = 0;
    } else {
      if (injector_->start_up_safe_check_count_ < kMaxStartUpCount &&
          injector_->start_up_safe_check_count_ > 0) {
        is_need_start_up_check = true;
      }
    }
    // adc start up already
    if (std::fabs(adc_speed) > kSartUpSpeed) {
      start_up_count_++;
    } else {
      start_up_count_ = 0;
    }
    if(start_up_count_ > kMaxStartUpCount){
      injector_->start_up_safe_check_count_ = 0;
      injector_->start_up_safe_check_s_ = 0.0;
    }
  }
  if (FLAGS_enable_check_start_up_safe) {
    // AINFO << "adc_speed = " << adc_speed;
    // AINFO << "target_a_ = " << target_a_;
    // AINFO << "init a = " << init_a;
    // AINFO << "is_need_start_up_check = " << is_need_start_up_check;
    // AINFO << "injector_->start_up_safe_check_count_ = "
    //       << injector_->start_up_safe_check_count_;
  }


    double close_key_point_s = std::numeric_limits<double>::max();
    const Obstacle* close_virtual_obs = GetCloseObstacle(&close_key_point_s);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    double curr_t = i * kDeltaT;
    double s_lower_bound = 0.0;
    double s_upper_bound = st_graph_data.path_length();
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;
      }
      switch (boundary->boundary_type()) {
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::FOLLOW:
          // TODO(Hongyi): unify follow buffer on decision side
          {
            s_upper_bound = std::fmin(s_upper_bound, s_upper);
            break;
          }
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    if (s_lower_bound > s_upper_bound) {
      return false;
    }
    if (FLAGS_enable_check_start_up_safe && is_need_start_up_check&&nullptr == close_virtual_obs) {
      if (injector_->start_up_safe_check_count_ < 5) {
        // AINFO << "no start up";
        s_lower_bound = 0.0;
        s_upper_bound = 0.0;
      } else {
        // AINFO << "need to start up";
      }
    }
    ptr_s_bounds->emplace_back(s_lower_bound, s_upper_bound);
  }

  if (FLAGS_enable_check_start_up_safe) {
    // double close_key_point_s = std::numeric_limits<double>::max();
    // const Obstacle* close_virtual_obs = GetCloseObstacle(&close_key_point_s);
    // AINFO << "close_key_point_s = " << close_key_point_s;
    if (FLAGS_enable_check_start_up_safe && nullptr == close_virtual_obs) {
      double min_bound = std::numeric_limits<double>::max();
      double max_bound = std::numeric_limits<double>::lowest();
      for (const auto& bound_pair : *ptr_s_bounds) {
        double s_upper = bound_pair.second;
        if (s_upper < min_bound) {
          min_bound = s_upper;
        }
        if (s_upper > max_bound) {
          max_bound = s_upper;
        }
      }
      if (injector_->start_up_safe_check_count_ == 1) {
        // AINFO << " set first stop s";
        injector_->start_up_safe_check_s_ = min_bound;
      }
      // AINFO << "injector_->start_up_safe_check_s_  = "
      //       << injector_->start_up_safe_check_s_;
      double tolerance = 0.1;
      double delta = max_bound - min_bound;
      bool boundary_is_almost_static = (delta <= tolerance);
      // AINFO << "boundary_is_almost_static = " << boundary_is_almost_static;
      if (boundary_is_almost_static) {
        // AINFO << "min_bound = " << min_bound;
        if (min_bound - injector_->start_up_safe_check_s_ <
                FLAGS_check_start_up_distance &&
            injector_->start_up_safe_check_count_ >= 5) {
          // AINFO << "keep stop";
          for (auto& bound_pair : *ptr_s_bounds) {
            bound_pair.second = injector_->start_up_safe_check_s_;
          }
        } else {
          // AINFO << "NO STARTUP OR LARGE 3M;";
        }
      }
    }
  }

  return true;
}

void PiecewiseJerkSpeedOptimizer::ReconstructSpeedLimitByKinematic(
    const double min_v_upper_bound, double init_speed, double init_acc,
    std::vector<std::pair<double, double>>* s_dot_bounds) {
  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  const double max_dec =
      config.reduction_ratio() * common::VehicleConfigHelper::GetConfig()
                                     .vehicle_param()
                                     .max_deceleration();
  const double max_jerk_lower_bound =
      config.reduction_ratio() * FLAGS_longitudinal_jerk_lower_bound;
  init_speed += config.speed_buffer();
  init_acc += config.deceleration_buffer();
  const double t_max_v_by_jerk = -init_acc / max_jerk_lower_bound;
  // setting first speed limit as the init_speed, when init_speed is bigger
  // than first speed limit
  if ((*s_dot_bounds)[0UL].second < init_speed) {
    (*s_dot_bounds)[0UL].second = init_speed;
  }
  // adjust the speed limit value according the vehicle kinematic
  for (size_t i = 1UL; i < (*s_dot_bounds).size(); ++i) {
    double& curr_v = (*s_dot_bounds)[i].second;
    double curr_t = i * kDeltaT;
    double min_v_by_acc = init_speed + max_dec * curr_t;
    double min_v_by_jerk = init_speed + init_acc * curr_t +
                           0.5 * max_jerk_lower_bound * curr_t * curr_t;
    if (min_v_by_acc <= min_v_upper_bound &&
        (curr_t >= t_max_v_by_jerk && min_v_by_jerk <= min_v_upper_bound)) {
      // Subsequent min_v_by_acc and min_v_by_jerk are no longer greater than
      // min_v_upper_bound, so there is no need to adjust subsequent speed limit
      // value.
      break;
    }
    curr_v = std::max(curr_v, std::max(min_v_by_acc, min_v_by_jerk));
    curr_v = std::max(curr_v, 0.0);
  }
}

}  // namespace planning
}  // namespace century
