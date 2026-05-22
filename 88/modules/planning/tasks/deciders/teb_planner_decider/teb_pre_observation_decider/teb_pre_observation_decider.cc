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
 * @file teb_pre_observation_decider.cc
 **/

#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_pre_observation_decider.h"

#include <memory>
#include <utility>

#include "cyber/time/clock.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/math/discrete_points_math.h"

namespace century {
namespace planning {
namespace {
using century::common::ErrorCode;
using century::common::SLPoint;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
// constexpr double kStopSpeed = 0.05;
constexpr double kFallBackStopTime = 2.0;
constexpr double kEpsilon = 1e-6;
constexpr double kIgnoreTime = 100;
}  // namespace

TEBPreObservationDecider::TEBPreObservationDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
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
  tar_decider_fsm_ = std::make_shared<TEBTarDeciderFsm>();
  tar_decider_fsm_->InitTarFsm(config, injector);
  prp_reverse_ = std::make_shared<TarVehicleInfo>();
  tar_reverse_ = std::make_shared<TarVehicleInfo>();
  if (injector->is_need_to_keep_right_) {
    AINFO << "add prp tar info success.";
    prp_reverse_->SetValid(true);
    prp_reverse_->SetId(injector->reverse_obj_.first);
    prp_reverse_->SetPerId(injector->reverse_obj_.second);
  }
  stop_time_ = 0;
}

Status TEBPreObservationDecider::Process(Frame* frame) {
  // 0, Sanity checks.
  if (!config_.teb_pre_observation_decider_config().enabled_switch()) {
    AINFO << "the switch of teb pre observation decider is closed.";
    return Status::OK();
  }
  if (nullptr == frame) {
    const std::string msg =
        "Invalid frame, fail to process the TEBPreObservationDecider.";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  frame->mutable_open_space_info()->set_tar_status(TEBTarStatus::NORMAL);
  const double interested_time =
      config_.teb_pre_observation_decider_config().tar_interested_duration();
  AINFO << interested_time;
  vehicle_state_ = frame->vehicle_state();
  obstacles_by_frame_ = frame->GetObstacleList();
  const auto adc_speed = vehicle_state_.linear_velocity();
  stop_time_ = std::fabs(adc_speed) < kStopSpeed ? stop_time_ + 1 : 0;
  stop_time_ = std::min(kMaxStopTime, stop_time_);
  AINFO << "adc stop time cycle: " << stop_time_;
  frame->mutable_open_space_info()->set_stop_time(stop_time_);
  const auto* previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool last_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  tar_reverse_->SetLastPlanSuccess(last_plan_success);
  // 1, adjust oncoming dynamic obstacle
  OncomingTarDecider(frame);
  AINFO << "tar id:" << tar_reverse_->Id();

  // 2, update infomation.
  if (!config_.teb_pre_observation_decider_config().enabled_fsm_switch()) {
    // provided a simple method with no fsm.
    SimpleHandleTar();
    return Status::OK();
  }
  // 3, update the state machine.
  if (config_.teb_pre_observation_decider_config().enabled_rude_switch()) {
    tar_decider_fsm_->ExecuteFsmRude(tar_reverse_, frame);
  } else {
    tar_decider_fsm_->ExecuteFsm(tar_reverse_, frame);
  }

  AINFO << "tar status: "
        << static_cast<int>(frame->open_space_info().tar_status());
  return Status::OK();
}

void TEBPreObservationDecider::OncomingTarDecider(Frame* const frame) {
  // 1 prp check tar info.
  if (CheckPrpTarInfo()) {
    AINFO << "prp tar";
    return;
  }
  // 2 rescue check tar info with trajectory intersection.
  if (CheckTarWithTrajInter()) {
    AINFO << "traj inter";
    return;
  }
  // 3 rescue check tar info with inital status.
  if (CheckTarWithAdcStatus()) {
    return;
  }
  // 4 reset
  tar_reverse_->Clear();
  return;
}

bool TEBPreObservationDecider::CheckPrpTarInfo() {
  if (tar_reverse_->Valid()) {
    return false;
  }
  if (nullptr == prp_reverse_) {
    return false;
  }
  if (prp_reverse_->Valid()) {
    AINFO << "prp_reverse_->Valid()";
    tar_reverse_ = prp_reverse_;
    tar_reverse_->SetStartTime(Clock::NowInSeconds() - kPrpCheckTimeDefault);
    // only tager once.
    prp_reverse_ = nullptr;
    if (!UpdateTarInfoWithId()) {
      AERROR << "No update Tar info.";
    }
    AINFO << "tar reserve time:" << tar_reverse_->Duration();
    return true;
  }
  return false;
}
bool TEBPreObservationDecider::CheckTarWithAdcStatus() {
  const Vec2d adc_position(vehicle_state_.x(), vehicle_state_.y());
  const double adc_heading = vehicle_state_.heading();
  // const auto adc_speed = vehicle_state_.linear_velocity();
  const auto& obstacles = obstacles_by_frame_->Items();
  double veh_length = vehicle_params_.length();
  double veh_width = vehicle_params_.width();
  double nearst_obstacle_s = std::numeric_limits<double>::max();
  double buffer_radian = 0.33 * M_PI_2;
  Vec2d tar_sl(0.0, 0.0);
  std::pair<std::string, int32_t> tar("\0", 0);
  for (const auto* it : obstacles) {
    if (nullptr == it || it->IsVirtual() || it->IsStatic() ||
        perception::PerceptionObstacle::VEHICLE != it->Perception().type()) {
      // AERROR << "Obstacle pointer is invalid.";
      continue;
    }
    const double obs_theta = it->Perception().theta();
    double theta_diff_obs_and_adc =
        century::common::math::NormalizeAngle(obs_theta - adc_heading);
    // pass no reverse car.
    if (theta_diff_obs_and_adc < -M_PI_2 - buffer_radian ||
        theta_diff_obs_and_adc > M_PI_2 + buffer_radian) {
    } else {
      AINFO << "not reverse car";
      continue;
    }
    if (!it->HasTrajectory()) {
      AINFO << "it has not trajectory.";
      continue;
    }
    auto& position = it->Trajectory().trajectory_point(0).path_point();
    const Vec2d it_position(position.x(), position.y());
    Vec2d it_sl(0.0, 0.0);
    CalcSLBasedPosition(adc_position, adc_heading, it_position, &it_sl);
    // skip back obstacle
    AINFO << "it sl " << it_sl.x() << " " << it_sl.y();
    if (it_sl.x() < veh_length * 0.5) {
      AINFO << "skip back obstacle";
      continue;
    }
    // skip deviate obstacle
    if (it_sl.y() > veh_width * 2) {
      AINFO << "skip deviate obstacle";
      continue;
    }

    // const double obs_theta = obstacle->Perception().theta();
    // const double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    // const auto& obs_reference_point =
    //     reference_line.GetNearestReferencePoint(obs_center_s);
    // double theta_diff_obs_and_ref = century::common::math::NormalizeAngle(
    //     obs_theta - obs_reference_point.heading());
    // double theta_diff_obs_and_adc =
    //     century::common::math::NormalizeAngle(obs_theta - adc_theta_);
    // double buffer_radian = kHalfReserveAngle / 90.0 * M_PI_2;
    // if (theta_diff_obs_and_ref < -M_PI_2 - buffer_radian ||
    //     theta_diff_obs_and_ref > M_PI_2 + buffer_radian ||
    //     theta_diff_obs_and_adc < -M_PI_2 - buffer_radian ||
    //     theta_diff_obs_and_adc > M_PI_2 + buffer_radian) {
    //   has_reverse_car = true;
    // }
    if (it_sl.x() < nearst_obstacle_s) {
      nearst_obstacle_s = it_sl.x();
      tar_sl = it_sl;
      tar.first = it->Id();
      tar.second = it->PerceptionId();
    }
  }
  if ("\0" == tar.first || (nearst_obstacle_s > kIgnoreDistance)) {
    AINFO << "no tar in adc status or ingorne.";
    return false;
  }
  tar_reverse_->ResetById(tar);
  if (!UpdateTarInfoWithId()) {
    AERROR << "No update Tar info.";
    return false;
  }
  AINFO << "tar reserve time:" << tar_reverse_->Duration();
  AINFO << "adc status tar";
  return true;
}
bool TEBPreObservationDecider::CheckTarWithTrajInter() {
  const auto* previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool is_last_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  bool is_last_block_dynamic_obj =
      previous_frame->open_space_info().is_blocked_by_dynamic_obj();
  if (!is_last_plan_success || !is_last_block_dynamic_obj) {
    return false;
  }
  const auto& tar_last =
      previous_frame->open_space_info().blocked_dynamic_obj();
  // TODO(zhiqiang.ding) should add hysteresis for tar.
  AINFO << "tar_last: " << tar_last.first << " " << tar_last.second;
  tar_reverse_->ResetById(tar_last);
  if (!UpdateTarInfoWithId()) {
    AERROR << "No update Tar info.";
    return false;
  }
  AINFO << "tar reserve time:" << tar_reverse_->Duration();
  AINFO << "trajectory tar";
  return true;
}
void TEBPreObservationDecider::SimpleHandleTar() {
  const auto* previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  bool is_last_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  bool is_last_fallback = previous_frame->open_space_info().fallback_flag();
  bool is_last_block_dynamic_obj =
      previous_frame->open_space_info().is_blocked_by_dynamic_obj();
  frame_->mutable_open_space_info()->set_is_yeild_flag(false);
  frame_->mutable_open_space_info()->set_is_tar_obj(true);

  if (is_last_plan_success && is_last_fallback && is_last_block_dynamic_obj) {
    // speed limit
    frame_->mutable_open_space_info()->set_is_tar_obj(true);
    // last stop because fallback.
    if (tar_reverse_->Duration() > kFallBackStopTime &&
        fabs(vehicle_state_.linear_velocity()) < kStopSpeed) {
      // yeild
      frame_->mutable_open_space_info()->set_is_yeild_flag(true);
    }
  }
  return;
}

bool TEBPreObservationDecider::UpdateTarInfoWithId() {
  // TODO(zhiqiang.ding) no use costmap because the lack of dynamic info and
  // id.
  if (!tar_reverse_->Valid()) {
    return false;
  }
  const auto* previous_frame = injector_->use_thread_in_play_street()
                                   ? injector_->frame_teb_history()->Latest()
                                   : injector_->frame_history()->Latest();
  // TODO(zhiqiang.ding): should use perception_id
  const auto& it = obstacles_by_frame_->Find(tar_reverse_->Id());
  if (nullptr == it) {
    AINFO << "not found tar in obstacles.";
    return false;
  }
  if (!it->HasTrajectory()) {
    AINFO << "tar has not trajectory.";
    return false;
  }
  auto& position = it->Trajectory().trajectory_point(0).path_point();
  const Vec2d tar_position(position.x(), position.y());
  Vec2d tar_sl(0.0, 0.0);
  // get vehicle current location
  const Vec2d adc_position(vehicle_state_.x(), vehicle_state_.y());
  const double adc_heading = vehicle_state_.heading();
  const auto adc_speed = vehicle_state_.linear_velocity();
  CalcSLBasedPosition(adc_position, adc_heading, tar_position, &tar_sl);
  AINFO << "tar_sl.x(): " << tar_sl.x() << "tar_sl.y(): " << tar_sl.y();
  tar_reverse_->SetLongDist(tar_sl.x());
  tar_reverse_->SetLatDist(tar_sl.y());
  tar_reverse_->SetDistanceToAdc(tar_sl.Length());
  tar_reverse_->SetSpeed(it->speed());
  // TODO(zhiqiang ding): use relative speed.
  //  it->Perception().velocity().x();
  //  it->Perception().velocity().y();
  double relative_v = std::fabs(adc_speed - it->speed());
  AINFO << "relative_v: " << relative_v;
  double ttc = tar_sl.x() / (kEpsilon + relative_v);
  double collision_time =
      previous_frame->open_space_info().will_collision_time();

  AINFO << "ttc: " << ttc << " ,  will_collision_time:" << collision_time;
  if (ttc < 0 || tar_sl.x() < 0 || collision_time < 0) {
    return false;
  }
  if (std::fabs(collision_time) < kEpsilon) {
    collision_time = kIgnoreTime;
  }
  tar_reverse_->SetCollisionTime(std::min(ttc, collision_time));
  return true;
}
void TEBPreObservationDecider::CalcSLBasedPosition(const Vec2d& start_point,
                                                   const double start_heading,
                                                   const Vec2d& end_point,
                                                   Vec2d* const result) {
  AINFO << "start_heading" << start_heading;
  const Vec2d position_diff = end_point - start_point;
  const Vec2d heading_vec = Vec2d::CreateUnitVec2d(start_heading);
  // x indicates s, y indicates l
  double s = position_diff.InnerProd(heading_vec);
  double l = Vec2d::ComputeNormalDistance(position_diff, heading_vec);
  if (l < 0) {
    AERROR << "normal distance error.";
  }
  result->set_x(s);
  result->set_y(l);
  return;
}

}  // namespace planning
}  // namespace century
