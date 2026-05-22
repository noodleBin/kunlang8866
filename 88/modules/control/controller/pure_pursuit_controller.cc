/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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

#include "modules/control/controller/pure_pursuit_controller.h"

#include <iomanip>
#include <algorithm>
#include <vector>
#include <cmath>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/common/control_gflags.h"

namespace century {
namespace control {

using century::common::ErrorCode;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::VehicleStateProvider;
using century::cyber::Clock;

#define RAD2DEG(x) (x) * 180.0 / M_PI
#define DEG2RAD(x) (x) * M_PI / 180.0

constexpr double preview_time = 0.2;            // response delay duration
constexpr double preview_distance_rate = 0.1;   // preview distance rate
constexpr double lateral_error_threshold = 0.1;
constexpr double smooth_ratio = 2.0 / 3.0;
constexpr size_t smooth_window_size = 10;

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "v"<< std::endl;
}

PPController::PPController() : name_("Pure Pursuit Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;  
}

PPController::~PPController() {}

bool PPController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[PPController] control_conf == nullptr";
    return false;
  }

  // vehicle parameter
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  wheelbase_ = vehicle_param_.wheel_base();
  half_wheelbase_ = wheelbase_ / 2.0;

  low_speed_bound_ = control_conf->lon_controller_conf().switch_speed();
  low_speed_window_ = control_conf->lon_controller_conf().switch_speed_window();

  steer_limit_ = control_conf->lat_controller_conf().steer_limit();

  min_radius_ = control_conf->lat_controller_conf().minimum_radius();

  // control cycle
  ts_ = control_conf->lat_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR << "[PPController] Invalid control update interval.";
    return false;
  }

  // preview window
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station();
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();

  min_lookahead_station_ =
      control_conf->lat_controller_conf().min_lookahead_station();
  last_preview_s_ = min_lookahead_station_;

  velocity_ratio_ = control_conf->lat_controller_conf().velocity_ratio();

  curvature_ratio_ = control_conf->lat_controller_conf().curvature_ratio();

  lateral_error_ratio_ = control_conf->lat_controller_conf().lateral_error_ratio();

  heading_error_ratio_ = control_conf->lat_controller_conf().heading_error_ratio();

  angular_velocity_threshold_ =
      control_conf->lat_controller_conf().angular_velocity_threshold();

  long_ld_lateral_error_threshold_ =
      control_conf->lat_controller_conf().long_ld_lateral_error_threshold();

  heading_weight_ =
      control_conf->lat_controller_conf().heading_weight();

  query_relative_time_ = control_conf->query_relative_time();
  return true;
}

void PPController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[PPController parameters]"
        << " wheelbase_ " << wheelbase_ << ","
        << " ts_ " << ts_ << ","
        << " low_speed_bound_ " << low_speed_bound_ << ","
        << " lookahead_station_low_speed_ " << lookahead_station_low_speed_ << ","
        << " lookback_station_low_speed_ " << lookback_station_low_speed_ << ","
        << " lookahead_station_high_speed_ " << lookahead_station_high_speed_ << ","
        << " lookback_station_high_speed_ " << lookback_station_high_speed_ << ","
        << " min_lookahead_station_ " << min_lookahead_station_ << ","
        << " velocity_ratio_ " << velocity_ratio_ << ","
        << " curvature_ratio_ " << curvature_ratio_ << ","
        << " lateral_error_ratio_ " << lateral_error_ratio_ << ","
        << " long_ld_lateral_error_threshold_ " << long_ld_lateral_error_threshold_ << ","
        << " heading_error_ratio_ " << heading_error_ratio_ << ","
        << " heading_weight_ " << heading_weight_ << ","
        << " steer_limit_ " << steer_limit_ << ","
        << " minimum_radius_ " << min_radius_ << ","
        << " angular_velocity_threshold_ " << angular_velocity_threshold_;
}

Status PPController::Init(std::shared_ptr<DependencyInjector> injector,
                          const ControlConf *control_conf) {
  control_conf_ = control_conf;
  injector_ = injector;
  if (!LoadControlConf(control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");  
  }
  LogInitParameters();
  cmd_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(10));
  last_gear_position_ = canbus::Chassis::GEAR_NONE;
  last_cmd_queue_.clear();
  return Status::OK();
}

Status PPController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    const planning::AebResult *aeb_result,
    ControlCommand *cmd) {

  // vehicle state provider
  auto vehicle_state = injector_->vehicle_state();
  // check driving mode and gear
  if (chassis->driving_mode() == canbus::Chassis::COMPLETE_MANUAL ||
      chassis->driving_mode() == canbus::Chassis::AUTO_SPEED_ONLY ||
      chassis->driving_mode() == canbus::Chassis::EMERGENCY_MODE ||
      vehicle_state->gear() != last_gear_position_) {
    AINFO << "driving mode is " << int(chassis->driving_mode());
    last_gear_position_ = canbus::Chassis::GearPosition(vehicle_state->gear());
    last_cmd_queue_.clear();
    cmd->set_steering_target(0.0);
    cmd->set_target_steering_angle_1axis(0.0);
    cmd->set_target_steering_angle_4axis(0.0);
    return Status::OK();
  }

  AINFO << "vehicle state " << vehicle_state->vehicle_state().ShortDebugString();

  // trajectory from planning module
  auto target_tracking_trajectory = *planning_published_trajectory;

  // trajectory analyzer
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory, true));

  // update driving_orientation_
  UpdateDrivingOrientation();

  // debug
  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // compute lateral error and heading error
  UpdateState(debug);

  double steer_angle = 0.0;
  if (trajectory_analyzer_.TotalLength() == 0.0 ||
      vehicle_state->gear() != planning_published_trajectory->gear()) {
    last_cmd_queue_.clear();
    steer_angle = 0.0;
  } else {
    /*
     * preview time
     */
    double vehicle_state_x = vehicle_state->x();
    double vehicle_state_y = vehicle_state->y();
    double vehicle_state_v = std::fabs(vehicle_state->linear_velocity());
    double vehicle_state_angular_v = vehicle_state->angular_velocity();
    driving_orientation_ = common::math::NormalizeAngle(
        driving_orientation_ + vehicle_state_angular_v * preview_time);
    vehicle_state_x += (vehicle_state_v * preview_time) * cos(driving_orientation_);
    vehicle_state_y += (vehicle_state_v * preview_time) * sin(driving_orientation_);

    AINFO << "control point path_point { "
          << "x: " << std::to_string(vehicle_state_x) << ' '
          << "y: " << std::to_string(vehicle_state_y) << ' '
          << "theta: " << driving_orientation_ << " } "
          << "v: " << vehicle_state->linear_velocity() << ' '
          << "a: " << vehicle_state->angular_velocity();

    const auto nearest_point =
        trajectory_analyzer_.QueryNearestPointByPosition(
        vehicle_state_x, vehicle_state_y);
    AINFO << "nearest point " << nearest_point.ShortDebugString();


    // get preview point
    double preview_s = velocity_ratio_
                     * std::fabs(log(std::fabs(vehicle_state->linear_velocity()) + M_E))
                     - curvature_ratio_ * std::fabs(nearest_point.path_point().kappa());
    if (std::fabs(debug->lateral_error()) >= lateral_error_threshold) {
      preview_s += lateral_error_ratio_ * std::fabs(debug->lateral_error());
    }
    preview_s += heading_error_ratio_ * std::fabs(debug->heading_error());
    if (std::fabs(preview_s - last_preview_s_) > preview_distance_rate) {
      preview_s = last_preview_s_ + copysign(1.0, preview_s - last_preview_s_)
                * preview_distance_rate;
    }
    if (std::fabs(vehicle_state->linear_velocity()) > low_speed_bound_) {
      preview_s = common::math::Clamp(preview_s, min_lookahead_station_,
                                      lookahead_station_high_speed_);
    } else {
      preview_s = common::math::Clamp(preview_s, min_lookahead_station_,
                                      lookahead_station_low_speed_);
    }
    last_preview_s_ = preview_s;
    const auto preview_point = trajectory_analyzer_.QueryNearestPointByS(
                                   nearest_point.path_point().s() + preview_s);
    AINFO << "preview point " << preview_point.ShortDebugString();

    // vector from vehicle center to preview point
    common::math::Vec2d delta = common::math::Vec2d(preview_point.path_point().x(),
                                                    preview_point.path_point().y())
                              - common::math::Vec2d(vehicle_state->x(),
                                                    vehicle_state->y());
    // ld
    double ld = delta.Length();

    // alpha angle
    double alpha = common::math::NormalizeAngle(delta.Angle() - driving_orientation_);
    // turning radius
    double R = ld / (2.0 * sin(alpha) + common::math::kMathEpsilon);
    R = std::copysign(1.0, R) * std::fmax(std::fabs(R), min_radius_);
    AINFO << "radius " << R;
    // four wheel steering model
    steer_angle = RAD2DEG(atan(half_wheelbase_ / R) -
                          heading_weight_ * debug->heading_error());
    // car model
    //steer_angle = atan(wheelbase_ / R);
  }

  // limit steer angle
  steer_angle = common::math::Clamp(steer_angle, -steer_limit_, steer_limit_);
  //steer_angle = cmd_filter_.Update(steer_angle);
  steer_angle = FilterCmd(steer_angle);

  // reverse
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    steer_angle *= -1.0;
  }

  AINFO << "steer angle " << steer_angle;
  AINFO << "wheel steer feedback ( "
        << (chassis->bridge_1_left_wheel_angle() +
            chassis->bridge_1_right_wheel_angle()) / 2.0 << ','
        << (chassis->bridge_4_left_wheel_angle() +
            chassis->bridge_4_right_wheel_angle()) / 2.0 << " )";
  cmd->set_steering_target(steer_angle);
  debug->set_steer_angle(steer_angle);
  debug->set_ref_speed(vehicle_state->linear_velocity());

  // front wheel
  cmd->set_target_steering_angle_1axis(steer_angle);
  // rear wheel
  cmd->set_target_steering_angle_4axis(-steer_angle);

  ProcessLogs(debug, chassis);
  return Status::OK();
}

double PPController::FilterCmd(const double steer) {
  // filter
  size_t size = last_cmd_queue_.size();
  double filtered_steer = (smooth_ratio + pow((1.0 - smooth_ratio), size + 1)) * steer;
  for (size_t i = 0; i < size; ++i) {
    filtered_steer += smooth_ratio * pow((1.0 - smooth_ratio), i + 1) * last_cmd_queue_[i];
  }
  // angular velocity constraint
  double last_steer = (last_cmd_queue_.empty() ? 0.0 : last_cmd_queue_.front());
  if (std::fabs(filtered_steer - last_steer) / ts_ > angular_velocity_threshold_) {
    filtered_steer = last_steer + copysign(1.0, filtered_steer - last_steer)
                   * angular_velocity_threshold_ * ts_;
  }
  // save cmd
  if (size >= smooth_window_size) {
    last_cmd_queue_.pop_back();
  }
  last_cmd_queue_.push_front(filtered_steer);
  return filtered_steer;
}

void PPController::UpdateState(SimpleLateralDebug *debug) {
  auto vehicle_state = injector_->vehicle_state();
  if (FLAGS_use_navigation_mode) {
    ComputeLateralErrors(
        vehicle_state->x(), vehicle_state->y(), driving_orientation_,
        std::fabs(vehicle_state->linear_velocity()), vehicle_state->angular_velocity(),
        std::fabs(vehicle_state->linear_acceleration()), trajectory_analyzer_, debug);
  } else {
    ComputeLateralErrors(
        vehicle_state->x(), vehicle_state->y(), driving_orientation_,
        std::fabs(vehicle_state->linear_velocity()), vehicle_state->angular_velocity(),
        std::fabs(vehicle_state->linear_acceleration()), trajectory_analyzer_, debug);
  }
}

// compute lateral error
void PPController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug) {

  // get target point
  TrajectoryPoint target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  // position error
  const double dx = x - target_point.path_point().x();
  const double dy = y - target_point.path_point().y();

  // set target point
  debug->mutable_current_target_point()->CopyFrom(target_point);

  ADEBUG << "x point: " << x << " y point: " << y;
  ADEBUG << "match point information: " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point().theta());
  const double sin_target_heading = std::sin(target_point.path_point().theta());

  // compute lateral error
  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  debug->set_lateral_error(lateral_error);

  // set target heading
  debug->set_ref_heading(target_point.path_point().theta());
  debug->set_heading(driving_orientation_);

  // heading error
  double heading_error = common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(heading_error);

  //
  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(linear_v) >= low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = common::math::lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    lookback_station = common::math::lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
  }

  double heading_error_feedback = 0.0;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = heading_error;
  } else {
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
        target_point.relative_time() +
        lookahead_station / (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
    heading_error_feedback = common::math::NormalizeAngle(
        heading_error + target_point.path_point().theta() -
        lookahead_point.path_point().theta());
  }
  // heading error with lookahead_point
  debug->set_heading_error_feedback(heading_error_feedback);

  double lateral_error_feedback = 0.0;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    lateral_error_feedback =
        lateral_error - lookback_station * std::sin(heading_error);
  } else {
    lateral_error_feedback =
        lateral_error + lookahead_station * std::sin(heading_error);
  }
  // lateral error with lookahead_point
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  // lateral velocity
  debug->set_lateral_error_rate(lateral_error_dot);
  // lateral acceleration
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  // lateral jerk
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  // angular rate
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  // target angular_v
  debug->set_ref_heading_rate(target_point.path_point().kappa() * target_point.v());
  // angular_v error
  debug->set_heading_error_rate(debug->heading_rate() - debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  previous_heading_rate_ = debug->heading_rate();

  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_error_acceleration(
      debug->heading_acceleration() - debug->ref_heading_acceleration());
  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  previous_heading_acceleration_ = debug->heading_acceleration();
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) / ts_);
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
  
  debug->set_heading_error_jerk(debug->heading_jerk() - debug->ref_heading_jerk());

  // target point kappa
  debug->set_curvature(target_point.path_point().kappa());
}

Status PPController::Reset() {
  return Status::OK();
}

void PPController::Stop() { CloseLogFile(); }

std::string PPController::Name() const { return name_; }

void PPController::UpdateDrivingOrientation() {
  // vehicle state
  auto vehicle_state = injector_->vehicle_state();
  // driving heading
  driving_orientation_ = vehicle_state->heading();
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {    // reverse
    driving_orientation_ =
        common::math::NormalizeAngle(driving_orientation_ + M_PI);
    ADEBUG << "orientation changed due to gear direction";
  }
}

void PPController::ProcessLogs(const SimpleLateralDebug *debug,
                               const canbus::Chassis *chassis) {
  const std::string log_str = absl::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
      ",", debug->heading_error(), ",", debug->heading_error_rate(), ",",
      debug->lateral_error_rate(), ",", debug->curvature(), ",",
      debug->steer_angle(), ",",
      injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  AINFO << "Steer_control_Detail: " << log_str;
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void PPController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

}
}
