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

#include "modules/control/controller/lqr_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/common/control_gflags.h"

namespace century {
namespace control {

using century::common::ErrorCode;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using century::canbus::Chassis;
using century::cyber::Clock;
using century::cyber::Time;

namespace {
constexpr double kSteerReduction = 0.98;
constexpr double kMaxQ0 = 0.15;
constexpr double kMinQ0 = 0.00025;
constexpr double kMaxK2 = 1.45;
constexpr double kMaxK0 = 1.5;
constexpr double kFixedVelocity = 5.0;
constexpr double kReachEndS = 0.3;
constexpr double kStopSpeed = 0.1;
constexpr int kLessTrajPointNum = 3;
constexpr uint32_t kMaxLessTrajPointCount = 500;
constexpr double kCurveProtected = 0.25;
constexpr double kMaxVUseOtherFeedforward = 3.5;
constexpr double kMinVUseOtherFeedforward = 1.75;
constexpr double kMaxVOpenspace = 2.0;
constexpr double kMinVOpenspace = 0.25;
constexpr double kPreviewTimeOpenspace = 0.4;
constexpr double kLateralFluctuationThreshold = 0.05;
constexpr double kMaxSteeringAngleLocked = 5.0;
constexpr double kMaxHeadingErrorThreshold = M_PI / 3;
constexpr double kMinTrajectoryLength = 0.25;
constexpr double kPathDataPreviewTime = 0.10;
constexpr double kVelocityAlmostZero = 0.12;
constexpr double kMinLatErrorDebug = 0.20;
constexpr double kMaxWheelAngle = 32;
constexpr double kMaxDiagonalCompensateWheelAngle = 30;
constexpr double kMaxHeadingErrorDiagonal = M_PI / 10;
constexpr double kMaxKPAtStright = 0.05;
constexpr double kMaxKHeadingAtStright = 0.75;
constexpr double kMaxHeading_rateAtStright = 0.2;
constexpr double kMax_l_rate_AtStright = 0.02;
constexpr double KturndecreaseSpd = 1.8;
constexpr int KheadingErrorStableCnt = 10;
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
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

LQRController::LQRController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  // AINFO << "Using " << name_;
}

LQRController::~LQRController() { CloseLogFile(); }

bool LQRController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[LQRController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf->lat_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station();
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base() / 2;
  steer_ratio_ = vehicle_param_.steer_ratio();

  //  need be 1 when use Wheel angle control
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();
  low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();
  low_speed_window_ =
      control_conf_->lon_controller_conf().switch_speed_window();
  min_q0_ = control_conf_->lat_controller_conf().min_q0();
  q0_offset_ = control_conf_->lat_controller_conf().q0_offset();
  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;
  // mass_ = cmd->debug()->simple_lon_debug().mass_update();

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia

  // iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  iz_ = mass_ *
        (vehicle_param_.width() * vehicle_param_.width() +
         vehicle_param_.length() * vehicle_param_.length()) /
        12;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

  query_relative_time_ = control_conf->query_relative_time();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  lateral_error_threshold_ =
      control_conf_->lat_controller_conf().lateral_error_threshold();

  steer_limit_velocity_threshold_ =
      control_conf_->lat_controller_conf().steer_limit_velocity_threshold();

  max_centripetal_acceleration_rate_ =
      control_conf_->lat_controller_conf().max_centripetal_acceleration_rate();

  return true;
}

void LQRController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  const std::string log_str = absl::StrCat(
      debug->preview_lateral_error(), ",", debug->ref_heading(), ",",
      debug->heading(), ",", debug->heading_error(), ",",
      debug->heading_error_rate(), ",", debug->lateral_error_rate(), ",",
      debug->curvature(), ",", debug->steer_angle(), ",",
      debug->steer_angle_feedforward(), ",",
      debug->steer_angle_lateral_contribution(), ",",
      debug->steer_angle_lateral_rate_contribution(), ",",
      debug->steer_angle_heading_contribution(), ",",
      debug->steer_angle_heading_rate_contribution(), ",",
      debug->steer_angle_feedback(), ",", chassis->bridge_1_right_wheel_angle(),
      ",", injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LQRController::LogInitParameters() {
  AINFO << name_ << " begin.";
}

void LQRController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  real_lateral_error_filter_ =
      common::MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf->lat_controller_conf().mean_filter_window_size()));
  l_rate_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf->lat_controller_conf().lateral_rate_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().heading_rate_filter_window_size()));
  heading_error_rate_filter_ = common::MeanFilter(
      static_cast<std::uint_fast8_t>(control_conf->lat_controller_conf()
                                         .heading_rate_filter_window_size()));
  curve_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  steering_straightaway_filter_ = common::MeanFilter(
      static_cast<std::uint_fast8_t>(control_conf->lat_controller_conf()
                                         .mean_secondary_filter_window_size()));

  steering_curve_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf()
          .mean_secondary_filter_window_size_curve()));
}

Status LQRController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  control_conf_ = control_conf;
  injector_ = injector;
  if (!LoadControlConf(control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);

  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
   (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    const auto error_msg = absl::StrCat(
        "lateral controller error: matrix_q size: ", q_param_size,
        "lateral controller error: reverse_matrix_q size: ",
        reverse_q_param_size,
        " in parameter file not equal to matrix_size: ", matrix_size);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) =
        lanefollow_flag_
            ? control_conf_->lat_controller_conf().matrix_q(i)
            : control_conf_->lat_controller_conf().matrix_q_openspace(i);
  }

  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf_);
  auto &lat_controller_conf = control_conf_->lat_controller_conf();
  LoadLatGainScheduler(lat_controller_conf);
  LogInitParameters();

  enable_mrac_ =
      control_conf_->lat_controller_conf().enable_steer_mrac_control();
  if (enable_mrac_) {
    mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(),
                          vehicle_param_.steering_latency_param(), ts_);
  }

  enable_look_ahead_back_control_ =
      control_conf_->lat_controller_conf().enable_look_ahead_back_control();

  return Status::OK();
}

void LQRController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LQRController::LoadLatGainScheduler(
    const LatControllerConf &lat_controller_conf) {
  const auto &lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();
  // AINFO << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

void LQRController::Stop() { CloseLogFile(); }

std::string LQRController::Name() const { return name_; }

Status LQRController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    const planning::AebResult *aeb_result,
    ControlCommand *const cmd) {
  const auto &start_time = Clock::Now();
  const auto &vehicle_state = injector_->vehicle_state();
  localization_ = localization;
  chassis_ = chassis;
  target_tracking_trajectory_ = *planning_published_trajectory;
  lanefollow_flag_ = century::planning::ADCTrajectory::LANEFOLLOW ==
                     planning_published_trajectory->trajectory_scenario();
  size_t traj_size = target_tracking_trajectory_.trajectory_point().size();
  ADEBUG << "SIZE: " << traj_size;
  cmd->set_guide1_zhuanxiang_mode(STEERINGMODE_FIGURE_EIGHT);
  auto path_array = target_tracking_trajectory_.debug().planning_data().path();
  google::protobuf::RepeatedPtrField<century::common::PathPoint> path_data;
  for (int i = 0; i < path_array.size(); ++i) {
    if ("Planning PathData" == path_array[i].name()) {
      path_data = path_array[i].path_point();

      break;
    }
  }
  path_data_size_ = path_data.size();
  if (!TrajectoryAnomalyDetection(traj_size, cmd)) {
    return Status::OK();
  }
  if (FLAGS_use_navigation_mode &&
      FLAGS_enable_navigation_mode_position_update) {
    NavigationModePositionUpdate();
  }

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory_));
  if (CalculateReachEndS(traj_size, path_data, cmd)) {
    return Status::OK();
  }

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  if (((FLAGS_trajectory_transform_to_com_reverse &&
         canbus::Chassis::GEAR_REVERSE == vehicle_state->gear()) ||
       (FLAGS_trajectory_transform_to_com_drive &&
        canbus::Chassis::GEAR_DRIVE == vehicle_state->gear())) &&
      enable_look_ahead_back_control_) {
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }
  UpdateStaticMatrix();

  UpdateDrivingOrientation();

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  if (!UpdateState(traj_size, debug, cmd)) {
    // ResetData();
    cmd->set_steering_target(pre_steer_angle_1axis_);
    // front wheel
    cmd->set_target_steering_angle_1axis(pre_steer_angle_1axis_);
    // rear wheel
    cmd->set_target_steering_angle_4axis(pre_steer_angle_4axis_);
    cmd->set_steering_rate(FLAGS_steer_angle_rate);
    pre_steer_angle_1axis_ = pre_steer_angle_1axis_ * kSteerReduction;
    pre_steer_angle_4axis_ = pre_steer_angle_4axis_ * kSteerReduction;
    const auto error_msg =
        absl::StrCat("Input detection failed, steering wheel gradually reset");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }

  if (FLAGS_reverse_heading_control &&
      canbus::Chassis::GEAR_DRIVE == injector_->vehicle_state()->gear() &&
      target_tracking_trajectory_.is_backward_trajectory()) {
    cmd->set_steering_target(pre_steer_angle_1axis_);
    // front wheel
    cmd->set_target_steering_angle_1axis(pre_steer_angle_1axis_);
    // rear wheel
    cmd->set_target_steering_angle_4axis(pre_steer_angle_4axis_);
    cmd->set_steering_rate(FLAGS_steer_angle_rate);
    pre_steer_angle_1axis_ = pre_steer_angle_1axis_ * kSteerReduction;
    pre_steer_angle_4axis_ = pre_steer_angle_4axis_ * kSteerReduction;
    const auto error_msg = absl::StrCat(
        "Is backward trajectory,the desired gear is inconsistent with the "
        "actual gear");
    AERROR << error_msg;
    return Status::OK();
  }

  if (DiagonalModeCompute(chassis, planning_published_trajectory, cmd, debug)) {
    return Status::OK();
  }
  
  UpdateMatrix();

  // Compound discrete matrix with road preview model
  UpdateMatrixCompound();

  // Adjust matrix_q_updated when in reverse gear
  UpdateMatrixQ(debug);

  // Add gain scheduler for higher speed steering

  SolveMatrixK();

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio

  SteeringAngleCalculate(chassis, debug, cmd);

  ProcessLogs(debug, chassis);
  const auto &end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "lat control time spend: " << time_diff_ms << " ms.";
  return Status::OK();
}

Status LQRController::Reset() {
  matrix_state_.setZero();
  if (enable_mrac_) {
    mrac_controller_.Reset();
  }
  return Status::OK();
}

bool LQRController::UpdateState(const size_t traj_size,
                                SimpleLateralDebug *debug,
                                ControlCommand *const cmd) {
  auto vehicle_state = injector_->vehicle_state();
  if (FLAGS_use_navigation_mode) {
    if (!ComputeLateralErrors(traj_size, 0.0, 0.0, driving_orientation_,
                              vehicle_state->linear_velocity(),
                              vehicle_state->angular_velocity(),
                              vehicle_state->linear_acceleration(),
                              trajectory_analyzer_, debug, cmd)) {
      AERROR << "Failed to calculate the lateral error";
      return false;
    }
  } else {
    // Transform the coordinate of the vehicle states from the center of the
    // rear-axis to the center of mass, if conditions matched
    const auto &com = vehicle_state->ComputeCOMPosition(0);
    if (!ComputeLateralErrors(traj_size, com.x(), com.y(), driving_orientation_,
                              vehicle_state->linear_velocity(),
                              vehicle_state->angular_velocity(),
                              vehicle_state->linear_acceleration(),
                              trajectory_analyzer_, debug, cmd)) {
      AERROR << "Failed to calculate the lateral error";
      return false;
    }
  }

  // State matrix update;
  // First four elements are fixed;
  if (enable_look_ahead_back_control_) {
    matrix_state_(0, 0) = debug->lateral_error_feedback();
    matrix_state_(2, 0) = debug->heading_error_feedback();
  } else {
    matrix_state_(0, 0) = debug->preview_lateral_error();
    matrix_state_(2, 0) = debug->heading_error();
  }
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);
    const auto preview_point =
        trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

    const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
        preview_point.path_point().x(), preview_point.path_point().y());

    const double dx =
        preview_point.path_point().x() - matched_point.path_point().x();
    const double dy =
        preview_point.path_point().y() - matched_point.path_point().y();

    const double cos_matched_theta =
        std::cos(matched_point.path_point().theta());
    const double sin_matched_theta =
        std::sin(matched_point.path_point().theta());
    const double preview_d_error =
        cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
  return true;
}

void LQRController::UpdateMatrix() {
  double v = 0;
  // At reverse driving, replace the lateral translational motion dynamics with
  // the corresponding kinematic models
  if (!need_forward_control_logic_) {
    v = std::min(injector_->vehicle_state()->linear_velocity(),
                 -minimum_speed_protection_);

    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    v = std::max(injector_->vehicle_state()->linear_velocity(),
                 minimum_speed_protection_);
    matrix_a_(0, 2) = 0.0;
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  ADEBUG << "[LQRController matrix_a_]"
         << " matrix_a_(1, 1): " << matrix_a_(1, 1) << ","
         << " matrix_a_(1, 3): " << matrix_a_(1, 3) << ","
         << " v: " << v << ","
         << " (lr_ * cr_ - lf_ * cf_) / mass_: "
         << (lr_ * cr_ - lf_ * cf_) / mass_;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LQRController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LQRController::ComputeFeedForward(double ref_curvature) const {
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  ADEBUG << "steering gradient " << kv;
  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = injector_->vehicle_state()->linear_velocity();
  double steer_angle_feedforwardterm = 0;
  double curve_feedfordward =
      wheelbase_ * ref_curvature * 180 / M_PI * steer_ratio_;
  double feedfordward_cmpt = 1.5;
  curve_feedfordward *= feedfordward_cmpt;
  double other_feedfordward =
      (kv * v * v * ref_curvature -
       matrix_k_(0, 2) * ((lr_ - lf_) * ref_curvature -
                          (lf_ / cr_ + lr_ / cf_) * mass_ * v * v *
                              ref_curvature / 2 / wheelbase_)) *
      180 / M_PI * steer_ratio_;
  ADEBUG << "curve_feedfordward " << curve_feedfordward << "other_feedfordward "
         << other_feedfordward;

  steer_angle_feedforwardterm = curve_feedfordward + other_feedfordward;
  return steer_angle_feedforwardterm / 2;
}

bool LQRController::ComputeLateralErrors(
    const size_t traj_size, const double x, const double y, const double theta,
    const double linear_v, const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug,
    ControlCommand *const cmd) {
  TrajectoryPoint adc_point =
      trajectory_analyzer.QueryNearestPointByPositionInterpolation(
          x, y, injector_->vehicle_state()->heading());
  TrajectoryPoint target_point;

  if (FLAGS_query_time_nearest_point_only) {
    target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
        Clock::NowInSeconds() + query_relative_time_);
  } else {
    if (FLAGS_use_navigation_mode &&
        !FLAGS_enable_navigation_mode_position_update) {
      target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
          Clock::NowInSeconds() + query_relative_time_);
    } else {
      // preview point is directly get here ,
      // not enable_look_ahead_back_control_
      
      auto min_preview_distance_lat =
          lanefollow_flag_
              ? control_conf_->lat_controller_conf().min_preview_distance_lat()
              : control_conf_->lat_controller_conf()
                    .min_preview_distance_lat_teb();

      size_t index_min = 0;
      for (size_t i = 1; i < trajectory_analyzer_.trajectory_points().size();
           ++i) {
        if (trajectory_analyzer_.trajectory_points()[i].path_point().s() <
            adc_point.path_point().s() + min_preview_distance_lat) {
          index_min = i;
        } else {
          break;
        }
      }
      double preview_time_temp = 0.0;
      if (!lanefollow_flag_) {
        preview_time_temp =
            control_conf_->lat_controller_conf().preview_time_openspace();
      } else {
        preview_time_temp =
            enable_use_path_data_ ? kPathDataPreviewTime : query_relative_time_;
      }

      auto final_preview_time = std::max(
          trajectory_analyzer_.trajectory_points()[index_min].relative_time(),
          adc_point.relative_time() + preview_time_temp);
      debug->set_final_preview_time(preview_time_temp);
      target_point =
          trajectory_analyzer.QueryNearestPointByRelativeTimeInterpolation(
              final_preview_time);
      debug->set_query_relative_time(query_relative_time_);
      debug->set_final_preview_time(final_preview_time);
    }
  }
  TrajectoryPoint curvature_preview_point;
  auto min_preview_distance_cur =
      control_conf_->lat_controller_conf().min_preview_distance_cur();
  auto curvature_preview_time =
      control_conf_->lat_controller_conf().curvature_preview();
  size_t index_min_cur = 0;
  for (size_t i = 1; i < trajectory_analyzer_.trajectory_points().size(); ++i) {
    if (trajectory_analyzer_.trajectory_points()[i].path_point().s() <
        adc_point.path_point().s() + min_preview_distance_cur) {
      index_min_cur = i;
    } else {
      break;
    }
  }
  auto final_preview_time_cur = std::max(
      trajectory_analyzer_.trajectory_points()[index_min_cur].relative_time(),
      adc_point.relative_time() + curvature_preview_time);
  debug->set_curvature_preview_time(curvature_preview_time);
  debug->set_final_preview_time_cur(final_preview_time_cur);
  curvature_preview_point =
      trajectory_analyzer.QueryNearestPointByRelativeTimeInterpolation(
          final_preview_time_cur);
  debug->set_curvature_preview(
      control_conf_->lat_controller_conf().curvature_preview());
  if (!InputAnomalyDetection(injector_->vehicle_state(), chassis_,
                             &target_tracking_trajectory_, target_point,
                             adc_point)) {
    AERROR << "There is abnormal input";
    // return false;
  }
  AINFO << " FLAGS_reverse_heading_control: " << FLAGS_reverse_heading_control;
  AINFO << " vehicle_state()->gear: " << injector_->vehicle_state()->gear();
  AINFO << " is_backward_trajectory: " << target_tracking_trajectory_.is_backward_trajectory();
  AINFO << " vehicle_state()->heading: " << injector_->vehicle_state()->heading() / M_PI *180;
  AINFO << " target_point.path_point.theta: " << target_point.path_point().theta() / M_PI *180;
  // if (FLAGS_reverse_heading_control &&
  //     canbus::Chassis::GEAR_REVERSE != injector_->vehicle_state()->gear() &&
  //     target_tracking_trajectory_.is_backward_trajectory()) {
  //   if (std::abs(common::math::NormalizeAngle(
  //           injector_->vehicle_state()->heading() + M_PI -
  //           target_point.path_point().theta())) > kMaxHeadingErrorThreshold) {
  //     AERROR
  //         << "The difference between the current heading and expected heading "
  //            "is too large";
  //     return false;
  //   }
  // } else {
  //   if (std::abs(common::math::NormalizeAngle(
  //           injector_->vehicle_state()->heading() -
  //           target_point.path_point().theta())) > kMaxHeadingErrorThreshold) {
  //     AERROR
  //         << "The difference between the current heading and expected heading "
  //            "is too large";
  //     return false;
  //   }
  // }
  if (FLAGS_reverse_heading_control &&
      canbus::Chassis::GEAR_REVERSE == injector_->vehicle_state()->gear() &&
      target_tracking_trajectory_.is_backward_trajectory()) {
    if (std::abs(common::math::NormalizeAngle(
            injector_->vehicle_state()->heading() -
            target_point.path_point().theta())) > kMaxHeadingErrorThreshold) {
      AERROR
          << "The difference between the current heading and expected heading "
             "is too large while R Direction";
      return false;
    }
  } else if (canbus::Chassis::GEAR_DRIVE == injector_->vehicle_state()->gear() &&
      !target_tracking_trajectory_.is_backward_trajectory()) {
    if (std::abs(common::math::NormalizeAngle(
            injector_->vehicle_state()->heading() -
            target_point.path_point().theta())) > kMaxHeadingErrorThreshold) {
      AERROR
          << "The difference between the current heading and expected heading "
             "is too large while D Direction";
      return false;
    }
  }
  need_forward_control_logic_ =
      canbus::Chassis::GEAR_DRIVE == injector_->vehicle_state()->gear() ||
      (FLAGS_reverse_heading_control &&
       canbus::Chassis::GEAR_REVERSE == injector_->vehicle_state()->gear() &&
       target_tracking_trajectory_.is_backward_trajectory());
  CalculateErrorSubfunction(x, y, theta, linear_v, linear_a, angular_v,
                            trajectory_analyzer, target_point, adc_point,
                            curvature_preview_point, debug, cmd);
  return true;
}

void LQRController::UpdateDrivingOrientation() {
  auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
}

void LQRController::NavigationModePositionUpdate() {
  auto time_stamp_diff = target_tracking_trajectory_.header().timestamp_sec() -
                         current_trajectory_timestamp_;

  auto curr_vehicle_x = localization_->pose().position().x();
  auto curr_vehicle_y = localization_->pose().position().y();

  double curr_vehicle_heading = 0.0;
  const auto &orientation = localization_->pose().orientation();
  if (localization_->pose().has_heading()) {
    curr_vehicle_heading = localization_->pose().heading();
  } else {
    curr_vehicle_heading = common::math::QuaternionToHeading(
        orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
  }

  // new planning trajectory
  if (time_stamp_diff > 1.0e-6) {
    init_vehicle_x_ = curr_vehicle_x;
    init_vehicle_y_ = curr_vehicle_y;
    init_vehicle_heading_ = curr_vehicle_heading;

    current_trajectory_timestamp_ =
        target_tracking_trajectory_.header().timestamp_sec();
  } else {
    auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
    auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
    auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

    auto cos_map_veh = std::cos(init_vehicle_heading_);
    auto sin_map_veh = std::sin(init_vehicle_heading_);

    auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
    auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

    auto cos_theta_diff = std::cos(-theta_diff);
    auto sin_theta_diff = std::sin(-theta_diff);

    auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
    auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

    auto ptr_trajectory_points =
        target_tracking_trajectory_.mutable_trajectory_point();
    std::for_each(ptr_trajectory_points->begin(), ptr_trajectory_points->end(),
                  [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
                   &theta_diff](common::TrajectoryPoint &p) {
                    auto x = p.path_point().x();
                    auto y = p.path_point().y();
                    auto theta = p.path_point().theta();

                    auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
                    auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
                    auto theta_new =
                        common::math::NormalizeAngle(theta - theta_diff);

                    p.mutable_path_point()->set_x(x_new);
                    p.mutable_path_point()->set_y(y_new);
                    p.mutable_path_point()->set_theta(theta_new);
                  });
  }
}

void LQRController::UpdateStaticMatrix() {
  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)
  if (!need_forward_control_logic_) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = -control_conf_->lat_controller_conf().cf();
    cr_ = -control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = control_conf_->lat_controller_conf().cf();
    cr_ = control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = (cf_ - cr_) / mass_;
  matrix_b_(3, 0) = (lf_ * cf_ + lr_ * cr_) / iz_;
  matrix_bd_ = matrix_b_ * ts_;
}

void LQRController::UpdateMatrixQ(SimpleLateralDebug *debug) {
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (!need_forward_control_logic_) {
    for (int i = 0; i < reverse_q_param_size; ++i) {
      matrix_q_(i, i) =
          control_conf_->lat_controller_conf().reverse_matrix_q(i);
    }
  } else {
    for (int i = 0; i < q_param_size; ++i) {
      matrix_q_(i, i) =
          lanefollow_flag_
              ? control_conf_->lat_controller_conf().matrix_q(i)
              : control_conf_->lat_controller_conf().matrix_q_openspace(i);
    }
  }

  if (need_forward_control_logic_ && lanefollow_flag_) {
    // TODO(all): the function need more test, now is do more harm than good
    // may you should adjust q2 and try futher test

    // double min_q0 = min_q0_ * heading_err_interpolation_->Interpolate(
    //                               std::fabs(vehicle_state->linear_velocity()));

    double min_q0 = min_q0_;
    double final_q0 = min_q0;
    double min_curvature_threshold =
        control_conf_->lat_controller_conf().min_curvature_threshold();
    double max_curvature_threshold =
        control_conf_->lat_controller_conf().max_curvature_threshold();
    if (std::fabs(debug->curvature()) <= min_curvature_threshold) {
      final_q0 = min_q0;
    } else if (std::fabs(debug->curvature()) < max_curvature_threshold) {
      final_q0 = (std::fabs(debug->curvature()) - min_curvature_threshold) *
                     (kMaxQ0 - min_q0) /
                     (max_curvature_threshold - min_curvature_threshold) +
                 min_q0;
    } else {
      final_q0 = kMaxQ0;
    }
    matrix_q_(0, 0) = std::min(std::max(final_q0, min_q0), kMaxQ0);
  }
  debug->set_matrix_q_lat(matrix_q_(0, 0));
  debug->set_matrix_q_heading(matrix_q_(2, 2));
}

void LQRController::MRACControl(const canbus::Chassis *chassis,
                                const double steering_position,
                                const double steer_limit,
                                const double steer_diff_with_max_rate,
                                SimpleLateralDebug *debug,
                                double *steer_angle) {
  const auto &vehicle_temp = injector_->vehicle_state();
  const int mrac_model_order =
      control_conf_->lat_controller_conf().steer_mrac_conf().mrac_model_order();
  Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
  steer_state(0, 0) = chassis->steering_percentage();
  if (mrac_model_order > 1) {
    steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
  }

  double adaption_rate = std::fabs(vehicle_temp->linear_velocity()) >
                                 control_conf_->minimum_speed_resolution()
                             ? 1.0
                             : 0.0;
  mrac_controller_.SetStateAdaptionRate(adaption_rate);
  mrac_controller_.SetInputAdaptionRate(adaption_rate);

  *steer_angle = mrac_controller_.Control(
      *steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
  // Set the steer mrac debug message
  MracDebug *mracdebug = debug->mutable_steer_mrac_debug();
  Matrix steer_reference = mrac_controller_.CurrentReferenceState();
  mracdebug->set_mrac_model_order(mrac_model_order);
  for (int i = 0; i < mrac_model_order; ++i) {
    mracdebug->add_mrac_reference_state(steer_reference(i, 0));
    mracdebug->add_mrac_state_error(steer_state(i, 0) - steer_reference(i, 0));
    mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
        mrac_controller_.CurrentStateAdaptionGain()(i, 0));
  }
  mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
      mrac_controller_.CurrentInputAdaptionGain()(0, 0));
  mracdebug->set_mrac_reference_saturation_status(
      mrac_controller_.ReferenceSaturationStatus());
  mracdebug->set_mrac_control_saturation_status(
      mrac_controller_.ControlSaturationStatus());
}

void LQRController::SteeringAngleCalculate(const canbus::Chassis *chassis,
                                           SimpleLateralDebug *debug,
                                           ControlCommand *const cmd) {
  const auto &vehicle_temp = injector_->vehicle_state();
  TrajectoryPoint target_point;
  target_point = trajectory_analyzer_.QueryNearestPointByAbsoluteTime(
      Clock::NowInSeconds() + query_relative_time_);
  static bool isStraightWay = false;
  if (std::abs(target_point.path_point().kappa() < 0.003) &&
      injector_->vehicle_state()->linear_velocity() > KturndecreaseSpd) {
    matrix_k_(0, 0) = std::min(matrix_k_(0, 0), kMaxKPAtStright);
    matrix_k_(0, 1) = std::min(matrix_k_(0, 1), kMax_l_rate_AtStright);
    matrix_k_(0, 2) = std::min(matrix_k_(0, 2), kMaxKHeadingAtStright);
    matrix_k_(0, 3) = std::min(matrix_k_(0, 3), kMaxHeading_rateAtStright);
    isStraightWay = true;
  } else {
    isStraightWay = false;
  }

const double steer_angle_feedback =
    -(matrix_k_ * matrix_state_)(0, 0) * 180 / M_PI * steer_ratio_ ;

const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());
double steer_angle = 0.0;
double steer_angle_feedback_augment = 0.0;

steer_angle = steer_angle_feedback + steer_angle_feedforward +
              steer_angle_feedback_augment;

double temp_velocity =
    std::max(std::abs(vehicle_temp->linear_velocity()), kStopSpeed);
// Compute the steering command limit with the given maximum lateral
// acceleration
const double steer_limit =
    (FLAGS_set_steer_limit &&
     vehicle_temp->linear_velocity() > steer_limit_velocity_threshold_)
        ? std::asin(max_lat_acc_ * wheelbase_ /
                    (temp_velocity * temp_velocity)) *
              steer_ratio_ * 180 / M_PI
        : kMaxWheelAngle;

 double steer_diff_with_max_rate =
    (FLAGS_enable_maximum_steer_rate_limit && lanefollow_flag_)
        ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI
        : kMaxWheelAngle;

if (debug->heading_error() <
        control_conf_->lat_controller_conf().little_heading_error_threshold() &&
    debug->lateral_error() <
        control_conf_->lat_controller_conf().little_l_error_threshold() &&
    isStraightWay) {
  steer_diff_with_max_rate =
      std::min(steer_diff_with_max_rate,
               control_conf_->lat_controller_conf().maxsteer_ratio_limit() *
                   ts_ * 180 / M_PI);
}
const double steering_position = chassis->steering_percentage();

// Re-compute the steering command if the MRAC control is enabled, with steer
// angle limitation and steer rate limitation
if (enable_mrac_) {
  MRACControl(chassis, steering_position, steer_limit, steer_diff_with_max_rate,
              debug, &steer_angle);
}
pre_steering_position_ = steering_position;
debug->set_steer_mrac_enable_status(enable_mrac_);

// Clamp the steer angle with steer limitations at current speed
double steer_angle_limited =
    common::math::Clamp(steer_angle, -steer_limit, steer_limit);
steer_angle = steer_angle_limited;
debug->set_steer_angle_limited(steer_angle_limited);

// Limit the steering command with the designed digital filter
steer_angle = digital_filter_.Filter(steer_angle);
double detection_curvature =
    control_conf_->lat_controller_conf().detection_curvature();
double lateral_error_threshold_straightaway =
    control_conf_->lat_controller_conf().lateral_error_threshold_straightaway();
steer_angle = steer_angle + steering_compensation_;
bool enable_straightaway_filter =
    FLAGS_enable_secondary_filter &&
    std::abs(debug->curvature()) < detection_curvature &&
    (std::abs(debug->lateral_error()) < lateral_error_threshold_straightaway) &&
    (canbus::Chassis::COMPLETE_AUTO_DRIVE == chassis->driving_mode());

if (enable_straightaway_filter) {
  steer_angle = steering_straightaway_filter_.Update(steer_angle);
} else {
  steering_straightaway_filter_.ResetData();
}
steer_angle = common::math::Clamp(steer_angle, -kMaxWheelAngle, kMaxWheelAngle);

if (FLAGS_reverse_heading_control &&
    canbus::Chassis::GEAR_REVERSE == injector_->vehicle_state()->gear() &&
    target_tracking_trajectory_.is_backward_trajectory()) {
  steer_angle = -steer_angle;
}
auto steer_angle_1axis = steer_angle;
auto steer_angle_4axis = -steer_angle;
if (FLAGS_enable_centripetal_acceleration_limited &&
    vehicle_temp->linear_velocity() > steer_limit_velocity_threshold_) {
  double steer_angle_limit_rate =
      std::atan(max_centripetal_acceleration_rate_ * wheelbase_ /
                (temp_velocity * temp_velocity)) *
      steer_ratio_ * 180 / M_PI;
  steer_angle_1axis = common::math::Clamp(
      steer_angle_1axis, pre_steer_angle_1axis_ - steer_angle_limit_rate,
      pre_steer_angle_1axis_ + steer_angle_limit_rate);
  steer_angle_4axis = common::math::Clamp(
      steer_angle_4axis, pre_steer_angle_4axis_ - steer_angle_limit_rate,
      pre_steer_angle_4axis_ + steer_angle_limit_rate);
  debug->set_steer_angle_limit_rate(steer_angle_limit_rate);
}
auto steering_temp_1axis = common::math::Clamp(
    steer_angle_1axis, pre_steer_angle_1axis_ - steer_diff_with_max_rate,
    pre_steer_angle_1axis_ + steer_diff_with_max_rate);
auto steering_temp_4axis = common::math::Clamp(
    steer_angle_4axis, pre_steer_angle_4axis_ - steer_diff_with_max_rate,
    pre_steer_angle_4axis_ + steer_diff_with_max_rate);

cmd->set_steering_target(steering_temp_1axis);
// front wheel
cmd->set_target_steering_angle_1axis(steering_temp_1axis);
// rear wheel
cmd->set_target_steering_angle_4axis(steering_temp_4axis);
cmd->set_steering_rate(FLAGS_steer_angle_rate);

pre_steer_angle_1axis_ = cmd->target_steering_angle_1axis();
pre_steer_angle_4axis_ = cmd->target_steering_angle_4axis();

// compute extra information for logging and debugging
const double steer_angle_lateral_contribution =
    -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_;

const double steer_angle_lateral_rate_contribution =
    -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_;

const double steer_angle_heading_contribution =
    -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_;

const double steer_angle_heading_rate_contribution =
    -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_;

static double timeStart_lqr = Time::Now().ToSecond();
double time_lqr = Time::Now().ToSecond();
time_lqr = time_lqr - timeStart_lqr;
// AINFO << "time_lqr = " << time_lqr;
// AINFO << "k_heading = " << matrix_k_(0, 2);
// AINFO << "k_heading_rate = " << matrix_state_(3, 0);
// AINFO << "k_l_error = " << matrix_k_(0, 0);
// AINFO << "lateral_error = " << matrix_state_(0, 0);
// AINFO << "k_l_rate = " << matrix_k_(0, 1);
// AINFO << "heading_error = " << matrix_state_(2, 0);
// AINFO << "heading_contribution = " << steer_angle_heading_contribution;
// AINFO << "l_c_" << steer_angle_lateral_contribution;
debug->set_heading(driving_orientation_);
debug->set_steer_angle(steer_angle);
debug->set_steer_angle_feedforward(steer_angle_feedforward);
debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
debug->set_steer_angle_lateral_rate_contribution(
    steer_angle_lateral_rate_contribution);
debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
debug->set_steer_angle_heading_rate_contribution(
    steer_angle_heading_rate_contribution);
debug->set_steer_angle_feedback(steer_angle_feedback);
debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
debug->set_steering_position(steering_position);
debug->set_ref_speed(vehicle_temp->linear_velocity());
}

void LQRController::ComputeHeadingData(
    const double linear_v, const double heading_error,
    const double lateral_error, const double linear_a, const double angular_v,
    const TrajectoryAnalyzer &trajectory_analyzer,
    const TrajectoryPoint &target_point, SimpleLateralDebug *debug) {
  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  double lookahead_station = 0.0;
  double lookback_station = 0.0;

  // wsh: note for simple not use? preview query_relative_time_ may be ok
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

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback = 0.0;
  if (!need_forward_control_logic_) {
    heading_error_feedback = heading_error;
  } else {
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
        target_point.relative_time() +
        lookahead_station /
            (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
    heading_error_feedback = common::math::NormalizeAngle(
        heading_error + target_point.path_point().theta() -
        lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback = 0.0;
  if (!need_forward_control_logic_) {
    lateral_error_feedback =
        lateral_error - lookback_station * std::sin(heading_error);
  } else {
    lateral_error_feedback =
        lateral_error + lookahead_station * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();
  double angular_v_filter = heading_error_rate_filter_.Update(angular_v);
  if (!need_forward_control_logic_) {
    debug->set_heading_rate(-angular_v_filter);
  } else {
    debug->set_heading_rate(angular_v_filter);
  }
  debug->set_ref_heading_rate(target_point.path_point().kappa() *
                              target_point.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
}

void LQRController::SolveMatrixK() {
  const auto &s_time = Clock::Now();
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) = std::max(
        matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(std::fabs(
                              injector_->vehicle_state()->linear_velocity())),
        kMinQ0);
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(std::fabs(
                              injector_->vehicle_state()->linear_velocity()));

    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  } else {
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  }
  matrix_k_(0, 0) = std::max(std::min(kMaxK0, matrix_k_(0, 0)), 0.0);
  matrix_k_(0, 1) = std::max(std::min(kMaxK0, matrix_k_(0, 1)), 0.0);
  matrix_k_(0, 2) = std::max(std::min(kMaxK2, matrix_k_(0, 2)), 0.0);
  matrix_k_(0, 3) = std::max(std::min(kMaxK2, matrix_k_(0, 3)), 0.0);
  ADEBUG << "matrix_k_(0,2) = " << matrix_k_(0, 2);

  const auto &e_time = Clock::Now();
  const double lqr_time = (e_time - s_time).ToSecond() * 1e3;
  ADEBUG << "lqr time spend: " << lqr_time << " ms.";
}

void LQRController::CalculateErrorSubfunction(
    const double x, const double y, const double theta, const double linear_v,
    const double linear_a, const double angular_v,
    const TrajectoryAnalyzer &trajectory_analyzer,
    const TrajectoryPoint &target_point, const TrajectoryPoint &adc_point,
    const TrajectoryPoint &curvature_preview_point, SimpleLateralDebug *debug,
    ControlCommand *const cmd) {
  const double dx = x - curvature_preview_point.path_point().x();
  const double dy = y - curvature_preview_point.path_point().y();

  const double adc_dx = x - adc_point.path_point().x();
  const double adc_dy = y - adc_point.path_point().y();

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      curvature_preview_point.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      curvature_preview_point.path_point().y());

  const double cos_target_heading =
      std::cos(curvature_preview_point.path_point().theta());
  const double sin_target_heading =
      std::sin(curvature_preview_point.path_point().theta());

  const double cos_adc_heading = std::cos(adc_point.path_point().theta());
  const double sin_adc_heading = std::sin(adc_point.path_point().theta());
  static double previous_lateral_error = 0.0;
  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  lateral_error = lateral_error_filter_.Update(lateral_error);
  double real_lateral_error =
      cos_adc_heading * adc_dy - sin_adc_heading * adc_dx;
  real_lateral_error = real_lateral_error_filter_.Update(real_lateral_error);
  double lateral_error_rate = (lateral_error - previous_lateral_error) / ts_;
  double l_rate_filted = l_rate_filter_.Update(lateral_error_rate);
  previous_lateral_error = lateral_error;
  debug->set_preview_lateral_error(lateral_error);
  debug->set_lateral_error(real_lateral_error);
  debug->set_lateral_error_rate(l_rate_filted);
  debug->set_ref_heading(target_point.path_point().theta());
  double heading_error =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  // auto compensate heading error at stright way
  static int tiny_lateral_error_count_ = 0;
  if (std::abs(heading_error - debug->heading_error()) <
      control_conf_->lat_controller_conf().heading_error_tiny_threshold()) {
    tiny_lateral_error_count_++;
  } else {
    tiny_lateral_error_count_ = 0;
  }
  if (tiny_lateral_error_count_ > KheadingErrorStableCnt) {
    heading_error *=
        control_conf_->lat_controller_conf().heading_error_discount();
  }

  heading_error = heading_error - heading_compensation_;
  heading_error = heading_error_filter_.Update(heading_error);
  debug->set_heading_error(heading_error);
  debug->set_heading_adc_point(
      common::math::NormalizeAngle(adc_point.path_point().theta()));
  debug->set_kappa_adc_point(adc_point.path_point().kappa());
  static double last_curvature = curvature_preview_point.path_point().kappa();

  double temp_velocity = std::max(std::abs(linear_v), kStopSpeed);
  double temp_curve =
      std::min(max_lat_acc_ / (temp_velocity * temp_velocity), kCurveProtected);
  double curvature_maximum = temp_curve;
  double curvature_minimum = -temp_curve;
  if (FLAGS_enable_centripetal_acceleration_limited &&
      linear_v > steer_limit_velocity_threshold_) {
    double curvature_rate =
        max_centripetal_acceleration_rate_ / (temp_velocity * temp_velocity);
    curvature_maximum = std::min(last_curvature + curvature_rate, temp_curve);
    curvature_minimum = std::max(last_curvature - curvature_rate, -temp_curve);
    debug->set_curvature_rate(curvature_rate);
  }
  double kappa = std::max(
      std::min(curvature_preview_point.path_point().kappa(), curvature_maximum),
      curvature_minimum);
  double ref_curve = curve_filter_.Update(kappa);
  debug->set_curvature(ref_curve);
  // There may be a problem if an empty trajectory is received
  last_curvature = ref_curve;

  ComputeHeadingData(linear_v, heading_error, lateral_error, linear_a,
                     angular_v, trajectory_analyzer, target_point, debug);
}

bool LQRController::TrajectoryAnomalyDetection(const size_t traj_size,
                                               ControlCommand *const cmd) {
  static uint32_t count = 0;
  static canbus::Chassis::GearPosition last_gear_location =
      canbus::Chassis::GEAR_NEUTRAL;
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE != chassis_->driving_mode() ||
      Chassis::GEAR_NEUTRAL == chassis_->gear_location() ||
      Chassis::GEAR_PARKING == chassis_->gear_location() ||
      last_gear_location != chassis_->gear_location()) {
    AERROR << "Data reset";
    ResetData();
  }
  last_gear_location = chassis_->gear_location();
  enable_use_path_data_ = false;
  bool need_diagonal = target_tracking_trajectory_.has_need_diagonal() &&
                       target_tracking_trajectory_.need_diagonal();
  if (FLAGS_enable_fill_trajectory_point &&
      century::planning::ADCTrajectory::LANEFOLLOW ==
          target_tracking_trajectory_.trajectory_scenario() &&
      !need_diagonal && need_forward_control_logic_) {
    if (path_data_size_ >= kLessTrajPointNum) {
      if (traj_size >= kLessTrajPointNum) {
        auto start_trajectory_point =
            target_tracking_trajectory_.trajectory_point().begin();
        auto end_trajectory_point =
            target_tracking_trajectory_.trajectory_point().rbegin();

        if ((!start_trajectory_point->has_path_point() ||
             !start_trajectory_point->path_point().has_s() ||
             !end_trajectory_point->has_path_point() ||
             !end_trajectory_point->path_point().has_s())) {
          enable_use_path_data_ = true;

        } else {
          if ((std::abs(end_trajectory_point->path_point().s() -
                        start_trajectory_point->path_point().s()) <
               kMinTrajectoryLength)) {
            enable_use_path_data_ = true;
          }
        }
      } else {
        enable_use_path_data_ = true;
      }
    }

  } else {
    enable_use_path_data_ = false;
  }  
  if ((traj_size < kLessTrajPointNum && !enable_use_path_data_) ||
      (path_data_size_ < kLessTrajPointNum && enable_use_path_data_)) {
    count++;
    cmd->set_steering_target(pre_steer_angle_1axis_);
    // front wheel
    cmd->set_target_steering_angle_1axis(pre_steer_angle_1axis_);
    // rear wheel
    cmd->set_target_steering_angle_4axis(pre_steer_angle_4axis_);
    cmd->set_steering_rate(FLAGS_steer_angle_rate);
    if (lanefollow_flag_ || count > kMaxLessTrajPointCount) {
      pre_steer_angle_1axis_ = pre_steer_angle_1axis_ * kSteerReduction;
      pre_steer_angle_4axis_ = pre_steer_angle_4axis_ * kSteerReduction;
    }
    AERROR << "lat_controller : traj is empty, use origin steer angle";
    return false;
  } else {
    count = 0;
  }
  return true;
}

bool LQRController::InputAnomalyDetection(
    const century::common::VehicleStateProvider *vehicle_state,
    const canbus::Chassis *chassis,
    const century::planning::ADCTrajectory *trajectory_message,
    const TrajectoryPoint &target_point, const TrajectoryPoint &adc_point) {
  if (!chassis->has_gear_location() || !chassis->has_driving_mode() ||
      !trajectory_message->has_trajectory_scenario() ||
      !trajectory_message->has_is_replan() || !trajectory_message->has_gear()) {
    AERROR << "Chassis or trajectory data missing";
    return false;
  }
  if (!LocalizationAnomalyDetection(vehicle_state, chassis,
                                    trajectory_message)) {
    return false;
  }
  if (!TrajectoryPointAnomalyDetection(target_point) ||
      !TrajectoryPointAnomalyDetection(adc_point)) {
    return false;
  }
  if (!TrajectoryDataAnomalyDetection(trajectory_message)) {
    return false;
  }

  return true;
}
// If the output is false, an exception is detected
bool LQRController::TrajectoryPointAnomalyDetection(
    const TrajectoryPoint &target_point) {
  if (target_point.has_path_point() && target_point.has_a() &&
      target_point.has_v() && target_point.has_relative_time()) {
    if (target_point.path_point().has_x() &&
        target_point.path_point().has_y() &&
        target_point.path_point().has_theta() &&
        target_point.path_point().has_s()) {
      bool trajectory_check_success =
          !CheckDataValidity(target_point.path_point().x()) &&
          !CheckDataValidity(target_point.path_point().y()) &&
          !CheckDataValidity(target_point.path_point().theta()) &&
          !CheckDataValidity(target_point.path_point().s()) &&
          !CheckDataValidity(target_point.a()) &&
          !CheckDataValidity(target_point.v());
      if (!trajectory_check_success) {
        AERROR << "The track point data is invalid(nan),return false";
        return false;
      }
    } else {
      AERROR << "Trace point missing data(nan),return false";
      return false;
    }

  } else {
    AERROR
        << "The trajectory is missing the trajectory point(nan),return false";
    return false;
  }
  return true;
}

bool LQRController::LocalizationAnomalyDetection(
    const century::common::VehicleStateProvider *vehicle_state,
    const canbus::Chassis *chassis,
    const century::planning::ADCTrajectory *trajectory_message) {
  if (CheckDataValidity(vehicle_state->x()) ||
      CheckDataValidity(vehicle_state->y()) ||
      CheckDataValidity(vehicle_state->heading()) ||
      CheckDataValidity(vehicle_state->linear_velocity()) ||
      CheckDataValidity(vehicle_state->angular_velocity()) ||
      CheckDataValidity(vehicle_state->linear_acceleration()) ||
      CheckDataValidity(vehicle_state->steering_percentage()) ||
      CheckDataValidity(vehicle_state->kappa()) ||
      CheckDataValidity(static_cast<uint8_t>(chassis->gear_location())) ||
      CheckDataValidity(static_cast<uint8_t>(chassis->driving_mode())) ||
      CheckDataValidity(
          static_cast<uint8_t>(trajectory_message->trajectory_scenario())) ||
      CheckDataValidity(trajectory_message->is_replan()) ||
      CheckDataValidity(static_cast<uint8_t>(trajectory_message->gear()))) {
    AERROR << "Locate input outliers(inf),return false";
    return false;
  }
  return true;
}

// Data abnormal returns true
bool LQRController::CheckDataValidity(const double data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}
bool LQRController::CheckDataValidity(const uint8_t data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}
bool LQRController::CheckDataValidity(const bool data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}

// Anomaly detection is performed on the time, length and coordinates of the
// trajectory
bool LQRController::TrajectoryDataAnomalyDetection(
    const century::planning::ADCTrajectory *trajectory_message) {
  double min_threshold_abnormal_data_detection =
      control_conf_->lat_controller_conf()
          .min_threshold_abnormal_data_detection_lat();
  if (trajectory_message->trajectory_point().size() >= kLessTrajPointNum) {
    auto start_trajectory_point =
        trajectory_message->trajectory_point().begin();
    auto end_trajectory_point = trajectory_message->trajectory_point().rbegin();

    if (start_trajectory_point->has_relative_time() &&
        end_trajectory_point->has_relative_time() &&
        start_trajectory_point->has_path_point() &&
        start_trajectory_point->path_point().has_s() &&
        end_trajectory_point->has_path_point() &&
        end_trajectory_point->path_point().has_s()) {
      if (!FLAGS_enable_fill_trajectory_point ||
          path_data_size_ < kLessTrajPointNum || !enable_use_path_data_) {
        if (std::abs(end_trajectory_point->relative_time() -
                     start_trajectory_point->relative_time()) <
                min_threshold_abnormal_data_detection ||
            std::abs(end_trajectory_point->path_point().s() -
                     start_trajectory_point->path_point().s()) <
                min_threshold_abnormal_data_detection) {
          AERROR
              << "The time or distance between the start and end of the track "
                 "is too close, and the data is abnormal";
          return false;
        }
      }
    } else {
      AERROR << "The trajectory is missing the time or s,return false";
      return false;
    }
  } else {
    if (!enable_use_path_data_) {
      AERROR << "Track points too little.return false";
      return false;
    }
  }

  return true;
}

void LQRController::ResetData() {
  digital_filter_.ResetData();
  lateral_error_filter_.ResetData();
  real_lateral_error_filter_.ResetData();
  l_rate_filter_.ResetData();

  heading_error_filter_.ResetData();
  heading_error_rate_filter_.ResetData();
  curve_filter_.ResetData();
  steering_straightaway_filter_.ResetData();
  steering_curve_filter_.ResetData();
  pre_steer_angle_1axis_ = 0.0;
  pre_steer_angle_4axis_ = 0.0;
  pre_steering_position_ = 0.0;
}

bool LQRController::CalculateReachEndS(
    const size_t traj_size,
    const google::protobuf::RepeatedPtrField<century::common::PathPoint>
        path_data,
    ControlCommand *const cmd) {
  if (traj_size <= kLessTrajPointNum && path_data_size_ <= kLessTrajPointNum) {
    return true;
  }
  if (!enable_use_path_data_ && traj_size >= kLessTrajPointNum) {
    const auto &end_point =
        target_tracking_trajectory_.trajectory_point(traj_size - 1)
            .path_point();
    double dx = injector_->vehicle_state()->x() - end_point.x();
    double dy = injector_->vehicle_state()->y() - end_point.y();
    double s = std::hypot(dx, dy);
    ADEBUG << "s " << s;
    if (s < kReachEndS &&
        std::fabs(injector_->vehicle_state()->linear_velocity()) < kStopSpeed) {
      cmd->set_steering_target(pre_steer_angle_1axis_);
      // front wheel
      cmd->set_target_steering_angle_1axis(pre_steer_angle_1axis_);
      // rear wheel
      cmd->set_target_steering_angle_4axis(pre_steer_angle_4axis_);
      cmd->set_steering_rate(FLAGS_steer_angle_rate);
      pre_steer_angle_1axis_ = pre_steer_angle_1axis_ * kSteerReduction;
      pre_steer_angle_4axis_ = pre_steer_angle_4axis_ * kSteerReduction;
      return true;
    }
  }
  return false;
}

bool LQRController::DiagonalModeCompute(
      const canbus::Chassis *chassis,
      const planning::ADCTrajectory *planning_published_trajectory,
      ControlCommand *const cmd, SimpleLateralDebug *debug) {
  bool need_diagonal = target_tracking_trajectory_.has_need_diagonal() &&
                       target_tracking_trajectory_.need_diagonal();
  if (need_diagonal && lanefollow_flag_ &&
      canbus::Chassis::COMPLETE_AUTO_DRIVE == chassis->driving_mode() &&
      planning_published_trajectory->has_diagonal_heading()) {
    cmd->set_guide1_zhuanxiang_mode(STEERINGMODE_DIAGONAL);
    double lateral_error_kp =
        control_conf_->lat_controller_conf().diagonal_lateral_error_kp();
    double diagonal_heading_error_kp =
        control_conf_->lat_controller_conf().diagonal_heading_error_kp();
    double lateral_error_compensate =
        -debug->preview_lateral_error() * lateral_error_kp;
    double steering_target =
        (-debug->heading_error() + lateral_error_compensate) * 180 / M_PI *
        steer_ratio_;
    double vehicle_heading = injector_->vehicle_state()->heading();
    double diagonal_heading_error = common::math::NormalizeAngle(
        vehicle_heading - planning_published_trajectory->diagonal_heading());

    diagonal_heading_error = (diagonal_heading_error - heading_compensation_) *
                             180 / M_PI * steer_ratio_;
    double diagonal_heading_error_compensate =
        -diagonal_heading_error * diagonal_heading_error_kp;
    debug->set_diagonal_heading_error(diagonal_heading_error);
    debug->set_diagonal_heading_error_compensate(
        diagonal_heading_error_compensate);

    debug->set_diagonal_lateral_error(debug->preview_lateral_error());
    debug->set_diagonal_lateral_error_compensate(lateral_error_compensate);

    // AERROR << "preview_lateral_error: " << debug->preview_lateral_error()
    //        << " lateral_error_compensate: " << lateral_error_compensate
    //        << " diagonal_heading_error: " << diagonal_heading_error
    //        << " diagonal_heading_error_compensate: "
    //        << diagonal_heading_error_compensate
    //        << " heading_error: " << debug->heading_error()
    //        << " steering_target: " << steering_target;
    steering_target =
        common::math::Clamp(steering_target, -kMaxWheelAngle, kMaxWheelAngle);

    diagonal_heading_error_compensate = common::math::Clamp(
        diagonal_heading_error_compensate, -kMaxDiagonalCompensateWheelAngle,
        kMaxDiagonalCompensateWheelAngle);
    //  calculate steeringratio limitation
    const double steer_diff_with_max_rate =
        (FLAGS_enable_maximum_steer_rate_limit && lanefollow_flag_)
            ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI
            : kMaxWheelAngle;

    if (canbus::Chassis::GEAR_DRIVE == injector_->vehicle_state()->gear()) {
      cmd->set_steering_target(steering_target);
      auto steer_angle_1axis =
          cmd->steering_target() + diagonal_heading_error_compensate;
      auto steer_angle_4axis = cmd->steering_target();

      auto steering_temp_1axis = common::math::Clamp(
          steer_angle_1axis, pre_steer_angle_1axis_ - steer_diff_with_max_rate,
          pre_steer_angle_1axis_ + steer_diff_with_max_rate);
      auto steering_temp_4axis = common::math::Clamp(
          steer_angle_4axis, pre_steer_angle_4axis_ - steer_diff_with_max_rate,
          pre_steer_angle_4axis_ + steer_diff_with_max_rate);
      cmd->set_target_steering_angle_1axis(steering_temp_1axis);
      // rear wheel
      cmd->set_target_steering_angle_4axis(steering_temp_4axis);
      pre_steer_angle_1axis_ = cmd->target_steering_angle_1axis();
      pre_steer_angle_4axis_ = cmd->target_steering_angle_4axis();
      cmd->set_steering_rate(FLAGS_steer_angle_rate);
    } else if (FLAGS_reverse_heading_control &&
               canbus::Chassis::GEAR_REVERSE ==
                   injector_->vehicle_state()->gear() &&
               target_tracking_trajectory_.is_backward_trajectory()) {
      cmd->set_steering_target(steering_target);
      auto steer_angle_1axis = cmd->steering_target();
      auto steer_angle_4axis =
          cmd->steering_target() + diagonal_heading_error_compensate;

      auto steering_temp_1axis = common::math::Clamp(
          steer_angle_1axis, pre_steer_angle_1axis_ - steer_diff_with_max_rate,
          pre_steer_angle_1axis_ + steer_diff_with_max_rate);
      auto steering_temp_4axis = common::math::Clamp(
          steer_angle_4axis, pre_steer_angle_4axis_ - steer_diff_with_max_rate,
          pre_steer_angle_4axis_ + steer_diff_with_max_rate);
      cmd->set_target_steering_angle_1axis(steering_temp_1axis);
      // rear wheel
      cmd->set_target_steering_angle_4axis(steering_temp_4axis);
      pre_steer_angle_1axis_ = cmd->target_steering_angle_1axis();
      pre_steer_angle_4axis_ = cmd->target_steering_angle_4axis();
      cmd->set_steering_rate(FLAGS_steer_angle_rate);

    } else {
      cmd->set_steering_target(-steering_target);
      auto steer_angle_1axis = cmd->steering_target();
      auto steer_angle_4axis =
          cmd->steering_target() - diagonal_heading_error_compensate;

      auto steering_temp_1axis = common::math::Clamp(
          steer_angle_1axis, pre_steer_angle_1axis_ - steer_diff_with_max_rate,
          pre_steer_angle_1axis_ + steer_diff_with_max_rate);
      auto steering_temp_4axis = common::math::Clamp(
          steer_angle_4axis, pre_steer_angle_4axis_ - steer_diff_with_max_rate,
          pre_steer_angle_4axis_ + steer_diff_with_max_rate);
      cmd->set_target_steering_angle_1axis(steering_temp_1axis);
      // rear wheel
      cmd->set_target_steering_angle_4axis(steering_temp_4axis);
      pre_steer_angle_1axis_ = cmd->target_steering_angle_1axis();
      pre_steer_angle_4axis_ = cmd->target_steering_angle_4axis();
      cmd->set_steering_rate(FLAGS_steer_angle_rate);
    }
    return true;
  }
  return false;
}

}  // namespace control
}  // namespace century
