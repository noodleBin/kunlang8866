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
#include "modules/control/controller/lon_controller.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace century {
namespace control {

using century::common::ErrorCode;
using century::common::PathPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::VehicleStateProvider;
using century::cyber::Time;

#define UB_MAX 255u
#define Brake_maxmin 250
#define PARK_BRAKE 0.1
#define M_PI 3.14159265358979323846 /* pi */
#define SOFT_BRAKE 75
#define EMERGENCY_BRAKE 150
#define RAD2DEG 57.2957795130823208768

constexpr double kMaxHeadingErrorThresholdPreview = M_PI / 4;
constexpr double kMaxHeadingErrorThresholdCurrent = M_PI / 4;
constexpr int kLessTrajPointNum = 3;
constexpr double GRA_ACC = 9.8;
constexpr double KmaxInteplate = 9999.0;
constexpr double KWrongNum = -1.0;
constexpr double KmassChangeThres = 200.0;
constexpr double KmassOrain = 23000.0;
constexpr double kMinSpeedErrorDebug = 0.10;
constexpr double kMinTimestampError = 0.06;
constexpr double kEpsilon = 1e-6;
constexpr double kMaxSpeedCompensationThreshold = 1.2;
constexpr double kMinSpeedCompensationThreshold = 0.8;
constexpr double kSpeedErrorThres = 5.0;
constexpr double kStandstillSpd = 0.05;
constexpr double kmaxEmergencyBrake = 125;
constexpr double kMaxStationError = 2.0;
constexpr double kMinBrake = 50.0;
constexpr double KSysTime = 0.02;
constexpr double KMassBase = 20000;
constexpr double kcoef_stand = 1.0322;
constexpr double KRadiusHalfLoad = 0.7033;
constexpr double KRadiusBase = 0.682;
constexpr double KStiffness_tire = 2169625;  // m/coef
constexpr double KorignCompenSpd = 1.0320;
constexpr double KLateralVelocityThres = 0.30;
constexpr uint16_t KLocalLateralCntMax = 10;
constexpr uint16_t KLocalPoseCntMax = 5;
constexpr uint16_t KMaxIntNum = 254;
constexpr int32_t Torque_control_mode = 1;
constexpr int32_t Speed_control_mode = 2;
constexpr double KMaxPoseDiffThres = 0.3;
constexpr double kDecelerationAtPoorLoc = 0.5;
constexpr double kSpeedAtPoorLocThres = 1.5;
constexpr double kWheelAngleErrorThreshold = 5.0;
constexpr double kVelocityAlmostZero = 0.12;
constexpr double kSpeedCheckGain = 1.5;
constexpr double kAEBcharacterNum = 88.0;
constexpr double kArrivalDis = 0.2;
constexpr double kGearSwitchSpeed = 3 / 3.6;

LonController::LonController()
    : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
    speed_log_file_ = fopen(name_buffer, "w");
    if (nullptr == speed_log_file_) {
      AERROR << "Fail to open file:" << name_buffer;
      FLAGS_enable_csv_debug = false;
    }
    if (nullptr != speed_log_file_) {
      fprintf(speed_log_file_,
              "station_reference,"
              "station_error,"
              "station_error_limited,"
              "preview_station_error,"
              "speed_reference,"
              "speed_error,"
              "speed_error_limited,"
              "preview_speed_reference,"
              "preview_speed_error,"
              "preview_acceleration_reference,"
              "acceleration_cmd_closeloop,"
              "acceleration_cmd,"
              "is_full_stop,"
              "\r\n");

      fflush(speed_log_file_);
    }
  }
}

void LonController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (nullptr != speed_log_file_) {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}
void LonController::Stop() { CloseLogFile(); }

double LonController::UpdateMass(double real_torq, double ego_acce) {
  if (velocity_compensation_coefficient_ < 0.1) {
    velocity_compensation_coefficient_ = 1.0;
  }
  double t_Vhicle_mass =
      KMassBase + KStiffness_tire * (kcoef_stand - velocity_compensation_coefficient_);
  t_Vhicle_mass = t_Vhicle_mass > KMassBase ? t_Vhicle_mass : KMassBase;
  return t_Vhicle_mass;
}

void TorqueTarget::CalcAxDevDeadZone(double &axdev) {
  if (std::fabs(axdev) < 0.03) {
    axdev = 0;
  }
}
void TorqueTarget::CalculateTorque_V(
    double TargetAceleration, double axact,
    std::shared_ptr<DependencyInjector> injector,
    const ControlConf *control_conf, double ts_d,
    common::VehicleParam vehicle_param_, const canbus::Chassis *chassis) {
  double t_dev_ax_d = TargetAceleration - axact;
  CalcAxDevDeadZone(t_dev_ax_d);
  double ks = control_conf->lon_controller_conf().torque_ks();
  double kp = control_conf->lon_controller_conf().torque_pid_conf().kp();
  double ki = control_conf->lon_controller_conf().torque_pid_conf().ki();
  double t_MaxTorque_d =
      control_conf->lon_controller_conf().max_torquerequest();
  ////prepare propotion part of
  /// torque///////////////////////////////////////////////////////////////
  uint8_t para_size_ub = 5;
  double t_Torq_Velocity_kp_x_p[para_size_ub] = {0,    1.39, 2.78,
                                                 4.17, 5.56, 6.94};
  double t_Torq_Velocity_kp_y_p[para_size_ub] = {1, 1, 1, 1, 1};
  double t_Torq_deviation_kp_x_p[para_size_ub] = {-1.0, -0.1, 0, 0.1, 1};
  double t_Torq_deviation_kp_y_p[para_size_ub] = {1, 1, 0.2, 1, 1};

  double t_kp_velocity_d =
      interplation(para_size_ub, t_Torq_Velocity_kp_x_p, t_Torq_Velocity_kp_y_p,
                   injector->vehicle_state()->linear_velocity());
  double t_kp_axdev_d = interplation(
      para_size_ub, t_Torq_deviation_kp_x_p, t_Torq_deviation_kp_y_p,
      injector->vehicle_state()->linear_velocity());
  m_kp_final_ = kp * t_kp_velocity_d * t_kp_axdev_d;
  m_acceleration_p_ = m_kp_final_ * t_dev_ax_d;
  ////prepare intergration part of
  /// torque///////////////////////////////////////////////////////////
  double t_Torq_Velocity_ki_x_p[para_size_ub] = {0,    1.39, 2.78,
                                                 4.17, 5.56, 6.94};
  double t_Torq_Velocity_ki_y_p[para_size_ub] = {0.5, 1, 1, 1, 1};
  double t_Torq_deviation_ki_x_p[para_size_ub] = {-0.8, -0.3, 0, 0.3, 0.8};
  double t_Torq_deviation_ki_y_p[para_size_ub] = {0.5, 1, 1, 1, 0.5};

  double t_ki_velocity_d =
      interplation(para_size_ub, t_Torq_Velocity_ki_x_p, t_Torq_Velocity_ki_y_p,
                   injector->vehicle_state()->linear_velocity());
  double t_ki_axdev_d = interplation(
      para_size_ub, t_Torq_deviation_ki_x_p, t_Torq_deviation_ki_y_p,
      injector->vehicle_state()->linear_velocity());
  m_ki_final_ = ki * t_ki_velocity_d * t_ki_axdev_d;

  m_acceleration_i_ = m_ki_final_ * t_dev_ax_d * ts_d + m_acceleration_i_;
  m_acceleration_i_ = std::max(std::min(m_acceleration_i_, 0.5), -1.0);
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE != chassis->driving_mode()) {
    m_acceleration_i_ = 0;
  }
  m_acceleration_s_ = ks * TargetAceleration;
  m_torque_request_ =
      (m_acceleration_s_ + m_acceleration_p_ + m_acceleration_i_) *
      vehicle_param_.mass() * vehicle_param_.wheel_rolling_radius() /
      vehicle_param_.transmission_gain();
  m_torque_request_ =
      std::max(std::min(m_torque_request_, (double)t_MaxTorque_d), 0.0);
}
LonController::~LonController() { CloseLogFile(); }

void LonController::CalcActuator_V(double &TorqueRequest, double &BrakeRequest,
                                   double &speed_cmd,
                                   double AccelerationDeviation) {
  if (AccelerationDeviation >
      control_conf_->lon_controller_conf().accelerate_thres()) {
    m_brake_control_enable_ = false;
    m_engine_control_enable_ = true;
  } else if ((AccelerationDeviation <
              control_conf_->lon_controller_conf().decelerate_thres())) {
    m_brake_control_enable_ = true;
    m_engine_control_enable_ = false;
  } else {
  }

  if ((trajectory_message_->gear() != chassis_->gear_location())) {
    m_brake_control_enable_ = true;
    m_engine_control_enable_ = false;
    BrakeRequest = (double)PARK_BRAKE;
  }

  if (!m_engine_control_enable_) {
    TorqueRequest = 0.0;
  }

  if (!m_brake_control_enable_) {
    BrakeRequest = 0.0;
  }

  AINFO << "m_brake_control_enable_: " << m_brake_control_enable_;
  AINFO << "m_engine_control_enable_: " << m_engine_control_enable_;
}

Status LonController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  m_time_start_ = Time::Now().ToSecond();
  control_conf_ = control_conf;
  if (nullptr == control_conf_) {
    controller_initialized_ = false;
    AERROR << "get_longitudinal_param() nullptr";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  injector_ = injector;
  velocity_compensation_coefficient_ = KorignCompenSpd;
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  double ts = lon_controller_conf.ts();
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
  station_pid_speed_controller_.Init(
      lon_controller_conf.station_pid_for_spd_ctrl_conf());
  torque_pid_controller_.Init(lon_controller_conf.reverse_torque_pid_conf());

  double cutoff_freq_speed =
      control_conf_->lon_controller_conf().speed_filter().cutoff_freq();

  double cutoff_freq_chassis_spd =
      control_conf_->lon_controller_conf().chassis_spd_filter().cutoff_freq();
  double cutoff_freq_local_spd =
      control_conf_->lon_controller_conf().local_spd_filter().cutoff_freq();
  double cutoff_freq_local_comp =
      control_conf_->lon_controller_conf().vel_compen_filter().cutoff_freq();
  SetDigitalFilter(ts, cutoff_freq_chassis_spd, &chassis_spd_lpfilter_);
  SetDigitalFilter(ts, cutoff_freq_local_spd, &local_spd_lpfilter_);
  SetDigitalFilter(ts, cutoff_freq_speed, &speed_lpfilter_);
  SetDigitalFilter(ts, cutoff_freq_local_comp, &vel_compen_lpfilter_);

  m_vehicle_mass_ = KMassBase;

  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_controller_conf.reverse_station_leadlag_conf(), ts);
    speed_leadlag_controller_.Init(
        lon_controller_conf.reverse_speed_leadlag_conf(), ts);
  }
  vehicle_param_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

  controller_initialized_ = true;

  BrakeTableInit();

  return Status::OK();
}

void LonController::BrakeTableInit() {
  /// brake calibration
  const auto &lon_brake_cali =
      control_conf_->lon_controller_conf().lon_brake_accel_table();
  const auto &lon_brake_velocity =
      control_conf_->lon_controller_conf().lon_brake_velocity_gain();

  for (const auto &brake_acce : lon_brake_cali.brake_acce()) {
    m_deceleration_table_.push_back(brake_acce.deceleration());
    m_open_cmd_table_.push_back(brake_acce.open_deg());
  }

  for (const auto &brake_spd : lon_brake_velocity.brake_spd()) {
    m_velocity_table_.push_back(brake_spd.velocity());
    m_spd_gain_table_.push_back(brake_spd.gain());
  }
}

double LonController::ComputeBrakeCommand(double Axdeviation,
                                          double acceleration_cmd, auto debug,
                                          double mass) {
  double gain_velocity = NearestInterpolated(
      m_velocity_table_, m_spd_gain_table_, m_lon_control_input_.m_vehicle_speed_);
  if (acceleration_cmd > 0.1) {
    return 0.0;
  }
  double brakecmd_feedfoward = NearestInterpolated(
      m_deceleration_table_, m_open_cmd_table_, acceleration_cmd);

  double mass_gain = mass / KMassBase;
  brakecmd_feedfoward *= gain_velocity;
  brakecmd_feedfoward *= mass_gain;

  debug->set_gain_velocity(gain_velocity);
  brake_pid_controller_.SetPID(
      control_conf_->lon_controller_conf().brake_pid_conf());

  if (gain_velocity > KmaxInteplate || brakecmd_feedfoward > KmaxInteplate) {
    return KWrongNum;
  }

  return brakecmd_feedfoward;
}

double LonController::CalcSpeedControl(
    SimpleLongitudinalDebug *debug, control::ControlCommand *cmd,
    const planning::ADCTrajectory *trajectory_message) {
  double t_time_d = Time::Now().ToSecond();
  TrajectoryPoint t_StationTar =
      trajectory_analyzer_.get()->QueryNearestPointByAbsoluteTime(t_time_d);
  PathPoint t_StationAct = trajectory_analyzer_.get()->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());

  double t_StationError_d = t_StationTar.path_point().s() - t_StationAct.s();
  t_StationError_d = std::min(t_StationError_d, kMaxStationError);
  if (canbus::Chassis::GEAR_REVERSE == trajectory_message->gear()) {
    if (!trajectory_message_->is_backward_trajectory()) {
      t_StationError_d = -t_StationError_d;
    }
  } else if (canbus::Chassis::GEAR_DRIVE == trajectory_message->gear()) {
    if (trajectory_message_->is_backward_trajectory()) {
      t_StationError_d = -t_StationError_d;
    }
  }
  debug->set_station_error(t_StationError_d);
  double t_SpeedControlOffset_d = station_pid_speed_controller_.Control(
      t_StationError_d, control_conf_->lon_controller_conf().ts());
  if (std::fabs(t_StationTar.v()) <
      control_conf_->lon_controller_conf().speed_standstill() &&
      std::abs(traj_length_front_total_) < kArrivalDis) {
    t_SpeedControlOffset_d = 0.0;
  }

  if (injector_->vehicle_state()->waiting_angle_flag()) {
    t_SpeedControlOffset_d = 0.0;
  }
  debug->set_speed_feedback(t_SpeedControlOffset_d);
  return t_SpeedControlOffset_d;
}

double LonController::NearestInterpolated(
    const std::vector<double> &input_array,
    const std::vector<double> &target_array, const double &input) {
  size_t input_num = input_array.size();
  if (input_num != target_array.size()) {
    return 1e+6;
  }
  if (input >= input_array.back()) {
    return target_array.back();
  } else if (input <= input_array.front()) {
    return target_array.front();
  } else {
    double interpolate_value = 0;
    size_t i = 0;
    while (input_array[i] < input) {
      ++i;
    }
    const double dist_base_return_index = input_array[i] - input_array[i - 1];
    const double dist_to_forward = input_array[i] - input;
    const double dist_to_backward = input - input_array[i - 1];
    interpolate_value = (dist_to_backward * target_array[i] +
                         dist_to_forward * target_array[i - 1]) /
                        dist_base_return_index;
    return interpolate_value;
  }
  return 1e+6;
}

Status LonController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    const planning::AebResult *aeb_result, control::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;
  aeb_result_ = aeb_result;
  AINFO << aeb_result->warning_level();
  trajectory_message_ = planning_published_trajectory;
  if (nullptr == trajectory_analyzer_ || trajectory_analyzer_->seq_num() != trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }
  m_target_gear_ = trajectory_message_->gear();
  AINFO << " trajectory_message_->gear: "<< m_target_gear_;
  f_or_r_direction_reverse_state_ = FRDirectionReverseCheck(chassis_,trajectory_message_);
  const LonControllerConf &lon_controller_conf = control_conf_->lon_controller_conf();
  ////prepare input/////////////////
  double preview_time = lon_controller_conf.preview_window() * KSysTime;
  bool enable_leadlag = lon_controller_conf.enable_reverse_leadlag_compensation();
  if (preview_time < 0.0) {
    const auto error_msg = absl::StrCat("Preview time set as: ", preview_time, " less than 0");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  m_lon_control_input_.m_vehicle_speed_ = injector_->vehicle_state()->linear_velocity() / velocity_compensation_coefficient_;
  m_lon_control_input_.m_vehicle_speed_ = speed_lpfilter_.Filter(m_lon_control_input_.m_vehicle_speed_);
  double t_axact = (m_lon_control_input_.m_vehicle_speed_ - m_lon_control_input_.m_vehicle_speedz1_) / KSysTime;
  double cutoff_freq = lon_controller_conf.acceleration_filter().cutoff_freq();
  SetDigitalFilter(KSysTime, cutoff_freq, &axact_lpfilter_);
  m_lon_control_input_.m_vehicle_accel_ = axact_lpfilter_.Filter(t_axact);
  m_lon_control_input_.m_act_torque_ = chassis_->back_motor_torque() + chassis_->front_motor_torque();
  m_lon_control_input_.m_vehicle_speedz1_ = m_lon_control_input_.m_vehicle_speed_;
  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();
  m_ego_acceleration_ = m_lon_control_input_.m_vehicle_accel_;
  CheckVehicleCalibration(debug, cmd);
  m_vehicle_mass_ = UpdateMass(m_lon_control_input_.m_act_torque_, m_lon_control_input_.m_vehicle_accel_);
  double mass_coef = m_vehicle_mass_ / KMassBase;
  static double m_AEB_spd_cmd = m_lon_control_input_.m_vehicle_speed_;
  if (aeb_result_->has_warning_level() && century::planning::AebWarningLevel::WARNING_LEVEL_NONE != aeb_result_->warning_level()) {
    Status AEB_Status = AEBResponse(aeb_result_,m_AEB_spd_cmd , cmd, mass_coef);
    if (Status::OK() != AEB_Status) {
      AERROR << "AEB enable";
      return AEB_Status;
    }
  } else {
    m_AEB_spd_cmd = m_lon_control_input_.m_vehicle_speed_;
  }
  Status aarch_state = MonitorAarchState(mass_coef,cmd);
  if (Status::OK() != aarch_state) {
    return aarch_state;
  }
  double ts = lon_controller_conf.ts();
  auto safety_status = SafetyCheck(preview_time, cmd);
  if (Status::OK() !=  safety_status) {
    auto error_msg = safety_status.error_message();
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts, debug);
  cmd->set_acceleration(m_lon_control_input_.m_vehicle_accel_);
  debug->set_mass_update(m_vehicle_mass_);
 /**************calculate station and speed PID***** priprotiy speed or station regulation **********************/
  StationSpeedWeightSet(debug);
  double station_error_limit = lon_controller_conf.station_error_limit();
  double station_error_limited = 0.0;
  if (FLAGS_use_preview_speed_for_table) {
    station_error_limited = common::math::Clamp(debug->station_error(), -station_error_limit, station_error_limit);
  } else {
    station_error_limited = common::math::Clamp(debug->station_error(), -station_error_limit, station_error_limit);
  }
  if (canbus::Chassis::GEAR_REVERSE == trajectory_message_->gear()) {
    station_pid_controller_.SetPID(lon_controller_conf.reverse_station_pid_conf());
    speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());
    if (enable_leadlag) {
      station_leadlag_controller_.SetLeadlag(lon_controller_conf.reverse_station_leadlag_conf());
      speed_leadlag_controller_.SetLeadlag(lon_controller_conf.reverse_speed_leadlag_conf());
    }
  } else if (injector_->vehicle_state()->linear_velocity() <= lon_controller_conf.switch_speed()) {
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
  } else {
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
  }
  double speed_offset = station_pid_controller_.Control(station_error_limited, ts);
  if (enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }
  double speed_controller_input = 0.0;
  double speed_controller_input_limit = lon_controller_conf.speed_controller_input_limit();
  double speed_controller_input_limited = 0.0;
  if (FLAGS_enable_speed_station_preview) {
    speed_controller_input = speed_offset + debug->preview_speed_error();
  } else {
    speed_controller_input = speed_offset + debug->speed_error();
  }
  speed_controller_input_limited = common::math::Clamp(speed_controller_input,-speed_controller_input_limit,speed_controller_input_limit);
  double acceleration_cmd_closeloop = 0.0;
  acceleration_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited, ts);
  debug->set_pid_saturation_status(speed_pid_controller_.IntegratorSaturationStatus());
  /**************************** speed control ********************************/
  t_speedcmd_d_ = std::abs(debug->preview_speed_reference());
  double t_SpeedFeedback = CalcSpeedControl(debug, cmd, trajectory_message_);
  debug->set_speed_feedback(t_SpeedFeedback);
  t_speedcmd_d_ = weight_speed_control_ * t_speedcmd_d_ + weight_station_control_ * t_SpeedFeedback;
  StopSpeedSet(debug);
  TurnSpeedSet(preview_time, ts, debug);
  PoorLocationAccuracySpeedSet(localization_);
  t_speedcmd_d_ = std::max(0.0, std::min(t_speedcmd_d_, 10.0));
  debug->set_is_full_stop(false);
  if (trajectory_message_->trajectory_point().size() > 0) {
    GetPathRemain(debug);
  }
  double acceleration_cmd = acceleration_cmd_closeloop + debug->preview_acceleration_reference();
  // At near-stop stage, replace the brake control command with the standstill
  // acceleration if the former is even softer than the latter
  if ((century::planning::ADCTrajectory::NORMAL == trajectory_message_->trajectory_type()) &&
      ((std::fabs(debug->preview_acceleration_reference()) <= control_conf_->max_acceleration_when_stopped() &&
        std::fabs(debug->preview_speed_reference()) <= vehicle_param_.max_abs_speed_when_stopped()) ||
        std::abs(debug->path_remain()) < control_conf_->max_path_remain_when_stopped())) {
    acceleration_cmd = std::min(acceleration_cmd,lon_controller_conf.standstill_acceleration());
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }
  acceleration_cmd = std::min(control_conf_->lon_controller_conf().acceleration_max(),
    std::max(acceleration_cmd,control_conf_->lon_controller_conf().acceleration_min()));
  if (canbus::Chassis::COMPLETE_AUTO_DRIVE != chassis->driving_mode()) {
    Reset();
  }
  Torque_s_p_i_cal(acceleration_cmd, ts, debug);
  AINFO << "AEB warning level: " << aeb_result_->warning_level();
  m_brake_request_ = ComputeBrakeCommand(debug->preview_speed_error(), acceleration_cmd, debug, m_vehicle_mass_) * 0;
  if (KWrongNum == m_brake_request_) {
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Brake interplation error");
  }
  double t_axdev = acceleration_cmd - m_lon_control_input_.m_vehicle_accel_;
  // CalcActuator_V(m_torque_request_, m_brake_request_, t_speedcmd_d_, debug->speed_error()); // torque&brake switch
  GearSwitchSet(chassis_);
  debug->set_acceleration_error(t_axdev);
  double brake_upperbound = lon_controller_conf.brake_max();
  uint32_t t_BrakeRequest = (uint32_t)(std::min(m_brake_request_, brake_upperbound) * Brake_maxmin);
  // TorqueRelatedProcessing(chassis,trajectory_message_,debug);
  double t_target_torque_2axis_cmd = m_torque_request_ / 2;
  double t_target_torque_3axis_cmd = m_torque_request_ / 2;
  debug->set_station_error_limited(station_error_limited);
  debug->set_speed_offset(speed_offset);
  debug->set_speed_controller_input_limited(speed_controller_input_limited);
  debug->set_acceleration_cmd(acceleration_cmd);
  debug->set_brake_cmd(t_BrakeRequest);
  debug->set_ego_acceleration(injector_->vehicle_state()->linear_acceleration());
  debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);
  DebugInfoPrinting(debug);
  VehicleStartSet(chassis, cmd);
  t_speed_k1_ = t_speedcmd_d_;
  debug->set_flag_vehicle_takeoff(flag_vehicle_takeoff_);
  cmd->set_guide1_brake(t_BrakeRequest);
  cmd->set_gear_location(m_target_gear_);
  AINFO << " cmd->set_gear_location: "<< m_target_gear_;
  cmd->set_waiting_steering_flag(flag_vehicle_takeoff_);
  if (TORQUE_CONTROL == control_conf_->lon_controller_conf().control_mode_switch()) {
    cmd->set_target_torque_2axis(t_target_torque_2axis_cmd);
    cmd->set_target_torque_3axis(t_target_torque_3axis_cmd);
  } else if (SPEED_CONTROL == control_conf_->lon_controller_conf().control_mode_switch()) {
    cmd->set_target_torque_2axis(t_speedcmd_d_);
    cmd->set_target_torque_3axis(t_speedcmd_d_);
  } else {
    cmd->set_target_torque_2axis(0);
    cmd->set_target_torque_2axis(0);
  }
  return Status::OK();
}

Status LonController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  station_pid_speed_controller_.Reset();
  return Status::OK();
}

std::string LonController::Name() const { return name_; }

bool LonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
    const double ts, SimpleLongitudinalDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  double t_time_d = Time::Now().ToSecond();
  double lon_speed = m_lon_control_input_.m_vehicle_speed_;
  auto vehicle_state = injector_->vehicle_state();
  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());

  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);
  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(t_time_d);
  TrajectoryPoint preview_point =
      trajectory_analyzer->QueryNearestPointByRelativeTime(
          reference_point.relative_time() + preview_time);
  debug->mutable_current_matched_point()->mutable_path_point()->set_x(
      matched_point.x());
  debug->mutable_current_matched_point()->mutable_path_point()->set_y(
      matched_point.y());
  debug->mutable_current_reference_point()->mutable_path_point()->set_x(
      reference_point.path_point().x());
  debug->mutable_current_reference_point()->mutable_path_point()->set_y(
      reference_point.path_point().y());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
      preview_point.path_point().x());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
      preview_point.path_point().y());
  double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                      matched_point.theta());
  double t_reference_speed = reference_point.v();
  double t_preview_speed = preview_point.v();
  double t_previewed_station_error = preview_point.path_point().s() - s_matched;
  double t_station_error = reference_point.path_point().s() - s_matched;
  traj_length_front_total_ = trajectory_analyzer->TotalLength() - s_matched;  // end path point s
  AINFO << "lon_controller-> traj_length_front_total_: " << traj_length_front_total_;
  t_preview_speed = std::abs(preview_point.v()); 
  lon_speed = std::abs(lon_speed);
  t_reference_speed = std::abs(reference_point.v());
  if (!trajectory_message_->is_backward_trajectory()) {
    if (canbus::Chassis::GEAR_REVERSE == chassis_->gear_location()) {
      t_previewed_station_error = -t_previewed_station_error;
      t_station_error = -t_station_error;
    }
  } else {
    if (canbus::Chassis::GEAR_DRIVE == chassis_->gear_location()) {
      t_previewed_station_error = -t_previewed_station_error;
      t_station_error = -t_station_error;
    }
  }
  double lon_acceleration = m_lon_control_input_.m_vehicle_accel_;
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);
  debug->set_station_reference(reference_point.path_point().s());
  debug->set_current_station(s_matched);
  debug->set_station_error(t_station_error);
  debug->set_speed_reference(reference_point.v());
  debug->set_current_speed(lon_speed);
  debug->set_speed_error(t_reference_speed - lon_speed);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_current_acceleration(lon_acceleration);
  debug->set_traj_length_front_total(traj_length_front_total_);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk =
      (debug->current_acceleration() - previous_acceleration_) / ts;
  debug->set_jerk_reference(jerk_reference);
  debug->set_current_jerk(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->current_acceleration();
  debug->set_preview_station_error(t_previewed_station_error);
  debug->set_preview_speed_error(t_preview_speed - lon_speed);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
  return true;
}

void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}

// TODO(all): Refactor and simplify
void LonController::GetPathRemain(SimpleLongitudinalDebug *debug) {
  double path_remain = 0;
  path_remain = trajectory_message_->trajectory_point().at(
      trajectory_message_->trajectory_point_size() - 1).path_point().s() -
      debug->current_station();
  path_remain = true == f_or_r_direction_reverse_state_ ?
      -path_remain : path_remain;
  debug->set_path_remain(path_remain);
  path_remain_ = path_remain;
}

Status LonController::SafetyCheck(double preview_time,
                                  control::ControlCommand *cmd) {
  double chassis_speed = chassis_->speed_mps();
  double localization_speed = localization_->pose().linear_velocity().x();

  std::string error_msg;

  if ((ControlState::DEFAULT != cmd->control_state())) {
    if (!m_time_check_first_b_) {
      m_decel_failed_check_ =
          std::abs(chassis_speed /
                   control_conf_->lon_controller_conf().time_to_collision());
      m_decel_failed_check_ = std::max(m_decel_failed_check_, 0.1);
      m_time_check_first_b_ = true;
      m_check_fail_spd_tar_ = std::abs(chassis_speed);
    } else if (std::abs(chassis_speed) >= kStandstillSpd) {
      m_check_fail_spd_tar_ -=
          m_decel_failed_check_ * control_conf_->lon_controller_conf().ts();
      m_check_fail_spd_tar_ = std::max(m_check_fail_spd_tar_, 0.0);
    } else {
    }
    double t_delerationAtCheckFail = -std::abs(m_decel_failed_check_);

    double gain_velocity = NearestInterpolated(
        m_velocity_table_, m_spd_gain_table_, m_lon_control_input_.m_vehicle_speed_);
    double brakecmd_t = NearestInterpolated(
        m_deceleration_table_, m_open_cmd_table_, t_delerationAtCheckFail);

    brakecmd_t *= gain_velocity * Brake_maxmin;

    brakecmd_t = std::max(brakecmd_t, kMinBrake);

    cmd->set_target_torque_2axis(m_check_fail_spd_tar_);
    cmd->set_target_torque_3axis(m_check_fail_spd_tar_);
    cmd->set_gear_location(m_target_gear_);
    cmd->set_guide1_brake((u_int32_t)(std::min(brakecmd_t, kmaxEmergencyBrake)));
    AERROR << "input check error, cannot follow trajectory";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "timestamp check error.");
  } else {
    m_time_check_first_b_ = false;
  }

  TrajectoryPoint t_reference_point =
      trajectory_analyzer_.get()->QueryNearestPointByPosition(
          injector_->vehicle_state()->x(), injector_->vehicle_state()->y());
  TrajectoryPoint t_preview_point =
      trajectory_analyzer_.get()->QueryNearestPointByRelativeTime(
          t_reference_point.relative_time() + preview_time);

  PathPoint t_matched_point = trajectory_analyzer_.get()->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());

  static double spd_for_speed_diff_ = m_lon_control_input_.m_vehicle_speed_;
  double mass_coef = m_vehicle_mass_ / KMassBase;
  if (std::fabs(chassis_speed - localization_speed) > kSpeedErrorThres) {
    spd_for_speed_diff_ -=
        control_conf_->lon_controller_conf().brake_decel_l3() * KSysTime;
    spd_for_speed_diff_ = std::max(spd_for_speed_diff_, 0.0);
    u_int32_t brake =
        std::min((u_int32_t)(SOFT_BRAKE * mass_coef), (u_int32_t)Brake_maxmin);
    cmd->set_guide1_brake(brake);
    cmd->set_target_torque_2axis(spd_for_speed_diff_);
    cmd->set_target_torque_3axis(spd_for_speed_diff_);
    error_msg =
        absl::StrCat("speed error between localization and chassis too large");
    AERROR << "chassis_speed: " << chassis_speed
           << "localization_speed: " << localization_speed;
    return Status(ErrorCode::LOCALIZATION_ERROR, error_msg);
  }

  bool inputAnomalFlag = InputAnomalyDetection(
      injector_->vehicle_state(), chassis_, trajectory_message_,
      t_reference_point, t_preview_point, error_msg);
  AINFO << "inputAnomalFlag= " << inputAnomalFlag;
  bool locationAnomalFlag =
      LocationAnomalyCheck(t_reference_point, error_msg, trajectory_message_);

  AINFO << "locationAnomalFlag= " << locationAnomalFlag;
  bool headingAnomalFlag =
      HeadingAnomalyCheck(t_reference_point, t_matched_point, error_msg);
  AINFO << "headingAnomalFlag= " << headingAnomalFlag;
  static double spd_for_safe_check = m_lon_control_input_.m_vehicle_speed_;
  if (!inputAnomalFlag || !locationAnomalFlag || !headingAnomalFlag) {
    spd_for_safe_check -=
        control_conf_->lon_controller_conf().brake_decel_l3() * KSysTime;
    spd_for_safe_check = std::max(spd_for_safe_check, 0.0);
    u_int32_t brake =
        std::min((u_int32_t)(SOFT_BRAKE * mass_coef), (u_int32_t)Brake_maxmin);
    cmd->set_guide1_brake(brake);
    cmd->set_target_torque_2axis(spd_for_safe_check);
    cmd->set_target_torque_3axis(spd_for_safe_check);
    cmd->set_gear_location(m_target_gear_);
    AINFO << "chassis_gear" << chassis_->gear_location();
    AINFO << "target_gear" << m_target_gear_;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  } else {
    spd_for_safe_check = m_lon_control_input_.m_vehicle_speed_;
    return Status::OK();
  }
}

bool LonController::HeadingAnomalyCheck(TrajectoryPoint reference_point,
                                        PathPoint matched_point,
                                        std::string &error_msg) {
  auto ego_heading = injector_->vehicle_state()->heading();
  // if (trajectory_message_->is_backward_trajectory() &&
  //     canbus::Chassis::GEAR_REVERSE != chassis_->gear_location()) {
  //   ego_heading += M_PI;
  // }
  if ((std::abs(common::math::NormalizeAngle(
           ego_heading - reference_point.path_point().theta())) >
       kMaxHeadingErrorThresholdPreview) ||
      (std::abs(
           common::math::NormalizeAngle(ego_heading - matched_point.theta())) >
       kMaxHeadingErrorThresholdCurrent)) {
    error_msg = absl::StrCat("heading error too large");
    return false;
  } else {
    return true;
  }
}

bool LonController::LocationAnomalyCheck(
    TrajectoryPoint reference_point, std::string &error_msg,
    const planning::ADCTrajectory *trajectory_message) {
  double dx =
      reference_point.path_point().x() - injector_->vehicle_state()->x();
  double dy =
      reference_point.path_point().y() - injector_->vehicle_state()->y();
  double wheel_angle = (chassis_->bridge_1_left_wheel_angle() +
                        chassis_->bridge_1_right_wheel_angle()) /
                       2;
  double closest_trajectory_distance_suqare = dx * dx + dy * dy;
  static uint16_t vy_error_cnt = 0;
  double x_loc = localization_->pose().position().x();
  double y_loc = localization_->pose().position().y();
  double heading = localization_->pose().heading();
  static double x_locz1 = 0.0;
  static double y_locz1 = 0.0;
  static uint16_t pos_error_cnt = 0;
  double vy = localization_->pose().linear_velocity().y();
  double vx = localization_->pose().linear_velocity().x();
  double vy_thres = KLateralVelocityThres;
  if (trajectory_message->has_need_diagonal() &&
      trajectory_message->need_diagonal()) {
    vy_thres += std::fabs(vx * std::tan(wheel_angle / RAD2DEG));
  }
  if (std::fabs(vy) > vy_thres) {
    vy_error_cnt++;
    vy_error_cnt = std::min(vy_error_cnt, KMaxIntNum);
  } else {
    vy_error_cnt = 0;
  }
  if (vy_error_cnt > KLocalLateralCntMax) {
    error_msg = absl::StrCat("lateral velocity too large ");
    AERROR << "Safetycheck: lateral velocity too large ";
    return false;
  }

  if (std::fabs(x_loc - x_locz1) >
          std::max(0.05,
                   std::abs(injector_->vehicle_state()->linear_velocity() *
                            std::cos(common::math::NormalizeAngle(heading)) *
                            kSpeedCheckGain)) ||
      std::fabs(y_loc - y_locz1) >
          std::max(std::abs(injector_->vehicle_state()->linear_velocity() *
                            std::sin(common::math::NormalizeAngle(heading)) *
                            kSpeedCheckGain),
                   0.05)) {
    pos_error_cnt++;
    pos_error_cnt = std::min(pos_error_cnt, KMaxIntNum);
    AERROR << "x_loc: " << x_loc;
    AERROR << "x_locz1: " << x_locz1;
    AERROR << "y_loc: " << y_loc;
    AERROR << "y_locz1: " << y_locz1;
    AERROR << "vy_thres: " << vy_thres;
  } else {
    pos_error_cnt = 0;
  }
  if (pos_error_cnt > KLocalPoseCntMax) {
    error_msg = absl::StrCat("localization pose jump too large ");
    AERROR << "Safetycheck: localization pose jump too large ";
    return false;
  }
  x_locz1 = x_loc;
  y_locz1 = y_loc;

  if (closest_trajectory_distance_suqare >
          control_conf_->lon_controller_conf()
              .max_closest_trajectory_distance() ||
      localization::LocalizationEstimate::NORMAL !=
              localization_->status_type() &&
          localization::LocalizationEstimate::POOR_ACCURACY !=
              localization_->status_type()) {
    error_msg =
        absl::StrCat("distance to reference too large or status not nomal");
    AERROR
        << "Safetycheck: distance to reference too large or status not nomal ";
    return false;
  } else
    return true;
}

bool LonController::InputAnomalyDetection(
    const century::common::VehicleStateProvider *vehicle_state,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *trajectory_message,
    const TrajectoryPoint &target_point, const TrajectoryPoint &adc_point,
    std::string &error_msg) {
  if (!chassis->has_gear_location() || !chassis->has_driving_mode() ||
      !trajectory_message->has_trajectory_scenario() ||
      !trajectory_message->has_is_replan() || !trajectory_message->has_gear()) {
    AERROR << "chassis->has_gear_location: " << chassis->has_gear_location();
    AERROR << "chassis->has_driving_mode: " << chassis->has_driving_mode();
    AERROR << "Chassis or trajectory data missing";
    error_msg = absl::StrCat("Chassis or trajectory data missing");
    return false;
  }
  if (!LocalizationAnomalyDetection(vehicle_state, chassis,
                                    trajectory_message)) {
    error_msg = absl::StrCat("Locate input outliers(inf),return false");
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
bool LonController::TrajectoryPointAnomalyDetection(
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

bool LonController::LocalizationAnomalyDetection(
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
bool LonController::CheckDataValidity(const double data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}
bool LonController::CheckDataValidity(const uint8_t data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}
bool LonController::CheckDataValidity(const bool data) {
  if (std::isinf(data) || std::isnan(data)) {
    return true;
  }
  return false;
}

// Anomaly detection is performed on the time, length and coordinates of the
// trajectory
bool LonController::TrajectoryDataAnomalyDetection(
    const century::planning::ADCTrajectory *trajectory_message) {
  double min_threshold_abnormal_data_detection =
      control_conf_->lon_controller_conf()
          .min_threshold_abnormal_data_detection_lon();
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
      if (std::abs(end_trajectory_point->relative_time() -
                   start_trajectory_point->relative_time()) <
              min_threshold_abnormal_data_detection ||
          std::abs(end_trajectory_point->path_point().s() -
                   start_trajectory_point->path_point().s()) <
              min_threshold_abnormal_data_detection) {
        AERROR << "The time or distance between the start and end of the track "
                  "is too close, and the data is abnormal";
        return false;
      }
    } else {
      AERROR << "The trajectory is missing the time or s,return false";
      return false;
    }
  } else {
    AERROR << "Track points too little.return false";
    return false;
  }
  return true;
}
void LonController::CheckVehicleCalibration(SimpleLongitudinalDebug *debug,
                                            ControlCommand *const cmd) {
  double detection_wheel_angle =
      control_conf_->lon_controller_conf().detection_wheel_angle();

  double chassis_wheel_angle = (chassis_->bridge_1_left_wheel_angle() +
                                chassis_->bridge_1_right_wheel_angle()) /
                               2;
  double chassis_speed = chassis_->speed_orign();
  double chassis_spd_filtered = chassis_spd_lpfilter_.Filter(chassis_speed);
  AINFO << "chassis_speed:" << chassis_speed;
  AINFO << " chassis_spd_filtered:" << chassis_spd_filtered;
  double localization_speed = localization_->pose().linear_velocity().x();
  double local_spd_filtered = local_spd_lpfilter_.Filter(localization_speed);
  AINFO << "localization_speed:" << localization_speed;
  AINFO << "local_spd_filtered:" << local_spd_filtered;
  if (std::fabs(local_spd_filtered) > 1.0 &&
      std::fabs(chassis_spd_filtered) > 1.0 &&
      std::abs(m_lon_control_input_.m_vehicle_accel_) < 0.1 &&
      std::abs(chassis_wheel_angle) < detection_wheel_angle) {
    double t_velocity_compensation_coefficient =
        local_spd_filtered / (chassis_spd_filtered + kEpsilon);
    static double velocity_compensation_coefficientk1 =
        t_velocity_compensation_coefficient;
    t_velocity_compensation_coefficient =
        velocity_compensation_coefficientk1 +
        0.01 * (t_velocity_compensation_coefficient -
                velocity_compensation_coefficientk1);
    velocity_compensation_coefficientk1 = t_velocity_compensation_coefficient;
    if (std::abs(t_velocity_compensation_coefficient -
                 velocity_compensation_coefficient_) > 0.001) {
      velocity_compensation_coefficient_ = t_velocity_compensation_coefficient;
    }
  }
  AINFO << "velocity_compensation_coefficient_: "
        << velocity_compensation_coefficient_;

  cmd->set_velocity_compensation_coefficient(
      velocity_compensation_coefficient_);
}

Status LonController::AEBResponse(
    const century::planning::AebResult *aeb_result, double &spdcmd_ori,
    ControlCommand *const cmd, double mass_gain) {
  double AEB_deceleration = 0.0;
  // uint32_t brake = 0;

  switch (aeb_result->warning_level()) {
    case century::planning::AebWarningLevel::WARNING_LEVEL_NONE:
      return Status::OK();
    case century::planning::AebWarningLevel::WARNING_LEVEL_LOW:
      AEB_deceleration =
          control_conf_->lon_controller_conf().aeb_brake_decel_l1();
      break;
    case century::planning::AebWarningLevel::WARNING_LEVEL_MEDIUM:
      AEB_deceleration =
          control_conf_->lon_controller_conf().aeb_brake_decel_l2();
      break;
    case century::planning::AebWarningLevel::WARNING_LEVEL_HIGH:
      AEB_deceleration =
          control_conf_->lon_controller_conf().aeb_brake_decel_l3();
      break;
    default:
      AEB_deceleration = 0.0;
  }
  spdcmd_ori += AEB_deceleration * KSysTime;
  spdcmd_ori = std::max(spdcmd_ori, 0.0);

  double brake = NearestInterpolated(m_deceleration_table_, m_open_cmd_table_,
                                     AEB_deceleration) *
                 mass_gain;
  u_int32_t brake_out = std::min((u_int32_t)brake, (u_int32_t)Brake_maxmin);

  AINFO << "AEB_warning_level: " << aeb_result->warning_level();
  AINFO << "AEB_deceleration: " << AEB_deceleration;
  AINFO << "m_AEB_spd_cmd: " << spdcmd_ori;
  AINFO << "brake: " << brake_out;

  cmd->set_guide1_brake(brake_out);
  cmd->set_gear_location(m_target_gear_);
  cmd->set_target_torque_2axis(spdcmd_ori);
  cmd->set_target_torque_3axis(spdcmd_ori);

  return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "AEB triggered");
}

bool LonController::FRDirectionReverseCheck(
      const canbus::Chassis *chassis,
      const century::planning::ADCTrajectory *trajectory_message) {
  if ((trajectory_message->is_backward_trajectory() && 
      canbus::Chassis::GEAR_DRIVE == chassis_->gear_location()) || 
    (!trajectory_message->is_backward_trajectory() && 
      canbus::Chassis::GEAR_REVERSE == chassis_->gear_location()) ) {
    return true;
  }
  return false;
}

void LonController::TorqueRelatedProcessing(
      const canbus::Chassis *chassis,
      const century::planning::ADCTrajectory *trajectory_message,
      SimpleLongitudinalDebug *debug) {
  ////////////////////////////////gear change/////////////////////////////////////
  /// request/////////////////////////////////////////////////////////////////////
  if ((std::fabs(injector_->vehicle_state()->linear_velocity()) <
       control_conf_->lon_controller_conf().speed_standstill()) &&
      (canbus::Chassis::COMPLETE_AUTO_DRIVE == chassis->driving_mode())) {
    ++m_stand_still_cont_;
    m_stand_still_cont_ = std::min(m_stand_still_cont_, (uint8_t)UB_MAX);
  } else {
    m_stand_still_cont_ = 0;
  }
  if (m_stand_still_cont_ > control_conf_->lon_controller_conf().standstill_thres()) {
    if (canbus::Chassis::GEAR_REVERSE == trajectory_message->gear()) {
      m_target_gear_ = canbus::Chassis::GEAR_REVERSE;
    } else if (canbus::Chassis::GEAR_DRIVE == trajectory_message->gear()) {
      m_target_gear_ = canbus::Chassis::GEAR_DRIVE;
    }
    ++m_gear_change_cont_;
    m_gear_change_cont_ = std::min(m_gear_change_cont_, UB_MAX);
  } else {
    m_gear_change_cont_ = 0;
  }
  if ((m_gear_change_cont_ < control_conf_->lon_controller_conf().gearchangetime()) &&
      (m_gear_change_cont_ > 0) && (trajectory_message->gear() != chassis_->gear_location())) {
    m_torque_request_ = 0.0;
  } else {
  }
  ////////// drive///
  /// off/////////////////////////////////////////////////////////////////////
  if ((m_stand_still_cont_ >
       control_conf_->lon_controller_conf().standstill_thres()) &&
      (debug->preview_speed_reference() >
       control_conf_->lon_controller_conf().speed_driveoff())) {
    m_torque_request_ +=
        control_conf_->lon_controller_conf().torque_driveoff_offset();
    m_torque_request_ =
        std::min(m_torque_request_,
                 control_conf_->lon_controller_conf().max_torquerequest());
  }
}

void LonController::DebugInfoPrinting(SimpleLongitudinalDebug *debug) {
  if (FLAGS_enable_csv_debug && nullptr != speed_log_file_) {
    fprintf(speed_log_file_,
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
            "%.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
            debug->station_reference(), debug->station_error(),
            debug->station_error_limited(), debug->preview_station_error(),
            debug->speed_reference(), debug->speed_error(),
            debug->speed_controller_input_limited(), debug->preview_speed_reference(),
            debug->preview_speed_error(),
            debug->preview_acceleration_reference(), debug->acceleration_cmd_closeloop(),
            debug->acceleration_cmd(), debug->ego_acceleration(),
            debug->speed_feedback(),
            debug->is_full_stop());
  }
}

Status LonController::MonitorAarchState(const double mass_coef,ControlCommand *const cmd) {
  static double spd_for_mon_4 = std::abs(m_lon_control_input_.m_vehicle_speed_);
  if (MonitorState::EMERGENCY_LEVEL_4 == cmd->monitor_aarch_state() &&
      FLAGS_minimum_execution_monitor_level <=
          MonitorState::EMERGENCY_LEVEL_4) {
    spd_for_mon_4 -= control_conf_->lon_controller_conf().brake_decel_l4() * KSysTime;
    spd_for_mon_4 = std::max(spd_for_mon_4, 0.0);
    u_int32_t brake = std::min((u_int32_t)(EMERGENCY_BRAKE * mass_coef),
                               (u_int32_t)Brake_maxmin);
    cmd->set_guide1_brake(brake);
    cmd->set_target_torque_2axis(spd_for_mon_4);
    cmd->set_target_torque_3axis(spd_for_mon_4);
    cmd->set_gear_location(m_target_gear_);
    AERROR << "monitor state == 4 , EMERGENCY_LEVEL_4, emergency brake. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "the monitor fault level is 4.");
  } else {
    spd_for_mon_4 = m_lon_control_input_.m_vehicle_speed_;
  }
  static double spd_for_mon_3 = m_lon_control_input_.m_vehicle_speed_;
  if (MonitorState::EMERGENCY_LEVEL_3 == cmd->monitor_aarch_state() &&
      FLAGS_minimum_execution_monitor_level <=
          MonitorState::EMERGENCY_LEVEL_3) {
    spd_for_mon_3 -= control_conf_->lon_controller_conf().brake_decel_l3() * KSysTime;
    spd_for_mon_3 = std::max(spd_for_mon_3, 0.0);
    u_int32_t brake =
        std::min((u_int32_t)(SOFT_BRAKE * mass_coef), (u_int32_t)Brake_maxmin);
    cmd->set_guide1_brake(brake);
    cmd->set_target_torque_2axis(spd_for_mon_3);
    cmd->set_target_torque_3axis(spd_for_mon_3);
    cmd->set_gear_location(m_target_gear_);
    AERROR << "monitor state == 3 , EMERGENCY_LEVEL_3, smooth brake. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "the monitor fault level is 3.");
  } else {
    spd_for_mon_3 = m_lon_control_input_.m_vehicle_speed_;
  }
  return Status::OK();
}

void LonController::StationSpeedWeightSet(SimpleLongitudinalDebug *debug) {
  bool vehicle_short_traj_follower_flag = false;
  if (0 <= traj_length_front_total_ &&
      traj_length_front_total_ < control_conf_->lon_controller_conf().min_traj_length() &&
      nullptr != trajectory_analyzer_) {
    weight_speed_control_ = 0.5;
    weight_station_control_ = 1.5;
    vehicle_short_traj_follower_flag = true;
  } else {
    weight_speed_control_ = 1.0;
    weight_station_control_ = 1.0;
    vehicle_short_traj_follower_flag = false;
  }
  debug->set_vehicle_short_traj_follower_flag(vehicle_short_traj_follower_flag);
  AINFO << "lon_controller-> vehicle_short_traj_follower_flag: " << vehicle_short_traj_follower_flag;
}

void LonController::Torque_s_p_i_cal(const double acceleration_cmd,const double ts,SimpleLongitudinalDebug *debug) {
  m_torque_target_.CalculateTorque_V(acceleration_cmd, m_lon_control_input_.m_vehicle_accel_,
                                    injector_,control_conf_, ts, vehicle_param_, chassis_);
  m_torque_request_ = m_torque_target_.GetTorqueRequest();
  double t_torque_s = m_torque_target_.GetAcceleration_s() * vehicle_param_.mass() *
                      vehicle_param_.wheel_rolling_radius() / vehicle_param_.transmission_gain();
  double t_torque_p = m_torque_target_.GetAcceleration_p() * vehicle_param_.mass() *
                      vehicle_param_.wheel_rolling_radius() / vehicle_param_.transmission_gain();
  double t_torque_i = m_torque_target_.GetAcceleration_i() * vehicle_param_.mass() *
                      vehicle_param_.wheel_rolling_radius() / vehicle_param_.transmission_gain();
  double t_kpTorqFinlal = m_torque_target_.GetkpFinalTorq_V();
  double t_kiTorqFinlal = m_torque_target_.GetkiFinalTorq_V();
  debug->set_torq_s(t_torque_s);
  debug->set_torq_p(t_torque_p);
  debug->set_torq_i(t_torque_i);
  debug->set_kp_torq(t_kpTorqFinlal);
  debug->set_ki_torq(t_kiTorqFinlal);
}

void LonController::TurnSpeedSet(double preview_time, double ts, SimpleLongitudinalDebug *debug) {
  TrajectoryPoint reference_point = trajectory_analyzer_->QueryNearestPointByAbsoluteTime(Time::Now().ToSecond());
  TrajectoryPoint preview_point = trajectory_analyzer_->QueryNearestPointByRelativeTime(reference_point.relative_time() + preview_time);
  if (std::fabs(preview_point.path_point().kappa()) > control_conf_->lon_controller_conf().kappa_for_turn()) {
    t_speedcmd_d_ = std::min(t_speedcmd_d_, std::abs(debug->preview_speed_reference()));
    double current_radius = std::fabs(1.0 / preview_point.path_point().kappa());
    double speed_thres_by_lat_acc = std::sqrt(current_radius);
    t_speedcmd_d_ = std::min(std::min(t_speedcmd_d_, speed_thres_by_lat_acc),
        control_conf_->lon_controller_conf().acceleration_threshold_for_turn() * ts + t_speed_k1_);
    if (std::abs(m_lon_control_input_.m_vehicle_speed_ < control_conf_->lon_controller_conf().driveoff_speed())) {
      t_speedcmd_d_ = std::min(t_speedcmd_d_,t_speed_k1_ + control_conf_->lon_controller_conf().driveoff_acc() * ts);
    }
  }
}

void LonController::StopSpeedSet(SimpleLongitudinalDebug *debug) {
  if (path_remain_ >= control_conf_->max_path_remain_when_stopped() && 
      path_remain_ <= control_conf_->lon_controller_conf().min_traj_length()) {
    t_speedcmd_d_ = min_response_wheel_speed_;
  } else if (path_remain_ < control_conf_->max_path_remain_when_stopped()) {
    t_speedcmd_d_ = 0.0;
  }
}

void LonController::PoorLocationAccuracySpeedSet(const localization::LocalizationEstimate *localization) {
  if (localization::LocalizationEstimate::POOR_ACCURACY == localization->status_type()) {
    static double speedcmdForPoorLoc = t_speedcmd_d_;
    if (t_speedcmd_d_ > kSpeedAtPoorLocThres) {
      speedcmdForPoorLoc -= KSysTime * kDecelerationAtPoorLoc;
      t_speedcmd_d_ = speedcmdForPoorLoc;
    } else {
      speedcmdForPoorLoc = t_speedcmd_d_;
    }
  }
}

void LonController::VehicleStartSet(const canbus::Chassis *chassis, control::ControlCommand *cmd) {
  double desired_wheel_angle = cmd->target_steering_angle_1axis();
  double actual_wheel_Angle = (chassis->bridge_1_left_wheel_angle() + chassis->bridge_1_right_wheel_angle()) / 2;
  // limit the speed cmd in terms of physical torque limit at the low speed. Just open loop command the minimum speed.
  if (canbus::Chassis::GEAR_PARKING == m_target_gear_prvs_ && 
     (canbus::Chassis::GEAR_DRIVE == m_target_gear_ ||
      canbus::Chassis::GEAR_REVERSE == m_target_gear_)) {
    flag_vehicle_takeoff_ = true;
  } else if (canbus::Chassis::GEAR_PARKING == m_target_gear_) {
    flag_vehicle_takeoff_ = false;
  }
  AINFO << "lon_controller-> flag_vehicle_takeoff_: " << flag_vehicle_takeoff_;
  if (flag_vehicle_takeoff_ && std::abs(chassis->front_drive_wheel_speed()) < min_response_wheel_speed_) {
    t_speedcmd_d_ = std::max(t_speedcmd_d_, min_response_wheel_speed_);
  }else if ( std::abs(chassis->front_drive_wheel_speed()) > min_response_wheel_speed_ + 0.02){
    flag_vehicle_takeoff_ = false;
  }
  m_target_gear_prvs_ =  m_target_gear_;
  // vehicle stop when steeering error too large 
  bool waiting_steering = std::abs(desired_wheel_angle - actual_wheel_Angle) > kWheelAngleErrorThreshold;
  bool condition = false;
  if (waiting_steering && std::abs(chassis->front_drive_wheel_speed()) < kVelocityAlmostZero) {
    condition = true;
  }
  if (condition) {
    t_speedcmd_d_ = 0.0;
    cmd->set_guide1_brake(SOFT_BRAKE);
  }
  injector_->vehicle_state()->set_waiting_angle_flag(condition);
}

void LonController::GearSwitchSet(const canbus::Chassis *chassis) {
  static auto pre_target_gear = m_target_gear_;
  static bool switch_flag = false; 
  auto current_gear = chassis->gear_location();
  if (pre_target_gear != m_target_gear_ || 
      (!switch_flag && current_gear != m_target_gear_)) {
    switch_flag = true;
    pre_target_gear = m_target_gear_;
  }
  if (switch_flag) {
    if (std::abs(chassis->front_drive_wheel_speed()) > kGearSwitchSpeed) {
      t_speedcmd_d_ = 0.1;
    } else {
      if (current_gear != canbus::Chassis::GEAR_NEUTRAL) {
        m_target_gear_ = canbus::Chassis::GEAR_NEUTRAL;
      } else {
        m_target_gear_ = m_target_gear_;
      }
      t_speedcmd_d_ = t_speedcmd_d_;
    }
    if (std::abs(chassis->front_drive_wheel_speed()) < kGearSwitchSpeed &&
        canbus::Chassis::GEAR_NEUTRAL == current_gear) {
      switch_flag = false;
    }
  }
  AINFO << " switch_flag: " << switch_flag 
        << " front_drive_wheel_speed: " << std::abs(chassis->front_drive_wheel_speed())
        << " m_target_gear: " << m_target_gear_;
}

}  // namespace control
}  // namespace century
