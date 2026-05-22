#include "amcl_worker/amcl_pose_safety_checker.h"

#include <algorithm>
#include <sstream>

namespace semantic_mapping {

namespace {

double SafeSqrt(double value) {
  return std::sqrt(std::max(0.0, value));
}

}  // namespace

AmclPoseSafetyChecker::AmclPoseSafetyChecker() : config_() {}

AmclPoseSafetyChecker::AmclPoseSafetyChecker(const Config& config)
    : config_(config) {}

void AmclPoseSafetyChecker::Reset() {
  has_last_pose_ = false;
  last_pose_.setZero();
  last_timestamp_sec_ = 0.0;
}

AmclPoseSafetyChecker::Result AmclPoseSafetyChecker::Evaluate(
    const Eigen::Vector3d& current_pose,
    double timestamp_sec,
    double abs_speed_mps,
    const Eigen::Vector3d& covariance_diag) {
  Result result;
  result.abs_speed_mps = abs_speed_mps;
  result.sigma_translation_m =
      SafeSqrt(covariance_diag.x() + covariance_diag.y());

  if (!has_last_pose_) {
    has_last_pose_ = true;
    last_pose_ = current_pose;
    last_timestamp_sec_ = timestamp_sec;
    result.has_reference = false;
    result.reason = "first accepted pose";
    return result;
  }

  result.has_reference = true;
  result.dt_sec = timestamp_sec - last_timestamp_sec_;
  if (result.dt_sec < config_.min_dt_sec) {
    result.accepted = false;
    result.status = Status::kRejectedTimestamp;
    std::ostringstream oss;
    oss << "invalid dt " << result.dt_sec << " sec";
    result.reason = oss.str();
    return result;
  }

  const double dx = current_pose.x() - last_pose_.x();
  const double dy = current_pose.y() - last_pose_.y();
  result.translation_jump_m = std::hypot(dx, dy);
  result.yaw_jump_rad = std::fabs(NormalizeAngle(current_pose.z() - last_pose_.z()));

  result.translation_limit_m =
      std::fabs(abs_speed_mps) * result.dt_sec +
      config_.base_translation_margin_m +
      config_.covariance_translation_scale * result.sigma_translation_m;
  result.yaw_limit_rad =
      config_.max_yaw_rate_rad_per_sec * result.dt_sec +
      config_.base_yaw_margin_rad;

  const bool translation_ok =
      result.translation_jump_m <= result.translation_limit_m;
  const bool yaw_ok = result.yaw_jump_rad <= result.yaw_limit_rad;
  if (translation_ok && yaw_ok) {
    last_pose_ = current_pose;
    last_timestamp_sec_ = timestamp_sec;
    result.reason = "jump within limits";
    return result;
  }

  result.accepted = false;
  result.status = Status::kRejectedJump;
  result.severe =
      (result.translation_limit_m > 1e-6 &&
       result.translation_jump_m >
           config_.severe_jump_ratio * result.translation_limit_m) ||
      (result.yaw_limit_rad > 1e-6 &&
       result.yaw_jump_rad > config_.severe_jump_ratio * result.yaw_limit_rad);

  std::ostringstream oss;
  if (!translation_ok) {
    oss << "translation jump " << result.translation_jump_m << " > "
        << result.translation_limit_m;
  }
  if (!yaw_ok) {
    if (!translation_ok) {
      oss << ", ";
    }
    oss << "yaw jump " << result.yaw_jump_rad << " > "
        << result.yaw_limit_rad;
  }
  result.reason = oss.str();
  return result;
}

double AmclPoseSafetyChecker::NormalizeAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

}  // namespace semantic_mapping
