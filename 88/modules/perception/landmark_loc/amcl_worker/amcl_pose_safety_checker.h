#pragma once

#include <Eigen/Core>
#include <cmath>
#include <string>

namespace semantic_mapping {

class AmclPoseSafetyChecker {
 public:
  enum class Status {
    kAccepted = 0,
    kRejectedJump,
    kRejectedTimestamp,
  };

  struct Config {
    double min_dt_sec = 0.05;
    double max_dt_sec = 1.0;
    double base_translation_margin_m = 0.5;
    double covariance_translation_scale = 2.5;
    double max_yaw_rate_rad_per_sec = 0.5;
    double base_yaw_margin_rad = 0.1;
    double severe_jump_ratio = 2.5;
  };

  struct Result {
    Status status = Status::kAccepted;
    bool accepted = true;
    bool severe = false;
    bool has_reference = false;
    double dt_sec = 0.0;
    double translation_jump_m = 0.0;
    double translation_limit_m = 0.0;
    double yaw_jump_rad = 0.0;
    double yaw_limit_rad = 0.0;
    double abs_speed_mps = 0.0;
    double sigma_translation_m = 0.0;
    std::string reason;
  };

  AmclPoseSafetyChecker();
  explicit AmclPoseSafetyChecker(const Config& config);

  void Reset();

  Result Evaluate(const Eigen::Vector3d& current_pose,
                  double timestamp_sec,
                  double abs_speed_mps,
                  const Eigen::Vector3d& covariance_diag = Eigen::Vector3d::Zero());

  bool HasLastAcceptedPose() const { return has_last_pose_; }
  const Eigen::Vector3d& LastAcceptedPose() const { return last_pose_; }
  double LastAcceptedTimestamp() const { return last_timestamp_sec_; }

 private:
  static double NormalizeAngle(double angle_rad);

  Config config_;
  bool has_last_pose_ = false;
  Eigen::Vector3d last_pose_ = Eigen::Vector3d::Zero();
  double last_timestamp_sec_ = 0.0;
};

}  // namespace semantic_mapping
