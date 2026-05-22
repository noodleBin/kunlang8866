#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <unordered_map>
#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include "delayed_ekf/async_delayed_measurement_ekf.h"
#include "modules/localization/proto/localization.pb.h"

namespace landmark_loc{
  using namespace century::localization;
  using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
class AmclPosePublisher {
public:
static AmclPosePublisher* GetInstance() {
  static AmclPosePublisher publisher;
  return &publisher;
}
LocalizationPtr RolloutFuseddAmclPose(const LocalizationPtr& cur_dr, const double ts);
void FeedUpAmclPose(const Eigen::VectorXd& amcl_pose, const double ts);
void Init(const LocalizationPtr& init_pose, const double ts, 
          const delayed_ekf::AsyncDelayedMeasurementEkf::Options& config);

private:
  AmclPosePublisher() = default;
  std::string latest_amcl_ts_;
  double latest_amcl_ts_double_;
  std::unordered_map<
    std::string,
    Eigen::Vector3d,
    std::hash<std::string>,
    std::equal_to<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>
  > amcl_pose_;
std::unordered_map<
  std::string,
  Eigen::Vector3d,
  std::hash<std::string>,
  std::equal_to<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>
 > dr_pose_;
  double AngleDiff(double a,  double b);
  double Normalize(const double z);
  Eigen::Vector3d LocalAddGlobal(const Eigen::Vector3d a, const Eigen::Vector3d b);
  bool FetchPredictedPose(const Eigen::Vector3d& dr_pose, const double ts, Eigen::Vector3d& predicted_pose);

  std::mutex amcl_pose_mutex_;
  bool check_delay_ = false;
  Eigen::Vector3d pre_dr_pose_ = Eigen::Vector3d(0, 0, 0);
  double pre_dr_pose_ts_ = 0.0;
  std::shared_ptr<delayed_ekf::AsyncDelayedMeasurementEkf> delayed_ekf_ = nullptr;
};
}
