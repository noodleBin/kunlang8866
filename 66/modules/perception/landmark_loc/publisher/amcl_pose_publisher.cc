#include "amcl_pose_publisher.h"
#include <iostream>
#include <iomanip>
#include "glog/logging.h"
#include "common/pose_utils.h"
namespace landmark_loc{
namespace {
Eigen::Vector3d Convert2DPose(const LocalizationPtr& publish_pose) {
  double siny_cosp = 2 * (publish_pose->pose().orientation().qw() * 
                          publish_pose->pose().orientation().qz() + 
                          publish_pose->pose().orientation().qx() * 
                          publish_pose->pose().orientation().qy());                        
  double cosy_cosp = 1 - 2 * (publish_pose->pose().orientation().qy() * 
                              publish_pose->pose().orientation().qy() + 
                              publish_pose->pose().orientation().qz() * 
                              publish_pose->pose().orientation().qz());
  return Eigen::Vector3d(publish_pose->pose().position().x(), publish_pose->pose().position().y(),
                         std::atan2(siny_cosp, cosy_cosp));
}

double NormalizeAngle(double angle) {
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::Vector3d CalcVelocityFromPose( const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double dt) {
  if (dt <= 1e-9) {
      throw std::invalid_argument("dt must be > 0");
  }

  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double dyaw = NormalizeAngle(p2.z() - p1.z());

  // Rotate world-frame displacement into body frame using p1's yaw,
  // because DelayedMeasurementEkf::Propagate expects body-frame velocity.
  const double yaw = p1.z();
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  double vx = ( cos_yaw * dx + sin_yaw * dy) / dt;
  double vy = (-sin_yaw * dx + cos_yaw * dy) / dt;
  double yaw_rate = dyaw / dt;

  return Eigen::Vector3d(vx, vy, yaw_rate);
}
}
void AmclPosePublisher::Init( const LocalizationPtr& init_pose, const double ts,
                              const delayed_ekf::AsyncDelayedMeasurementEkf::Options& config) {
  if(config.use_delayed_ekf) {
    delayed_ekf_.reset(new delayed_ekf::AsyncDelayedMeasurementEkf(config));
    delayed_ekf_->Initialize(ts, Convert2DPose(init_pose));
  }
}

double AmclPosePublisher::Normalize(const double z) {
  return atan2(sin(z),cos(z));
}
  
double AmclPosePublisher::AngleDiff( double a,  double b)
{
  double d1, d2;
  a = Normalize(a);
  b = Normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

Eigen::Vector3d AmclPosePublisher::LocalAddGlobal(const Eigen::Vector3d a, const Eigen::Vector3d b)
{
  Eigen::Vector3d c;
  c[0] = b[0] + a[0] * cos(b[2]) - a[1] * sin(b[2]);
  c[1] = b[1] + a[0] * sin(b[2]) + a[1] * cos(b[2]);
  c[2] = b[2] + a[2];
  c[2] = atan2(sin(c[2]), cos(c[2]));
  
  return c;
}
void AmclPosePublisher::FeedUpAmclPose(const Eigen::VectorXd& amcl_pose, const double ts) {
  if(delayed_ekf_) {
    delayed_ekf_->AddMeasurement(ts, amcl_pose.head<3>(), amcl_pose.tail<3>());
    return;
  }
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(3) << ts;
//  LOG(INFO) <<"feed up amcl ts is "<< timestamp_stream.str();
  std::unique_lock<std::mutex> lock(amcl_pose_mutex_);
  latest_amcl_ts_double_ = ts;
  latest_amcl_ts_ = timestamp_stream.str();
  amcl_pose_[latest_amcl_ts_] = amcl_pose.head<3>();

} 
bool AmclPosePublisher::FetchPredictedPose(const Eigen::Vector3d& dr_pose, const double ts, Eigen::Vector3d& predicted_pose) {
  if(!semantic_mapping::InMap(dr_pose(0), dr_pose(1))) {
    return false;
  }

  if(delayed_ekf_) {
    bool ret = std::abs(pre_dr_pose_ts_) > 1e-6 ;
    if(ret) {
      delayed_ekf_->AddControl(ts, CalcVelocityFromPose(pre_dr_pose_, dr_pose, ts- pre_dr_pose_ts_));
      auto ekf_est = delayed_ekf_->latest_estimate();
      predicted_pose = ekf_est.state;
    }
    pre_dr_pose_ts_ = ts;
    pre_dr_pose_ = dr_pose;
    return ret;
  }
  const double kMaxDelayWarning = 1.0;
  const double kDownWarningThres = 60.0;
  const double delta_ts = ts - latest_amcl_ts_double_;
  if((((delta_ts > kMaxDelayWarning) && (latest_amcl_ts_double_ > 1e-6))) || (check_delay_)){
    std::ostringstream oss;
    oss << std::fixed
        << "PREDICTED ts: " << delta_ts
        << " via dr ts " << ts
        << " less than latest amcl ts " << latest_amcl_ts_double_;

    if (delta_ts > kDownWarningThres) {
      LOG_EVERY_N(INFO,100) << oss.str();
    } else {
      LOG(INFO) << oss.str();
    }
  }
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(3) << ts;
 // LOG(INFO) <<"Insert dr ts "<< timestamp_stream.str();
  dr_pose_[timestamp_stream.str()] = dr_pose;
  Eigen::Vector3d amcl_pose;
  std::string latest_amcl_ts;
  {
    std::unique_lock<std::mutex> lock(amcl_pose_mutex_);
    latest_amcl_ts = latest_amcl_ts_;
    if (dr_pose_.find(latest_amcl_ts_) == dr_pose_.end()) {
    //  LOG(WARNING) << "No dr pose for latest amcl ts " << latest_amcl_ts_;
      return false;
    }


    if (amcl_pose_.find(latest_amcl_ts_) == amcl_pose_.end()) {
    //  LOG(WARNING) << "No amcl pose exist " << latest_amcl_ts_;
      return false;
    }
    amcl_pose = amcl_pose_.at(latest_amcl_ts_);
  }
  
  Eigen::Vector3d delta_pose;
  delta_pose(0) = dr_pose(0) - dr_pose_.at(latest_amcl_ts)(0);
  delta_pose(1) = dr_pose(1) - dr_pose_.at(latest_amcl_ts)(1);
  delta_pose(2) = AngleDiff(dr_pose(2), dr_pose_.at(latest_amcl_ts)(2));
  predicted_pose(0) = delta_pose(0) + amcl_pose(0);
  predicted_pose(1) = delta_pose(1) + amcl_pose(1);
  predicted_pose(2) = delta_pose(2) + amcl_pose(2);

  
  return true;
}

LocalizationPtr AmclPosePublisher::RolloutFuseddAmclPose(const LocalizationPtr& cur_dr, const double ts) {
  LocalizationPtr publish_pose = std::make_shared<century::localization::LocalizationEstimate>();
  publish_pose->mutable_header()->set_timestamp_sec( std::round(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count() / 1e9 * 1.0));
  *publish_pose = *cur_dr;
  double siny_cosp = 2 * (publish_pose->pose().orientation().qw() * 
                          publish_pose->pose().orientation().qz() + 
                          publish_pose->pose().orientation().qx() * 
                          publish_pose->pose().orientation().qy());                        
  double cosy_cosp = 1 - 2 * (publish_pose->pose().orientation().qy() * 
                              publish_pose->pose().orientation().qy() + 
                              publish_pose->pose().orientation().qz() * 
                              publish_pose->pose().orientation().qz());
  Eigen::Vector3d dr_pose( publish_pose->pose().position().x(),  
                           publish_pose->pose().position().y(),
                           std::atan2(siny_cosp, cosy_cosp));
  Eigen::Vector3d predicted_pose;
  if(!FetchPredictedPose(dr_pose,ts,predicted_pose)) {
  //  LOG(INFO) <<"Fail to fetch predicted pose a@ "<< std::fixed << ts;
    return nullptr;
  }
  publish_pose->mutable_pose()->mutable_position()->set_x(predicted_pose(0));
  publish_pose->mutable_pose()->mutable_position()->set_y(predicted_pose(1));

  Eigen::Quaterniond rotation = semantic_mapping::PoseUtils::zyx_euler_to_quat(predicted_pose(2), 0, 0);
  publish_pose->mutable_pose()->mutable_orientation()->set_qw(rotation.w());
  publish_pose->mutable_pose()->mutable_orientation()->set_qx(rotation.x());
  publish_pose->mutable_pose()->mutable_orientation()->set_qy(rotation.y());
  publish_pose->mutable_pose()->mutable_orientation()->set_qz(rotation.z());
  return publish_pose;
}
}