/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/common/virtual_prediction/prediction_virtual_obstacles_component.h"

#include "Eigen/Dense"

#include "modules/common/virtual_prediction/proto/prediction_virtual_obstacles.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "modules/common/util/message_util.h"

using century::cyber::Rate;
using century::cyber::Time;

namespace century {
namespace prediction {
using century::cyber::Clock;
using century::localization::LocalizationEstimate;

// seconds
constexpr int kTotalTime = 5;
constexpr double kDistanceThreshold = 0.5;

class VirtualObstacle {
 public:
  VirtualObstacle(/* args */);
  VirtualObstacle(
      const PredictionVirtualObstaclesConf& prediction_conf,
      const LocalizationReaderPtr& localization_estimate_reader,
      const std::shared_ptr<PredictionObstacles>& prediction_obstacles)
      : obstacle_conf_(prediction_conf),
        localization_estimate_reader_(localization_estimate_reader),
        prediction_obstacles_(prediction_obstacles) {}
  ~VirtualObstacle() {}

  void Init() {
    current_pos_ = obstacle_conf_.start_point();
    theta_ = GetTheta();
    CalculateVelocity();
  }

  void StartPrediction() noexcept {
    if (!is_trigger_ && !obstacle_conf_.is_static()) {
      is_trigger_ = CheckCondition();
    } else {
      pred_obstacle_ = PredictionObstacle();
      pred_obstacle_.set_is_static(obstacle_conf_.is_static());
      pred_obstacle_.mutable_priority()->set_priority(ObstaclePriority::NORMAL);
      auto perception_obstacle = CreatePredictionObstacle();
      prediction_obstacles_->add_prediction_obstacle()->CopyFrom(
          perception_obstacle);
    }
  }

  void Update(const double time_elapsed) {
    if (!is_trigger_) {
      return;
    }

    if (obstacle_conf_.is_static()) {
      return;
    }

    current_pos_.set_x(current_pos_.x() + velocity_.x() * time_elapsed);
    current_pos_.set_y(current_pos_.y() + velocity_.y() * time_elapsed);
    current_pos_.set_z(current_pos_.z() + velocity_.z() * time_elapsed);

    double distance =
        std::pow(obstacle_conf_.end_point().x() - current_pos_.x(), 2) +
        std::pow(obstacle_conf_.end_point().y() - current_pos_.y(), 2);

    if (distance <= kDistanceThreshold * kDistanceThreshold) {
      current_pos_ = obstacle_conf_.start_point();
      RemovePredictionObstacle(obstacle_conf_.id());
      AINFO << "Reach The End.";
      is_trigger_ = false;
    } else {
      is_trigger_ = true;
    }
  }

 private:
  void RemovePredictionObstacle(int obstacle_id) {
    auto it = std::remove_if(
        prediction_obstacles_->mutable_prediction_obstacle()->begin(),
        prediction_obstacles_->mutable_prediction_obstacle()->end(),
        [obstacle_id](const PredictionObstacle& obstacle) {
          return obstacle.perception_obstacle().id() == obstacle_id;
        });

    prediction_obstacles_->mutable_prediction_obstacle()->erase(
        it, prediction_obstacles_->mutable_prediction_obstacle()->end());
    AINFO << "Removed obstacle with ID: " << obstacle_id;
  }

  bool CheckCondition() {
    localization_estimate_reader_->Observe();
    auto localization_estimate =
        localization_estimate_reader_->GetLatestObserved();

    if (!localization_estimate) {
      return false;
    }
    auto vehicle_pose = localization_estimate->pose();

    double distance = std::sqrt(
        std::pow(vehicle_pose.position().x() - obstacle_conf_.start_point().x(),
                 2) +
        std::pow(vehicle_pose.position().y() - obstacle_conf_.start_point().y(),
                 2));

    if (kDistanceThreshold >=
        std::fabs(distance - obstacle_conf_.distance_condition())) {
      AINFO << "Trigg condition: " << distance << ", "
            << obstacle_conf_.distance_condition() << ", pose: ("
            << vehicle_pose.position().x() << ", "
            << vehicle_pose.position().y() << ")";
      return true;
    }

    return false;
  }

  PredictionObstacle CreatePredictionObstacle() noexcept {
    auto perception_obstacle = pred_obstacle_.mutable_perception_obstacle();
    CreatePerceptionObstacle(obstacle_conf_.id(), current_pos_, velocity_,
                             obstacle_conf_.bbox_size(), theta_,
                             perception_obstacle);
    if (obstacle_conf_.is_static()) {
      AddStaticArea(obstacle_conf_.id(), prediction_obstacles_);
      return pred_obstacle_;
    }

    auto trajectory = pred_obstacle_.add_trajectory();
    trajectory->set_probability(0.9);
    AddTrajectoryPoint(trajectory, theta_, current_pos_, velocity_);

    return pred_obstacle_;
  }

  void CreatePerceptionObstacle(
      int id, const common::Point3D& position, const common::Point3D& velocity,
      const common::Point3D& size, double theta,
      perception::PerceptionObstacle* perception_obstacle) {
    perception_obstacle->set_id(id);
    perception_obstacle->set_type(obstacle_conf_.obstacle_type());
    perception_obstacle->set_tracking_time(kTotalTime);
    perception_obstacle->set_theta(theta);

    auto set_xyz_value = [&](auto& obj, double x, double y, double z) {
      obj->set_x(x);
      obj->set_y(y);
      obj->set_z(z);
    };

    auto pos = perception_obstacle->mutable_position();
    set_xyz_value(pos, position.x(), position.y(), position.z());

    auto vel = perception_obstacle->mutable_velocity();
    set_xyz_value(vel, velocity.x(), velocity.y(), velocity.z());

    perception_obstacle->set_length(size.x());
    perception_obstacle->set_width(size.y());
    perception_obstacle->set_height(size.z());

    perception_obstacle->set_confidence_type(
        perception::PerceptionObstacle::CONFIDENCE_CNN);
    perception_obstacle->set_timestamp(Time::Now().ToSecond());

    AddObstacleCornerPoints(perception_obstacle, position, size, theta);
    perception_obstacle->set_confidence(0.9);
    auto acceleration = perception_obstacle->mutable_acceleration();
    set_xyz_value(acceleration, 0.0, 0.0, 0.0);

    auto anchor_point = perception_obstacle->mutable_anchor_point();
    set_xyz_value(anchor_point, position.x(), position.y(), position.z());

    SetBoundingBox2D(perception_obstacle, position, size, theta);

    perception_obstacle->set_height_above_ground(0.0);
    SetObstacleCovariances(perception_obstacle);
  }

  void AddObstacleCornerPoints(
      perception::PerceptionObstacle* perception_obstacle,
      const common::Point3D& position, const common::Point3D& size,
      double theta) noexcept {
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double half_length = size.x() * 0.5;
    double half_width = size.y() * 0.5;

    auto add_point = [&](double x, double y) {
      auto point = perception_obstacle->add_polygon_point();
      point->set_x(position.x() + (x * cos_theta - y * sin_theta));
      point->set_y(position.y() + (x * sin_theta + y * cos_theta));
      point->set_z(0.0);
    };

    add_point(-half_length, half_width);
    add_point(half_length, half_width);
    add_point(half_length, -half_width);
    add_point(-half_length, -half_width);
  }

  void SetBoundingBox2D(perception::PerceptionObstacle* perception_obstacle,
                        const common::Point3D& position,
                        const common::Point3D& size, double theta) {
    auto bbox2d = perception_obstacle->mutable_bbox2d();
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double half_length = size.x() * 0.5;
    double half_width = size.y() * 0.5;

    std::vector<common::Point3D> corners(4);
    corners[0].set_x(-half_length);
    corners[0].set_y(-half_width);
    corners[1].set_x(half_length);
    corners[1].set_y(-half_width);
    corners[2].set_x(half_length);
    corners[2].set_y(half_width);
    corners[3].set_x(-half_length);
    corners[3].set_y(half_width);

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& corner : corners) {
      double rotated_x =
          corner.x() * cos_theta - corner.y() * sin_theta + position.x();
      double rotated_y =
          corner.x() * sin_theta + corner.y() * cos_theta + position.y();

      min_x = std::min(min_x, rotated_x);
      min_y = std::min(min_y, rotated_y);
      max_x = std::max(max_x, rotated_x);
      max_y = std::max(max_y, rotated_y);
    }

    bbox2d->set_xmin(min_x);
    bbox2d->set_ymin(min_y);
    bbox2d->set_xmax(max_x);
    bbox2d->set_ymax(max_y);
  }

  void AddTrajectoryPoint(Trajectory* trajectory, double theta,
                          const common::Point3D& position,
                          const common::Point3D& velocity) {
    // auto kVel = velocity.x();
    constexpr auto kStep = 0.1;
    constexpr auto kTotalSteps = kTotalTime / kStep;
    const auto line_velocity = obstacle_conf_.velocity();
    for (size_t i = 0; i < kTotalSteps; i++) {
      double current_time = i * kStep;
      auto point = trajectory->add_trajectory_point();
      point->set_v(line_velocity);
      point->set_a(0.0);
      point->set_relative_time(current_time);
      point->set_da(0.0);
      point->set_steer(0.0);

      auto path_point = point->mutable_path_point();
      path_point->set_x(position.x() + velocity.x() * current_time);
      path_point->set_y(position.y() + velocity.y() * current_time);
      path_point->set_z(position.z());
      path_point->set_theta(theta);
      path_point->set_kappa(0.0);

      auto s = current_time * line_velocity;

      path_point->set_s(s);
      path_point->set_dkappa(0.0);
      path_point->set_ddkappa(0.0);
      path_point->set_lane_id("0");
      path_point->set_x_derivative(velocity.x());
      path_point->set_y_derivative(velocity.y());
    }
  }

  void SetObstacleCovariances(
      perception::PerceptionObstacle* perception_obstacle) {
    const std::vector<double> default_covariance = {1.0, 0.0, 0.0, 0.0, 1.0,
                                                    0.0, 0.0, 0.0, 1.0};

    auto set_covariance =
        [&default_covariance](
            google::protobuf::RepeatedField<double>* covariance) {
          covariance->Clear();
          for (const auto& value : default_covariance) {
            covariance->Add(value);
          }
        };

    set_covariance(perception_obstacle->mutable_position_covariance());
    set_covariance(perception_obstacle->mutable_velocity_covariance());
    set_covariance(perception_obstacle->mutable_acceleration_covariance());
  }

  void AddStaticArea(
      int id,
      const std::shared_ptr<PredictionObstacles>& prediction_obstacles) {
    perception::PerceptionStacticArea static_area;
    static_area.set_id(id);
    static_area.set_static_area_type(
        perception::PerceptionStacticArea::SA_UNKNOWN);
    prediction_obstacles->add_perception_static_areas()->CopyFrom(static_area);
  }

  double GetTheta() noexcept {
    // Calculate theta angle
    auto start_pt = obstacle_conf_.start_point();
    auto end_pt = obstacle_conf_.end_point();
    double dx = end_pt.x() - start_pt.x();
    double dy = end_pt.y() - start_pt.y();

    if (0 == dx) {
      if (0 < dy) {
        return M_PI * 0.5;
      } else if (0 > dy) {
        return -M_PI * 0.5;
      } else {
        return 0.0;
      }
    }
    auto theta = std::atan2(dy, dx);
    return theta;
  }

  void CalculateVelocity() noexcept {
    double velocity = obstacle_conf_.velocity();
    velocity_.set_x(velocity * std::cos(theta_));
    velocity_.set_y(velocity * std::sin(theta_));
    velocity_.set_z(0.0);
  }

  // common::Point3D CalculateAcc() noexcept {
  //   common::Point3D accVec;
  //   double acc = obstacle_conf_.acc();
  //   accVec.set_x(acc * std::cos(theta_));
  //   accVec.set_y(acc * std::sin(theta_));
  //   accVec.set_z(0.0);

  //   return accVec;
  // }

 private:
  bool is_trigger_{false};
  double theta_{0.0};
  PredictionObstacle pred_obstacle_;
  PredictionVirtualObstaclesConf obstacle_conf_;
  common::Point3D current_pos_;
  common::Point3D velocity_;
  LocalizationReaderPtr localization_estimate_reader_;
  std::shared_ptr<PredictionObstacles> prediction_obstacles_;
};

class VirtualObstacleManager {
 public:
  VirtualObstacleManager() = default;
  VirtualObstacleManager(const NodePtr& node,
                         const PredictionVirtualObstaclesVec& obstacles_conf)
      : node_(node), prediction_virtual_obstacles_(obstacles_conf) {}
  ~VirtualObstacleManager() {}

  void Init() noexcept {
    prediction_obstacles_ = std::make_shared<PredictionObstacles>();
    prediction_writer_ = node_->CreateWriter<PredictionObstacles>(
        prediction_virtual_obstacles_.prediction_topic());

    localization_estimate_reader_ = node_->CreateReader<LocalizationEstimate>(
        prediction_virtual_obstacles_.localization_topic());

    for (const auto& obstacle :
         prediction_virtual_obstacles_.mock_obstacles()) {
      auto virtual_obstacle = std::make_shared<VirtualObstacle>(
          obstacle, localization_estimate_reader_, prediction_obstacles_);
      virtual_obstacle->Init();
      prediction_obstacles_map_[obstacle.id()] = virtual_obstacle;
    }
  }

  void StartSimulation(const double time_elapsed) {
    SetupPredictionObstacles();
    StartPrediction();
    PublishPrediction();
    Update(time_elapsed);
  }

 private:
  void SetupPredictionObstacles() {
    prediction_obstacles_->clear_prediction_obstacle();
    prediction_obstacles_->clear_perception_static_areas();

    auto frame_start_time = Clock::NowInSeconds();
    prediction_obstacles_->set_start_timestamp(frame_start_time);
    prediction_obstacles_->set_end_timestamp(frame_start_time + kTotalTime);

    auto header = prediction_obstacles_->mutable_header();
    auto now = Time::Now().ToNanosecond();
    header->set_lidar_timestamp(now);
    header->set_camera_timestamp(now);
    header->set_radar_timestamp(now);

    prediction_obstacles_->set_perception_error_code(common::ErrorCode::OK);
  }

  void StartPrediction() noexcept {
    for (const auto& item : prediction_obstacles_map_) {
      item.second->StartPrediction();
    }
  }

  void Update(const double time_elapsed) noexcept {
    for (const auto& item : prediction_obstacles_map_) {
      item.second->Update(time_elapsed);
    }
  }

  void PublishPrediction() {
    common::util::FillHeader(node_->Name(), &(*prediction_obstacles_));
    prediction_obstacles_->mutable_header()->set_module_name(
        "prediction module");
    prediction_writer_->Write(*prediction_obstacles_);
  }

 private:
  NodePtr node_;
  PredictionWriterPtr prediction_writer_;
  PredictionVirtualObstaclesVec prediction_virtual_obstacles_;
  LocalizationReaderPtr localization_estimate_reader_;
  std::shared_ptr<PredictionObstacles> prediction_obstacles_;
  std::unordered_map<int, std::shared_ptr<VirtualObstacle>>
      prediction_obstacles_map_;
};

PredictionVirtualObstaclesComponent::~PredictionVirtualObstaclesComponent() {
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}

bool PredictionVirtualObstaclesComponent::Init() {
  AINFO << "Start to init virtual obstacle detection component.";
  task_thread_ = std::thread(std::bind(
      &PredictionVirtualObstaclesComponent::PublishObstaclesTask, this));

  return true;
}

void PredictionVirtualObstaclesComponent::PublishObstaclesTask() noexcept {
  PredictionVirtualObstaclesVec mock_data;
  if (!ComponentBase::GetProtoConfig(&mock_data)) {
    AERROR << "Unable to load prediction conf file: "
           << ComponentBase::ConfigFilePath();
    return;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << mock_data.ShortDebugString();

  VirtualObstacleManager obstacle_manager(node_, mock_data);
  obstacle_manager.Init();

  constexpr double kOutputRate = 10.0;
  Rate rate(kOutputRate);
  auto duration = rate.ExpectedCycleTime().ToSecond();

  while (century::cyber::OK()) {
    obstacle_manager.StartSimulation(duration);
    rate.Sleep();
  }
}

}  // namespace prediction
}  // namespace century