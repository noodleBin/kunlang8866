/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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

#include "obstacles.h"
#include <iostream>
#include <cmath>

namespace century {
namespace dreamview {

Obstacles::Obstacles()
    : speed_(0.0),
      obstacles_type_(0),
      trigger_type_(0),
      trigger_time_(0.0),
      run_time_all_(0.0) {
  node_ = NodeSingleton::GetNodeInstance();
  localization_reader_ =
  node_->CreateReader<century::localization::LocalizationEstimate>(
      "/century/loc/pose", [this](const std::shared_ptr<
          century::localization::LocalizationEstimate>& msg) {
         vehicle_x_ = msg->pose().position().x();
         vehicle_y_ = msg->pose().position().y();
      });
}

Obstacles::~Obstacles() {}

void Obstacles::createObstacle(const nlohmann::json& obstacle_json) {
  if (!obstacle_json.contains("id") || !obstacle_json.contains("position") ||
      !obstacle_json.contains("heading") || !obstacle_json.contains("length") ||
      !obstacle_json.contains("width") || !obstacle_json.contains("height") ||
      !obstacle_json.contains("trace")) {
    std::cerr << "Error: Missing required fields in JSON." << std::endl;
    return;
  }

  speed_ = obstacle_json["speed"].get<double>();

  perception_obstacle_.set_id(obstacle_json["id"].get<int>());
  const auto& position = obstacle_json["position"];
  perception_obstacle_.mutable_position()->set_x(position[0].get<double>());
  perception_obstacle_.mutable_position()->set_y(position[1].get<double>());
  perception_obstacle_.mutable_position()->set_z(position[2].get<double>());

  auto heading = obstacle_json["heading"].get<double>();
  perception_obstacle_.set_theta(obstacle_json["heading"].get<double>());
  // perception_obstacle_.set_
  auto velocity = perception_obstacle_.mutable_velocity();
  velocity->set_x(speed_ * std::cos(heading));
  velocity->set_y(speed_ * std::sin(heading));
  velocity->set_z(0.0);

  perception_obstacle_.set_length(obstacle_json["length"].get<double>());
  perception_obstacle_.set_width(obstacle_json["width"].get<double>());
  perception_obstacle_.set_height(obstacle_json["height"].get<double>());

  century::perception::PerceptionObstacle_Type type;
  century::perception::PerceptionObstacle_Type_Parse(obstacle_json["type"], &type);
  perception_obstacle_.set_type(type);

  setPolygonPoints(perception_obstacle_.position().x(),
                   perception_obstacle_.position().y(),
                   perception_obstacle_.length(), perception_obstacle_.width(),
                   perception_obstacle_.theta());

  obstacles_type_ = obstacle_json["move_way"] == "move" ? 1 : 0;

  const auto& trace_array = obstacle_json["trace"];
  for (const auto& trace_json : trace_array) {
    Trace trace;
    trace.position.id = trace_json["id"].get<std::string>();
    const auto& trace_position = trace_json["position"];
    trace.position.x = trace_position[0].get<float>();
    trace.position.y = trace_position[1].get<float>();
    trace.position.z = trace_position[2].get<float>();
    trace.speed = trace_json["speed"].get<float>();
    trace_postion.push_back(trace);
  }

  if (obstacle_json.contains("trigger_position") &&
      !obstacle_json["trigger_position"].empty()) {
    trigger_radius_ = obstacle_json["trigger_radius"].get<float>();
    trigger_postion_.x =
        obstacle_json["trigger_position"][0]["position"][0].get<float>();
    trigger_postion_.y =
        obstacle_json["trigger_position"][0]["position"][1].get<float>();
    trigger_postion_.z =
        obstacle_json["trigger_position"][0]["position"][2].get<float>();
    trigger_type_ = 2;
  }
}

bool Obstacles::Trigger(float run_time) {
  run_time_all_ += run_time;
  if (trigger_type_ == 0) {
    return true;
  } else if (trigger_type_ == 1 && run_time_all_ >= trigger_time_) {
    return true;
  } else if (trigger_type_ == 2) {
    return IsCloseToTriggerPosition();
  }
  return false;
}

bool Obstacles::IsCloseToTriggerPosition() {
  if (trgger_) {
    return true;
  }
 //  auto current_position = localization->pose();
  double distance_to_trigger = std::sqrt(
      std::pow(trigger_postion_.x - vehicle_x_, 2) +
      std::pow(trigger_postion_.y - vehicle_y_, 2));
  trgger_ = distance_to_trigger <= trigger_radius_;
  // const double proximity_threshold = 5.0;
  return trgger_;
}

void Obstacles::setPolygonPoints(double center_x, double center_y,
                                 double length, double width, double theta) {
  double half_length = length / 2.0;
  double half_width = width / 2.0;

  std::vector<std::pair<double, double>> corners = {
      {center_x - half_length * cos(theta) - half_width * sin(theta),
       center_y - half_length * sin(theta) + half_width * cos(theta)},
      {center_x + half_length * cos(theta) - half_width * sin(theta),
       center_y + half_length * sin(theta) + half_width * cos(theta)},
      {center_x + half_length * cos(theta) + half_width * sin(theta),
       center_y + half_length * sin(theta) - half_width * cos(theta)},
      {center_x - half_length * cos(theta) + half_width * sin(theta),
       center_y - half_length * sin(theta) - half_width * cos(theta)}};

  perception_obstacle_.clear_polygon_point();
  for (const auto& corner : corners) {
    auto point = perception_obstacle_.add_polygon_point();
    point->set_x(corner.first);
    point->set_y(corner.second);
    point->set_z(0.0);
  }
}

PerceptionObstacle* Obstacles::GetObstacle() {
  if (!Trigger(0.0)) {
    return nullptr;
  }
  return &perception_obstacle_;
}

void Obstacles::UpdateObstacle(float run_time) {
  const auto& current_position = perception_obstacle_.position();
  float current_speed = speed_;
  float delta_time = run_time;
  std::cout << "run_time : " << run_time << std::endl;
  if (trace_postion.empty()) {
    std::cerr << "Error: No trace positions available." << std::endl;
    return;
  }

  Trace& current_trace = trace_postion.front();
  Position& next_position = current_trace.position;

  double new_x = current_position.x();
  double new_y = current_position.y();
  double new_z = current_position.z();

  double distance_to_next = std::sqrt(std::pow(next_position.x - new_x, 2) +
                                      std::pow(next_position.y - new_y, 2));

  double move_distance = current_speed * delta_time;

  if (move_distance >= distance_to_next) {
    new_x = next_position.x;
    new_y = next_position.y;
    trace_postion.erase(trace_postion.begin());
  } else {
    double move_ratio = move_distance / distance_to_next;
    new_x += (next_position.x - new_x) * move_ratio;
    new_y += (next_position.y - new_y) * move_ratio;
  }

  double new_theta =
      std::atan2(next_position.y - new_y, next_position.x - new_x);

  double current_theta = perception_obstacle_.theta();
  double theta_diff = new_theta - current_theta;

  if (theta_diff > M_PI) {
    theta_diff -= 2 * M_PI;
  } else if (theta_diff < -M_PI) {
    theta_diff += 2 * M_PI;
  }

  double smoothing_factor = 0.1;
  auto theta = current_theta + theta_diff * smoothing_factor;
  perception_obstacle_.set_theta(theta);

  perception_obstacle_.mutable_position()->set_x(new_x);
  perception_obstacle_.mutable_position()->set_y(new_y);
  perception_obstacle_.mutable_position()->set_z(new_z);
  setPolygonPoints(new_x, new_y, perception_obstacle_.length(),
                   perception_obstacle_.width(), perception_obstacle_.theta());

  auto velocity = perception_obstacle_.mutable_velocity();
  velocity->set_x(speed_ * std::cos(theta));
  velocity->set_y(speed_ * std::sin(theta));

  float tracking_time = perception_obstacle_.tracking_time();
  perception_obstacle_.set_tracking_time(tracking_time + delta_time);
}

ObstacleManager::ObstacleManager() = default;

ObstacleManager::~ObstacleManager() { ClearObstacle(); }

void ObstacleManager::ClearObstacle() { obstacles_.clear(); }

void ObstacleManager::AddObstacle(std::shared_ptr<Obstacles> obstacle) {
  obstacles_.push_back(obstacle);
}

void ObstacleManager::UpdateAll(float run_time) {
  for (auto& obstacle : obstacles_) {
    if (obstacle->Trigger(run_time)) {
      obstacle->UpdateObstacle(run_time);
    }
  }
}

void ObstacleManager::AddObstacleFromJson(const nlohmann::json& json) {
  auto obstacle = std::make_shared<Obstacles>();
  obstacle->createObstacle(json);
  AddObstacle(obstacle);
}

std::vector<PerceptionObstacle> ObstacleManager::GetObstacles() {
  std::vector<PerceptionObstacle> obstacles_list;
  for (const auto& obstacle : obstacles_) {
    if (obstacle->GetObstacle() != nullptr) {
      obstacles_list.push_back(*obstacle->GetObstacle());
    }
  }
  return obstacles_list;
}

ObstacleTask::~ObstacleTask() { writer_.reset(); }

ObstacleTask::ObstacleTask(const std::string& writer_channel,
                           double update_interval, SimControl* sim_control)
    : update_interval_(update_interval),
      writer_channel_(writer_channel),
      paused_(true),
      sim_control_(sim_control) {
  node_ = NodeSingleton::GetNodeInstance();
  writer_ = node_->CreateWriter<century::perception::PerceptionObstacles>(
      writer_channel_);
      routing_result_reader_ = node_->CreateReader<century::routing::RoutingResult>(
        "/century/routing_result",
        [this](const std::shared_ptr<century::routing::RoutingResult>& result) {
          this->paused_ = !result->routing_result();
        });
}

void ObstacleTask::Init(const nlohmann::json& obstacle_json) {
  manager_.ClearObstacle();
  if (obstacle_json.contains("obstacles") &&
      obstacle_json["obstacles"].is_array()) {
    for (const auto& obstacle : obstacle_json["obstacles"]) {
      manager_.AddObstacleFromJson(obstacle);
    }
  } else {
    std::cerr << "Error: JSON does not contain a valid 'obstacles' array."
              << std::endl;
  }
}

void ObstacleTask::Start() {
  century::cyber::Rate rate(10.0);
  uint64_t seq = 0;

  while (century::cyber::OK()) {
    if (!paused_ && sim_control_->IsEnabled()) {
      UpdateObstaclesAndSend();
      AINFO << "Message sent! No. " << seq;
      seq++;
    }
    rate.Sleep();
  }
}

void ObstacleTask::UpdateObstaclesAndSend() {
  auto msg = std::make_shared<century::perception::PerceptionObstacles>();
  manager_.UpdateAll(update_interval_);
  for (const auto& obstacle : manager_.GetObstacles()) {
    msg->add_perception_obstacle()->CopyFrom(obstacle);
  }

  writer_->Write(msg);
}

void ObstacleTask::Pause() { paused_ = true; }

void ObstacleTask::Resume() { paused_ = false; }

}  // namespace dreamview
}  // namespace century
