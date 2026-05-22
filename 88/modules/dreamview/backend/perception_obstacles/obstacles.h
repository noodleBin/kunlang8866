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

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <cmath>
#include "nlohmann/json.hpp"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/dreamview/backend/sim_control/sim_control.h"
#include "modules/routing/proto/routing.pb.h"
#include "cyber/cyber.h"

namespace century {
namespace dreamview {

using century::perception::PerceptionObstacle;

struct Position {
  std::string id;
  float x;
  float y;
  float z;
};

struct Trace {
  Position position;
  float speed;
};

class Obstacles {
 public:
  Obstacles();
  virtual ~Obstacles();  

  void createObstacle(const nlohmann::json& obstacle_json);

  int8_t GetObstaclesType() const;

  bool Trigger(float run_time);

  bool IsCloseToTriggerPosition();

  void setPolygonPoints(double center_x, double center_y, double length,
                        double width, double theta);

  PerceptionObstacle* GetObstacle();  

  void UpdateObstacle(float run_time);

 private:
  std::shared_ptr<century::cyber::Node> node_;
  std::shared_ptr<cyber::Reader<century::localization::LocalizationEstimate>> localization_reader_;
  PerceptionObstacle perception_obstacle_;
  float speed_;
  int8_t obstacles_type_;
  int8_t trigger_type_;
  Position trigger_postion_;
  float trigger_radius_ = {0.0};
  bool trgger_ = {false};
  float trigger_time_;
  std::vector<Trace> trace_postion;
  float run_time_all_;
  float vehicle_x_;
  float vehicle_y_;
};

class ObstacleManager {
 public:
  ObstacleManager();
  ~ObstacleManager();  

  void AddObstacle(std::shared_ptr<Obstacles> obstacle);
  void UpdateAll(float run_time);
  void AddObstacleFromJson(const nlohmann::json& json);
  std::vector<PerceptionObstacle> GetObstacles();
  void ClearObstacle();
 private:
  std::vector<std::shared_ptr<Obstacles>> obstacles_;
};

class ObstacleTask {
 public:
  ObstacleTask(
               const std::string& writer_channel,
               double update_interval, SimControl *sim_control);
  ~ObstacleTask();  

  void Init(const nlohmann::json& obstacle_json);



  void Start();

  
  void Pause();   
  void Resume();  
  bool IsPaused() const { return paused_; }

 private:
  void UpdateObstaclesAndSend();

  std::shared_ptr<century::cyber::Node> node_;
  std::shared_ptr<century::cyber::Writer<century::perception::PerceptionObstacles>> writer_;
  std::shared_ptr<cyber::Reader<century::routing::RoutingResult>> routing_result_reader_;
  century::dreamview::ObstacleManager manager_;
  double update_interval_;

  std::string writer_channel_;
  bool paused_; 
  SimControl *sim_control_ = nullptr; 
};



class NodeSingleton {
 public:
  static std::shared_ptr<century::cyber::Node> GetNodeInstance() {
    static std::shared_ptr<century::cyber::Node> instance =
        century::cyber::CreateNode("dreamview_obstacles");
    return instance;
  }


 private:
  NodeSingleton() = default;
  ~NodeSingleton() = default;
  NodeSingleton(const NodeSingleton&) = delete;
  NodeSingleton& operator=(const NodeSingleton&) = delete;
};


}  // namespace dreamview
}  // namespace century
