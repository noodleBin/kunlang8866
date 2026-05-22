/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once

#include <chrono>
#include <fstream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/dreamview/proto/hmi_status.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/dependency_injector.h"

namespace century {
namespace planning {

class MessageProcess {
 public:
  bool Init(const PlanningConfig& planning_config);
  bool Init(const PlanningConfig& planning_config,
            const std::shared_ptr<DependencyInjector>& injector);

  void Close();

  void OnChassis(const century::canbus::Chassis& chassis);

  void OnHMIStatus(century::dreamview::HMIStatus hmi_status);

  void OnLocalization(const century::localization::LocalizationEstimate& le);

  void OnPrediction(
      const century::prediction::PredictionObstacles& prediction_obstacles);

  void OnRoutingResponse(
      const century::routing::RoutingResponse& routing_response);

  void OnStoryTelling(const century::storytelling::Stories& stories);

  void OnTrafficLightDetection(
      const century::perception::TrafficLightDetection& traffic_light_detection);

  void ProcessOfflineData(const std::string& record_file);

 private:
  struct ADCCurrentInfo {
    std::pair<double, double> adc_cur_position_;
    std::pair<double, double> adc_cur_velocity_;
    std::pair<double, double> adc_cur_acc_;
    double adc_cur_heading_;
  };

  century::hdmap::LaneInfoConstPtr GetCurrentLane(
      const century::common::PointENU& position);
  bool GetADCCurrentRoutingIndex(int* adc_road_index, int* adc_passage_index,
                                 double* adc_passage_s);

  int GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info);

  void GenerateObstacleTrajectory(const int frame_num, const int obstacle_id,
                                  const ADCCurrentInfo& adc_curr_info,
                                  ObstacleFeature* obstacle_feature);

  void GenerateObstaclePrediction(
      const int frame_num,
      const century::prediction::PredictionObstacle& prediction_obstacle,
      const ADCCurrentInfo& adc_curr_info, ObstacleFeature* obstacle_feature);

  void GenerateObstacleFeature(LearningDataFrame* learning_data_frame);

  bool GenerateLocalRouting(const int frame_num,
                            RoutingResponseFeature* local_routing,
                            std::vector<std::string>* local_routing_lane_ids);

  void CalculateRoadLength(
      std::vector<std::pair<std::string, double>>* road_lengths);

  void GetLocalRoutingStartInfo(
      const int adc_road_index, const double adc_passage_s,
      const std::vector<std::pair<std::string, double>>& road_lengths,
      int* local_routing_start_road_index, double* local_routing_start_road_s);

  void GetLocalRoutingEndInfo(
      const int adc_road_index, const double adc_passage_s,
      const std::vector<std::pair<std::string, double>>& road_lengths,
      int* local_routing_end_road_index, double* local_routing_end_road_s);

  void GetLocalRoutingRoadPassageSegmentInfo(
      const int local_routing_start_road_index,
      const double local_routing_start_road_s,
      const int local_routing_end_road_index,
      const double local_routing_end_road_s, const int adc_road_index,
      const int adc_passage_index, RoutingResponseFeature* local_routing,
      std::vector<std::string>* local_routing_lane_ids);

  void GenerateRoutingFeature(
      const RoutingResponseFeature& local_routing,
      const std::vector<std::string>& local_routing_lane_ids,
      LearningDataFrame* learning_data_frame);

  void GenerateTrafficLightDetectionFeature(
      LearningDataFrame* learning_data_frame);
  void GenerateADCTrajectoryPoints(
      const std::list<century::localization::LocalizationEstimate>& localizations,
      LearningDataFrame* learning_data_frame);
  void GenerateADCTrajectoryPlanningTag(
      const int trajectory_point_index,
      const std::vector<ADCTrajectoryPoint>& adc_trajectory_points,
      const century::common::PointENU& cur_point, std::string* clear_area_id,
      double* clear_area_distance, std::string* crosswalk_id,
      double* crosswalk_distance, std::string* pnc_junction_id,
      double* pnc_junction_distance, std::string* signal_id,
      double* signal_distance, std::string* stop_sign_id,
      double* stop_sign_distance, std::string* yield_sign_id,
      double* yield_sign_distance, PlanningTag* planning_tag);

  void GenerateADCTrajectoryPlanningTagFirst(
      const int trajectory_point_index,
      const std::vector<ADCTrajectoryPoint>& adc_trajectory_points,
      const century::common::PointENU& cur_point, std::string* clear_area_id,
      double* clear_area_distance, std::string* crosswalk_id,
      double* crosswalk_distance, std::string* pnc_junction_id,
      double* pnc_junction_distance, std::string* signal_id,
      double* signal_distance, std::string* stop_sign_id,
      double* stop_sign_distance, std::string* yield_sign_id,
      double* yield_sign_distance, PlanningTag* planning_tag);

  void GenerateADCTrajectoryPlanningTagSecond(
      const int trajectory_point_index,
      const std::vector<ADCTrajectoryPoint>& adc_trajectory_points,
      const century::common::PointENU& cur_point, std::string* clear_area_id,
      double* clear_area_distance, std::string* crosswalk_id,
      double* crosswalk_distance, std::string* pnc_junction_id,
      double* pnc_junction_distance, std::string* signal_id,
      double* signal_distance, std::string* stop_sign_id,
      double* stop_sign_distance, std::string* yield_sign_id,
      double* yield_sign_distance, PlanningTag* planning_tag);

  void GeneratePlanningTag(LearningDataFrame* learning_data_frame);

  bool GenerateLearningDataFrame(LearningDataFrame* learning_data_frame);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::ofstream log_file_;
  std::string record_file_;
  std::unordered_map<std::string, std::string> map_m_;
  LearningData learning_data_;
  int learning_data_file_index_ = 0;
  std::list<century::localization::LocalizationEstimate> localizations_;
  std::unordered_map<int, century::prediction::PredictionObstacle>
      prediction_obstacles_map_;
  std::unordered_map<int, std::list<PerceptionObstacleFeature>>
      obstacle_history_map_;
  ChassisFeature chassis_feature_;
  std::string map_name_;
  PlanningTag planning_tag_;
  century::routing::RoutingResponse routing_response_;
  double traffic_light_detection_message_timestamp_;
  std::vector<TrafficLightFeature> traffic_lights_;
  int total_learning_data_frame_num_ = 0;
  double last_localization_message_timestamp_sec_ = 0.0;
};

}  // namespace planning
}  // namespace century
