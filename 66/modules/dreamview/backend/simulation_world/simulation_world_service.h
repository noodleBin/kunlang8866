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

/**
 * @file
 */

#pragma once

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "gtest/gtest_prod.h"

#include "nlohmann/json.hpp"

#include "modules/audio/proto/audio.pb.h"
#include "modules/audio/proto/audio_event.pb.h"
#include "modules/common/proto/drive_event.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/storytelling/proto/story.pb.h"
#include "modules/task_manager/proto/task_manager.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/dreamview/proto/barrier_command.pb.h"
#include "modules/dreamview/proto/background_music.pb.h"

#include "cyber/common/log.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/planning/proto/lane_borrow_response.pb.h"

/**
 * @namespace century::dreamview
 * @brief century::dreamview
 */
namespace century {
namespace dreamview {

/**
 * @class SimulationWorldService
 * @brief This is a major component of the Simulation backend, which
 * maintains a SimulationWorld object and keeps updating it. The SimulationWorld
 * represents the most up-to-date information about all the objects
 * in the emulated world, including the car, the planning trajectory, etc.
 * NOTE: This class is not thread-safe.
 */
class SimulationWorldService {
 public:
  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

  /**
   * @brief Constructor of SimulationWorldService.
   * @param map_service the pointer of MapService.
   * @param routing_from_file whether to read initial routing from file.
   */
  SimulationWorldService(const MapService *map_service,
                         bool routing_from_file = false);

  /**
   * @brief Get a read-only view of the SimulationWorld.
   * @return Constant reference to the SimulationWorld object.
   */
  inline const SimulationWorld &world() const { return world_; }

  // The destination is expected to be in the same map frame as localization
  // and perception obstacle positions.
  void SetDestinationPosition(
      const century::common::PointENU &destination_position);
  void ClearDestinationPosition();

  /**
   * @brief Returns the json representation of the SimulationWorld object.
   *        This is a public API used by offline dreamview.
   * @param radius the search distance from the current car location
   * @return Json object equivalence of the SimulationWorld object.
   */
  nlohmann::json GetUpdateAsJson(double radius) const;

  /**
   * @brief Returns the binary representation of the SimulationWorld object.
   * @param radius the search distance from the current car location.
   * @param sim_world output of binary format sim_world string.
   * @param sim_world_with_planning_data output of binary format sim_world
   * string with planning_data.
   */
  void GetWireFormatString(double radius, std::string *sim_world,
                           std::string *sim_world_with_planning_data);

  /**
   * @brief Returns the json representation of the map element Ids and hash
   * within the given radius from the car.
   * @param radius the search distance from the current car location
   * @return Json object that contains mapElementIds and mapHash.
   */
  nlohmann::json GetMapElements(double radius) const;

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the external messages. All the updates will be written to the
   * SimulationWorld object to reflect the latest status.
   */
  void Update();

  /**
   * @brief Sets the flag to clear the owned simulation world object.
   */
  void SetToClear() { to_clear_ = true; }

  /**
   * @brief Check whether the SimulationWorld object has enough information.
   * The backend won't push the SimulationWorld to frontend if it is not ready.
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const { return ready_to_push_.load(); }

  /**
   * @brief Publish message to the monitor
   * @param msg the string to send to monitor
   * @param log_level defined in
   *        modules/common/monitor_log/proto/monitor_log.proto
   */
  void PublishMonitorMessage(
      century::common::monitor::MonitorMessageItem::LogLevel log_level,
      const std::string &msg);

  void PublishNavigationInfo(
      const std::shared_ptr<century::relative_map::NavigationInfo> &);
  void PublishRoutingRequest(
      const std::shared_ptr<century::routing::RoutingRequest> &);
  void PublishRoutingRequestWithoutHeader(
      const std::shared_ptr<century::routing::RoutingRequest> &);
  void PublishRoutingRequest(const nlohmann::json& json);
  std::shared_ptr<century::routing::RoutingRequest>
  GetLatestRoutingRequest();

  void PublishImmediatelyArrived(
      const std::shared_ptr<century::mcloud::McloudInfo> mcloud_info);

  void PublishTask(const std::shared_ptr<century::task_manager::Task> &);

  void PublishBorrowResponse(
      const std::shared_ptr<century::planning::BorrowResponse>
          &borrow_response);

  void PublishPassStackerResponse(
      const std::shared_ptr<century::planning::PassStackerResponse>
          &pass_stacker_response);

  void GetMapElementIds(double radius, MapElementIds *ids) const;

  const century::hdmap::Map &GetRelativeMap() const;

  nlohmann::json GetRoutePathAsJson() const;

  void DumpMessages();

  nlohmann::json GetSystemInfo();
  void SendRoutingResponse(size_t idx);
  nlohmann::json ModifyFasAebInfo(const nlohmann::json& fast_aeb_info);
  nlohmann::json PublishBarrierCommand(
      const nlohmann::json& barrier_command_info);
  bool IsBarrierCommandActive() const { return barrier_command_active_; }

  void ChangeBckMusicSwitch(const nlohmann::json& json);

 private:
  void InitReaders();
  void InitWriters();

  /**
   * @brief Update simulation world with incoming data, e.g., chassis,
   * localization, planning, perception, etc.
   */
  template <typename DataType>
  void UpdateSimulationWorld(const DataType &data);

  template <typename DataType>
  void UpdateSimulationAroundEgoWorld(const DataType &data);

  void UpdateBarrierCommand();
  void WriteBarrierCommand(
      bool enabled,
      century::dreamview::BarrierCommand::CommandType command_type =
          century::dreamview::BarrierCommand::COMMAND_TYPE_UNKNOWN);
  void UpdateMonitorMessages();

  Object &CreateWorldObjectIfAbsent(
      const century::perception::PerceptionObstacle &obstacle);
  void CreateWorldObjectFromSensorMeasurement(
      const century::perception::SensorMeasurement &sensor,
      Object *world_object);
  void SetObstacleInfo(const century::perception::PerceptionObstacle &obstacle,
                       Object *world_object);
  void SetObstaclePolygon(
      const century::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void SetObstacleSensorMeasurements(
      const century::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void SetObstacleSource(
      const century::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void UpdatePlanningTrajectory(
      const century::planning::ADCTrajectory &trajectory);
  void UpdateRSSInfo(const century::planning::ADCTrajectory &trajectory);

  void UpdateBorrowRequest();

  bool LocateMarker(const century::planning::ObjectDecisionType &decision,
                    Decision *world_decision);
  void FindNudgeRegion(const century::planning::ObjectDecisionType &decision,
                       const Object &world_obj, Decision *world_decision);
  void UpdateDecision(const century::planning::DecisionResult &decision_res,
                      double header_time);
  void UpdateMainStopDecision(
      const century::planning::MainDecision &main_decision,
      double update_timestamp_sec, Object *world_main_stop);

  void MaybePrintNearestObstacleWhenNearDestination(
      const century::localization::LocalizationEstimate &localization);
  const century::perception::PerceptionObstacle *FindNearestPerceptionObstacle(
      const century::common::PointENU &current_position,
      const century::perception::PerceptionObstacles &obstacles) const;
  const century::planning::StackerInfo *FindNearestStackerFromStackersInfo(
      const century::common::PointENU &destination_position) const;
  template <typename MainDecision>
  void UpdateMainChangeLaneDecision(const MainDecision &decision,
                                    Object *world_main_decision) {
    if (decision.has_change_lane_type() &&
        (decision.change_lane_type() ==
             century::routing::ChangeLaneType::LEFT ||
         decision.change_lane_type() ==
             century::routing::ChangeLaneType::RIGHT)) {
      auto *change_lane_decision = world_main_decision->add_decision();
      change_lane_decision->set_change_lane_type(decision.change_lane_type());

      const auto &adc = world_.auto_driving_car();
      change_lane_decision->set_position_x(adc.position_x());
      change_lane_decision->set_position_y(adc.position_y());
      change_lane_decision->set_heading(adc.heading());
    }
  }

  void CreatePredictionTrajectory(
      const century::prediction::PredictionObstacle &obstacle,
      Object *world_object);

  void DownsamplePath(const century::common::Path &paths,
                      century::common::Path *downsampled_path);

  void UpdatePlanningData(const century::planning_internal::PlanningData &data);

  void PopulateMapInfo(double radius);

  /**
   * @brief Get the latest observed data from reader to update the
   * SimulationWorld object when triggered by refresh timer.
   */
  template <typename MessageT>
  void UpdateWithLatestObserved(cyber::Reader<MessageT> *reader,
                                bool logging = true) {
    if (reader->Empty()) {
      if (logging) {
        AINFO_EVERY(100) << "Has not received any data from "
                         << reader->GetChannelName();
      }
      return;
    }

    const std::shared_ptr<MessageT> msg = reader->GetLatestObserved();
    UpdateSimulationWorld(*msg);
  }

  template <typename MessageT>
  void UpdateAroundEgoWithLatestObserved(cyber::Reader<MessageT> *reader) {
    if (reader->Empty()) {
      return;
    }

    const std::shared_ptr<MessageT> msg = reader->GetLatestObserved();
    UpdateSimulationAroundEgoWorld(*msg);
  }

  /**
   * @brief Get the latest observed data from reader and dump it to a local
   * file.
   */
  template <typename MessageT>
  void DumpMessageFromReader(cyber::Reader<MessageT> *reader) {
    if (reader->Empty()) {
      AWARN << "Has not received any data from " << reader->GetChannelName()
            << ". Cannot dump message!";
      return;
    }

    century::common::util::DumpMessage(reader->GetLatestObserved());
  }

  void ReadRoutingFromFile(const std::string &routing_response_file);

  template <typename MessageT>
  void UpdateLatency(const std::string &module_name,
                     cyber::Reader<MessageT> *reader) {
    if (reader->Empty()) {
      return;
    }

    const auto header = reader->GetLatestObserved()->header();
    const double publish_time_sec = header.timestamp_sec();
    const double sensor_time_sec =
        century::cyber::Time(
            std::max({header.lidar_timestamp(), header.camera_timestamp(),
                      header.radar_timestamp()}))
            .ToSecond();

    Latency latency;
    latency.set_timestamp_sec(publish_time_sec);
    latency.set_total_time_ms((publish_time_sec - sensor_time_sec) * 1.0e3);
    (*world_.mutable_latency())[module_name] = latency;
  }

  /**
   * @brief update delayes of modules.
   * @detail Delay is calculated based on the received time from a module
   * reader. If the reader has not received any message, delay is -1. Otherwise,
   * it is the max of (current_time - last_received_time) and
   * (last_received_time - second_to_last_received_time)
   */
  void UpdateDelays();

  /**
   * @brief update latencies of modules, where latency is how long it takes for
   * sensor data (lidar, radar and/or camera) to be processed by
   * a module.
   */
  void UpdateLatencies();

  template <typename Points>
  void DownsampleSpeedPointsByInterval(const Points &points,
                                       size_t downsampleInterval,
                                       Points *downsampled_points) {
    if (points.empty()) {
      return;
    }

    for (int i = 0; i + 1 < points.size(); i += downsampleInterval) {
      *downsampled_points->Add() = points[i];
    }

    // add the last point
    *downsampled_points->Add() = *points.rbegin();
  }

  std::unique_ptr<cyber::Node> node_;

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;

  // Downsampled route paths to be rendered in frontend.
  mutable boost::shared_mutex route_paths_mutex_;
  std::vector<std::vector<RoutePath>> route_paths_;

  // The handle of MapService, not owned by SimulationWorldService.
  const MapService *map_service_;

  // The map holding obstacle string id to the actual object
  std::unordered_map<std::string, Object> obj_map_;

  // A temporary cache for all the monitor messages coming in.
  std::mutex monitor_msgs_mutex_;
  std::list<std::shared_ptr<common::monitor::MonitorMessage>> monitor_msgs_;

  // The SIMULATOR monitor for publishing messages.
  century::common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // Whether to clear the SimulationWorld in the next timer cycle, set by
  // frontend request.
  bool to_clear_ = false;

  // Relative map used/retrieved in navigation mode
  century::hdmap::Map relative_map_;

  // Whether the sim_world is ready to push to frontend
  std::atomic<bool> ready_to_push_;

  // Latest rss info
  double current_real_dist_ = 0.0;
  double current_rss_safe_dist_ = 0.0;

  // Gear Location
  century::canbus::Chassis_GearPosition gear_location_;

  // Readers.
  std::shared_ptr<cyber::Reader<century::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<century::localization::Gps>> gps_reader_;
  std::shared_ptr<cyber::Reader<century::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<century::perception::PerceptionObstacles>>
      perception_obstacle_reader_;
  std::shared_ptr<cyber::Reader<century::perception::PerceptionObstacles>>
      perception_aound_ego_obstacle_reader_;
  std::shared_ptr<cyber::Reader<century::perception::TrafficLightDetection>>
      perception_traffic_light_reader_;
  std::shared_ptr<cyber::Reader<century::prediction::PredictionObstacles>>
      prediction_obstacle_reader_;
  std::shared_ptr<cyber::Reader<century::planning::ADCTrajectory>>
      planning_reader_;
  std::shared_ptr<cyber::Reader<century::control::ControlCommand>>
      control_command_reader_;
  std::shared_ptr<cyber::Reader<century::relative_map::NavigationInfo>>
      navigation_reader_;
  std::shared_ptr<cyber::Reader<century::relative_map::MapMsg>>
      relative_map_reader_;
  std::shared_ptr<cyber::Reader<century::audio::AudioEvent>>
      audio_event_reader_;
  std::shared_ptr<cyber::Reader<century::common::DriveEvent>>
      drive_event_reader_;
  std::shared_ptr<cyber::Reader<century::common::monitor::MonitorMessage>>
      monitor_reader_;
  std::shared_ptr<cyber::Reader<century::routing::RoutingRequest>>
      routing_request_reader_;
  std::shared_ptr<cyber::Reader<century::routing::RoutingResponse>>
      routing_response_reader_;

  std::shared_ptr<cyber::Reader<century::routing::RoutingResponses>>
      routing_responses_reader_;

  std::shared_ptr<cyber::Reader<century::storytelling::Stories>>
      storytelling_reader_;
  std::shared_ptr<cyber::Reader<century::audio::AudioDetection>>
      audio_detection_reader_;
  std::shared_ptr<cyber::Reader<century::task_manager::Task>> task_reader_;
  std::shared_ptr<cyber::Reader<century::planning::StackersInfo>>
      stackers_info_reader_;

  std::shared_ptr<cyber::Reader<century::monitor::MonitoredData>>
      system_monitor_reader_;

  std::shared_ptr<cyber::Reader<century::fas_aeb_backend::FasAebInfo>> fas_aeb_info_reader_;
  // Writers.
  std::shared_ptr<cyber::Writer<century::relative_map::NavigationInfo>>
      navigation_writer_;
  std::shared_ptr<cyber::Writer<century::routing::RoutingRequest>>
      routing_request_writer_;
  std::shared_ptr<cyber::Writer<century::routing::RoutingResponse>>
      routing_response_writer_;
  std::shared_ptr<cyber::Writer<century::task_manager::Task>> task_writer_;
  std::shared_ptr<cyber::Writer<century::mcloud::McloudInfo>>
      cloud_info_writer_;
  std::shared_ptr<cyber::Writer<century::planning::BorrowResponse>>
      planning_borrow_response_writer_;
  std::shared_ptr<cyber::Client<century::fas_aeb_backend::Request, century::fas_aeb_backend::Response>>
      fas_aeb_backend_client_;
  std::shared_ptr<cyber::Writer<century::planning::PassStackerResponse>>
      planning_pass_stacker_writer_;
  std::shared_ptr<cyber::Writer<century::dreamview::BarrierCommand>>
      barrier_command_writer_;
  std::shared_ptr<cyber::Writer<century::dreamview::BackgroundMusic>>
      bck_music_writer_;
  std::shared_ptr<century::routing::RoutingRequest> last_external_routing_request_;
  century::common::PointENU destination_position_;
  bool has_destination_position_ = false;

  bool barrier_command_active_ = false;
  double barrier_command_stop_time_sec_ = 0.0;
  double barrier_command_last_publish_time_sec_ = 0.0;
  century::dreamview::BarrierCommand::CommandType barrier_command_type_ =
      century::dreamview::BarrierCommand::COMMAND_TYPE_UNKNOWN;

  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorSuccess);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorRemove);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorTruncate);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateChassisInfo);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateLatency);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateLocalization);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePerceptionObstacles);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePlanningTrajectory);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateDecision);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePrediction);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateRouting);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateGps);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateControlCommandWithSimpleLonLat);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateControlCommandWithSimpleMpc);
  FRIEND_TEST(SimulationWorldServiceTest, DownsampleSpeedPointsByInterval);
};

}  // namespace dreamview
}  // namespace century
