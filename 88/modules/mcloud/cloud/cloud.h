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

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <vector>

// #define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
// #include <proj_api.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <queue>
#include <thread>
#include <atomic>
// #include "data_handler.h"
// #include "vehicle_message.h"
// #include "client.hpp"
#include "nlohmann/json.hpp"

#include "modules/localization/proto/localization.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/planning/proto/blocking_area_response.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/planning/proto/v2x_info.pb.h"
#include "modules/planning/proto/lane_borrow_response.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/mcloud/proto/emergency_stop.pb.h"
#include "modules/monitor/proto/system_status.pb.h"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/mcloud/network/client.hpp"
#include "modules/mcloud/network/data_handler.h"
#include "modules/mcloud/parser/vehicle_message.h"
#include "modules/monitor/common/monitor_utils.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/planning/proto/temporary_parking_request.pb.h"
#include "modules/dreamview/proto/background_music.pb.h"
#include "cyber/timer/timer.h"
// #define DEG_TO_RAD (M_PI / 180.0)
//  #include "modules/planning/proto/"

using century::hdmap::HDMapUtil;

namespace century {
namespace mcloud {

enum MESSAGE_TYPE : uint8_t {
  TASK_RECV = 0xC5,
  CONTROL_REQUEST = 0xCA,
  TASK_CONTRONL = 0xC7,
  PARAMETER_QUERY = 0xC8,
  SET_QUERY = 0xC9,
  INFORM_BROADCAST = 0xD5,
  REPLY_MESSAGE = 0XD6,
  INFORM_NOTIFY = 0xD7,
};

enum CONTROL_MESSAGE : uint8_t {
  REBOOT = 0x01,
  FINE_TUNING = 0x02,
  // IMMEDIATELY_STOP = 0x05,
  MOVE = 0x06,
  IMMEDIATELY_ARRIVE = 0x09,
  TASK_STATUS_CONTRONL = 0x0A,
  INFORM_BORROW = 0X0C,
  STOP = 0x0D,
  MULTI_BEGIN = 0x0F,
  MULTI_SELECT = 0x0E,
};

enum BoxType : uint8_t {
  DOUBLE_BOX = 0x00,
  SINGLE_BOX = 0x01,
};

enum MoveType : uint8_t {
  MOVEIN = 0X01,
  MOVEOUT = 0x02,
};
enum OperationType : uint8_t { LOADING = 0x00, UNLOADING = 0x01 };

enum AreaType : uint8_t {
  YARD_OPERATION_AREA = 0x01,
  YARD_WAITING_AREA = 0x02,
  RAILWAY_OPERATION_AREA = 0x03,
  RAILWAY_FIXED_WAITING_AREA = 0x04,
  RAILWAY_WAITING_AREA = 0x05,
  POWER_RECHARGE_AREA = 0x06,
  PARKING_AREA = 0x07,
};

enum POS_TYPE : uint8_t {
  SITE = 0x01,
  REACHSTACKER,
};

enum STATI_TYPE : uint8_t {
  OPERATION = 0x01,
  WAITING,
  RAILWAY_OPERATION,
  FIXED,
  RAILWA_WAITING,
  BATTERY,
  PARKING,
};

enum DEVICE_TYPE : uint8_t {
  DEVICE_IGV = 0x01,
  REACH_STACKER = 0x02,
  CONTAINER_Handler = 0X03,
};

struct SystemInfo {
  uint8_t system_status;
  uint8_t autoDriving_status;
  uint8_t task_status;
  uint8_t software_version[10];
  uint16_t cpu;
  uint16_t memory;
  uint8_t temprate;
  uint8_t tbox_status;
  uint8_t map_status;
  uint8_t box_status;
  uint8_t box_status_back;
  uint8_t fas_aeb_status;
  uint8_t reserve_status;
  uint8_t takeover_status;
  uint16_t borrow_status;
};

class CircularBuffer {
 public:
  CircularBuffer(size_t size);

  void push(const uint8_t* data, size_t length);
  std::vector<uint8_t> get_data() const;
  size_t available() const;
  void pop(size_t length);

 private:
  std::vector<uint8_t> buffer;
  size_t head;
  size_t tail;
  size_t count;
  mutable std::mutex mutex_;
};

class Cloud : public IDataHandler {
 public:
  Cloud();
  ~Cloud() override = default;

  void HandleData(const std::vector<uint8_t>& data) override;
  void RegisterClient(TcpClient* client, TcpClient* mqttClient);
  bool SendRoutingRequest(double longitude, double latitude,
                          double direction_angle, bool is_loading,
                          uint8_t hardware,
                          century::routing::TaskType task_type,
                          const std::string& crane_id = "");
  // bool ConvertLatLonToUTM(double latitude, double longitude, double& utm_x,
  //                         double& utm_y, std::string& utm_zone);
  // bool ConvertUTMToLatLon(double easting, double northing, int zone,
  //                         bool is_northern_hemisphere, double& latitude,
  //                         double& longitude);
  void Start();

 private:
  century::cyber::proto::QosProfile CreateQosProfile();
  //   void CreateQosProfile();
 public:
  void ProcessCommands();
  void ProcessCommand(const VehicleMessage& message);

 private:
  void ControlCommand(const VehicleMessage& message);
  void ReceTask(const VehicleMessage& message);
  void ParameterQuery(const VehicleMessage& message);
  void SetQuery(const VehicleMessage& message);

  bool SendRoutingRequest(double longitude, double latitude,
                          double direction_angle);
  bool SendRoutingRequest(double longitude, double latitude,
                          double direction_angle,
                          century::routing::TaskType task_type);

  bool GetNearestLaneWithHeading(const double x, const double y,
                                 century::hdmap::LaneInfoConstPtr* nearest_lane,
                                 double* nearest_s, double* nearest_l,
                                 const double heading) const;
  bool GetNearestLane(const double x, const double y,
                      century::hdmap::LaneInfoConstPtr* nearest_lane,
                      double* nearest_s, double* nearest_l) const;
  int CalculateUTMZone(double longitude);

  void Reply(const VehicleMessage& message, uint32_t id, uint8_t status);
  void SendSystemInfo();
  void GetSystemInfo();
  std::vector<uint8_t> GetTime();
  void Report();
  void ConvertBoxStatus();
  void AppendSystemStatusSection(
      const std::shared_ptr<century::planning::ADCTrajectory>& planning_msg,
      std::vector<uint8_t>& data_unit);
  void AppendResourceStatusSection(std::vector<uint8_t>& data_unit);
  void AppendFasAebStatusSection(std::vector<uint8_t>& data_unit);
  std::shared_ptr<century::planning::ADCTrajectory> GetLatestPlanningMessage();
  void AppendTakeoverAndBorrowStatusSection(
      const std::shared_ptr<century::planning::ADCTrajectory>& planning_msg,
      std::vector<uint8_t>& data_unit);
  void AppendFixedSystemSections(std::vector<uint8_t>& data_unit);
  void AppendRoadEventSection(std::vector<uint8_t>& data_unit);
  void AppendLocalizationSection(std::vector<uint8_t>& data_unit);
  void AppendMultiResponsesSection(std::vector<uint8_t>& data_unit);
  // century::localization::Pose GetLocalization();

  void TaskContronl(const VehicleMessage& message);

  void HandRequestTask();
  void CheckAndSendDefaultData();

 public:
  century::routing::TaskType ConvertType(uint8_t task_type, uint8_t pose_type,
                                         uint8_t station_type, uint8_t boxes);

 private:
  void OnRoutingResult(
      const std::shared_ptr<century::routing::RoutingResult>& result);
  bool SendRoutingRequest(century::routing::TaskType task_type, double distance,
                          double longitude, double latitude, double heading,
                          uint8_t operator_type,
                          const std::string& crane_id = "");

  void OnPlanningRequest(
      const std::shared_ptr<century::planning::ADCTrajectory>& result);

  void SendPassStackerInform(const std::string& vehicle_id,
                             const std::string& vin_id, uint32_t vehicle_type,
                             const std::string& target_no, uint32_t action,
                             uint32_t operate, const std::string& msg_id);

 private:
  void HandleRebootCommand(const VehicleMessage& message, uint32_t sequence);
  void HandleFineCommand(const VehicleMessage& message, uint32_t sequence);
  void HandleMoveCommand(const VehicleMessage& message, uint32_t sequence);
  void HandleStopCommand(const VehicleMessage& message, uint32_t sequence);

  void HandleImmediatelyStopCommand(bool stop);
  void HandleImmediatelyStopCommand(const VehicleMessage& message,
                                    uint32_t sequence);
  void HandleImmediatelyArriveCommand(const VehicleMessage& message,
                                      uint32_t sequence);
  void HandleDataTaskStatusCommand(const VehicleMessage& message,
                                   uint32_t sequence);

  void HandleMultiBegin(const VehicleMessage& message, uint32_t sequence);
  void HandleMultiSelect(const VehicleMessage& message, uint32_t sequence);

  void HandleBorrowCommand(const VehicleMessage& messagem, uint32_t sequence);

  void CalculateBoxStatus();
  void SaveRoutingTask(const VehicleMessage& message,
                       const std::string& status);
  bool LoadRoutingTask(VehicleMessage* message, const std::string& status);
  bool ModifyLastRoutingTask(const std::string& status);

 private:
  bool stop_flag_;
  int8_t temporary_parking_ = 0x01;
  std::string borrow_id;
  CircularBuffer buffer_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  std::vector<uint32_t> last_action_;

  std::unordered_map<std::string, std::vector<uint8_t>> request_data;
  std::mutex request_mutex;
  std::chrono::steady_clock::time_point last_stacker_info_time_;
  std::unique_ptr<std::thread> stacker_timer_thread_;
  std::unique_ptr<cyber::Timer> temporary_parking_release_timer_;
  // std::atomic<bool> stop_flag_{false};

  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<century::routing::RoutingRequest>>
      routing_request_writer_;

  std::shared_ptr<cyber::Reader<century::routing::RoutingRequest>>
      routing_request_reader_;

  std::shared_ptr<cyber::Reader<century::routing::RoutingResponses>>
      routing_routing_responses_reader_;

  std::shared_ptr<century::routing::RoutingRequest> last_routing_request_;

  std::shared_ptr<cyber::Reader<century::routing::RoutingResponse>>
      routing_response_request_reader_;
  std::shared_ptr<cyber::Reader<century::localization::LocalizationEstimate>>
      localization_reader_;

  std::shared_ptr<cyber::Reader<century::planning::ADCTrajectory>>
      planning_reader_;
  std::shared_ptr<cyber::Reader<century::routing::RoutingResult>>
      routing_result_reader_;

  std::shared_ptr<cyber::Writer<century::mcloud::McloudInfo>>
      cloud_info_writer_;

  std::shared_ptr<century::cyber::Writer<planning::BlockingAreaResponse>>
      blocking_area_writer_;

  std::shared_ptr<cyber::Writer<century::planning::V2xInfo>> v2x_info_writer_;
  std::shared_ptr<cyber::Writer<century::planning::BorrowResponse>>
      borrow_writer_;
  std::shared_ptr<cyber::Writer<century::planning::PassStackerResponse>>
      pass_stacker_writer_;

  std::shared_ptr<cyber::Writer<century::planning::TemporaryParkingRequest>>
      temporary_parking_request_writer_;

  std::shared_ptr<cyber::Writer<century::planning::TemporaryParkingRequest>>
      multi_temporary_parking_request_writer_;
  std::shared_ptr<cyber::Reader<century::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<century::control::ControlCommand>>
      control_command_reader_;

  std::shared_ptr<cyber::Writer<century::planning::StackersInfo>>
      stacker_info_writer_;
  std::shared_ptr<cyber::Client<century::fas_aeb_backend::Request,
                                century::fas_aeb_backend::Response>>
      fas_aeb_backend_client_;

  std::shared_ptr<cyber::Reader<century::fas_aeb_backend::FasAebInfo>>
      fas_aeb_info_reader_;
  std::shared_ptr<cyber::Reader<century::monitor::MonitoredData>>
      system_monitor_x86_reader_;
  std::shared_ptr<cyber::Reader<century::monitor::MonitoredData>>
      system_monitor_aarch_reader_;

  std::queue<VehicleMessage> command_que_;

  std::unique_ptr<std::thread> recv_task_;
  std::unique_ptr<std::thread> send_task_;
  std::shared_ptr<cyber::Writer<century::mcloud::EmergencyStop>>
      emergency_stop_;

  std::shared_ptr<cyber::Writer<century::dreamview::BackgroundMusic>>
      bck_music_writer_;

  SystemInfo system_info_;
  const std::vector<uint8_t> startFlag = {0x23, 0x23};
  const std::vector<uint8_t> endFlag = {0x23, 0x23};
  std::unordered_map<uint32_t, century::routing::TaskType> convert_task_type_;
  TcpClient* client_;
  TcpClient* mqttClient_;
  uint32_t seq_ = {0};
  uint32_t box_status_ = {0};
  uint32_t is_loading_ = {0};
  bool reached_station_ = {false};
  BoxType box_type_;
  century::routing::TaskType task_type_;
  double offset_x_ = {250932.85};
  double offset_y_ = {3987498.59};
  std::string zone_id_ = {"51S"};
  inline static constexpr char kLastTaskFile[] =
      "/century/modules/mcloud/conf/routing_task.json";

  std::mutex responses_mutex_;

  bool immediately_stop_ = {false};
  bool background_music_enable_ = {false};
  std::vector<uint8_t> multi_responses_;
};

}  // namespace mcloud
}  // namespace century
