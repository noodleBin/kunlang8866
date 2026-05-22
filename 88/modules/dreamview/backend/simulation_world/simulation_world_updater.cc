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

#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"

#include "google/protobuf/util/json_util.h"

#include "cyber/common/file.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/fuel_monitor/fuel_monitor_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"

#include <fstream>
#include <sstream>
#include <chrono>

namespace century {
namespace dreamview {

using century::common::monitor::MonitorMessageItem;
using century::common::util::ContainsKey;
using century::common::util::JsonUtil;
using century::cyber::common::GetProtoFromASCIIFile;
using century::cyber::common::SetProtoToASCIIFile;
using century::hdmap::DefaultRoutingFile;
using century::hdmap::EndWayPointFile;
using century::relative_map::NavigationInfo;
using century::routing::LaneWaypoint;
using century::routing::RoutingRequest;
using century::task_manager::CycleRoutingTask;
using century::task_manager::DeadEndRoutingTask;
using century::task_manager::ParkingRoutingTask;
using century::task_manager::Task;

using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

namespace {
constexpr int kRecordStatusWaitMs = 200;
}  // namespace

SimulationWorldUpdater::SimulationWorldUpdater(
    WebSocketHandler *websocket, WebSocketHandler *map_ws,
    WebSocketHandler *camera_ws, SimControl *sim_control,
    const MapService *map_service,
    PerceptionCameraUpdater *perception_camera_updater, bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      camera_ws_(camera_ws),
      sim_control_(sim_control),
      perception_camera_updater_(perception_camera_updater),
      obstacle_task_(std::make_shared<century::dreamview::ObstacleTask>(
          "/century/perception/obstacles", 0.1, sim_control_)) {
  record_node_ = cyber::CreateNode("dreamview_record_control");
  record_cmd_writer_ =
      record_node_->CreateWriter<RecordCommand>(FLAGS_record_cmd_topic);
  record_status_reader_ =
      record_node_->CreateReader<RecordStatus>(
          FLAGS_record_status_topic,
          [this](const std::shared_ptr<RecordStatus>& msg) {
            if (!msg) {
              return;
            }
            std::string error = msg->has_error() ? msg->error() : "";
            {
              std::lock_guard<std::mutex> lock(record_status_mutex_);
              record_status_value_ = msg->status();
              record_status_error_ = error;
              record_status_valid_ = true;
              ++record_status_seq_;
            }
            record_status_cv_.notify_all();
          });

  RegisterMessageHandlers();

  std::thread task_thread(
      [obstacle_task = obstacle_task_]() { obstacle_task->Start(); });
  task_thread_ = std::move(task_thread);
}

bool ExecCommand(const std::string& cmd, std::string& output) {
  output.clear();

  if (cmd.empty()) {
    return false;
  }

  std::array<char, 256> buffer{};
  std::string full_cmd = cmd + " 2>&1";

  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(full_cmd.c_str(), "r"),
                                                pclose);

  if (!pipe) {
    return false;
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    output.append(buffer.data());
  }

  int status = pclose(pipe.release());
  if (-1 == status) {
    return false;
  }

  return true;
}

void SimulationWorldUpdater::RegisterConnectionReadyHandlers() {
  // Send current sim_control status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "SimControlStatus";
        response["enabled"] = sim_control_->IsEnabled();
        websocket_->SendData(conn, response.dump());

        Json barrier_response;
        barrier_response["type"] = "BarrierCommandStatus";
        barrier_response["enabled"] = sim_world_service_.IsBarrierCommandActive();
        websocket_->SendData(conn, barrier_response.dump());
      });
}

void SimulationWorldUpdater::RegisterMapDataHandlers() {
  map_ws_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids;
          if (JsonStringToMessage(iter->dump(), &map_element_ids).ok()) {
            auto retrieved = map_service_->RetrieveMapElements(map_element_ids);

            std::string retrieved_map_string;
            retrieved.SerializeToString(&retrieved_map_string);

            map_ws_->SendBinaryData(conn, retrieved_map_string, true);
          } else {
            AERROR << "Failed to parse MapElementIds from json";
          }
        }
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveRelativeMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        {
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = relative_map_string_;
        }
        map_ws_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "Binary",
      [this](const std::string &data, WebSocketHandler::Connection *conn) {
        // Navigation info in binary format
        auto navigation_info = std::make_shared<NavigationInfo>();
        if (navigation_info->ParseFromString(data)) {
          sim_world_service_.PublishNavigationInfo(navigation_info);
        } else {
          AERROR << "Failed to parse navigation info from string. String size: "
                 << data.size();
        }
      });

  websocket_->RegisterMessageHandler(
      "RetrieveMapElementIdsByRadius",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto radius = json.find("radius");
        if (radius == json.end()) {
          AERROR << "Cannot retrieve map elements with unknown radius.";
          return;
        }

        if (!radius->is_number()) {
          AERROR << "Expect radius with type 'number', but was "
                 << radius->type_name();
          return;
        }

        Json response;
        response["type"] = "MapElementIds";
        response["mapRadius"] = *radius;

        MapElementIds ids;
        sim_world_service_.GetMapElementIds(*radius, &ids);
        std::string elementIds;
        MessageToJsonString(ids, &elementIds);
        response["mapElementIds"] = Json::parse(elementIds);

        websocket_->SendData(conn, response.dump());
      });
}

void SimulationWorldUpdater::RegisterRoutingAndTaskHandlers() {
  RegisterRoutingBasicHandlers();
  RegisterTemporaryMoveHandlers();
  RegisterRoutingTaskRequestHandlers();
}

void SimulationWorldUpdater::RegisterRoutingBasicHandlers() {
  websocket_->RegisterMessageHandler(
      "CheckRoutingPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = CheckRoutingPoint(json);
        response["type"] = "RoutingPointCheckResult";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
    "ProjectVersion",
    [this](const Json &json, WebSocketHandler::Connection *conn){
      Json response = CheckVersion("/century/version.json");
      response["type"] = "ProjectVersionCheckResult";
      websocket_->SendData(conn, response.dump());
    });

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (SendRoutingRequestFromJson(json)) {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
      });
}

void SimulationWorldUpdater::RegisterTemporaryMoveHandlers() {
  websocket_->RegisterMessageHandler(
      "StartTemporaryMove",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "TemporaryMoveResult";
        response["status"] = false;
        if (!ContainsKey(json, "start") || !ContainsKey(json, "end")) {
          const std::string error =
              "Missing start/end point for temporary move request.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        if (json.contains("end") && json["end"].contains("type") &&
            "SEARCH_REACH_STACKER" == json["end"]["type"]) {
          if (SendRoutingRequestFromJson(json)) {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::INFO,
                "Temporary move routing request sent.");
          } else {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::ERROR,
                "Failed to send a temporary move routing request.");
          }
          return;
        }

        auto latest_routing_request =
            sim_world_service_.GetLatestRoutingRequest();
        if (!latest_routing_request) {
          const std::string error =
              "Failed to get latest routing request from routing channels.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        const bool latest_is_temporary_move_routing =
            latest_routing_request->has_task_type() &&
            century::routing::TaskType::TEMPORARY_VEH_RELOCATION ==
                latest_routing_request->task_type();
        if (!latest_is_temporary_move_routing) {
          temporary_move_original_routing_request_.CopyFrom(
              *latest_routing_request);
          has_temporary_move_original_routing_request_ = true;
        } else if (!has_temporary_move_original_routing_request_) {
          const std::string error =
              "Latest routing request is temporary move routing, but no valid "
              "non-temporary routing is cached.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        Json temporary_routing_json;
        temporary_routing_json["start"] = json["start"];
        temporary_routing_json["waypoint"] = Json::array();
        temporary_routing_json["end"] = json["end"];
        temporary_routing_json["end"]["type"] = "TEMPORARY_VEH_RELOCATION";

        sim_world_service_.ClearDestinationPosition();
        auto temporary_routing_request = std::make_shared<RoutingRequest>();
        if (!ConstructRoutingRequest(temporary_routing_json,
                                     temporary_routing_request.get())) {
          const std::string error =
              "Failed to construct temporary move routing request.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        SetDestinationTriggerPoint(*temporary_routing_request);
        sim_world_service_.PublishRoutingRequest(temporary_routing_request);
        sim_world_service_.PublishMonitorMessage(
            MonitorMessageItem::INFO, "Temporary move routing request sent.");

        response["status"] = true;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "EndTemporaryMove",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "EndTemporaryMoveResult";
        response["status"] = false;

        if (!has_temporary_move_original_routing_request_) {
          const std::string error =
              "No original routing cached for temporary move restore.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }
        if (!ContainsKey(json, "start")) {
          const std::string error =
              "Missing current vehicle position for temporary move restore.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        auto original_routing_request = std::make_shared<RoutingRequest>(
            temporary_move_original_routing_request_);
        if (0 == original_routing_request->waypoint_size()) {
          const std::string error =
              "Cached original routing request has no waypoint.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }
        if (!ValidateCoordinate(json["start"])) {
          const std::string error =
              "Invalid current vehicle position for temporary move restore.";
          response["error"] = error;
          websocket_->SendData(conn, response.dump());
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   error);
          return;
        }

        auto *start_waypoint = original_routing_request->mutable_waypoint(0);
        start_waypoint->mutable_pose()->set_x(json["start"]["x"]);
        start_waypoint->mutable_pose()->set_y(json["start"]["y"]);
        if (ContainsKey(json["start"], "z") &&
            json["start"]["z"].is_number()) {
          start_waypoint->mutable_pose()->set_z(json["start"]["z"]);
        }
        if (ContainsKey(json["start"], "heading") &&
            json["start"]["heading"].is_number()) {
          start_waypoint->set_heading(json["start"]["heading"]);
        }

        sim_world_service_.ClearDestinationPosition();
        SetDestinationTriggerPoint(*original_routing_request);
        sim_world_service_.PublishRoutingRequestWithoutHeader(
            original_routing_request);

        sim_world_service_.PublishMonitorMessage(
            MonitorMessageItem::INFO,
            "Original routing request restored after temporary move.");
        response["status"] = true;
        websocket_->SendData(conn, response.dump());
      });
}

void SimulationWorldUpdater::RegisterRoutingTaskRequestHandlers() {
  websocket_->RegisterMessageHandler(
      "FineTuningRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.PublishRoutingRequest(json);
      });
  websocket_->RegisterMessageHandler(
      "RequestImmediatelyArrived",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto mcloud_info = std::make_shared<century::mcloud::McloudInfo>();
        mcloud_info->set_immediately_parking(true);
        sim_world_service_.PublishImmediatelyArrived(mcloud_info);
      });

  websocket_->RegisterMessageHandler(
    "SaveCurCarPosition",
    [this](const Json &json, WebSocketHandler::Connection *conn) {
      std::ofstream position("/century/modules/dreamview/position.txt", std::ios::app);
      if (!position.is_open()) {
          AERROR << "dont't open output.txt!";
          return;
      }
      position << json["name"].get<std::string>() << "\n";
      position << "x : " << json["position"]["x"] << "\n";
      position << "y : " << json["position"]["y"] << "\n";
      position << "z : " << json["position"]["z"] << "\n";
      position << "\n";
      position.close();
    });  

  websocket_->RegisterMessageHandler(
      "RequestBypassObstacle",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto borrow_response = std::make_shared<century::planning::BorrowResponse>();
        auto data = json["data"];
        std::string id = data["id"];
        borrow_response->set_has_response(true);
        if (0 == id.compare(0, 7, "stacker")) {
          borrow_response->set_block_obs_id(id.substr(7));
        } else {
          borrow_response->set_block_obs_id(id);
        }
        borrow_response->set_response_type(
            data["agree"] ? century::planning::ResponseType::ACCEPT
                                   : century::planning::ResponseType::REFUSE);
        sim_world_service_.PublishBorrowResponse(borrow_response);

        if (0 == id.compare(0, 7, "stacker")) {
          auto pass_stacker_response = std::make_shared<century::planning::PassStackerResponse>();
          pass_stacker_response->set_has_response(true);
          pass_stacker_response->set_stacker_id(id.substr(7));
          pass_stacker_response->set_pass_stacker_response_type(data["agree"] ? century::planning::PassStackerResponseType::PASS : century::planning::PassStackerResponseType::NOPASS);
          sim_world_service_.PublishPassStackerResponse(pass_stacker_response);
        }
      });

  websocket_->RegisterMessageHandler(
      "AddObstacles",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        virtual_obstacle_ = json;
        obstacle_task_->Init(virtual_obstacle_);
      });

  websocket_->RegisterMessageHandler(
      "SendDeadEndJunctionRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json result = CheckDeadEndJunctionPoints(json);
        if (result.contains("error")) {
          AINFO << result["error"];
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   result["error"]);
        } else {
          auto task = std::make_shared<Task>();
          auto *dead_junction_routing_task =
              task->mutable_dead_end_routing_task();
          bool succeed = ConstructDeadJunctionRoutingTask(
              result, dead_junction_routing_task);
          if (succeed) {
            task->set_task_name("dead_end_junction_routing_task");
            task->set_task_type(
                century::task_manager::TaskType::DEAD_END_ROUTING);
            sim_world_service_.PublishTask(task);
            AINFO << task->DebugString();
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::INFO, "dead junction routing task sent.");
          } else {
            sim_world_service_.PublishMonitorMessage(
                MonitorMessageItem::ERROR,
                "Failed to send a dead junction routing task to task manager "
                "module.");
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SendDefaultCycleRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto task = std::make_shared<Task>();
        auto *cycle_routing_task = task->mutable_cycle_routing_task();
        auto *routing_request = cycle_routing_task->mutable_routing_request();
        if (!ContainsKey(json, "cycleNumber") ||
            !json.find("cycleNumber")->is_number()) {
          AERROR << "Failed to prepare a cycle routing request: Invalid cycle "
                    "number";
          return;
        }
        bool succeed = ConstructRoutingRequest(json, routing_request);
        if (succeed) {
          cycle_routing_task->set_cycle_num(
              static_cast<int>(json["cycleNumber"]));
          task->set_task_name("cycle_routing_task");
          task->set_task_type(century::task_manager::TaskType::CYCLE_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << "The task is : " << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Default cycle routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a default cycle routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendParkingRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto task = std::make_shared<Task>();
        auto *parking_routing_task = task->mutable_parking_routing_task();
        bool succeed = ConstructParkingRoutingTask(json, parking_routing_task);
        if (SendRoutingRequestFromJson(json)) {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
        if (succeed) {
          task->set_task_name("parking_routing_task");
          task->set_task_type(century::task_manager::TaskType::PARKING_ROUTING);
          sim_world_service_.PublishTask(task);
          AINFO << task->DebugString();
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "parking routing task sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a parking routing task to task manager module.");
        }
      });
}

void SimulationWorldUpdater::RegisterSimulationWorldHandlers() {
  websocket_->RegisterMessageHandler(
      "RequestSimulationWorld",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!sim_world_service_.ReadyToPush()) {
          AWARN_EVERY(100)
              << "Not sending simulation world as the data is not ready!";
          return;
        }

        bool enable_pnc_monitor = false;
        auto planning = json.find("planning");
        if (planning != json.end() && planning->is_boolean()) {
          enable_pnc_monitor = json["planning"];
        }
        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = enable_pnc_monitor ? simulation_world_with_planning_data_
                                       : simulation_world_;
        }
        if (FLAGS_enable_update_size_check && !enable_pnc_monitor &&
            to_send.size() > FLAGS_max_update_size) {
          AWARN << "update size is too big:" << to_send.size();
          return;
        }
        websocket_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "RequestSystemInfo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["data"] = sim_world_service_.GetSystemInfo();
        response["type"] = "SystemInfo";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
    "ModifyFasAebInfo",
    [this](const Json &json, WebSocketHandler::Connection *conn) {
      Json response;
      response["data"] = sim_world_service_.ModifyFasAebInfo(json);
      response["type"] = "ModifyFasAebInfoResult";
      std::cout << "response : " << response.dump() << std::endl;
      websocket_->SendData(conn, response.dump());
    });

  websocket_->RegisterMessageHandler(
    "ChangeBckMusicSwitch",
    [this](const Json &json, WebSocketHandler::Connection *conn) {
      sim_world_service_.ChangeBckMusicSwitch(json);
    });

  websocket_->RegisterMessageHandler(
      "RequestRoutePath",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = sim_world_service_.GetRoutePathAsJson();
        response["type"] = "RoutePath";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
    "SelectRoutePath",
    [this](const Json &json, WebSocketHandler::Connection *conn) {
      // Json response = sim_world_service_.GetRoutePathAsJson();
      // response["type"] = "RoutePath";
      // websocket_->SendData(conn, response.dump());
      size_t idx = json["id"];
      sim_world_service_.SendRoutingResponse(idx);
    });

  websocket_->RegisterMessageHandler(
      "GetDefaultEndPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultEndPoint";

        Json poi_list = Json::array();
        if (LoadPOI()) {
          for (const auto &landmark : poi_.landmark()) {
            Json place;
            place["name"] = landmark.name();

            Json parking_info =
                century::common::util::JsonUtil::ProtoToTypedJson(
                    "parkingInfo", landmark.parking_info());
            place["parkingInfo"] = parking_info["data"];

            Json waypoint_list;
            for (const auto &waypoint : landmark.waypoint()) {
              waypoint_list.push_back(GetPointJsonFromLaneWaypoint(waypoint));
            }
            place["waypoint"] = waypoint_list;

            poi_list.push_back(place);
          }
        } else {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to load default "
                                                   "POI. Please make sure the "
                                                   "file exists at " +
                                                       EndWayPointFile());
        }
        response["poi"] = poi_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultRoutings";
        response["threshold"] =
            FLAGS_loop_routing_end_to_start_distance_threshold;

        Json default_routing_list = Json::array();
        if (LoadDefaultRoutings()) {
          for (const auto &landmark : default_routings_.landmark()) {
            Json drouting;
            drouting["name"] = landmark.name();
            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
            }
            drouting["point"] = point_list;
            default_routing_list.push_back(drouting);
          }
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to load default "
              "routing. Please make sure the "
              "file exists at " +
                  DefaultRoutingFile());
        }
        response["defaultRoutings"] = default_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.SetToClear();
        sim_control_->Reset();
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.DumpMessages();
      });
}

void SimulationWorldUpdater::RegisterRecordHandlers() {
  websocket_->RegisterMessageHandler(
      "StartRecordCurrent",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        uint64_t seq_before = 0;
        {
          std::lock_guard<std::mutex> lock(record_status_mutex_);
          seq_before = record_status_seq_;
          record_status_valid_ = false;
          record_status_error_.clear();
        }
        RecordCommand cmd;
        cmd.set_type(RecordCommand::START_RECORD_CURRENT);
        cmd.set_timestamp_ms(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        bool sent = record_cmd_writer_ && record_cmd_writer_->Write(cmd);
        Json response;
        response["type"] = "StartRecordCurrent";
        {
          std::unique_lock<std::mutex> lock(record_status_mutex_);
          record_status_cv_.wait_for(
              lock, std::chrono::milliseconds(kRecordStatusWaitMs),
              [this, seq_before]() { return record_status_seq_ > seq_before; });
          if (record_status_valid_) {
            response["record_status"] = record_status_value_;
            response["status"] = record_status_value_ > 0;
            if (!record_status_error_.empty()) {
              response["error"] = record_status_error_;
            }
          } else {
            response["status"] = false;
            response["error"] = "record status not available";
          }
        }
        if (!sent) {
          response["status"] = false;
          response["error"] = "failed to publish record start request";
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "RecordStatus",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "RecordStatus";
        response["status"] = record_status_value_;
        websocket_->SendData(conn, response.dump());
      });
}

void SimulationWorldUpdater::RegisterControlAndConfigHandlers() {
  websocket_->RegisterMessageHandler(
      "ToggleSimControl",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto enable = json.find("enable");
        if (enable != json.end() && enable->is_boolean()) {
          if (*enable) {
            sim_control_->Start();
          } else {
            sim_control_->Stop();
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SendBarrierCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = sim_world_service_.PublishBarrierCommand(json);
        response["type"] = "BarrierCommandResult";
        if (response.value("status", false)) {
          const bool enabled = response.value("enabled", false);
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO,
              enabled
                  ? absl::StrCat("Barrier command sent: ",
                                 response.value("command", "UNKNOWN"))
                  : "Barrier command stopped.");
          BroadcastBarrierCommandStatus(enabled);
          last_barrier_command_active_ = enabled;
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              absl::StrCat("Failed to send barrier command: ",
                           response.value("error", "unknown error")));
        }
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "RequestDataCollectionProgress",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
        if (monitors) {
          const auto iter = monitors->find("DataCollectionMonitor");
          if (iter != monitors->end() && iter->second->IsEnabled()) {
            Json response;
            response["type"] = "DataCollectionProgress";
            response["data"] = iter->second->GetProgressAsJson();
            websocket_->SendData(conn, response.dump());
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "GetParkingRoutingDistance",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkingRoutingDistance";
        response["threshold"] = FLAGS_parking_routing_distance_threshold;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "RequestPreprocessProgress",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
        if (monitors) {
          const auto iter = monitors->find("PreprocessMonitor");
          if (iter != monitors->end() && iter->second->IsEnabled()) {
            Json response;
            response["type"] = "PreprocessProgress";
            response["data"] = iter->second->GetProgressAsJson();
            websocket_->SendData(conn, response.dump());
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "SaveDefaultRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        bool succeed = AddDefaultRouting(json);
        if (succeed) {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::INFO, "Successfully add default routing.");
          if (!default_routing_) {
            AERROR << "Failed to add a default routing" << std::endl;
          }
          Json response = JsonUtil::ProtoToTypedJson("AddDefaultRoutingPath",
                                                     *default_routing_);
          websocket_->SendData(conn, response.dump());
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to add a default routing.");
        }
      });

      websocket_->RegisterMessageHandler(
        "GetVehicleConfig",
        [this](const Json& json, WebSocketHandler::Connection* conn) {
          Json response;
          GetConfig();
          response["type"] = "VehicleConfigInfo";
          
          for (auto conf : dynamic_confs_.items()) {
            response["config"][conf.key()] = dynamic_confs_[conf.key()]["value"];
          }
          websocket_->SendData(conn, response.dump());
        });
  
    websocket_->RegisterMessageHandler(
        "ModifyVehicleConfig",
        [this](const Json& json, WebSocketHandler::Connection* conn) {
          Json response;
          ModifyConfig(json["config"]);
          response["type"] = "ModifyVehicleConfigResult";
          response["status"] = true;
          websocket_->SendData(conn, response.dump());
        });
}

void SimulationWorldUpdater::RegisterCameraHandlers() {
  camera_ws_->RegisterMessageHandler(
      "RequestCameraData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!perception_camera_updater_->IsEnabled()) {
          return;
        }
        std::string to_send;
        perception_camera_updater_->GetUpdate(&to_send);
        camera_ws_->SendBinaryData(conn, to_send, true);
      });
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
  RegisterConnectionReadyHandlers();
  RegisterMapDataHandlers();
  RegisterRoutingAndTaskHandlers();
  RegisterSimulationWorldHandlers();
  RegisterRecordHandlers();
  RegisterControlAndConfigHandlers();
  RegisterCameraHandlers();
}

Json SimulationWorldUpdater::CheckRoutingPoint(const Json &json) {
  Json result;
  if (!ContainsKey(json, "point")) {
    result["error"] = "Failed to check routing point: point not found.";
    AERROR << result["error"];
    return result;
  }
  auto point = json["point"];
  if (!ValidateCoordinate(point) || !ContainsKey(point, "id")) {
    result["error"] = "Failed to check routing point: invalid point.";
    AERROR << result["error"];
    return result;
  }
  if (!map_service_->CheckRoutingPoint(point["x"], point["y"])) {
    result["pointId"] = point["id"];
    result["error"] = "Selected point cannot be a routing point.";
    AWARN << result["error"];
  }
  return result;
}

Json SimulationWorldUpdater::CheckVersion(const std::string &file_path) {
  Json json_content;
  std::ifstream file_stream(file_path);
  if (file_stream.is_open()) {
    try {
      file_stream >> json_content;
    } catch (const std::exception &e) {
      AERROR << "Failed to parse JSON from file " << file_path 
             << ", error: " << e.what();
    }
  } else {
    AERROR << "Failed to open version info file at " << file_path;
    return Json();
  }

  // Basic validation as before
  if(!ContainsKey(json_content, "composite_version") || 
     !ContainsKey(json_content, "system_name") || 
     !ContainsKey(json_content, "build_type")) {
    AERROR << "Missing required fields in version info file";
    return Json();
  }

  std::string version_str = 
      json_content["system_name"].get<std::string>() + "-" + 
      json_content["build_type"].get<std::string>() + "-" + 
      json_content["composite_version"].get<std::string>();

  Json result;
  result["version"] = version_str;
  return result;
}


Json SimulationWorldUpdater::GetPointJsonFromLaneWaypoint(
    const century::routing::LaneWaypoint &waypoint) {
  Json point;
  point["x"] = waypoint.pose().x();
  point["y"] = waypoint.pose().y();
  if (waypoint.has_heading()) {
    point["heading"] = waypoint.heading();
  }
  return point;
}

Json SimulationWorldUpdater::CheckDeadEndJunctionPoints(const Json &json) {
  Json result;
  if (!ContainsKey(json, "start1")) {
    result["error"] = "Failed to check start point for dead end junction.";
    AERROR << result["error"];
    return result;
  }
  if (!ContainsKey(json, "end2")) {
    result["error"] = "Failed to check end point for dead end junction.";
    AERROR << result["error"];
    return result;
  }
  auto iter = json.find("inLaneIds");
  if (iter == json.end() || !iter->is_array()) {
    result["error"] = "Failed to check start point for dead end junction.";
    return result;
  }
  std::vector<std::string> laneIds;
  auto point = json["start1"];
  for (size_t i = 0; i < iter->size(); ++i) {
    auto &id = (*iter)[i];
    laneIds.push_back(id);
  }
  if (!map_service_->CheckRoutingPointLaneId(point["x"], point["y"], laneIds)) {
    result["error"] = "Error start point for dead end junction.";
  }
  laneIds.clear();
  point = json["end2"];
  iter = json.find("outLaneIds");
  if (iter == json.end() || !iter->is_array()) {
    result["error"] = "Failed to check end point for dead end junction.";
    return result;
  }
  for (size_t i = 0; i < iter->size(); ++i) {
    auto &id = (*iter)[i];
    laneIds.push_back(id);
  }
  if (!map_service_->CheckRoutingPointLaneId(point["x"], point["y"], laneIds)) {
    result["error"] = "Error end point for dead end junction.";
    return result;
  }

  result["routing1"]["start"] = json["start1"];
  result["routing1"]["end"] = json["end1"];
  result["routing2"]["start"] = json["start2"];
  result["routing2"]["end"] = json["end2"];
  result["routing2"]["waypoint"] = json["routingPoint"];
  return result;
}

bool SimulationWorldUpdater::ConstructRoutingRequest(
    const Json &json, RoutingRequest *routing_request) {
  routing_request->clear_waypoint();
  // set start point
  if (!ContainsKey(json, "start")) {
    AERROR << "Failed to prepare a routing request: start point not found.";
    return false;
  }

  if (ContainsKey(json, "isLoopRunning")) {
    bool isLoopRunning = json["isLoopRunning"];
    routing_request->set_is_loop_running(isLoopRunning);
  }

  if (ContainsKey(json, "isOneClickLoopRunning")) {
    bool isOneClickLoopRunning = json["isOneClickLoopRunning"];
    routing_request->set_is_one_click_loop_running(isOneClickLoopRunning);
  }
 
  if (ContainsKey(json, "blacklistedLane")) {
    for (auto &road : json["blacklistedLane"]) {
      std::string road_str = road.get<std::string>();
      boost::algorithm::trim(road_str);
      auto lane = routing_request->add_blacklisted_lane();
      lane->set_id(road_str);
    }
  }

  auto start = json["start"];
  if (!ValidateCoordinate(start)) {
    AERROR << "Failed to prepare a routing request: invalid start point.";
    return false;
  }
  if (ContainsKey(start, "heading")) {
    if (!map_service_->ConstructLaneWayPointWithHeading(
            start["x"], start["y"], start["heading"],
            routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with heading: "
             << start["heading"] << " cannot locate start point on map.";
      return false;
    }
  } else if (ContainsKey(start, "id")) {
    if (!map_service_->ConstructLaneWayPointWithLaneId(
            start["x"], start["y"], start["id"],
            routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with lane id: "
             << start["id"] << " cannot locate end point on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(start["x"], start["y"],
                                             routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate start point on map.";
      return false;
    }
  }

  // set way point(s) if any
  auto iter = json.find("waypoint");
  if (iter != json.end() && iter->is_array()) {
    auto *waypoint = routing_request->mutable_waypoint();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to prepare a routing request: invalid waypoint.";
        return false;
      }

      if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                               waypoint->Add())) {
        AERROR << "Failed to construct a LaneWayPoint, skipping.";
        waypoint->RemoveLast();
      }
    }
  }

  // set end point
  if (!ContainsKey(json, "end")) {
    AERROR << "Failed to prepare a routing request: end point not found.";
    return false;
  }

  auto end = json["end"];
  if (!ValidateCoordinate(end)) {
    AERROR << "Failed to prepare a routing request: invalid end point.";
    return false;
  }
  if (ContainsKey(end, "id")) {
    if (!map_service_->ConstructLaneWayPointWithLaneId(
            end["x"], end["y"], end["id"], routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with lane id: "
             << end["id"] << " cannot locate end point on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                             routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate end point on map.";
      return false;
    }
  }

  if (ContainsKey(end, "type")) {
    routing_request->set_task_type(StringToTaskType(end["type"]));
  }

  // set parking info
  if (ContainsKey(json, "parkingInfo")) {
    auto *requested_parking_info = routing_request->mutable_parking_info();
    if (!JsonStringToMessage(json["parkingInfo"].dump(), requested_parking_info)
             .ok()) {
      AERROR << "Failed to prepare a routing request: invalid parking info."
             << json["parkingInfo"].dump();
      return false;
    }
    if (ContainsKey(json, "cornerPoints")) {
      auto point_iter = json.find("cornerPoints");
      auto *points =
          requested_parking_info->mutable_corner_point()->mutable_point();
      if (point_iter != json.end() && point_iter->is_array()) {
        for (size_t i = 0; i < point_iter->size(); ++i) {
          auto &point = (*point_iter)[i];
          auto *p = points->Add();
          if (!ValidateCoordinate(point)) {
            AERROR << "Failed to add a corner point: invalid corner point.";
            return false;
          }
          p->set_x(static_cast<double>(point["x"]));
          p->set_y(static_cast<double>(point["y"]));
        }
      }
    }
  }

  AINFO << "Constructed RoutingRequest to be sent:\n"
        << routing_request->DebugString();
  century::common::TrajectoryPoint start_pose;

  start_pose.mutable_path_point()->set_x(static_cast<double>(json["start"]["x"]));
  start_pose.mutable_path_point()->set_y(static_cast<double>(json["start"]["y"]));
  if (ContainsKey(start, "z")) {
    start_pose.mutable_path_point()->set_z(
        static_cast<double>(json["start"]["z"]));
  }

  if (ContainsKey(start, "heading")) {
    start_pose.mutable_path_point()->set_theta(
        static_cast<double>(json["start"]["heading"]));
  }

  start_pose.set_a(0.0);
  start_pose.set_v(0.0);

  sim_control_->SetStartPoint(start_pose);

  return true;
}

bool SimulationWorldUpdater::ConstructParkingRoutingTask(
    const Json &json, ParkingRoutingTask *parking_routing_task) {
  auto *routing_request = parking_routing_task->mutable_routing_request();
  // set parking Space
  if (!ContainsKey(json, "laneWidth")) {
    AERROR << "Failed to prepare a parking routing task: "
           << "lane width not found.";
    return false;
  }
  bool succeed = ConstructRoutingRequest(json, routing_request);
  if (succeed) {
    parking_routing_task->set_lane_width(
        static_cast<double>(json["laneWidth"]));
    return true;
  }
  return false;
}

bool SimulationWorldUpdater::ConstructDeadJunctionRoutingTask(
    const Json &json, DeadEndRoutingTask *dead_junction_routing_task) {
  auto *routing_request_in =
      dead_junction_routing_task->mutable_routing_request_in();
  bool succeed = ConstructRoutingRequest(json["routing1"], routing_request_in);
  if (!succeed) {
    AERROR << "Failed to construct the first routing request for dead end "
              "junction routing task";
    return false;
  }
  auto *routing_request_out =
      dead_junction_routing_task->mutable_routing_request_out();
  succeed = ConstructRoutingRequest(json["routing2"], routing_request_out);
  if (!succeed) {
    AERROR << "Failed to construct the second routing request for dead end "
              "junction routing task";
    return false;
  }
  return true;
}

bool SimulationWorldUpdater::ValidateCoordinate(const nlohmann::json &json) {
  if (!ContainsKey(json, "x") || !ContainsKey(json, "y")) {
    AERROR << "Failed to find x or y coordinate.";
    return false;
  }
  if (json.find("x")->is_number() && json.find("y")->is_number()) {
    return true;
  }
  AERROR << "Both x and y coordinate should be a number.";
  return false;
}

void SimulationWorldUpdater::Start() {
  timer_.reset(new cyber::Timer(
      kSimWorldTimeIntervalMs, [this]() { this->OnTimer(); }, false));
  timer_->Start();
}

void SimulationWorldUpdater::OnTimer() {
  sim_world_service_.Update();

  const bool barrier_command_active = sim_world_service_.IsBarrierCommandActive();
  if (last_barrier_command_active_ != barrier_command_active) {
    BroadcastBarrierCommandStatus(barrier_command_active);
    last_barrier_command_active_ = barrier_command_active;
  }

  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    last_pushed_adc_timestamp_sec_ =
        sim_world_service_.world().auto_driving_car().timestamp_sec();
    sim_world_service_.GetWireFormatString(
        FLAGS_sim_map_radius, &simulation_world_,
        &simulation_world_with_planning_data_);
    sim_world_service_.GetRelativeMap().SerializeToString(
        &relative_map_string_);
  }
}

void SimulationWorldUpdater::BroadcastBarrierCommandStatus(bool enabled) {
  Json response;
  response["type"] = "BarrierCommandStatus";
  response["enabled"] = enabled;
  websocket_->BroadcastData(response.dump());
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

bool SimulationWorldUpdater::LoadDefaultRoutings() {
  if (GetProtoFromASCIIFile(DefaultRoutingFile(), &default_routings_)) {
    return true;
  }

  AWARN << "Failed to load default routings of DefaultRoutings from "
        << DefaultRoutingFile();
  return false;
}

bool SimulationWorldUpdater::AddDefaultRouting(const Json &json) {
  if (!ContainsKey(json, "name")) {
    AERROR << "Failed to save a default routing: routing name not found.";
    return false;
  }

  if (!ContainsKey(json, "point")) {
    AERROR << "Failed to save a default routing: default routing points not "
              "found.";
    return false;
  }

  std::string name = json["name"];
  auto iter = json.find("point");
  default_routing_ = default_routings_.add_landmark();
  default_routing_->clear_name();
  default_routing_->clear_waypoint();
  default_routing_->set_name(name);
  auto *waypoint = default_routing_->mutable_waypoint();
  if (iter != json.end() && iter->is_array()) {
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to save a default routing: invalid waypoint.";
        return false;
      }
      auto *p = waypoint->Add();
      auto *pose = p->mutable_pose();
      pose->set_x(static_cast<double>(point["x"]));
      pose->set_y(static_cast<double>(point["y"]));
      if (ContainsKey(point, "heading")) {
        p->set_heading(point["heading"]);
      }
    }
  }
  AINFO << "Default Routing Points to be saved:\n";
  std::string file_name = DefaultRoutingFile();
  if (!SetProtoToASCIIFile(default_routings_, file_name)) {
    AERROR << "Failed to set proto to ascii file " << file_name;
    return false;
  }
  AINFO << "Success in setting proto to cycle_routing file" << file_name;

  return true;
}

century::routing::TaskType SimulationWorldUpdater::StringToTaskType(
    const std::string &task_type_str) {
  // static const std::unordered_map<std::string, century::routing::TaskType>
  //     string_to_enum = {
  //         {"DEFAULT", century::routing::TaskType::DEFAULT},
  //         {"PARKINGSPACE", century::routing::TaskType::PARKINGSPACE},
  //         {"RAILWAY_WAITINGAREA_STATIC",
  //          century::routing::TaskType::RAILWAY_WAITINGAREA_STATIC},
  //         {"RAILWAY_WAITINGAREA_DYNAMIC",
  //          century::routing::TaskType::RAILWAY_WAITINGAREA_DYNAMIC},
  //         {"RAILWAY_OPERATIONAREA_DYNAMIC",
  //          century::routing::TaskType::RAILWAY_OPERATIONAREA_DYNAMIC},
  //         {"LOADING_OPERATIONAREA_SAMEDIRECTION_1",
  //          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_1},
  //         {"LOADING_OPERATIONAREA_SAMEDIRECTION_2",
  //          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_2},
  //         {"LOADING_OPERATIONAREA_SAMEDIRECTION_3_0",
  //          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0},
  //         {"LOADING_OPERATIONAREA_SAMEDIRECTION_3_1",
  //          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1},
  //         {"LOADING_OPERATIONAREA_DIFFDIRECTION_1",
  //          century::routing::TaskType::LOADING_OPERATIONAREA_DIFFDIRECTION_1},
  //         {"YARD_WAITINGAREA_STATIC",
  //          century::routing::TaskType::YARD_WAITINGAREA_STATIC},
  //         {"YARD_OPERATIONAREA_STATIC",
  //          century::routing::TaskType::YARD_OPERATIONAREA_STATIC},
  //         {"YARD_OPERATIONAREA_DYNAMIC",
  //          century::routing::TaskType::YARD_OPERATIONAREA_DYNAMIC},
  //         {"UNLOAD_OPERATIONAREA_SAMEDIRECTION_1",
  //          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1},
  //         {"UNLOAD_OPERATIONAREA_SAMEDIRECTION_2",
  //          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2},
  //         {"UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0",
  //          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0},
  //         {"UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1",
  //          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1},
  //         {"UNLOAD_OPERATIONAREA_DIFFDIRECTION_1",
  //          century::routing::TaskType::UNLOAD_OPERATIONAREA_DIFFDIRECTION_1},
  //          {"BACKWARD_ROUTING_NEED_PLANNING_REROUTING",
  //         century::routing::TaskType::BACKWARD_ROUTING_NEED_PLANNING_REROUTING},
  //         {"BACKWARD_ROUTING_DIRECTLY",
  //           century::routing::TaskType::BACKWARD_ROUTING_DIRECTLY},
  //         {"LONG_ADJUSTMENT_FRONT",
  //           century::routing::TaskType::LONG_ADJUSTMENT_FRONT},
  //         {"LONG_ADJUSTMENT_BACK",
  //           century::routing::TaskType::LONG_ADJUSTMENT_BACK}
  //     };
  century::routing::TaskType type;
  century::routing::TaskType_Parse(task_type_str, &type);

  return type;
}

void SimulationWorldUpdater::readConfigFile(const std::string &file_path,
                    std::unordered_set<std::string> &variables,
                    std::unordered_map<std::string, std::string> &config_map) {
  std::ifstream conf_file(file_path);
  if (!conf_file.is_open()) {
    return;
  }

  std::string line;
  while (std::getline(conf_file, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    std::string key, value;
    if (std::getline(iss, key, '=') && std::getline(iss, value)) {
      if (key.rfind("--", 0) == 0) {
        key.erase(0, 2);
      }

      if (variables.count(key)) {
        config_map[key] = value;
      }
    }
  }
}

void System(std::string_view cmd) {
  const int ret = std::system(cmd.data());
  if (ret == 0) {
    AINFO << "SUCCESS: " << cmd;
  } else {
    AERROR << "FAILED(" << ret << "): " << cmd;
  }
}

void SimulationWorldUpdater::GetConfig() {
  std::ifstream file(FLAGS_dynamic_file_path);
  if (!file.is_open()) {
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  Json configs_info = Json::parse(buffer.str());
  std::cout << configs_info.dump() << std::endl;
  for (const auto &config_block : configs_info["dynamic_conf"]) {
    std::string config_file_path = config_block["path"];
    std::string type = config_block["type"];
    bool restart = config_block["restart"];

    std::unordered_set<std::string> variables;
    for (const auto& item : config_block["variable"]) {
        variables.insert(item.get<std::string>());
    }

    std::unordered_map<std::string, std::string> config_map;

    readConfigFile(config_file_path, variables, config_map);

    for (const auto &element : config_map) {
      dynamic_confs_[element.first]["value"] = element.second;
      dynamic_confs_[element.first]["type"] = type;
      dynamic_confs_[element.first]["restart"] = restart;
      dynamic_confs_[element.first]["path"] = config_file_path;
    }
  }
}

void SimulationWorldUpdater::ModifyConfig(const nlohmann::json &json) {
  bool restart = false;
  for (auto conf : json.items()) {
    auto key = conf.key();
    std::string value = conf.value(); 
    auto type = dynamic_confs_[key]["type"];
    auto file_path = dynamic_confs_[key]["path"];
    restart |= dynamic_confs_[key]["restart"].get<bool>();
    if (type == "gflags") {
      std::ofstream file(file_path, std::ios::app);
      file << "--" << key << "=" << value << std::endl;
    }
  }

  if (restart) {
    System("pkill -9 -f planning");
  }
}

bool SimulationWorldUpdater::SendRoutingRequestFromJson(const Json &json) {
  sim_world_service_.ClearDestinationPosition();
  auto routing_request = std::make_shared<RoutingRequest>();
  if (!ConstructRoutingRequest(json, routing_request.get())) {
    return false;
  }

  if (json.contains("end") && json["end"].contains("type") &&
      "SEARCH_REACH_STACKER" == json["end"]["type"]) {
    SetDestinationTriggerPoint(*routing_request);
  }
  sim_world_service_.PublishRoutingRequest(routing_request);
  obstacle_task_->Init(virtual_obstacle_);
  return true;
}

void SimulationWorldUpdater::SetDestinationTriggerPoint(
    const RoutingRequest &routing_request) {
  if (0 == routing_request.waypoint_size()) {
    return;
  }

  const auto &destination_waypoint =
      routing_request.waypoint(routing_request.waypoint_size() - 1);
  century::common::PointENU destination_position;
  destination_position.set_x(destination_waypoint.pose().x());
  destination_position.set_y(destination_waypoint.pose().y());
  destination_position.set_z(destination_waypoint.pose().z());
  sim_world_service_.SetDestinationPosition(destination_position);
}

}  // namespace dreamview
}  // namespace century
