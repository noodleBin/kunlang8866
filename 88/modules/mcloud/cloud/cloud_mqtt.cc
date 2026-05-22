/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/mcloud/cloud/cloud_mqtt.h"

#include <cstring>
#include <sstream>

#include <mosquitto.h>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/mcloud/cloud/convert.h"
#include "modules/dreamview/proto/barrier_command.pb.h"
#include "modules/mcloud/common/cloud_gflags.h"

namespace century {
namespace mcloud {

namespace {
constexpr int64_t kReportTickMs = 100;
constexpr int64_t kNotifyRetryTickMs = 3000;
constexpr int64_t kDefaultStackerTimeoutSec = 2;
constexpr int64_t kBarrierOfflineTimeoutSec = 5;
constexpr uint32_t kReportInfoSampleEveryN = 50;
constexpr const char* kBoomBarrierTopicWildcard = "/boom_barrier/+/status";
constexpr const char* kVehicleDataTopicPrefix = "vehicle/data/";
constexpr const char* kBarrierIndexJ1E = "J1_E";
constexpr const char* kBarrierIndexJ1W = "J1_W";
constexpr const char* kBarrierIndexJ4E = "J4_E";
constexpr const char* kBarrierIndexJ4W = "J4_W";

bool IsValidQosInt(int qos) { return qos >= 0 && qos <= 2; }

MqttQos ToQosWithFallback(int raw_qos, MqttQos fallback) {
  if (!IsValidQosInt(raw_qos)) {
    return fallback;
  }
  return static_cast<MqttQos>(raw_qos);
}

}  // namespace

CloudMqtt::CloudMqtt()
    : mqtt_client_(
          std::make_unique<MqttClient>("mcloud_mqtt_" + FLAGS_vehicle_id)),
      node_(cyber::CreateNode("cloud_mqtt_broadcast")) {
  gflags::ReadFromFlagsFile(FLAGS_cloud_server_conf, "cloud", true);

  offset_x_ = FLAGS_offset_x;
  offset_y_ = FLAGS_offset_y;
  zone_id_ = FLAGS_zone_id;

  v2x_info_writer_ =
      node_->CreateWriter<century::planning::V2xInfo>("/century/v2x_info");
  stacker_info_writer_ = node_->CreateWriter<century::planning::StackersInfo>(
      "/century/stackers_info");
  borrow_writer_ = node_->CreateWriter<century::planning::BorrowResponse>(
      "/century/borrow_response");
  pass_stacker_writer_ =
      node_->CreateWriter<century::planning::PassStackerResponse>(
          "/century/pass_stacker_response");
  barrier_writer_ =
      node_->CreateWriter<century::planning::Barrier>("/century/barrier");

  chassis_reader_ =
      node_->CreateReader<century::canbus::Chassis>(FLAGS_chassis_topic);
  routing_response_request_reader_ =
      node_->CreateReader<century::routing::RoutingResponse>(
          FLAGS_routing_response_topic,
          [this](
              const std::shared_ptr<century::routing::RoutingResponse>& msg) {
            if (!msg || !msg->has_routing_request()) {
              return;
            }
            CacheRoutingRequest(msg->routing_request());
          });
  localization_reader_ =
      node_->CreateReader<century::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  planning_reader_ = node_->CreateReader<century::planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<century::planning::ADCTrajectory>& result) {
        OnPlanningRequest(result);
      });
  top_bull_reader_ = node_->CreateReader<century::planning::TopBullInfo>(
      "/century/top_bull_info");
  barrier_command_reader_ =
      node_->CreateReader<century::dreamview::BarrierCommand>(
          "/century/barrier_command");

  last_stacker_info_time_ = std::chrono::steady_clock::now();
}

CloudMqtt::~CloudMqtt() { Stop(); }

bool CloudMqtt::BuildMqttOptionsLocked(MqttOptions* options) const {
  if (nullptr == options) {
    return false;
  }
  options->host = FLAGS_cloud_mqtt_server_ip;
  options->port = FLAGS_cloud_mqtt_server_port;
  options->username = FLAGS_cloud_mqtt_username;
  options->password = FLAGS_cloud_mqtt_password;
  return true;
}

bool CloudMqtt::ConnectAndSubscribeLocked(const MqttOptions& options) {
  if (!mqtt_client_->Connect(options)) {
    AERROR << "[CloudMqtt] Failed to connect mqtt broker " << options.host
           << ":" << options.port;
    return false;
  }

  InitWriterTopicsLocked();
  InitQosConfigLocked();

  if (!InitReaderTopicsLocked() || !RegisterInternalTopicsLocked()) {
    mqtt_client_->Disconnect();
    return false;
  }

  for (const auto& kv : registered_topics_) {
    const std::string& topic = kv.first;
    const auto qos = ResolveSubQos(topic);
    const bool ok = mqtt_client_->SubscribeJson(
        topic,
        [this](const std::string& recv_topic, const nlohmann::json& payload) {
          this->OnMqttMessage(recv_topic, payload);
        },
        qos);
    if (!ok) {
      AERROR << "[CloudMqtt] Failed to subscribe topic: " << topic;
      mqtt_client_->Disconnect();
      return false;
    }
    AINFO << "[CloudMqtt] Subscribed topic: " << topic;
  }

  return true;
}

bool CloudMqtt::StartReportThreadLocked() {
  if (report_running_) {
    return true;
  }
  report_running_ = true;
  report_thread_ =
      std::make_unique<std::thread>(&CloudMqtt::HandReportTaskLoop, this);
  return true;
}

void CloudMqtt::StopReportThreadUnlocked() {
  if (report_thread_ && report_thread_->joinable()) {
    report_thread_->join();
  }
  report_thread_.reset();
}

bool CloudMqtt::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) {
    return true;
  }

  MqttOptions options;
  if (!BuildMqttOptionsLocked(&options)) {
    AERROR << "[CloudMqtt] Failed to build MQTT options.";
    return false;
  }

  if (!ConnectAndSubscribeLocked(options)) {
    return false;
  }

  if (!StartReportThreadLocked()) {
    mqtt_client_->Disconnect();
    return false;
  }

  started_ = true;
  return true;
}

void CloudMqtt::Stop() {
  AINFO << "[CloudMqtt] Stop begin";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      AINFO << "[CloudMqtt] Stop skip: not started";
      return;
    }

    if (mqtt_client_) {
      mqtt_client_->Disconnect();
    }
    started_ = false;
    report_running_ = false;
  }

  // Join outside the mutex to avoid deadlock with report thread calling
  // IsRunning(), which also takes mutex_.
  StopReportThreadUnlocked();
  AINFO << "[CloudMqtt] Stop end";
}

bool CloudMqtt::WriteVehicleData(const nlohmann::json& payload) {
  std::string topic;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (writer_vehicle_data_topic_.empty()) {
      AERROR << "[CloudMqtt] vehicle/data writer topic is empty.";
      return false;
    }
    topic = writer_vehicle_data_topic_;
  }
  return PublishJson(topic, payload, ResolvePubQos(topic));
}

bool CloudMqtt::IsRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return started_;
}

std::vector<std::string> CloudMqtt::SubscribedTopics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> topics;
  topics.reserve(registered_topics_.size());
  for (const auto& kv : registered_topics_) {
    topics.emplace_back(kv.first);
  }
  return topics;
}

bool CloudMqtt::InitReaderTopicsLocked() {
  registered_topics_.clear();
  return true;
}

void CloudMqtt::InitWriterTopicsLocked() {
  writer_vehicle_data_topic_ = BuildVehicleDataTopic();
}

void CloudMqtt::InitQosConfigLocked() {
  sub_qos_default_ =
      ToQosWithFallback(FLAGS_cloud_mqtt_sub_qos_default,
                        MqttQos::kAtLeastOnce);
  pub_qos_default_ =
      ToQosWithFallback(FLAGS_cloud_mqtt_pub_qos_default,
                        MqttQos::kAtLeastOnce);

  sub_qos_overrides_ = ParseQosOverrides(FLAGS_cloud_mqtt_sub_qos_overrides);
  pub_qos_overrides_ = ParseQosOverrides(FLAGS_cloud_mqtt_pub_qos_overrides);
}

std::unordered_map<std::string, MqttQos> CloudMqtt::ParseQosOverrides(
    const std::string& raw_json) {
  std::unordered_map<std::string, MqttQos> overrides;
  auto parsed = nlohmann::json::parse(raw_json, nullptr, false);
  if (parsed.is_discarded() || !parsed.is_object()) {
    return overrides;
  }

  for (auto it = parsed.begin(); it != parsed.end(); ++it) {
    if (!it.value().is_number_integer()) {
      continue;
    }
    const int qos_value = it.value().get<int>();
    if (!IsValidQosInt(qos_value)) {
      continue;
    }
    overrides[it.key()] = static_cast<MqttQos>(qos_value);
  }
  return overrides;
}

MqttQos CloudMqtt::ResolveSubQos(const std::string& topic) const {
  const auto it = sub_qos_overrides_.find(topic);
  if (it != sub_qos_overrides_.end()) {
    return it->second;
  }
  return sub_qos_default_;
}

MqttQos CloudMqtt::ResolvePubQos(const std::string& topic) const {
  const auto it = pub_qos_overrides_.find(topic);
  if (it != pub_qos_overrides_.end()) {
    return it->second;
  }
  return pub_qos_default_;
}

bool CloudMqtt::RegisterInternalTopicsLocked() {
  const std::string broadcast_topic = BuildVehicleBroadcastTopic();
  const std::string data_topic = BuildVehicleDataWildcardTopic();
  const std::string reply_topic = BuildVehicleReplyTopic();
  if (broadcast_topic.empty()) {
    AERROR
        << "[CloudMqtt] vehicle_id is empty, mqtt topic registration failed.";
    return false;
  }

  registered_topics_[broadcast_topic] =
      TopicEntry{[this](const std::string& topic,
                        const nlohmann::json& payload) {
                   HandleVehicleBroadcast(topic, payload);
                 },
                 MqttQos::kAtLeastOnce};
  registered_topics_[data_topic] =
      TopicEntry{[this](const std::string& topic,
                        const nlohmann::json& payload) {
                   HandleVehicleData(topic, payload);
                 },
                 MqttQos::kAtLeastOnce};
  registered_topics_[reply_topic] =
      TopicEntry{[this](const std::string& topic,
                        const nlohmann::json& payload) {
                   HandleVehicleReply(topic, payload);
                 },
                 MqttQos::kAtLeastOnce};

  registered_topics_[kBoomBarrierTopicWildcard] =
      TopicEntry{[this](const std::string& recv_topic,
                        const nlohmann::json& payload) {
                   HandleBoomBarrierStatus(recv_topic, payload);
                 },
                 MqttQos::kAtLeastOnce};
  return true;
}

bool CloudMqtt::ParseVehicleArrayItem(const nlohmann::json& in,
                                      VehicleBroadcastItem* out) {
  if (nullptr == out) {
    return false;
  }
  if (!in.contains("vehicleType")) {
    AERROR << "[CloudMqtt] Missing vehicleType field in vehicle_json";
    return false;
  }

  out->vehicle_type = in["vehicleType"].get<uint32_t>();
  if (1U == out->vehicle_type) {
    if (!in.contains("vehicleId") || !in.contains("taskType") ||
        !in.contains("speed") || !in.contains("driveMode") ||
        !in.contains("lng") || !in.contains("lat") ||
        !in.contains("heading") || !in.contains("gear") ||
        !in.contains("timestamp")) {
      AERROR << "[CloudMqtt] Missing required fields for VehicleInfo";
      return false;
    }
    out->vehicle_id = in["vehicleId"].get<std::string>();
    out->task_type = in["taskType"].get<uint32_t>();
    out->speed = in["speed"].get<float_t>();
    out->drive_mode = in["driveMode"].get<uint32_t>();
    out->lng = in["lng"].get<double>();
    out->lat = in["lat"].get<double>();
    out->heading = in["heading"].get<float_t>();
    out->gear = in["gear"].get<uint32_t>();
    out->timestamp_ms = in["timestamp"].get<long>();
    if (in.contains("topBull") && in["topBull"].is_object()) {
      out->top_bull = in["topBull"];
    }
    return true;
  }

  if (2U == out->vehicle_type || 4U == out->vehicle_type) {
    if (!in.contains("vehicleId") || !in.contains("lng") ||
        !in.contains("lat") || !in.contains("heading") ||
        !in.contains("speed") || !in.contains("taskType")) {
      AERROR << "[CloudMqtt] Missing required fields for StackerInfo";
      return false;
    }
    out->vehicle_id = in["vehicleId"].get<std::string>();
    out->lng = in["lng"].get<double>();
    out->lat = in["lat"].get<double>();
    out->heading = in["heading"].get<float_t>();
    out->speed = in["speed"].get<double>();
    out->task_type = in["taskType"].get<uint32_t>();
    out->timestamp_ms = in["timestamp"].get<long>();
    return true;
  }

  return false;
}

void CloudMqtt::CacheRoutingRequest(
    const century::routing::RoutingRequest& request) {
  std::lock_guard<std::mutex> lock(routing_request_mutex_);
  last_routing_request_ = request;
  has_cached_routing_request_ = true;
}

bool CloudMqtt::GetCachedRoutingRequest(
    century::routing::RoutingRequest* request) const {
  if (nullptr == request) {
    return false;
  }

  std::lock_guard<std::mutex> lock(routing_request_mutex_);
  if (!has_cached_routing_request_) {
    return false;
  }
  *request = last_routing_request_;
  return true;
}

bool CloudMqtt::GetCachedVehicleData(const std::string& vehicle_id,
                                     VehicleBroadcastItem* out) const {
  if (nullptr == out || vehicle_id.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(vehicle_data_mutex_);
  const auto it = vehicle_data_cache_.find(vehicle_id);
  if (it == vehicle_data_cache_.end()) {
    return false;
  }
  *out = it->second;
  return true;
}

void CloudMqtt::ConvertWgs84ToLocal(double lng, double lat, double* x,
                                    double* y) const {
  char zone[20];
  Wgs84toUtm(lng * DEG_TO_RAD, lat * DEG_TO_RAD, x, y, nullptr, nullptr, zone);
  *x -= offset_x_;
  *y -= offset_y_;
}

void CloudMqtt::FillVehicleInfoProto(const VehicleBroadcastItem& in,
                                     century::planning::VehicleInfo* out) {
  if (nullptr == out) {
    return;
  }
  VehicleBroadcastItem effective = in;
  VehicleBroadcastItem vehicle_data;
  if (GetCachedVehicleData(in.vehicle_id, &vehicle_data)) {
    effective.task_type = vehicle_data.task_type;
    effective.speed = vehicle_data.speed;
    effective.drive_mode = vehicle_data.drive_mode;
    effective.lng = vehicle_data.lng;
    effective.lat = vehicle_data.lat;
    effective.heading = vehicle_data.heading;
    effective.gear = vehicle_data.gear;
    effective.timestamp_ms = vehicle_data.timestamp_ms;
    if (!vehicle_data.top_bull.empty()) {
      effective.top_bull = vehicle_data.top_bull;
    }
  }

  out->set_id(effective.vehicle_id);
  out->set_task_type(
      static_cast<century::routing::TaskType>(effective.task_type));
  out->set_speed(effective.speed);
  out->set_driving_mode(
      static_cast<century::canbus::Chassis::DrivingMode>(effective.drive_mode));

  double x = 0.0;
  double y = 0.0;
  ConvertWgs84ToLocal(effective.lng, effective.lat, &x, &y);
  out->set_x(x);
  out->set_y(y);

  out->set_heading(effective.heading);
  out->set_gear(
      static_cast<century::canbus::Chassis::GearPosition>(effective.gear));
  out->mutable_header()->set_timestamp_sec(
      static_cast<double>(effective.timestamp_ms) / 1000.0);

  // Prefer the latest vehicle/data/<vehicle_id> payload when rebuilding
  // VehicleInfo, and fall back to the broadcast JSON when data has not arrived.
  if (!effective.top_bull.empty() && effective.top_bull.is_object()) {
    const auto find_top_bull_field =
        [&effective](const char* primary_key,
                     const char* fallback_key) -> nlohmann::json::const_iterator {
      auto it = effective.top_bull.find(primary_key);
      if (it != effective.top_bull.end()) {
        return it;
      }
      return effective.top_bull.find(fallback_key);
    };
    const auto is_load_it = effective.top_bull.find("is_load");
    if (is_load_it != effective.top_bull.end()) {
      if (is_load_it->is_boolean()) {
        out->set_is_loading(is_load_it->get<bool>());
      } else if (is_load_it->is_number_integer()) {
        out->set_is_loading(0 != is_load_it->get<int>());
      }
    }
    const auto is_in_top_bull_it =
        find_top_bull_field("isInTopBull", "is_in_top_bull");
    if (is_in_top_bull_it != effective.top_bull.end()) {
      if (is_in_top_bull_it->is_boolean()) {
        out->set_is_in_top_bull(is_in_top_bull_it->get<bool>());
      } else if (is_in_top_bull_it->is_number_integer()) {
        out->set_is_in_top_bull(0 != is_in_top_bull_it->get<int>());
      }
    }
    if (effective.top_bull.contains("pathInfo") &&
        effective.top_bull["pathInfo"].is_array()) {
      for (const auto& pi : effective.top_bull["pathInfo"]) {
        auto* p = out->add_path_info();
        if (pi.contains("x")) p->set_x(pi["x"].get<double>());
        if (pi.contains("y")) p->set_y(pi["y"].get<double>());
        if (pi.contains("theta")) p->set_theta(pi["theta"].get<double>());
      }
    }
    if (effective.top_bull.contains("turnType")) {
      out->set_turn_type(
          static_cast<century::planning::TurnType>(
              effective.top_bull["turnType"].get<int>()));
    }
    if (effective.top_bull.contains("randomNumber")) {
      out->set_random_number(
          effective.top_bull["randomNumber"].get<uint64_t>());
    }
    const auto top_bull_type_it =
        find_top_bull_field("topBullType", "top_bull_type");
    if (top_bull_type_it != effective.top_bull.end()) {
      int top_bull_type = static_cast<int>(century::planning::TB_NONE);
      bool parsed = false;
      if (top_bull_type_it->is_number_integer()) {
        top_bull_type = top_bull_type_it->get<int>();
        parsed = true;
      } else if (top_bull_type_it->is_string()) {
        const auto& top_bull_type_name =
            top_bull_type_it->get_ref<const std::string&>();
        if ("TB_NONE" == top_bull_type_name) {
          top_bull_type = static_cast<int>(century::planning::TB_NONE);
          parsed = true;
        } else if ("TB_WAITING" == top_bull_type_name) {
          top_bull_type = static_cast<int>(century::planning::TB_WAITING);
          parsed = true;
        } else if ("TB_BORROW" == top_bull_type_name) {
          top_bull_type = static_cast<int>(century::planning::TB_BORROW);
          parsed = true;
        } else if ("TB_REVERSE" == top_bull_type_name) {
          top_bull_type = static_cast<int>(century::planning::TB_REVERSE);
          parsed = true;
        }
      }
      if (parsed &&
          top_bull_type >= static_cast<int>(century::planning::TB_NONE) &&
          top_bull_type <= static_cast<int>(century::planning::TB_REVERSE)) {
        out->set_top_bull_type(
            static_cast<century::planning::TopBullType>(top_bull_type));
      }
    }
  }
}

void CloudMqtt::FillStackerInfoProto(const VehicleBroadcastItem& in,
                                     century::planning::StackerInfo* out) {
  if (nullptr == out) {
    return;
  }
  out->set_stacker_id(in.vehicle_id);

  double x = 0.0;
  double y = 0.0;
  ConvertWgs84ToLocal(in.lng, in.lat, &x, &y);
  out->mutable_stacker_point()->set_x(x);
  out->mutable_stacker_point()->set_y(y);
  out->mutable_stacker_point()->set_heading(in.heading);
  out->set_speed(in.speed);

  static const std::unordered_map<uint32_t, century::planning::StackerType>
      k_vehicle_type_map = {
          {2, century::planning::StackerType::STACKER},
          {4, century::planning::StackerType::WHEELCRANE}};
  auto type_it = k_vehicle_type_map.find(in.vehicle_type);
  if (type_it != k_vehicle_type_map.end()) {
    out->set_stacker_type(type_it->second);
  }

  out->set_task_type(static_cast<century::routing::TaskType>(in.task_type));
  out->mutable_header()->set_timestamp_sec(static_cast<double>(in.timestamp_ms) /
                                           1000.0);
}

void CloudMqtt::PublishBroadcastProtos(
    century::planning::V2xInfo* v2x_info,
    century::planning::StackersInfo* stackers_info) {
  if (v2x_info != nullptr && v2x_info->vehicle_info_size() > 0) {
    century::common::util::FillHeader("v2x_info", v2x_info);
    if (v2x_info_writer_) {
      v2x_info_writer_->Write(*v2x_info);
    }
  }

  if (stackers_info != nullptr && stackers_info->stacker_info_size() > 0) {
    century::common::util::FillHeader("stackers_info", stackers_info);
    if (stacker_info_writer_) {
      stacker_info_writer_->Write(*stackers_info);
    }
    last_stacker_info_time_ = std::chrono::steady_clock::now();
  }
}

void CloudMqtt::HandleBroadcastNotifyObject(const nlohmann::json& obj) {
  century::planning::BorrowResponse borrow_response;
  century::planning::PassStackerResponse pass_stacker_response;

  const std::string target_no = obj.value("targetNo", "");
  const uint32_t operate = obj.value("operate", 0U);
  const long timestamp = obj.value("timestamp", 0L);

  borrow_response.set_block_obs_id(target_no);
  borrow_response.set_has_response(1);
  borrow_response.set_response_type(
      1 == operate ? century::planning::ResponseType::ACCEPT
                   : century::planning::ResponseType::REFUSE);
  borrow_response.mutable_header()->set_timestamp_sec(timestamp);

  pass_stacker_response.set_stacker_id(target_no);
  pass_stacker_response.set_has_response(1);
  pass_stacker_response.set_pass_stacker_response_type(
      1 == operate ? century::planning::PassStackerResponseType::PASS
                   : century::planning::PassStackerResponseType::NOPASS);
  pass_stacker_response.mutable_header()->set_timestamp_sec(timestamp);

  if (borrow_writer_) {
    borrow_writer_->Write(borrow_response);
  }
  if (pass_stacker_writer_) {
    pass_stacker_writer_->Write(pass_stacker_response);
  }
}

void CloudMqtt::HandleVehicleBroadcast(const std::string& topic,
                                       const nlohmann::json& json_data) {
  (void)topic;

  if (json_data.is_array()) {
    century::planning::V2xInfo v2x_info;
    century::planning::StackersInfo stackers_info;

    for (const auto& vehicle_json : json_data) {
      VehicleBroadcastItem item;
      if (!ParseVehicleArrayItem(vehicle_json, &item)) {
        continue;
      }

      if (1U == item.vehicle_type) {
        auto* vehicle_info = v2x_info.add_vehicle_info();
        FillVehicleInfoProto(item, vehicle_info);
      } else if (2U == item.vehicle_type || 4U == item.vehicle_type) {
        auto* stacker_info = stackers_info.add_stacker_info();
        FillStackerInfoProto(item, stacker_info);
      }
    }

    PublishBroadcastProtos(&v2x_info, &stackers_info);
    return;
  }

  if (json_data.is_object()) {
    HandleBroadcastNotifyObject(json_data);
    return;
  }

  AWARN << "[CloudMqtt] vehicle/broadcast expects JSON array/object payload.";
}

void CloudMqtt::HandleVehicleData(const std::string& topic,
                                  const nlohmann::json& json_data) {
  if (!json_data.is_object()) {
    AWARN << "[CloudMqtt] vehicle/data expects JSON object payload, topic="
          << topic;
    return;
  }
  if (topic.rfind(kVehicleDataTopicPrefix, 0) != 0 ||
      topic.size() <= std::strlen(kVehicleDataTopicPrefix)) {
    AWARN << "[CloudMqtt] invalid vehicle/data topic=" << topic;
    return;
  }

  const std::string topic_vehicle_id =
      topic.substr(std::strlen(kVehicleDataTopicPrefix));
  nlohmann::json normalized_payload = json_data;
  const std::string payload_vehicle_id =
      normalized_payload.value("vehicleId", std::string());
  if (payload_vehicle_id.empty()) {
    normalized_payload["vehicleId"] = topic_vehicle_id;
  } else if (payload_vehicle_id != topic_vehicle_id) {
    AWARN << "[CloudMqtt] vehicle/data vehicleId mismatch, topic=" << topic
          << " payload_vehicle_id=" << payload_vehicle_id
          << ", use topic vehicle id";
    normalized_payload["vehicleId"] = topic_vehicle_id;
  }

  VehicleBroadcastItem item;
  if (!ParseVehicleArrayItem(normalized_payload, &item)) {
    AWARN << "[CloudMqtt] vehicle/data payload parse failed, topic=" << topic;
    return;
  }
  if (1U != item.vehicle_type) {
    return;
  }

  std::lock_guard<std::mutex> lock(vehicle_data_mutex_);
  vehicle_data_cache_[item.vehicle_id] = std::move(item);
}

void CloudMqtt::HandleVehicleReply(const std::string& topic,
                                   const nlohmann::json& json_data) {
  (void)topic;
  if (!json_data.is_object()) {
    AWARN << "[CloudMqtt] vehicle/reply expects JSON object payload.";
    return;
  }

  if (!json_data.contains("msgId") || !json_data.contains("status") ||
      !json_data.contains("msg") || !json_data.contains("timestamp")) {
    AWARN << "[CloudMqtt] vehicle/reply missing required fields "
             "(msgId/status/msg/timestamp).";
    return;
  }

  const std::string msg_id = json_data.value("msgId", "");
  const int status = json_data.value("status", 0);
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_data_.erase(msg_id);
  }

  std::vector<std::string> parts;
  parts.reserve(4);
  std::stringstream ss(msg_id);
  std::string item;
  while (std::getline(ss, item, '_')) {
    parts.emplace_back(item);
  }
  if (parts.size() >= 2 && "P" == parts[1] && 2 == status) {
    century::planning::PassStackerResponse pass_stacker_response;
    century::common::util::FillHeader("cloud_mqtt", &pass_stacker_response);
    pass_stacker_response.set_stacker_id(parts[0]);
    pass_stacker_response.set_has_response(1);
    pass_stacker_response.set_pass_stacker_response_type(
        century::planning::PassStackerResponseType::PASS);
    if (pass_stacker_writer_) {
      pass_stacker_writer_->Write(pass_stacker_response);
    }
  }
}

void CloudMqtt::HandleBoomBarrierStatus(const std::string& topic,
                                        const nlohmann::json& json_data) {
  if (!json_data.is_object()) {
    AWARN << "[CloudMqtt] boom barrier payload must be JSON object, topic="
          << topic;
    return;
  }
  if (!json_data.contains("arm_status")) {
    AWARN << "[CloudMqtt] boom barrier payload missing required fields "
             "(arm_status), topic="
          << topic;
    return;
  }

  const std::string barrier_index = ResolveBarrierIndex(topic, json_data);
  if (barrier_index.empty()) {
    AWARN << "[CloudMqtt] cannot resolve boom_barrier_index, topic=" << topic;
    return;
  }
  const int arm_status = json_data.value("arm_status", 0);
  const bool opened = (1 == arm_status);

  {
    std::lock_guard<std::mutex> lock(barrier_mutex_);
    barrier_status_cache_[barrier_index] = opened;
    const auto now = std::chrono::steady_clock::now();
    barrier_last_seen_[barrier_index] = now;
    RecomputeBarrierStateLocked(now);
  }

  ADEBUG << "[CloudMqtt] boom barrier cache update topic=" << topic
         << " index=" << barrier_index << " arm_status=" << arm_status
         << " opened=" << opened;
}

std::string CloudMqtt::ResolveBarrierIndex(const std::string& topic,
                                           const nlohmann::json& payload) const {
  const std::string from_payload =
      payload.value("boom_barrier_index", std::string());
  if (!from_payload.empty()) {
    return from_payload;
  }

  // Expected format: /boom_barrier/<INDEX>/status
  constexpr const char* kPrefix = "/boom_barrier/";
  constexpr const char* kSuffix = "/status";
  if (topic.rfind(kPrefix, 0) != 0) {
    return "";
  }
  if (topic.size() <= std::strlen(kPrefix) + std::strlen(kSuffix)) {
    return "";
  }
  if (topic.compare(topic.size() - std::strlen(kSuffix), std::strlen(kSuffix),
                    kSuffix) != 0) {
    return "";
  }
  const size_t begin = std::strlen(kPrefix);
  const size_t end = topic.size() - std::strlen(kSuffix);
  if (end <= begin) {
    return "";
  }
  return topic.substr(begin, end - begin);
}

bool CloudMqtt::ApplyBarrierStateByIndex(
    const std::string& barrier_index, bool opened,
    century::planning::Barrier* barrier) const {
  if (nullptr == barrier) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto it = barrier_last_seen_.find(barrier_index);
  if (it == barrier_last_seen_.end() ||
      std::chrono::duration_cast<std::chrono::seconds>(now - it->second)
              .count() > kBarrierOfflineTimeoutSec) {
    opened = false;
  }
  if (kBarrierIndexJ1E == barrier_index) {
    barrier->set_east_south(opened);
    return true;
  }
  if (kBarrierIndexJ1W == barrier_index) {
    barrier->set_west_south(opened);
    return true;
  }
  if (kBarrierIndexJ4E == barrier_index) {
    barrier->set_east_north(opened);
    return true;
  }
  if (kBarrierIndexJ4W == barrier_index) {
    barrier->set_west_north(opened);
    return true;
  }
  return false;
}

bool CloudMqtt::IsBarrierStatusTimedOut(
    const std::string& barrier_index,
    const std::chrono::steady_clock::time_point& now) const {
  const auto it = barrier_last_seen_.find(barrier_index);
  if (it == barrier_last_seen_.end()) {
    return true;
  }
  const auto elapsed_sec =
      std::chrono::duration_cast<std::chrono::seconds>(now - it->second)
          .count();
  return elapsed_sec > kBarrierOfflineTimeoutSec;
}

bool CloudMqtt::GetEffectiveOpen(
    const std::string& barrier_index,
    const std::chrono::steady_clock::time_point& now) const {
  if (IsBarrierStatusTimedOut(barrier_index, now)) {
    return false;
  }
  const auto it = barrier_status_cache_.find(barrier_index);
  if (it == barrier_status_cache_.end()) {
    return false;
  }
  return it->second;
}

void CloudMqtt::RecomputeBarrierStateLocked(
    const std::chrono::steady_clock::time_point& now) {
  // barrier_command override: GetLatestObserved + header timestamp_sec
  auto barrier_cmd = barrier_command_reader_->GetLatestObserved();
  if (barrier_cmd && barrier_cmd->enabled() && barrier_cmd->has_header()) {
    const double elapsed =
        cyber::Time::Now().ToSecond() - barrier_cmd->header().timestamp_sec();
    if (elapsed <= 5.0) {
      const bool forced_open =
          barrier_cmd->command() ==
          century::dreamview::BarrierCommand::FORCE_OPEN;
      barrier_state_.set_east_south(forced_open);
      barrier_state_.set_west_south(forced_open);
      barrier_state_.set_east_north(forced_open);
      barrier_state_.set_west_north(forced_open);
      return;
    }
  }

  const bool j1_e_open = GetEffectiveOpen(kBarrierIndexJ1E, now);
  const bool j1_w_open = GetEffectiveOpen(kBarrierIndexJ1W, now);
  const bool j4_e_open = GetEffectiveOpen(kBarrierIndexJ4E, now);
  const bool j4_w_open = GetEffectiveOpen(kBarrierIndexJ4W, now);

  // planning::Barrier uses true to mean "open / passable".
  const bool all_barriers_open =
      j1_e_open && j1_w_open && j4_e_open && j4_w_open;

  barrier_state_.set_east_south(all_barriers_open);
  barrier_state_.set_west_south(all_barriers_open);
  barrier_state_.set_east_north(all_barriers_open);
  barrier_state_.set_west_north(all_barriers_open);
}

void CloudMqtt::PublishBarrierStatePeriodic() {
  std::lock_guard<std::mutex> lock(barrier_mutex_);
  RecomputeBarrierStateLocked(std::chrono::steady_clock::now());
  century::common::util::FillHeader("cloud_mqtt_barrier", &barrier_state_);
  if (barrier_writer_) {
    barrier_writer_->Write(barrier_state_);
  }
}

void CloudMqtt::OnPlanningRequest(
    const std::shared_ptr<century::planning::ADCTrajectory>& request) {
  if (!request || !request->has_notice_stacker()) {
    return;
  }

  const auto& pass_stacker_request = request->notice_stacker();
  std::string target_no;
  uint32_t operate = 0;
  std::vector<uint32_t> action{0x22};

  if (pass_stacker_request.has_stacker_id()) {
    target_no = pass_stacker_request.stacker_id();
  }
  if (pass_stacker_request.has_request_for_pass_stacker()) {
    operate = pass_stacker_request.request_for_pass_stacker();
  }
  if (pass_stacker_request.has_request_type()) {
    using RequestType = century::planning::PassStackerRequestType;
    static const std::unordered_map<RequestType, std::vector<uint32_t>>
        k_type_to_action = {
            {RequestType::PASS_DEFAULT, {0x22}},
            {RequestType::PASSED, {0x22}},
            {RequestType::PASS_READY, {0x22}},
            {RequestType::PASSING, {0x42}},
        };
    const auto type_it = k_type_to_action.find(pass_stacker_request.request_type());
    if (type_it != k_type_to_action.end()) {
      action = type_it->second;
    }
  }

  if (!operate || last_action_ == action) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_data_.clear();
  }
  last_action_ = action;
  const std::string vehicle_id = FLAGS_vehicle_id;
  const std::string vin_id = FLAGS_vehicle_unique_id;
  const uint32_t vehicle_type = 1;
  const std::string msg_id = pass_stacker_request.message_id();
  for (const auto& current_action : action) {
    SendPassStackerInform(vehicle_id, vin_id, vehicle_type, target_no,
                          current_action, operate, msg_id);
  }
}

void CloudMqtt::SendPassStackerInform(const std::string& vehicle_id,
                                      const std::string& vin_id,
                                      uint32_t vehicle_type,
                                      const std::string& target_no,
                                      uint32_t action, uint32_t operate,
                                      const std::string& msg_id) {
  const long timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const nlohmann::json inform_request_data = {{"vehicleId", vehicle_id},
                                              {"vin", vin_id},
                                              {"vehicleType", vehicle_type},
                                              {"targetNo", target_no},
                                              {"action", action},
                                              {"operate", operate},
                                              {"timestamp", timestamp},
                                              {"msgId", msg_id}};
  AINFO << "[CloudMqtt] inform_request_data: " << inform_request_data.dump();

  const std::string payload = inform_request_data.dump();
  const std::string notify_topic = BuildVehicleNotifyTopic(target_no);
  if (!Publish(notify_topic, payload, ResolvePubQos(notify_topic))) {
    AERROR << "[CloudMqtt] Failed to publish vehicle notify payload.";
  } else {
    AINFO << "[CloudMqtt] Sent vehicle notify msgId=" << msg_id;
  }
  if (!msg_id.empty()) {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_data_[msg_id] = payload;
  }
}

void CloudMqtt::RetryPendingNotifyMessages() {
  std::vector<std::pair<std::string, std::string>> pending_messages;
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    for (const auto& kv : request_data_) {
      pending_messages.emplace_back(kv.first, kv.second);
    }
  }

  for (const auto& kv : pending_messages) {
    auto payload_json = nlohmann::json::parse(kv.second, nullptr, false);
    if (payload_json.is_discarded() || !payload_json.is_object()) {
      AWARN << "[CloudMqtt] Retry vehicle notify skipped invalid payload msgId="
            << kv.first;
      continue;
    }
    const std::string target_no = payload_json.value("targetNo", "");
    if (target_no.empty()) {
      AWARN << "[CloudMqtt] Retry vehicle notify skipped missing targetNo msgId="
            << kv.first;
      continue;
    }
    const std::string notify_topic = BuildVehicleNotifyTopic(target_no);
    const auto notify_qos = ResolvePubQos(notify_topic);
    if (Publish(notify_topic, kv.second, notify_qos)) {
      AINFO << "[CloudMqtt] Retry vehicle notify msgId=" << kv.first;
    } else {
      AWARN << "[CloudMqtt] Retry vehicle notify failed msgId=" << kv.first;
    }
  }
}

bool CloudMqtt::Publish(const std::string& topic, const std::string& payload,
                        MqttQos qos, bool retain) {
  if (topic.empty()) {
    AERROR << "[CloudMqtt] publish topic is empty.";
    return false;
  }
  if (!mqtt_client_) {
    AERROR << "[CloudMqtt] mqtt client is null.";
    return false;
  }
  return mqtt_client_->Publish(topic, payload, qos, retain);
}

bool CloudMqtt::PublishJson(const std::string& topic,
                            const nlohmann::json& payload, MqttQos qos,
                            bool retain) {
  return Publish(topic, payload.dump(), qos, retain);
}

std::string CloudMqtt::BuildVehicleBroadcastTopic() {
  return "vehicle/broadcast/" + FLAGS_vehicle_id;
}

std::string CloudMqtt::BuildVehicleDataTopic() {
  return "vehicle/data/" + FLAGS_vehicle_id;
}

std::string CloudMqtt::BuildVehicleDataWildcardTopic() {
  return std::string(kVehicleDataTopicPrefix) + "+";
}

std::string CloudMqtt::BuildVehicleNotifyTopic(
    const std::string& target_no) {
  return "vehicle/notify/" + target_no;
}

std::string CloudMqtt::BuildVehicleReplyTopic() {
  return "vehicle/reply/" + FLAGS_vehicle_id;
}

void CloudMqtt::OnMqttMessage(const std::string& topic,
                              const nlohmann::json& payload) {
  TopicCallback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = registered_topics_.find(topic);
    if (it != registered_topics_.end()) {
      callback = it->second.callback;
    } else {
      for (const auto& kv : registered_topics_) {
        bool match = false;
        mosquitto_topic_matches_sub(kv.first.c_str(), topic.c_str(), &match);
        if (match) {
          callback = kv.second.callback;
          break;
        }
      }
    }
  }
  if (callback) {
    callback(topic, payload);
  } else {
    AWARN << "[CloudMqtt] recv unregistered topic=" << topic
          << ", no callback registered";
  }
}

void CloudMqtt::SleepUntilNextReportTick() const {
  const auto now = std::chrono::system_clock::now();
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count();
  const auto next_100ms_ms = ((now_ms / kReportTickMs) + 1) * kReportTickMs;
  const auto next_tick =
      std::chrono::system_clock::time_point(std::chrono::milliseconds(next_100ms_ms));
  std::this_thread::sleep_until(next_tick);
  last_report_tick_ms_.store(next_100ms_ms);
}

void CloudMqtt::MaybeRetryPendingNotify(int64_t tick_ms) {
  if (0 == (tick_ms % kNotifyRetryTickMs)) {
    RetryPendingNotifyMessages();
  }
}

void CloudMqtt::MaybePublishEmptyStackerInfo() {
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(now - last_stacker_info_time_)
          .count();
  if (elapsed <= kDefaultStackerTimeoutSec) {
    return;
  }

  century::planning::StackersInfo stackers_info;
  century::common::util::FillHeader("stackers_info", &stackers_info);
  if (stacker_info_writer_) {
    stacker_info_writer_->Write(stackers_info);
  }
}

void CloudMqtt::ObserveAllReaders() {
  if (chassis_reader_) {
    chassis_reader_->Observe();
  }
  if (routing_response_request_reader_) {
    routing_response_request_reader_->Observe();
  }
  if (localization_reader_) {
    localization_reader_->Observe();
  }
  if (top_bull_reader_) {
    top_bull_reader_->Observe();
  }
  if (barrier_command_reader_) {
    barrier_command_reader_->Observe();
  }
}

void CloudMqtt::BuildReportPayload(nlohmann::json* payload) {
  if (nullptr == payload) {
    return;
  }

  auto chassis_msg = chassis_reader_ ? chassis_reader_->GetLatestObserved() : nullptr;
  auto localization_msg =
      localization_reader_ ? localization_reader_->GetLatestObserved() : nullptr;

  std::string vehicle_id = FLAGS_vehicle_id;
  std::string vin_id = FLAGS_vehicle_unique_id;
  uint32_t vehicle_type = 1;
  float_t speed = 0.0f;
  uint32_t drive_mode = 0;
  double_t lng = 0.0;
  double_t lat = 0.0;
  float_t heading = 0.0f;
  uint32_t gear = 0;
  uint32_t task_type = 0;
  bool is_loading = false;
  long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();

  if (chassis_msg) {
    speed = chassis_msg->speed_mps();
    drive_mode = chassis_msg->driving_mode();
    gear = chassis_msg->gear_location();
  }

  century::routing::RoutingRequest routing_request;
  bool has_routing_request = GetCachedRoutingRequest(&routing_request);
  if (!has_routing_request && routing_response_request_reader_) {
    auto routing_msg = routing_response_request_reader_->GetLatestObserved();
    if (routing_msg && routing_msg->has_routing_request()) {
      routing_request = routing_msg->routing_request();
      CacheRoutingRequest(routing_request);
      has_routing_request = true;
    }
  }
  if (has_routing_request) {
    task_type = routing_request.task_type();
    is_loading = routing_request.is_loading();
  }

  if (localization_msg && localization_msg->has_pose()) {
    if (localization_msg->pose().has_position()) {
      double_t lng_ = offset_x_ + localization_msg->pose().position().x();
      double_t lat_ = offset_y_ + localization_msg->pose().position().y();
      UtmtoWgs84(lng_, lat_, zone_id_.c_str(), &lng, &lat, nullptr, nullptr);
      lng *= RAD_TO_DEG;
      lat *= RAD_TO_DEG;
    }
    heading = localization_msg->pose().heading();
  }

  // Build top_bull JSON from the latest TopBullInfo message.
  nlohmann::json top_bull_json;
  auto top_bull_msg =
      top_bull_reader_ ? top_bull_reader_->GetLatestObserved() : nullptr;
  if (top_bull_msg) {
    nlohmann::json path_array = nlohmann::json::array();
    for (const auto& pi : top_bull_msg->path_info()) {
      path_array.push_back(
          {{"x", pi.x()}, {"y", pi.y()}, {"theta", pi.theta()}});
    }
    top_bull_json["pathInfo"] = path_array;
    top_bull_json["turnType"] = static_cast<int>(top_bull_msg->turn_type());
    top_bull_json["randomNumber"] = top_bull_msg->random_number();
  }
  top_bull_json["is_load"] = is_loading;
  top_bull_json["isInTopBull"] =
      top_bull_msg ? top_bull_msg->is_in_top_bull() : false;
  top_bull_json["topBullType"] =
      top_bull_msg ? static_cast<int>(top_bull_msg->top_bull_type())
                   : static_cast<int>(century::planning::TB_NONE);

  *payload = {{"vehicleId", vehicle_id},
              {"vin", vin_id},
              {"vehicleType", vehicle_type},
              {"speed", speed},
              {"driveMode", drive_mode},
              {"lng", lng},
              {"lat", lat},
              {"heading", heading},
              {"gear", gear},
              {"taskType", task_type},
              {"timestamp", timestamp},
              {"topBull", top_bull_json}};
}

void CloudMqtt::PublishReportPayload(const nlohmann::json& payload) {
  ++report_log_count_;
  if (0 == (report_log_count_ % kReportInfoSampleEveryN)) {
    AINFO << "[CloudMqtt] report data sample " << payload.dump();
  }
  WriteVehicleData(payload);
}

void CloudMqtt::HandReportTaskLoop() {
  while (report_running_ && !cyber::IsShutdown()) {
    SleepUntilNextReportTick();
    if (cyber::IsShutdown() || !IsRunning()) {
      continue;
    }

    const int64_t tick_ms = last_report_tick_ms_.load();
    MaybeRetryPendingNotify(tick_ms);
    MaybePublishEmptyStackerInfo();
    ObserveAllReaders();
    PublishBarrierStatePeriodic();

    nlohmann::json report_data;
    BuildReportPayload(&report_data);
    PublishReportPayload(report_data);
  }
}

}  // namespace mcloud
}  // namespace century
