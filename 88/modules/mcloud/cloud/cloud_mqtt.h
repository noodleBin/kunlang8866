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

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "cyber/cyber.h"
#include "modules/planning/proto/top_bull_info.pb.h"
#include "modules/planning/proto/v2x_info.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/planning/proto/lane_borrow_response.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/dreamview/proto/barrier_command.pb.h"
#include "modules/planning/proto/barrier.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include <thread>
#include <atomic>

#include "nlohmann/json.hpp"
#include "modules/mcloud/mqtt/mqtt_client.h"

namespace century {
namespace mcloud {

class CloudMqtt final {
 public:
  using TopicCallback = std::function<void(const std::string& topic,
                                           const nlohmann::json& payload)>;

  CloudMqtt();
  ~CloudMqtt();

  bool Start();
  void Stop();
  bool IsRunning() const;

  // Writer entrypoints (business payload formatting is intentionally left
  // TODO).
  bool WriteVehicleData(const nlohmann::json& payload);

  std::vector<std::string> SubscribedTopics() const;

 private:
  bool InitReaderTopicsLocked();
  void InitWriterTopicsLocked();
  bool RegisterInternalTopicsLocked();
  void InitQosConfigLocked();
  static std::unordered_map<std::string, MqttQos> ParseQosOverrides(
      const std::string& raw_json);
  MqttQos ResolveSubQos(const std::string& topic) const;
  MqttQos ResolvePubQos(const std::string& topic) const;
  void HandleVehicleBroadcast(const std::string& topic,
                              const nlohmann::json& payload);
  void HandleVehicleData(const std::string& topic,
                         const nlohmann::json& payload);
  void HandleVehicleReply(const std::string& topic,
                          const nlohmann::json& payload);
  void HandleBoomBarrierStatus(const std::string& topic,
                               const nlohmann::json& payload);
  std::string ResolveBarrierIndex(const std::string& topic,
                                  const nlohmann::json& payload) const;
  bool ApplyBarrierStateByIndex(const std::string& barrier_index, bool opened,
                                century::planning::Barrier* barrier) const;
  bool IsBarrierStatusTimedOut(
      const std::string& barrier_index,
      const std::chrono::steady_clock::time_point& now) const;
  bool GetEffectiveOpen(
      const std::string& barrier_index,
      const std::chrono::steady_clock::time_point& now) const;
  void RecomputeBarrierStateLocked(
      const std::chrono::steady_clock::time_point& now);
  void PublishBarrierStatePeriodic();
  bool Publish(const std::string& topic, const std::string& payload,
               MqttQos qos = MqttQos::kAtLeastOnce, bool retain = false);
  bool PublishJson(const std::string& topic, const nlohmann::json& payload,
                   MqttQos qos = MqttQos::kAtLeastOnce, bool retain = false);
  void OnMqttMessage(const std::string& topic, const nlohmann::json& payload);
  static std::string BuildVehicleBroadcastTopic();
  static std::string BuildVehicleDataTopic();
  static std::string BuildVehicleDataWildcardTopic();
  static std::string BuildVehicleNotifyTopic(const std::string& target_no);
  static std::string BuildVehicleReplyTopic();
  bool BuildMqttOptionsLocked(MqttOptions* options) const;
  bool ConnectAndSubscribeLocked(const MqttOptions& options);
  bool StartReportThreadLocked();
  void StopReportThreadUnlocked();
  struct VehicleBroadcastItem {
    uint32_t vehicle_type = 0;
    std::string vehicle_id;
    uint32_t task_type = 0;
    float_t speed = 0.0f;
    uint32_t drive_mode = 0;
    double lng = 0.0;
    double lat = 0.0;
    float_t heading = 0.0f;
    uint32_t gear = 0;
    long timestamp_ms = 0;
    nlohmann::json top_bull;  // optional topBull from broadcast JSON
  };
  bool ParseVehicleArrayItem(const nlohmann::json& in,
                             VehicleBroadcastItem* out);
  void CacheRoutingRequest(const century::routing::RoutingRequest& request);
  bool GetCachedRoutingRequest(
      century::routing::RoutingRequest* request) const;
  bool GetCachedVehicleData(const std::string& vehicle_id,
                            VehicleBroadcastItem* out) const;
  void FillVehicleInfoProto(const VehicleBroadcastItem& in,
                            century::planning::VehicleInfo* out);
  void FillStackerInfoProto(const VehicleBroadcastItem& in,
                            century::planning::StackerInfo* out);
  void PublishBroadcastProtos(century::planning::V2xInfo* v2x_info,
                              century::planning::StackersInfo* stackers_info);
  void HandleBroadcastNotifyObject(const nlohmann::json& obj);
  void ConvertWgs84ToLocal(double lng, double lat, double* x, double* y) const;
  void SleepUntilNextReportTick() const;
  void MaybeRetryPendingNotify(int64_t tick_ms);
  void MaybePublishEmptyStackerInfo();
  void ObserveAllReaders();
  void BuildReportPayload(nlohmann::json* payload);
  void PublishReportPayload(const nlohmann::json& payload);

  void OnPlanningRequest(
      const std::shared_ptr<century::planning::ADCTrajectory>& request);
  void SendPassStackerInform(const std::string& vehicle_id,
                             const std::string& vin_id, uint32_t vehicle_type,
                             const std::string& target_no, uint32_t action,
                             uint32_t operate, const std::string& msg_id);
  void RetryPendingNotifyMessages();
  void HandReportTaskLoop();

 private:
  struct TopicEntry {
    TopicCallback callback;
    MqttQos qos = MqttQos::kAtLeastOnce;
  };

  mutable std::mutex mutex_;
  std::unique_ptr<MqttClient> mqtt_client_;
  std::unordered_map<std::string, TopicEntry> registered_topics_;
  std::unordered_map<std::string, MqttQos> sub_qos_overrides_;
  std::unordered_map<std::string, MqttQos> pub_qos_overrides_;
  MqttQos sub_qos_default_ = MqttQos::kAtLeastOnce;
  MqttQos pub_qos_default_ = MqttQos::kAtLeastOnce;
  std::string writer_vehicle_data_topic_;
  bool started_ = false;

  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<century::planning::V2xInfo>> v2x_info_writer_;
  std::shared_ptr<cyber::Writer<century::planning::StackersInfo>>
      stacker_info_writer_;
  std::shared_ptr<cyber::Writer<century::planning::BorrowResponse>>
      borrow_writer_;
  std::shared_ptr<cyber::Writer<century::planning::PassStackerResponse>>
      pass_stacker_writer_;
  std::shared_ptr<cyber::Writer<century::planning::Barrier>> barrier_writer_;

  std::shared_ptr<cyber::Reader<century::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<century::routing::RoutingResponse>>
      routing_response_request_reader_;
  std::shared_ptr<cyber::Reader<century::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<century::planning::ADCTrajectory>>
      planning_reader_;
  std::shared_ptr<cyber::Reader<century::planning::TopBullInfo>>
      top_bull_reader_;
  std::shared_ptr<cyber::Reader<century::dreamview::BarrierCommand>>
      barrier_command_reader_;

  std::unique_ptr<std::thread> report_thread_;
  std::atomic<bool> report_running_{false};
  mutable std::atomic<int64_t> last_report_tick_ms_{0};
  uint32_t report_log_count_ = 0;

  double offset_x_ = 0.0;
  double offset_y_ = 0.0;
  std::string zone_id_ = "51S";
  std::chrono::steady_clock::time_point last_stacker_info_time_;
  std::vector<uint32_t> last_action_;
  std::mutex request_mutex_;
  std::unordered_map<std::string, std::string> request_data_;
  mutable std::mutex routing_request_mutex_;
  century::routing::RoutingRequest last_routing_request_;
  bool has_cached_routing_request_ = false;
  mutable std::mutex vehicle_data_mutex_;
  std::unordered_map<std::string, VehicleBroadcastItem> vehicle_data_cache_;
  std::mutex barrier_mutex_;
  century::planning::Barrier barrier_state_;
  std::unordered_map<std::string, bool> barrier_status_cache_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point>
      barrier_last_seen_;
};

}  // namespace mcloud
}  // namespace century
