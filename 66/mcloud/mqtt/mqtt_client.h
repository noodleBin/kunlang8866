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

#ifndef CENTURY_MODULES_MCLOUD_MQTT_MQTT_CLIENT_H_
#define CENTURY_MODULES_MCLOUD_MQTT_MQTT_CLIENT_H_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <deque>
#include <unordered_map>
#include <utility>

#include <mosquitto.h>
#include "nlohmann/json.hpp"

namespace century {
namespace mcloud {

// Callback signature: (topic, payload)
using MqttMessageCallback =
    std::function<void(const std::string& topic, const std::string& payload)>;
using MqttJsonMessageCallback =
    std::function<void(const std::string& topic, const nlohmann::json& payload)>;

// MQTT QoS levels
enum class MqttQos : int {
  kAtMostOnce = 0,   // fire and forget
  kAtLeastOnce = 1,  // acknowledged delivery
  kExactlyOnce = 2,  // assured delivery
};

struct MqttOptions {
  // Broker host. Supports plain host/ip or URI like "tcp://127.0.0.1".
  std::string host = "localhost";
  int port = 1883;
  std::string username;
  std::string password;
  int keepalive_s = 60;
  bool clean_session = true;
};


class MqttClient {
 public:
  explicit MqttClient(const std::string& client_id);
  ~MqttClient();

  // Non-copyable, non-movable
  MqttClient(const MqttClient&) = delete;
  MqttClient& operator=(const MqttClient&) = delete;

  // Connect to the broker. Returns true on success.
  bool Connect(const MqttOptions& options);

  // Disconnect from the broker and stop the network loop.
  void Disconnect();

  bool IsConnected() const { return connected_; }

  // Subscribe to a topic pattern (supports wildcards '+' and '#').
  // The callback is invoked on the internal network-loop thread —
  // keep it short or dispatch to another thread.
  // Returns false if the client is not connected or subscribe fails.
  bool Subscribe(const std::string& topic, MqttMessageCallback callback,
                 MqttQos qos = MqttQos::kAtLeastOnce);
  bool SubscribeJson(const std::string& topic, MqttJsonMessageCallback callback,
                     MqttQos qos = MqttQos::kAtLeastOnce);

  // Remove a subscription. Returns false if topic was not subscribed.
  bool Unsubscribe(const std::string& topic);

  // Publish a payload to a topic.
  bool Publish(const std::string& topic, const std::string& payload,
               MqttQos qos = MqttQos::kAtLeastOnce, bool retain = false);

 private:
  // libmosquitto static callbacks (bridge to instance methods)
  static void OnConnectCb(struct mosquitto* mosq, void* userdata, int rc);
  static void OnDisconnectCb(struct mosquitto* mosq, void* userdata, int rc);
  static void OnMessageCb(struct mosquitto* mosq, void* userdata,
                          const struct mosquitto_message* msg);

  void HandleConnect(int rc);
  void HandleDisconnect(int rc);
  void HandleMessage(const struct mosquitto_message* msg);

  // Re-subscribe all registered topics after reconnect.
  void ResubscribeAll();
  void FlushPendingPublishes();
  bool EnqueuePendingPublish(const std::string& topic,
                             const std::string& payload, MqttQos qos,
                             bool retain);

  struct SubscriptionEntry {
    MqttMessageCallback raw_callback;
    MqttJsonMessageCallback json_callback;
    MqttQos qos;
  };
  struct PendingPublish {
    std::string topic;
    std::string payload;
    MqttQos qos = MqttQos::kAtLeastOnce;
    bool retain = false;
  };

 private:
  struct mosquitto* mosq_ = nullptr;
  std::string client_id_;
  MqttOptions options_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};

  std::mutex subs_mutex_;
  std::unordered_map<std::string, SubscriptionEntry> subscriptions_;

  std::mutex publish_queue_mutex_;
  std::deque<PendingPublish> pending_publishes_;
  static constexpr size_t kMaxPendingPublishes = 2000;

  std::unique_ptr<std::thread> loop_thread_;
};

}  // namespace mcloud
}  // namespace century

#endif  // CENTURY_MODULES_MCLOUD_MQTT_MQTT_CLIENT_H_
