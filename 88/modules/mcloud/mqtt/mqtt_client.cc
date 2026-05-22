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

#include "modules/mcloud/mqtt/mqtt_client.h"

#include <cstring>

#include <mosquitto.h>

#include "cyber/cyber.h"

namespace century {
namespace mcloud {

namespace {

// Convert our QoS enum to the raw int mosquitto expects.
inline int ToInt(MqttQos qos) { return static_cast<int>(qos); }

// Map mosquitto reason codes to a human-readable string.
const char* ConnectRcToString(int rc) {
  switch (rc) {
    case 0:  return "success";
    case 1:  return "unacceptable protocol version";
    case 2:  return "identifier rejected";
    case 3:  return "broker unavailable";
    case 4:  return "bad username or password";
    case 5:  return "not authorized";
    default: return "unknown error";
  }
}

std::string NormalizeBrokerHost(const std::string& host) {
  static const char* kTcp = "tcp://";
  static const char* kMqtt = "mqtt://";
  if (0 == host.rfind(kTcp, 0)) {
    return host.substr(std::strlen(kTcp));
  }
  if (0 == host.rfind(kMqtt, 0)) {
    return host.substr(std::strlen(kMqtt));
  }
  return host;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MqttClient::MqttClient(const std::string& client_id) : client_id_(client_id) {
  mosquitto_lib_init();
  mosq_ = mosquitto_new(client_id_.c_str(), /*clean_session=*/true,
                        /*userdata=*/this);
  if (!mosq_) {
    AERROR << "[MqttClient] Failed to create mosquitto instance.";
    return;
  }
  const int threaded_rc = mosquitto_threaded_set(mosq_, true);
  if (threaded_rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Failed to enable threaded mode: "
           << mosquitto_strerror(threaded_rc);
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    return;
  }

  mosquitto_connect_callback_set(mosq_, &MqttClient::OnConnectCb);
  mosquitto_disconnect_callback_set(mosq_, &MqttClient::OnDisconnectCb);
  mosquitto_message_callback_set(mosq_, &MqttClient::OnMessageCb);
}

MqttClient::~MqttClient() {
  Disconnect();
  if (mosq_) {
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  mosquitto_lib_cleanup();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool MqttClient::Connect(const MqttOptions& options) {
  if (!mosq_) {
    AERROR << "[MqttClient] Mosquitto instance not initialized.";
    return false;
  }

  options_ = options;
  options_.host = NormalizeBrokerHost(options_.host);
  if (options_.host.empty()) {
    AERROR << "[MqttClient] Broker host is empty.";
    return false;
  }
  if (options_.port <= 0) {
    AERROR << "[MqttClient] Invalid broker port: " << options_.port;
    return false;
  }

  int credential_rc = MOSQ_ERR_SUCCESS;
  if (!options_.username.empty()) {
    credential_rc = mosquitto_username_pw_set(
        mosq_, options_.username.c_str(),
        options_.password.empty() ? nullptr : options_.password.c_str());
  } else {
    credential_rc = mosquitto_username_pw_set(mosq_, nullptr, nullptr);
  }
  if (credential_rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Failed to set credentials: "
           << mosquitto_strerror(credential_rc);
    return false;
  }

  // Attempt the initial connection. If it fails (e.g. broker not yet online),
  // we still start the background threads — the watchdog will keep retrying.
  const int rc = mosquitto_connect(mosq_, options_.host.c_str(),
                                   options_.port, options_.keepalive_s);
  if (rc != MOSQ_ERR_SUCCESS) {
    AWARN << "[MqttClient] Initial connect to " << options_.host << ":"
          << options_.port << " failed (" << mosquitto_strerror(rc)
          << "), watchdog will retry.";
  } else {
    AINFO << "[MqttClient] Connecting to " << options_.host << ":"
          << options_.port;
  }

  // Start the network I/O loop thread. Only drives libmosquitto I/O;
  // reconnection is handled exclusively by the watchdog thread.
  running_ = true;
  loop_thread_ = std::make_unique<std::thread>([this]() {
    while (running_) {
      const int loop_rc = mosquitto_loop(mosq_, 100, 1);
      if (!running_) break;
      if (MOSQ_ERR_SUCCESS == loop_rc) continue;
      if (MOSQ_ERR_NO_CONN == loop_rc) {
        // No active connection; yield while the watchdog reconnects.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      AWARN << "[MqttClient] Network loop error: "
            << mosquitto_strerror(loop_rc);
    }
  });

  reconnect_thread_ = std::make_unique<std::thread>([this]() {
    WatchdogLoop();
  });

  return true;
}

void MqttClient::Disconnect() {
  if (!mosq_) {
    return;
  }

  running_ = false;
  // Wake the watchdog thread so it exits promptly.
  reconnect_cv_.notify_all();

  mosquitto_disconnect(mosq_);

  if (loop_thread_ && loop_thread_->joinable()) {
    loop_thread_->join();
  }
  loop_thread_.reset();

  if (reconnect_thread_ && reconnect_thread_->joinable()) {
    reconnect_thread_->join();
  }
  reconnect_thread_.reset();

  connected_ = false;
  AINFO << "[MqttClient] Disconnected.";
}

bool MqttClient::Subscribe(const std::string& topic,
                           MqttMessageCallback callback, MqttQos qos) {
  if (!mosq_) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    subscriptions_[topic] = {std::move(callback), nullptr, qos};
  }

  if (!connected_) {
    // Subscription will be applied in HandleConnect → ResubscribeAll.
    AINFO << "[MqttClient] Queued subscription for topic '" << topic
          << "' (not yet connected).";
    return true;
  }

  int rc = mosquitto_subscribe(mosq_, /*mid=*/nullptr, topic.c_str(),
                               ToInt(qos));
  if (rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Subscribe to '" << topic
           << "' failed: " << mosquitto_strerror(rc);
    return false;
  }

  AINFO << "[MqttClient] Subscribed to '" << topic << "' (qos=" << ToInt(qos)
        << ").";
  return true;
}

bool MqttClient::SubscribeJson(const std::string& topic,
                               MqttJsonMessageCallback callback, MqttQos qos) {
  if (!mosq_) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    subscriptions_[topic] = {nullptr, std::move(callback), qos};
  }

  if (!connected_) {
    AINFO << "[MqttClient] Queued JSON subscription for topic '" << topic
          << "' (not yet connected).";
    return true;
  }

  int rc = mosquitto_subscribe(mosq_, /*mid=*/nullptr, topic.c_str(),
                               ToInt(qos));
  if (rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] SubscribeJson to '" << topic
           << "' failed: " << mosquitto_strerror(rc);
    return false;
  }

  AINFO << "[MqttClient] JSON subscribed to '" << topic
        << "' (qos=" << ToInt(qos) << ").";
  return true;
}

bool MqttClient::Unsubscribe(const std::string& topic) {
  if (!mosq_) return false;

  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    if (subscriptions_.erase(topic) == 0) {
      AWARN << "[MqttClient] Unsubscribe: topic '" << topic
            << "' was not subscribed.";
      return false;
    }
  }

  if (connected_) {
    int rc = mosquitto_unsubscribe(mosq_, /*mid=*/nullptr, topic.c_str());
    if (rc != MOSQ_ERR_SUCCESS) {
      AERROR << "[MqttClient] Unsubscribe from '" << topic
             << "' failed: " << mosquitto_strerror(rc);
      return false;
    }
  }

  AINFO << "[MqttClient] Unsubscribed from '" << topic << "'.";
  return true;
}

bool MqttClient::Publish(const std::string& topic, const std::string& payload,
                         MqttQos qos, bool retain) {
  if (!mosq_) {
    AERROR << "[MqttClient] Publish failed: mqtt client is null.";
    return false;
  }
  if (topic.empty()) {
    AERROR << "[MqttClient] Publish failed: empty topic.";
    return false;
  }

  if (!connected_) {
    const bool queued = EnqueuePendingPublish(topic, payload, qos, retain);
    if (!queued) {
      AERROR << "[MqttClient] Publish failed: disconnected and queue full.";
      return false;
    }
    AWARN << "[MqttClient] Disconnected, queued publish topic='" << topic
          << "'.";
    return true;
  }

  int rc = mosquitto_publish(
      mosq_, /*mid=*/nullptr, topic.c_str(),
      static_cast<int>(payload.size()),
      reinterpret_cast<const void*>(payload.data()), ToInt(qos), retain);

  if (rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Publish to '" << topic
           << "' failed, queued for retry: " << mosquitto_strerror(rc);
    return EnqueuePendingPublish(topic, payload, qos, retain);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MqttClient::WatchdogLoop() {
  // Retry delays: start at 500 ms, double on each failure, cap at 30 s.
  constexpr auto kInitDelay = std::chrono::milliseconds(500);
  constexpr auto kMaxDelay  = std::chrono::milliseconds(5000);

  while (running_) {
    {
      std::unique_lock<std::mutex> lk(reconnect_mutex_);
      reconnect_cv_.wait(lk, [this] {
        return !running_ || !connected_;
      });
    }
    if (!running_) break;

    auto delay = kInitDelay;
    while (running_ && !connected_) {
      AINFO << "[MqttClient] Watchdog: reconnecting to " << options_.host
            << ":" << options_.port << " (next retry in "
            << delay.count() << " ms) ...";
      mosquitto_reconnect(mosq_);

      {
        std::unique_lock<std::mutex> lk(reconnect_mutex_);
        reconnect_cv_.wait_for(lk, delay, [this] {
          return !running_ || connected_;
        });
      }
      if (connected_ || !running_) break;

      delay = std::min(delay * 2, kMaxDelay);
    }

    if (connected_) {
      AINFO << "[MqttClient] Watchdog: connection established.";
    }
  }
  AINFO << "[MqttClient] Watchdog thread exiting.";
}

void MqttClient::ResubscribeAll() {
  std::lock_guard<std::mutex> lock(subs_mutex_);
  for (const auto& kv : subscriptions_) {
    int rc = mosquitto_subscribe(mosq_, nullptr, kv.first.c_str(),
                                 ToInt(kv.second.qos));
    if (rc != MOSQ_ERR_SUCCESS) {
      AERROR << "[MqttClient] Re-subscribe to '" << kv.first
             << "' failed: " << mosquitto_strerror(rc);
    } else {
      AINFO << "[MqttClient] Re-subscribed to '" << kv.first << "'.";
    }
  }
}

bool MqttClient::EnqueuePendingPublish(const std::string& topic,
                                       const std::string& payload, MqttQos qos,
                                       bool retain) {
  std::lock_guard<std::mutex> lock(publish_queue_mutex_);
  if (pending_publishes_.size() >= kMaxPendingPublishes) {
    pending_publishes_.pop_front();
    AWARN << "[MqttClient] Pending publish queue full, dropped oldest message.";
  }
  pending_publishes_.push_back(PendingPublish{topic, payload, qos, retain});
  return true;
}

void MqttClient::FlushPendingPublishes() {
  if (!connected_) {
    return;
  }

  size_t flushed_count = 0;
  while (connected_) {
    PendingPublish msg;
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      if (pending_publishes_.empty()) {
        break;
      }
      msg = std::move(pending_publishes_.front());
      pending_publishes_.pop_front();
    }

    const int rc = mosquitto_publish(
        mosq_, /*mid=*/nullptr, msg.topic.c_str(),
        static_cast<int>(msg.payload.size()),
        reinterpret_cast<const void*>(msg.payload.data()), ToInt(msg.qos),
        msg.retain);
    if (rc != MOSQ_ERR_SUCCESS) {
      AERROR << "[MqttClient] Flush pending publish failed on topic '"
             << msg.topic << "': " << mosquitto_strerror(rc);
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      pending_publishes_.push_front(std::move(msg));
      break;
    }
    ++flushed_count;
  }

  if (flushed_count > 0) {
    AINFO << "[MqttClient] Flushed pending publishes: " << flushed_count;
  }
}

void MqttClient::HandleConnect(int rc) {
  if (0 == rc) {
    connected_ = true;
    reconnect_cv_.notify_all();
    AINFO << "[MqttClient] Connected to " << options_.host << ":"
          << options_.port;
    ResubscribeAll();
    FlushPendingPublishes();
  } else {
    AERROR << "[MqttClient] Connection refused: " << ConnectRcToString(rc);
    connected_ = false;
    reconnect_cv_.notify_all();
  }
}

void MqttClient::HandleDisconnect(int rc) {
  connected_ = false;
  reconnect_cv_.notify_all();
  if (rc != 0 && running_) {
    AWARN << "[MqttClient] Unexpected disconnection (rc=" << rc
          << "), watchdog will reconnect.";
  } else {
    AINFO << "[MqttClient] Disconnected cleanly.";
  }
}

void MqttClient::HandleMessage(const struct mosquitto_message* msg) {
  if (!msg || !msg->topic) return;

  const std::string topic(msg->topic);
  const std::string payload(static_cast<const char*>(msg->payload),
                            static_cast<size_t>(msg->payloadlen));
  AINFO << "[MqttClient] Recv topic='" << topic << "' bytes=" << payload.size()
        << " payload=" << payload;

  SubscriptionEntry entry;
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    auto it = subscriptions_.find(topic);
    if (it != subscriptions_.end()) {
      entry = it->second;
      found = true;
    } else {
      for (const auto& kv : subscriptions_) {
        bool match = false;
        mosquitto_topic_matches_sub(kv.first.c_str(), topic.c_str(), &match);
        if (match) {
          entry = kv.second;
          found = true;
          break;
        }
      }
    }
  }

  if (!found) {
    AWARN << "[MqttClient] No callback for topic '" << topic << "'.";
    return;
  }

  if (entry.raw_callback) {
    entry.raw_callback(topic, payload);
  }

  if (entry.json_callback) {
    if (payload.empty()) {
      AWARN << "[MqttClient] Empty payload for JSON callback, topic='" << topic
            << "'";
      return;
    }
    try {
      const auto json_payload = nlohmann::json::parse(payload);
      entry.json_callback(topic, json_payload);
    } catch (const std::exception& e) {
      AERROR << "[MqttClient] JSON parse failed for topic '" << topic
             << "': " << e.what();
    }
  }
}

// ---------------------------------------------------------------------------
// Static callbacks (bridge to instance)
// ---------------------------------------------------------------------------

void MqttClient::OnConnectCb(struct mosquitto* /*mosq*/, void* userdata,
                             int rc) {
  static_cast<MqttClient*>(userdata)->HandleConnect(rc);
}

void MqttClient::OnDisconnectCb(struct mosquitto* /*mosq*/, void* userdata,
                                int rc) {
  static_cast<MqttClient*>(userdata)->HandleDisconnect(rc);
}

void MqttClient::OnMessageCb(struct mosquitto* /*mosq*/, void* userdata,
                             const struct mosquitto_message* msg) {
  static_cast<MqttClient*>(userdata)->HandleMessage(msg);
}

}  // namespace mcloud
}  // namespace century
