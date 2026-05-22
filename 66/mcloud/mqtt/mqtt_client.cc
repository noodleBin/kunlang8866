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

  // Apply clean-session flag (requires re-create if it differs from ctor value)
  int credential_rc = MOSQ_ERR_SUCCESS;
  if (!options_.username.empty()) {
    credential_rc = mosquitto_username_pw_set(
        mosq_, options_.username.c_str(),
        options_.password.empty() ? nullptr : options_.password.c_str());
  } else {
    // Explicitly clear credentials to avoid reusing stale auth values.
    credential_rc = mosquitto_username_pw_set(mosq_, nullptr, nullptr);
  }
  if (credential_rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Failed to set credentials: "
           << mosquitto_strerror(credential_rc);
    return false;
  }

  int rc = mosquitto_connect(mosq_, options_.host.c_str(), options_.port,
                             options_.keepalive_s);
  if (rc != MOSQ_ERR_SUCCESS) {
    AERROR << "[MqttClient] Connect to " << options_.host << ":"
           << options_.port << " failed: " << mosquitto_strerror(rc);
    return false;
  }

  // Start the internal network loop on a dedicated thread.
  running_ = true;
  loop_thread_ = std::make_unique<std::thread>([this]() {
    while (running_) {
      const int rc = mosquitto_loop(mosq_, /*timeout_ms=*/100,
                                    /*max_packets=*/1);
      if (!running_) {
        break;
      }
      if (MOSQ_ERR_SUCCESS == rc) {
        continue;
      }

      // Keep reconnecting while running so Disconnect() can stop loop quickly.
      if (rc != MOSQ_ERR_NO_CONN) {
        AERROR << "[MqttClient] Network loop error: " << mosquitto_strerror(rc);
      }
      const int reconnect_rc = mosquitto_reconnect(mosq_);
      if (reconnect_rc != MOSQ_ERR_SUCCESS && running_) {
        AWARN << "[MqttClient] Reconnect failed: "
              << mosquitto_strerror(reconnect_rc);
      }
    }
  });

  AINFO << "[MqttClient] Connecting to " << options_.host << ":"
        << options_.port;
  return true;
}

void MqttClient::Disconnect() {
  if (!mosq_) {
    return;
  }

  running_ = false;

  // Request broker disconnect to wake network loop as soon as possible.
  mosquitto_disconnect(mosq_);

  // Loop thread exits when running_ becomes false.
  if (loop_thread_ && loop_thread_->joinable()) {
    loop_thread_->join();
  }
  loop_thread_.reset();
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
    // Drop oldest to keep process memory bounded.
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
    AINFO << "[MqttClient] Connected to " << options_.host << ":"
          << options_.port;
    ResubscribeAll();
    FlushPendingPublishes();
  } else {
    AERROR << "[MqttClient] Connection refused: " << ConnectRcToString(rc);
  }
}

void MqttClient::HandleDisconnect(int rc) {
  connected_ = false;
  if (rc != 0 && running_) {
    AWARN << "[MqttClient] Unexpected disconnection (rc=" << rc
          << "). mosquitto_loop_forever will reconnect.";
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

  // Dispatch to all callbacks whose topic matches (exact or wildcard already
  // resolved by the broker; we just iterate registered topics).
  SubscriptionEntry entry;
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    auto it = subscriptions_.find(topic);
    if (it != subscriptions_.end()) {
      entry = it->second;
      found = true;
    } else {
      // Wildcard subscription: find first pattern that applies.
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
