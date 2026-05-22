      
/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/bag_record/bag_record.h"
#include "cyber/service_discovery/topology_manager.h"
#include "nlohmann/json.hpp"

using Json = nlohmann::json;

namespace century {
namespace dreamview {

BagRecord::BagRecord(WebSocketHandler *websocket) : websocket_(websocket), is_started_(false) {
  node_ = ::century::cyber::CreateNode("bag_record");
  auto opt_header = century::cyber::record::HeaderBuilder::GetHeader();
  opt_header.set_segment_interval(60 * 1000000000ULL);
  opt_header.set_segment_raw_size(1024 * 1024 * 1024ULL);
  writer_ = std::make_shared<RecordWriter>(opt_header);
  // writer_->Open("/century/data/bag/dreamview_");
  // writer_->SetDirectory("/century/data/bag/dreamview_");
}

void BagRecord::GetAllChannels(std::vector<std::string>* channels) {
  if (!channels) {
    AERROR << "Invalid channels pointer.";
    return;
  }

  auto channel_manager =
      century::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  std::vector<century::cyber::proto::RoleAttributes> role_attr_vec;
  channel_manager->GetWriters(&role_attr_vec);
  std::unordered_set<std::string> sorted_channels;
  channels->reserve(role_attr_vec.size());
  for (const auto& role_attr : role_attr_vec) {
    sorted_channels.insert(role_attr.channel_name());
    // set_.emplace_back(role_attr.channel_name());
  }

  channels->assign(sorted_channels.begin(), sorted_channels.end());
}

void BagRecord::StartRecord(const std::vector<std::string>& channels) {
  if (is_started_) {
    AWARN << "Record has already started.";
    return;
  }

  writer_->Open("/century/data/bag/dreamview_");
  writer_->SetDirectory("/century/data/bag/dreamview_");

  is_started_ = true;
  std::weak_ptr<BagRecord> weak_this = shared_from_this();

  century::cyber::ReaderConfig config;
  config.pending_queue_size =
      gflags::Int32FromEnv("CYBER_PENDING_QUEUE_SIZE", 50);

  for (const auto& channel : channels) {
    if (channel_reader_map_.count(channel)) {
      AWARN << "Channel already being recorded: " << channel;
      continue;
    }

    config.channel_name = channel;

    auto callback = [weak_this, channel](const std::shared_ptr<RawMessage>& raw_message) {
      auto shared_this = weak_this.lock();
      if (!shared_this) {
        return;
      }
      shared_this->ReaderCallback(raw_message, channel);
    };

    auto reader = node_->CreateReader<RawMessage>(config, callback);

    channel_reader_map_[channel] = std::move(reader);

  }
}

void BagRecord::ReaderCallback(const std::shared_ptr<RawMessage>& message,
                               const std::string& channel_name) {
  if (!is_started_) {
    AERROR << "Reader callback invoked while recording is stopped.";
    return;
  }

  auto message_time = century::cyber::Time::Now().ToNanosecond();
  if (!writer_ || !writer_->WriteMessage(channel_name, message, message_time)) {
    AERROR << "Failed to write data for channel: " << channel_name;
    return;
  }
}

void BagRecord::StopRecord() {
  if (!is_started_) {
    AWARN << "StopRecord called, but recording is not active.";
    return;
  }

  for (auto& channel : channel_reader_map_) {
    node_->DeleteReader(channel.first);
  }

  channel_reader_map_.clear();
  is_started_ = false;
}

void BagRecord::RegisterMessageHandlers() {
  std::weak_ptr<BagRecord> weak_this = shared_from_this();

  websocket_->RegisterMessageHandler(
      "GetAllChannels",
      [weak_this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        std::vector<std::string> channels;
        auto shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        shared_this->GetAllChannels(&channels);
        response["data"]["channel"] = Json::array();
        for (auto channel : channels) {
          response["data"]["channel"].push_back(channel);
        }
        response["type"] = "AllChannelsInfo";
        shared_this->websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StartRecord",
      [weak_this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        std::vector<std::string> channels(json["channels"].begin(), json["channels"].end());
        auto shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        shared_this->StartRecord(channels);
        response["type"] = "BagRecordStatus";
        response["status"] = true;
        shared_this->websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "StopRecord",
      [weak_this](const Json& json, WebSocketHandler::Connection* conn) {
        Json response;
        auto shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        shared_this->StopRecord();
        response["type"] = "BagRecordStatus";
        response["status"] = false;
        shared_this->websocket_->SendData(conn, response.dump());
      });
}


}  // namespace dreamview
}  // namespace cyber

    