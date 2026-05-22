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

#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <memory>

#include "cyber/cyber.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_writer.h"
#include "cyber/message/raw_message.h"
#include "cyber/node/node_channel_impl.h"

#include "modules/dreamview/backend/handlers/websocket_handler.h"

using century::cyber::ReaderBase;
using century::cyber::message::RawMessage;
using century::cyber::record::RecordWriter;

namespace century {
namespace dreamview {

class BagRecord : public std::enable_shared_from_this<BagRecord> {
public:
  BagRecord(WebSocketHandler *websocket);

  void GetAllChannels(std::vector<std::string>* channels);

  void StartRecord(const std::vector<std::string>& channels);

  void StopRecord();

  void RegisterMessageHandlers();
private:
  void SetStorePath(const std::string& path);

  void ReaderCallback(const std::shared_ptr<RawMessage>& message,
                      const std::string& channel_name);

private:
  std::string file_path_;
  std::shared_ptr<RecordWriter> writer_ = nullptr;
  std::shared_ptr<century::cyber::Node> node_ = nullptr;

  std::unordered_map<std::string, std::shared_ptr<ReaderBase>>
      channel_reader_map_;

  WebSocketHandler *websocket_;
  std::atomic<bool> is_started_{false};
};

}  // namespace dreamview
}  // namespace cyber
