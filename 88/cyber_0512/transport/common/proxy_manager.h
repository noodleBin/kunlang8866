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

#ifndef CYBER_TRANSPORT_COMMON_PROXY_MANAGER_H_
#define CYBER_TRANSPORT_COMMON_PROXY_MANAGER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace century {
namespace cyber {
namespace transport {

class ProxyManager {
 public:
  using PromoteCallback = std::function<void()>;

  static ProxyManager* Instance();

  bool TryAcquire(uint64_t channel_id, const std::string& proto);
  void Release(uint64_t channel_id, const std::string& proto);
  bool IsOwner(uint64_t channel_id, const std::string& proto);

  bool StartWatch(uint64_t channel_id, const std::string& proto,
                  const PromoteCallback& on_promote);
  void StopWatch(uint64_t channel_id, const std::string& proto);

 private:
  struct Entry {
    int fd = -1;
    bool is_owner = false;
    bool running = false;
    PromoteCallback on_promote;
    std::thread thread;
  };

  ProxyManager() = default;
  ~ProxyManager();
  ProxyManager(const ProxyManager&) = delete;
  ProxyManager& operator=(const ProxyManager&) = delete;

  using Key = std::string;

  bool TryAcquireLocked(Entry* entry, const std::string& path);
  void ReleaseLocked(Entry* entry);
  void WatchLoop(const Key& key, uint64_t channel_id,
                 const std::string& proto);
  Key MakeKey(uint64_t channel_id, const std::string& proto) const;
  std::string MakePath(uint64_t channel_id, const std::string& proto) const;

  std::mutex mutex_;
  std::unordered_map<Key, std::unique_ptr<Entry>> entries_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_COMMON_PROXY_MANAGER_H_
