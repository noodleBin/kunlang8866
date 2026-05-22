
/**
 * @file century_ad_ipc.h
 * @brief century_ad_ipc class
 * @Author: songlf
 * @Date Created: 10/10/2022
 */

#ifndef CENTURY_AD_IPC_H_
#define CENTURY_AD_IPC_H_

#include <string>
#include <vector>

#include "modules/canbus/century_ad_ipc_msg.h"
#include "modules/canbus/century_ad_ipc_util.h"
#include <thread>
/**
 * @namespace century::ad::ipc
 * @brief
 */

namespace century {
namespace ad {
namespace ipc {

using ipc_url = century::ad::util::st_ad_ipc_url;
using ipc_sub = century::ad::util::st_ad_ipc_sub;
using ipc_payload = century::ad::msg::st_ad_ipc_payload;
using ipc_type = century::ad::util::en_century_ad_ipc_type;
using ipc_error = century::ad::util::en_century_ad_ipc_error;

class IPC {
 public:
  IPC() = default;
  IPC(ipc_url owner, std::vector<ipc_sub> subs, std::string name);
  virtual ~IPC() = default;

 public:
  ipc_error Publish(ipc_type type, void *data, int len);

 private:
  void Init();
  void ServerThread();
  void SubscribeThread(const uint8_t num);
  uint8_t Pub(const ipc_payload &data) const;

 private:
  std::thread thread_server_id_;
  std::vector<std::thread> thread_subscribe_id_{
      255};  // upper limit support subscribe 255
 private:
  int32_t sock_{};
  ipc_url owner_{};
  std::string name_{};
  std::vector<ipc_sub> subscribers_{};
};

}  // namespace ipc
}  // namespace ad
}  // namespace century
#endif  // century_AD_IPC_H_
