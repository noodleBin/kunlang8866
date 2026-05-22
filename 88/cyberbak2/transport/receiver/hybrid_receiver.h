/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#ifndef CYBER_TRANSPORT_RECEIVER_HYBRID_RECEIVER_H_
#define CYBER_TRANSPORT_RECEIVER_HYBRID_RECEIVER_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/types.h"
#include "cyber/proto/cyber_conf.pb.h"
#include "cyber/proto/role_attributes.pb.h"
#include "cyber/service_discovery/role/role.h"
#include "cyber/task/task.h"
#include "cyber/time/time.h"
#include "cyber/transport/receiver/intra_receiver.h"
#include "cyber/transport/receiver/rtps_receiver.h"
#include "cyber/transport/receiver/shm_receiver.h"
#include "cyber/transport/receiver/tcp_receiver.h"
#include "cyber/transport/rtps/participant.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/proto/topology_change.pb.h"

namespace century {
namespace cyber {
namespace transport {

using century::cyber::proto::OptionalMode;
using century::cyber::proto::QosDurabilityPolicy;
using century::cyber::proto::RoleAttributes;

template <typename M>
class HybridReceiver : public Receiver<M> {
 public:
  using HistoryPtr = std::shared_ptr<History<M>>;
  using ReceiverPtr = std::shared_ptr<Receiver<M>>;
  using ReceiverContainer =
      std::unordered_map<OptionalMode, ReceiverPtr, std::hash<int>>;
  using TransmitterContainer =
      std::unordered_map<OptionalMode,
                         std::unordered_map<uint64_t, RoleAttributes>,
                         std::hash<int>>;
  using CommunicationModePtr = std::shared_ptr<proto::CommunicationMode>;
  using MappingTable =
      std::unordered_map<Relation, OptionalMode, std::hash<int>>;

  HybridReceiver(const RoleAttributes& attr,
                 const typename Receiver<M>::MessageListener& msg_listener,
                 const ParticipantPtr& participant);
  virtual ~HybridReceiver();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

 private:
  void InitMode();
  void ObtainConfig();
  void InitHistory();
  void InitReceivers();
  void ClearReceivers();
  void InitTransmitters();
  void ClearTransmitters();
  OptionalMode SelectMode(const RoleAttributes& opposite_attr,
                          Relation relation);
  bool EnsureReceiver(OptionalMode mode);
  void ReannounceSelf();
  void ReceiveHistoryMsg(const RoleAttributes& opposite_attr);
  void ThreadFunc(const RoleAttributes& opposite_attr);
  Relation GetRelation(const RoleAttributes& opposite_attr);

  HistoryPtr history_;
  ReceiverContainer receivers_;
  TransmitterContainer transmitters_;
  std::mutex mutex_;

  CommunicationModePtr mode_;
  MappingTable mapping_table_;

  ParticipantPtr participant_;
};

template <typename M>
HybridReceiver<M>::HybridReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener,
    const ParticipantPtr& participant)
    : Receiver<M>(attr, msg_listener),
      history_(nullptr),
      participant_(participant) {
  InitMode();
  ObtainConfig();
  InitHistory();
  InitReceivers();
  InitTransmitters();
}

template <typename M>
HybridReceiver<M>::~HybridReceiver() {
  ClearTransmitters();
  ClearReceivers();
}

template <typename M>
void HybridReceiver<M>::Enable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : receivers_) {
    item.second->Enable();
  }
}

template <typename M>
void HybridReceiver<M>::Disable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : receivers_) {
    item.second->Disable();
  }
}

template <typename M>
void HybridReceiver<M>::Enable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  RETURN_IF(NO_RELATION == relation);

  auto mode = SelectMode(opposite_attr, relation);
  uint64_t id = opposite_attr.id();
  bool created = false;
  ReceiverPtr receiver;
  bool need_reannounce = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    created = EnsureReceiver(mode);
    if (0 == transmitters_[mode].count(id)) {
      transmitters_[mode].insert(std::make_pair(id, opposite_attr));
      receivers_[mode]->Enable(opposite_attr);
      ReceiveHistoryMsg(opposite_attr);
    }
    auto recv_iter = receivers_.find(mode);
    if (recv_iter != receivers_.end()) {
      receiver = recv_iter->second;
    }
  }
  if (OptionalMode::TCP == mode && receiver &&
      receiver->attributes().has_socket_addr()) {
    const auto& socket_addr = receiver->attributes().socket_addr();
    if (!this->attr_.has_socket_addr() ||
        this->attr_.socket_addr().port() != socket_addr.port() ||
        this->attr_.socket_addr().ip() != socket_addr.ip()) {
      this->attr_.mutable_socket_addr()->CopyFrom(socket_addr);
      need_reannounce = true;
    }
  }
  if (created && OptionalMode::TCP == mode) {
    need_reannounce = true;
  }
  if (need_reannounce) {
    ReannounceSelf();
  }
}

template <typename M>
void HybridReceiver<M>::Disable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  RETURN_IF(NO_RELATION == relation);

  auto mode = SelectMode(opposite_attr, relation);
  uint64_t id = opposite_attr.id();
  ReceiverPtr receiver_to_close = nullptr;
  bool reannounce = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto tx_iter = transmitters_.find(mode);
    if (tx_iter == transmitters_.end() ||
        0 == tx_iter->second.count(id)) {
      return;
    }
    tx_iter->second.erase(id);
    auto recv_iter = receivers_.find(mode);
    if (recv_iter == receivers_.end()) {
      return;
    }
    if (tx_iter->second.empty()) {
      receiver_to_close = recv_iter->second;
      receivers_.erase(recv_iter);
      transmitters_.erase(tx_iter);
      if (OptionalMode::TCP == mode && this->attr_.has_socket_addr()) {
        this->attr_.clear_socket_addr();
        reannounce = true;
      }
    } else {
      recv_iter->second->Disable(opposite_attr);
    }
  }
  if (receiver_to_close) {
    receiver_to_close->Disable();
  }
  if (reannounce) {
    ReannounceSelf();
  }
}

template <typename M>
void HybridReceiver<M>::InitMode() {
  mode_ = std::make_shared<proto::CommunicationMode>();
  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename M>
void HybridReceiver<M>::ObtainConfig() {
  proto::CyberConfig cfg;
  std::string conf("conf/");
  conf.append(common::GlobalData::Instance()->SchedName()).append(".conf");
  auto cfg_file = common::GetAbsolutePath(common::WorkRoot(), conf);
  if (!common::PathExists(cfg_file) || !common::GetProtoFromFile(cfg_file, &cfg)) {
    cfg = common::GlobalData::Instance()->Config();
  }
  if (!cfg.has_transport_conf()) {
    return;
  }
  if (!cfg.transport_conf().has_communication_mode()) {
    return;
  }
  mode_->CopyFrom(cfg.transport_conf().communication_mode());

  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename M>
void HybridReceiver<M>::InitHistory() {
  HistoryAttributes history_attr(this->attr_.qos_profile().history(),
                                 this->attr_.qos_profile().depth());
  history_ = std::make_shared<History<M>>(history_attr);
  if (QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL ==
      this->attr_.qos_profile().durability()) {
    history_->Enable();
  }
}

template <typename M>
void HybridReceiver<M>::InitReceivers() {
  receivers_.clear();
}

template <typename M>
void HybridReceiver<M>::ClearReceivers() {
  receivers_.clear();
}

template <typename M>
void HybridReceiver<M>::InitTransmitters() {
  transmitters_.clear();
}

template <typename M>
OptionalMode HybridReceiver<M>::SelectMode(
    const RoleAttributes& opposite_attr, Relation relation) {
  if (NO_RELATION == relation) {
    return OptionalMode::HYBRID;
  }
  if (opposite_attr.has_communication_mode()) {
    const auto& mode = opposite_attr.communication_mode();
    switch (relation) {
      case SAME_PROC:
        return mode.same_proc();
      case DIFF_PROC:
        return mode.diff_proc();
      case DIFF_HOST:
        return mode.diff_host();
      default:
        break;
    }
  }
  return mapping_table_[relation];
}

template <typename M>
bool HybridReceiver<M>::EnsureReceiver(OptionalMode mode) {
  if (receivers_.count(mode) > 0) {
    return false;
  }
  auto listener = std::bind(&HybridReceiver<M>::OnNewMessage, this,
                            std::placeholders::_1, std::placeholders::_2);
  switch (mode) {
    case OptionalMode::INTRA:
      receivers_[mode] =
          std::make_shared<IntraReceiver<M>>(this->attr_, listener);
      break;
    case OptionalMode::SHM:
      receivers_[mode] =
          std::make_shared<ShmReceiver<M>>(this->attr_, listener);
      break;
    case OptionalMode::TCP: {
      auto tcp_receiver =
          std::make_shared<TcpReceiver<M>>(this->attr_, listener);
      if (tcp_receiver->attributes().has_socket_addr()) {
        this->attr_.mutable_socket_addr()->CopyFrom(
            tcp_receiver->attributes().socket_addr());
      }
      receivers_[mode] = tcp_receiver;
      break;
    }
    default:
      receivers_[mode] =
          std::make_shared<RtpsReceiver<M>>(this->attr_, listener);
      break;
  }
  if (0 == transmitters_.count(mode)) {
    transmitters_[mode] = std::unordered_map<uint64_t, RoleAttributes>();
  }
  return true;
}

template <typename M>
void HybridReceiver<M>::ReannounceSelf() {
  auto channel_manager =
      service_discovery::TopologyManager::Instance()->channel_manager();
  if (!channel_manager) {
    return;
  }
  channel_manager->Leave(this->attr_, proto::RoleType::ROLE_READER);
  channel_manager->Join(this->attr_, proto::RoleType::ROLE_READER,
                        true);
}

template <typename M>
void HybridReceiver<M>::ClearTransmitters() {
  for (auto& item : transmitters_) {
    for (auto& upper_reach : item.second) {
      receivers_[item.first]->Disable(upper_reach.second);
    }
  }
  transmitters_.clear();
}

template <typename M>
void HybridReceiver<M>::ReceiveHistoryMsg(const RoleAttributes& opposite_attr) {
  // check qos
  if (opposite_attr.qos_profile().durability() !=
      QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL) {
    return;
  }

  auto attr = opposite_attr;
  cyber::Async(&HybridReceiver<M>::ThreadFunc, this, attr);
}

template <typename M>
void HybridReceiver<M>::ThreadFunc(const RoleAttributes& opposite_attr) {
  std::string channel_name =
      std::to_string(opposite_attr.id()) + std::to_string(this->attr_.id());
  uint64_t channel_id = common::GlobalData::RegisterChannel(channel_name);

  RoleAttributes attr(this->attr_);
  attr.set_channel_name(channel_name);
  attr.set_channel_id(channel_id);
  attr.mutable_qos_profile()->CopyFrom(opposite_attr.qos_profile());

  volatile bool is_msg_arrived = false;
  auto listener = [&](const std::shared_ptr<M>& msg,
                      const MessageInfo& msg_info, const RoleAttributes& attr) {
    is_msg_arrived = true;
    this->OnNewMessage(msg, msg_info);
  };

  auto receiver = std::make_shared<RtpsReceiver<M>>(attr, listener);
  receiver->Enable();

  do {
    if (is_msg_arrived) {
      is_msg_arrived = false;
    }
    cyber::USleep(1000000);
  } while (is_msg_arrived);

  receiver->Disable();
  ADEBUG << "recv threadfunc exit.";
}

template <typename M>
Relation HybridReceiver<M>::GetRelation(const RoleAttributes& opposite_attr) {
  if (opposite_attr.channel_name() != this->attr_.channel_name()) {
    return NO_RELATION;
  }

  if (opposite_attr.host_ip() != this->attr_.host_ip()) {
    return DIFF_HOST;
  }

  if (opposite_attr.process_id() != this->attr_.process_id()) {
    return DIFF_PROC;
  }

  return SAME_PROC;
}

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_RECEIVER_HYBRID_RECEIVER_H_
