/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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

// #include "cloud.h"
// #include "convert.h"
// #include "cloud.h"

#include <endian.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <type_traits>
#include <utility>

#include "modules/mcloud/proto/mcloud_info.pb.h"

#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/mcloud/cloud/cloud.h"
#include "modules/mcloud/cloud/convert.h"
#include "modules/mcloud/common/cloud_gflags.h"

using century::common::util::FillHeader;

constexpr const int CONTROL_COMMAND_LEN = 11;
constexpr uint8_t kTemporaryVehRelocationStationType = 0x08;


namespace std {
template <>
struct hash<std::tuple<century::mcloud::BoxType, century::mcloud::OperationType,
                       uint8_t>> {
  size_t operator()(
      const std::tuple<century::mcloud::BoxType, century::mcloud::OperationType,
                       uint8_t>& key) const {
    auto hash1 = std::hash<int>()(static_cast<int>(std::get<0>(key)));
    auto hash2 = std::hash<int>()(static_cast<int>(std::get<1>(key)));
    auto hash3 = std::hash<int>()(std::get<2>(key));
    return hash1 ^ (hash2 << 1) ^ (hash3 << 2);
  }
};
}  // namespace std

namespace century {
namespace mcloud {

namespace {

struct RoadEvent {
  uint8_t level = 0x00;
  uint32_t error_code = 0;
};

constexpr uint8_t kRoadEventHighLevel = 0x03;
constexpr uint8_t kRoadEventMediumLevel = 0x02;
constexpr uint8_t kRoadEventNormalLevel = 0x00;

template <typename T>
void AppendDataInReverse(std::vector<uint8_t>& data_unit, const T& src) {
  static_assert(std::is_trivially_copyable<T>::value,
                "AppendDataInReverse requires trivially copyable data.");
  const uint8_t* byte_src = reinterpret_cast<const uint8_t*>(&src);
  data_unit.insert(data_unit.end(),
                   std::reverse_iterator<const uint8_t*>(byte_src + sizeof(T)),
                   std::reverse_iterator<const uint8_t*>(byte_src));
}

// Append one normalized road event as: level + big-endian uint32 error code.
bool AppendRoadEvent(std::vector<uint8_t>& data_unit, const size_t count_pos,
                     const RoadEvent& event) {
  if (data_unit[count_pos] == 0xFF) {
    return false;
  }
  data_unit.emplace_back(event.level);
  data_unit.emplace_back((event.error_code >> 24) & 0xFF);
  data_unit.emplace_back((event.error_code >> 16) & 0xFF);
  data_unit.emplace_back((event.error_code >> 8) & 0xFF);
  data_unit.emplace_back(event.error_code & 0xFF);
  ++data_unit[count_pos];
  return true;
}

bool AppendRoadEventIfBuilt(std::vector<uint8_t>& data_unit,
                            const size_t count_pos,
                            const bool event_built,
                            const RoadEvent& event) {
  return event_built && AppendRoadEvent(data_unit, count_pos, event);
}

// Monitor uses code_msg as the stable key for cloud-code mapping.
bool BuildMonitorRoadEvent(const century::monitor::FaultData& fault,
                           RoadEvent* event) {
  if (event == nullptr) {
    return false;
  }

  static constexpr std::pair<const char*, uint32_t> kMonitorErrorCodeMap[] = {
      {"LOW_FREQ_LOCALIZATION", 0x400},
      {"NO_FREQ_LOCALIZATION", 0x401},
      {"LOCALIZATION_STATUS_ABNORMAL", 0x402},
      {"LOW_FREQ_PERCEPTION", 0x410},
      {"NO_FREQ_PERCEPTION", 0x411},
      {"LOW_FREQ_PLANNING", 0x420},
      {"NO_FREQ_PLANNING", 0x421},
      {"LOW_FREQ_CONTROL", 0x430},
      {"NO_FREQ_CONTROL", 0x431},
      {"LOW_FREQ_PREDICTION", 0x440},
      {"NO_FREQ_PREDICTION", 0x441},
      {"NO_FREQ_TRAFFICLIGHT", 0x450},
      {"LOW_FREQ_CHASSIS", 0x460},
      {"NO_FREQ_CHASSIS", 0x461},
      {"EXCEEDING_TIME_DIFF", 0x470},
  };
  for (const auto& mapping : kMonitorErrorCodeMap) {
    if (fault.code_msg() == mapping.first) {
      event->level = kRoadEventHighLevel;
      event->error_code = mapping.second;
      return true;
    }
  }

  return false;
}

// Planning status uses internal ErrorCode enums; map them to cloud codes here.
bool BuildPlanningRoadEvent(const century::common::StatusPb& status,
                            RoadEvent* event) {
  if (event == nullptr || status.error_code() == century::common::OK) {
    return false;
  }

  static constexpr std::pair<century::common::ErrorCode, uint32_t>
      kPlanningErrorCodeMap[] = {
          {century::common::PLANNING_ERROR, 0x100},
          {century::common::PLANNING_ERROR_NOT_READY, 0x101},
          {century::common::PLANNING_ERROR_NEED_RESTART, 0x102},
      };
  for (const auto& mapping : kPlanningErrorCodeMap) {
    if (status.error_code() == mapping.first) {
      event->level = kRoadEventHighLevel;
      event->error_code = mapping.second;
      return true;
    }
  }

  return false;
}

// Decision stop reason codes have a dedicated cloud-code mapping table.
bool BuildDecisionStopRoadEvent(
    const century::planning::MainStop& stop_decision, RoadEvent* event) {
  if (event == nullptr || !stop_decision.has_reason_code()) {
    return false;
  }

  static constexpr std::pair<century::planning::StopReasonCode, uint32_t>
      kDecisionStopErrorCodeMap[] = {
          {century::planning::STOP_REASON_HEAD_VEHICLE, 0x200},
          {century::planning::STOP_REASON_DESTINATION, 0x201},
          {century::planning::STOP_REASON_PEDESTRIAN, 0x202},
          {century::planning::STOP_REASON_OBSTACLE, 0x203},
          {century::planning::STOP_REASON_PREPARKING, 0x204},
          {century::planning::STOP_REASON_SPEED, 0x205},
          {century::planning::STOP_REASON_STACKER, 0x206},
          {century::planning::STOP_REASON_WEIGHING_POINT, 0x207},
          {century::planning::STOP_REASON_WHEEL_CRANE, 0x208},
          {century::planning::STOP_REASON_TEMPORARY_PARKING, 0x209},
          {century::planning::STOP_REASON_LINE_UP_STOP_POINT, 0x20A},
          {century::planning::STOP_REASON_SIGNAL, 0x20B},
          {century::planning::STOP_REASON_STOP_SIGN, 0x20C},
          {century::planning::STOP_REASON_YIELD_SIGN, 0x20D},
          {century::planning::STOP_REASON_CLEAR_ZONE, 0x20E},
          {century::planning::STOP_REASON_CROSSWALK, 0x20F},
          {century::planning::STOP_REASON_CREEPER, 0x210},
          {century::planning::STOP_REASON_REFERENCE_END, 0x211},
          {century::planning::STOP_REASON_YELLOW_SIGNAL, 0x212},
          {century::planning::STOP_REASON_PULL_OVER, 0x213},
          {century::planning::STOP_REASON_SIDEPASS_SAFETY, 0x214},
          {century::planning::STOP_REASON_PRE_OPEN_SPACE_STOP, 0x215},
          {century::planning::STOP_REASON_LANE_CHANGE_URGENCY, 0x216},
          {century::planning::STOP_REASON_EMERGENCY, 0x217},
          {century::planning::STOP_REASON_SLOW_BREAKING, 0x218},
          {century::planning::STOP_REASON_LOW_RIGHT, 0x219},
          {century::planning::STOP_REASON_ELECTRIC_FENCE, 0x21A},
      };
  for (const auto& mapping : kDecisionStopErrorCodeMap) {
    if (stop_decision.reason_code() == mapping.first) {
      event->level =
          stop_decision.reason_code() == century::planning::STOP_REASON_DESTINATION
              ? kRoadEventNormalLevel
              : kRoadEventMediumLevel;
      event->error_code = mapping.second;
      return true;
    }
  }

  return false;
}

}  // namespace

uint64_t htonll(uint64_t value) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
  return ((uint64_t)htonl((uint32_t)(value & 0xFFFFFFFFULL)) << 32) |
         htonl((uint32_t)(value >> 32));
#elif __BYTE_ORDER == __BIG_ENDIAN
  return value;
#else
#error "not ensure code"
#endif
}

uint64_t ntohll(uint64_t value) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
  return ((uint64_t)ntohl((uint32_t)(value & 0xFFFFFFFFULL)) << 32) |
         ntohl((uint32_t)(value >> 32));
#elif __BYTE_ORDER == __BIG_ENDIAN
  return value;
#else
#error "not ensure code"
#endif
}

CircularBuffer::CircularBuffer(size_t size)
    : buffer(size), head(0), tail(0), count(0) {}

void CircularBuffer::push(const uint8_t* data, size_t length) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < length; ++i) {
    buffer[head] = data[i];
    head = (head + 1) % buffer.size();
    if (count < buffer.size()) {
      ++count;
    } else {
      tail = (tail + 1) % buffer.size();
    }
  }
}

std::vector<uint8_t> CircularBuffer::get_data() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  for (size_t i = 0; i < count; ++i) {
    data.emplace_back(buffer[(tail + i) % buffer.size()]);
  }
  return data;
}

size_t CircularBuffer::available() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return count;
}

void CircularBuffer::pop(size_t length) {
  std::lock_guard<std::mutex> lock(mutex_);
  tail = (tail + length) % buffer.size();
  count -= length;
}

Cloud::Cloud() : buffer_(buffer_size), node_(cyber::CreateNode("cloud")) {
  stop_flag_ = false;

  gflags::ReadFromFlagsFile(FLAGS_cloud_server_conf, "cloud", true);

  offset_x_ = FLAGS_offset_x;
  offset_y_ = FLAGS_offset_y;
  zone_id_ = FLAGS_zone_id;

  // routing_request_writer_ =
  //     node_->CreateWriter<century::routing::RoutingRequest>("/century/routing_request");
  localization_reader_ =
      node_->CreateReader<century::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  routing_response_request_reader_ =
      node_->CreateReader<century::routing::RoutingResponse>(
          FLAGS_routing_response_topic);

  chassis_reader_ =
      node_->CreateReader<century::canbus::Chassis>(FLAGS_chassis_topic);
  control_command_reader_ =
      node_->CreateReader<century::control::ControlCommand>(
          FLAGS_control_command_topic);
  system_monitor_x86_reader_ =
      node_->CreateReader<century::monitor::MonitoredData>(
          FLAGS_system_monitor_x86_topic);
  system_monitor_aarch_reader_ =
      node_->CreateReader<century::monitor::MonitoredData>(
          FLAGS_system_monitor_aarch_topic);

  planning_reader_ = node_->CreateReader<century::planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<century::planning::ADCTrajectory>& result) {
        //ToDo
      });

  routing_result_reader_ = node_->CreateReader<century::routing::RoutingResult>(
      "/century/routing_result",
      [this](const std::shared_ptr<century::routing::RoutingResult>& result) {
        this->OnRoutingResult(result);
      });

  routing_request_reader_ = node_->CreateReader<century::routing::RoutingRequest>(
      "/century/routing_request",
      [this](const std::shared_ptr<century::routing::RoutingRequest>& result) {
        //ToDo
      });

  routing_routing_responses_reader_ =
      node_->CreateReader<century::routing::RoutingResponses>(
          "/century/routing_responses",
          [this](const std::shared_ptr<century::routing::RoutingResponses>&
                     result) {
            auto append_data = [&](const void* src, size_t size) {
              const uint8_t* byte_src = reinterpret_cast<const uint8_t*>(src);
              multi_responses_.insert(
                  multi_responses_.end(),
                  std::reverse_iterator<const uint8_t*>(byte_src + size),
                  std::reverse_iterator<const uint8_t*>(byte_src));
            };
            std::unique_lock<std::mutex> lock(responses_mutex_);
            std::vector<uint8_t>().swap(multi_responses_);
            multi_responses_.emplace_back(result->routing_response().size());
            if (result->routing_response().size() <= 1) {
              return;
            }
            int8_t lane_size = 0;
            int32_t idx = 1;
            for (const auto& routing_response : result->routing_response()) {
              for (const auto& point : routing_response.road_points()) {
                append_data(&idx, sizeof(int32_t));
                auto x = point.x();
                lane_size++;

                auto y = point.y();
                double lat = 0.0;
                double lon = 0.0;
                double utm_x = offset_x_ + x;
                double utm_y = offset_y_ + y;

                UtmtoWgs84(utm_x, utm_y, zone_id_.c_str(), &lon, &lat, nullptr,
                           nullptr);

                lon *= RAD_TO_DEG;
                lat *= RAD_TO_DEG;
                uint64_t t_lat = lat * 1e8;
                uint64_t t_lon = lon * 1e8;
                append_data(&t_lon, sizeof(uint64_t));
                append_data(&t_lat, sizeof(uint64_t));

                multi_responses_.emplace_back(0x00);
                multi_responses_.emplace_back(0x00);
                multi_responses_.emplace_back(0x00);
                multi_responses_.emplace_back(0x00);
              }
              idx++;
            }
            multi_responses_[0] = lane_size;
            AINFO << "multi_responses_ size: " << static_cast<int>(lane_size)
                  << "  " << multi_responses_.size();
          });

  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name("/century/routing_request");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  routing_request_writer_ =
      node_->CreateWriter<century::routing::RoutingRequest>(writer_attr);


  writer_attr.set_channel_name("/century/mcloud");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  cloud_info_writer_ =
      node_->CreateWriter<century::mcloud::McloudInfo>(writer_attr);

  writer_attr.set_channel_name("/century/blocking_area_response");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  blocking_area_writer_ =
      node_->CreateWriter<century::planning::BlockingAreaResponse>(writer_attr);

  writer_attr.set_channel_name("/century/v2x_info");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  v2x_info_writer_ =
      node_->CreateWriter<century::planning::V2xInfo>(writer_attr);

  writer_attr.set_channel_name("/century/borrow_response");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  borrow_writer_ =
      node_->CreateWriter<century::planning::BorrowResponse>(writer_attr);

  writer_attr.set_channel_name("/century/pass_stacker_response");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  pass_stacker_writer_ =
      node_->CreateWriter<century::planning::PassStackerResponse>(writer_attr);

  writer_attr.set_channel_name("/century/temporary_parking_request");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  temporary_parking_request_writer_ =
      node_->CreateWriter<century::planning::TemporaryParkingRequest>(
          writer_attr);

  writer_attr.set_channel_name("/century/multi_path_temp_stop_request");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  multi_temporary_parking_request_writer_ =
      node_->CreateWriter<century::planning::TemporaryParkingRequest>(
          writer_attr);

  writer_attr.set_channel_name("/century/stackers_info");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  stacker_info_writer_ =
      node_->CreateWriter<century::planning::StackersInfo>(writer_attr);


  writer_attr.set_channel_name("/century/mcloud/emergency_stop");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  emergency_stop_ = node_->CreateWriter<century::mcloud::EmergencyStop>(writer_attr);

  writer_attr.set_channel_name("/century/background_music");
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  bck_music_writer_ = node_->CreateWriter<century::dreamview::BackgroundMusic>(writer_attr);

  std::string status;
  VehicleMessage message;

  fas_aeb_backend_client_ =
      node_->CreateClient<century::fas_aeb_backend::Request,
                          century::fas_aeb_backend::Response>(
          "fas_aeb_backend");
  fas_aeb_info_reader_ =
      node_->CreateReader<century::fas_aeb_backend::FasAebInfo>(
          "/century/fas_aeb_info");
  if (LoadRoutingTask(&message, status) && "continue" == status) {
    ReceTask(message);
  }
}

century::cyber::proto::QosProfile Cloud::CreateQosProfile() {
  century::cyber::proto::QosProfile qos;
  qos.set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos.set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos.set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  return qos;
}

void Cloud::Start() {
  recv_task_ = std::unique_ptr<std::thread>(
      new std::thread(&Cloud::ProcessCommands, this));
  send_task_ =
      std::unique_ptr<std::thread>(new std::thread(&Cloud::Report, this));
}

century::routing::TaskType Cloud::ConvertType(uint8_t task_type,
                                              uint8_t pose_type,
                                              uint8_t station_type,
                                              uint8_t boxes) {
  // station_type unused
  // Define mapping of input to TaskType
  static const std::unordered_map<std::tuple<BoxType, OperationType, uint8_t>,
                                  century::routing::TaskType>
      taskMap = {
          // Single Box - Loading
          {{BoxType::SINGLE_BOX, OperationType::LOADING,
            AreaType::YARD_OPERATION_AREA},
           century::routing::TaskType::YARD_OPERATIONAREA_DYNAMIC},
          {{BoxType::SINGLE_BOX, OperationType::LOADING,
            AreaType::YARD_WAITING_AREA},
           century::routing::TaskType::YARD_WAITINGAREA_STATIC},
          {{BoxType::SINGLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_OPERATION_AREA},
           century::routing::TaskType::RAILWAY_OPERATIONAREA_DYNAMIC},
          {{BoxType::SINGLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_FIXED_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_STATIC},
          {{BoxType::SINGLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_DYNAMIC},

          // Single Box - Unloading
          {{BoxType::SINGLE_BOX, OperationType::UNLOADING,
            AreaType::YARD_OPERATION_AREA},
           century::routing::TaskType::YARD_OPERATIONAREA_STATIC},
          {{BoxType::SINGLE_BOX, OperationType::UNLOADING,
            AreaType::YARD_WAITING_AREA},
           century::routing::TaskType::YARD_WAITINGAREA_STATIC},
          {{BoxType::SINGLE_BOX, OperationType::UNLOADING,
            AreaType::RAILWAY_OPERATION_AREA},
           century::routing::TaskType::RAILWAY_OPERATIONAREA_DYNAMIC},
          {{BoxType::SINGLE_BOX, OperationType::UNLOADING,
            AreaType::RAILWAY_FIXED_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_STATIC},
          {{BoxType::SINGLE_BOX, OperationType::UNLOADING,
            AreaType::POWER_RECHARGE_AREA},
           century::routing::TaskType::PARKINGSPACE},

          // Double Box - Loading
          {{BoxType::DOUBLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_FIXED_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_STATIC},
          {{BoxType::DOUBLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_DYNAMIC},
          {{BoxType::DOUBLE_BOX, OperationType::LOADING,
            AreaType::RAILWAY_OPERATION_AREA},
           century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_1},
          {{BoxType::DOUBLE_BOX, OperationType::UNLOADING,
            AreaType::RAILWAY_FIXED_WAITING_AREA},
           century::routing::TaskType::RAILWAY_WAITINGAREA_STATIC},
          {{BoxType::DOUBLE_BOX, OperationType::UNLOADING,
            AreaType::YARD_WAITING_AREA},
           century::routing::TaskType::YARD_WAITINGAREA_STATIC},
          {{BoxType::DOUBLE_BOX, OperationType::UNLOADING,
            AreaType::RAILWAY_OPERATION_AREA},
           century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1},
          {{BoxType::DOUBLE_BOX, OperationType::LOADING,
            AreaType::YARD_WAITING_AREA},
           century::routing::TaskType::YARD_WAITINGAREA_STATIC},
          {{BoxType::DOUBLE_BOX, OperationType::LOADING,
            AreaType::YARD_OPERATION_AREA},
           century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_1},
          {{BoxType::DOUBLE_BOX, OperationType::UNLOADING,
            AreaType::YARD_OPERATION_AREA},
           century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1},
      };
      
  if (kTemporaryVehRelocationStationType == station_type) {
    return century::routing::TaskType::TEMPORARY_VEH_RELOCATION;
  }

  // Look up TaskType in the map
  auto it = taskMap.find(std::make_tuple(static_cast<BoxType>(boxes),
                                         static_cast<OperationType>(task_type),
                                         station_type));
  if (taskMap.end() != it) {
    AINFO << "convert type : " << century::routing::TaskType_Name(it->second);
    return it->second;
  }

  return century::routing::TaskType::DEFAULT;
}

void Cloud::ConvertBoxStatus() {
  if (BoxType::SINGLE_BOX == box_type_) {
    bool is_yard_or_railway_operation =
        (century::routing::TaskType::YARD_OPERATIONAREA_STATIC == task_type_ ||
         century::routing::TaskType::YARD_OPERATIONAREA_DYNAMIC == task_type_ ||
         century::routing::TaskType::RAILWAY_OPERATIONAREA_DYNAMIC == task_type_);
    system_info_.box_status =
        is_loading_ * 2 + (is_yard_or_railway_operation ? 2 : 1);
    system_info_.box_status_back = 0;
  } else {
    switch (task_type_) {
      case century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1:
      case century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0:
        system_info_.box_status_back = is_loading_ * 2 + 2;
        break;

      case century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0:
      case century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_1:
        system_info_.box_status = is_loading_ * 2 + 2;
        break;

      case century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_2:
      case century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2:
        break;

      default:
        system_info_.box_status = is_loading_ * 2 + 1;
        system_info_.box_status_back = is_loading_ * 2 + 1;
        break;
    }
  }
}

void Cloud::HandRequestTask() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    if (!client_->IsConnect()) {
      continue;
    }
    std::lock_guard<std::mutex> lock(request_mutex);
    for (auto& it : request_data) {
      const auto& msg = it.second;
      if (client_) {
        client_->SendMessage(msg);
        AINFO << "The request information has been resent. " << it.first;
      }
    }
  }
}

void Cloud::CheckAndSendDefaultData() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       now - last_stacker_info_time_)
                       .count();
    century::planning::StackersInfo stackers_info;

    if (elapsed > 2) {
      AINFO << "No new message data for a while, sending default data.";
      FillHeader("stackers_info", &stackers_info);
      stacker_info_writer_->Write(stackers_info);
    }
  }
}

void Cloud::Report() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!client_->IsConnect()) {
      continue;
    }
    // get planning info
    planning_reader_->Observe();
    auto msg = planning_reader_->GetLatestObserved();
    if (msg) {
      std::vector<uint8_t> data_unit;
      if (msg->has_reached_station()) {
        reached_station_ = true;
        std::thread([this]() {
          if (!this) {
            return;
          }

          ConvertBoxStatus();
          AINFO << "system_info_.box_status : "
                << static_cast<unsigned int>(system_info_.box_status) << " "
                << "system_info_.box_status_back : "
                << static_cast<unsigned int>(system_info_.box_status_back);
        }).detach();

        VehicleMessage message(0x06, 0x01, FLAGS_vehicle_unique_id, 0x01,
                               data_unit);
        ModifyLastRoutingTask("success");
        Reply(message, seq_, 0x03);
      }
    }
    GetSystemInfo();
    SendSystemInfo();
  }
}

void Cloud::GetSystemInfo() {}

void Cloud::HandleData(const std::vector<uint8_t>& data) {
  buffer_.push(data.data(), data.size());

  while (true) {
    auto currentData = buffer_.get_data();

    if (currentData.size() < 24) {
      break;
    }

    auto startPos = std::search(currentData.begin(), currentData.end(),
                                startFlag.begin(), startFlag.end());
    if (currentData.end() == startPos) {
      break;
    }

    if (std::distance(startPos, currentData.end()) < 24) {
      break;
    }

    size_t dataUnitLength = (*(startPos + 22) << 8) | *(startPos + 23);

    size_t messageLength = 24 + dataUnitLength + 1;

    if (std::distance(startPos, currentData.end()) < messageLength) {
      break;
    }

    std::vector<uint8_t> message(startPos, startPos + messageLength);

    try {
      VehicleMessage vehicle_message = VehicleMessage::unpack_message(message);

      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_que_.push(vehicle_message);
      }
      queue_cond_.notify_one();
    } catch (const std::invalid_argument& e) {
    }

    buffer_.pop(messageLength);
  }
}

void Cloud::RegisterClient(TcpClient* client, TcpClient* mqttClient) {
  client_ = client;
  mqttClient_ = mqttClient;
}

void Cloud::ProcessCommands() {
  while (true) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cond_.wait(lock,
                     [this] { return !command_que_.empty() || stop_flag_; });

    if (stop_flag_ && command_que_.empty()) {
      break;
    }

    while (!command_que_.empty()) {
      VehicleMessage message = command_que_.front();
      command_que_.pop();
      ProcessCommand(message);
    }
  }
}

void printHex(const std::vector<uint8_t>& data_unit,
              std::string msg_type = "") {
  std::ostringstream oss;
  for (uint8_t byte : data_unit) {
    oss << "0x" << std::hex << std::uppercase << std::setw(2)
        << std::setfill('0') << static_cast<int>(byte) << " ";
  }
  AINFO << msg_type << " : " << oss.str();
}

void Cloud::ProcessCommand(const VehicleMessage& message) {
  AINFO << "command_id : " << message.command_id;
  // printHex(message.data_unit);
  switch (message.command_id) {
    case CONTROL_REQUEST:
      ControlCommand(message);
      break;
    case TASK_RECV:
      ReceTask(message);
      break;
    case TASK_CONTRONL:
      TaskContronl(message);
      break;
    case PARAMETER_QUERY:
      ParameterQuery(message);
      break;
    case SET_QUERY:
      SetQuery(message);
      break;
    // MQTT message handling is migrated to cloud_mqtt.cc to avoid duplicate
    // consumption paths in cloud.cc and cloud_mqtt.cc.
    default:
      AINFO << "Unknown Command ID: " << message.command_id;
      break;
  }
}

void Cloud::SetQuery(const VehicleMessage& message) {
  auto data = message.data_unit;
  uint32_t sequence = 0;
  std::memcpy(&sequence, &data[6], sizeof(sequence));
  sequence = ntohl(sequence);

  uint8_t count = data[10];
  count;

  std::vector<uint8_t> data_unit = GetTime();

  for (size_t i = 6; i < data.size(); i++) {
    data_unit.emplace_back(data[i]);
  }

  for (int i = 1; i <= count; i++) {
    AINFO << "set_query : " << static_cast<int>(data[10 + i]);
    if (0x04 == data[10 + i]) {
      uint16_t speed_limit;
      std::memcpy(&speed_limit, &data[12], sizeof(speed_limit));
      speed_limit = ntohs(speed_limit);
      AINFO << "speed_limit : " << speed_limit;

      std::ofstream configFile("/century/modules/planning/conf/planning.conf",
                               std::ios::app);
      if (!configFile) {
        std::cerr << "planning.conf open error" << std::endl;
        return;
      }

      double speed_limit_mps = static_cast<double>(speed_limit) / 36;
      speed_limit_mps = std::round(speed_limit_mps * 10) / 10;

      std::string line = "\n--planning_upper_speed_limit = " +
                         std::to_string(speed_limit_mps) + "\n";
      configFile << line;
      configFile.close();

      century::monitor::KillProcessByName("planning.dag");
    } else if (0x06 == data[10 + i]) {
      uint8_t fas_aeb;
      std::memcpy(&fas_aeb, &data[12], sizeof(fas_aeb));
      if (fas_aeb_backend_client_->ServiceIsReady()) {
        nlohmann::json fas_aeb_info;
        AINFO << "fas_aeb : " << static_cast<int>(fas_aeb);
        auto request = std::make_shared<century::fas_aeb_backend::Request>();
        request->set_type(
            century::fas_aeb_backend::Request::MODIFY_CONFIG_CLOUD);
        request->mutable_modify_config()->set_key("switch_fas_aeb");
        request->mutable_modify_config()->set_value(0x02 == fas_aeb ? "true"
                                                                    : "false");
        auto ret = fas_aeb_backend_client_->SendRequest(request);
      }
    } else if (0x07 == data[10 + i]) {
      uint8_t enable;
      std::memcpy(&enable, &data[12], sizeof(enable));
      auto request =
          std::make_shared<century::planning::TemporaryParkingRequest>();
      common::util::FillHeader("temporary_parking_request", request.get());

      std::cout << "enable : 0x" << std::hex << (int)enable << std::endl;
      temporary_parking_ = enable;

      request->set_need_stop(0x02 == enable);
      temporary_parking_request_writer_->Write(request);
    } else if (0x08 == data[10 + i]) {
      uint8_t immediately_stop = 0;
      std::memcpy(&immediately_stop, &data[12], sizeof(immediately_stop));
      AINFO << "recv immediately stop : " << std::hex << immediately_stop;
      bool stop = 0x02 == immediately_stop ? true : false;
      immediately_stop_ = stop;
      HandleImmediatelyStopCommand(stop);
    } else if (0x0A == data[10 + i]) {
      uint8_t bck_music;
      std::memcpy(&bck_music, &data[12], sizeof(bck_music));
      auto request =
          std::make_shared<century::dreamview::BackgroundMusic>();
      common::util::FillHeader(FLAGS_cloud_module_name, request.get());
      const auto now_sec = century::cyber::Time::Now().ToSecond();
      request->set_last_modify_time(now_sec);
      request->set_background_music_switch(0x02 == bck_music ? true : false);
      bck_music_writer_->Write(request);
    }
  }

  VehicleMessage data_send(message.command_id, 0x01, message.unique_id,
                           message.encryption_method, data_unit);
  auto msg = data_send.pack_message();
  if (client_) {
    printHex(msg);
    client_->SendMessage(msg);
  }
}

void Cloud::ParameterQuery(const VehicleMessage& message) {
  auto data = message.data_unit;
  uint32_t sequence = 0;
  std::memcpy(&sequence, &data[6], sizeof(sequence));
  int count = data[10];

  std::vector<uint8_t> data_unit = GetTime();
  for (int i = 0; i < 4; i++) {
    data_unit.emplace_back(data[6 + i]);
  }
  data_unit.emplace_back(count);
  for (int i = 1; i <= count; i++) {
    AINFO << "parameter_query : " << static_cast<int>(data[10 + i]);
    if (0x01 == data[10 + i]) {
      data_unit.emplace_back(0xFF);
      data_unit.emplace_back(0xFF);
    } else if (0x02 == data[10 + i]) {
      data_unit.emplace_back(0xFF);
      data_unit.emplace_back(0xFF);
    } else if (0x03 == data[10 + i]) {
      data_unit.emplace_back(0xFF);
      data_unit.emplace_back(0xFF);
    } else if (0x04 == data[10 + i]) {
    } else if (0x05 == data[10 + i]) {
    } else if (0x0A == data[10 + i]) {
      planning_reader_->Observe();
      auto planning_msg = planning_reader_->GetLatestObserved();
      if (planning_msg && planning_msg->has_background_music_enable()) {
        background_music_enable_ = planning_msg->background_music_enable();
        data_unit.emplace_back(background_music_enable_ ? 0x02 : 0x01);
      } else {
        data_unit.emplace_back(0xFF);
      }
    }
  }

  VehicleMessage data_send(message.command_id, 0x01, message.unique_id,
                           message.encryption_method, data_unit);
  auto msg = data_send.pack_message();
  if (client_) {
    printHex(msg);
    client_->SendMessage(msg);
  }
}

void Cloud::ReceTask(const VehicleMessage& message) {
  const auto& data = message.data_unit;

  if (data.size() < 11) {
    AERROR << "ReceTask: invalid data size: " << data.size();
    return;
  }

  size_t idx = 0;

  idx += 6;

  uint32_t sequence = 0;
  std::memcpy(&sequence, &data[idx], sizeof(sequence));
  sequence = ntohl(sequence);
  seq_ = sequence;
  idx += sizeof(sequence);

  uint8_t commandID = data[idx++];
  AINFO << "sequence : " << sequence
        << " commandID: " << static_cast<int>(commandID);

  if (0x01 == commandID) {
    SaveRoutingTask(message, "continue");

    if (data.size() < 38) {
      Reply(message, sequence, false);
      return;
    }

    int status = data[idx++];
    if (status) {
      Reply(message, sequence, false);
      return;
    }

    uint32_t task_info = 0;
    std::memcpy(&task_info, &data[idx], sizeof(task_info));
    task_info = ntohl(task_info);
    idx += sizeof(task_info);

    uint8_t is_loading = static_cast<uint8_t>(task_info & 0x00000001);
    uint8_t boxes = static_cast<uint8_t>((task_info & 0b00011000) >> 3);

    box_type_ = static_cast<BoxType>(boxes);

    uint8_t crane_id_len = data[idx++];

    if (idx + crane_id_len > data.size()) {
      AERROR << "Invalid crane_id length";
      return;
    }

    std::string crane_id(reinterpret_cast<const char*>(&data[idx]),
                         crane_id_len);

    idx += crane_id_len;

    AINFO << "Crane ID: " << crane_id << std::endl;

    uint16_t pose_size = 0;
    std::memcpy(&pose_size, &data[idx], sizeof(pose_size));
    pose_size = ntohs(pose_size);
    idx += sizeof(pose_size);

    uint64_t raw_longitude = 0;
    std::memcpy(&raw_longitude, &data[idx], sizeof(raw_longitude));
    idx += sizeof(raw_longitude);

    uint64_t raw_latitude = 0;
    std::memcpy(&raw_latitude, &data[idx], sizeof(raw_latitude));
    idx += sizeof(raw_latitude);

    raw_longitude = ntohll(raw_longitude);
    raw_latitude = ntohll(raw_latitude);

    double longitude = raw_longitude / 1e8;
    double latitude = raw_latitude / 1e8;

    uint16_t direction_angle = 0;
    std::memcpy(&direction_angle, &data[idx], sizeof(direction_angle));
    direction_angle = ntohs(direction_angle);
    idx += sizeof(direction_angle);

    uint8_t pose_type = data[idx++];

    uint8_t station_type = data[idx++];

    is_loading_ = is_loading;
    task_type_ = ConvertType(is_loading, pose_type, station_type, boxes);
    ConvertBoxStatus();

    AINFO << std::fixed << std::setprecision(10) << " jobInfo : " << task_info
          << " longitude: " << longitude << " latitude: " << latitude
          << " direction Angle: " << direction_angle
          << " targetType : " << static_cast<int>(pose_type);

    bool result = SendRoutingRequest(
        longitude, latitude, direction_angle, is_loading, pose_type,
        ConvertType(is_loading, pose_type, station_type, boxes), crane_id);

    if (!result) {
      ModifyLastRoutingTask("fail");
      Reply(message, sequence, 0x06);
      return;
    }

  } else if (0x02 == commandID) {
    int status = data[idx++];
    (void)status;

    uint16_t seq = 0;
    std::memcpy(&seq, &data[idx], sizeof(seq));
    seq = ntohs(seq);
    idx += sizeof(seq);

    uint16_t pose_size = 0;
    std::memcpy(&pose_size, &data[idx], sizeof(pose_size));
    pose_size = ntohs(pose_size);
    idx += sizeof(pose_size);

    Reply(message, sequence, true);

  } else if (0x03 == commandID) {
  }
}

void Cloud::ControlCommand(const VehicleMessage& message) {
  const auto& data = message.data_unit;
  if (data.size() < CONTROL_COMMAND_LEN) {
    AERROR << "Invalid data size: " << data.size();
    return;
  }

  uint32_t sequence = 0;
  std::memcpy(&sequence, &data[6], sizeof(sequence));
  sequence = ntohl(sequence);
  // seq_ = sequence;

  uint8_t command_id = data[10];

  using CommandHandler = void (Cloud::*)(const VehicleMessage&, uint32_t);
  static const std::unordered_map<uint8_t, CommandHandler> command_handlers = {
      {CONTROL_MESSAGE::REBOOT, &Cloud::HandleRebootCommand},
      {CONTROL_MESSAGE::FINE_TUNING, &Cloud::HandleFineCommand},
      {CONTROL_MESSAGE::MOVE, &Cloud::HandleMoveCommand},
      {CONTROL_MESSAGE::STOP, &Cloud::HandleStopCommand},
      {CONTROL_MESSAGE::MULTI_BEGIN, &Cloud::HandleMultiBegin},
      {CONTROL_MESSAGE::MULTI_SELECT, &Cloud::HandleMultiSelect},
      {CONTROL_MESSAGE::IMMEDIATELY_ARRIVE,
       &Cloud::HandleImmediatelyArriveCommand},
      {CONTROL_MESSAGE::TASK_STATUS_CONTRONL,
       &Cloud::HandleDataTaskStatusCommand},
      {CONTROL_MESSAGE::INFORM_BORROW, &Cloud::HandleBorrowCommand}};

  auto handlerIt = command_handlers.find(command_id);
  if (command_handlers.end() != handlerIt) {
    (this->*(handlerIt->second))(message, sequence);
  }

  std::vector<uint8_t> sendData(data.begin(), data.end());
  VehicleMessage reply(0xCA, 0x01, message.unique_id, 0x01, sendData);
  auto msg = reply.pack_message();

  if (client_) {
    printHex(msg);
    client_->SendMessage(msg);
  }
}

void Cloud::HandleMultiBegin(const VehicleMessage& message, uint32_t sequence) {
    AINFO << "recv multi request";
    FillHeader("cloud", last_routing_request_.get());
    auto *begin = last_routing_request_->mutable_waypoint(0);
    localization_reader_->Observe();
    auto msg = localization_reader_->GetLatestObserved();
    if (!msg) {
      AERROR << "No Localization Message Found!";
    }
    double nearest_s, nearest_l;
    century::hdmap::LaneInfoConstPtr line_id;
    if (GetNearestLaneWithHeading(
            msg->pose().position().x(), msg->pose().position().y(), &line_id,
            &nearest_s, &nearest_l, msg->pose().heading())) {
      begin->set_s(nearest_s);
    }

    begin->mutable_pose()->set_x(msg->pose().position().x());
    begin->mutable_pose()->set_y(msg->pose().position().y());
    begin->set_heading(msg->pose().heading());    

    last_routing_request_->set_multi_routing_type(routing::MultipleRoutingType::PROCESSINGSTART);
    routing_request_writer_->Write(last_routing_request_);
}

void Cloud::HandleMultiSelect(const VehicleMessage& message, uint32_t sequence) {
    uint32_t path_id = 0;
    auto data = message.data_unit;
    // AINFO << "recv mulit select request";
    std::memcpy(&path_id, &data[11], sizeof(path_id));
    path_id = ntohl(path_id);
    path_id = path_id - 1;
    {
      std::unique_lock<std::mutex> lock(responses_mutex_);
      std::vector<uint8_t>().swap(multi_responses_);
    }
    FillHeader("cloud", last_routing_request_.get());
    auto *begin = last_routing_request_->mutable_waypoint(0);
    localization_reader_->Observe();
    auto msg = localization_reader_->GetLatestObserved();
    if (!msg) {
      AERROR << "No Localization Message Found!";
    }
    double nearest_s, nearest_l;
    century::hdmap::LaneInfoConstPtr line_id;
    if (GetNearestLaneWithHeading(
            msg->pose().position().x(), msg->pose().position().y(), &line_id,
            &nearest_s, &nearest_l, msg->pose().heading())) {
      begin->set_s(nearest_s);
    }

    begin->mutable_pose()->set_x(msg->pose().position().x());
    begin->mutable_pose()->set_y(msg->pose().position().y());
    begin->set_heading(msg->pose().heading());    

    AINFO << "select routing path id as : " << static_cast<int32_t>(path_id);
    last_routing_request_->set_select_routing_route(static_cast<int32_t>(path_id));
    last_routing_request_->set_multi_routing_type(routing::MultipleRoutingType::NOPROCESS);

    auto request = std::make_shared<century::planning::TemporaryParkingRequest>();
    common::util::FillHeader("cloud", request.get());
    request->set_need_stop(false);
    multi_temporary_parking_request_writer_->Write(request);

    routing_request_writer_->Write(last_routing_request_);
}



void Cloud::HandleDataTaskStatusCommand(const VehicleMessage& message,
                                        uint32_t sequence) {
  auto data = message.data_unit;
  printHex(data);
  century::planning::BlockingAreaResponse blocking_areas;

  bool event_type = (0x01 == data[11]);

  blocking_areas.set_need_stop(event_type);
  uint8_t area = data[23];
  auto blocking_area = blocking_areas.add_blocking_areas();
  printHex(data, "HandleDataTaskStatusCommand");
  if (0x31 == area) {
    blocking_area->set_blocking_area_response_type(
        century::planning::BLOCKING_AREA_J1);
  } else if (0x32 == area || 0x033 == area) {
    blocking_area->set_blocking_area_response_type(
        century::planning::BLOCKING_AREA_J2J3);
  } else if (0x34 == area) {
    blocking_area->set_blocking_area_response_type(
        century::planning::BLOCKING_AREA_J4);
  }

  blocking_area_writer_->Write(blocking_areas);
}

void Cloud::HandleBorrowCommand(const VehicleMessage& message,
                                uint32_t sequence) {
  auto data = message.data_unit;
  uint8_t status = data[11];
  century::planning::BorrowResponse borrow_response;
  FillHeader("cloud", &borrow_response);

  std::string idStr = borrow_id;

  if ("0" != idStr) {
    century::planning::PassStackerResponse pass_stacker_response;
    FillHeader("cloud", &pass_stacker_response);

    pass_stacker_response.set_stacker_id(idStr);
    pass_stacker_response.set_has_response(1);
    if (0x01 == status) {
      pass_stacker_response.set_pass_stacker_response_type(
          century::planning::PassStackerResponseType::PASS);
    } else {
      pass_stacker_response.set_pass_stacker_response_type(
          century::planning::PassStackerResponseType::NOPASS);
    }

    pass_stacker_writer_->Write(pass_stacker_response);
  }

  borrow_response.set_block_obs_id(idStr);
  if (0x01 == status) {
    borrow_response.set_response_type(century::planning::ResponseType::ACCEPT);
  } else {
    borrow_response.set_response_type(century::planning::ResponseType::REFUSE);
  }
  borrow_response.set_has_response(1);

  borrow_writer_->Write(borrow_response);
}

void Cloud::HandleRebootCommand(const VehicleMessage& message,
                                uint32_t sequence) {
  AINFO << "Handling REBOOT command with sequence: " << sequence;
  // const auto& data = message.data_unit;
  // Reboot logic here
}

void Cloud::HandleFineCommand(const VehicleMessage& message,
                              uint32_t sequence) {
  AINFO << "Handling FINE_TUNING command with sequence: " << sequence;

  const auto& data = message.data_unit;

  if (data.size() < 15) {
    AERROR << "HandleFineCommand: data size too small, size = " << data.size();
    return;
  }
  printHex(data, "Fine Command Data");

  size_t idx = 11;

  // status: 1 byte
  uint8_t status = data[idx++];

  // distance: 2 bytes
  uint16_t distance = 0;
  if (idx + sizeof(distance) > data.size()) {
    AERROR << "HandleFineCommand: not enough data for distance";
    return;
  }
  std::memcpy(&distance, &data[idx], sizeof(distance));
  distance = ntohs(distance);
  idx += sizeof(distance);

  double operator_type = 0.0;
  if (idx >= data.size()) {
    AERROR << "HandleFineCommand: not enough data for operator_type";
    return;
  }
  uint8_t operator_type_raw = data[idx++];

  double longitude = 0.0;
  double latitude = 0.0;
  uint16_t direction_angle = 0;
  std::string crane_id;
  if (0x02 == operator_type_raw) {
    operator_type = static_cast<double>(operator_type_raw);

    if (idx + 8 + 8 + 2 > data.size()) {
      AERROR << "HandleFineCommand: not enough data for "
                "longitude/latitude/direction";
      return;
    }

    // longitude: uint64_t
    uint64_t raw_longitude = 0;
    std::memcpy(&raw_longitude, &data[idx], sizeof(raw_longitude));
    raw_longitude = ntohll(raw_longitude);
    idx += sizeof(raw_longitude);
    longitude = raw_longitude / 1e8;

    // latitude: uint64_t
    uint64_t raw_latitude = 0;
    std::memcpy(&raw_latitude, &data[idx], sizeof(raw_latitude));
    raw_latitude = ntohll(raw_latitude);
    idx += sizeof(raw_latitude);
    latitude = raw_latitude / 1e8;

    // direction_angle: uint16_t
    std::memcpy(&direction_angle, &data[idx], sizeof(direction_angle));
    direction_angle = ntohs(direction_angle);
    idx += sizeof(direction_angle);
    uint8_t crane_id_len = data[idx++];
    if (idx + crane_id_len > data.size()) {
      AERROR << "Invalid crane_id length";
      return;
    }
    crane_id = std::string(reinterpret_cast<const char*>(&data[idx]),
                         crane_id_len);
    idx += crane_id_len;
    AINFO << "Crane ID: " << crane_id << std::endl;
  }

  printHex(data);

  auto task_type = (0x01 == status)
                       ? century::routing::TaskType::TINY_ADJUSTMENT_FRONT
                       : century::routing::TaskType::TINY_ADJUSTMENT_BACK;

  double distance_in_double = static_cast<double>(distance) / 100.0;

  bool result = SendRoutingRequest(task_type, distance_in_double, longitude,
                                   latitude, direction_angle, operator_type, crane_id);
  (void)result;
}

void Cloud::HandleMoveCommand(const VehicleMessage& message,
                              uint32_t sequence) {
  AINFO << "Handling MOVE command with sequence: " << sequence;
  const auto& data = message.data_unit;
  if (data.size() < 11) {
    return;
  }
  printHex(data, "HandleMoveCommand : ");
  uint8_t status = data[11];

  uint64_t raw_longitude;
  uint64_t raw_latitude;

  std::memcpy(&raw_longitude, &data[12], sizeof(raw_longitude));
  std::memcpy(&raw_latitude, &data[20], sizeof(raw_latitude));

  raw_longitude = ntohll(raw_longitude);
  raw_latitude = ntohll(raw_latitude);

  double longitude = raw_longitude / 1e8;
  double latitude = raw_latitude / 1e8;

  uint16_t direction_angle;
  std::memcpy(&direction_angle, &data[28], sizeof(uint16_t));
  direction_angle = ntohs(direction_angle);

  century::routing::TaskType task_type;
  if (MoveType::MOVEIN == status) {
    if (OperationType::LOADING == is_loading_) {
      task_type =
          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0;
    } else {
      task_type =
          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0;
    }
  } else if (MoveType::MOVEOUT == status) {
    if (OperationType::LOADING == is_loading_) {
      task_type =
          century::routing::TaskType::LOADING_OPERATIONAREA_SAMEDIRECTION_2;
    } else {
      task_type =
          century::routing::TaskType::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2;
    }
  }
  task_type_ = task_type;
  ConvertBoxStatus();
  bool result = SendRoutingRequest(longitude, latitude, direction_angle,
                                   is_loading_, true, task_type);
  result;
}

void Cloud::HandleStopCommand(const VehicleMessage& message,
                              uint32_t sequence) {
  AINFO << "Handling STOP command with sequence: " << sequence;
  AINFO << "Handling STOP command with sequence: " << sequence;

  auto request = std::make_shared<century::planning::TemporaryParkingRequest>();
  common::util::FillHeader("cloud", request.get());
  request->set_need_stop(false);
  multi_temporary_parking_request_writer_->Write(request);

  bool result = SendRoutingRequest(
      century::routing::TaskType::TINY_ADJUSTMENT_STOP, 1.0, 0.0, 0.0, 0.0, 0);
  result;
  // const auto& data = message.dataUnit;
  // Stop logic here
}

void Cloud::HandleImmediatelyArriveCommand(const VehicleMessage& message,
                                           uint32_t sequence) {
  AINFO << "Handling Immediately Arrive command with sequence: " << sequence;
  auto mcloud_info = std::make_shared<century::mcloud::McloudInfo>();
  mcloud_info->set_immediately_parking(true);

  FillHeader("cloud", mcloud_info.get());
  cloud_info_writer_->Write(mcloud_info);
}

bool Cloud::SendRoutingRequest(double longitude, double latitude,
                               double direction_angle) {
  double utm_x = 0.0;
  double utm_y = 0.0;
  char zone[20];

  century::routing::RoutingRequest routing_request;
  auto begin = routing_request.add_waypoint();
  localization_reader_->Observe();
  auto msg = localization_reader_->GetLatestObserved();
  if (!msg) {
    localization_reader_->Observe();
    msg = localization_reader_->GetLatestObserved();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!msg) {
      AERROR << "No Localization Message Found!";
      return false;
    }
  }

  double nearest_s, nearest_l;
  century::hdmap::LaneInfoConstPtr line_id;
  if (!GetNearestLaneWithHeading(
          msg->pose().position().x(), msg->pose().position().y(), &line_id,
          &nearest_s, &nearest_l, msg->pose().heading())) {
    return false;
  }

  // begin->set_id(line_id->id().id());
  begin->set_s(nearest_s);
  begin->mutable_pose()->set_x(msg->pose().position().x());
  begin->mutable_pose()->set_y(msg->pose().position().y());
  begin->set_heading(msg->pose().heading());

  auto end = routing_request.add_waypoint();
  Wgs84toUtm(longitude * DEG_TO_RAD, latitude * DEG_TO_RAD, &utm_x, &utm_y,
             nullptr, nullptr, zone);

  utm_x -= offset_x_;
  utm_y -= offset_y_;
  if (!GetNearestLane(utm_x, utm_y, &line_id, &nearest_s, &nearest_l)) {
    return false;
  }
  // end->set_id(line_id->id().id());
  end->set_s(nearest_s);
  end->mutable_pose()->set_x(utm_x);
  end->mutable_pose()->set_y(utm_y);
  common::util::FillHeader("cloud", &routing_request);
    auto request = std::make_shared<century::planning::TemporaryParkingRequest>();
    common::util::FillHeader("cloud", request.get());
    request->set_need_stop(false);
    multi_temporary_parking_request_writer_->Write(request);
last_routing_request_ =
    std::make_shared<century::routing::RoutingRequest>(routing_request);  
  routing_request_writer_->Write(routing_request);
  return true;
}

double beiyun_to_century(const double& beiyun_degree) {
  double century_degree = 90 - beiyun_degree;
  if (century_degree < -180) {
    century_degree += 360;
  }

  return century_degree;
}

bool Cloud::SendRoutingRequest(double longitude, double latitude,
                               double direction_angle, bool is_loading,
                               uint8_t hardware,
                               century::routing::TaskType task_type, const std::string& crane_id) {
  double utm_x = 0.0;
  double utm_y = 0.0;
  char zone[20];
  century::routing::RoutingRequest routing_request;
  auto begin = routing_request.add_waypoint();
  localization_reader_->Observe();
  auto msg = localization_reader_->GetLatestObserved();
  if (!msg) {
    AERROR << "No Localization Message Found!";
    return false;
  }

  double nearest_s, nearest_l;
  century::hdmap::LaneInfoConstPtr line_id;
  if (GetNearestLaneWithHeading(
          msg->pose().position().x(), msg->pose().position().y(), &line_id,
          &nearest_s, &nearest_l, msg->pose().heading())) {
    begin->set_s(nearest_s);
  }

  begin->mutable_pose()->set_x(msg->pose().position().x());
  begin->mutable_pose()->set_y(msg->pose().position().y());
  begin->set_heading(msg->pose().heading());

  auto end = routing_request.add_waypoint();
  Wgs84toUtm(longitude * DEG_TO_RAD, latitude * DEG_TO_RAD, &utm_x, &utm_y,
             nullptr, nullptr, zone);

  if (GetNearestLane(utm_x, utm_y, &line_id, &nearest_s, &nearest_l)) {
    end->set_s(nearest_s);
  }

  AINFO << "longitude : " << longitude << " latitude: " << latitude
        << " utm_x : " << utm_x << " utm_y : " << utm_y;
  utm_x -= offset_x_;
  utm_y -= offset_y_;
  end->mutable_pose()->set_x(utm_x);
  end->mutable_pose()->set_y(utm_y);

  double heading = beiyun_to_century(direction_angle) * DEG_TO_RAD;
  if (0x01 != hardware) {
    routing_request.set_coordinate_type(
        century::routing::CoordinateType::STACKER_COORDINATE);

    if (0x04 == hardware) {
      routing_request.set_coordinate_type(
          century::routing::CoordinateType::WHEEL_CRANE_COORDINATE);
    }

    end->set_heading(heading);
  }
  if (!crane_id.empty()) {
    routing_request.set_operation_stacker_id(crane_id);
  }
  routing_request.set_is_loading(is_loading);
  routing_request.set_task_type(task_type);
  common::util::FillHeader("cloud", &routing_request);

  auto request = std::make_shared<century::planning::TemporaryParkingRequest>();
  common::util::FillHeader("cloud", request.get());
  request->set_need_stop(false);
  multi_temporary_parking_request_writer_->Write(request);
  last_routing_request_ =
      std::make_shared<century::routing::RoutingRequest>(routing_request);
  routing_request_writer_->Write(routing_request);
  return true;
}

bool Cloud::SendRoutingRequest(century::routing::TaskType task_type,
                               double distance, double longitude,
                               double latitude, double heading,
                               uint8_t operator_type, const std::string& crane_id) {
  century::routing::RoutingRequest routing_request;
  auto begin = routing_request.add_waypoint();
  localization_reader_->Observe();
  auto msg = localization_reader_->GetLatestObserved();
  if (!msg) {
    AERROR << "No Localization Message Found!";
    return false;
  }

  double nearest_s, nearest_l;
  century::hdmap::LaneInfoConstPtr line_id;
  if (GetNearestLaneWithHeading(
          msg->pose().position().x(), msg->pose().position().y(), &line_id,
          &nearest_s, &nearest_l, msg->pose().heading())) {
    begin->set_s(nearest_s);
  }

  begin->mutable_pose()->set_x(msg->pose().position().x());
  begin->mutable_pose()->set_y(msg->pose().position().y());
  begin->set_heading(msg->pose().heading());

  auto end = routing_request.add_waypoint();
  if (0x02 != operator_type) {
    end->CopyFrom(*begin);
    routing_request.set_task_type(task_type);

    // routing_request.set_tiny_adjustment_type(century::routing::TinyAdjustmentType::DEFAULT_ADJUSTMENT);
  } else {
    double x, y;
    char zone[20];
    Wgs84toUtm(longitude * DEG_TO_RAD, latitude * DEG_TO_RAD, &x, &y, NULL,
               NULL, zone);
    x -= offset_x_;
    y -= offset_y_;
    double real_heading = beiyun_to_century(heading) * DEG_TO_RAD;

    end->mutable_pose()->set_x(x);
    end->mutable_pose()->set_y(y);
    end->set_heading(real_heading);
    routing_request.set_task_type(
        century::routing::TaskType::TINY_ADJUSTMENT_FRONT == task_type
            ? century::routing::TaskType::TINY_ADJUSTMENT_LEFT
            : century::routing::TaskType::TINY_ADJUSTMENT_RIGHT);

    routing_request.set_tiny_adjustment_type(
        century::routing::TinyAdjustmentType::STACKER_ADIUSTMENT);
  }
  if (century::routing::TaskType::TINY_ADJUSTMENT_STOP != task_type) {
    routing_request.set_tiny_adjustment_distance(distance);
  }
  if (!crane_id.empty()) {
    routing_request.set_operation_stacker_id(crane_id);
  }
  routing_request.set_is_loading(is_loading_);

  auto request = std::make_shared<century::planning::TemporaryParkingRequest>();
  common::util::FillHeader("cloud", request.get());
  common::util::FillHeader("cloud", &routing_request);

  request->set_need_stop(false);
  multi_temporary_parking_request_writer_->Write(request);

  last_routing_request_ =
    std::make_shared<century::routing::RoutingRequest>(routing_request);  
  routing_request_writer_->Write(routing_request);
  return true;
}

bool Cloud::GetNearestLaneWithHeading(
    const double x, const double y,
    century::hdmap::LaneInfoConstPtr* nearest_lane, double* nearest_s,
    double* nearest_l, const double heading) const {
  century::common::PointENU point;
  point.set_x(x);
  point.set_y(y);

  static constexpr double kSearchRadius = 3.0;
  static constexpr double kMaxHeadingDiff = 1.0;

  auto map_ptr = HDMapUtil::BaseMapPtr();
  if (!map_ptr || map_ptr->GetNearestLaneWithHeading(
                      point, kSearchRadius, heading, kMaxHeadingDiff,
                      nearest_lane, nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane with heading.";
    return false;
  }
  return true;
}

bool Cloud::GetNearestLane(const double x, const double y,
                           century::hdmap::LaneInfoConstPtr* nearest_lane,
                           double* nearest_s, double* nearest_l) const {
  century::common::PointENU point;
  point.set_x(x);
  point.set_y(y);

  auto map_ptr = HDMapUtil::BaseMapPtr();
  if (!map_ptr ||
      map_ptr->GetNearestLane(point, nearest_lane, nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane!";
    return false;
  }
  return true;
}

int Cloud::CalculateUTMZone(double longitude) {
  return static_cast<int>(std::floor((longitude + 180) / 6)) + 1;
}

void Cloud::TaskContronl(const VehicleMessage& message) {
  auto data = message.data_unit;

  uint32_t task_sequence = 0;
  std::memcpy(&task_sequence, &data[6], sizeof(task_sequence));
  task_sequence = ntohl(task_sequence);
  task_sequence;
  uint8_t task_contronl = data[11];
  task_contronl;

  std::vector<uint8_t> data_unit = GetTime();

  for (size_t i = 6; i < data.size(); i++) {
    data_unit.emplace_back(data[i]);
  }

  VehicleMessage send_data(0xC7, 0x01, message.unique_id,
                           message.encryption_method, data_unit);
  auto msg = send_data.pack_message();
  if (client_) {
    printHex(msg);
    client_->SendMessage(msg);
  }
}

std::vector<uint8_t> Cloud::GetTime() {
  std::vector<uint8_t> time;
  std::time_t current_time = std::time(nullptr);

  std::tm now;
  if (localtime_r(&current_time, &now) == nullptr) {
    AERROR << "Failed to get local time." << std::endl;
    return time;
  }

  time.emplace_back((now.tm_year + 1900) % 100);

  time.emplace_back(now.tm_mon + 1);
  time.emplace_back(now.tm_mday);
  time.emplace_back(now.tm_hour);
  time.emplace_back(now.tm_min);
  time.emplace_back(now.tm_sec);

  return time;
}

void Cloud::SendSystemInfo() {
  std::vector<uint8_t> data_unit = GetTime();
  auto planning_msg = GetLatestPlanningMessage();
  AppendSystemStatusSection(planning_msg, data_unit);
  AppendResourceStatusSection(data_unit);
  AppendFasAebStatusSection(data_unit);
  AppendTakeoverAndBorrowStatusSection(planning_msg, data_unit);
  AppendFixedSystemSections(data_unit);
  AppendRoadEventSection(data_unit);
  AppendLocalizationSection(data_unit);
  AppendMultiResponsesSection(data_unit);

  VehicleMessage reply(0x02, 0xfe, FLAGS_vehicle_unique_id, 0x01, data_unit);

  auto msg = reply.pack_message();
  if (client_) {
    printHex(data_unit, "System info : ");
    client_->SendMessage(msg);
  }
}

void Cloud::AppendSystemStatusSection(
    const std::shared_ptr<century::planning::ADCTrajectory>& planning_msg,
    std::vector<uint8_t>& data_unit) {
  data_unit.emplace_back(0x81);
  system_info_.system_status = 0x02;
  system_info_.autoDriving_status = 0x02;

  if (planning_msg && planning_msg->has_enable_auto_drive()) {
    system_info_.autoDriving_status =
        planning_msg->enable_auto_drive() ? 0x01 : 0x02;
  }
  system_info_.task_status = 0x01;

  data_unit.emplace_back(system_info_.system_status);
  data_unit.emplace_back(system_info_.autoDriving_status);
  data_unit.emplace_back(system_info_.task_status);

  system_info_.software_version[0] = 'v';
  system_info_.software_version[1] = '1';
  system_info_.software_version[2] = '.';
  system_info_.software_version[3] = '1';
  system_info_.software_version[4] = '.';
  system_info_.software_version[5] = '1';

  for (int i = 0; i < 10; ++i) {
    data_unit.emplace_back(system_info_.software_version[i]);
  }
}

void Cloud::AppendResourceStatusSection(std::vector<uint8_t>& data_unit) {
  const double cpu = 2.1;
  const double memory = 2.2;

  system_info_.cpu = cpu * 10;
  system_info_.memory = memory * 10;
  system_info_.temprate = 0xff;
  system_info_.tbox_status = box_status_;
  system_info_.map_status = 0x02;

  AppendDataInReverse(data_unit, system_info_.cpu);
  AppendDataInReverse(data_unit, system_info_.memory);
  AppendDataInReverse(data_unit, system_info_.temprate);
  AppendDataInReverse(data_unit, system_info_.tbox_status);
  AppendDataInReverse(data_unit, system_info_.map_status);
  AppendDataInReverse(data_unit, system_info_.box_status);
  AppendDataInReverse(data_unit, system_info_.box_status_back);
}

void Cloud::AppendFasAebStatusSection(std::vector<uint8_t>& data_unit) {
  system_info_.fas_aeb_status = 0x00;

  fas_aeb_info_reader_->Observe();
  auto fas_aeb_info_msg = fas_aeb_info_reader_->GetLatestObserved();
  if (fas_aeb_info_msg) {
    system_info_.fas_aeb_status =
        fas_aeb_info_msg->fas_aeb_switch() ? 0x01 : 0x00;
  }

  const uint8_t temporary_parking_status =
      (0x02 == temporary_parking_) ? 0x01 : 0x00;
  const uint8_t stop = immediately_stop_ ? 0x01 : 0x00;
  const uint8_t bck_music_status = background_music_enable_ ? 0x01 : 0x00;

  system_info_.fas_aeb_status |= (temporary_parking_status << 1);
  system_info_.fas_aeb_status |= (stop << 3);
  system_info_.fas_aeb_status |= (bck_music_status << 4);

  uint8_t aeb_triggered = 0x00;
  control_command_reader_->Observe();
  auto control_msg = control_command_reader_->GetLatestObserved();
  if (control_msg && control_msg->has_aeb_enable()) {
    aeb_triggered =
        common::util::IsFloatEqual(control_msg->aeb_enable(), 1.0) ? 0x01 : 0x00;
  }
  system_info_.fas_aeb_status |= (aeb_triggered << 5);
  data_unit.emplace_back(system_info_.fas_aeb_status);
}

std::shared_ptr<century::planning::ADCTrajectory>
Cloud::GetLatestPlanningMessage() {
  planning_reader_->Observe();
  return planning_reader_->GetLatestObserved();
}

void Cloud::AppendTakeoverAndBorrowStatusSection(
    const std::shared_ptr<century::planning::ADCTrajectory>& planning_msg,
    std::vector<uint8_t>& data_unit) {
  bool is_remote_status = false;
  if (planning_msg && planning_msg->has_remote_status()) {
    is_remote_status = planning_msg->remote_status().is_enabled();
  }

  chassis_reader_->Observe();
  auto chassis_msg = chassis_reader_->GetLatestObserved();

  system_info_.takeover_status = 0x01;
  if (chassis_msg && is_remote_status &&
      canbus::Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE ==
          chassis_msg->driving_mode()) {
    system_info_.takeover_status = 0x02;
  }
  data_unit.emplace_back(system_info_.takeover_status);

  system_info_.borrow_status = 0x00;
  if (planning_msg) {
    if (planning_msg->has_pass_stacker_request() &&
        planning_msg->pass_stacker_request().has_request_for_pass_stacker() &&
        planning_msg->pass_stacker_request().request_for_pass_stacker()) {
      auto msg = planning_msg->pass_stacker_request();
      if (msg.has_stacker_id()) {
        borrow_id = msg.stacker_id();
      }
      const bool borrow_flag = msg.request_for_pass_stacker();
      if (borrow_flag) {
        AINFO << "borrowFlag - pass_stacker : " << borrow_flag;
      }
      system_info_.borrow_status = (borrow_flag ? 1 : 0) << 0;
    } else if (planning_msg->has_borrow_request() &&
               planning_msg->borrow_request()) {
      if (planning_msg->has_block_obs_id()) {
        borrow_id = "0";
      }
      const bool borrow_flag = planning_msg->borrow_request();
      if (borrow_flag) {
        AINFO << "borrowFlag - block_obs : " << borrow_flag;
      }
      system_info_.borrow_status = (borrow_flag ? 1 : 0) << 0;
    }
  }
  AppendDataInReverse(data_unit, system_info_.borrow_status);
}

void Cloud::AppendFixedSystemSections(std::vector<uint8_t>& data_unit) {
  for (int i = 0; i < 8; ++i) {
    data_unit.emplace_back(0xFF);
  }

  data_unit.emplace_back(0x82);
  data_unit.emplace_back(0x01);
  data_unit.emplace_back(0x01);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x01);
}

void Cloud::AppendRoadEventSection(std::vector<uint8_t>& data_unit) {
  data_unit.emplace_back(0x83);
  const size_t count_pos = data_unit.size();
  data_unit.emplace_back(0x00);

  auto append_faults = [&data_unit, count_pos](const auto& reader) {
    if (!reader || data_unit[count_pos] == 0xFF) {
      return;
    }
    reader->Observe();
    const auto monitor_msg = reader->GetLatestObserved();
    if (!monitor_msg) {
      return;
    }

    for (const auto& fault : monitor_msg->fault_data()) {
      if (data_unit[count_pos] == 0xFF) {
        return;
      }
      RoadEvent event;
      AppendRoadEventIfBuilt(data_unit, count_pos,
                             BuildMonitorRoadEvent(fault, &event), event);
    }
  };

  append_faults(system_monitor_x86_reader_);
  append_faults(system_monitor_aarch_reader_);

  auto planning_msg = GetLatestPlanningMessage();
  if (!planning_msg || data_unit[count_pos] == 0xFF) {
    return;
  }

  if (planning_msg->has_header() && planning_msg->header().has_status()) {
    const auto& status = planning_msg->header().status();
    RoadEvent event;
    AppendRoadEventIfBuilt(data_unit, count_pos,
                           BuildPlanningRoadEvent(status, &event), event);
  }

  if (data_unit[count_pos] == 0xFF || !planning_msg->has_decision() ||
      !planning_msg->decision().has_main_decision()) {
    return;
  }

  const auto& main_decision = planning_msg->decision().main_decision();
  if (main_decision.has_stop()) {
    RoadEvent event;
    AppendRoadEventIfBuilt(
        data_unit, count_pos,
        BuildDecisionStopRoadEvent(main_decision.stop(), &event), event);
  }
}

void Cloud::AppendLocalizationSection(std::vector<uint8_t>& data_unit) {
  localization_reader_->Observe();
  auto localization_msg = localization_reader_->GetLatestObserved();
  if (!localization_msg) {
    return;
  }

  double lat = 0.0;
  double lon = 0.0;
  const double utm_x = offset_x_ + localization_msg->pose().position().x();
  const double utm_y = offset_y_ + localization_msg->pose().position().y();

  std::cout << std::fixed << std::setprecision(11);
  UtmtoWgs84(utm_x, utm_y, zone_id_.c_str(), &lon, &lat, nullptr, nullptr);

  lon *= RAD_TO_DEG;
  lat *= RAD_TO_DEG;
  data_unit.emplace_back(0x84);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x01);

  const uint64_t t_lat = lat * 1e8;
  const uint64_t t_lon = lon * 1e8;
  AppendDataInReverse(data_unit, t_lon);
  AppendDataInReverse(data_unit, t_lat);

  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x00);
  data_unit.emplace_back(0x00);
}

void Cloud::AppendMultiResponsesSection(std::vector<uint8_t>& data_unit) {
  if (multi_responses_.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(responses_mutex_);
  if (multi_responses_.empty()) {
    return;
  }

  data_unit.emplace_back(0x89);
  for (const auto& response_data : multi_responses_) {
    data_unit.emplace_back(response_data);
  }
}

void Cloud::Reply(const VehicleMessage& message, uint32_t id, uint8_t status) {
  std::vector<uint8_t> data_unit = GetTime();

  data_unit.emplace_back((id >> 24) & 0xFF);
  data_unit.emplace_back((id >> 16) & 0xFF);
  data_unit.emplace_back((id >> 8) & 0xFF);
  data_unit.emplace_back(id & 0xFF);

  data_unit.emplace_back(status);

  VehicleMessage data(0xc6, 0x01, message.unique_id, message.encryption_method,
                      data_unit);
  auto msg = data.pack_message();
  if (client_) {
    printHex(msg);
    client_->SendMessage(msg);
  }
}

void Cloud::OnRoutingResult(
    const std::shared_ptr<century::routing::RoutingResult>& result) {
  AINFO << "reciver routing result";
  VehicleMessage message(0x06, 0x01, FLAGS_vehicle_unique_id, 0x01,
                         std::vector<uint8_t>());
  if (result->routing_result()) {
    Reply(message, seq_, 0x02);
  } else {
    Reply(message, seq_, 0x06);
  }

  if (result->has_temporary_parking() && result->temporary_parking()) {
    auto request =
        std::make_shared<century::planning::TemporaryParkingRequest>();
    common::util::FillHeader("cloud", request.get());
    request->set_need_stop(true);
    request->set_need_clear_routing(true);
    multi_temporary_parking_request_writer_->Write(request);
    AINFO << "Sent temporary parking request with need_clear_routing=true";

    if (temporary_parking_release_timer_) {
      temporary_parking_release_timer_->Stop();
    }

    const double delay_s = FLAGS_temporary_parking_release_delay_s;
    if (delay_s <= 0.0) {
      auto release_request =
          std::make_shared<century::planning::TemporaryParkingRequest>();
      common::util::FillHeader("cloud", release_request.get());
      release_request->set_need_stop(false);
      multi_temporary_parking_request_writer_->Write(release_request);
      AINFO << "Sent temporary parking release request immediately";
      return;
    }

    const uint32_t delay_ms = static_cast<uint32_t>(delay_s * 1000.0 + 0.5);
    temporary_parking_release_timer_.reset(new cyber::Timer(
        delay_ms,
        [this]() {
          auto release_request =
              std::make_shared<century::planning::TemporaryParkingRequest>();
          common::util::FillHeader("cloud", release_request.get());
          release_request->set_need_stop(false);
          multi_temporary_parking_request_writer_->Write(release_request);
          AINFO << "Sent temporary parking release request";
        },
        true));
    temporary_parking_release_timer_->Start();
  }
}

void Cloud::OnPlanningRequest(
    const std::shared_ptr<century::planning::ADCTrajectory>& request) {
  if (!client_) return;

  std::string vehicle_id = FLAGS_vehicle_id;
  std::string vin_id = FLAGS_vehicle_unique_id;
  uint32_t vehicleType = 1;
   std::vector<uint32_t> action{0x22};
  std::string targetNo;
  uint32_t operate;
  std::string msgId;
  if (request->has_notice_stacker()) {
    const auto& pass_stacker_request = request->notice_stacker();
    if (pass_stacker_request.has_stacker_id())
      targetNo = pass_stacker_request.stacker_id();
    if (pass_stacker_request.has_request_for_pass_stacker())
      operate = pass_stacker_request.request_for_pass_stacker();
    if (pass_stacker_request.has_request_type()) {
      using RequestType = century::planning::PassStackerRequestType;
      const auto request_type = pass_stacker_request.request_type();

      static const std::unordered_map<RequestType, std::vector<uint32_t>> kTypeToAction = {
          {RequestType::PASS_DEFAULT, {0x22}},
          {RequestType::PASSED, {0x22}},
          {RequestType::PASS_READY, {0x22}},
          {RequestType::PASSING, {0x42}},
      };

      if (const auto it = kTypeToAction.find(request_type);
          kTypeToAction.end() != it) {
        action = it->second;
      }
    }
    if (operate && last_action_ != action) {
      {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_data.clear();
      }
      last_action_ = action;
      msgId = pass_stacker_request.message_id();
      for (const auto& action_current : action) {
        SendPassStackerInform(vehicle_id, vin_id, vehicleType, targetNo, action_current,
                            operate, msgId);
      }
    }
  }
}

void Cloud::SendPassStackerInform(const std::string& vehicle_id,
                                  const std::string& vin_id,
                                  uint32_t vehicle_type,
                                  const std::string& target_no,
                                  uint32_t action,
                                  uint32_t operate,
                                  const std::string& msg_id) {
  // last_action_ = action;
  long timestamp;
  timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();

  nlohmann::json informRequestData = {
      {"vehicleId", vehicle_id},    {"vin", vin_id},
      {"vehicleType", vehicle_type}, {"targetNo", target_no},
      {"action", action},           {"operate", operate},
      {"timestamp", timestamp},     {"msgId", msg_id}};

  std::string informRequest = informRequestData.dump();
  std::vector<uint8_t> data_unit(informRequest.begin(), informRequest.end());
  VehicleMessage message(0xD4, 0xFE, FLAGS_vehicle_unique_id, 0x01, data_unit);

  auto msg = message.pack_message();
  if (client_) {
    std::thread([this, msg]() {
      for (int i = 0; i < 3; ++i) {
        if (client_) {
          client_->SendMessage(msg);
          AINFO << "0xD4 has been sent" << (i + 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }).detach();
  }

  {
    std::lock_guard<std::mutex> lock(request_mutex);
    request_data[msg_id] = msg;
  }
}

void Cloud::SaveRoutingTask(const VehicleMessage& message,
                            const std::string& status) {
  nlohmann::json task_json;
  task_json["command_id"] = message.command_id;
  task_json["response_flag"] = message.response_flag;
  task_json["unique_id"] = message.unique_id;
  task_json["encryption_method"] = message.encryption_method;
  task_json["data_length"] = message.data_length;
  task_json["data_unit"] = nlohmann::json::array();
  for (uint8_t byte : message.data_unit) {
    task_json["data_unit"].emplace_back(byte);
  }
  task_json["checksum"] = message.checksum;
  task_json["status"] = status;
  std::ofstream file(kLastTaskFile, std::ios::trunc);
  if (file.is_open()) {
    file << task_json << std::endl;
    file.close();
  }
}

bool Cloud::ModifyLastRoutingTask(const std::string& status) {
  VehicleMessage message;
  return LoadRoutingTask(&message, status);
}

bool Cloud::LoadRoutingTask(VehicleMessage* message,
                            const std::string& status) {
  std::ifstream file(kLastTaskFile);
  if (!file.is_open()) {
    AERROR << "Unable to open file: " << kLastTaskFile;
    return false;
  }

  nlohmann::json task_json;
  file >> task_json;
  file.close();
  std::cout << task_json << std::endl;
  try {
    message->command_id = task_json["command_id"];
    message->response_flag = task_json["response_flag"];
    message->unique_id = task_json["unique_id"];
    message->encryption_method = task_json["encryption_method"];
    message->data_length = task_json["data_length"];
    message->checksum = task_json["checksum"];

    message->data_unit.clear();
    for (const auto& byte : task_json["data_unit"]) {
      message->data_unit.emplace_back(byte);
    }

    return true;
  } catch (const std::exception& e) {
    AERROR << "JSON parsing error: " << e.what();
    return false;
  }
}


void Cloud::HandleImmediatelyStopCommand(const VehicleMessage& message,
                              uint32_t sequence) {
  AINFO << "Handling Immediately Arrive command with sequence: " << sequence;
  auto stop_info = std::make_shared<century::mcloud::EmergencyStop>();
  // mcloud_info->set_immediately_parking(true);
  stop_info->set_emergency_stop(true);
  FillHeader("cloud", stop_info.get());
  emergency_stop_->Write(stop_info);
  // const auto& data = message.dataUnit;
  // Stop logic here
}


void Cloud::HandleImmediatelyStopCommand(bool stop) {
  // AINFO << "Handling Immediately Arrive command with sequence: " << sequence;
  auto stop_info = std::make_shared<century::mcloud::EmergencyStop>();
  // mcloud_info->set_immediately_parking(true);
  stop_info->set_emergency_stop(stop);
  FillHeader("cloud", stop_info.get());

  immediately_stop_ = stop;
  std::thread([this, stop_info](){
    for (int i = 0; i < 3; i++) {
      emergency_stop_->Write(stop_info);
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  }).detach();
}


}  // namespace mcloud
}  // namespace century
