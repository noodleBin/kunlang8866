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

#include "modules/routing/routing_component.h"

#include <string>
#include <unordered_set>
#include <utility>

#include "modules/map/proto/map_junction.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"

namespace century {
namespace routing {

using century::common::ErrorCode;
using century::common::PointENU;
using century::common::Status;
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
using century::hdmap::HDMapUtil;
using century::hdmap::Junction;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::Lane;
using century::hdmap::LaneBoundary;
using century::hdmap::LaneBoundaryType;
using century::hdmap::LaneInfoConstPtr;
using century::routing::LaneWaypoint;

namespace {
constexpr double kProcessRequestIntervalTime = 5.0;  // s
constexpr int kIsEmptyIntValue = 0;                  // int size
}  // namespace

bool RoutingComponent::Init() {
  ACHECK(cyber::ComponentBase::GetProtoConfig(&routing_config_))
      << "Unable to load routing conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";

  planning_reader_ = node_->CreateReader<planning::ADCTrajectory>(
      routing_config_.topic_config().planning_trajectory_topic(),
      [this](
          const std::shared_ptr<planning::ADCTrajectory> &planning_trajectory) {
        std::lock_guard<std::mutex> lock(mutex_);
        planning_trajectory_.CopyFrom(*planning_trajectory);
        has_reached_destination_ =
            planning_trajectory->has_reached_destination();
      });

  century::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(
      routing_config_.topic_config().routing_response_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);
  century::cyber::proto::RoleAttributes attrs;
  attrs.set_channel_name("/century/routing_responses");
  auto qoss = attrs.mutable_qos_profile();
  qoss->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qoss->set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qoss->set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  responses_writer_ = node_->CreateWriter<RoutingResponses>(attrs);
  century::cyber::proto::RoleAttributes attr_history;
  attr_history.set_channel_name(
      routing_config_.topic_config().routing_response_history_topic());
  auto qos_history = attr_history.mutable_qos_profile();
  qos_history->set_history(
      century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos_history->set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos_history->set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);

  routing_result_writer_ = node_->CreateWriter<RoutingResult>(
      routing_config_.topic_config().routing_result_topic());
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
  timer_.reset(new ::century::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_ != nullptr) {
            auto timestamp = century::cyber::Clock::NowInSeconds();
            response_->mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(*response_);
          }
        }
      },
      false));
  timer_->Start();

  request_timer_.reset(new ::century::cyber::Timer(
      FLAGS_routing_request_interval_ms,
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          ProcessNextRequest();
        }
      },
      false));

  hdmap_ = century::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << century::hdmap::BaseMapFile();

  return routing_.Init().ok() && routing_.Start().ok();
}

void RoutingComponent::ProcessNextRequest() {
  ADEBUG << "reached_destination: " << has_reached_destination_;
  ADEBUG << "need_process_now: " << need_process_now_;
  if (has_reached_destination_ && need_process_now_) {
    AINFO << "ProcessNextRequest now.";
    need_process_now_ = false;
    last_process_time_ = Clock::NowInSeconds();
    if (!routing_requests_.empty()) {
      RoutingRequest first_request = routing_requests_.front();
      auto timestamp = Clock::NowInSeconds();
      first_request.mutable_header()->set_timestamp_sec(timestamp);
      routing_requests_.erase(routing_requests_.begin());
      ADEBUG << "Try ProcessNextRequest.";
      auto response = std::make_shared<RoutingResponse>();
      std::vector<RoutingResponse> routing_responses;
      const std::shared_ptr<RoutingRequest> &request =
          std::make_shared<RoutingRequest>(first_request);
      ADEBUG << "request:\n" << request->DebugString();
      bool succeed =
          routing_.Process(request, response.get(), routing_responses);
      routing_result_.mutable_header()->set_timestamp_sec(timestamp);
      routing_result_.set_routing_result(succeed);
      routing_result_.set_from_cloud(true);
      routing_result_writer_->Write(routing_result_);
      if (succeed) {
        ADEBUG << "response:\n" << response->DebugString();
        common::util::FillHeader(node_->Name(), response.get());
        response_writer_->Write(response);
        {
          std::lock_guard<std::mutex> guard(mutex_);
          response_ = std::move(response);
        }
      } else {
        AERROR << "Try ProcessNextRequest failed.";
      }
    } else {
      AERROR << "ProcessNextRequest is empty, so failed.";
      request_timer_->Stop();
    }
  }

  if (!need_process_now_) {
    if (Clock::NowInSeconds() - last_process_time_ >
        kProcessRequestIntervalTime) {
      need_process_now_ = true;
      AERROR << "need_process_now_ is now true.";
    }
  }
}

bool RoutingComponent::CheckIsNeedToSearchMoreResponse(
    const std::shared_ptr<RoutingRequest> &request) {
  if (responses_.routing_response_size() > 0 &&
      request->multi_routing_type() == routing::NOPROCESS &&
      request->select_routing_route() < responses_.routing_response_size()) {
    AINFO << "select routing route:" << request->select_routing_route();
    auto iter_route = static_cast<int>(request->select_routing_route());
    auto obj_response = responses_.routing_response(iter_route);
    auto response_ptr = std::make_shared<RoutingResponse>(obj_response);
        common::util::FillHeader(node_->Name(), response_ptr.get());
    response_writer_->Write(response_ptr);
    return true;
  }
  responses_.clear_routing_response();
  return false;
}

bool RoutingComponent::Proc(const std::shared_ptr<RoutingRequest> &request) {
  AINFO << "=======> :" << request->DebugString();
  if (!FLAGS_enable_search_more_routing_flag) {
    request->set_multi_routing_type(routing::NOPROCESS);
  }
  auto timestamp = Clock::NowInSeconds();
  bool result = false;
  routing_result_.mutable_header()->set_timestamp_sec(timestamp);
  if (request->has_from_cloud() && request->from_cloud()) {
    result = ProcessCloudRequest(request);
    routing_result_.set_from_cloud(true);
  } else {
    result = ProcessLocalRequest(request);
    routing_result_.set_from_cloud(false);
  }
  routing_result_.set_routing_result(result);
  if (result &&
      request->multi_routing_type() == routing::PROCESSINGSTART) {
    routing_result_.set_temporary_parking(true);
  } else {
    routing_result_.set_temporary_parking(false);
  }
  routing_result_writer_->Write(routing_result_);
  return result;
}

bool RoutingComponent::ProcessRequest(const std::shared_ptr<RoutingRequest>& request) {
  if (CheckIsNeedToSearchMoreResponse(request)) {
    return true;
  }
  auto response = std::make_shared<RoutingResponse>();
  std::vector<RoutingResponse> routing_responses;
  if (!routing_.Process(request, response.get(), routing_responses)) {
    AERROR << "Failed, request is: \n"
           << request->DebugString();
    common::util::FillHeader(node_->Name(), response.get());
    response_writer_->Write(response);
    return false;
  }
  if (FLAGS_enable_extend_passage) {
    if (kIsEmptyIntValue == response.get()->road_size()) {
      AERROR << "response.get()->road_size() == 0 | ProcessLocalRequest "
                "failed, request is: \n"
             << request->DebugString();
      return false;
    } else {
      if (!ExtendPassage(response.get())) {
        AERROR << "ExtendPassage failed, request is: \n"
               << request->DebugString();
        return false;
      }
    }
  }
  common::util::FillHeader(node_->Name(), response.get());
  if (FLAGS_enable_search_more_routing_flag &&
      request->multi_routing_type() == routing::PROCESSINGSTART) {
    for (auto routing_response : routing_responses) {
      auto new_response = responses_.add_routing_response();
      new_response->CopyFrom(routing_response);
    }
    common::util::FillHeader(node_->Name(), &responses_);
    responses_writer_->Write(responses_);
  }
  if (request->multi_routing_type() != routing::PROCESSINGSTART) {
    response_writer_->Write(response);
  }

  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  return true;
}

bool RoutingComponent::ProcessLocalRequest(
    const std::shared_ptr<RoutingRequest> &request) {
  if (ProcessRequest(request)) {
    return true;
  } else {
    return false;
  }
}

bool RoutingComponent::ProcessCloudRequest(
    const std::shared_ptr<RoutingRequest> &request) {
  request_timer_->Stop();
  routing_requests_.clear();
  if (ProcessRequest(request)) {
    return true;
  } else {
    return false;
  }
  request_timer_->Start();
  return true;
}

bool RoutingComponent::ExtendPassage(routing::RoutingResponse *routing) {
  std::unordered_set<std::string> left_neighbor_lanes;
  std::pair<std::string, std::string> end_left_and_right_lane;
  std::string start_lane_id;
  std::string end_lane_id;

  if (routing == nullptr) {
    AERROR << "routing == nullptr";
    return false;
  }
  if (kIsEmptyIntValue == routing->road_size()) {
    AERROR << "routing->road_size()== 0";
    return false;
  }

  std::unordered_set<std::string> all_routing_lanes_id;
  for (int road_index = 0; road_index < routing->road_size(); ++road_index) {
    auto road_segment = routing->mutable_road(road_index);
    ADEBUG << "road_segment->passage_size()==" << road_segment->passage_size();
    for (int passage_index = 0; passage_index < road_segment->passage_size();
         ++passage_index) {
      auto passage = road_segment->passage(passage_index);
      ADEBUG << "passage.segment_size()==" << passage.segment_size();
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        all_routing_lanes_id.emplace(passage.segment(lane_index).id());
        if (0 == road_index && 0 == passage_index && 0 == lane_index) {
          start_lane_id = passage.segment(0).id();
          ADEBUG << "start_lane_id==" << start_lane_id;
        }
        if (road_index == routing->road_size() - 1 &&
            passage_index == road_segment->passage_size() - 1 &&
            lane_index == passage.segment_size() - 1) {
          end_lane_id = passage.segment(lane_index).id();
          ADEBUG << "end_lane_id==" << end_lane_id;
        }
      }
    }
  }

  if (!GetEndLane(end_lane_id, &end_left_and_right_lane)) {
    return false;
  }

  if (!ProcessLeftLanes(start_lane_id, end_lane_id, routing,
                        all_routing_lanes_id, end_left_and_right_lane)) {
    return false;
  }

  return true;
}

bool RoutingComponent::GetEndLane(
    const std::string &end_lane_id,
    std::pair<std::string, std::string> *end_left_and_right_lane) {
  auto end_lane = hdmap_->GetLaneById(hdmap::MakeMapId(end_lane_id));
  if (end_lane == nullptr) {
    AERROR << "end_lane == nullptr";
    return false;
  }

  auto end_lane_right_neighbor_ids =
      end_lane->lane().right_neighbor_forward_lane_id();
  if (end_lane_right_neighbor_ids.empty()) {
    ADEBUG << "end_lane_right_neighbor_ids is empty.";
  } else {
    const auto &end_lane_right_neighbor_id =
        end_lane_right_neighbor_ids.at(0).id();
    end_left_and_right_lane->second = end_lane_right_neighbor_id;
    ADEBUG << "end_lane_right_neighbor_id=" << end_lane_right_neighbor_id;
  }

  auto end_lane_left_neighbor_ids =
      end_lane->lane().left_neighbor_forward_lane_id();
  if (end_lane_left_neighbor_ids.empty()) {
    ADEBUG << "end_lane_left_neighbor_ids is empty.";
  } else {
    const auto &end_lane_left_neighbor_id =
        end_lane_left_neighbor_ids.at(0).id();
    end_left_and_right_lane->first = end_lane_left_neighbor_id;
    ADEBUG << "end_lane_left_neighbor_id=" << end_lane_left_neighbor_id;
  }

  return true;
}

century::routing::Passage RoutingComponent::SetLeftPassage(
    const std::string &left_lane_id,
    const std::unordered_set<std::string> &all_routing_lanes_id,
    bool neight_end_lane, bool *const neight_routing) {
  century::routing::Passage left_passage;
  auto left_lane = hdmap_->GetLaneById(hdmap::MakeMapId(left_lane_id));
  auto left_lane_successor = left_lane->lane().successor_id();
  if (left_lane->lane().successor_id().empty()) {
    left_passage.set_can_exit(false);
    left_passage.set_change_lane_type(routing::RIGHT);
  } else {
    for (auto iter = left_lane_successor.begin();
         iter != left_lane_successor.end(); ++iter) {
      auto left_lane_successor_right_neighbor =
          hdmap_->GetLaneById(hdmap::MakeMapId(iter->id()));
      auto left_lane_successor_right_neighbor_ids =
          left_lane_successor_right_neighbor->lane()
              .right_neighbor_forward_lane_id();
      if (left_lane_successor_right_neighbor_ids.empty()) {
        ADEBUG << "left_lane_successor_right_neighbor_ids is empty.";
        continue;
      }
      const auto &left_lane_successor_right_neighbor_id =
          left_lane_successor_right_neighbor_ids.at(0).id();

      for (auto iter_all = all_routing_lanes_id.begin();
           iter_all != all_routing_lanes_id.end(); ++iter_all) {
        if (iter_all->data() == left_lane_successor_right_neighbor_id) {
          ADEBUG << "left_lane_successor_right_neighbor_id="
                 << left_lane_successor_right_neighbor_id;

          auto left_lane_successor_right_neighbor = hdmap_->GetLaneById(
              hdmap::MakeMapId(left_lane_successor_right_neighbor_id));
          bool next_lane_is_virtual = left_lane_successor_right_neighbor->lane()
                                          .left_boundary()
                                          .virtual_();
          auto next_lane_left_boundary_type =
              left_lane_successor_right_neighbor->lane()
                  .left_boundary()
                  .boundary_type(0)
                  .types(0);
          if (FLAGS_enable_extend_passage_junction) {
            *neight_routing = true;
            break;
          } else if (!next_lane_is_virtual &&
                     LaneBoundaryType::DOTTED_WHITE ==
                         next_lane_left_boundary_type) {
            *neight_routing = true;
            break;
          }
        }
      }
      if (*neight_routing) {
        break;
      }
    }
    if (*neight_routing && !neight_end_lane) {
      left_passage.set_can_exit(true);
      left_passage.set_change_lane_type(routing::FORWARD);
    } else {
      left_passage.set_can_exit(false);
      left_passage.set_change_lane_type(routing::RIGHT);
    }
  }
  return left_passage;
}

century::routing::LaneSegment RoutingComponent::SetLaneSegment(
    const std::string &start_lane_id, const std::string &end_lane_id,
    const Passage &passage, const routing::RoutingResponse *routing,
    century::hdmap::LaneInfoConstPtr left_lane) {
  century::routing::LaneSegment left_lane_segment;
  left_lane_segment.set_id(left_lane->id().id());

  if (0 ==
      std::strcmp(start_lane_id.c_str(), passage.segment(0).id().c_str())) {
    const auto &start_pose =
        routing->routing_request().waypoint().begin()->pose();
    double s = 0.0;
    double l = 0.0;
    if (!left_lane->GetProjection({start_pose.x(), start_pose.y()}, &s, &l)) {
      left_lane_segment.set_start_s(
          left_lane->total_length() -
          (passage.segment(0).end_s() - passage.segment(0).start_s()));
    } else {
      left_lane_segment.set_start_s(s);
    }
    left_lane_segment.set_end_s(left_lane->total_length());
  } else if (0 == std::strcmp(end_lane_id.c_str(),
                              passage.segment(0).id().c_str())) {
    left_lane_segment.set_start_s(0);
    const auto &end_pose =
        routing->routing_request().waypoint().rbegin()->pose();
    double s = 0.0;
    double l = 0.0;
    if (!left_lane->GetProjection({end_pose.x(), end_pose.y()}, &s, &l)) {
      left_lane_segment.set_end_s(passage.segment(0).end_s());
    } else {
      left_lane_segment.set_end_s(s);
    }
  } else {
    left_lane_segment.set_start_s(0);
    left_lane_segment.set_end_s(left_lane->total_length());
  }

  return left_lane_segment;
}

void RoutingComponent::LogRoutingInfo(const routing::RoutingResponse *routing) {
  for (int i = 0; i < routing->road_size(); ++i) {
    for (int j = 0; j < routing->road(i).passage_size(); ++j) {
      for (int n = 0; n < routing->road(i).passage(j).segment_size(); ++n) {
        AINFO << "road[" << i << "]<" << routing->road(i).id() << ">passage["
              << j << "]<" << routing->road(i).passage(j).can_exit() << "><"
              << routing::ChangeLaneType_Name(
                     routing->road(i).passage(j).change_lane_type())
              << ">lane[" << n << "]<"
              << routing->road(i).passage(j).segment(n).id() << "><"
              << routing->road(i).passage(j).segment(n).start_s() << "><"
              << routing->road(i).passage(j).segment(n).end_s() << ">";
      }
    }
  }
}

bool RoutingComponent::ProcessLeftLanes(
    std::string start_lane_id, std::string end_lane_id,
    routing::RoutingResponse *routing,
    const std::unordered_set<std::string> &all_routing_lanes_id,
    const std::pair<std::string, std::string> &end_left_and_right_lane) {
  for (int road_index = 0; road_index < routing->road_size(); ++road_index) {
    auto road_segment = routing->mutable_road(road_index);
    int passage_size = road_segment->passage_size();
    for (int passage_index = 0; passage_index < passage_size; ++passage_index) {
      auto passage = road_segment->passage(passage_index);
      const auto &current_lane =
          hdmap_->GetLaneById(hdmap::MakeMapId(passage.segment(0).id()));
      auto current_lane_type = current_lane->lane().type();
      bool current_lane_is_virtual =
          current_lane->lane().left_boundary().virtual_();
      auto current_lane_left_boundary_type =
          current_lane->lane().left_boundary().boundary_type(0).types(0);
      if (hdmap::Lane::PLAY_STREET == current_lane_type) {
        break;
      }

      if (!FLAGS_enable_extend_passage_junction &&
          (current_lane_is_virtual ||
           LaneBoundaryType::DOTTED_WHITE != current_lane_left_boundary_type)) {
        break;
      }
      const auto &left_ids =
          current_lane->lane().left_neighbor_forward_lane_id();

      if (left_ids.empty()) {
        ADEBUG << "left_ids is empty.";
        break;
      }

      const auto &left_lane_id = left_ids.at(0).id();
      bool in_routing_lanes = false;
      for (auto iter = all_routing_lanes_id.begin();
           iter != all_routing_lanes_id.end(); ++iter) {
        if (iter->data() == left_lane_id) {
          in_routing_lanes = true;
          break;
        }
      }
      if (in_routing_lanes) {
        continue;
      }
      bool neight_end_lane = false;
      if (left_lane_id == end_left_and_right_lane.first) {
        neight_end_lane = true;
      }

      bool neight_routing = false;
      century::routing::Passage left_passage = SetLeftPassage(
          left_lane_id, all_routing_lanes_id, neight_end_lane, &neight_routing);
      auto left_lane = hdmap_->GetLaneById(hdmap::MakeMapId(left_lane_id));
      century::routing::LaneSegment left_lane_segment = SetLaneSegment(
          start_lane_id, end_lane_id, passage, routing, left_lane);
      left_passage.add_segment()->CopyFrom(left_lane_segment);

      road_segment->add_passage()->CopyFrom(left_passage);
    }
  }

  // Update original routing direction
  UpdateOriginalRoutingPassageDirection(routing);

  // Check route change lane length
  CheckRouteChangeLaneLength(routing);

  // print routing info
  LogRoutingInfo(routing);
  return true;
}

void RoutingComponent::UpdateOriginalRoutingPassageDirection(
    routing::RoutingResponse *routing) {
  if (nullptr == routing) {
    AERROR << "routing == nullptr";
    return;
  }

  if (kIsEmptyIntValue == routing->road_size()) {
    AERROR << "routing->road_size() == 0";
    return;
  }

  for (int road_index = 0; road_index < routing->road_size() - 1;
       ++road_index) {
    for (int passage_index = 0;
         passage_index < routing->road(road_index).passage_size();
         ++passage_index) {
      auto *current_passage =
          routing->mutable_road(road_index)->mutable_passage(passage_index);
      const auto &current_lane_id = current_passage->segment().rbegin()->id();
      const auto &current_lane =
          hdmap_->GetLaneById(hdmap::MakeMapId(current_lane_id));
      if (nullptr == current_lane) {
        continue;
      }
      auto current_lane_successor = current_lane->lane().successor_id();
      for (auto iter = current_lane_successor.begin();
           iter != current_lane_successor.end(); ++iter) {
        for (int next_road_passage_index = 0;
             next_road_passage_index <
             routing->road(road_index + 1).passage_size();
             ++next_road_passage_index) {
          if (iter->id() == routing->road(road_index + 1)
                                .passage(next_road_passage_index)
                                .segment()
                                .begin()
                                ->id() &&
              !current_passage->can_exit()) {
            // Due to the fact that there may be multiple transition points in
            // the actual situation, it is necessary to search for the entire
            // routing, so there is no need to break.
            current_passage->set_can_exit(true);
            current_passage->set_change_lane_type(routing::FORWARD);
            AINFO << "Current passage(lane id[" << current_lane_id
                  << "]) is not forward direction, need to update forward "
                     "direction.";
          }
        }
      }
    }
  }
}

void RoutingComponent::CheckRouteChangeLaneLength(
    routing::RoutingResponse *routing) {
  if (nullptr == routing) {
    AERROR << "routing == nullptr";
    return;
  }

  if (kIsEmptyIntValue == routing->road_size()) {
    AERROR << "routing->road_size() == 0";
    return;
  }

  for (int road_index = 0; road_index < routing->road_size() - 1;
       ++road_index) {
    for (int passage_index = 0;
         passage_index < routing->road(road_index).passage_size();
         ++passage_index) {
      const auto &passage = routing->road(road_index).passage(passage_index);
      double lane_change_length = CalculatePassageLength(passage);
      if (routing::RIGHT == passage.change_lane_type() && road_index > 0 &&
          lane_change_length <
              FLAGS_min_length_for_extend_passage_lane_change) {
        int update_road_index = road_index - 1;
        std::string first_passage_lane_id = passage.segment().begin()->id();
        while (lane_change_length <
                   FLAGS_min_length_for_extend_passage_lane_change &&
               update_road_index >= 0) {
          bool find_predecessor_passage = false;
          for (int p_index = 0;
               p_index < routing->road(update_road_index).passage_size();
               ++p_index) {
            auto *m_passage = routing->mutable_road(update_road_index)
                                  ->mutable_passage(p_index);
            if (IsPredecessorPassage(*m_passage, first_passage_lane_id)) {
              find_predecessor_passage = true;
              m_passage->set_can_exit(false);
              m_passage->set_change_lane_type(routing::RIGHT);
              lane_change_length += CalculatePassageLength(*m_passage);
              update_road_index -= 1;
              first_passage_lane_id = m_passage->segment().begin()->id();
              break;
            }
          }
          if (!find_predecessor_passage) {
            break;
          }
        }
      }
    }
  }
}

double RoutingComponent::CalculatePassageLength(
    const routing::Passage &passage) {
  double total_length = 0.0;
  for (int i = 0; i < passage.segment_size(); ++i) {
    total_length += passage.segment(i).end_s() - passage.segment(i).start_s();
  }
  return total_length;
}

bool RoutingComponent::IsPredecessorPassage(
    const routing::Passage &passage, const std::string &successor_lane_id) {
  const auto &successor_lane =
      hdmap_->GetLaneById(hdmap::MakeMapId(successor_lane_id));
  if (nullptr == successor_lane) {
    return false;
  }
  const auto &predecessor_lane = successor_lane->lane().predecessor_id();
  for (auto iter = predecessor_lane.begin(); iter != predecessor_lane.end();
       ++iter) {
    if (iter->id() == passage.segment().rbegin()->id()) {
      return true;
    }
  }
  return false;
}

}  // namespace routing
}  // namespace century
