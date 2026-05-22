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

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/routing/routing.h"

namespace century {
namespace routing {

class RoutingComponent final
    : public ::century::cyber::Component<RoutingRequest> {
 public:
  RoutingComponent() = default;
  ~RoutingComponent() = default;

 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<RoutingRequest>& request) override;
  bool CheckIsNeedToSearchMoreResponse(
      const std::shared_ptr<RoutingRequest>& request);
  bool ProcessLocalRequest(const std::shared_ptr<RoutingRequest>& request);
  bool ProcessCloudRequest(const std::shared_ptr<RoutingRequest>& request);
  bool ProcessRequest(const std::shared_ptr<RoutingRequest>& request);
  bool GetEndLane(const std::string& end_lane_id,
                  std::pair<std::string, std::string>* end_left_and_right_lane);

  century::routing::Passage SetLeftPassage(
      const std::string& left_lane_id,
      const std::unordered_set<std::string>& all_routing_lanes_id,
      bool neight_end_lane, bool* const neight_routing);
  century::routing::LaneSegment SetLaneSegment(
      const std::string& start_lane_id, const std::string& end_lane_id,
      const Passage& passage, const routing::RoutingResponse* routing,
      century::hdmap::LaneInfoConstPtr left_lane);
  void LogRoutingInfo(const routing::RoutingResponse* routing);

  bool ProcessLeftLanes(
      std::string start_lane_id, std::string end_lane_id,
      routing::RoutingResponse* routing,
      const std::unordered_set<std::string>& all_routing_lanes_id,
      const std::pair<std::string, std::string>& end_left_and_right_lane);

  bool ExtendPassage(routing::RoutingResponse* routing);

  //   bool ExtendPassage(routing::RoutingResponse* routing);
  void UpdateOriginalRoutingPassageDirection(routing::RoutingResponse* routing);
  void CheckRouteChangeLaneLength(routing::RoutingResponse* routing);

 private:
  void ProcessNextRequest();
  double CalculatePassageLength(const routing::Passage& passage);
  bool IsPredecessorPassage(const routing::Passage& passage,
                            const std::string& successor_lane_id);

 private:
  std::shared_ptr<::century::cyber::Writer<RoutingResponses>>
      responses_writer_ = nullptr;
  std::shared_ptr<::century::cyber::Writer<RoutingResponse>> response_writer_ =
      nullptr;
  std::shared_ptr<::century::cyber::Writer<RoutingResponse>>
      response_history_writer_ = nullptr;
  std::shared_ptr<::century::cyber::Writer<RoutingResult>>
      routing_result_writer_ = nullptr;
  RoutingResult routing_result_;
  Routing routing_;
  std::shared_ptr<RoutingResponse> response_ = nullptr;
  RoutingResponses responses_;
  std::unique_ptr<::century::cyber::Timer> timer_;
  std::mutex mutex_;
  RoutingConfig routing_config_;

  const hdmap::HDMap* hdmap_ = nullptr;
  century::hdmap::Junction dead_end_junction_;
  std::vector<common::math::Vec2d> dead_end_vertices_;

  std::vector<RoutingRequest> routing_requests_;
  size_t seq_num_ = 0;

  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> planning_reader_;
  planning::ADCTrajectory planning_trajectory_;
  bool has_reached_destination_ = false;
  bool need_process_now_ = true;
  double last_process_time_;
  std::unique_ptr<::century::cyber::Timer> request_timer_;
};

CYBER_REGISTER_COMPONENT(RoutingComponent)

}  // namespace routing
}  // namespace century
