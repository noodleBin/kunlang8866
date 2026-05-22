/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

#include "modules/routing/proto/routing.pb.h"

#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "modules/common/adapters/adapter_gflags.h"

DEFINE_bool(enable_remove_lane_id, false,
            "True to remove lane id in routing request");

DEFINE_string(routing_test_file,
              "modules/routing/tools/routing_tester/routing_request.pb.txt",
              "Used for sending routing request to routing node.");

using century::cyber::Rate;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  // init cyber framework
  // century::cyber::Init(argv[0]);
  century::cyber::Init("routing request node");
  FLAGS_alsologtostderr = true;

  century::routing::RoutingRequest routing_request;
  if (!century::cyber::common::GetProtoFromFile(FLAGS_routing_test_file,
                                              &routing_request)) {
    AERROR << "failed to load file: " << FLAGS_routing_test_file;
    return -1;
  }

  if (FLAGS_enable_remove_lane_id) {
    for (int i = 0; i < routing_request.waypoint_size(); ++i) {
      routing_request.mutable_waypoint(i)->clear_id();
      routing_request.mutable_waypoint(i)->clear_s();
    }
  }

  std::shared_ptr<century::cyber::Node> node(
      century::cyber::CreateNode("routing_request"));
  auto writer = node->CreateWriter<century::routing::RoutingRequest>(
      FLAGS_routing_request_topic);

  Rate rate(1.0);
  int send_num = 1;
  int i = 0;
  AINFO << "------Start send routing request------";
  while (century::cyber::OK() && i < send_num) {
    rate.Sleep();
    double timestamp = ::century::cyber::Clock::NowInSeconds();
    routing_request.mutable_header()->set_timestamp_sec(timestamp);
    writer->Write(routing_request);
    AINFO << "-- " << i
          << " --Send out routing request: " << routing_request.DebugString();
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }
  century::cyber::AsyncShutdown();
  century::cyber::WaitForShutdown();
  AINFO << "------Finished send routing request------";

  return 0;
}
