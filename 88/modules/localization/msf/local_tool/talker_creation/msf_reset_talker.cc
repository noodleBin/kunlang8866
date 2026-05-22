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

#include "modules/localization/proto/msf_reset.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using century::cyber::Rate;
using century::cyber::Time;

int main(int argc, char* argv[]) {
  // init cyber framework
  century::cyber::Init(argv[0]);
  // create talker node
  auto talker_node = century::cyber::CreateNode("msf_reset_talker");
  // create talker
  auto talker = talker_node->CreateWriter<::century::localization::MsfReset>(
      "/century/loc/msf_reset");
  Rate rate(1.0);
  uint64_t seq = 0;
  unsigned int index = 0;
  while (century::cyber::OK()) {
    auto msg = std::make_shared<::century::localization::MsfReset>();
    auto* header_msg = msg->mutable_header();
    header_msg->set_timestamp_sec(::century::cyber::Clock::NowInSeconds());
    header_msg->set_frame_id("map");
    static std::atomic<uint64_t> sequence_num = {0};
    header_msg->set_sequence_num(
        static_cast<unsigned int>(sequence_num.fetch_add(1)));

    msg->set_is_reset(++index > 30 ? true : false);
    talker->Write(msg);
    AINFO << "talker sent a message! No. " << seq;
    seq++;
    rate.Sleep();
  }
  return 0;
}
