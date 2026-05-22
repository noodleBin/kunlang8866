/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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

#include "absl/strings/match.h"
#include "gflags/gflags.h"

#include "modules/map/proto/map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap_util.h"

using century::common::PointENU;
using century::cyber::common::GetProtoFromFile;
using century::hdmap::Curve;
using century::hdmap::Map;
using century::hdmap::adapter::OpendriveAdapter;

#define MAXWIDTHDIFF 1.0

DEFINE_string(output_dir, "/tmp/", "output map checker directory");

double GetWidthFromSample(const google::protobuf::RepeatedPtrField<
                              century::hdmap::LaneSampleAssociation> &samples,
                          const double s) {
  if (samples.empty()) {
    return 0.0;
  }

  if (s <= samples[0].s()) {
    return samples[0].width();
  }
  if (s >= samples.rbegin()->s()) {
    return samples.rbegin()->width();
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].s() <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const century::hdmap::LaneSampleAssociation &sample1 = samples[low];
  const century::hdmap::LaneSampleAssociation &sample2 = samples[high];
  const double ratio = ((sample2.s() - sample1.s()) < 1e-9)
                           ? 0
                           : (sample2.s() - s) / (sample2.s() - sample1.s());
  return sample1.width() * ratio + sample2.width() * (1.0 - ratio);
}

int main(int32_t argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_v = 3;
  std::ofstream checker_txt_file("/century/data/log/base_map_check.txt");
  // checker_txt_file << map_pb.DebugString();
  Map map_pb;
  const auto map_file = century::hdmap::BaseMapFile();
  if (absl::EndsWith(map_file, ".xml")) {
    ACHECK(OpendriveAdapter::LoadData(map_file, &map_pb));
  } else {
    ACHECK(GetProtoFromFile(map_file, &map_pb)) << "Fail to open: " << map_file;
  }

  for (auto road_i : map_pb.road()) {
    for (auto section_i : road_i.section()) {
      std::vector<::century::hdmap::Lane> section_lane;
      section_lane.clear();
      // record the lane info in every road section
      for (auto lane_id : section_i.lane_id()) {
        for (auto lanei : map_pb.lane()) {
          if (lanei.id().id() == lane_id.id()) {
            section_lane.emplace_back(lanei);
          }
        }
      }
      // record the s sample s localization in the first lane
      std::vector<double> s_sample;
      s_sample.clear();

      for (auto sample : section_lane.at(0).left_sample()) {
        s_sample.emplace_back(sample.s());
      }

      int section_lane_size = section_lane.size();

      for (auto s_loc : s_sample) {
        bool is_need_to_break = false;
        double road_width_accumulator = 0;

        // checker_txt_file << "section_lane_size is " <<
        // section_lane_size<<"\n";
        for (int i = 0; i < section_lane_size; i++) {
          double lane_width =
              GetWidthFromSample(section_lane[i].right_sample(), s_loc) +
              GetWidthFromSample(section_lane[i].left_sample(), s_loc);
          road_width_accumulator += lane_width;
        }
        for (int i = 0; i < section_lane_size; i++) {
          double road_width_s =
              GetWidthFromSample(section_lane[i].right_road_sample(), s_loc) +
              GetWidthFromSample(section_lane[i].left_road_sample(), s_loc);
          if (abs(road_width_accumulator - road_width_s) > MAXWIDTHDIFF) {
            checker_txt_file
                << "road width is different with sum of lane width  " << "\n";
            checker_txt_file << "lane id is " << section_lane[i].id().id() << "\n";
            checker_txt_file << "section_lane_size is " << section_lane_size << "\n";
            checker_txt_file << "road width checked is " << road_width_accumulator << "\n";
            checker_txt_file << "road width from hdmap is " << road_width_s << "\n";
            checker_txt_file << "=============================================================" << "\n";
            is_need_to_break = true;
          }
        }
        if (is_need_to_break) {
          break;
        }
      }
    }
  }

  checker_txt_file.close();
  return 0;
}