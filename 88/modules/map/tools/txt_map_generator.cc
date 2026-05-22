/* Copyright 2022 The Century Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "gflags/gflags.h"

#include "modules/map/proto/map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"

/**
 * A map tool to transform .bin map to .txt map
 */

DEFINE_string(output_dir, "/century/modules/map/data/bin_to_txt",
              "output map directory");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_map_dir = "/century/modules/map/data/bin_to_txt";

  const auto map_filename = FLAGS_map_dir + "/base_map.bin";
  century::hdmap::Map pb_map;
  if (!century::cyber::common::GetProtoFromFile(map_filename, &pb_map)) {
    AERROR << "Failed to load bin map from " << map_filename;
    return -1;
  } else {
    AINFO << "Loaded bin map from " << map_filename;
  }

  const std::string output_txt_file = FLAGS_output_dir + "/base_map.txt";
  if (!century::cyber::common::SetProtoToASCIIFile(pb_map, output_txt_file)) {
    AERROR << "Failed to generate txt base map";
    return -1;
  }

  pb_map.Clear();
  ACHECK(century::cyber::common::GetProtoFromFile(output_txt_file, &pb_map))
      << "Failed to load generated txt base map";

  AINFO << "Successfully converted .bin map to .txt map: " << output_txt_file;

  return 0;
}
