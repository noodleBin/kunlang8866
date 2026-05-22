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
 * A map tool to load txt map
 */

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_map_dir = "/century/modules/map/data/gen";

  const auto map_filename = FLAGS_map_dir + "/base_map.txt";

  century::hdmap::HDMap hdmap;

  AINFO << "map file: " << map_filename;
  if (hdmap.LoadMapFromFile(map_filename) != 0) {
    AERROR << "Failed to load map: " << map_filename;
    ACHECK(false);
  } else {
    AERROR << "Succeed to load map: " << map_filename;
  }

  return 0;
}
