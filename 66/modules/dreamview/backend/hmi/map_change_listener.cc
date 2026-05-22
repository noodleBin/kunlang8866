/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

#include "cyber/cyber.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/dreamview/proto/map_change_cmd.pb.h"

DEFINE_string(map_change_cmd_topic, "/century/map_change_cmd",
              "Cyber channel for map change commands");

namespace century {
namespace dreamview {

class MapChangeListener {
 public:
  MapChangeListener() : node_(cyber::CreateNode("map_change_listener")) {}

  void Init() {
    reader_ = node_->CreateReader<MapChangeCmd>(
        FLAGS_map_change_cmd_topic,
        [this](const std::shared_ptr<MapChangeCmd>& cmd) {
          this->OnMapChangeCmd(cmd);
        });
  }

 private:
  void OnMapChangeCmd(const std::shared_ptr<MapChangeCmd>& cmd) {
    const std::string& new_map_dir = cmd->map_dir();

    if (new_map_dir.empty()) {
      return;
    }

    if (new_map_dir == FLAGS_map_dir) {
      return;
    }

    // Update the local FLAGS_map_dir.
    FLAGS_map_dir = new_map_dir;

    // Persist the new map_dir to global_flagfile.txt so that restarted
    // modules pick up the correct value (gflags uses the last occurrence).
    UpdateGlobalFlagfile(new_map_dir);

    // Stop modules.
    StopModules();
  }

  void UpdateGlobalFlagfile(const std::string& map_dir) {
    constexpr char kGlobalFlagfile[] =
        "/century/modules/common/data/global_flagfile.txt";
    std::ofstream fout(kGlobalFlagfile, std::ios_base::app);
    if (!fout) {
      return;
    }
    fout << "\n--map_dir=" << map_dir << std::endl;
  }

  void StopModules() {
    System("pkill -9 -f ins_loc.dag");
    System("pkill -9 -f static_transform.dag");
    System("pkill -9 -f dag_streaming_kl_perception_lidar.dag");
    System("pkill -9 -f sample_lidar_tracking");
  }

  static void System(const std::string& cmd) {
    const int ret = std::system(cmd.c_str());
    if (ret != 0) {
      AWARN << "Command returned non-zero: " << ret << ", cmd: " << cmd;
    }
  }

  std::shared_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<MapChangeCmd>> reader_;
};

}  // namespace dreamview
}  // namespace century

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  century::cyber::Init(argv[0]);

  century::dreamview::MapChangeListener listener;
  listener.Init();

  century::cyber::WaitForShutdown();
  return 0;
}
