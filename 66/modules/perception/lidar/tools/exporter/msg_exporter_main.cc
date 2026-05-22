/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
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

#include <fstream>
#include <memory>

#include "absl/strings/str_split.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/perception/lidar/tools/exporter/msg_exporter.h"

namespace century {
namespace perception {
namespace lidar {

bool config_parser(const std::string& config_file,
                   std::vector<std::string>* channels,
                   std::vector<std::string>* child_frame_ids) {
  std::ifstream fin(config_file);
  if (!fin.is_open()) {
    return false;
  }
  channels->clear();
  child_frame_ids->clear();
  std::string line;
  std::getline(fin, line);
  while (!fin.eof()) {
    std::cout << line << std::endl;
    if (line[0] == '#') {
      std::getline(fin, line);
      continue;
    }
    const std::vector<std::string> splits = absl::StrSplit(line, ' ');
    if (splits.size() == 3 && std::stoi(splits[2]) > 0) {
      channels->push_back(splits[0]);
      child_frame_ids->push_back(splits[1]);
    }
    std::getline(fin, line);
  }
  fin.close();
  for (std::size_t i = 0; i < channels->size(); ++i) {
    std::cout << "\t\tRead channel " << channels->at(i) << " child_frame_id "
              << child_frame_ids->at(i) << std::endl;
  }
  return true;
}
}  // namespace lidar
}  // namespace perception
}  // namespace century

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "export_msgs config_file" << std::endl;
    return -1;
  }
  // century::cyber::Logger::Init(argv[0]);
  century::cyber::Init(argv[0]);  // cybertron init function
  std::shared_ptr<century::cyber::Node> node(
      century::cyber::CreateNode("export_node"));
  if (!node) {
    std::cout << "Failed to create export node." << std::endl;
    return -1;
  }
  std::vector<std::string> channels;
  std::vector<std::string> child_frame_ids;
  std::cout << "start to load config file: " << argv[1] << std::endl;
  if (!century::perception::lidar::config_parser(argv[1], &channels,
                                                &child_frame_ids)) {
    std::cout << "Failed to read config file" << std::endl;
    return -1;
  }
  century::perception::lidar::MsgExporter msg_exporter(node, channels,
                                                      child_frame_ids);
  while (century::cyber::OK()) {
    sleep(1);
  }
  return 0;
}
