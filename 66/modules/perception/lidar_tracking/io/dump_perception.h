#pragma once

#include <limits>

#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "cyber/common/file.h"


namespace century {
namespace perception {
  
static void DumpRos2Result(const century::perception::PerceptionObstacles& item , std::ofstream* of, 
                          const double anchor_x, const double anchor_y) {
  if(!of) {
    return;
  }

  if(!of->is_open()) {
    return;
  }

  if(item.perception_obstacle_size() == 0) {
    return;
  }
  for(int i=0; i< item.perception_obstacle_size(); i++) {
    *of << std::fixed << item.header().timestamp_sec() << " "
        << item.perception_obstacle(i).id() << " "
        << item.perception_obstacle(i).anchor_point().x() - anchor_x << " "
        << item.perception_obstacle(i).anchor_point().y() - anchor_y<< " "
        << item.perception_obstacle(i).theta() << " "
        << static_cast<int>( item.perception_obstacle(i).type()) << " "
        << item.perception_obstacle(i).velocity().x()<< " "
        << item.perception_obstacle(i).velocity().y() << " "
        << std::endl;
  }
}
static void DumpPerceptionObstacles(const google::protobuf::Message& item , const std::string& tag, const double ts) {

  std::string folder_name = FLAGS_log_dir + "/" + std::to_string(ts);
  std::replace(folder_name.begin(), folder_name.end(), '.', '_');
  lidar::EnsureDirectory(folder_name);

  century::cyber::common::SetProtoToASCIIFile(item, folder_name + "/"+ tag + ".pb.txt");
}

static void DumpPreceptionLog(const std::stringstream& ss, const double& ts, const std::string& tag) {
  
  std::string folder_name = FLAGS_log_dir + "/" + std::to_string(ts);
  std::replace(folder_name.begin(), folder_name.end(), '.', '_');
  lidar::EnsureDirectory(folder_name);
  std::ofstream log_file(folder_name + "/" + tag + ".txt", std::ios::app);
  if(log_file.is_open()) {
    log_file << ss.str();
    log_file.close();
  }
}
}
}