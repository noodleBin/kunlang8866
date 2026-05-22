#include "duplicated_object_filter.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "cyber/common/file.h"


using namespace century::perception::lidar;

int main(int argc, char** argv) {
  if(argc < 2) {
    AINFO << "Usage: " << argv[0] << " <config_file>" << std::endl;
    return -1;
  }
  std::string  config_file = argv[1];
  size_t last_slash_pos = config_file.rfind('/');
  if (last_slash_pos == std::string::npos) {
      return -1;
  }
  const std::string folder_path = config_file.substr(0, last_slash_pos);

  century::perception::PerceptionObstacles* perception_msg = new century::perception::PerceptionObstacles();
  if(!century::cyber::common::GetProtoFromASCIIFile(config_file , perception_msg)) {
    return -1;
  }

  DuplicatedObjectFilter duplicate_filter(2.0);
  std::ofstream type(folder_path +  "/type_filter.txt", std::ios::trunc);
  type << 0.0 << " , " << 0.0 << " , " << 0 << std::endl;

  std::ofstream unknown(folder_path + "/unknown.txt", std::ios::trunc);
  unknown << 0.0 << " , " << 0.0 << std::endl;
  for(int i=0; i<perception_msg->perception_obstacle_size(); i++) {
    if(perception_msg->perception_obstacle(i).type() !=  century::perception::PerceptionObstacle::UNKNOWN) {
      duplicate_filter.AddPoint(Eigen::Vector3d(perception_msg->perception_obstacle(i).anchor_point().x(), 
                                                perception_msg->perception_obstacle(i).anchor_point().y(),
                                                perception_msg->perception_obstacle(i).theta()),
                                perception_msg->perception_obstacle(i).length(),
                                perception_msg->perception_obstacle(i).width(),
                                perception_msg->perception_obstacle(i).type() == century::perception::PerceptionObstacle::PEDESTRIAN);
      type << perception_msg->perception_obstacle(i).anchor_point().x() << " , " 
           << perception_msg->perception_obstacle(i).anchor_point().y() << " , "
           << perception_msg->perception_obstacle(i).type() << std::endl;
    }
  }
  for(int i=0;i<perception_msg->perception_obstacle_size(); i++) {
    if(perception_msg->perception_obstacle(i).type() ==  century::perception::PerceptionObstacle::UNKNOWN) {
      unknown << perception_msg->perception_obstacle(i).anchor_point().x() <<" , " 
              << perception_msg->perception_obstacle(i).anchor_point().y() << std::endl;
      if(duplicate_filter.IsDuplicated(Eigen::Vector2d(perception_msg->perception_obstacle(i).anchor_point().x(), 
                                                    perception_msg->perception_obstacle(i).anchor_point().y()))) {
        AINFO << "duplicated: " << std::endl;
        AINFO << perception_msg->perception_obstacle(i).anchor_point().x() << " , " 
              << perception_msg->perception_obstacle(i).anchor_point().y() << " , "
              << perception_msg->perception_obstacle(i).length() << " , "
              << perception_msg->perception_obstacle(i).width()
              << std::endl;
      }
    }
  }
  type.close();
  unknown.close();
  return 0;

}