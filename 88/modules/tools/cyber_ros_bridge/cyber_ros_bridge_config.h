#ifndef MODULES_TOOLS_CYBER_ROS_BRIDGE_CYBER_ROS_BRIDGE_CONFIG_H_
#define MODULES_TOOLS_CYBER_ROS_BRIDGE_CYBER_ROS_BRIDGE_CONFIG_H_

#include <string>
#include <vector>

namespace century {
namespace tools {

struct LidarTopicRuntimeConfig {
  std::string topic;
  std::string child_frame_id;
};

struct LidarShmBridgeRuntimeConfig {
  std::string node_name;
  std::string tf_topic;
  std::string tf_static_topic;
  std::string lidar_pose_topic;
  std::string tf_frame_id;
  std::string tf_child_frame_id;
  std::string default_log_dir;
  std::string point_xyzirt_cloud_type;
  std::string point_cloud_packed_type;
  std::string transform_stampeds_type;
  std::string localization_estimate_type;
  std::string pose_with_cov_type;
  size_t verbose_startup_log_count = 0;
  size_t periodic_log_interval = 0;
  std::vector<LidarTopicRuntimeConfig> lidar_configs;
};

struct ShmReaderRuntimeConfig {
  std::string mode_tf;
  std::string mode_tf_static;
  std::string mode_cloud;
  std::string mode_compare;
  std::string mode_all;
  std::string mode_rate;
  std::string arg_topic;
  std::string arg_duration_sec;
  std::string arg_preview_count;
  size_t default_preview_point_count = 0;
  size_t default_preview_transform_count = 0;
  double default_duration_sec = 0.0;
  std::vector<std::string> lidar_topics;
};

struct CyberRosBridgeRuntimeConfig {
  LidarShmBridgeRuntimeConfig lidar_shm_bridge;
  ShmReaderRuntimeConfig shm_reader;
};

bool LoadCyberRosBridgeRuntimeConfig(CyberRosBridgeRuntimeConfig* config);

}  // namespace tools
}  // namespace century

#endif  // MODULES_TOOLS_CYBER_ROS_BRIDGE_CYBER_ROS_BRIDGE_CONFIG_H_
