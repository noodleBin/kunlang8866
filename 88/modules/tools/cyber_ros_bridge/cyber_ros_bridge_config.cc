#include "modules/tools/cyber_ros_bridge/cyber_ros_bridge_config.h"

#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "modules/tools/cyber_ros_bridge/cyber_ros_bridge_conf.pb.h"

namespace century {
namespace tools {
namespace {

constexpr char kRelativeConfigPath[] =
    "modules/tools/cyber_ros_bridge/conf/cyber_ros_bridge_conf.pb.txt";
constexpr char kAbsoluteConfigPath[] =
    "/century/modules/tools/cyber_ros_bridge/conf/"
    "cyber_ros_bridge_conf.pb.txt";

bool ValidateNonEmpty(const std::string& value, const char* field_name) {
  if (!value.empty()) {
    return true;
  }
  AERROR << "cyber_ros_bridge config field is empty: " << field_name;
  return false;
}

bool ValidatePositiveSize(const size_t value, const char* field_name) {
  if (value > 0) {
    return true;
  }
  AERROR << "cyber_ros_bridge config field must be > 0: " << field_name;
  return false;
}

bool ValidatePositiveDouble(const double value, const char* field_name) {
  if (value > 0.0) {
    return true;
  }
  AERROR << "cyber_ros_bridge config field must be > 0: " << field_name;
  return false;
}

bool LoadProtoConfig(CyberRosBridgeConfig* config) {
  const std::vector<std::string> candidate_paths = {
      kRelativeConfigPath,
      kAbsoluteConfigPath,
  };

  for (const auto& path : candidate_paths) {
    if (!cyber::common::PathExists(path)) {
      continue;
    }
    if (cyber::common::GetProtoFromFile(path, config)) {
      AINFO << "Loaded cyber_ros_bridge config from: " << path;
      return true;
    }
    AERROR << "Failed to parse cyber_ros_bridge config: " << path;
    return false;
  }

  AERROR << "cyber_ros_bridge config file not found. Tried: "
         << kRelativeConfigPath << ", " << kAbsoluteConfigPath;
  return false;
}

bool FillLidarBridgeConfig(const CyberRosBridgeConfig& proto_config,
                           LidarShmBridgeRuntimeConfig* config) {
  if (!proto_config.has_lidar_shm_bridge()) {
    AERROR << "cyber_ros_bridge config missing lidar_shm_bridge section";
    return false;
  }

  const auto& proto = proto_config.lidar_shm_bridge();
  config->node_name = proto.node_name();
  config->tf_topic = proto.tf_topic();
  config->tf_static_topic = proto.tf_static_topic();
  config->lidar_pose_topic = proto.lidar_pose_topic();
  config->tf_frame_id = proto.tf_frame_id();
  config->tf_child_frame_id = proto.tf_child_frame_id();
  config->default_log_dir = proto.default_log_dir();
  config->point_xyzirt_cloud_type = proto.point_xyzirt_cloud_type();
  config->point_cloud_packed_type = proto.point_cloud_packed_type();
  config->transform_stampeds_type = proto.transform_stampeds_type();
  config->localization_estimate_type = proto.localization_estimate_type();
  config->pose_with_cov_type = proto.pose_with_cov_type();
  config->verbose_startup_log_count = proto.verbose_startup_log_count();
  config->periodic_log_interval = proto.periodic_log_interval();

  if (!ValidateNonEmpty(config->node_name, "lidar_shm_bridge.node_name") ||
      !ValidateNonEmpty(config->tf_topic, "lidar_shm_bridge.tf_topic") ||
      !ValidateNonEmpty(config->tf_static_topic,
                        "lidar_shm_bridge.tf_static_topic") ||
      !ValidateNonEmpty(config->lidar_pose_topic,
                        "lidar_shm_bridge.lidar_pose_topic") ||
      !ValidateNonEmpty(config->tf_frame_id,
                        "lidar_shm_bridge.tf_frame_id") ||
      !ValidateNonEmpty(config->tf_child_frame_id,
                        "lidar_shm_bridge.tf_child_frame_id") ||
      !ValidateNonEmpty(config->default_log_dir,
                        "lidar_shm_bridge.default_log_dir") ||
      !ValidateNonEmpty(config->point_xyzirt_cloud_type,
                        "lidar_shm_bridge.point_xyzirt_cloud_type") ||
      !ValidateNonEmpty(config->point_cloud_packed_type,
                        "lidar_shm_bridge.point_cloud_packed_type") ||
      !ValidateNonEmpty(config->transform_stampeds_type,
                        "lidar_shm_bridge.transform_stampeds_type") ||
      !ValidateNonEmpty(config->localization_estimate_type,
                        "lidar_shm_bridge.localization_estimate_type") ||
      !ValidateNonEmpty(config->pose_with_cov_type,
                        "lidar_shm_bridge.pose_with_cov_type") ||
      !ValidatePositiveSize(config->verbose_startup_log_count,
                            "lidar_shm_bridge.verbose_startup_log_count") ||
      !ValidatePositiveSize(config->periodic_log_interval,
                            "lidar_shm_bridge.periodic_log_interval")) {
    return false;
  }

  config->lidar_configs.clear();
  config->lidar_configs.reserve(proto.lidar_topic_config_size());
  for (const auto& lidar_config : proto.lidar_topic_config()) {
    if (!ValidateNonEmpty(lidar_config.topic(),
                          "lidar_shm_bridge.lidar_topic_config.topic") ||
        !ValidateNonEmpty(
            lidar_config.child_frame_id(),
            "lidar_shm_bridge.lidar_topic_config.child_frame_id")) {
      return false;
    }
    config->lidar_configs.push_back(
        {lidar_config.topic(), lidar_config.child_frame_id()});
  }

  if (config->lidar_configs.empty()) {
    AERROR << "cyber_ros_bridge config requires at least one lidar topic";
    return false;
  }

  return true;
}

bool FillShmReaderConfig(const CyberRosBridgeConfig& proto_config,
                         const LidarShmBridgeRuntimeConfig& lidar_bridge_config,
                         ShmReaderRuntimeConfig* config) {
  if (!proto_config.has_shm_reader()) {
    AERROR << "cyber_ros_bridge config missing shm_reader section";
    return false;
  }

  const auto& proto = proto_config.shm_reader();
  config->mode_tf = proto.mode_tf();
  config->mode_tf_static = proto.mode_tf_static();
  config->mode_cloud = proto.mode_cloud();
  config->mode_compare = proto.mode_compare();
  config->mode_all = proto.mode_all();
  config->mode_rate = proto.mode_rate();
  config->arg_topic = proto.arg_topic();
  config->arg_duration_sec = proto.arg_duration_sec();
  config->arg_preview_count = proto.arg_preview_count();
  config->default_preview_point_count = proto.default_preview_point_count();
  config->default_preview_transform_count =
      proto.default_preview_transform_count();
  config->default_duration_sec = proto.default_duration_sec();

  if (!ValidateNonEmpty(config->mode_tf, "shm_reader.mode_tf") ||
      !ValidateNonEmpty(config->mode_tf_static,
                        "shm_reader.mode_tf_static") ||
      !ValidateNonEmpty(config->mode_cloud, "shm_reader.mode_cloud") ||
      !ValidateNonEmpty(config->mode_compare, "shm_reader.mode_compare") ||
      !ValidateNonEmpty(config->mode_all, "shm_reader.mode_all") ||
      !ValidateNonEmpty(config->mode_rate, "shm_reader.mode_rate") ||
      !ValidateNonEmpty(config->arg_topic, "shm_reader.arg_topic") ||
      !ValidateNonEmpty(config->arg_duration_sec,
                        "shm_reader.arg_duration_sec") ||
      !ValidateNonEmpty(config->arg_preview_count,
                        "shm_reader.arg_preview_count") ||
      !ValidatePositiveSize(config->default_preview_point_count,
                            "shm_reader.default_preview_point_count") ||
      !ValidatePositiveSize(config->default_preview_transform_count,
                            "shm_reader.default_preview_transform_count") ||
      !ValidatePositiveDouble(config->default_duration_sec,
                              "shm_reader.default_duration_sec")) {
    return false;
  }

  config->lidar_topics.clear();
  config->lidar_topics.reserve(lidar_bridge_config.lidar_configs.size());
  for (const auto& lidar_config : lidar_bridge_config.lidar_configs) {
    config->lidar_topics.emplace_back(lidar_config.topic);
  }

  return true;
}

}  // namespace

bool LoadCyberRosBridgeRuntimeConfig(CyberRosBridgeRuntimeConfig* config) {
  if (nullptr == config) {
    AERROR << "cyber_ros_bridge runtime config output is null";
    return false;
  }

  CyberRosBridgeConfig proto_config;
  if (!LoadProtoConfig(&proto_config)) {
    return false;
  }

  CyberRosBridgeRuntimeConfig runtime_config;
  if (!FillLidarBridgeConfig(proto_config, &runtime_config.lidar_shm_bridge) ||
      !FillShmReaderConfig(proto_config, runtime_config.lidar_shm_bridge,
                           &runtime_config.shm_reader)) {
    return false;
  }

  *config = runtime_config;
  return true;
}

}  // namespace tools
}  // namespace century
