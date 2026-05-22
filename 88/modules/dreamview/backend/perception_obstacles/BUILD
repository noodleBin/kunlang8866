load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "obstacles",
    srcs = ["obstacles.cc"],
    hdrs = ["obstacles.h"],
    deps = [
      "//cyber",
      "//modules/audio/proto:audio_cc_proto",
      "//modules/audio/proto:audio_event_cc_proto",
      "//modules/canbus/proto:chassis_cc_proto",
      "//modules/common/adapters:adapter_gflags",
      "//modules/common/configs:vehicle_config_helper",
      "//modules/common/math:quaternion",
      "//modules/common/monitor_log",
      "//modules/common/proto:drive_event_cc_proto",
      "//modules/common/proto:geometry_cc_proto",
      "//modules/common/util",
      "//modules/common/util:map_util",
      "//modules/common/util:points_downsampler",
      "//modules/control/proto:control_cmd_cc_proto",
      "//modules/dreamview/backend/common:dreamview_gflags",
      "//modules/dreamview/backend/map:map_service",
      "//modules/dreamview/proto:simulation_world_cc_proto",
      "//modules/localization/proto:gps_cc_proto",
      "//modules/localization/proto:localization_cc_proto",
      "//modules/map/hdmap:hdmap_util",
      "//modules/perception/proto:perception_obstacle_cc_proto",
      "//modules/planning/proto:planning_cc_proto",
      "//modules/prediction/proto:prediction_obstacle_cc_proto",
      "//modules/storytelling/proto:story_cc_proto",
      "//modules/task_manager/proto:task_manager_cc_proto",
      "//modules/dreamview/backend/sim_control",
      "@com_github_nlohmann_json//:json",
      "@com_google_absl//absl/strings",
    ],
)