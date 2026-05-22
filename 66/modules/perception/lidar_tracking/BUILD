load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


PERCEPTION_COPTS = ['-DMODULE_NAME=\\"sample_lidar_tracking\\"']

filegroup(
    name = "lidar_tracking_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)
cc_library(
    name = "fast_lidar_tracking",
    srcs = [
        "fast_tracker/association/lidar_track_association.cc",
        "fast_tracker/config/lidar_track_config.cc",
        "fast_tracker/ekf/lidar_track_ekf_filter.cc",
        "fast_tracker/util/lidar_track_util.cc",
        "fast_tracker/fast_engine.cc",
    ],
    hdrs = [
        "fast_tracker/association/lidar_track_association.h",
        "fast_tracker/config/lidar_track_config.h",
        "fast_tracker/ekf/lidar_track_ekf_filter.h",
        "fast_tracker/util/lidar_track_util.h",
        "fast_tracker/fast_engine.h",
        "fast_tracker/lidar_track_type.h",
        "interface/base_multi_target_tracker.h",
    ], 
    deps = [   
        "//cyber",  
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:century_perception_common_algorithm",
        "//modules/perception/base:base",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/common:lidar_frame",
      #  "//modules/perception/onboard:century_perception_common_onboard",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//absl/strings",
    ]
)

cc_library(
    name = "century_perception_lidar_tracking",
    srcs = [
        "classifier/fused_classifier/ccrf_type_fusion.cc",
        "classifier/fused_classifier/fused_classifier.cc",
        "classifier/fused_classifier/fused_classifier_test.cc",
        "classifier/fused_classifier/util.cc",
        "tracker/association/distance_collection.cc",
        "tracker/association/gnn_bipartite_graph_matcher.cc",
        "tracker/association/multi_hm_bipartite_graph_matcher.cc",
        "tracker/common/mlf_track_data.cc",
        "tracker/common/track_data.cc",
        "tracker/common/track_pool_types.cc",
        "tracker/common/tracked_object.cc",
        "tracker/measurement/measurement_collection.cc",
        "tracker/multi_lidar_fusion/mlf_engine.cc",
        "tracker/multi_lidar_fusion/mlf_motion_filter.cc",
        "tracker/multi_lidar_fusion/mlf_motion_measurement.cc",
        "tracker/multi_lidar_fusion/mlf_motion_refiner.cc",
        "tracker/multi_lidar_fusion/mlf_shape_filter.cc",
        "tracker/multi_lidar_fusion/mlf_track_object_distance.cc",
        "tracker/multi_lidar_fusion/mlf_track_object_matcher.cc",
        "tracker/multi_lidar_fusion/mlf_tracker.cc",
        "tracker/multi_lidar_fusion/mlf_type_filter.cc",
        "tracker/multi_lidar_fusion/util.cc",
        "tracker/type_fusion/ccrf_type_fusion.cc",
        "tracker/type_fusion/util.cc",
    ],
    hdrs = [
        "classifier/fused_classifier/ccrf_type_fusion.h",
        "classifier/fused_classifier/fused_classifier.h",
        "classifier/fused_classifier/type_fusion_interface.h",
        "classifier/fused_classifier/util.h",
        "interface/base_bipartite_graph_matcher.h",
        "interface/base_classifier.h",
        "interface/base_multi_target_tracker.h",
        "tracker/association/distance_collection.h",
        "tracker/association/gnn_bipartite_graph_matcher.h",
        "tracker/association/multi_hm_bipartite_graph_matcher.h",
        "tracker/common/common.h",
        "tracker/common/convert.h",
        "tracker/common/mlf_track_data.h",
        "tracker/common/track_data.h",
        "tracker/common/track_pool_types.h",
        "tracker/common/tracked_object.h",
        "tracker/measurement/measurement_collection.h",
        "tracker/multi_lidar_fusion/mlf_base_filter.h",
        "tracker/multi_lidar_fusion/mlf_engine.h",
        "tracker/multi_lidar_fusion/mlf_motion_filter.h",
        "tracker/multi_lidar_fusion/mlf_motion_measurement.h",
        "tracker/multi_lidar_fusion/mlf_motion_refiner.h",
        "tracker/multi_lidar_fusion/mlf_shape_filter.h",
        "tracker/multi_lidar_fusion/mlf_track_object_distance.h",
        "tracker/multi_lidar_fusion/mlf_track_object_matcher.h",
        "tracker/multi_lidar_fusion/mlf_tracker.h",
        "tracker/multi_lidar_fusion/mlf_type_filter.h",
        "tracker/multi_lidar_fusion/util.h",
        "tracker/type_fusion/ccrf_type_fusion.h",
        "tracker/type_fusion/type_fusion_interface.h",
        "tracker/type_fusion/util.h"
    ],
    copts = PERCEPTION_COPTS + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:century_perception_common_algorithm",
        "//modules/perception/base:base",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/common:object_sequence",
        "//modules/perception/lidar/common:feature_descriptor",
        "//modules/perception/lidar/common:common", 
        "//modules/perception/onboard/msg_serializer:msg_serializer",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:ccrf_type_fusion_config_cc_proto",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:fused_classifier_config_cc_proto",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_component_config_cc_proto",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/lidar_tracking/tracker/type_fusion/proto:ccrf_type_filter_config_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/perception/lidar_tracking/io:lidar_tracking_io",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "liblidar_tracking_component",
    srcs = ["lidar_tracking_component.cc"],
    hdrs = ["lidar_tracking_component.h"],
    copts = PERCEPTION_COPTS + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/lidar_tracking:fast_lidar_tracking",
        ":century_perception_lidar_tracking",
        "//modules/perception/onboard/inner_component_messages:inner_component_messages",
        "//modules/perception/onboard/component:lidar_inner_component_messages",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_component_config_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
    ],
)

cc_binary(
    name = "sample_lidar_tracking",
    srcs = ["sample_lidar_tracking.cc",
            "sample_lidar_tracking.h",
            ],
    deps = [
        "//modules/perception/lidar_tracking:liblidar_tracking_component",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_node_config_cc_proto",
    ],
)


install(
    name = "install",
    data = [
        ":lidar_tracking_files"
    ],
    targets = [
        ":sample_lidar_tracking",
    ],
)

cc_library(
    name = "shell_component",
    srcs = ["shell_component.cc"],
    deps = ["//cyber"],
)

cpplint()
