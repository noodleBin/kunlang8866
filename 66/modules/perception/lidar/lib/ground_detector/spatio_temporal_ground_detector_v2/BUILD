load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spatio_temporal_ground_detector_v2",
    srcs = ["spatio_temporal_ground_detector_v2.cc"],
    hdrs = ["spatio_temporal_ground_detector_v2.h"],
    copts = select({
        "@platforms//cpu:x86_64": [
            "-msse4.1",
        ],
        "@platforms//cpu:aarch64": [],
        "//conditions:default": [],
    }) + PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lib/registerer",
        "@eigen",
        "//modules/perception/common/i_lib",
        "//modules/perception/common/i_lib/pc:i_ground",
        "//modules/perception/common/i_lib/pc:i_struct_s",
        "//modules/perception/common/i_lib/pc:i_util",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/i_lib/algorithm:i_sort",
        #"//modules/perception/common/algorithm:century_perception_common_algorithm",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector_v2/proto:spatio_temporal_ground_detector_config_cc_proto_v2",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/proto:perception_config_schema_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
