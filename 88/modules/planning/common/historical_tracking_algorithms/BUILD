load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hysteresis_interval",
    hdrs = ["hysteresis_interval.h"],
    srcs = ["hysteresis_interval.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:obstacle",
    ],
)

cc_test(
    name = "hysteresis_interval_test",
    size = "small",
    srcs = ["hysteresis_interval_test.cc"],
    data = [
        "//modules/planning/common:common_testdata",
    ],
    deps = [
        ":hysteresis_interval",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "obstacle_stabilization_for_teb_speed",
    hdrs = ["obstacle_stabilization_for_teb_speed.h"],
    srcs = ["obstacle_stabilization_for_teb_speed.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
    ],
)

cc_test(
    name = "obstacle_stabilization_for_teb_speed_test",
    size = "small",
    srcs = ["obstacle_stabilization_for_teb_speed_test.cc"],
    deps = [
        ":obstacle_stabilization_for_teb_speed",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "obstacle_history_value",
    hdrs = ["obstacle_history_value.h"],
    srcs = ["obstacle_history_value.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "obstacle_history_diff_value",
    hdrs = ["obstacle_history_diff_value.h"],
    srcs = ["obstacle_history_diff_value.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "obstacle_speed_distance_history",
    hdrs = ["obstacle_speed_distance_history.h"],
    srcs = ["obstacle_speed_distance_history.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "start_up_vehicle_position_history",
    hdrs = ["start_up_vehicle_position_history.h"],
    srcs = ["start_up_vehicle_position_history.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:file",
        "//modules/planning/common:obstacle",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/proto/common_config:historical_tracking_config_cc_proto",
        "//modules/planning/proto:sl_boundary_cc_proto",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
    ],
)

cc_library(
    name = "obstacle_heading_history",
    hdrs = ["obstacle_heading_history.h"],
    srcs = ["obstacle_heading_history.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:file",
        "//modules/planning/common:obstacle",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/proto/common_config:historical_tracking_config_cc_proto",
        "//modules/planning/proto:sl_boundary_cc_proto",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
    ],
)

cpplint()
