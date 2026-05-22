load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_bounds_decider",
    srcs = ["speed_bounds_decider.cc"],
    hdrs = ["speed_bounds_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_boundary_mapper",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/planning/common/historical_tracking_algorithms:hysteresis_interval",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_history_value",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_history_diff_value",
        "//modules/planning/common/trajectory1d:constant_acc_to_target_v_trajectory1d",
        "//modules/planning/proto:svm_feature_data_cc_proto",
        "@opencv//:core",
        "@opencv//:ml",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks/utils:st_gap_estimator",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
        "//modules/planning/common/historical_tracking_algorithms:hysteresis_interval",
    ],
)

cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = ["st_boundary_mapper_test.cc"],
    data = [
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//cyber/common:log",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)


cc_test(
    name = "speed_bounds_decider_test",
    size = "small",
    srcs = ["speed_bounds_decider_test.cc"],
    deps = [
        "speed_bounds_decider",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)





cpplint()
