load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teb_trajectory_partition",
    srcs = ["teb_trajectory_partition.cc"],
    hdrs = ["teb_trajectory_partition.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:normal_util",
        "//modules/planning/tasks/utils:openspace_util",
        "//modules/planning/open_space/openspace_common:openspace_common",
        "//modules/common/status",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_fsm_common",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_stabilization_for_teb_speed",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/smoothers:bspline",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/common:trajectory_stitcher",
    ],
)

cc_test(
    name = "teb_trajectory_partition_test",
    size = "small",
    srcs = ["teb_trajectory_partition_test.cc"],
    deps = [
        ":teb_trajectory_partition",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
