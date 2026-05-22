load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kinematic_speed_base_optimizer",
    hdrs = ["kinematic_speed_base_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util",
    ],
)

cc_library(
    name = "constant_jerk_speed_optimizer",
    hdrs = ["constant_jerk_speed_optimizer.h"],
    srcs = ["constant_jerk_speed_optimizer.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":kinematic_speed_base_optimizer",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common:speed_profile_generator",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "kinematic_deceleration_speed_optimizer",
    srcs = ["kinematic_deceleration_speed_optimizer.cc"],
    hdrs = ["kinematic_deceleration_speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "two_piecewise_jerk_speed_optimizer",
    srcs = ["two_piecewise_jerk_speed_optimizer.cc"],
    hdrs = ["two_piecewise_jerk_speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":kinematic_speed_base_optimizer",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "two_piecewise_stationary_jerk_speed_optimizer",
    srcs = ["two_piecewise_stationary_jerk_speed_optimizer.cc"],
    hdrs = ["two_piecewise_stationary_jerk_speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":kinematic_speed_base_optimizer",
        "//cyber/time:clock",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
