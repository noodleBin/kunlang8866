load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_decider_obstacle_utils",
    srcs = ["path_decider_obstacle_utils.cc"],
    hdrs = ["path_decider_obstacle_utils.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/common/historical_tracking_algorithms:hysteresis_interval",
    ],
)

cpplint()
