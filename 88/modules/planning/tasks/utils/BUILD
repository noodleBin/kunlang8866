load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_gap_estimator",
    srcs = ["st_gap_estimator.cc"],
    hdrs = ["st_gap_estimator.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
)

cc_library(
    name = "openspace_util",
    srcs = ["openspace_util.cc"],
    hdrs = ["openspace_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math/smoothing_spline:spline_1d_constraint",
        "//modules/planning/math/smoothing_spline:spline_1d",
        "//modules/planning/math/smoothing_spline:spline_2d",
        "//modules/planning/math/smoothing_spline:spline_2d_seg",
        "//modules/planning/math/smoothing_spline:quintic_spline_2d",
    ],
)

cpplint()
