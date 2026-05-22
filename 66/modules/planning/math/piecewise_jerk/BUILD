load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_jerk_problem",
    srcs = ["piecewise_jerk_problem.cc"],
    hdrs = ["piecewise_jerk_problem.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/status",
        "//modules/planning/common:planning_gflags",
        "//third_party/safec:safec",
        "@osqp",
    ],
)

cc_library(
    name = "piecewise_jerk_path_problem",
    srcs = ["piecewise_jerk_path_problem.cc"],
    hdrs = ["piecewise_jerk_path_problem.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":piecewise_jerk_problem",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_problem",
    srcs = ["piecewise_jerk_speed_problem.cc"],
    hdrs = ["piecewise_jerk_speed_problem.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":piecewise_jerk_problem",
    ],
)

cpplint()
