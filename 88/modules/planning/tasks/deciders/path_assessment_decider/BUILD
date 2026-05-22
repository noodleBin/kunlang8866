load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_assessment_decider",
    srcs = ["path_assessment_decider.cc"],
    hdrs = ["path_assessment_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/path_bounds_decider",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
)

cc_test(
    name = "path_assessment_decider_test",
    size = "small",
    srcs = ["path_assessment_decider_test.cc"],
    deps = [
        "path_assessment_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
