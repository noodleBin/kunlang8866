load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "risk_shift_decider",
    srcs = ["risk_shift_decider.cc"],
    hdrs = ["risk_shift_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_status_cc_proto",
        "//modules/planning/proto:task_config_cc_proto",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
)

cc_test(
    name = "risk_shift_decider_test",
    size = "small",
    srcs = ["risk_shift_decider_test.cc"],
    deps = [
        "risk_shift_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
