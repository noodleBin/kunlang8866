load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_decider",
    srcs = ["speed_decider.cc"],
    hdrs = ["speed_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/utils:st_gap_estimator",
    ],
)

cc_test(
    name = "speed_decider_test",
    size = "small",
    srcs = ["speed_decider_test.cc"],
    deps = [
        "speed_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
