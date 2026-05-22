load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "creep_decider",
    srcs = ["creep_decider.cc"],
    hdrs = ["creep_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cc_test(
    name = "creep_decider_test",
    size = "small",
    srcs = ["creep_decider_test.cc"],
    deps = [
        "creep_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
