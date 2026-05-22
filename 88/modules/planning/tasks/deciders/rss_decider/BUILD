load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rss_decider",
    srcs = ["rss_decider.cc"],
    hdrs = ["rss_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/tasks:task",
        "@ad_rss_lib//:ad_rss",
    ],
)

cc_test(
    name = "rss_decider_test",
    size = "small",
    srcs = ["rss_decider_test.cc"],
    deps = [
        ":rss_decider",
        "@ad_rss_lib//:ad_rss",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
