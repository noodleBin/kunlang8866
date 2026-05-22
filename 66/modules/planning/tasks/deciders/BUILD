load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider_base",
    srcs = ["decider.cc"],
    hdrs = ["decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
