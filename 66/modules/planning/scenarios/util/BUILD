load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/util:normal_util",
        "//modules/common/util:map_util",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:dependency_injector",
    ],
)

cpplint()
