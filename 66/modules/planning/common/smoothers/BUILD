load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "smoother",
    srcs = ["smoother.cc"],
    hdrs = ["smoother.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_cc_proto",
    ],
)

cc_library(
    name = "bspline",
    srcs = ["bspline.cc"],
    hdrs = ["bspline.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
