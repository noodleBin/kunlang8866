load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_gflags",
    srcs = ["perception_gflags.cc"],
    hdrs = ["perception_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)
cc_library(
    name = "perception_common_util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = ["//cyber"],
)

cc_library(
    name = "perception_timer_util",
    srcs = ["timer_util.cc"],
    hdrs = ["timer_util.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber/common:log",
        "//cyber/common:macros",
        "//cyber/time",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "fps_counter",
    hdrs = ["fps_counter.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber/common:log",
        "//cyber/time",
    ],
)

cpplint()
