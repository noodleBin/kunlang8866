load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "ROUTING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "routing_gflags",
    srcs = ["routing_gflags.cc"],
    hdrs = ["routing_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "routing_math",
    srcs = ["routing_math.cc"],
    hdrs = ["routing_math.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util:point_factory",
        "//modules/common/util:util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:routing_point_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
    ],
)

cpplint()
