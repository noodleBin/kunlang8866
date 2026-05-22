load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "ROUTING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_dump",
    srcs = ["routing_dump.cc"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/planning/proto:planning_cc_proto",
        "//modules/routing",
    ],
)

cc_binary(
    name = "routing_cast",
    srcs = ["routing_cast.cc"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing",
    ],
)

cpplint()
