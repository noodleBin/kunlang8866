load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "ROUTING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "strategy",
    deps = [
        ":routing_a_star_strategy",
    ],
)

cc_library(
    name = "routing_a_star_strategy",
    srcs = ["a_star_strategy.cc"],
    hdrs = [
        "a_star_strategy.h",
        "strategy.h",
    ],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing/graph",
    ],
)

cpplint()
