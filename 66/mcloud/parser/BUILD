load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = ['-DMODULE_NAME=\\"mcloud\\"']

cc_library(
    name = "encryption_strategy",
    srcs = ["encryption_strategy.cc"],
    hdrs = ["encryption_strategy.h"],
    copts = MCLOUD_COPTS,
)


cc_library(
    name = "vehicle_message",
    srcs = ["vehicle_message.cc"],
    hdrs = ["vehicle_message.h"],
    deps = [
      ":encryption_strategy",
    ],
    copts = MCLOUD_COPTS,
)

cpplint()