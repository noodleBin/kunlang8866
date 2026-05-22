load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = ['-DMODULE_NAME=\\"mcloud\\"']

cc_library(
    name = "cloud_gflags",
    srcs = ["cloud_gflags.cc"],
    hdrs = ["cloud_gflags.h"],
    copts = MCLOUD_COPTS,
    deps = [
      "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()