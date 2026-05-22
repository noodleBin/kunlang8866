load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threadlib_conf_data",
    srcs = ["threadlib_conf_data.cc"],
    hdrs = ["threadlib_conf_data.h"],
    deps = [
        "//cyber/common:common",
        "//cyber/proto:threadlib_conf_cc_proto"
    ]
)

cc_library(
    name = "threadlib",
    hdrs = ["threadlib.h"],
    deps = [
        "//cyber/scheduler:pin_thread",
        ":threadlib_conf_data",
    ]
)

cpplint()
