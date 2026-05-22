load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trace",
    hdrs = ["trace.h"],
    deps = [
        ":writer",
        ":common"
    ],
)

cc_library(
    name = "writer",
    hdrs = ["writer.h"],
    deps = [
        "@concurrent_queue//:concurrent_queue",
        "//cyber/threadlib:threadlib",
        ":common"
    ],
    linkopts = ["-lstdc++fs"]
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
    srcs = ["common.cc"],
    deps = [
        "//cyber/base",
        "//cyber/common:common",
        ":trace_enums"
    ]
)

cc_library(
    name = "trace_enums",
    hdrs = ["trace_enums.h"],
)

cpplint()
