load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timedelay",
    hdrs = ["timedelay.h"],
    deps = [
        ":timewrite",
    ],
)

cc_library(
    name = "timewrite",
    hdrs = [
        "timewrite.h"
    ],
    deps = [
        "//cyber/threadlib:threadlib",
        "//third_party/lwrb",
        "//third_party/safec"
    ],
    linkopts = [
        "-lstdc++fs",
        "-llwrb"
    ]
)

cpplint()
