load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "centuryadipclib",
    srcs = select(
        {
            "@platforms//cpu:x86_64": [
                "x86/libcenturyadipc.a",
            ],
            "@platforms//cpu:aarch64": [
                "aarch64/libcenturyadipc.a",
            ],
        },
        no_match_error = "Please Build with a Linux aarch64 or x86_64 platform",
    ),
)

cpplint()
