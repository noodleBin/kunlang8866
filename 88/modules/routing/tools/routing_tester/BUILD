load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "ROUTING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    copts = ROUTING_COPTS,
    data = [
        ":test_data",
    ],
    deps = [
        "//modules/routing",
    ],
)

cc_binary(
    name = "routing_request",
    srcs = ["routing_request.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        "//modules/routing",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "*.pb.txt",
    ]),
)

cpplint()
