load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "historical_tracking_config_cc_proto",
    deps = [
        ":historical_tracking_config_proto",
    ],
)

proto_library(
    name = "historical_tracking_config_proto",
    srcs = ["historical_tracking_config.proto"],
)

py_proto_library(
    name = "historical_tracking_config_py_pb2",
    deps = [
        ":historical_tracking_config_proto",
    ],
)
