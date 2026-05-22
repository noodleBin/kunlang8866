load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "segmentation_ground_detector_config_proto",
    srcs = ["segmentation_ground_detector_config.proto"],
)

cc_proto_library(
    name = "segmentation_ground_detector_config_cc_proto",
    deps = [":segmentation_ground_detector_config_proto"],
)
