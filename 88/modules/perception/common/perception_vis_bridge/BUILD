load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_vis_shm",
    hdrs = ["perception_vis_shm.h"],
)

cc_library(
    name = "vis_sink",
    srcs = ["vis_sink.cc"],
    hdrs = ["vis_sink.h"],
    deps = [
        ":perception_vis_shm",
        "//cyber/transport/shm:shm_queue",
    ],
)

cc_library(
    name = "vis_topic_registry",
    srcs = ["vis_topic_registry.cc"],
    hdrs = ["vis_topic_registry.h"],
)

cc_library(
    name = "raw_vis_bridge",
    srcs = ["raw_vis_bridge.cc"],
    hdrs = ["raw_vis_bridge.h"],
    deps = [
        ":perception_vis_shm",
        ":vis_sink",
        ":vis_topic_registry",
        "//cyber",
    ],
)

cc_binary(
    name = "perception_vis_bridge",
    srcs = ["perception_vis_bridge.cc"],
    deps = [
        ":raw_vis_bridge",
        ":vis_sink",
        ":vis_topic_registry",
        "//cyber",
    ],
)

cpplint()
