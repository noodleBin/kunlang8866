load("@rules_cc//cc:defs.bzl", "cc_library")

# DO NOT MODIFY
# THIS FILE IS GENERATED AUTOMATICALLY TO HELP WRITE BUILD FILE
# DELETE THIS FILE WHEN YOU HAVE CREATED THE COMPLETE BUILD FILE
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msg_buffer",
    srcs = ["msg_buffer.cc"],
    hdrs = ["msg_buffer.h"],
    deps = [
        "//cyber",
        "@boost",
    ],
)

cc_library(
    name = "time_window_synchronizer",
    srcs = ["time_window_synchronizer.cc"],
    hdrs = ["time_window_synchronizer.h"],
    deps = [
        "//cyber/common:log",
        "//modules/drivers/proto:sensor_image_cc_proto",
    ],
)

cpplint()
