load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"', "-DTZTEK_V4L2_DMA"]

cc_library(
    name = "tztek_headers_lib",
    hdrs = glob(
        ["**/*.h"],       
        exclude = [],
    ),
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
    ],
)

cpplint()