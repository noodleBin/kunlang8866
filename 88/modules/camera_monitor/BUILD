load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_MONITOR_COPTS = [
    '-DMODULE_NAME=\\"camera_monitor\\"',
    '-std=c++17',
]

cc_library(
    name = "camera_monitor_component_lib",
    srcs = ["camera_monitor_component.cc"],
    hdrs = ["camera_monitor_component.h" ],
    copts = CAMERA_MONITOR_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@com_google_absl//absl/strings"
    ],
)

cc_binary(
    name = "libcamera_monitor_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":camera_monitor_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcamera_monitor_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

cpplint()
