load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64")

package(default_visibility = ["//visibility:public"])

config_setting(
    name  = "is_x86_64",
    values = {"cpu": "x86_64"},
)
config_setting(
    name  = "is_aarch64",
    values = {"cpu": "aarch64"},
)

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

cc_binary(
    name = "libcamera_component.so",
    linkopts = [
        "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
    deps       = select({
        ":is_aarch64": [":camera_component_lib"],
        "//conditions:default": [],
    }),
)

filegroup(
    name = "runtime_data",
    srcs = select({
        ":is_aarch64": glob([
            "conf/*.txt",
            "conf/*.json",
            "dag/*.dag",
            "launch/*.launch",
        ]),
        "//conditions:default": [],
    }),
)

install(
    name = "install",
    data = [
      ":runtime_data",
    ],
    targets = [
      ":libcamera_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

cc_library(
    name = "camera_component_lib",
    srcs = select({
        ":is_aarch64": ["camera_component.cc"],
        "//conditions:default": [],
    }),
    hdrs = select({
        ":is_aarch64": ["camera_component.h"],
        "//conditions:default": [],
    }),
    copts = CAMERA_COPTS,
    deps  = select({
        ":is_aarch64": [
        "//cyber",
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/drivers/camera/tztek:tztek_camera_lib",
        ],
        "//conditions:default": [],
    }),
)

cpplint()
