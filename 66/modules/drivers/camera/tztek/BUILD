load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"', "-DTZTEK_V4L2_DMA"]

config_setting(
    name  = "is_x86_64",
    values = {"cpu": "x86_64"},
)
config_setting(
    name  = "is_aarch64",
    values = {"cpu": "aarch64"},
)

cc_library(
    name = "tztek_camera_lib",
    srcs = select({
        ":is_aarch64": [
        "mgr_camera_jpegenc.cc",
        "mgr_camera.cc",
        "NvBufSurface.cc",
        "tztek_camera.cc",
        "v4l2cam.cc",
        ],
        "//conditions:default": [],
    }),
    hdrs = [],
    copts = CAMERA_COPTS,
    includes = ["./inc"],
    deps = select({
        ":is_aarch64":[
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/drivers/camera/tztek/inc:tztek_headers_lib",
        ],
        "//conditions:default": [],
    }),
    linkopts = select({
        ":is_aarch64": [
        "-lgeaccam",
        "-ljpegenc",
        "-lyuv",
        ],
        "//conditions:default": [],
    }),
    linkstatic = False,
)


cpplint()
