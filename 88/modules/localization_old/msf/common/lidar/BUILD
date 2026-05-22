load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_utils",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    copts = [
        "-DMODULE_NAME=\\\"loc\\\"",
      ],
    deps = [
        "@eigen",
        "@local_config_pcl//:pcl",
        "//cyber",
        "//third_party/mmath",
        "//modules/localization/msf/common/io:common_io",
    ],
)

cpplint()
