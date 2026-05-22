load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = ['-DMODULE_NAME=\\"mcloud\\"', "-Werror=reorder"]

cc_library(
    name = "mqtt_client",
    srcs = ["mqtt_client.cc"],
    hdrs = ["mqtt_client.h"],
    copts = MCLOUD_COPTS,
    linkopts = ["-lmosquitto"],
    deps = [
        "//cyber",
        "@com_github_nlohmann_json//:json",
    ],
)

cpplint()

