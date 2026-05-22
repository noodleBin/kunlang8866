load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LED_MONITOR_COPTS = [
    '-DMODULE_NAME=\\"led_monitor\\"',
    '-std=c++17',
]

cc_library(
    name = "led_monitor_component_lib",
    srcs = ["led_monitor_component.cc"],
    hdrs = ["led_monitor_component.h" ],
    copts = LED_MONITOR_COPTS,
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
        "//modules/led_monitor/controller:led_controller",
    ],
)

cc_binary(
    name = "libled_monitor_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":led_monitor_component_lib"],
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
        ":libled_monitor_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

cpplint()
