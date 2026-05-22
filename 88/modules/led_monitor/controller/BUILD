load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LED_MONITOR_COPTS = ['-DMODULE_NAME=\\"led_monitor\\"']

cc_library(
    name = "led_controller",
    srcs = ["led_controller.cc"],
    hdrs = ["led_controller.h"],
    copts = LED_MONITOR_COPTS,
    deps = [
      "//cyber",
      "//modules/led_monitor/cpphttplib:cpp_http_lib",
      "//modules/planning/proto:planning_cc_proto",
      "//modules/led_monitor/proto:led_monitor_config_cc_proto",
      "@com_github_nlohmann_json//:json",
      "@boost",
    ],
)

cpplint()