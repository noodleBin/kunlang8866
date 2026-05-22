load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LED_MONITOR_COPTS = ['-DMODULE_NAME=\\"led_monitor\\"']


cc_library(
    name = "cpp_http_lib",
    hdrs = [
      "httplib.h"
    ],
    copts = LED_MONITOR_COPTS,
    deps = [
    ],
)

cpplint()