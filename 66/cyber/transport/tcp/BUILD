load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tcp_socket",
    srcs = ["tcp_socket.cc"],
    hdrs = [
        "tcp_socket.h",
        "tcp_buffer_conf.h",
    ],
    deps = [
        "//cyber/common",
        "//cyber/proto:tcp_buffer_conf_cc_proto",
    ],
)

cpplint()
