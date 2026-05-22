load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "receiver",
    deps = [
        ":hybrid_receiver",
        ":intra_receiver",
        ":rtps_receiver",
        ":shm_receiver",
        ":tcp_receiver",
    ],
)

cc_library(
    name = "receiver_interface",
    hdrs = ["receiver.h"],
    deps = [
        "//cyber/transport/common:endpoint",
        "//cyber/transport/message:history",
        "//cyber/transport/message:message_info",
    ],
)

cc_library(
    name = "hybrid_receiver",
    hdrs = ["hybrid_receiver.h"],
    deps = [
        ":receiver_interface",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/service_discovery:topology_manager",
    ],
)

cc_library(
    name = "intra_receiver",
    hdrs = ["intra_receiver.h"],
    deps = [
        ":receiver_interface",
    ],
)

cc_library(
    name = "rtps_receiver",
    hdrs = ["rtps_receiver.h"],
    deps = [
        ":receiver_interface",
        ":shm_receiver",
        "//cyber/message:message_traits",
        "//cyber/transport/common:proxy_manager",
        "//cyber/transport/transmitter:shm_transmitter",
    ],
)

cc_library(
    name = "shm_receiver",
    hdrs = ["shm_receiver.h"],
    deps = [
        ":receiver_interface",
        "//cyber/transport/shm:readable_info",
    ],
)

cc_library(
    name = "tcp_receiver",
    hdrs = ["tcp_receiver.h"],
    deps = [
        ":receiver_interface",
        ":shm_receiver",
        "//cyber/common",
        "//cyber/message:message_traits",
        "//cyber/proto:tcp_buffer_conf_cc_proto",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/service_discovery:topology_manager",
        "//cyber/transport/common:proxy_manager",
        "//cyber/transport/tcp:tcp_socket",
        "//cyber/transport/transmitter:shm_transmitter",
    ],
)

cpplint()
