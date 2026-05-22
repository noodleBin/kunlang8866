load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")


package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "century_ad_ipc",
    srcs = [],
    hdrs = ["century_ad_ipc.h"],
    deps = [
        #"//cyber/threadlib:threadlib"
        ],
    copts = CANBUS_COPTS,
)

cc_library(
    name = "century_ad_ipc_util",
    srcs = [],
    hdrs = ["century_ad_ipc_util.h"],
    copts = CANBUS_COPTS,
)

cc_library(
    name = "century_ad_ipc_msg",
    srcs = [],
    hdrs = ["century_ad_ipc_msg.h"],
    copts = CANBUS_COPTS,
)

cc_library(
    name = "ipc_header",
    srcs = [],
    hdrs = glob(["century*.h"]),
    deps = [
        #"//cyber/threadlib:threadlib"
        ],
    copts = CANBUS_COPTS,
)

cc_library(
    name = "canbus_component_lib",
    srcs = ["canbus_component.cc"],
    hdrs = [
        "canbus_component.h",
        "century_ad_ipc.h",
        "century_ad_ipc_msg.h",
        "century_ad_ipc_util.h",
    ],
    copts = CANBUS_COPTS,
    linkopts = [
        "-lrt",
        "-lpthread",
        "-lanl",
    ],
    deps = [
        "//cyber",
        #"//cyber/timedelay",
        "//modules/canbus/common:canbus_common",
        "//third_party/dbc:can_dbc",
        "//third_party/dbc:can_drv",
        "//third_party/dbc:can_header",
        "//third_party/dbc:can_math",
        "//third_party/dbc:can_parser_if",
        "//third_party/dbc:can_parser_imp_18",
        "//third_party/dbc:can_parser_imp_18EFD011",
        "//third_party/dbc:can_parser_imp_18EFD014",
        "//third_party/dbc:can_parser_imp_18FF1080",
        "//third_party/dbc:can_parser_imp_18FF1081",
        "//third_party/dbc:can_parser_imp_18FF1082",
        "//third_party/dbc:can_parser_imp_18FF1480",
        "//third_party/dbc:can_parser_imp_18FF1084",
        "//third_party/dbc:can_parser_imp_18FF1180",
        "//third_party/dbc:can_parser_imp_203",
        "//third_party/dbc:can_parser_imp_18FF8010",
        "//third_party/dbc:can_parser_imp_18FF8110",
        "//third_party/dbc:can_parser_imp_18FF8510",
        "//third_party/dbc:can_parser_imp_19",
        "//third_party/dbc:can_parser_imp_B",
        "//third_party/dbc:can_parser_imp_1883EFF3",
        "//third_party/dbc:can_parser_imp_1884EFF3",
        "//third_party/dbc:can_parser_imp_18FF1280",
        "//third_party/dbc:socket_can_vehicleinforx",
        "//third_party/dbc:socket_can_vehicleinfotx",
        "//modules/canbus/lib/ipc:centuryadipclib",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/guardian/proto:guardian_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

#cc_test(
    #   name = "canbus_test",
    #   size = "small",
    #  srcs = ["canbus_test.cc"],
    #  deps = [
        #    ":canbus_component_lib",
        #   "//modules/common/status",
        #   "@com_google_googletest//:gtest_main",
        #  ],
        #)

cc_binary(
    name = "libcanbus_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":canbus_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcanbus_component.so",
    ],
    deps = [
        ":pb_canbus",
        "//cyber:install",
    ],
)

install_files(
    name = "pb_canbus",
    dest = "modules/canbus",
    files = [
        "//modules/canbus/proto:chassis_py_pb2",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

#TODO(storypku):
# split test_data for each vehicle
filegroup(
    name = "test_data",
    srcs = glob(["testdata/**"]),
)

cpplint()
