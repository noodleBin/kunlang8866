load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "ROUTING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "routing",
    srcs = ["routing.cc"],
    hdrs = ["routing.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util:point_factory",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:routing_point_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/routing/common:routing_math",
    ],
)

cc_library(
    name = "routing_component_lib",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":routing",
        "//cyber/time:clock",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
    ],
)

cc_binary(
    name = "librouting_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":routing_component_lib"],
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
        ":librouting_component.so",
    ],
    deps = [
        ":pb_routing",
        "//cyber:install",
        "//modules/routing/topo_creator:install",
    ],
)

install_files(
    name = "pb_routing",
    dest = "modules/routing",
    files = [
        "//modules/routing/proto:routing_py_pb2",
    ],
)

cpplint()
