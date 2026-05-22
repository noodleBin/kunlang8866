load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

COMMON_DEPS = [
    "//cyber",
    "//cyber/time:clock",
    "//cyber/common:time_conversion",
    "//cyber/threadlib:threadlib",
    "//modules/common/util:time_util",
    "//modules/common/adapters:adapter_gflags",
    "//modules/common/math",
    "//modules/common/monitor_log",
    "//modules/common/proto:geometry_cc_proto",
    "//modules/common/status",
    "//modules/common/configs:config_gflags",
    "//modules/transform:transform_broadcaster",
    "//modules/drivers/gnss/proto:config_cc_proto",
    "//modules/drivers/gnss/proto:imu_cc_proto",
    "//modules/drivers/gnss/proto:gnss_cc_proto",
    "//modules/drivers/gnss/proto:ins_cc_proto",
    "//modules/localization/proto:ins_config_cc_proto",
    "//modules/localization/proto:localization_cc_proto",
    "//modules/localization/common:loc_common",

    "//third_party/mmath",
    "@boost",
    "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    "@eigen",
]

cc_library(
    name = "ins_loc_component_lib",
    srcs = [
        "ins_loc_component.cc",
    ],
    hdrs = [
        "ins_loc_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"loc\\\"",
        "-O1",
    ],
    deps = COMMON_DEPS,
)

cc_binary(
    name = "libins_loc_component.so",
    linkshared = True,
    linkstatic = False,
    copts = [
      "-DMODULE_NAME=\\\"loc\\\"",
      "-O1",
    ],
    deps = [
        ":ins_loc_component_lib",
    ],
)

install(
    name = "install",
    targets = [
        ":libins_loc_component.so",
    ],
)

cpplint()
