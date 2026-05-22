load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


COMMON_DEPS = [
    "//cyber",
    "//cyber/time:clock",
    # "//cyber/transport/shm:shm_loc",
    "//cyber/transport/shm:shm_queue",
    "//cyber/common:time_conversion",
    "//cyber/threadlib:threadlib",
    "//modules/common/util:time_util",
    "//modules/common/adapters:adapter_gflags",
    "//modules/common/math",
    "//modules/common/monitor_log",
    "//modules/common/proto:geometry_cc_proto",
    "//modules/common/status",
    "//modules/common/configs:config_gflags",
    "//modules/drivers/gnss/proto:config_cc_proto",
    "//modules/drivers/gnss/proto:imu_cc_proto",
    "//modules/drivers/proto:pointcloud_cc_proto",
    "//modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg:rs_driver_msg",
    "//modules/transform:transform_broadcaster",

    "//modules/localization/proto:lidar_config_cc_proto",
    "//modules/localization/proto:localization_cc_proto",
    "//modules/localization/proto:pose_cc_proto",
    "//modules/localization/proto:msf_reset_cc_proto",
    "//modules/localization/common:loc_common",
    "//modules/localization/common:loc_time",
    "//modules/localization/common:localization_common",
    "//modules/localization/msf/common/io:common_io",
    "//modules/localization/msf/common/lidar:lidar_utils",
    "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",


    "@boost",
    "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    "@eigen",
]

cc_library(
    name = "lidar_loc_component_lib",
    srcs = [
        "lidar_loc_component.cc",
    ],
    hdrs = [
        "lidar_loc_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"NDTlocalization\\\"",
        "-O1",
    ],
    deps = COMMON_DEPS,
)

cc_library(
    name = "msf_task",
    hdrs = [
        "msf_task.h",
    ],
    srcs = [
        "msf_task.cc",
    ],
    deps = [
        "//cyber",
        "//third_party/mmath",
        "//modules/localization/common:loc_time",
    ],
)

cc_binary(
    name = "liblidar_loc_component.so",
    linkshared = True,
    linkstatic = False,
    copts = [
      "-DMODULE_NAME=\\\"NDTlocalization\\\"",
      "-O1",
    ],
    deps = [
        ":lidar_loc_component_lib",
    ],
)

install(
    name = "install",
    targets = [
        ":liblidar_loc_component.so",
    ],
)

MSF_COMMON_DEPS = [
    ":msf_task",
    "//cyber",
    "//cyber/time:clock",
    "//cyber/time:rate",
    "//cyber/timer:timer",
    "//cyber/common:time_conversion",
    "//cyber/threadlib:threadlib",
    "//modules/common/util:time_util",
    "//modules/common/adapters:adapter_gflags",
    "//modules/common/math",
    "//modules/common/monitor_log",
    "//modules/common/proto:geometry_cc_proto",
    "//modules/common/status",
    "//modules/common/configs:config_gflags",
    "//modules/localization/proto:msf_config_cc_proto",
    "//modules/localization/proto:localization_cc_proto",
    "//modules/localization/proto:msf_reset_cc_proto",
    "//modules/localization/common:loc_common",
    "//modules/localization/msf/common/filter:kalman_filter",
    "//modules/canbus/proto:chassis_cc_proto",
    "//modules/drivers/gnss/proto:ins_cc_proto",

    "//third_party/mmath",
    "@boost",
    "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    "@eigen",
]

cc_library(
    name = "msf_loc_component_lib",
    srcs = [
        "msf_loc_component.cc",
    ],
    hdrs = [
        "msf_loc_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"loc\\\"",
        "-O1",
    ],
    deps = MSF_COMMON_DEPS,
)

cc_binary(
    name = "libmsf_loc_component.so",
    linkshared = True,
    linkstatic = False,
    copts = [
      "-DMODULE_NAME=\\\"loc\\\"",
      "-O1",
    ],
    deps = [
        ":msf_loc_component_lib",
    ],
)

install(
    name = "install_msf",
    targets = [
        ":libmsf_loc_component.so",
    ],
)

cc_library(
    name = "record_to_bin_lib",
    srcs = [
        "record_to_bin.cc",
    ],
    hdrs = [
        "record_to_bin.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"loc\\\"",
        "-O1",
    ],
    deps = MSF_COMMON_DEPS,
)

cc_binary(
    name = "librecord_to_bin.so",
    linkshared = True,
    linkstatic = False,
    copts = [
      "-DMODULE_NAME=\\\"loc\\\"",
      "-O1",
    ],
    deps = [
        ":record_to_bin_lib",
    ],
)

install(
    name = "install_rtb",
    targets = [
        ":librecord_to_bin.so",
    ],
)

cpplint()
