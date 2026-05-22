load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "loc_time",
    hdrs = [
        "loc_time.h",
    ],
    deps = [
        "//cyber/time:clock",
    ],
)

cc_library(
    name = "localization_common",
    srcs = [
        "localization_gflags.cc",
    ],
    hdrs = [
        "localization_gflags.h",
    ],
    deps = [
        "//cyber",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/localization/proto:localization_cc_proto",
    ],
)

cc_library(
    name = "loc_common",
    srcs = glob([
        "io/*.cc",
    ]),
    hdrs = glob([
        "io/*.h",
    ]),
    copts = [
    #    "-DMODULE_NAME=\\\"loc\\\"",
        "-O2",
    ],
    deps = [
        "@boost",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "//third_party/tf2",
        "//third_party/mmath",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
        "//cyber/threadlib:threadlib",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/status",
        "//modules/common/util:time_util",
        "//modules/common/math",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/transform:transform_broadcaster",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
    ],
    includes = ["."],
    include_prefix = "modules/localization/common/",
)

cpplint()
