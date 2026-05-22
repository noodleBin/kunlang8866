load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

exports_files([
    "conf/cyber_ros_bridge_conf.pb.txt",
])

cc_proto_library(
    name = "cyber_ros_bridge_conf_cc_proto",
    deps = [
        ":cyber_ros_bridge_conf_proto",
    ],
)

proto_library(
    name = "cyber_ros_bridge_conf_proto",
    srcs = ["cyber_ros_bridge_conf.proto"],
)

cc_library(
    name = "cyber_ros_bridge_config",
    srcs = ["cyber_ros_bridge_config.cc"],
    hdrs = ["cyber_ros_bridge_config.h"],
    deps = [
        ":cyber_ros_bridge_conf_cc_proto",
        "//cyber/common:file",
    ],
)

cc_binary(
    name = "lidar_shm_bridge",
    srcs = [
        "lidar_shm_bridge.cc",
        "tf_static_shm.h",
    ],
    deps = [
        "//cyber",
        ":cyber_ros_bridge_config",
        "//cyber/transport/shm:shm_queue",
        "//modules/common/util",
        "//modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg:rs_driver_msg",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
    ],
)

cc_binary(
    name = "shm_reader",
    srcs = [
        "shm_reader.cc",
        "tf_static_shm.h",
    ],
    linkopts = [
        "-pthread",
        "-lrt",
    ],
    deps = [
        ":cyber_ros_bridge_config",
        "//cyber/transport/shm:shm_queue",
        "//modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg:rs_driver_msg",
    ],
)

cpplint()
