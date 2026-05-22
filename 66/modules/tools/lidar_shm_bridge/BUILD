load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lidar_shm_bridge",
    srcs = [
        "lidar_shm_bridge.cc",
        "tf_static_shm.h",
    ],
    deps = [
        "//cyber",
        "//cyber/transport/shm:shm_queue",
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
        "//cyber/transport/shm:shm_queue",
        "//modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg:rs_driver_msg",
    ],
)

cpplint()
