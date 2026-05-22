load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "scan_record_to_pointcloud",
    srcs = ["scan_record_to_pointcloud.cc"],
    copts = ["-std=c++17", "-O2"],
    deps = [
        "//cyber",
        "//cyber/common:file",
        "//cyber/record:record_reader",
        "//cyber/record:record_writer",
        "//modules/drivers/lidar/proto:robosense_proto",
        "//modules/drivers/lidar/robosense:robosense_driver_lib",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
