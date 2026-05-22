load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "dump_amcl_pose",
    srcs = ["dump_amcl_pose.cc"
           ],
    deps = [
      "//modules/perception/landmark_loc/proto:amcl_recorder_cc_proto",
      "//cyber:cyber",
      "@boost",
      "@com_github_nlohmann_json//:json",
    ]
)
cc_binary(
    name = "colllect_dr_camera_data",
    srcs = ["colllect_dr_camera_data.cc"
           ],
    deps = [
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/common/visualization:lidar_detector_viz",
        "//modules/perception/lidar/lib/pointcloud_preprocessor:pointcloud_preprocessor",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:config_proto",
        "//cyber:cyber",
        "@opencv//:imgcodecs",
        "@opencv//:highgui",
        "@local_config_pcl//:pcl",

    ]
)