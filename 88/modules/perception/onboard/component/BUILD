load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libperception_component_camera.so",
    linkshared = True,
    linkstatic = False,
#    deps = [":perception_component_inner_camera"],
)

install(
    name = "install",
    targets = [
        ":libperception_component_camera.so",
        ":libperception_component_lidar.so",
    ],
    deps = [
        "//cyber:install",
        "//modules/perception/production:install",
    ],
)

#cc_library(
#    name = "perception_component_inner_camera",
#    srcs = [
#        "camera_obstacle_detection_component.cc",
#        "camera_perception_viz_message.cc",
#        "fusion_camera_detection_component.cc",
#        "lane_detection_component.cc",
#        "trafficlights_perception_component.cc",
#    ],
#    hdrs = [
#        "camera_obstacle_detection_component.h",
#        "camera_perception_viz_message.h",
#        "fusion_camera_detection_component.h",
#        "lane_detection_component.h",
#        "trafficlights_perception_component.h",
#    ],
#    copts = PERCEPTION_COPTS,
#    deps = [
#        "//cyber",
#        "//cyber/time:clock",
#        "//modules/common/math",
#        "//modules/common/proto:error_code_cc_proto",
#        "//modules/common/proto:geometry_cc_proto",
#        "//modules/common/proto:header_cc_proto",
#        "//modules/common/util:eigen_defs",
#        "//modules/common/util:time_util",
#        "//modules/drivers/proto:sensor_image_cc_proto",
#        "//modules/map/proto:map_cc_proto",
#        "//modules/perception/base",
#        "//modules/perception/camera/app:cipv_camera",
#        "//modules/perception/camera/app:lane_camera_perception",
#        "//modules/perception/camera/app:obstacle_camera_perception",
#        "//modules/perception/camera/app:obstacle_detection_camera",
#        "//modules/perception/camera/app:traffic_light_camera_perception",
#        "//modules/perception/camera/common",
#        "//modules/perception/camera/lib/interface",
#        "//modules/perception/camera/lib/motion_service:motion_service_lib",
#        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
#        "//modules/perception/camera/tools/offline:transform_server",
#        "//modules/perception/camera/tools/offline:visualizer",
#        "//modules/perception/common/sensor_manager",
#        "//modules/perception/map/hdmap:hdmap_input",
#        "//modules/perception/onboard/common_flags",
#        "//modules/perception/onboard/inner_component_messages",
#        "//modules/perception/onboard/msg_serializer",
#        "//modules/perception/onboard/proto:fusion_camera_detection_component_cc_proto",
#        "//modules/perception/onboard/proto:lane_perception_component_cc_proto",
#        "//modules/perception/onboard/proto:trafficlights_perception_component_cc_proto",
#        "//modules/perception/onboard/transform_wrapper",
#        "//modules/perception/proto:perception_camera_cc_proto",
#        "//modules/perception/proto:perception_lane_cc_proto",
#        "//modules/perception/proto:traffic_light_detection_cc_proto",
#        "//modules/transform/proto:transform_cc_proto",
#        "//modules/v2x/common:v2x_proxy_gflags",
#        "//modules/v2x/proto:v2x_traffic_light_cc_proto",
#        "@eigen",
#        "@opencv//:highgui",
#    ],
#    alwayslink = True,
#)

cc_binary(
    name = "libperception_component_lidar.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_component_inner_lidar"],
)

cc_library(
    name = "perception_component_inner_lidar",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":lidar_detector_component",
        ":lidar_freespace_component",
        ":lidar_segmentation_component",
        ":lidar_visualizer_component",
        ":liblidar_camera_visual_component",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_inner_component_messages",
    hdrs = [
        "lidar_inner_component_messages.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/lidar/common",
        "//modules/perception/onboard/inner_component_messages",
    ],
)

cc_library(
    name = "detection_component",
    srcs = ["detection_component.cc"],
    hdrs = ["detection_component.h"],
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_segmentation_component",
    srcs = ["lidar_segmentation_component.cc"],
    hdrs = ["lidar_segmentation_component.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/common:perception_timer_util",
        "//modules/perception/common:fps_counter",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_detector_component",
    srcs = ["lidar_detector_component.cc"],
    hdrs = ["lidar_detector_component.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:string_util",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/lib/nvjpeg_decoder:nvjpeg_decoder_lib",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/common/visualization:lidar_detector_viz",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detector",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/freespace:freespace_mask",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/pointcloud_cluster",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/msg_buffer:time_window_synchronizer",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/lidar_tracking:sample_lidar_tracking",
        "//modules/perception/lidar_tracking:shell_component",
        "//modules/perception/common:perception_timer_util",
        "//modules/perception/proto:perception_obstacle_debug_cc_proto",
        "//modules/perception/lidar/lib/object_filter_bank/cone_filter",
        "//modules/perception/base:singleton",
        "@eigen",
        "@opencv//:calib3d",
    ],
)

cc_library(
    name = "lidar_freespace_component",
    srcs = ["lidar_freespace_component.cc"],
    hdrs = ["lidar_freespace_component.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":lidar_inner_component_messages",
        "//cyber",
        "//modules/perception/lidar/common:lidar_log",
        "//modules/perception/lidar/lib/freespace:freespace_mask",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:perception_obstacle_debug_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_visualizer_component",
    srcs = ["lidar_visualizer_component.cc"],
    hdrs = [
        "lidar_camera_visual_component.h",
        "lidar_visualizer_component.h",
    ],
    # Fix OpenCV/NvJPEG symbol conflicts on Orin
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-Wl,--exclude-libs,libjpeg.a",
            "-Wl,--exclude-libs,libjpeg.so",
            "-Wl,--exclude-libs,libjpeg.so.8",
        ],
        "//conditions:default": [],
    }),
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/common/visualization:lidar_detector_viz",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detector",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/pointcloud_cluster",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/proto:visualizer_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/lidar_tracking:sample_lidar_tracking",
        "//modules/perception/lidar_tracking:shell_component",
        "//modules/perception/common:perception_timer_util",
        "//modules/perception/proto:perception_obstacle_debug_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//third_party/pangolin:pangolin",
        "@eigen",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ],
    copts = ["-DHAVE_GLEW",
            "-DGLEW_NO_GLU"],
)

cc_library(
    name = "liblidar_camera_visual_component",
    srcs = ["lidar_camera_visual_component.cc"],
    hdrs = ["lidar_camera_visual_component.h"],
    linkopts = ["-lgomp"] + select({
        "@platforms//cpu:aarch64": [
            "-Wl,--exclude-libs,libjpeg.a",
            "-Wl,--exclude-libs,libjpeg.so",
            "-Wl,--exclude-libs,libjpeg.so.8",
        ],
        "//conditions:default": [],
    }),
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/perception/proto:perception_obstacle_debug_cc_proto",
        "//modules/perception/common/visualization:lidar_detector_viz",
        "//modules/perception/onboard/proto:visualizer_component_config_cc_proto",
        "//modules/perception/common/sensor_manager:camera_sensor_config",
        "//modules/perception/lib/thread:concurrent_queue",
        "//modules/perception/base",
        "//third_party/pangolin:pangolin",
        "@eigen",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ] + select({
        "@platforms//cpu:aarch64": ["//modules/perception/camera/lib/nvjpeg_decoder:nvjpeg_decoder_lib"],
        "//conditions:default": [],
    }),
    copts = ["-fopenmp", "-DHAVE_GLEW",
            "-DGLEW_NO_GLU"],
)

cc_library(
    name = "fusion_component",
    srcs = ["fusion_component.cc"],
    hdrs = ["fusion_component.h"],
    deps = [
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/fusion/lib/interface:base_multisensor_fusion",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:fusion_component_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cc_binary(
    name = "test_point_pillar",
    srcs = [
        "test/test_point_pillar_detection.cc",
    ],
    deps = [
        "//modules/perception/lidar/lib/detector/pointpillars_detection_nv:point_pillars_detector",
        "@local_config_pcl//:pcl",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cpplint()
