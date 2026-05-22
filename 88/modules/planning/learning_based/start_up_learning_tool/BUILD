load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "start_up_learning_tool",
    srcs = ["start_up_learning_tool.cc"],
    deps = [
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:ml",
        "//cyber/common:file",
        "//modules/planning/proto:svm_feature_data_cc_proto",
    ]
)



filegroup(
    name = "start_up",
    srcs = glob([
        "start_up.xml"
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = [":start_up"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
)


cpplint()
