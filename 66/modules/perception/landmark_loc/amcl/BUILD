load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
cc_library(
    name = "amcl_loc",
    srcs = glob([
        "src/amcl/map/*.cpp",
        "src/amcl/map/*.c",
        "src/amcl/pf/*.c",
        "src/amcl/sensors/*.cpp",   
    ]),
    hdrs = glob([
        "include/amcl/map/*.h",
        "include/amcl/pf/*.h",
        "include/amcl/sensors/*.h",
    ]),
    includes = [
        "./include",
        "./src/include",
    ],
    copts=["-DHAVE_DRAND48","-g"],
    visibility = ["//visibility:public"],
    strip_include_prefix ="./include",
    deps = [    
      "@com_github_google_glog//:glog", 
    ],
)