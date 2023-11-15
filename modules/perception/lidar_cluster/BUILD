load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_cluster_files",
    srcs = glob([
        "dag/**",
        "launch/**",
        "conf/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_lidar_cluster",
    srcs = [
        "cluster/euclidean_cluster/euclidean_cluster.cc",
    ],
    hdrs = [
        "cluster/euclidean_cluster/euclidean_cluster.h",
        "interface/base_object_cluster.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_cluster/cluster/proto:euclidean_cluster_config_proto",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_component(
    name = "liblidar_cluster_component.so",
    srcs = [
        "lidar_cluster_component.cc",
    ],
    hdrs = [
        "lidar_cluster_component.h",
    ],
    linkstatic = True,
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/lidar_cluster/proto:lidar_cluster_component_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        ":apollo_perception_lidar_cluster",
    ],
)

apollo_package()

cpplint()