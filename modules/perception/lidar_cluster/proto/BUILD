load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "lidar_cluster_component_proto",
    srcs = ["lidar_cluster_component.proto"],
    deps = [
        "//modules/perception/common/proto:plugin_param_proto",
    ],
)

apollo_package()

cpplint()