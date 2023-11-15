load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_lidar_detection",
    srcs = [
        "object_builder/object_builder.cc",
    ],
    hdrs = [
        "interface/base_lidar_detector.h",
        "object_builder/object_builder.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        # "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        # "//modules/perception/lidar_detection/detector/center_point_detection/proto:center_point_model_param_cc_proto",
        # "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        # "//modules/perception/lidar_detection/detector/mask_pillars_detection/proto:model_param_cc_proto",
        # "//modules/perception/lidar_detection/detector/point_pillars_detection/proto:model_param_cc_proto",
        # "//modules/perception/lidar_detection/proto:lidar_detection_component_config_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@eigen",
        # "@libtorch_gpu",
        # "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        # "@local_config_tensorrt//:tensorrt",
    ] + select({
        "@platforms//cpu:x86_64": ["@centerpoint_infer_op-x86_64//:centerpoint_infer_op"],
        "@platforms//cpu:aarch64": ["@centerpoint_infer_op-aarch64//:centerpoint_infer_op"]
    }) + select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"]
    }),
)

apollo_component(
    name = "liblidar_detection_component.so",
    srcs = ["lidar_detection_component.cc"],
    hdrs = ["lidar_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        ":apollo_perception_lidar_detection",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_detection/proto:lidar_detection_component_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "cnn_segmentation_test",
    size = "large",
    srcs = ["detector/cnn_segmentation/cnn_segmentation_test.cc"],
    deps = [
        ":apollo_perception_lidar_detection",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "spp_cluster_list_test",
    size = "small",
    srcs = ["detector/cnn_segmentation/spp_engine/spp_cluster_list_test.cc"],
    deps = [
        ":apollo_perception_lidar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
