# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "neural_network",
    srcs = [
    ],
    hdrs = [
        "neural_network.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "feedforward_neural_network",
    srcs = [
        "feedforward_neural_network.cc",
    ],
    hdrs = [
        "feedforward_neural_network.h",
    ],
    deps = [
        ":neural_network",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "point_cloud",
    srcs = ["point_cloud.cc"],
    hdrs = ["point_cloud.h"],
    deps = [
        "//drake/common",
        "//drake/systems/sensors:image",
    ],
)

cc_test(
    name = "point_cloud_test",
    srcs = ["test/point_cloud_test.cc"],
    deps = [
        ":point_cloud",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:drake_cc_googletest_main",
    ],
)

drake_cc_library(
    name = "point_cloud_vtk",
    deps = [
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonDataModel",
        "@vtk//:vtkCommonTransforms",
        "@vtk//:vtkFiltersGeneral",
        "@vtk//:vtkFiltersSources",
    ],
)

cc_test(
    name = "point_cloud_system_test",
    srcs = ["test/point_cloud_system_test.cc"],
    deps = [
        ":point_cloud",
        "//drake/systems/framework",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:drake_cc_googletest_main",
        "@vtk//:vtkFiltersGeneral",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "feedforward_neural_network_test",
    srcs = ["test/feedforward_neural_network_test.cc"],
    deps = [
        ":feedforward_neural_network",
        "//drake/common/test_utilities:eigen_matrix_compare",
    ],
)

add_lint_tests()
