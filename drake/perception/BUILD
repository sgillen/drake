# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "point_cloud",
    srcs = ["point_cloud.cc"],
    hdrs = ["point_cloud.h"],
    deps = [
        "//drake/common:copyable_unique_ptr",
        "//drake/common:essential",
        "//drake/systems/sensors:image",
    ],
)

drake_cc_googletest(
    name = "point_cloud_test",
    srcs = ["test/point_cloud_test.cc"],
    deps = [
        ":point_cloud",
        "//drake/common/test_utilities:eigen_matrix_compare",
    ],
)

add_lint_tests()
