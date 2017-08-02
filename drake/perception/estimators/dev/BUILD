# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "models",
    srcs = glob([
        "**/*.urdf",
    ]),
    visibility = ["//visibility:private"],
    testonly = 1,
)

drake_cc_library(
    name = "icp",
    srcs = ["icp.cc"],
    hdrs = ["icp.h"],
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_library(
    name = "scene",
    srcs = ["scene.cc"],
    hdrs = ["scene.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
    ],
)

drake_cc_library(
    name = "articulated_icp",
    srcs = ["articulated_icp.cc"],
    hdrs = ["articulated_icp.h"],
    deps = [
        ":scene",
        "//drake/solvers:cost",
    ],
)

drake_cc_library(
    name = "test_util",
    srcs = ["test/test_util.cc"],
    hdrs = ["test/test_util.h"],
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody:rigid_body_tree",
        "@lcmtypes_bot2_core",
    ],
    testonly = 1,
)

drake_cc_googletest(
    name = "icp_test",
    deps = [
        ":icp",
        ":test_util",
    ],
)

drake_cc_googletest(
    name = "articulated_icp_test",
    data = [
        ":models",
    ],
    deps = [
        ":articulated_icp",
        ":test_util",
        "//drake/common:find_resource",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/solvers:mathematical_program",
    ],
)
