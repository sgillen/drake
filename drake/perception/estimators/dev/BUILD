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
    # TODO(eric.cousineau): Make this private and testonly.
    visibility = ["//visibility:public"],
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

#drake_cc_googletest(
cc_binary(
    name = "articulated_icp_bin",
    data = [
        ":models",
    ],
    srcs = ["test/articulated_icp_test.cc"],
    testonly = 1,
    deps = [
        ":articulated_icp",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/solvers:mathematical_program",
        "@gtest//:main",
        "@lcmtypes_bot2_core",
    ],
)
