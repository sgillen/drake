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
    name = "vector_slice",
    hdrs = [
        "vector_slice.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "vector_slice_test",
    deps = [
        ":vector_slice",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_library(
    name = "scoped_timer",
    hdrs = [
        "scoped_timer.h",
    ],
    deps = [
        "//drake/common",
    ],
)


drake_cc_library(
    name = "abstract_zoh",
    hdrs = [
        "abstract_zoh.h",
    ],
    deps = [
        "//drake/common",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "abstract_zoh_test",
    deps = [
        ":abstract_zoh",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_library(
    name = "dart_util",
    srcs = [
        "dart_util.cc",
    ],
    hdrs = [
        "dart_util.h",
    ],
    deps = [
        ":scoped_timer",
        ":vector_slice",
        "//drake/common:common",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/solvers:decision_variable",
        "//drake/systems/sensors:rgbd_camera",
        "@fmt",
    ],
)

drake_cc_googletest(
    name = "dart_util_test",
    deps = [
        ":dart_util",
    ],
)

drake_cc_library(
    name = "dart_icp",
    srcs = [
        "dart_icp.cc",
    ],
    hdrs = [
        "dart_icp.h",
    ],
    deps = [
        ":dart_util",
        "//drake/solvers:cost",
    ],
)

drake_cc_library(
    name = "dart",
    srcs = [
        "dart.cc",
        "dart_objectives.cc",
        "dart_depth_image_icp_objective.cc",
    ],
    hdrs = [
        "dart.h",
        "dart_objectives.h",
        "dart_depth_image_icp_objective.h",
    ],
    deps = [
        ":dart_icp",
        ":dart_util",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_googletest(
    name = "dart_icp_test",
    data = [
        ":models",
    ],
    deps = [
        ":dart_icp",
        "//drake/lcm",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/solvers:mathematical_program",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_googletest(
    name = "dart_test",
    deps = [
        ":dart",
    ],
)
