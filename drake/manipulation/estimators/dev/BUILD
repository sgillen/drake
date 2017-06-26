# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "dart_util",
    srcs = [
        "dart_util.cc",
    ],
    hdrs = [
        "abstract_zoh.h",
        "vector_slice.h",
        "dart_util.h",
    ],
    deps = [
        "//drake/common:common",
        "//drake/common:drake_path",
        "//drake/multibody:rigid_body_tree",
        "//drake/solvers:decision_variable",
        "//drake/systems/sensors:rgbd_camera",
        "@fmt",
    ],
)
