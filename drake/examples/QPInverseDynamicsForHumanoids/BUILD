# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
)

exports_files(
    ["config/valkyrie.alias_groups"],
)

exports_files(
    ["config/valkyrie.id_controller_config"],
)

exports_files(
    ["config/iiwa.alias_groups"],
)

exports_files(
    ["config/iiwa.id_controller_config"],
)

# === test/ ===

add_lint_tests()
