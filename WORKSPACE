# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

# -- START drake externals
# Copy and paste this code section to any external Bazel projects that depend
# on drake and need its dependencies for convenience.
# @note See @drake//tools:externals.bzl for more info.

# Change this to the path of drake relative to the active WORKSPACE.
# TODO(eric.cousineau): See if there is a way to get:
#   Label("@drake//:install").workspace_root to NOT always return "external/drake"
drake_workspace_dir = "."

# # Enable this if `drake` is being consumed as an external local repository.
# local_repository(
#     name = "drake",
#     path = drake_workspace_dir,
# )

# Change this to the path of the CMake build/ directory to consume drake-visualizer.
# For more information, see #5621.
drake_cmake_install_dir = drake_workspace_dir + "/build/install"

# Load external rules that are immediately used to define other externals.
load("@drake//tools:externals_rules.bzl", "drake_external_rule_repositories")
drake_external_rule_repositories(drake_workspace_dir)

# Load external repostories.
load("@drake//tools:externals.bzl", "drake_external_repositories")
drake_external_repositories(
    cmake_install_dir = drake_cmake_install_dir,
)
# -- END drake externals
