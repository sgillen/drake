# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()


# WARNING: Bazel also craps out here if `__workspace_dir__ + path` is used
# rather than just `path`.
# N.B. This error is *stateful*. You will get different behavior depending on
# what has been built / run previously in Bazel. In one mode, the error
# will be:
#   Encountered error while [...]
#   /home/${USER}/.cache/bazel/_bazel_${USER}/${HASH}/external/bazel_external_data_pkg  # noqa
#   must  be an existing directory
# In another mode, you will get Java errors:
#   java.lang.IllegalArgumentException: PathFragment
#   tools/external_data/workspace is not beneath
#   /home/${USER}/${WORKSPACE_DIR}/tools/external_data/workspace
local_repository(
    name = "bazel_external_data_pkg",
    path = "tools/external_data/workspace",
)
