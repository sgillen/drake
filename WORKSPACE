# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

# To temporarily use a local copy of a github_archive, within this file add a
# local_repository_archive argument to its github_archive macro call, e.g.:
#
# github_archive(
#     name = "foobar",
#     local_repository_override = "/path/to/local/foo/bar",
#     repository = "foo/bar",
#     commit = "0123456789abcdef0123456789abcdef01234567",
#     sha256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
# )

workspace(name = "drake")

# -- START Required Load-Level Transitive Dependencies
load("//tools:github.bzl", "github_archive")
github_archive(
    name = "io_bazel_rules_go",
    repository = "bazelbuild/rules_go",
    commit = "0.4.4",
    sha256 = "afec53d875013de6cebe0e51943345c587b41263fdff36df5ff651fbf03c1c08",
)
# -- END Required Load-Level Transitive Dependencies

# -- Transferrable dependencies
load(":deps.bzl", "drake_deps")
drake_deps(
    install_dir = __workspace_dir__ + "/build/install",
)
