# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# # Using the `drake` branch of this repository.
# _REPOSITORY = "RobotLocomotion/pybind11"

# Using `master` from upstream.
_REPOSITORY = "pybind/pybind11"

# This is the commit used in pytorch@v1.0.0:
# https://github.com/pytorch/pytorch/tree/v1.0.0/third_party
_COMMIT = "5c8746ff135abb390bf95944be593e895a586a50"

_SHA256 = "ffe077d0fccc34aa95c2aba7a3abadbdc2a480c44a238879381733d38322e921"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
    # native.new_local_repository(
        name = name,
        # path = "/home/eacousineau/proj/tri/repo/externals/pybind11",
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
        mirrors = mirrors,
    )

def generate_pybind11_version_py_file(name):
    vars = dict(
        repository = repr(_REPOSITORY),
        commit = repr(_COMMIT),
        sha256 = repr(_SHA256),
    )
    generate_file(
        name = name,
        content = '''
"""
Provides information on the external fork of `pybind11` used by `pydrake`.
"""

repository = {repository}
commit = {commit}
sha256 = {sha256}
'''.format(**vars),
    )
