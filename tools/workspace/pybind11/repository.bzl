# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# # Using the `drake` branch of this repository.
# _REPOSITORY = "RobotLocomotion/pybind11"
_REPOSITORY = "pybind/pybind11"

_COMMIT = "12e8774bc9aa4603136f2979088619b495850ca2"  # upstream/master

_SHA256 = "fbfda54cc97c2d1468ff83142b143d9bba901444e746218add812283f41ea863"

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
