# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "204525db008b473a2b1108422bb518abaf871010"

_SHA256 = "0"*64

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        local_repository_override = "/home/eacousineau/proj/tri/repo/repro/externals/pybind11",
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
'''.format(**vars)
    )
