# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "254be27d44a3dc5e3c2e79b1dbcd10271e7c3a83"

_SHA256 = "f973c3cda2b261f3ac9deff1136037d390b5b77bcf71ea755bc32b18f42a3a77"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        local_repository_override = "/home/eacousineau/proj/tri/repo/repro/externals/pybind11",
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
'''.format(**vars)
    )
