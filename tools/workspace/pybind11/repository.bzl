# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# PR DRAFT(eric.cousineau): Change this once RobotLocomotion/pybind11#13 lands.
_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "41723b25bd2c97d1eddc2df0b2499e0f62874ec2"

_SHA256 = "2629f0cdcb331bc55b41e75b5cdad500180c2a1c3bdc971a0e62777d8a4245d2"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
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
'''.format(**vars)
    )
