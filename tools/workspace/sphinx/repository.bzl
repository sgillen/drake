# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def sphinx_repository(name):
    which_repository(
        name = name,
        command = "sphinx-build",
    )

    native.new_local_repository(
        name = "sphinx_py",
        path = "/home/eacousineau/proj/tri/repo/externals/sphinx",
        build_file_content = """
py_library(
    name = "sphinx_py",
    data = glob(["sphinx/**"]),
    imports = ["."],
    visibility = ["//visibility:public"],
)
""")
