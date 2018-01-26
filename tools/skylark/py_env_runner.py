#!/usr/bin/env python

"""
Wrapper Python script to ensure we can execute a binary with access to
Python libraries using an environment established by Bazel.
"""

# TODO(eric.cousineau): See if there is a way to do this in pure C++, such
# that it is easier to debug.

import os
import subprocess
import sys

runfiles_dir = os.getcwd()

def add_path(env, path):
    print(env, os.path.join(runfiles_dir, path))
    os.environ[env] = (
        os.path.join(runfiles_dir, path) + ":" + os.environ.get(env, ''))

args = sys.argv[1:]
while args:
    arg = args[0]
    # Determine anchor for `runfiles`.
    runfiles_flag = "--runfiles_relpath="
    # Add library paths.
    ld_flag = "--add_library_path="
    # Add python paths.
    py_flag = "--add_py_path="
    if arg.startswith(runfiles_flag):
        script_dir = os.path.dirname(__file__)
        exec_relpath = arg[len(runfiles_flag):]
        runfiles_relpath = os.path.relpath(".", exec_relpath)
        runfiles_dir = os.path.abspath(
            os.path.join(script_dir, runfiles_relpath))
    elif arg.startswith(ld_flag):
        env = (sys.platform.startswith("linux") and
            "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
        add_path(env, arg[len(ld_flag):])
    elif arg.startswith(py_flag):
        add_path("PYTHONPATH", arg[len(py_flag):])
    else:
        break
    del args[0]

assert len(args) >= 1
bin_path = os.path.join(runfiles_dir, args[0])
subprocess.check_call([bin_path] + args[1:])
