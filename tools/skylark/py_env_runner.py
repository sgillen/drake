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
        relpath = arg[len(runfiles_flag):]
        parent = os.path.relpath(".", relpath)
        os.chdir(os.path.join(script_dir, parent))
    elif arg.startswith(ld_flag):
        path = arg[len(ld_flag):]
        env = (sys.platform.startswith("linux") and
            "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
        os.environ[env] = path + ":" + os.environ[env]
    elif arg.startswith(py_flag):
        path = arg[len(py_flag):]
        env = "PYTHONPATH"
        os.environ[env] = path + ":" + os.environ[env]
    else:
        break
    del args[0]

assert len(args) >= 1
subprocess.check_call(args)
