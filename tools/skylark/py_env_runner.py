#!/usr/bin/env python

"""
Wrapper Python script to ensure we can execute a C++ binary with access to
Python libraries using an environment established by Bazel.
"""

# TODO(eric.cousineau): See if there is a way to do this in pure C++, such
# that it is easier to debug.

import os
import subprocess
import sys

assert len(sys.argv) >= 2
args = sys.argv[1:]
print(args)
while args:
    arg = args[0]
    runfiles_flag = "--runfiles_relpath="
    ld_flag = "--add_library_path="
    py_flag = "--add_py_path="
    if arg.startswith(runfiles_flag):
        script_dir = os.path.dirname(__file__)
        relpath = arg[len(runfiles_flag):]
        parent = os.path.relpath(".", relpath)
        os.chdir(os.path.join(script_dir, parent))
        print(parent)
        print(os.getcwd())
    elif arg.startswith(ld_flag):
        path = arg[len(ld_flag):]
        env = (sys.platform.startswith("linux") and
            "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
        os.environ[env] = path + ":" + os.environ[env]
    elif arg.startswith(py_flag):
        path = arg[len(py_flag):]
        # sys.path.insert(0, os.path.abspath(path))
        env = "PYTHONPATH"
        os.environ[env] = path + ":" + os.environ[env]
    else:
        break
    del args[0]

print(args)

assert len(args) >= 1
subprocess.check_call(args)
