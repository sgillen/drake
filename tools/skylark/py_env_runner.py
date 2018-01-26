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
    ld_flag = "--add_library_path="
    py_flag = "--add_py_path="
    first_flag = "--get_first="
    if arg.startswith(ld_flag):
        path = arg[len(ld_flag):]
        env = (sys.platform.startswith("linux") and
            "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
        os.environ[env] = path + ":" + os.environ[env]
    elif arg.startswith(py_flag):
        path = arg[len(py_flag):]
        # sys.path.insert(0, os.path.abspath(path))
        env = "PYTHONPATH"
        os.environ[env] = path + ":" + os.environ[env]
    elif arg == first_flag:
        meta_arg = arg[len(first_flag):]
        args[0] = meta_arg.split()[0]
    else:
        break
    del args[0]

print("\n".join(sys.path))
print("---")
print("\n".join(os.environ["PYTHONPATH"].split(":")))

print(args)
assert len(args) >= 1
subprocess.check_call(args)
