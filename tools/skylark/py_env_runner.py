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

library_env = (sys.platform.startswith("linux") and "LD_LIBRARY_PATH" or
    "DYLD_LIBRARY_PATH")

assert len(sys.argv) >= 2
args = sys.argv[1:]
print(args)
while args:
    arg = args[0]
    ld_flag = "--add_library_path="
    if arg.startswith(ld_flag):
        path = arg[len(ld_flag):]
        os.environ[library_env] = path + ":" + os.environ[library_env]
        print("add: ", path)
        del args[0]
    else:
        break

assert len(args) >= 1
subprocess.check_call(args)
