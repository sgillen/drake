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

# N.B. We must defer these, because of bazelbuild/bazel#3998, which affects
# @vtk.
defer_py = os.environ.get("DEFER_PYTHONPATH", "")
if defer_py:
    os.environ["PYTHONPATH"] = (
        defer_py + ":" + os.environ.get("PYTHONPATH", ""))

args = sys.argv[1:]
assert len(args) >= 1
subprocess.check_call(args)
