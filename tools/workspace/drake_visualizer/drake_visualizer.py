from __future__ import print_function

import os
from os.path import exists, join
import subprocess
import sys


def resolve_path(relpath):
    test_paths = [
        # This is for bazel run //tools:drake_visualizer.
        join(runfiles_dir, relpath),
    ]
    if not relpath.startswith("external/"):
        # This is for bazel run @drake//tools:drake_visualizer.
        test_paths.insert(0, join(runfiles_dir, "external/drake", relpath))
    for p in test_paths:
        if exists(p):
            return p
    assert False, (
        "No available paths:\n{}".format("\n".join(test_paths)))


def set_path(key, relpath):
    os.environ[key] = resolve_path(relpath)


def prepend_path(key, relpath):
    os.environ[key] = resolve_path(relpath) + ":" + os.environ.get(key, '')


assert __name__ == "__main__"
runfiles_dir = os.environ.get("DRAKE_BAZEL_RUNFILES")
assert runfiles_dir, (
    "This must be called by a script generated using the " +
    "`drake_runfiles_binary` macro.")

# Stub out pydrake (refer to our ./BUILD.bazel comments for rationale).
#
# We add it to PYTHONPATH within the script, rather than `imports = ["stub"]`
# on the stub py_library, to avoid any other target accidentally pulling in the
# stubbed pydrake onto its PYTHONPATH.  Only the visualizer, when launched via
# this wrapper script, should employ the stub.
prepend_path("PYTHONPATH", "tools/workspace/drake_visualizer/stub")

# Don't use DRAKE_RESOURCE_ROOT; the stub getDrakePath should always win.  This
# also placates the drake-visualizer logic that puts it into Director mode when
# DRAKE_RESOURCE_ROOT is set (thus requiring more than just getDrakePath).
try:
    del os.environ["DRAKE_RESOURCE_ROOT"]
except KeyError:
    pass

# TODO(eric.cousineau): Remove these shims if we can teach Bazel how to handle
# these on its own.
if sys.platform.startswith("linux"):
    # Ensure that we handle LD_LIBRARY_PATH for @lcm and @vtk and PYTHONPATH
    # for @vtk.
    set_path("LD_LIBRARY_PATH", "external/lcm")
    prepend_path("LD_LIBRARY_PATH", "external/vtk/lib")
    prepend_path("PYTHONPATH", "external/vtk/lib/python2.7/site-packages")
elif sys.platform == "darwin":
    # Ensure that we handle DYLD_LIBRARY_PATH for @lcm.
    set_path("DYLD_LIBRARY_PATH", "external/lcm")

# Execute binary.
bin_path = resolve_path("external/drake_visualizer/bin/drake-visualizer")
args = [bin_path] + sys.argv[1:]

# Check for default script.
default_arg = "--use-drake-scripts"
if default_arg in args:
    args.remove(default_arg)
    script_path = resolve_path(
        "tools/workspace/drake_visualizer/drake_scripts.py")
    args += ["--script", script_path]

os.execv(bin_path, args)
