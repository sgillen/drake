"""
Hacky workaround for https://github.com/bazelbuild/bazel/issues/4594

WARNING: This may leak in additional libraries, or not respect proper order!
"""

from collections import OrderedDict
import os
import subprocess
import sys

def _fix_external_bazel_shared_libs(module, runfiles_dir, check_dir):
    """Ensures all shared libraries are loadable, even if they are incorrectly
    RPATH linked.
    @see https://stackoverflow.com/a/25457751/7829525
    """
    key_workaround = "BAZEL_4594_WORKAROUND_" + module
    if key_workaround in os.environ:
        # Only do this hack workaround once.
        return
    # Find all libraries, and get the directories.
    out = subprocess.check_output(
        "find {} -name *.so -o -name *.so.*".format(runfiles_dir).split())
    lines = out.split("\n")
    # Collect all unique directories in order.
    dirs = []
    for line in lines:
        file = line.strip()
        d = os.path.dirname(file)
        if d not in dirs:
            assert os.path.exists(d)
            assert os.path.isabs(d)
            dirs.append(d)
    # Append to path variable.
    if sys.platform.startswith("darwin"):
        key = "DYLD_LIBRARY_PATH"
    else:
        key = "LD_LIBRARY_PATH"
    paths = os.environ.get(key, "").split(os.pathsep)
    paths += dirs
    os.environ[key] = os.pathsep.join(paths)
    # Ensure that this only happens once.
    os.environ[key_workaround] = "1"
    # N.B. `python` needs to have arg[0] be itself, not the script.
    args = [sys.executable] + sys.argv
    sys.stdout.flush()
    os.execv(args[0], args)
