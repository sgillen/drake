"""
Hacky workaround for https://github.com/bazelbuild/bazel/issues/4594

WARNING: This may leak in additional libraries, or not respect proper order!
"""

from collections import OrderedDict
import os
import subprocess
import sys


def _fix_external_bazel_shared_libs(module, runfiles_dir):
    """Ensures all shared libraries are loadable, even if they are incorrectly
    RPATH linked by Bazel.
    @see https://stackoverflow.com/a/25457751/7829525
    """
    key_workaround = "BAZEL_4594_WORKAROUND_" + module
    if key_workaround in os.environ:
        # Only do this hack workaround once.
        return
    _fix_library_path(runfiles_dir)
    # Ensure that this only happens once.
    os.environ[key_workaround] = "1"
    # N.B. `python` needs to have arg[0] be itself, not the script.
    args = [sys.executable] + sys.argv
    sys.stdout.flush()
    os.execv(args[0], args)


def _fix_library_path(runfiles_dir):
    # Find all libraries, and get the directories.
    out = subprocess.check_output(
        "find {} -name *.so -o -name *.so.*".format(runfiles_dir).split())
    lines = out.split("\n")
    # Collect all unique directories in order.
    dirs = []
    for line in lines:
        file = line.strip()
        if not file:
            continue
        d = os.path.dirname(file)
        if d not in dirs:
            assert os.path.exists(d), "{}\n{}".format(d, file)
            assert os.path.isabs(d), "{}\n{}".format(d, file)
            dirs.append(d)
    # Append to path variable.
    is_mac = sys.platform.startswith("darwin")
    key = is_mac and "DYLD_LIBRARY_PATH" or "LD_LIBRARY_PATH"
    paths = os.environ.get(key, "").split(os.pathsep)
    paths = dirs + paths
    os.environ[key] = os.pathsep.join(paths)
