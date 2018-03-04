"""
Works around: https://github.com/bazelbuild/bazel/issues/4594

Import this module to ensure all shared libraries in Bazel are loaded.
"""

from collections import OrderedDict
import os
import subprocess
import sys

def _handle_external_bazel_shared_libs():
    """Ensures all shared libraries are loadable, even if they are incorrectly
    RPATH linked.
    @see https://stackoverflow.com/a/25457751/7829525
    """
    key_workaround = "DRAKE_BAZEL_4594_WORKAROUND"
    if key_workaround in os.environ:
        return
    pwd = os.getcwd()
    is_bazel_runfiles = os.path.dirname(pwd).endswith(".runfiles")
    is_bazel_external = os.path.exists("external/drake")
    if is_bazel_runfiles and is_bazel_external:
        # Find all libraries, and get the directories.
        out = subprocess.check_output(
            "find . -name *.so -o -name *.so.*".split())
        lines = out.split("\n")
        # Collect all unique directories.
        dirs = []
        for line in lines:
            file = line.strip()
            d = os.path.abspath(os.path.dirname(file))
            if d not in dirs:
                assert os.path.exists(d)
                dirs.append(d)
        # Append to path variable.
        delim = ":"
        key = "LD_LIBRARY_PATH"
        paths = os.environ.get(key, "").split(delim)
        paths += dirs
        os.environ[key] = delim.join(paths)
        # Ensure that this only happens once.
        os.environ[key_workaround] = "1"
        # N.B. `python` needs to have arg[0] be itself, not the script.
        args = [sys.executable] + sys.argv
        sys.stdout.flush()
        exit(os.execv(args[0], args))

_handle_external_bazel_shared_libs()
