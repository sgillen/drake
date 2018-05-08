"""
Workaround for https://github.com/bazelbuild/bazel/issues/4594

Caveats:
* This may leak in additional libraries.
* May not respect proper path ordering.
* If `pydrake` is imported later in a program, it may change the overall
program behavior.
"""

import os
import subprocess
import sys


def _module_key(module):
    return "BAZEL_4594_WORKAROUND_" + module


def _reexec_with_new_environment(module, runfiles_dir):
    # Ensures all shared libraries are loadable, even if they are incorrectly
    # RPATH linked by Bazel.
    # See https://stackoverflow.com/a/25457751/7829525
    key = _module_key(module)
    if key in os.environ:
        # Only do this workaround once.
        return
    _add_library_paths(runfiles_dir)
    # Ensure that this only happens once.
    os.environ[key] = "1"
    # N.B. `python` needs to have arg[0] be itself, not the script.
    args = [sys.executable] + sys.argv
    sys.stdout.flush()
    os.execv(args[0], args)


def _add_library_paths(start_dir):
    # Find all libraries, get the directories, and add to the appropriatve
    # environment variable.
    out = subprocess.check_output(
        "find {} -name *.so -o -name *.so.*".format(start_dir).split())
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
