# -*- python -*-

load("//tools/skylark:6996.bzl", "adjust_labels_for_drake_hoist")

def drake_py_library(
        name,
        deps = None,
        data = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    native.py_library(
        name = name,
        deps = deps,
        data = data,
        **kwargs)

def drake_py_binary(
        name,
        deps = None,
        data = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    native.py_binary(
        name = name,
        deps = deps,
        data = data,
        **kwargs)

def drake_py_test(
        name,
        srcs = None,
        deps = None,
        data = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    if srcs == None:
        srcs = ["test/%s.py" % name]
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    native.py_test(
        name = name,
        srcs = srcs,
        deps = deps,
        data = data,
        **kwargs)

def _exec_impl(ctx):
    data = ctx.attr.data + [ctx.attr.shim, ctx.attr.executable]
    print(dir(ctx.attr.shim))
    print(ctx.attr.shim.label)
    print(ctx.executable.executable.path)
    info = dict(
        shim_relpath = ctx.executable.shim.short_path,
        exec_relpath = ctx.executable.executable.short_path,
        add_library_paths = [
            ctx.expand_location(p, data) for p in ctx.attr.add_library_paths],
        add_py_paths = [
            ctx.expand_location(p, data) for p in ctx.attr.add_py_paths],
    )
    content = """#!/usr/bin/env python
import os
import subprocess
import sys

# Ensure that we run from `runfiles`.
# This assumes that this script neighbors `exec`.
shim_relpath = "{shim_relpath}"
runfiles_dir = os.getcwd()
print(runfiles_dir)
if not os.path.dirname(runfiles_dir).endswith(".runfiles"):
    script = os.path.abspath(__file__)
    runfiles_dir = script + ".runfiles"
    assert os.path.exists(runfiles_dir)
    runfiles_relpath = os.path.relpath(".", os.path.dirname(shim_relpath))
    script_dir = os.path.dirname(__file__)
    runfiles_dir = os.path.abspath(os.path.join(script_dir, runfiles_relpath))
shim_path = os.path.join(runfiles_dir, shim_relpath)
print(runfiles_dir)
print("{exec_relpath}")
exec_path = os.path.join(runfiles_dir, "{exec_relpath}")

def _add_paths(env, paths):
    abspaths = [os.path.join(runfiles_dir, p) for p in paths]
    os.environ[env] = ":".join(abspaths) + ":" + os.environ.get(env, '')

# Add library paths.
env = (sys.platform.startswith("linux") and
    "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
_add_paths(env, {add_library_paths})

# Add Python paths.
_add_paths("PYTHONPATH", {add_py_paths})

# Execute.
args = [shim_path, exec_path] + sys.argv[1:]
print(args)
subprocess.check_call(args)
""".format(**info)
    print(content)
    # Collect runfiles.
    files = depset()
    for d in data:
        files += d.data_runfiles.files
    ctx.file_action(
        output=ctx.outputs.executable,
        content=content,
        executable=True)
    return [DefaultInfo(
        runfiles=ctx.runfiles(files=list(files))
    )]

# Embeds arguments in a script, that can be run via `bazel-bin`, preserving PWD.
_exec = rule(
    implementation=_exec_impl,
    executable=True,
    attrs={
        "shim": attr.label(cfg="data"),
        "executable": attr.label(cfg="data", allow_files=True),
        # "embed_args": attr.label_list(),
        "data": attr.label_list(cfg="data", allow_files=True),
        "add_library_paths": attr.string_list(),
        "add_py_paths": attr.string_list(),
    },
)

def drake_exec(
        name,
        executable,
        add_library_paths = [],
        add_py_paths = [],
        py_deps = [],
        data = [],
        **kwargs):
    """Runs an arbitrary command within a Bazel Python environment. """
    py_main = "//tools/skylark:py_env_runner.py"
    if "deps" in kwargs:
        fail("Use `py_deps`.")
    impl = name + ".impl"
    drake_py_binary(
        name = impl,
        srcs = [py_main],
        main = py_main,
        deps = py_deps,
        data = data,
        **kwargs
    )
    # Encode arguments into a script.
    _exec(
        name = name,
        shim = impl,
        executable = executable,
        add_library_paths = add_library_paths,
        add_py_paths = add_py_paths,
        data = data,
    )
