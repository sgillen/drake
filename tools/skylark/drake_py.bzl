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

def _py_exec_impl(ctx):
    data = ctx.attr.data + [ctx.attr.shim]
    files = depset()
    for d in data:
        files += d.data_runfiles.files
    info = dict(
        shim_basename = ctx.executable.shim.basename,
        workspace_name = ctx.workspace_name,
        embed_args = [
            ctx.expand_location(arg, data) for arg in ctx.attr.embed_args
        ],
        add_library_paths = [
            ctx.expand_location(p, data) for p in ctx.attr.add_library_paths
        ],
        add_py_paths = [
            ctx.expand_location(p, data) for p in ctx.attr.add_py_paths
        ],
    )
    content = """#!/usr/bin/env python
import os
import subprocess
import sys

shim_path = os.path.dirname(__file__) + "/{shim_basename}"
runfiles_dir = os.getcwd()
runfiles_suffix = ".runfiles/{workspace_name}"
if not runfiles_dir.endswith(runfiles_suffix):
    runfiles_dir = __file__ + runfiles_suffix

def add_paths(env, paths):
    abspaths = [os.path.join(runfiles_dir, p) for p in paths]
    os.environ[env] = ":".join(abspaths) + ":" + os.environ.get(env, '')

library_path = (sys.platform.startswith("linux") and
    "LD_LIBRARY_PATH" or "DYLD_LIBRARY_PATH")
add_paths(library_path, {add_library_paths})
add_paths("PYTHONPATH", {add_py_paths})

args = [shim_path] + {embed_args}
subprocess.check_call(args + sys.argv[1:])
""".format(**info)
    ctx.file_action(
        output=ctx.outputs.executable,
        content=content,
        executable=True)
    return [DefaultInfo(
        runfiles=ctx.runfiles(files=list(files))
    )]

# Embeds arguments in a script, that can be run via `bazel run` or `bazel-bin`.
_py_exec = rule(
    implementation=_py_exec_impl,
    executable=True,
    attrs={
        "shim": attr.label(cfg="target", executable=True),
        "embed_args": attr.string_list(),
        "data": attr.label_list(cfg="data", allow_files=True),
        "add_library_paths": attr.string_list(),
        "add_py_paths": attr.string_list(),
    },
)

def drake_py_exec(
        name,
        executable,
        add_library_paths = [],
        add_py_paths = [],
        py_deps = [],
        data = [],
        args = [],
        **kwargs):
    """Runs an arbitrary command within a Bazel Python environment. """
    py_main = "//tools/skylark:py_env_runner.py"
    if "deps" in kwargs:
        fail("Use `py_deps` instead of `deps` to avoid ambiguity.")
    shim = name + ".shim"
    drake_py_binary(
        name = shim,
        srcs = [py_main],
        main = py_main,
        deps = py_deps,
        data = data,
        **kwargs
    )
    # Embed arguments into a script.
    embed_args = ["$(location {})".format(executable)] + args
    _py_exec(
        name = name,
        embed_args = embed_args,
        shim = shim,
        data = data + [executable],
        add_library_paths = add_library_paths,
        add_py_paths = add_py_paths,
    )
