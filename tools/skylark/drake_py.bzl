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
    files = ctx.attr.cmd.data_runfiles.files
    for d in ctx.attr.data:
        files += d.data_runfiles.files
    args_raw = [
        "--runfiles_relpath={}".format(ctx.executable.cmd.short_path),
    ] + ctx.attr.embed_args
    args = ctx.expand_location(" ".join(args_raw), ctx.attr.data)
    relpath = ctx.executable.cmd.basename
    content = "$(dirname $0)/{} {} \"$@\"".format(relpath, args)
    ctx.file_action(
        output=ctx.outputs.executable,
        content=content,
        executable=True)
    return [DefaultInfo(
        runfiles=ctx.runfiles(files=list(files))
    )]

# Embeds arguments in a script, that can be run via `bazel-bin`.
_exec = rule(
    implementation=_exec_impl,
    executable=True,
    attrs={
        "cmd": attr.label(cfg="target", executable=True),
        "embed_args": attr.string_list(),
        "data": attr.label_list(cfg="data", allow_files=True),
    },
)

def drake_py_exec(
        name,
        args,
        add_library_paths = [],
        add_py_paths = [],
        py_deps = [],
        data = [],
        **kwargs):
    """Runs an arbitrary command within a Bazel Python environment. """
    py_main = "//tools/skylark:py_env_runner.py"
    if "deps" in kwargs:
        fail("Use `py_deps` instead of `deps` to avoid ambiguity.")
    args = list(args)
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
        cmd = impl,
        embed_args = args,
        data = data,
    )
