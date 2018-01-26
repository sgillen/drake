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
    data = ctx.attr.data + [ctx.attr.cmd]
    files = depset()
    for d in data:
        files += d.data_runfiles.files
    # @note We use `short_path` here because `$(locations {impl})` returns
    # two files...
    embed_args = [
        "--runfiles_relpath={}".format(ctx.executable.cmd.short_path),
    ] + ctx.attr.embed_args
    info = dict(
        relpath = ctx.executable.cmd.basename,
        embed_args = ctx.expand_location(" ".join(embed_args), data),
    )
    content = "$(dirname $0)/{relpath} {embed_args} \"$@\"".format(**info)
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
    impl = name + ".impl"
    drake_py_binary(
        name = impl,
        srcs = [py_main],
        main = py_main,
        deps = py_deps,
        data = data,
        **kwargs
    )
    embed_args = ["--add_library_path={}".format(p) for p in add_library_paths]
    embed_args += ["--add_py_path={}".format(p) for p in add_py_paths]
    embed_args += ["$(location {})".format(executable)] + args
    # Encode arguments into a script.
    _exec(
        name = name,
        cmd = impl,
        embed_args = embed_args,
        data = data + [executable],
    )
