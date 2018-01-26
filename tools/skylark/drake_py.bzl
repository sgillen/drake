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
    # Generate bash script with arguments embedded.
    executable = ctx.outputs.executable
    command = ctx.expand_location(" ".join(ctx.attr.embed_args), ctx.attr.data)
    ctx.file_action(
        output=executable,
        content=command + " \"$@\"",
        executable=True)
    return [DefaultInfo(
        runfiles=ctx.runfiles(files=ctx.files.data)
    )]

# Embeds arguments in a script.
_exec = rule(
    implementation=_exec_impl,
    executable=True,
    attrs={
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
    # TODO(eric.cousineau): Is there a better type check for Skylark?
    args_final = []
    args = list(args)
    if not args:
        fail("`args` must be a list with at least one item (the binary).")
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
        embed_args = ["python $(location {})".format(py_main)] + args,
        data = [py_main, impl] + data,
    )
