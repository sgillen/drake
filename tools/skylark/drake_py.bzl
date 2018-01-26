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

def drake_py_exec(
        name,
        args,
        py_deps = [],
        **kwargs):
    """Runs an arbitrary command within a Bazel Python environment. """
    py_main = "//tools/skylark:py_env_runner.py"
    if "deps" in kwargs:
        fail("Use `py_deps` instead of `deps` to avoid ambiguity.")
    # TODO(eric.cousineau): Is there a better type check for Skylark?
    args = list(args)
    if not args:
        fail("`args` must be a list with at least one item (the binary).")
    drake_py_binary(
        name = name,
        srcs = [py_main],
        main = py_main,
        args = args,
        deps = py_deps,
        **kwargs
    )
