load("//tools/workspace:generate_file.bzl", "generate_file")
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_test",
)

# Generate file, because we wish to bake the file directly in, and not require
# it be passed as an argument.
_JUPYTER_PY_TEMPLATE = """
import os
import sys

from jupyter_bazel import _jupyter_bazel_notebook_main

cur_dir = os.path.dirname(__file__)
notebook = {notebook}
_jupyter_bazel_notebook_main(cur_dir, notebook, sys.argv[1:])
""".lstrip()

def jupyter_py_binary(
        name,
        notebook = None,
        data = [],
        deps = [],
        add_test_rule = 0,
        tags = [],
        test_timeout = None,
        test_flaky = 0,
        _rule_binary = drake_py_binary,
        _rule_test = drake_py_test,
        **kwargs):
    """Creates a target to run a Jupyter notebook.

    Please see `//tools/jupyter:README.md` for examples.

    @param notebook
        Notebook file to use. Be default, will be `{name}.ipynb`.
    """
    if notebook == None:
        notebook = name + ".ipynb"
    main = "{}_jupyter_py_main.py".format(name)
    tags = tags + ["jupyter"]
    generate_file(
        name = main,
        content = _JUPYTER_PY_TEMPLATE.format(notebook = repr(notebook)),
        is_executable = False,
    )
    _rule_binary(
        name = name,
        srcs = [main],
        main = main,
        data = data + [notebook],
        deps = deps + [
            "//tools/jupyter:jupyter_bazel_py",
        ],
        # `generate_file` output is still marked as executable :(
        tags = tags + ["nolint"],
        **kwargs
    )
    if add_test_rule:
        target = ":{}".format(name)
        _rule_test(
            name = "{}_test".format(name),
            args = ["--test"],
            main = main,
            srcs = [main],
            deps = [target],
            tags = tags + ["nolint"],
            timeout = test_timeout,
            flaky = test_flaky,
            **kwargs
        )
