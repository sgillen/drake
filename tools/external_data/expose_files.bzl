patterns_map = dict(
    all_files = [
        "*",
    ],
    bazel_lint_files = [
        "BUILD.bazel",
        "WORKSPACE",
        "*.bzl",
    ],
    python_lint_files = [
        "*.py",
    ],
)

def _recursive_filegroup_impl(ctx):
    files = depset()
    for d in ctx.attr.data:
        files += d.data_runfiles.files
    if ctx.attr.dummy and not files:
        # Expand to avoid error of empty "$(locations ...)" expansion:
        # https://github.com/bazelbuild/bazel/blob/c3bedec/src/main/java/com/google/devtools/build/lib/analysis/LocationExpander.java#L273  # noqa
        files = [ctx.attr.dummy]
    return [DefaultInfo(
        files = files,
        data_runfiles = ctx.runfiles(
            files = list(files),
        ),
    )]

"""
Provides all files (including `data` dependencies) such that they are
expandable via `$(locations ...)`.
"""

recursive_filegroup = rule(
    attrs = {
        "data": attr.label_list(
            cfg = "data",
            allow_files = True,
        ),
        "dummy": attr.label(allow_single_file = True),
    },
    implementation = _recursive_filegroup_impl,
)

def _prefix_list(prefix, items):
    return [prefix + item for item in items]

def expose_files(sub_packages = [], sub_dirs = []):
    """
    Declares files to be consumed externally (for Bazel workspace tests,
    linting, etc).
    Creates "${type}_files" and "${type}_files_recursive", where `type` can be
    one of {"all", "bazel_lint", "python_lint"}.

    @param sub_packages
        Child packages, only the first level.
    @param sub_dirs
        Any directories that are not packages.
    """
    # @note It'd be nice if this could respect *ignore files, but meh.
    # Also, it'd be **super** nice if Bazel did not let `**` globs leak into other
    # packages and then error out.
    package_name = native.package_name()
    if package_name:
        package_prefix = "//" + package_name + "/"
    else:
        package_prefix = "//"  # Root case.
    for name, patterns in patterns_map.items():
        srcs = native.glob(patterns)
        for sub_dir in sub_dirs:
            srcs += native.glob(_prefix_list(sub_dir + "/", patterns))
        native.filegroup(
            name = name,
            srcs = srcs,
            # Trying to use `data = deps` here only exposes the files in
            # runfiles, but not for expansion via `$(locations...)`.
            visibility = ["//visibility:public"],
        )
        # Expose all files at one level.
        deps = [
            package_prefix + sub_package + ":" + name
                for sub_package in sub_packages]
        recursive_filegroup(
            name = name + "_recursive",
            data = [name] + deps,
        )
