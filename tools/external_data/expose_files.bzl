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

def _filegroup_recursive_impl(ctx):
    files = depset()
    for d in ctx.attr.data:
        runfiles = d.data_runfiles.files
        files += runfiles
    return [DefaultInfo(
        files=files,
    )]

"""
Provides all files (including `data` dependencies) such that they are
expandable via `$(locations ...)`.
"""
_filegroup_recursive = rule(
    implementation = _filegroup_recursive_impl,
    attrs = {
        "data": attr.label_list(cfg = "data", allow_files = True),
    },
)

def expose_files(sub_packages = [], sub_dirs = []):
    """
    Declares files to be consumed externally (for Bazel workspace tests, linting, etc).

    @param sub_packages Child packages, only the first level.
    @param sub_dirs Any directories that are not packages.
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
            srcs += native.glob([sub_dir + "/" + pattern for pattern in patterns])
        deps = [package_prefix + sub_package + ":" + name for sub_package in sub_packages]
        native.filegroup(
            name = name,
            srcs = srcs,
            # This permits all files to be available in `runfiles`; however, it
            # does not expose the files for location expansion.
            data = deps,
            visibility = ["//visibility:public"],
        )
        # Rely on existing `data` declaration above.
        _filegroup_recursive(
            name = name + "_recursive",
            data = [name],
        )

    # TODO(eric.cousineau): Is there a way to avoid this?
    ws = native.glob(["WORKSPACE"])
    if ws:
        native.exports_files(
            srcs = ws,
            visibility = ["//visibility:public"],
        )
