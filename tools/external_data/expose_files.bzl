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

def expose_files(sub_packages = [], sub_dirs = []):
    """
    Declares files to be consumed externally (for Bazel workspace tests, linting, etc).

    @param sub_packages Child packages, only the first level.
    @param sub_dirs Any directories that are not packages.
    """
    # @note It'd be nice if this could respect *ignore files, but meh.
    # Also, it'd be **super** nice if Bazel did not let `**` globs leak into other
    # packages and then error out.
    package_prefix = native.package_name() + "/"
    for name, patterns in patterns_map.items():
        files = native.glob(pattern)
        for sub_dir in sub_dirs:
            files += native.glob([subdir + "/" + item for item in pattern])
        deps = [package_prefix + sub_package + ":" + name for sub_package in sub_packages]
        native.filegroup(
            name = name,
            srcs = native.glob(
            visibility = ["//visibility:public"],
            deps = deps,
        )
