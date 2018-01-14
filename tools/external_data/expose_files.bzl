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
            data = deps,
            visibility = ["//visibility:public"],
        )
        # TODO(eric.cousineau): Is there a way to avoid this?
        native.exports_files(
            srcs = native.glob(["WORKSPACE"]),
            visibility = ["//visibility:public"],
        )
