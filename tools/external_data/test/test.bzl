load(
    "//tools/external_data:expose_files.bzl",
    "patterns_map",
    "filegroup_recursive",
)

_workspace_list = ["bazel_pkg_test"]

def _workspace_name(workspace):
    return "external_data_" + workspace

def import_workspace_files():
    # To be consumed by `workspace_test`.
    packages = ["//tools/external_data"]
    for workspace in _workspace_list:
        package = "@" + _workspace_name(workspace) + "//"
        # Alias in `all_files` groups (non-recursive).
        native.alias(
            name = workspace + "_all_files_recursive",
            actual = package + ":all_files_recursive",
        )
        # Expose anchor.
        native.alias(
            name = workspace + "_anchor",
            actual = package + ":WORKSPACE",
        )
        # Prepare to expose all files recursively.
        packages.append(package)
    # Expose files for consuming for linting.
    # This is done to (a) avoid the following constraint:
    # https://github.com/bazelbuild/bazel/blob/c3bedec/src/main/java/com/google/devtools/build/lib/analysis/LocationExpander.java#L273  # noqa
    # and (b) to permit `$(locations ...)` expansion.
    dummy = ":EMPTY"
    for name in patterns_map.keys():
        data = [package + ":" + name + "_recursive" for package in packages]
        filegroup_recursive(
            name = "external_data_" + name,
            data = data,
            dummy = dummy,
        )

def external_data_test_repositories(workspace_dir):
    # Ignores any targets under this directory so that `test ...` will not leak
    # into them.
    # WARNING: Bazel also craps out here if `__workspace_dir__ + path` is used
    # rather than just `path`.
    # N.B. This error is *stateful*. You will get different behavior depending
    # on what has been built / run previously in Bazel. In one mode, the error
    # will be:
    #   Encountered error while [...]
    #   /home/${USER}/.cache/bazel/_bazel_${USER}/${HASH}/external/bazel_external_data_pkg  # noqa
    #   must  be an existing directory
    # In another mode, you will get Java errors:
    #   java.lang.IllegalArgumentException: PathFragment
    #   tools/external_data/workspace is not beneath
    #   /home/${USER}/${WORKSPACE_DIR}/tools/external_data/workspace
    test_base_dir = "tools/external_data/test"
    for workspace in _workspace_list:
        native.local_repository(
            name = _workspace_name(workspace),
            path = test_base_dir + "/" + workspace,
        )
