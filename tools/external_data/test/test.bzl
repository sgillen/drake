load("//tools/external_data:expose_files.bzl", "patterns_map", "filegroup_recursive")

_workspace_list = ["bazel_pkg_test"]

def _workspace_name(workspace):
    return "external_data_" + workspace

def import_workspace_files():
    # To be consumed by `workspace_test`.
    cur = dict(
        all_files = ["//tools/external_data:all_files"],
        bazel_lint_files = ["//tools/external_data:bazel_lint_files"],
        python_lint_files = ["//tools/external_data:python_lint_files"],
    )
    for workspace in _workspace_list:
        prefix = "@" + _workspace_name(workspace) + "//"
        # Alias in `all_files` groups (non-recursive).
        native.alias(
            name = workspace + "_all_files",
            actual = prefix + ":all_files",
        )
        # Expose anchor.
        native.alias(
            name = workspace + "_anchor",
            actual = prefix + ":WORKSPACE",
        )
        # Add list for data to be stubbed.
        for name in patterns_map.keys():
            cur[name].append(prefix + ":" + name)
    # Create final consumption.
    dummy = ":EMPTY"
    for name in patterns_map.keys():
        filegroup_recursive(
            name = "external_data_" + name,
            data = cur[name],
            dummy = dummy,
        )

def get_workspace_files():
    workspace_files = dict()
    for name in patterns_map.keys():
        cur = []
        for workspace in _workspace_list:
            cur.append(workspace + "_" + name + "_recursive")
        workspace_files[name] = cur
    return workspace_files

def external_data_test_repositories(workspace_dir):    
    # Ignores any targets under this directory so that `test ...` will not leak
    # into them.
    # WARNING: Bazel also craps out here if `__workspace_dir__ + path` is used
    # rather than just `path`.
    # N.B. This error is *stateful*. You will get different behavior depending on
    # what has been built / run previously in Bazel. In one mode, the error
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
