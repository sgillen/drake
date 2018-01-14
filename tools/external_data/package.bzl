test_workspaces = ["bazel_pkg_test"]

def _name(workspace):
    return "external_data_test__" + workspace

def add_workspace_files():
    # To be consumed by `workspace_test`.
    for workspace in test_workspaces:
        native.filegroup(
            name = workspace + "_files",
            srcs = "@" + _name + "//:all_files",
        )
        native.alias(
            name = workspace + "_anchor",
            actual = "@" + _name + "//:WORKSPACE",
        )

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
    test_base_dir = "tools/external_data/test/workspaces"
    for workspace in test_workspaces:
        native.local_repository(
            name = _name(workspace),
            path = test_base_dir + "/" + workspace,
        )
