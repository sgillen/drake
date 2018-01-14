load(":expose_files.bzl", "patterns_map")

test_workspaces = ["bazel_pkg_test"]

def _name(workspace):
    return "external_data_test__" + workspace

def add_workspace_set_files():
    # To be consumed by `workspace_test`.
    for workspace in test_workspaces:
        prefix = "@" + _name(workspace) + "//"
        # Alias in `expose_files` file groups.
        for name in patterns_map.keys():
            native.alias(
                name = workspace + "_" + name,
                actual = prefix + ":" + name,
            )
        # Expose anchor.
        native.alias(
            name = workspace + "_anchor",
            actual = prefix + ":WORKSPACE",
        )

def _get_workspace_set_files():
    workspace_set = dict()
    for name in patterns_map.keys():
        cur = []
        for workspace in test_workspaces:
            cur.append(workspace + "_" + name)
        workspace_set[name] = cur
    return workspace_set

workspace_set = _get_workspace_set_files()

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
    for workspace in test_workspaces:
        native.local_repository(
            name = _name(workspace),
            path = test_base_dir + "/" + workspace,
        )
