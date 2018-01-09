# -*- python -*-

# Default command. Do not run linting tests, so that the downstream test
# packages do not need to consume `drake`.
# Needs quotes, because `sh_test(args = [...])` just concatenates them???
_CMD_DEFAULT = "'bazel test //...'"

def _get_target(name):
    return "@bazel_external_data_pkg//test:" + name

def workspace_test(
        name,
        workspace,
        cmd = _CMD_DEFAULT,
        data = [],
        writeable = 0):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param workspace
        Workspace directory relative to this package.
    @param cmd
        Command to run. By default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    @param writeable
        If the data should be copied such that modifications can be made. """

    if writeable:
        args = [cmd, "$(location {})".format(workspace)]
        for datum in data:
            args.append("$(locations {})".format(datum))
        native.sh_test(
            name = name,
            srcs = [_get_target("workspace_writeable_test.sh")],
            args = args,
            data = [workspace] + data,
        )
    else:
        # Use a symlink forest, so `bazel run` of `workspace_test` will not
        # pollute the source directory with  `bazel-*` symlinks.
        workspace_filegroup = name + "_filegroup"
        # We must use a file, and not just `workspace`, because Bazel will not
        # give us the location unless it's in `data`. We do NOT want this,
        # because we want a symlink forest. (See above.)
        workspace_file = workspace + "/WORKSPACE"
        native.filegroup(
            name = workspace_filegroup,
            srcs = native.glob(
                [workspace + "/*"],
                exclude_directories = 0,
            ),
        )
        native.sh_test(
            name = name,
            srcs = [_get_target("workspace_test.sh")],
            args = [
                cmd,
                "$(location {})".format(workspace_file)],
            data = [
                # Must explicitly list this file so that we can get its
                # location.
                workspace_file,
                workspace_filegroup,
            ] + data,
        )
