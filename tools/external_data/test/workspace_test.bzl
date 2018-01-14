# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        anchor,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param anchor
        File that is used to dictate directory location.
    @param cmd
        Command to run. Default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    """
    args = [cmd, "$(location {})".format(anchor)]
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = [":workspace_writeable_test.sh"],
        args = args,
        data = [anchor] + data,
    )
