# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` just concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param cmd
        Command to run. By default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    """

    anchor = name + "_anchor"
    all_files = name + "_all_files"
    args = [cmd, "$(location {})".format(anchor)]
    # Pass all all_files to be copied.
    data_out = [all_files] + data
    for datum in data_out:
        args.append("$(locations {})".format(datum))
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = ["@drake//tools/external_data:workspace_writeable_test.sh"],
        args = args,
        data = [anchor] + data_out,
    )
