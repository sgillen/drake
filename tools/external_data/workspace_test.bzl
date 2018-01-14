# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` just concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"
_REMAP_DEFAULT = struct(
    orig = "external/",
    new = "tools/external_data/test/",
)

def workspace_test(
        name,
        workspace,
        remap = _REMAP_DEFAULT,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param workspace
        Workspace directory relative to this package.
    @param remap
        struct(from, to) Remap path
    @param cmd
        Command to run. By default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    """

    workspace_anchor = workspace + "_anchor"
    workspace_files = workspace + "_files"
    args = [
        cmd,
        "--anchor", "$(location {})".format(workspace_anchor),
    ]
    if remap:
        args += ["--remap", remap.orig, remap.new]
    data_out = [workspace_files] + data
    for datum in data:
        args.append("$(locations {})".format(datum))
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = ["@drake//tools/external_data:workspace_writeable_test.sh"],
        args = args,
        data = data,
    )
