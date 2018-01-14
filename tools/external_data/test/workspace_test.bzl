# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        cmd = _CMD_DEFAULT,
        args = [],
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project in a writeable (copied) context.
    """
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = [":workspace_test.sh"],
        args = [cmd] + args,
        data = data,
    )
