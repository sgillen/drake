# -*- python -*-

def _drake_sh_test_impl(ctx):
    # `sh_test` is sensitive about `$(location ...)` expansion, especially for
    # Python targets. This permits performing a `sh_test` on a Python binary.
    target_relpath = ctx.executable.target.short_path
    if target_relpath.startswith("../"):
        fail("External targets not supported")
    content = """#!/bin/bash
set -e -u -o pipefail
exec "{target_relpath}" "$@"
""".format(target_relpath=target_relpath)
    ctx.actions.write(
        output = ctx.outputs.executable,
        content = content,
        is_executable = True,
    )
    return [DefaultInfo(
        runfiles = ctx.runfiles(
            # Inherit `target`s runfiles.
            files = ctx.attr.target.data_runfiles.files.to_list(),
        ),
    )]

drake_sh_test = rule(
    attrs = {
        "target": attr.label(
            cfg = "target",
            executable = True,
            mandatory = True,
        ),
    },
    executable = True,
    impelmentation = _drake_sh_test_impl,
)
