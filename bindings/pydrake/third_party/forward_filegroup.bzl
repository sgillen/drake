load("//tools/skylark:pathutils.bzl", "dirname", "basename")

# def _impl(ctx):
#     files = ctx.attr.srcs
#     return [DefaultInfo(
#         files = files,
#     )]

# forward_files = rule(
#     attrs = {
#         "srcs": attr.label_list(
#             cfg = "data",
#             allow_files = True,
#         ),
#         "dest": attr.string(mandatory = True),
#     },
#     implementation = _impl,
# )

def forward_files(name, srcs, strip_prefix, dest_prefix, visibility = None):
    # Requires at least one file.
    # Assume we have direct files.
    cur_dir = dirname(srcs[0])
    outs = []
    for src in srcs:
        if not src.startswith(strip_prefix):
            fail("Files must be under the same directory: {} not under {}"
                 .format(src, strip_prefix))
        out = dest_prefix + src[len(strip_prefix):]
        native.genrule(
            name = out,
            srcs = [src],
            outs = [out],
            cmd = "ln -s $< $@",
            visibility = visibility,
        )
        outs.append(out)
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
