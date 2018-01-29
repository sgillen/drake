
def forward_files(srcs, strip_prefix, dest_prefix, visibility = None):
    # Assumes we have direct files.
    outs = []
    for src in srcs:
        if not src.startswith(strip_prefix):
            fail("'{}' not under '{}'".format(src, strip_prefix))
        out = dest_prefix + src[len(strip_prefix):]
        native.genrule(
            name = out + ".forward",
            srcs = [src],
            outs = [out],
            cmd = "cp $< $@",
            visibility = visibility,
        )
        outs.append(out)
