def cc_shared_library(
        name,
        copts = [],
        hdrs = [],
        srcs = [],
        deps = [],
        includes = [],
        **kwargs):
    """Creates a shared library."""
    solib = "lib" + name + ".so"
    hdrlib = "_" + name + "_hdrs"

    native.cc_library(
        name = hdrlib,
        hdrs = hdrs,
        deps = deps,
        includes = includes,
        **kwargs
    )

    deps = [hdrlib]

    # Create main shared library.
    native.cc_binary(
        name = solib,
        srcs = srcs,
        linkshared = 1,
        linkstatic = 1,
        deps = deps,
        **kwargs
    )

    # Expose shared library and headers for transitive dependencies.
    native.cc_library(
        name = name,
        hdrs = hdrs,
        srcs = [solib],
        deps = deps,
        **kwargs
    )
