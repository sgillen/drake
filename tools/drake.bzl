# -*- python -*-

# TODO(6996) Once the dust settles down, remove this convenience forwarding,
# for consistency with the other drake_{py,java,...} languages.
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)


# Without this, we still install a `*.so` if we do not explicitly set `linkstatic`.
# On top of that, `libdrake.so` doesn't actually consume this, so we needlessly install it.
enable_experimental_shared_lib = True

def drake_shared_cc_library(
        name,
        binary_name = None,
        hdrs = None,
        srcs = None,
        deps = [],
        so_fmt = "lib{}.so",
        **kwargs):
    """Creates a rule to declare C++ library for external and internal consumption

    @see https://github.com/bazelbuild/bazel/issues/492
    """
    # Create header-only library.
    if enable_experimental_shared_lib:
        so_lib = so_fmt.format(name)

        # Create binary.
        native.cc_binary(
            name = so_lib,
            srcs = srcs + hdrs,
            deps = deps,
            linkshared = 1,
            **kwargs)

        # Create internally-consumable external. Make sure this links statically, and use
        # the *.so to enforce ODR safety.
        native.cc_library(
            name = name,
            hdrs = hdrs,
            srcs = [
                so_lib,
            ],
            deps = deps,
            linkstatic = 1,
            **kwargs)
    else:
        native.cc_library(
            name = name,
            hdrs = hdrs,
            srcs = srcs,
            deps = deps,
            linkstatic = 1,
            **kwargs)

def drake_shared_cc_library_install_targets(name, so_fmt = "lib{}.so", prefix = ":"):
    if enable_experimental_shared_lib:
        return [prefix + so_fmt.format(name)]
    else:
        return [prefix + name]
