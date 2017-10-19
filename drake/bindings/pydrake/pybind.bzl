# -*- python -*-

load("//tools:drake.bzl", "drake_cc_binary", "drake_py_library")

load(
    "//tools:pathutils.bzl",
    "dirname",
    "basename",
    "join_paths",
)

is_devel = False
# TODO: Figure out how to get a relative path here...
DEFAULT_IMPORT = [".."]
SO_FMT = '_{}.so'

# TODO(eric.cousineau): Make a `PybindProvider`, so that we can do a
# better job of tracking transitives, and can do a better job of figuring out what
# our sources, dependencies, etc., actually are.
# Right now, we lose all of this information and have to duplicate it...
# This can also simplify the sorting of `cc_deps` and `cc_devel_deps`; if a dependency
# is header-only and if it's under `//bindings`, then we should consider it a necessary
# dependency.

# TODO(eric.cousineau): Discard `is_devel` once #7294 is satisfied, and is sufficiently
# convenient for developers.

def drake_pybind_library(name,
                         cc_srcs = [],
                         cc_deps = [], cc_devel_deps = [], copts = [],
                         py_srcs = [], py_deps = [],
                         py_imports = DEFAULT_IMPORT,
                         **kwargs):
    """Declare a pybind11 shared library with the given name and srcs.  The
    libdrake.so library and its headers are already automatically depended-on
    by this rule.

    The deps, linkshared, and linkstatic parameters cannot be set by the
    caller; this rule must fully control their values.
    """

    # Disallow `linkshared` and `linkstatic` because Bazel requires them to be
    # set to 1 when making a ".so" library.
    #
    # Disallow `deps` because we _must not_ deps on any given object code more
    # than once or else we risk linking in multiple copies of it into different
    # _pybind_foo.so files, which breaks C++ global variables.  All object code
    # must come in through libdrake.so.  (Conceivably a header-only library
    # could be allowed in deps, but we can fix that when we need it.)
    for key in ["deps", "linkshared", "linkstatic"]:
        if key in kwargs:
            fail("%s cannot be set by the caller" % key)

    # TODO(eric.cousineau): Is there a way to check a dependency's target type?

    # These copts are per pybind11 deficiencies.
    copts_pybind11 = [
        "-Wno-#warnings",
        "-Wno-cpp",
        "-Wno-unknown-warning-option",
    ]

    cc_so = join_paths(dirname(name), SO_FMT.format(basename(name)))
    py_name = name

    if not is_devel:
        drake_cc_binary(
            name = cc_so,
            # This is how you tell Bazel to link in a shared library.
            srcs = cc_srcs + ["//drake:libdrake.so"],
            copts = copts_pybind11 + copts,
            # This is how you tell Bazel to create a shared library.
            linkshared = 1,
            # TODO(eric.cousineau): I believe the upstream linking will be static, but not
            # yet sure...
            linkstatic = 0,
            # For all pydrake_foo.so, always link to Drake and pybind11.
            deps = cc_deps + [
                # Even though "libdrake.so" appears in srcs above, we have to list
                # :drake_shared_library here in order to get its headers onto the
                # include path, and its prerequisite *.so's onto LD_LIBRARY_PATH.
                "//drake:drake_shared_library",
                "@pybind11",
                # TODO(jwnimmer-tri) We should be getting stx header path from
                # :drake_shared_library, but that isn't working yet.
                "@stx",
            ],
            **kwargs
        )
    else:
        drake_cc_binary(
            name = cc_so,
            copts = copts_pybind11 + copts,
            srcs = cc_srcs,
            deps = cc_deps + cc_devel_deps + [
                "@pybind11",
            ],
            linkshared = 1,
            linkstatic = 0,
            **kwargs
        )

    # Add Python library.
    drake_py_library(
        name = py_name,
        data = [cc_so],
        srcs = py_srcs,
        deps = py_deps,
        imports = py_imports,
    )

    # TODO: Expose *.so as an installable target, preferrably with some sort transitive_libs
    # setup.
