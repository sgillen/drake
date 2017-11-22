# -*- python -*-

load("//tools:drake.bzl", "drake_cc_binary")

def _drake_pybind_cc_binary(name, srcs = [], copts = [], **kwargs):
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

    drake_cc_binary(
        name = name,
        # This is how you tell Bazel to link in a shared library.
        srcs = srcs + ["//tools/install/libdrake:libdrake.so"],
        # These copts are per pybind11 deficiencies.
        copts = [
            "-Wno-#warnings",
            "-Wno-cpp",
            "-Wno-unknown-warning-option",
        ] + copts,
        # This is how you tell Bazel to create a shared library.
        linkshared = 1,
        linkstatic = 1,
        # For all pydrake_foo.so, always link to Drake and pybind11.
        deps = [
            # Even though "libdrake.so" appears in srcs above, we have to list
            # :drake_shared_library here in order to get its headers onto the
            # include path, and its prerequisite *.so's onto LD_LIBRARY_PATH.
            "//tools/install/libdrake:drake_shared_library",
            "@pybind11",
            # TODO(jwnimmer-tri) We should be getting stx header path from
            # :drake_shared_library, but that isn't working yet.
            "@stx",
        ],
        **kwargs
    )

PY_IMPORTS_DEFAULT = ["drake/bindings"]
PY_VERSION = "2.7"

# TODO(eric.cousineau): Consider making a `PybindProvider`, to sort
# out dependencies, sources, etc.

def drake_pybind_library(name,
                         cc_srcs = [],
                         cc_deps = [], cc_devel_deps = [], copts = [],
                         cc_so_name = None,
                         py_srcs = [], py_deps = [],
                         py_imports = PY_IMPORTS_DEFAULT,
                         py_pkg_install = None,
                         **kwargs):
    """ Declare a pybind11 library, with C++ base code and Python interface code.

    @param cc_srcs
        C++ source files.
    @param cc_deps (optional)
        C++ dependencies.
        At present, these should be header only, as they will violate ODR with
        statically-linked libraries.
    @param cc_devel_deps (optional)
        C++ development dependencies.
        At present, these are no-op, as `libdrake.so` (and the relevant headers) are used.
        (In the future, `cc_devel_deps` and `cc_deps` will be combined.
    @param cc_so_name (optional)
        Shared object name. By default, this is `_${name}`, so that the C++ code
        can be then imported in a more controlled fashion in Python.
        If overridden, this could be the public interface exposed to the user.
    @param py_srcs
        Python sources.
    @param py_deps
        Python dependencies.
    @param py_imports
        Python import directories.
        This tells `py_library` how to structure the Python directories so we can
        type "import pydrake.${module}".
    @param py_pkg_install
        Installation location, relative to "{LIB_DIR}/python{PY_VER}/site-packages".
    """

    # TODO(eric.cousineau): Is there a way to check a dependency's target type?

    # These copts are per pybind11 deficiencies.
    copts_pybind11 = [
        "-Wno-#warnings",
        "-Wno-cpp",
        "-Wno-unknown-warning-option",
    ]

    install_name = name + "_install"

    if not py_name:
        py_name = "_" + name
    cc_so = py_name + ".so"
    # TODO(eric.cousineau): Ensure `cc_deps` is header-only.
    _drake_pybind_cc_binary(
        name = cc_so,
        srcs = cc_srcs,
        deps = cc_deps,
    )

    # Add Python library.
    drake_py_library(
        name = name,
        data = [cc_so],
        srcs = py_srcs,
        deps = py_deps,
        imports = py_imports,
    )

    # Add installation.
    if py_pkg_install:
        py_dest = "lib/python{}/site-packages/{}".format(PY_VERSION, py_pkg_install)
        # TODO(eric.cousineau): Somehow incorporate a warning if this is in development mode?
        install(
            name = install_name,
            targets = [
                py_name,
                cc_so,
            ],
            py_dest = py_dest,
            library_dest = py_dest,
        )

def _get_install(target):
    if target.contains(":"):
        # Append suffix to target.
        return target + "_install"
    else:
        # Assume that the package has an ":install" target.
        return target + ":install"

def get_drake_pybind_installs(targets):
    return [_get_install(target) for target in targets]
