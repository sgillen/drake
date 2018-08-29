# -*- python -*-

# Should include everything any consumer of Drake would ever need.
#
# Do not update this list by hand; instead, run build_components_refresh.py.
LIBDRAKE_COMPONENTS = [
    # "//bindings/pydrake/util:cpp_param_pybind",  # unpackaged
    # "//bindings/pydrake/util:cpp_template_pybind",  # unpackaged
    # "//bindings/pydrake/util:deprecation_pybind",  # unpackaged
    # "//bindings/pydrake/util:drake_optional_pybind",  # unpackaged
    # "//bindings/pydrake/util:eigen_geometry_pybind",  # unpackaged
    # "//bindings/pydrake/util:eigen_pybind",  # unpackaged
    # "//bindings/pydrake/util:type_pack",  # unpackaged
    # "//bindings/pydrake/util:type_safe_index_pybind",  # unpackaged
    # "//bindings/pydrake/util:wrap_function",  # unpackaged
    # "//bindings/pydrake/util:wrap_pybind",  # unpackaged
    # "//bindings/pydrake:autodiff_types_pybind",  # unpackaged
    # "//bindings/pydrake:pydrake_pybind",  # unpackaged
    # "//bindings/pydrake:symbolic_types_pybind",  # unpackaged
    "//common",
    "//common/proto",
    "//common/trajectories",
    "//common:drake_marker_shared_library",  # unpackaged
    "//common:text_logging_gflags_h",  # unpackaged
    "//lcm",
    "//math",
    "//util",
]
