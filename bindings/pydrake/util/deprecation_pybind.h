#pragma once

/// @file
/// Provides access to Python deprecation utilities from C++.

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/// Deprecates an attribute `name` of a class `cls`.
/// This *only* works with class attributes (unbound members or methods) as it
/// is implemented with a Python property descriptor.
inline void DeprecateAttribute(
    py::object cls, py::str name, py::str message) {
  py::object deprecated =
      py::module::import("pydrake.util.deprecation").attr("deprecated");
  py::object original = cls.attr(name);
  cls.attr(name) = deprecated(message)(original);
}

/// Raises a deprecation warning.
///
/// @note If you are deprecating a class's member or method, please use
/// `DeprecateAttribute` so that the warning is issued immediately when
/// accessed, not only when it is called.
inline void WarnDeprecated(py::str message) {
  py::object warn_deprecated =
      py::module::import("pydrake.util.deprecation").attr("_warn_deprecated");
  warn_deprecated(message);
}

/// Provides a string to indicate deprecation; to replace documentation of an
/// *overload only*. Other attributes should use `DeprecateAttribute`, which
/// will automatically override documentation.
inline std::string GetDeprecationDoc(const std::string& message) {
  py::object get_deprecation_doc =
      py::module::import("pydrake.util.deprecation")
      .attr("_get_deprecation_doc");
  return get_deprecation_doc(message).cast<std::string>();
}

}  // namespace pydrake
}  // namespace drake
