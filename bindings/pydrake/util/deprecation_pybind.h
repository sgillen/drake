#pragma once

/// @file
/// Provides access to Python deprecation utilites from C++.

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/// Deprecates an attribute `name` of a class `cls`.
/// This *only* works with class attributes (unbound members or methods) as it
/// is implemented with a Python property descriptor.
inline void DeprecateAttribute(
    py::object cls, py::str name, py::str message) {
  py::object deprecate_attribute =
      py::module::import("pydrake.util.deprecation")
      .attr("_deprecate_attribute");
  deprecate_attribute(cls, name, message);
}

/// Deprecates an alias attribute.
inline void SetDeprecatedAliasAttribute(
    py::object cls, py::str alias, py::str original) {
  cls.attr(alias) = cls.attr(original);
  DeprecateAttribute(
    cls, alias,
    py::str("`{}` is deprecated; please use `{}` instead").format(
        alias, original));
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

}  // namespace pydrake
}  // namespace drake
