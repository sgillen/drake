#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace pydrake {
namespace pysystems {

/// Defines an instantiation of `pydrake.systems.framework.Value[...]`.
/// @prereq `T` must have already been exposed to `pybind11`.
/// @param scope Parent scope.
/// @tparam T Inner parameter of `Value<T>`.
/// @tparam Class Class to be bound. By default, `Value<T>` is used.
/// @returns `py::class_<>` object, for defining emplace constructors.
template <typename T, typename Class = systems::Value<T>>
auto AddValueInstantiation(py::module scope) {
  py::class_<Class, systems::AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  // Only use copy (clone) construction.
  // Do NOT allow Python to use the `unique_ptr<T>` constructor for move-only
  // values, because the lifetime of C++ objects will be ended the moment the
  // `Value<>` container decides to reset it via `set_value`, and Python will
  // be left with a invalid reference.
  // TODO(eric.cousineau): If we wish to preserve `unique_ptr` references, this
  // motivates either a snowflake `unique_ptr_tracked`, or `shared_ptr`.
  py_class.def(py::init<const T&>());
  // N.B. `reference_internal` for pybind POD types (int, str, etc.) does not
  // really do anything meaningful.
  py_class
    .def("get_value", &Class::get_value, py_reference_internal)
    .def("get_mutable_value", &Class::get_mutable_value, py_reference_internal)
    .def("set_value", &Class::set_value);
  py::module py_module = py::module::import("pydrake.systems.framework");
  AddTemplateClass(py_module, "Value", py_class, GetPyParam<T>());
  return py_class;
}

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
