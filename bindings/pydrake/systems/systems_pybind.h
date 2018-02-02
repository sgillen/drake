#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace pydrake {
namespace pysystems {

namespace internal {

// Add value constructor for copy-constructible class.
template <typename Class, typename PyClass>
void AddValueConstructor(PyClass& py_class, std::true_type) {
  // Always copy if we can.
  py_class.def(py::init<const Class&>());
}

template <typename Class, typename PyClass>
void AddValueConstructor(PyClass& py_class, std::false_type) {
  // Only define `unique_ptr` overload if we cannot copy.
  py_class.def(py::init<std::unique_ptr<Class>>());
}

}  // namespace internal

/// Defines an instantiation of `Value`.
template <typename T, typename Class = systems::Value<T>>
py::object AddValueInstantiation(py::module scope) {
  py::class_<Class, AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  detail::AddValueConstructor<Class>(
      py_class, std::is_copy_constructible<Class>{});
  py_class
    .def("get_value", &Class::get_value)
    .def("get_mutable_value", &Class::get_mutable_value)
    .def("set_value", &Class::set_value);
  using Param = type_pack<T>;
  py::module py_module = py::module::import("pydrake.systems.framework");
  AddTemplateClass(py_module, "Value", py_class, Param{});
  return py_class;
}

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
