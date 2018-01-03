#include <pybind11/pybind11.h>

#include "python/bindings/pymodule/tpl/cpp_types.h"

using drake::pydrake::internal::TypeRegistry;

PYBIND11_MODULE(_cpp_types_py, m) {
  py::class_<TypeRegistry> type_registry_cls(m, "_TypeRegistry");
  type_registry_cls
    .def(py::init<>())
    .def("GetPyTypeCanonical", &TypeRegistry::GetPyTypeCanonical)
    .def("GetName", &TypeRegistry::GetName);
  // Create singleton instance.
  m.attr("_type_registry") = type_registry_cls();
}
