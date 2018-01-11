#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

namespace py = pybind11;

PYBIND11_MODULE(_cpp_types_py, m) {
  using drake::pydrake::internal::RegisterTypes;
  py::exec("import ctypes; import numpy as np");

  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterTypes<bool>(py::eval("bool,"));
  RegisterTypes<std::string>(py::eval("str,"));
  RegisterTypes<double>(py::eval("float, np.double, ctypes.c_double"));
  RegisterTypes<float>(py::eval("np.float32, ctypes.c_float"));
  RegisterTypes<int>(py::eval("int, np.int32, ctypes.c_int32"));
  RegisterTypes<uint32_t>(py::eval("np.uint32, ctypes.c_uint32"));
  RegisterTypes<int64_t>(py::eval("np.int64, ctypes.c_int64"));
  // For supporting generic Python types.
  RegisterTypes<py::object>(py::eval("object,"));
}
