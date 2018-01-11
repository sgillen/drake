#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

namespace py = pybind11;

PYBIND11_MODULE(_cpp_types_py, m) {
  using drake::pydrake::internal::RegisterTypes;
  py::exec("import ctypes; import numpy as np");
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterTypes<bool>("bool");
  RegisterTypes<std::string>("str");
  RegisterTypes<double>("float");
  RegisterTypes<float>("np.float32");
  RegisterTypes<int>("int");
  RegisterTypes<uint32_t>("np.uint32");
  RegisterTypes<int64_t>("np.int64");
  // For supporting generic Python types.
  RegisterTypes<py::object>("object");
}
