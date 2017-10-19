#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/symbolic_types_py.h"

namespace py = pybind11;

PYBIND11_PLUGIN(_odr_test_py) {
  using drake::symbolic::Variable;

  py::module m("_odr_test_py",
               "Test ODR using Variable.");

  m.def("new_variable", [](const std::string& name) {
    return new Variable(name);
  });

  return m.ptr();
}
