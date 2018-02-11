#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/controllers/dynamic_programming.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;

  py::module::import("pydrake.math");
  py::module::import("pydrake.systems.primitives");

  py::class_<DynamicProgrammingOptions>(m, "DynamicProgrammingOptions")
      .def(py::init<>())
      .def_readwrite("discount_factor",
                    &DynamicProgrammingOptions::discount_factor);

  m.def("FittedValueIteration", &FittedValueIteration);
}

}  // namespace pydrake
}  // namespace drake