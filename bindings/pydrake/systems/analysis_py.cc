#include <pybind11/pybind11.h>

#include "drake/systems/analysis/simulator.h"

namespace py = pybind11;

PYBIND11_MODULE(analysis, m) {
  using drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  py::class_<Simulator>(m, "Simulator")
    .def(py::init<>());
}
