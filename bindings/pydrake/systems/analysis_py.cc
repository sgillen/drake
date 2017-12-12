#include <pybind11/pybind11.h>

#include "drake/systems/analysis/simulator.h"

namespace py = pybind11;

using std::unique_ptr;

PYBIND11_MODULE(analysis, m) {
  using namespace drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  using T = double;

  py::class_<Simulator<T>>(m, "Simulator")
    .def(py::init<const System<T>&, unique_ptr<Context<T>>>())
    .def("StepTo", &Simulator<T>::StepTo)
    .def("get_mutable_context", &Simulator<T>::get_mutable_context);
}
