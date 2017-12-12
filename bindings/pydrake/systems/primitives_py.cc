#include <pybind11/pybind11.h>

#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace py = pybind11;

PYBIND11_MODULE(primitives, m) {
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  using T = double;

  py::class_<ConstantVectorSource>(m, "ConstantVectorSource");

  py::class_<Adder<T>, LeafSystem<T>>(m, "Adder")
    .def(py::init<int, int>());

  py::class_<Integrator<T>, LeafSystem<T>>(m, "Integrator")
    .def(py::init<int>());

  // py::class_<ConstantValueSource>(m, "ConstantValueSource");
}
