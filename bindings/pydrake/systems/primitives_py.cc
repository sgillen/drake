#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/interface_py.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"

namespace py = pybind11;

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  using pydrake::drake_class;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  using T = double;

  drake_class<ConstantVectorSource<T>, LeafSystem<T>>(m, "ConstantVectorSource")
    .def(py::init<VectorX<T>>());

  drake_class<ConstantValueSource<T>, LeafSystem<T>>(m, "ConstantValueSource");

  drake_class<Adder<T>, LeafSystem<T>>(m, "Adder")
    .def(py::init<int, int>());

  drake_class<Integrator<T>, LeafSystem<T>>(m, "Integrator")
    .def(py::init<int>());
}
