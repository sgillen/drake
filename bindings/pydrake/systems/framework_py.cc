#include <pybind11/pybind11.h>

#include "drake/systems/framework/system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/abstract_value.h"
#include "drake/systems/framework/basic_vector.h"

namespace py = pybind11;

PYBIND11_MODULE(framework, m) {
  using drake::systems;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<System<T>>(m, "System")
    .def(py::init<>());
  py::class_<LeafSystem<T>, System<T>>(m, "LeafSystem")
    .def(py::init<>());

  py::class_<Context<T>>(m, "Context")
    .def(py::init<>());
  py::class_<LeafContext<T>, Context<T>>(m, "LeafContext")
    .def(py::init<>());
  py::class_<Diagram<T>, System<T>>(m, "Diagram");

  // Glue mechanisms.
  py::class_<DiagramBuilder<T>>(m, "DiagramBuilder");

  // Value types.
  py::class_<VectorBase<T>>(m, "VectorBase");
  py::class_<BasicVector<T>, VectorBase<T>>(m, "BasicVector");
  py::class_<AbstractValue<T>>(m, "AbstractValue");

  // Parameters.
  py::class_<Parameters<T>>(m, "Parameters");
  // State.
  py::class_<State<T>>(m, "State");
  // - Constituents.
  py::class_<ContinuousState<T>>(m, "ContinuousState");
  py::class_<DiscreteValues<T>>(m, "DiscreteValues");
  py::class_<AbstractValues>(m, "AbstractValues");
}
