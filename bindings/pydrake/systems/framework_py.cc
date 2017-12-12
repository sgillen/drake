#include <pybind11/pybind11.h>

#include "drake/systems/framework/system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/abstract_value.h"
#include "drake/systems/framework/basic_vector.h"

namespace py = pybind11;

using std::unique_ptr;
using std::vector;

PYBIND11_MODULE(framework, m) {
  using namespace drake::systems;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<System<T>>(m, "System")
    // .def(py::init<>())
    .def("set_name", &System<T>::set_name)
    .def("get_input_port", &System<T>::get_input_port)
    .def("get_output_port", &System<T>::get_output_port);

  py::class_<LeafSystem<T>, System<T>>(m, "LeafSystem");
    // .def(py::init<>());

  py::class_<Context<T>>(m, "Context")
    // .def(py::init<>())
    .def("FixInputPort",
         py::overload_cast<int, unique_ptr<BasicVector<T>>>(
             &Context<T>::FixInputPort))
    .def("get_time", &Context<T>::get_time);

  py::class_<LeafContext<T>, Context<T>>(m, "LeafContext")
    // .def(py::init<>());

  py::class_<Diagram<T>, System<T>>(m, "Diagram")
    // .def(py::init<>())
    .def("CreateDefaultContext", &Diagram<T>::CreateDefaultContext)
    .def("AllocateOutput", &Diagram<T>::AllocateOutput)
    .def("GetGraphvizString", &Diagram<T>::GetGraphvizString)
    .def("GetMutableSubsystemState", &State<T>::GetMutableSubsystemState);

  // Glue mechanisms.
  py::class_<DiagramBuilder<T>>(m, "DiagramBuilder")
    .def(py::init<>())
    .def("Connect",
         py::overload_cast<const OutputPort<T>&, const InputPortDescriptor<T>&>(
             &DiagramBuilder<T>::Connect))
    .def("ExportInput", &DiagramBuilder<T>::ExportInput)
    .def("ExportOutput", &DiagramBuilder<T>::ExportOutput)
    .def("Build", &DiagramBuilder<T>::Build)
    .def("BuildInto", &DiagramBuilder<T>::BuildInto);

  py::class_<OutputPort<T>>(m, "OutputPort");

  py::class_<InputPortDescriptor<T>>(m, "InputPortDescriptor");

  // Value types.
  py::class_<VectorBase<T>>(m, "VectorBase")
    .def("SetFromVector", &VectorBase<T>::SetFromVector);
  py::class_<BasicVector<T>, VectorBase<T>>(m, "BasicVector")
    .def_static("Make", [](const vector<T>& in) {
       return BasicVector<T>::Make(in);
    });
    .def("get_value", BasicVector<T>::get_value);

  py::class_<AbstractValue<T>>(m, "AbstractValue");

  // Parameters.
  py::class_<Parameters<T>>(m, "Parameters");
  // State.
  py::class_<State<T>>(m, "State")
    .def(py::init<>())
    .def("get_mutable_continuous_state",
         &State<T>::get_mutable_continuous_state);

  // - Constituents.
  py::class_<ContinuousState<T>>(m, "ContinuousState")
    .def(py::init<>())
    .def("get_mutable_vector", &ContinuousState<T>::get_mutable_vector);
  py::class_<DiscreteValues<T>>(m, "DiscreteValues");
  py::class_<AbstractValues<T>>(m, "AbstractValues");
}
