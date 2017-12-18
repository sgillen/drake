#include <pybind11/eigen.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/interface_py.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"
#include "drake/systems/framework/system.h"

namespace py = pybind11;

using std::make_unique;
using std::shared_ptr;
using std::vector;
using pydrake::drake_class;

PYBIND11_MODULE(framework, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  using rvp = py::return_value_policy;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  // TODO(eric.cousineau): Resolve `str_py` workaround.
  auto str_py = py::eval("str");

  // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
  // knows this is abstract?
  drake_class<System<T>>(m, "System")
    .def("set_name", &System<T>::set_name)
    .def("get_input_port", &System<T>::get_input_port, rvp::reference_internal)
    .def("get_output_port",
         &System<T>::get_output_port, rvp::reference_internal)
    .def("CreateDefaultContext", &System<T>::CreateDefaultContext)
    .def("AllocateOutput", &System<T>::AllocateOutput)
    .def(
        "GetGraphvizString",
        [str_py](const System<T>* self) {
          // @note This is a workaround; for some reason,
          // casting this using `py::str` does not work, but directly
          // calling the Python function (`str_py`) does.
          return str_py(self->GetGraphvizString());
        });

  drake_class<LeafSystem<T>, System<T>>(m, "LeafSystem");

  drake_class<Context<T>>(m, "Context")
    .def("FixInputPort",
         py::overload_cast<int, shared_ptr<BasicVector<T>>>(
             &Context<T>::FixInputPort))
    .def("get_time", &Context<T>::get_time)
    .def("Clone", &Context<T>::Clone)
    .def("__copy__", &Context<T>::Clone)
    .def("get_state", &Context<T>::get_state, rvp::reference_internal)
    .def("get_mutable_state",
         &Context<T>::get_mutable_state, rvp::reference_internal);

  drake_class<LeafContext<T>, Context<T>>(m, "LeafContext");

  drake_class<Diagram<T>, System<T>>(m, "Diagram")
    .def("GetMutableSubsystemState",
        [](Diagram<T>* self, const System<T>& arg1, Context<T>* arg2)
        -> auto&& {
          // @note Use `auto&&` to get perfect forwarding.
          // @note Compiler does not like `py::overload_cast` with this setup?
          return self->GetMutableSubsystemState(arg1, arg2);
        }, rvp::reference, py::keep_alive<0, 3>());

  // Glue mechanisms.
  drake_class<DiagramBuilder<T>>(m, "DiagramBuilder")
    .def(py::init<>())
    .def(
        "AddSystem",
        [](DiagramBuilder<T>* self, shared_ptr<System<T>> arg1) {
          return self->AddSystem(arg1);
        })
    .def("Connect",
         py::overload_cast<const OutputPort<T>&, const InputPortDescriptor<T>&>(
             &DiagramBuilder<T>::Connect))
    .def("ExportInput", &DiagramBuilder<T>::ExportInput)
    .def("ExportOutput", &DiagramBuilder<T>::ExportOutput)
    .def("Build", &DiagramBuilder<T>::Build)
    .def("BuildInto", &DiagramBuilder<T>::BuildInto);

  drake_class<OutputPort<T>>(m, "OutputPort");
  drake_class<SystemOutput<T>>(m, "SystemOutput");

  drake_class<InputPortDescriptor<T>>(m, "InputPortDescriptor");

  // Value types.
  drake_class<VectorBase<T>>(m, "VectorBase")
    .def("CopyToVector", &VectorBase<T>::CopyToVector)
    .def("SetFromVector", &VectorBase<T>::SetFromVector);

  drake_class<BasicVector<T>, VectorBase<T>>(m, "BasicVector")
    .def(py::init<VectorX<T>>())
    .def("get_value", &BasicVector<T>::get_value);

  drake_class<Supervector<T>, VectorBase<T>>(m, "Supervector");

  drake_class<Subvector<T>, VectorBase<T>>(m, "Subvector");

  // TODO(eric.cousineau): Interfacing with the C++ abstract value types may be
  // a tad challenging. This should be more straightforward once
  // scalar-type conversion is supported, as the template-exposure mechanisms
  // should be relatively similar.
  drake_class<AbstractValue>(m, "AbstractValue");

  // Parameters.
  // TODO(eric.cousineau): Fill this out.
  drake_class<Parameters<T>>(m, "Parameters");

  // State.
  drake_class<State<T>>(m, "State")
    .def(py::init<>())
    .def("get_continuous_state",
         &State<T>::get_continuous_state, rvp::reference_internal)
    .def("get_mutable_continuous_state",
         &State<T>::get_mutable_continuous_state, rvp::reference_internal);

  // - Constituents.
  drake_class<ContinuousState<T>>(m, "ContinuousState")
    .def(py::init<>())
    .def("get_vector", &ContinuousState<T>::get_vector, rvp::reference_internal)
    .def("get_mutable_vector",
         &ContinuousState<T>::get_mutable_vector, rvp::reference_internal);

  drake_class<DiscreteValues<T>>(m, "DiscreteValues");

  drake_class<AbstractValues>(m, "AbstractValues");
}
