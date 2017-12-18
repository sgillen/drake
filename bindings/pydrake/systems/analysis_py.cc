#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/interface_py.h"
#include "drake/systems/analysis/simulator.h"

namespace py = pybind11;

using std::shared_ptr;
using pydrake::drake_class;

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  using rvp = py::return_value_policy;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  using T = double;

  drake_class<Simulator<T>>(m, "Simulator")
    .def(py::init<const System<T>&>())
    .def(py::init<const System<T>&, shared_ptr<Context<T>>>())
    .def("Initialize", &Simulator<T>::Initialize)
    .def("StepTo", &Simulator<T>::StepTo)
    .def("get_mutable_context", &Simulator<T>::get_mutable_context,
         rvp::reference_internal);
}
