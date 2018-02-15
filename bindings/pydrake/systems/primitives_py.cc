#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  using T = double;

  py::class_<ConstantVectorSource<T>, LeafSystem<T>>(m, "ConstantVectorSource")
    .def(py::init<VectorX<T>>());

  py::class_<ConstantValueSource<T>, LeafSystem<T>>(m, "ConstantValueSource");

  py::class_<Adder<T>, LeafSystem<T>>(m, "Adder")
    .def(py::init<int, int>());

  py::class_<Integrator<T>, LeafSystem<T>>(m, "Integrator")
    .def(py::init<int>());

  py::class_<ZeroOrderHold<T>, LeafSystem<T>>(m, "ZeroOrderHold")
    .def(py::init<double, int>());

  py::class_<SignalLogger<T>, LeafSystem<T>>(m, "SignalLogger")
    .def(py::init<int>())
    .def(py::init<int, int>())
    .def("sample_times", &SignalLogger<T>::sample_times)
    .def("data", &SignalLogger<T>::data);

  py::class_<Multiplexer<T>, LeafSystem<T>>(m, "Multiplexer")
    .def(py::init<int>())
    .def(py::init<std::vector<int>>())
    .def(py::init<const BasicVector<T>&>());

  py::class_<TrajectorySource<T>, LeafSystem<T>>(m, "TrajectorySource")
    .def(py::init<const PiecewisePolynomialTrajectory&>());

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
