#include <pybind11/eigen.h>
#include <pybind11/eval.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"

using std::shared_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(framework, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  // Ensure we have bindings for dependencies.
  py::module::import("pydrake.rbtree");
  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<RigidBodyPlant<T>, LeafSystem<T>> cls;
  cls
      .def(py::init<shared_ptr<const RigidBodyTree<T>>, double>(),
           py::arg("tree"), py::arg("timestep") = 0.0)
      .def("get_rigid_body_tree", &RigidBodyPlant<T>::get_rigid_body_tree,
           py_reference_internal)
      .def("get_num_bodies", &RigidBodyPlant<T>::get_num_bodies)
      .def("get_num_positions",
           py::overload_cast<>(&RigidBodyPlant<T>::get_num_positions))
      .def("get_num_positions",
           py::overload_cast<int>(&RigidBodyPlant<T>::get_num_positions))
      .def("get_num_velocities",
           py::overload_cast<>(&RigidBodyPlant<T>::get_num_velocities))
      .def("get_num_velocities",
           py::overload_cast<int>(&RigidBodyPlant<T>::get_num_velocities))
}

}  // namespace pydrake
}  // namespace drake
