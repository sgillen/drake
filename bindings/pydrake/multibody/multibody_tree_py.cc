#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"

namespace drake {
namespace pydrake {

void init_math(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "MultibodyTree math functionality.";

  using T = double;

  py::class_<SpatialVector<SpatialVelocity, T>>(m, "SpatialVector")
      .def("rotational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->rotational(); },
           py_reference_internal)
      .def("translational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->translational(); },
           py_reference_internal);

  py::class_<SpatialVelocity<T>, SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVelocity")
      .def(py::init())
      .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                    const Eigen::Ref<const Vector3<T>>&>(),
           py::arg("w"), py::arg("v"));

  // This is defined in the same order as `multibody_tree_indexes.h`.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex");
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex");
  BindTypeSafeIndex<MobilizerIndex>(m, "MobilizerIndex");
  BindTypeSafeIndex<BodyNodeIndex>(m, "BodyNodeIndex");
  BindTypeSafeIndex<ForceElementIndex>(m, "ForceElementIndex");
  BindTypeSafeIndex<JointIndex>(m, "JointIndex");
  BindTypeSafeIndex<JointActuatorIndex>(m, "JointActuatorIndex");
  BindTypeSafeIndex<ModelInstanceIndex>(m, "ModelInstanceIndex");
  m.def("world_index", &world_index);

  // TODO(jadecastro, eric.cousineau): Bind additional classes as necessary.
  {
    using Class = MultibodyPlant<T>;
    py::class_<Class>(m, "MultibodyPlant")
        .def(py::init<double>(), py::arg("time_step") = 0.)
        .def("num_bodies", &Class::num_bodies)
        .def("num_joints", &Class::num_joints)
        .def("num_actuators", &Class::num_actuators)
        .def("num_model_instances", &Class::num_model_instances)
        .def("num_positions", py::overload_cast<>(&Class::num_positions))
        .def("num_positions",
             py::overload_cast<ModelInstanceIndex>(&Class::num_positions),
             py::arg("model_instance"))
        .def("num_velocities", py::overload_cast<>(&Class::num_velocities))
        .def("num_velocities",
             py::overload_cast<ModelInstanceIndex>(&Class::num_velocities))
        .def("num_multibody_states", &Class::num_multibody_states)
        .def("num_actuated_dofs", &Class::num_actuated
  }
  {
    using Class = BodyTopology
  }
  {
    using Class = PositionKinematicsCache<T>;
    py::class_<Class>(m, "PositionKinematicsCache")
        .def(py::init<const MultibodyTreeTopology&>(), py::arg("topology"))
        .def("get_X_WB", &Class::get_X_WB);
  }
}

void init_all(py::module m) {
  // Not sure if relative imports will work in this context, so we will
  // manually spell it out.
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.multibody.multibody_tree.math import *",
      py::globals(), vars);
}

PYBIND11_MODULE(multibody_tree, m) {
  m.doc() = "MultibodyTree functionality.";

  // N.B. At present, we cannot have `math` as a submodule here, and in
  // `pydrake`. The current solution is to manually define submodules.
  // See the dicussion in #8282 for more information.
  init_math(m.def_submodule("math"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
