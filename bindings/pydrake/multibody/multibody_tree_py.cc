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
namespace {

using std::string;

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

// Binds MutlibodyTreeElement methods.
// N.B. We do this rather than inheritance because this template is more of a
// mixin than it is a parent class (since it is not used for its dynamic
// polymorphism).
template <typename PyClass>
void BindMultibodyTreeElementMixin(PyClass& cls) {
  using Class = typename PyClass::type;
  cls
      .def("get_parent_tree", &Class::get_parent_tree, py_reference_internal)
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance);
}

void init(py::module m) {
  // These are defined in the same order as `multibody_tree_indexes.h`.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex");
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex");
  BindTypeSafeIndex<MobilizerIndex>(m, "MobilizerIndex");
  BindTypeSafeIndex<BodyNodeIndex>(m, "BodyNodeIndex");
  BindTypeSafeIndex<ForceElementIndex>(m, "ForceElementIndex");
  BindTypeSafeIndex<JointIndex>(m, "JointIndex");
  BindTypeSafeIndex<JointActuatorIndex>(m, "JointActuatorIndex");
  BindTypeSafeIndex<ModelInstanceIndex>(m, "ModelInstanceIndex");
  m.def("world_index", &world_index);

  {
    using Class = Frame<T>;
    py::class_<Class> cls(m, "Frame");
    BindMultibodyTreeElementMixin(cls);
    cls
        .def("body", &Class::body, py_reference);
  }

  {
    using Class = BodyFrame<T>;
    py::class_<Class, Frame<T>> cls;
    // No need to re-bind element mixins from `Frame`.
  }

  {
    using Class = Body<T>;
    py::class_<Class> cls(m, "Body")
    BindMultibodyTreeElementMixin(cls);
  }

  {
    using Class = Joint<T>;
    py::class_<Class> cls(m, "Joint");
    BindMultibodyTreeElementMixin(cls);
    cls
        .def("name", &Class::name)
        .def("parent_body", &Class::parent_body, py_reference)
        .def("child_body", &Class::child_body, py_reference)
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal)
        .def("frame_on_child", &Class:frame_on_child, py_reference_internal)
        .def("num_dofs", &Class::num_dofs);
  }

  {
    // N.B. We purposely do not expose much functionality, as users should
    // generally be using `MultibodyPlant`. We simply enable passing the object
    // around.
    using Class = MultibodyTree<T>;
    py::class_<Class>(m, "MultibodyTree");
  }
}

void init_math(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "MultibodyTree math functionality.";

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
}

void init_multibody_plant(py::module m) {
  {
    using Class = MultibodyPlant<T>;
    py::class_<Class, systems::LeafSystem<T>> cls(m, "MultibodyPlant");
    // N.B. These are defined as they appear in the class declaration.
    // TODO(eric.cousineau): Add model-instance based overloads beyond
    // forwarded methods.
    // Forwarded methods from `MultibodyTree`.
    cls
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
        .def("num_actuated_dofs", &Class::num_actuated);
    // TODO(eric.cousineau): Add construction methods, `AddRigidBody`, etc.
    // Topology queries.
    cls
        .def("HasBodyNamed", py::overload_cast<string>(&Class::HasBodyNamed),
             py::arg("name"))
        .def("HasJointNamed", py::overload_cast<string>(&Class::HasJointNamed),
             py::arg("name"))
        .def("GetBodyByName", py::overload_cast<string>(&Class::GetBodyByName),
             py::arg("name"), py_reference_internal)
        .def("GetJointByName",
             py::overload_cast<string>(&Class::GetJointByName),
             py::arg("name"), py_reference_internal)
        .def("GetJointActuatorByName",
             py::overload_cast<string>(&Class::GetJointActuatorByName),
             py::arg("name"), py_reference_internal);
    // Port accessors.
    cls
        .def("get_actuation_input_port",
             py::overload_cast<>(&Class::get_actuation_input_port),
             py_reference_internal)
        .def("get_continuous_state_output_port",
             py::overload_cast<>(&Class::get_continuous_state_output_port),
             py_reference_internal)
        .def("get_contact_results_output_port",
             &Class::get_contact_results_output_port, py_reference_internal)
    // Property accessors.
    cls
        .def("world_body", &Class::world_body, py_reference_internal)
        .def("model", &Class::model, py_reference_internal)
        .def("is_finalized", &Class::is_finalized);
  }
}

void init_parsing(py::module m) {
  m.def("AddModelFromSdfFile",
        py::overload_cast<string, string, MultibodyPlant<T>*, SceneGraph<T>*>(
            &AddModelFromSdfFile),
        py::arg("file_name"), py::arg("model_name"), py::arg("plant"),
        py::arg("scene_graph") = nullptr);
  m.def("AddModelFromSdfFile",
        py::overload_cast<string, MultibodyPlant<T>*, SceneGraph<T>*>(
            &AddModelFromSdfFile),
        py::arg("file_name"), py::arg("plant"),
        py::arg("scene_graph") = nullptr);
}

void init_all(py::module m) {
  // Not sure if relative imports will work in this context, so we will
  // manually spell it out.
  py::dict vars = m.attr("__dict__");
  py::exec(
      R"""(
from pydrake.multibody.multibody_tree import *
from pydrake.multibody.multibody_tree.math import *
from pydrake.multibody.multibody_tree.multibody_tree import *
from pydrake.multibody.multibody_tree.parsing import *
)""", py::globals(), vars);
}

}  // namespace

PYBIND11_MODULE(multibody_tree, m) {
  m.doc() = "MultibodyTree functionality.";

  // N.B. At present, we cannot have `math` as a submodule here, and in
  // `pydrake`. The current solution is to manually define submodules.
  // See the dicussion in #8282 for more information.
  init(m);
  init_math(m.def_submodule("math"));
  init_multibody_plant(m.def_submodule("multibody_plant"));
  init_parsing(m.def_submodule("parsing"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
