#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_vector.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/parsing/sdf_parser.h"
#include "drake/multibody/plant/contact_info.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace pydrake {
namespace {

using std::string;

using geometry::SceneGraph;
using systems::Context;
using systems::State;

PYBIND11_MODULE(multibody_tree, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
