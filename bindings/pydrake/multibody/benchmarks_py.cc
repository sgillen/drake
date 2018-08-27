 #include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

namespace drake {
namespace pydrake {

using geometry::SceneGraph;
using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

void init_acrobot(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::benchmarks::acrobot;

  py::class_<AcrobotParameters>(m, "AcrobotParameters")
      .def(py::init());

  m.def("MakeAcrobotPlant",
        py::overload_cast<const AcrobotParameters&, bool, SceneGraph<double>*>(
            &MakeAcrobotPlant),
        py::arg("default_parameters"), py::arg("finalize"),
        py::arg("scene_graph") = nullptr);
}

PYBIND11_MODULE(benchmarks, m) {
  py::module::import("pydrake.multibody.multibody_tree");
  init_acrobot(m.def_submodule("acrobot"));
}

}  // namespace pydrake
}  // namespace drake
