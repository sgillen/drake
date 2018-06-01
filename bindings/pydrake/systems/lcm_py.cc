#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = LcmPublisherSystem;
    py::class_<Class>(m, "DrakeLcmInterface");
  }

}

}  // namespace pydrake
}  // namespace drake
