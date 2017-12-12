#include <pybind11/pybind11.h>

#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace py = pybind11;

PYBIND11_MODULE(primitives, m) {
  using drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  py::class_<ConstantValueSource>(m, "ConstantValueSource");
  py::class_<ConstantVectorSource>(m, "ConstantVectorSource");
}
