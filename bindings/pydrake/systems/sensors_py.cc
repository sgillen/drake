#include <pybind11/eigen.h>
#include <pybind11/eval.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/pixel_types.h"

namespace py = pybind11;

PYBIND11_MODULE(sensors, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  auto py_iref = py::return_value_policy::reference_internal;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  using T = double;

  py::enum_<PixelType>(m, "PixelType")
    .value("kRgb8U", kRgb8U)
    .value("kBgr8U", kBgr8U)
    .value("kRgba8U", kRgba8U)
    .value("kBgra8U", kBgra8U)
    .value("kGrey8U", kGrey8U)
    .value("kDepth16U", kDepth16U)
    .value("kDepth32F", kDepth32F)
    .value("kLabel16I", kLabel16I);

  // Expose image traits.
}
