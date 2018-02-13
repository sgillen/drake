#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(dragway, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput;

  m.doc() = "Bindings for Maliput.";

  py::class_<dragway::RoadGeometry>(m, "RoadGeometry")
      .def(py::init<api::RoadGeometryId, int, double, double, double, double,
           double, double>())
      .def("linear_tolerance", &dragway::RoadGeometry::linear_tolerance)
      .def("angular_tolerance", &dragway::RoadGeometry::angular_tolerance)
      .def("junction", &dragway::RoadGeometry::junction, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Junction>(m, "Junction")
      .def("segment", &dragway::Junction::segment, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Segment>(m, "Segment")
      .def("lane", &dragway::Segment::lane, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Lane>(m, "Lane");
}

}  // namespace pydrake
}  // namespace drake
