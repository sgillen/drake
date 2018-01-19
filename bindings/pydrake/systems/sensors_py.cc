#include <vector>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/util/cpp_template.h"
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/pixel_types.h"

namespace py = pybind11;

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

template <typename T, T ... Values>
using constant_pack = drake::type_pack<constant<T, Values>...>;

namespace drake {
namespace pydrake {

// Allow `Type` or `const Type`
template <typename ImageT, typename T>
py::object GetImageArray() {
  // Create flat array.
  Eigen::Map<VectorX<T>> data(self->at(0, 0), self->size());
  // Reshape with NumPy.
  py::object array =
      py::cast(data).attr("reshape")(
          self->height(), self->width(), ImageT::kNumChannels);
  return array;
}

}  // namespace pydrake
}  // namespace drake

PYBIND11_MODULE(sensors, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::pydrake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;

  auto py_iref = py::return_value_policy::reference_internal;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  using T = double;

  // Expose only types that are used.
  py::enum_<PixelFormat>(m, "PixelFormat")
    .value("kRgba", PixelFormat::kRgba)
    .value("kDepth", PixelFormat::kDepth)
    .value("kLabel", PixelFormat::kLavel);
  // Expose image traits.
  {
    py::enum_<PixelType> pixel_type(m, "PixelType");
    vector<string> names = {
        "kRgba8U",
        "kDepth32F",
        "kLabel16I",
    };
    using ParamList = constant_pack<PixelType,
        PixelType::kRgba8U,
        PixelType::kDepth32F,
        PixelType::kLabel16I>;
    // Simple constexpr for-loop.
    int i = 0;
    auto iter = [&](auto param) {
      constexpr PixelType Value = decltype(param)::template type<0>::value;
      py::tuple py_param = GetPyParam(param);
      using ImageTraitsT = ImageTraits<PixelType>;
      using T = typename ImageTraitsT::ChannelType;

      // Add definition to enum.
      pixel_type.value(names[i], value);

      // Add traits.
      // TODO(eric.cousineau): Use C++ template.
      py::class_<ImageTraitsT> traits(
          m, TemporaryClassName<ImageTraitsT>().c_str());
      traits.attr("ChannelType") = GetPyParam<T>()[0];
      traits.attr("kNumChannels") = ImageTraitsT::kNumChannels;
      traits.attr("kPixelFormat") = ImageTraitsT::kPixelFormat;
      AddTemplateClass(m, "ImageTraits", traits, py_param);
      ++i;

      using ImageT = Image<Value>;
      py::class_<ImageT> image(m, ("Image_" + names[i]))
          .def(py::init<int, int>())
          .def(py::init<int, int, T>())
          .def("width", &ImageT::width)
          .def("height", &ImageT::height)
          .def("size", &ImageT::size)
          .def("resize", &ImageT::resize)
          .def("get", [](const ImageT* self, int x, int y) {
                return *self->at(x, y);
              })
          .def("set", [](ImageT* self, int x, int y, T value) {
                *self->at(x, y) = value;
              })
          .def("mutable_array", [](ImageT* self) {
                return GetImageArray<ImageT, T>(self);
              })
          .def("array", [](const ImageT* self) {
                return GetImageArray<const ImageT, const T>(self);
              });
      // Constants.
      image.attr("ImageTraits") = traits;
      // - Do not duplicate aliases (e.g. `kNumChannels`) for now.
      AddTemplateClass(m, "Image", image, py_param);
    };
    type_visit(iter, ParamList{});
  }

  // Constants.
  py::class_<InvalidDepth> invalid_depth(m, "InvalidDepth");
  invalid_depth.attr("kTooFar") = InvalidDepth::kTooFar;
  invalid_depth.attr("kTooClose") = InvalidDepth::kTooClose;

  py::class_<Lable> label(m, "Label");
  label.attr("kTooFar") = Label::kNoBody;
  label.attr("kTooClose") = Label::kFlatTerrain;
}
