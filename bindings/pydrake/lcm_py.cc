#include  <cstring>

#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_mock_lcm.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;

  // At present, it's easier to use `string` in lieu of `vector<uint8_t>` due to
  // the strictness of `pybind`s STL containers and how `lcm` Python handles
  // serialization.
  using LcmBuffer = std::string;
  using VectorHandlerFunction = std::function<void(const LcmBuffer&)>;

  {
    using Class = DrakeLcmInterface;
    py::class_<Class>(m, "DrakeLcmInterface")
        .def("Subscribe", [](
              Class* self, const std::string& channel,
              VectorHandlerFunction handler) {
            DrakeLcmInterface::HandlerFunction wrap_handler =
                [handler](const void* data, int size) {
              LcmBuffer buffer(size, ' ');
              std::memcpy(&buffer[0], data, size);
              handler(buffer);
            };
            self->Subscribe(channel, wrap_handler);
          }, py::arg("channel"), py::arg("handler"))
        .def("Publish", [](
              Class* self, const std::string& channel,
              const LcmBuffer& buffer, optional<double> time_sec) {
            self->Publish(channel, buffer.data(), buffer.size(), time_sec);
          },
          py::arg("channel"), py::arg("buffer"),
          py::arg("time_sec") = py::none());
  }

  {
    using Class = DrakeLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeLcm")
        .def(py::init<>())
        .def("StartReceiveThread", &Class::StartReceiveThread)
        .def("StopReceiveThread", &Class::StopReceiveThread);
    // TODO(eric.cousineau): Add remaining methods.
  }

  {
    using Class = DrakeMockLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeMockLcm")
        .def(py::init<>())
        .def("InduceSubscriberCallback", [](
              Class* self, const std::string& channel,
              const LcmBuffer& buffer) {
            self->InduceSubscriberCallback(
                channel, buffer.data(), buffer.size());
          })
        .def("get_last_published_message", [](
              const Class* self, const std::string& channel) {
            const auto& raw = self->get_last_published_message(channel);
            LcmBuffer buffer(raw.size(), ' ');
            std::memcpy(&buffer[0], raw.data(), raw.size());
            return buffer;
          });
    // TODO(eric.cousineau): Add remaining methods.
  }
}

}  // namespace pydrake
}  // namespace drake
