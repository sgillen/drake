#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace pydrake {

class PySerializeInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;
  using Base::Base;

  std::unique_ptr<AbstractValue> CreateDefaultValue() {
    PYBIND11_OVERLOAD_PURE(
        std::unique_ptr<AbstractValue>, SerializerInterface,
        "CreateDefaultValue");
  }

  void Deserialize(
      const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const override {
    auto wrap = [&]() -> std::unique_ptr<AbstractValue> {
      py::bytes buffer(message_bytes, message_length);
      PYBIND11_OVERLOAD_INT(
          std::unique_ptr<AbstractValue>, SerializerInterface, "Deserialize", buffer);
    };
    auto new_value = wrap();
    abstract_value->SetFrom(*new_value);
    // Fail if overload not found.
    py::pybind11_fail("No overload defined!");
  }

  void Serialize(const AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    std::vector::bytes buffer(message_bytes, message_length);
    // Capture return.
    auto wrap = [&]() -> py::bytes {
      PYBIND11_OVERLOAD_INT(
        void, SerializerInterface, "Deserialize", buffer);
      // Fail if overload not found.
      py::pybind11_fail("No overload defined!");
    };
    // Fail if overload not found.
    py::pybind11_fail("No overload defined!"); 
  }
};

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = SerializerInterface;
    py::class_<Class>(m, "SerializerInterface")
        .def("CreateDefaultValue", &Class::CreateDefaultValue)
        .def("Deserialize", [](const Class* self, py::bytes buffer) {
            std::string str = buffer;
            std::unique_ptr<AbstractValue> value = self->CreateDefaultValue();
            self->Deserialize(str.data(), str.size(), value.get());
            return value;
          }, py::arg("buffer"))
        .def("Serialize", [](
              const Class* self, const AbstractValue* abstract_value) {
            std::vector<uint8_t> bytes;
            self->Serialize(*abstract_value, bytes);
            return py::bytes(
                reinterpret_cast<const char*>(bytes.data()), bytes.size());
          });
  }

  {
    using Class = LcmPublisherSystem;
    py::class_<Class>(m, "LcmPublisherSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>())
        .def(
            "set_publish_period", &Class::set_publish_period,
            py::arg("period"));
  }

  {
    using Class = LcmSubscriberSystem;
    py::class_<Class>(m, "LcmSubscriberSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>());
  }

  // Evaluate additional Python code.
  // - Inject `__file__` for ease of 
  py::str pydrake_file = py::module::import("pydrake").attr("__file__");
  py::module path = py::module::import("os.path");
  py::str py_code = path.attr("join")(
      path.attr("dirname")(pydrake_file), "systems", "_lcm.py");
  py::globals()["__file__"] = path.attr("join")(
      path.attr("dirname")(pydrake_file), "systems", "lcm.so");
  py::eval_file(py_code);
}

}  // namespace pydrake
}  // namespace drake
