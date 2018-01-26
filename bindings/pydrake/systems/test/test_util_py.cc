#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::unique_ptr;

namespace drake {

using systems::BasicVector;
using systems::ConstantVectorSource;

namespace pydrake {
namespace {

using T = double;

// Informs listener when this class is deleted.
class DeleteListenerSystem : public ConstantVectorSource<T> {
 public:
  explicit DeleteListenerSystem(std::function<void()> delete_callback)
    : ConstantVectorSource(VectorX<T>::Constant(1, 0.)),
      delete_callback_(delete_callback) {}

  ~DeleteListenerSystem() override {
    delete_callback_();
  }
 private:
  std::function<void()> delete_callback_;
};

class DeleteListenerVector : public BasicVector<T> {
 public:
  explicit DeleteListenerVector(std::function<void()> delete_callback)
    : BasicVector(VectorX<T>::Constant(1, 0.)),
      delete_callback_(delete_callback) {}

  ~DeleteListenerVector() override {
    delete_callback_();
  }
 private:
  std::function<void()> delete_callback_;
};

}  // namespace

PYBIND11_MODULE(test_util, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace systems;

  // Import dependencies.
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DeleteListenerSystem, ConstantVectorSource<T>>(
      m, "DeleteListenerSystem")
    .def(py::init<std::function<void()>>());
  py::class_<DeleteListenerVector, BasicVector<T>>(
      m, "DeleteListenerVector")
    .def(py::init<std::function<void()>>());

  // Call overrides to ensure a custom Python class can override these methods.

  m.def("call_leaf_system_overrides", [](const LeafSystem<T>& system) {
    auto context = system.AllocateContext();
    // Call `Publish` to test `DoPublish`.
    auto events =
        LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
    system.Publish(*context, *events);
  });

  m.def("call_vector_system_overrides", [](
      const VectorSystem<T>& system, Context<T>* context,
      bool is_discrete, double dt) {
    // While this is not convention, update state first to ensure that our
    // output incorporates it correctly, for testing purposes.
    if (is_discrete) {
      auto& state = context->get_mutable_discrete_state();
      auto state_copy = state.Clone();
      system.CalcDiscreteVariableUpdates(
          *context, state_copy.get());
      state.SetFrom(state_copy);
    } else {
      auto& state = context->get_mutable_continuous_state();
      auto state_dot = state.Clone();
      system.CalcTimeDerivatives(*context, state_dot.get());
      state.SetFromVector(
          state.CopyToVector() + dt * state_dot.CopyToVector());
    }
    // Calculate output.
    auto output = system.AllocateOutput(*context);
    system.CalcOutput(context, output.get());
    return output;
  });
}

}  // namespace pydrake
}  // namespace drake
