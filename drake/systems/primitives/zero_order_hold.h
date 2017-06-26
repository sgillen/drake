#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/siso_vector_system.h"

namespace drake {
namespace systems {

/// A ZeroOrderHold block with input `u`, which may be discrete or continuous,
/// and discrete output `y`, where the y is sampled from u with a fixed period.
/// @ingroup primitive_systems
template <typename T>
class ZeroOrderHold : public SisoVectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZeroOrderHold)

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// vector-valued input of size @p size.
  ZeroOrderHold(double period_sec, int size);

 protected:
  // System<T> override.  Returns a ZeroOrderHold<symbolic::Expression> with
  // the same dimensions as this ZeroOrderHold.
  ZeroOrderHold<symbolic::Expression>* DoToSymbolic() const override;

  /// Sets the output port value to the value that is currently latched in the
  /// zero-order hold.
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  /// Latches the input port into the discrete state.
  void DoCalcVectorDiscreteVariableUpdates(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* discrete_updates) const override;

 private:
  const double period_sec_{};
};

/// An AbstractZeroOrderHold block with an abstract input `u`, which may be
/// discrete or continuous, and discrete abstract output `y`, where the y is
/// sampled from u with a fixed period.
/// @tparam ValueType The abstract type to be stored.
/// @tparam T The base type for the LeafSystem.
template <typename ValueType, typename T = double>
class AbstractZeroOrderHold : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractZeroOrderHold)

  typedef std::function<void(double time, const ValueType& value)>
      UpdateCallback;

  /// @param ic Initial condition value.
  /// @param period_sec Update period.
  /// @param use_autoinit Override initial condition by evaluating the upstream
  /// block's output at t = 0, such that the supplied initial condition will
  /// implicitly match the block's output without precomputation.
  AbstractZeroOrderHold(const ValueType& ic, double period_sec,
                        bool use_autoinit = false)
      : use_autoinit_(use_autoinit) {
    this->DeclareAbstractInputPort();
    // TODO(eric.cousineau): Is there a way to not care about the default
    // constructor, and just inherit this from an upstream system given
    // type erasure?
    this->DeclareAbstractState(std::make_unique<Value<ValueType>>(ic));
    this->DeclareAbstractOutputPort(ic, &AbstractZeroOrderHold::CalcOutput);
    this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0.);
  }

  /// Construct an abstract zero order hold with a default-constructed value.
  AbstractZeroOrderHold(double period_sec, bool use_autoinit = false)
      : AbstractZeroOrderHold(ValueType(), period_sec, use_autoinit) {}

  // /// Set the callback to be called each time the discrete value is to be
  // /// called, supplying the time and the value provided.
  // /// This is meant for testing / debugging purposes.
  // void set_update_callback(const UpdateCallback& on_update) {
  //   DRAKE_ASSERT(!update_callback_);
  //   update_callback_ = on_update;
  // }

 protected:
  void DoCalcUnrestrictedUpdate(const Context<T>& context,
                                State<T>* state) const override {
    const ValueType& input_value =
        this->EvalAbstractInput(context, 0)->template GetValue<ValueType>();
    ValueType& stored_value =
        state->template get_mutable_abstract_state<ValueType>(0);
    stored_value = input_value;
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    if (use_autoinit_) {
      // Update initial values with the ICs from upstream blocks.
      this->DoCalcUnrestrictedUpdate(context, state);
    } else {
      LeafSystem<T>::SetDefaultState(context, state);
    }
  }

  void CalcOutput(const Context<T>& context,
                    ValueType* poutput) const {
    const ValueType& stored_value =
        context.template get_abstract_state<ValueType>(0);
    *poutput = stored_value;
  }

 private:
  // UpdateCallback update_callback_;
  bool use_autoinit_{};
};


}  // namespace systems
}  // namespace drake
