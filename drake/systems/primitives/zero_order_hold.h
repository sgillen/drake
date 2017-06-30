#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A ZeroOrderHold block with input `u`, which may be vector-valued (discrete
/// or continuous) or abstract, and discrete output `y`, where the y is sampled
/// from u with a fixed period.
/// @ingroup primitive_systems
template <typename T>
class ZeroOrderHold : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZeroOrderHold)

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// vector-valued input of size @p size.
  ZeroOrderHold(double period_sec, int size);

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// abstract-valued input @p value.
  ZeroOrderHold(double period_sec, const AbstractValue& value);

  ~ZeroOrderHold() override {}

 protected:
  // System<T> override.  Returns a ZeroOrderHold<symbolic::Expression> with
  // the same dimensions as this ZeroOrderHold.
  ZeroOrderHold<symbolic::Expression>* DoToSymbolic() const override;

  /// Sets the output port value to the vector value that is currently
  /// latched in the zero-order hold.
  void DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const;

  /// Latches the input port into the discrete vector-valued state.
  void DoCalcDiscreteVariableUpdates(
      const Context <T>& context,
      const std::vector<const DiscreteUpdateEvent <T> *>& events,
      DiscreteValues <T> *discrete_state) const override;

  // Return a cloned copy of the initial abstract value.
  std::unique_ptr<AbstractValue> AllocateAbstractValue(const Context<T>&) const;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void DoCalcAbstractOutput(
      const Context<T>& context,
      AbstractValue* output) const;

  // Same as `DoCalcDiscreteVariablesUpdate`, but for abstract values.
  void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const override;

 private:
  bool is_abstract() const { return abstract_value_ != nullptr; }

  const double period_sec_{};
  const std::unique_ptr<AbstractValue> abstract_value_;
};

}  // namespace systems
}  // namespace drake
