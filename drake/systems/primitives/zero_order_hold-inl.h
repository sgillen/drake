#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <memory>
#include <vector>

#include "drake/common/unused.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {

template <typename T>
void ZeroOrderHold<T>::Construct(double period_sec, int size) {
  vector_size_ = size;
  // TODO(david-german-tri): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  BasicVector<T> model_value(size);
  this->DeclareVectorInputPort(model_value);
  this->DeclareVectorOutputPort(
      model_value, &ZeroOrderHold::DoCalcVectorOutput);
  this->DeclareDiscreteState(size);
  this->DeclarePeriodicDiscreteUpdate(period_sec);
}

template <typename T>
void ZeroOrderHold<T>::Construct(
    double period_sec, std::unique_ptr<const AbstractValue> model_value) {
  abstract_model_value_ = std::move(model_value);
  // TODO(eric.cousineau): Remove value parameter from the constructor once
  // the equivalent of #3109 for abstract values is also resolved.
  this->DeclareAbstractInputPort(*abstract_model_value_);
  // Use the std::function<> overloads to work with `AbstractValue` type
  // directly and maintain type erasure.
  auto allocate_abstract_value = [this](const Context<T>&) {
    return abstract_model_value_->Clone();
  };
  namespace sp = std::placeholders;
  this->DeclareAbstractOutputPort(
      allocate_abstract_value,
      std::bind(&ZeroOrderHold::DoCalcAbstractOutput, this, sp::_1, sp::_2));
  this->DeclareAbstractState(abstract_model_value_->Clone());
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0.);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = *context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& input_value = *this->EvalVectorInput(context, 0);
  BasicVector<T>& state_value = *discrete_state->get_mutable_vector(0);
  state_value.SetFrom(input_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcAbstractOutput(const Context<T>& context,
                                            AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  // Do not use `get_abstract_state<AbstractValue>` since it would cast
  // the value to `Value<AbstractValue>`, which is an invalid type by design.
  const AbstractValue& state_value =
      context.template get_abstract_state()->get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T> *state) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& input_value = *this->EvalAbstractInput(context, 0);
  // See `DoCalcAbstractOutput` for rationale regarding non-templated value
  // accessor.
  AbstractValue& state_value =
      state->get_mutable_abstract_state()->get_mutable_value(0);
  state_value.SetFrom(input_value);
}

template <typename T>
optional<bool> ZeroOrderHold<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a zero-order hold will not have direct feedthrough, as the
  // output only depends on the state, not the input.
  return false;
}

}  // namespace systems
}  // namespace drake
