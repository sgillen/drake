#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/common/unused.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(
    int size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : vector_size_(size),
      abstract_model_value_(std::move(abstract_model_value)) {
  DRAKE_DEMAND(vector_size_ == -1 || abstract_model_value_ == nullptr);
  if (!is_abstract()) {
    BasicVector<T> model_value(size);
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(
        model_value, &PassThrough::DoCalcVectorOutput);
  } else {
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
        std::bind(&PassThrough::DoCalcAbstractOutput, this, sp::_1, sp::_2));
  }
}

inline std::unique_ptr<AbstractValue> CloneIfNotNull(
    std::unique_ptr<const AbstractValue> value) {
  return value ? value->Clone() : nullptr;
}

template <typename T>
template <typename U>
PassThrough<T>::PassThrough(const PassThrough<U>& other)
    : PassThrough(other.vector_size,
                  CloneIfNotNull(other.abstract_value_)) {}

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& input = *this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(input.size() == output->size());
  output->SetFrom(input);
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& input =
      *this->EvalAbstractInput(context, 0);
  output->SetFrom(input);
}

template <typename T>
optional<bool> PassThrough<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a pass-through will have direct feedthrough, as the
  // output depends directly on the input.
  return true;
}

}  // namespace systems
}  // namespace drake
