#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

//// Assume all are copy-and-move constructible.
//// More permissive than Value, in that it allows default construction
//// (but will puke if it is null.)
//// TODO: Consider using std::optional? Does it permit non-default constructible
//// objects?
//template <typename T>
//class DefaultValue : public AbstractValue {
// public:
//  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultValue);
//  DefaultValue() {}
//  void SetFrom(const AbstractValue& other) override {
//    // First, assume that the input is of type Value<T>.
//    const auto* value_ptr = dynamic_cast<const Value<T>*>(&other);
//    if (value_ptr) {
//      value_ = *value_ptr;
//    } else {
//      DefaultValue* out = new DefaultValue();
//      out->value_.emplace
//      const auto* default_ptr = dynamic_cast<const DefaultValue*>(&other);
//    }
//  }
//  void SetFromOrThrow(const AbstractValue& other) override {
//    // Lazy
//    SetFrom(other);
//  }
//  std::unique_ptr<AbstractValue> Clone() const override {
//    return new DefaultValue<T>(*this);
//  }
//  void set_value(const T& value) {
//    value_ = Value<T>(value);
//  }
//  bool has_value() const {
//    return value_ != nullopt;
//  }
//  T& value() {
//    DRAKE_DEMAND(has_value());
//    return value_->get_mutable_value();
//  }
//  const T& value() const {
//    DRAKE_DEMAND(has_value());
//    return value_->get_value();
//  }
// protected:
//  const AbstractValue* GetUserValue() const override {
//    return &value_.value();
//  }
// private:
//  optional<Value<T>> value_;
////  using Traits = systems::value_detail::ValueTraits<T>;
////  typename Traits::Storage value_;
//};

template <typename T>
class AbstractZOH : public LeafSystem<double> {
 public:
  typedef std::function<void(double time, const T& value)> OnUpdate;

  AbstractZOH(double period_sec, bool use_autoinit = false)
      : AbstractZOH(T(), period_sec, 0., use_autoinit) {}

  AbstractZOH(const T& ic, double period_sec, double offset_sec = 0.,
              bool use_autoinit = false)
      : use_autoinit_(use_autoinit) {
    // Using LcmSubscriberSystem as a basis
    // This will receive Value<T>, not Value<Data>.
    this->DeclareAbstractInputPort();
    // TODO(eric.cousineau): Is there a way to not care about the default constructor,
    // and just inherit this from an upstream system?
    this->DeclareAbstractState(std::make_unique<Value<T>>(ic));
    this->DeclareAbstractOutputPort(Value<T>(ic));
    DeclarePeriodicUnrestrictedUpdate(period_sec, offset_sec);
  }

  void ConnectOnUpdate(const OnUpdate& on_update) {
    DRAKE_ASSERT(!on_update_);
    on_update_ = on_update;
  }

 protected:
  void DoCalcUnrestrictedUpdate(const Context<double>& context,
                                State<double>* state) const override {
    const T& input_value =
        EvalAbstractInput(context, 0)->template GetValue<T>();
    T& stored_value =
        state->get_mutable_abstract_state()
            ->get_mutable_value(0).GetMutableValue<T>();
    stored_value = input_value;
    if (on_update_) {
      on_update_(context.get_time(), input_value);
    }
  }

  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {
    if (use_autoinit_) {
      // Update initial values with the ICs from upstream blocks.
      DoCalcUnrestrictedUpdate(context, state);
    } else {
      LeafSystem<double>::SetDefaultState(context, state);
    }
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    const T& stored_value =
        context.get_abstract_state()->get_value(0).GetValue<T>();
    T& output_value =
      output->GetMutableData(0)->GetMutableValue<T>();
    output_value = stored_value;
  }
 private:
  OnUpdate on_update_;
  bool use_autoinit_{};
};

//template <typename T, typename... Extra>
//std::unique_ptr<AbstractZOH<T>> MakeAbstractZOH(const T& ic, Extra&&... extra) {
//  return std::make_unique<AbstractZOH<T>>(ic, std::forward<Extra>(extra)...);
//}

//template <typename Visitor>
//void pack_visit(Visitor&& visitor) {}

//// Visit a set of parameter pack template arguments, using Visitor::run<T>
//template <typename Visitor, typename T, typename... Ts>
//void pack_visit(Visitor&& visitor) {
//  visitor.template run<T>();
//  pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
//};

//// To infer caller type
//template <typename... Ts>
//struct pack_visitor {
//  template <typename Visitor>
//  static void run(Visitor&& visitor) {
//    pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
//  }
//};

//template <typename Visitor>
//void param_visit(Visitor&& visitor) {}

//// Visit a set of parameter, using Visitor::run<T>(T&& t)
//template <typename Visitor, typename T, typename... Ts>
//void param_visit(Visitor&& visitor, T&& arg, Ts&&... args) {
//  visitor(std::forward<T>(arg));
//  pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor),
//                             std::forward<Ts>(args)...);
//};

/**
 * Stack a group of SISO systems.
 * Example:
 *   auto* stack = StackSystems(true, true, {
 *     new AbstractZOH(...),
 *     new AbstractZOH(...)});
 */
template <typename T>
std::unique_ptr<Diagram<T>> StackSystems(
    bool export_input, bool export_output,
    const std::vector<LeafSystem<T>*>& systems) {
  DRAKE_ASSERT(export_input || export_output);
  DiagramBuilder<T> builder;
  for (LeafSystem<T>* system : systems) {
    builder.AddSystem(std::unique_ptr<LeafSystem<T>>(system));
    if (export_input) {
      builder.ExportInput(system->get_input_port(0));
    }
    if (export_output) {
      builder.ExportOutput(system->get_output_port(0));
    }
  }
  return builder.Build();
}

//template <typename... Ts>
//class AbstractZOHDiagram : public systems::Diagram<double> {
// public:
//  using T = double;
//  using Builder = systems::DiagramBuilder<T>;

//  AbstractZOHDiagram(double period_sec, const Ts&... ic)
//      : period_sec_(period_sec) {
//    Builder builder;
//    pack_visitor<Ts...>::run(Adder{this, &builder});
//    builder.BuildInto(this);
//  }

// protected:
//  struct Adder {
//    AbstractZOHDiagram* self;
//    Builder* builder;
//    template <typename T>
//    void run() {
//      auto* zoh = builder->template AddSystem<AbstractZOH<T>>(self->period_sec_);
//      builder->ExportInput(zoh->get_input_port(0));
//      builder->ExportOutput(zoh->get_output_port(0));
//    }
//  };

// private:
//  double period_sec_;
//};

}  // namespace systems
}  // namespace drake
