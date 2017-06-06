#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// Assume all are copy-and-move constructible.
// More permissive than Value, in that it allows default construction
// (but will puke if it is null.)
// TODO: Consider using std::optional? Does it permit non-default constructible
// objects?
template <typename T>
class DefaultData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultData);
  DefaultData() {}
  void set_value(const T& value) {
    *value_ = value;
  }
  T& value() {
    DRAKE_DEMAND(value_ != nullopt);
    return *value_;
  }
  const T& value() const {
    DRAKE_DEMAND(value_ != nullopt);
    return *value_;
  }
 private:
  optional<T> value_;
//  using Traits = systems::value_detail::ValueTraits<T>;
//  typename Traits::Storage value_;
};

template <typename T>
class AbstractZOH : public LeafSystem<double> {
 public:
  typedef DefaultData<T> Data;
  typedef std::function<void(double time, const T& value)> OnUpdate;

  AbstractZOH(double period_sec, double offset_sec = 0.) {
    // Using LcmSubscriberSystem as a basis
    // This will receive Value<T>, not Value<Data>.
    this->DeclareAbstractInputPort();
    // TODO(eric.cousineau): Is there a way to not care about the type?
    // And ignore using DefaultData<> altogether?
    this->DeclareAbstractState(std::make_unique<systems::Value<Data>>());
    this->DeclareAbstractOutputPort(systems::Value<Data>());
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
    Data& stored_value =
        state->get_mutable_abstract_state()
            ->get_mutable_value(0).GetMutableValue<Data>();
    stored_value.set_value(input_value);
    if (on_update_) {
      on_update_(context.get_time(), input_value);
    }
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    const Data& stored_value =
        context.get_abstract_state()->get_value(0).GetValue<Data>();
    Data& output_value =
        output->GetMutableData(0)->GetMutableValue<Data>();
    output_value.set_value(stored_value.value());
  }
 private:
  OnUpdate on_update_;
};


template <typename Visitor>
void pack_visit(Visitor&& visitor) {}

template <typename Visitor, typename T, typename... Ts>
void pack_visit(Visitor&& visitor) {
  visitor.template run<T>();
  pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
};

// To infer caller type
template <typename... Ts>
struct pack_visitor {
  template <typename Visitor>
  static void run(Visitor&& visitor) {
    pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
  }
};

template <typename... Ts>
class AbstractZOHDiagram : public systems::Diagram<double> {
 public:
  using T = double;
  using Builder = systems::DiagramBuilder<T>;

  AbstractZOHDiagram(double period_sec)
      : period_sec_(period_sec) {
    Builder builder;
    pack_visitor<Ts...>::run(Adder{this, &builder});
    builder.BuildInto(this);
  }

 protected:
  struct Adder {
    AbstractZOHDiagram* self;
    Builder* builder;
    template <typename T>
    void run() {
      auto* zoh = builder->template AddSystem<AbstractZOH<T>>(self->period_sec_);
      builder->ExportInput(zoh->get_input_port(0));
      builder->ExportOutput(zoh->get_output_port(0));
    }
  };

 private:
  double period_sec_;
};

}  // namespace systems
}  // namespace drake
