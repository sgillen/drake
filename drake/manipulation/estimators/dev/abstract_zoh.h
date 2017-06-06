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
    value_ = value;
  }
  bool has_value() const {
    return value_ != nullopt;
  }
  T& value() {
    DRAKE_DEMAND(has_value());
    return *value_;
  }
  const T& value() const {
    DRAKE_DEMAND(has_value());
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

  AbstractZOH(double period_sec, double offset_sec = 0.,
              bool permit_autoinit = true)
      : permit_autoinit_(permit_autoinit) {
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

  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {
    std::cout << "SetDefaultState" << std::endl;
    DoCalcUnrestrictedUpdate(context, state);
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    const Data& stored_value =
        context.get_abstract_state()->get_value(0).GetValue<Data>();
    Data& output_value =
      output->GetMutableData(0)->GetMutableValue<Data>();
    if (!stored_value.has_value()) {
      // HACK(eric.cousineau): Figure out how to resolve this.
      // NOTE: Presently will not be useful until all ports are cached.
      throw std::runtime_error("Not implemented");
//      std::cout << "HACCCK" << std::endl;
//      auto& mutable_context = const_cast<Context<double>&>(context);
//      const T& input_value =
//          EvalAbstractInput(mutable_context, 0)->template GetValue<T>();
//      Data& mutable_stored_value =
//          mutable_context.template get_mutable_abstract_state<Data>(0);
//      mutable_stored_value.set_value(input_value);
//      output_value.set_value(mutable_stored_value.value());
    } else {
      output_value.set_value(stored_value.value());
    }
  }
 private:
  OnUpdate on_update_;
  bool permit_autoinit_{};
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
