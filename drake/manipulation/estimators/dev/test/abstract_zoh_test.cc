#include "drake/manipulation/estimators/dev/abstract_zoh.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
class TestValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestValue);
  TestValue() {}
  TestValue(const T& value)
      : value_(value) {}
  const T& value() const { return value_; }
  T& value() { return value_; }
 private:
  T value_{};
};

using namespace std;

template <typename T>
class SimpleSystem : public LeafSystem<double> {
 public:
  SimpleSystem() {
    DeclareAbstractOutputPort(Value<TestValue<T>>());
  }
  void DoCalcOutput(
        const Context<double>& context,
        SystemOutput<double>* output) const override {
    cout << "SimpleSystem::CalcOutput" << endl;
    auto&& y = output->GetMutableData(0)->GetMutableValue<TestValue<T>>();
    y.value() = context.get_time();
  }
};

template <typename T>
class PeriodicQuery : public LeafSystem<double> {
 public:
  PeriodicQuery() {
    // TODO(eric.cousineau): Figure out why this is publishing every timestep
    // (@ 0.001s) versus printing at desired rate (@ 0.05s).
    DeclareAbstractInputPort();
    DeclarePublishPeriodSec(0.05);
  }
  void DoPublish(const Context<double> &context) const override {
    // Query input.
    auto&& y = EvalAbstractInput(context, 0)->template GetValue<TestValue<T>>();
    cout << "Periodic Query: " << context.get_time() << ": "
         << y.value() << endl;
  }
  void DoCalcOutput(
        const Context<double>&, SystemOutput<double>*) const override {}
};

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(AbstractZOHTest, ModelAbstractState) {
  const double dt = 0.1;

  DiagramBuilder<double> builder;
  auto* sys = builder.AddSystem<SimpleSystem<double>>();
  auto* zoh = builder.AddSystem<AbstractZOH<TestValue<double>>>(dt);
  vector<double> times;
  vector<double> values;
  zoh->ConnectOnUpdate([&](double t, const TestValue<double>& value) {
    times.push_back(t);
    values.push_back(value.value());
  });
  builder.Connect(sys->get_output_port(0),
                  zoh->get_input_port(0));
  auto* query = builder.AddSystem<PeriodicQuery<double>>();
  builder.Connect(zoh->get_output_port(0),
                  query->get_input_port(0));

  auto full_sys = builder.Build();
  Simulator<double> simulator(*full_sys);
  simulator.Initialize();
  simulator.StepTo(0.2);

  vector<double> times_expected{0.1, 0.2};
  vector<double> values_expected = times_expected;
  EXPECT_EQ(times_expected, times);
  EXPECT_EQ(values_expected, values);
}

// Tests that DeclareAbstractState works expectedly.
// WARNING(eric.cousineau): Stack ZOH is not a good idea, at least in the short
// term, since the entire system will be evaluated per port evaluation.
GTEST_TEST(AbstractZOHTest, StackTest) {
  DiagramBuilder<double> builder;
  auto* sys_stack = builder.AddSystem(
      StackSystems<double>(false, true,
            {new SimpleSystem<double>(), new SimpleSystem<float>()}));

  const double dt = 0.1;
  auto* zoh_stack = builder.AddSystem(
      StackSystems<double>(true, true,
           {new AbstractZOH<TestValue<double>>(dt),
            new AbstractZOH<TestValue<float>>(dt)}));
  auto* query_stack = builder.AddSystem(
      StackSystems<double>(true, false,
           {new PeriodicQuery<double>(),
            new PeriodicQuery<float>()}));

  for (int i = 0; i < 2; ++i) {
    builder.Connect(sys_stack->get_output_port(i),
                    zoh_stack->get_input_port(i));
    builder.Connect(zoh_stack->get_output_port(i),
                    query_stack->get_input_port(i));
  }

  auto full_sys = builder.Build();
  Simulator<double> simulator(*full_sys);
  simulator.Initialize();
  simulator.StepTo(0.2);
}

}  // namespace
}  // namespace systems
}  // namespace drake
