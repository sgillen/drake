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

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(AbstractZOHTest, ModelAbstractState) {
  class SimpleSystem : public LeafSystem<double> {
   public:
    SimpleSystem() {
      DeclareAbstractOutputPort(Value<TestValue<double>>());
    }
    void DoCalcOutput(
          const Context<double>& context,
          SystemOutput<double>* output) const override {
      auto&& y = output->GetMutableData(0)->GetMutableValue<TestValue<double>>();
      y.value() = context.get_time();
    }
  };
  class PeriodicQuery : public LeafSystem<double> {
   public:
    PeriodicQuery() {
      DeclareAbstractInputPort();
      DeclarePublishPeriodSec(0.001);
    }
    void DoPublish(const Context<double> &context) const override {
      // Query input.
      auto&& y = EvalAbstractInput(context, 0)->GetValue<TestValue<double>>();
      cout << "Periodic Query: " << context.get_time() << ": "
           << y.value() << endl;
    }
    void DoCalcOutput(
          const Context<double>&, SystemOutput<double>*) const override {}
  };

  DiagramBuilder<double> builder;
  auto* sys = builder.AddSystem<SimpleSystem>();
  auto* zoh = builder.AddSystem<AbstractZOH<TestValue<double>>>(0.1);
  vector<double> times;
  vector<double> values;
  zoh->ConnectOnUpdate([&](double t, const TestValue<double>& value) {
    times.push_back(t);
    values.push_back(value.value());
  });
  builder.Connect(sys->get_output_port(0),
                  zoh->get_input_port(0));
  auto* query = builder.AddSystem<PeriodicQuery>();
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

}  // namespace
}  // namespace systems
}  // namespace drake
