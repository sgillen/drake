// Copied from: leaf_system_test.cc

#include "drake/systems/framework/leaf_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

class TestValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestValue);

  TestValue() {}
  TestValue(int value)
      : value_(value) {}
  const int& value() const { return value_; }
  int& value() { return value_; }
 private:
  int value_{};
};

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(ModelLeafSystemTest, ModelAbstractState) {
  class SimpleStateSystem : public LeafSystem<double> {
   public:
    SimpleStateSystem() {
      DeclareAbstractState(AbstractValue::Make<TestValue>(0));
      DeclareDiscreteState(1);
      DeclareOutputPort(kVectorValued, 2);
      DeclarePeriodicUnrestrictedUpdate(0.1, 0.);
    }
    void DoCalcUnrestrictedUpdate(
          const Context<double>& context,
          State<double>* state) const override {
      auto& test = state->get_mutable_abstract_state<TestValue>(0);
      test.value() += 2;
      auto&& x =
          state->get_mutable_discrete_state()
          ->get_mutable_vector(0)->get_mutable_value();
      x[0] += 4;
    }
    void DoCalcOutput(
          const Context<double>& context,
          SystemOutput<double>* output) const override {
      auto&& test = context.get_abstract_state<TestValue>(0);
      auto&& x =
          context.get_discrete_state(0)->get_value();

      auto&& y = output->GetMutableVectorData(0)->get_mutable_value();
      y[0] = test.value();
      y[1] = x[0];
    }
  };

  SimpleStateSystem dut;
  auto context = dut.CreateDefaultContext();

  Simulator<double> simulator(dut);
  simulator.Initialize();
  simulator.StepTo(0.2);

  EXPECT_EQ(context->get_abstract_state<int>(0), 1);
  EXPECT_EQ(context->get_abstract_state<std::string>(1), "wow");
}

}  // namespace
}  // namespace systems
}  // namespace drake
