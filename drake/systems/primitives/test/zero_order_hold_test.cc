#include "drake/systems/primitives/zero_order_hold.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const int kLength = 3;

struct SimpleType {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimpleType);

  SimpleType(const Eigen::Vector3d& value)
      : value(value) {}

  Eigen::Vector3d value;
};

class ZeroOrderHoldTest : public ::testing::TestWithParam<bool> {
 protected:
  ZeroOrderHoldTest()
      : is_abstract_(GetParam()) {}
  void SetUp() override {
    state_override_ << 1.0, 3.14, 2.18;
    input_ << 1.0, 1.0, 3.0;

    if (!is_abstract_) {
      hold_ = std::make_unique<ZeroOrderHold<double>>(kTenHertz, kLength);
      action_type_ = DiscreteEvent<double>::kDiscreteUpdateAction;
    } else {
      // Reflect initial state of NaN.
      const double nan = std::numeric_limits<double>::quiet_NaN();
      hold_ = std::make_unique<ZeroOrderHold<double>>(
          kTenHertz, Value<SimpleType>(Eigen::Vector3d::Constant(nan)));
      action_type_ = DiscreteEvent<double>::kUnrestrictedUpdateAction;
    }
    context_ = hold_->CreateDefaultContext();
    output_ = hold_->AllocateOutput(*context_);
    if (!is_abstract_) {
      context_->FixInputPort(
          0, std::make_unique<BasicVector<double>>(input_));
    } else {
      context_->FixInputPort(
          0, AbstractValue::Make<SimpleType>(input_));
    }
  }

  std::unique_ptr<System<double>> hold_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;

  const bool is_abstract_{};
  DiscreteEvent<double>::ActionType action_type_;
  Eigen::Vector3d state_override_;
  Eigen::Vector3d input_;
};

// Tests that the zero-order hold has one input and one output.
TEST_P(ZeroOrderHoldTest, Topology) {
  EXPECT_EQ(1, hold_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());

  EXPECT_EQ(1, output_->get_num_ports());
  EXPECT_EQ(1, hold_->get_num_output_ports());

  if (!is_abstract_) {
    // Do not test direct feedthrough for abstract types, given that
    // `DoToSymbolic` is not supported in this case.
    EXPECT_FALSE(hold_->HasAnyDirectFeedthrough());
  }
}

// Tests that the zero-order hold has discrete state.
TEST_P(ZeroOrderHoldTest, ReservesState) {
  if (!is_abstract_) {
    const VectorBase<double>* xd = context_->get_discrete_state(0);
    ASSERT_NE(nullptr, xd);
    EXPECT_EQ(kLength, xd->size());
  } else {
    const SimpleType& state_value = context_->get_abstract_state<SimpleType>(0);
    const Eigen::Vector3d value = state_value.value;
    EXPECT_TRUE((value.array() != value.array()).all());
  }
}

// Tests that the output is the state.
TEST_P(ZeroOrderHoldTest, Output) {
  Eigen::Vector3d output;
  Eigen::Vector3d output_expected = state_override_;
  if (!is_abstract_) {
    BasicVector<double>* xd = dynamic_cast<BasicVector<double>*>(
        context_->get_mutable_discrete_state(0));
    xd->get_mutable_value() << output_expected;

    hold_->CalcOutput(*context_, output_.get());

    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    ASSERT_NE(nullptr, output_vector);
    output = output_vector->CopyToVector();
  } else {
    SimpleType& state_value =
        context_->get_mutable_abstract_state<SimpleType>(0);
    state_value = SimpleType{output_expected};

    hold_->CalcOutput(*context_, output_.get());

    output = output_->get_data(0)->GetValue<SimpleType>().value;
  }
  EXPECT_EQ(output_expected, output);
}

// Tests that when the current time is exactly on the sampling period, a update
// is requested in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeMustNotBeCurrentTime) {
  // Calculate the next update time.
  context_->set_time(0.0);
  UpdateActions<double> actions;
  double next_t = hold_->CalcNextUpdateTime(*context_, &actions);

  // Check that the time is correct.
  EXPECT_NEAR(0.1, next_t, 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<double>& event = actions.events[0];
  EXPECT_EQ(action_type_, event.action);
}

// Tests that when the current time is between updates, a update is requested
// at the appropriate time in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeIsInTheFuture) {
  // Calculate the next update time.
  context_->set_time(76.32);
  UpdateActions<double> actions;

  // Check that the time is correct.
  double next_t = hold_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_NEAR(76.4, next_t, 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<double>& event = actions.events[0];
  EXPECT_EQ(action_type_, event.action);
}

// Tests that discrete updates update the state.
TEST_P(ZeroOrderHoldTest, Update) {
  // Fire off an update event.
  DiscreteEvent<double> update_event;
  update_event.action = action_type_;

  Eigen::Vector3d value;
  if (!is_abstract_) {
    std::unique_ptr<DiscreteValues<double>> update =
        hold_->AllocateDiscreteVariables();
    hold_->CalcDiscreteVariableUpdates(*context_, {update_event}, update.get());
    // Check that the state has been updated to the input.
    const VectorBase<double>* xd = update->get_vector(0);
    value = xd->CopyToVector();
  } else {
    State<double>* state = context_->get_mutable_state();
    hold_->CalcUnrestrictedUpdate(*context_, update_event, state);
    value = state->get_abstract_state<SimpleType>(0).value;
  }
  EXPECT_EQ(input_, value);
}

INSTANTIATE_TEST_CASE_P(test, ZeroOrderHoldTest,
    ::testing::Values(false, true));

class SymbolicZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double period_sec = 0.5;
    const int size = 1;
    hold_ = std::make_unique<ZeroOrderHold<symbolic::Expression>>(period_sec,
                                                                  size);

    // Initialize the context with symbolic variables.
    context_ = hold_->CreateDefaultContext();
    context_->FixInputPort(0, BasicVector<symbolic::Expression>::Make(
        symbolic::Variable("u0")));
    auto& xd = *context_->get_mutable_discrete_state(0);
    xd[0] = symbolic::Variable("x0");

    output_ = hold_->AllocateOutput(*context_);
    update_ = hold_->AllocateDiscreteVariables();
  }

  std::unique_ptr<ZeroOrderHold<symbolic::Expression>> hold_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
  std::unique_ptr<SystemOutput<symbolic::Expression>> output_;
  std::unique_ptr<DiscreteValues<symbolic::Expression>> update_;
};

TEST_F(SymbolicZeroOrderHoldTest, Output) {
  hold_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(1, output_->get_num_ports());
  const auto& out = *output_->get_vector_data(0);
  EXPECT_EQ("x0", out[0].to_string());
}

TEST_F(SymbolicZeroOrderHoldTest, Update) {
  DiscreteEvent<symbolic::Expression> update_event;
  update_event.action =
      DiscreteEvent<symbolic::Expression>::kDiscreteUpdateAction;

  hold_->CalcDiscreteVariableUpdates(*context_, {update_event}, update_.get());
  const auto& xd = *update_->get_vector(0);
  EXPECT_EQ("u0", xd[0].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
