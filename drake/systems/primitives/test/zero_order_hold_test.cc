#include "drake/systems/primitives/zero_order_hold.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/extract_double.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/output_port_value.h"

using std::shared_ptr;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const int kLength = 3;

template <typename S>
struct TestBase {
  using deferred_false =
      std::integral_constant<bool, !std::is_same<S, S>::value>;
  static_assert(deferred_false::value, "This should not be called.");
};

template <typename T_>
class VectorTestBase : public ::testing::Test {
 public:
  using T = T_;
  using system = ZeroOrderHold<T>;
  using value_type = BasicVector<T>;  // Wrapped value type.

  VectorX<T> fixed_input_;
  VectorX<T> state_update_;
  std::unique_ptr<System<T>> hold_;
  std::unique_ptr<Context<T>> context_;
  std::unique_ptr<SystemOutput<T>> output_;

  virtual void SetValues() = 0;

  void SetUp() override {
    SetValues();
    hold_ = std::make_unique<ZeroOrderHold<T>>(kTenHertz, kLength);
    context_ = hold_->CreateDefaultContext();
    output_ = hold_->AllocateOutput(*context_);
    context_->FixInputPort(0, make_unique<BasicVector<T>>(fixed_input_));
  }

  void UpdateState() {
    BasicVector<T>* xd = dynamic_cast<BasicVector<T>*>(
        context_->get_mutable_discrete_state(0));
    xd->get_mutable_value() << state_update_;
  }

  void CheckOutput(const VectorX<T>& value) {
    const BasicVector<T>* output_vector = output_->get_vector_data(0);
    ASSERT_NE(nullptr, output_vector);
    EXPECT_EQ(value(0), output_vector->GetAtIndex(0));
    EXPECT_EQ(value(1), output_vector->GetAtIndex(1));
    EXPECT_EQ(value(2), output_vector->GetAtIndex(2));
  }

  void DispatchUpdate() {
    DiscreteEvent<T> update_event;
    update_event.action = GetActionType();
    std::unique_ptr<DiscreteValues<T>> update =
        hold_->AllocateDiscreteVariables();
    hold_->CalcDiscreteVariableUpdates(*context_, {update_event}, update.get());
  }

  void CheckReservesState() {
    const VectorBase<T>* xd = context_->get_discrete_state(0);
    ASSERT_NE(nullptr, xd);
    EXPECT_EQ(kLength, xd->size());
  }

  typename DiscreteEvent<T>::ActionType GetActionType() const {
    return DiscreteEvent<T>::kDiscreteUpdateAction;
  }
};

template <>
class TestBase<ZeroOrderHold<double>> : public VectorTestBase<double> {
 protected:
  void SetValues() override {
    fixed_input_.resize(kLength);
    fixed_input_ << 1.0, 1.0, 3.0;
    state_update_.resize(kLength);
    state_update_ << 1.0, 3.14, 2.18;
  }
};

template <>
class TestBase<ZeroOrderHold<symbolic::Expression>>
  : public VectorTestBase<symbolic::Expression> {
 protected:
  void SetValues() override {
    using Var = symbolic::Variable;
    fixed_input_.resize(kLength);
    fixed_input_ << Var("x0"), Var("x1"), Var("x2");
    state_update_.resize(kLength);
    state_update_ << Var("u0"), Var("u1"), Var("u2");
  }
};

template <typename S>
class ZeroOrderHoldTest : public TestBase<S> {
 public:
//  typedef typename TestBase<S>::T T;
};

// // Simple trivially constructible class, but not copyable.
// class TriviallyConstructible {
//  public:
//   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TriviallyConstructible)

//   TriviallyConstructible() {}
//   explicit TriviallyConstructible(double value)
//     : value_(value) {}
//   double value() const { return value_; }
//   void set_value(double value) { value_ = value; }
//  private:
//   double value_;
// };

// // Nontrivially constructible class.
// class NontriviallyConstructible {
//  public:
//   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TriviallyConstructible)

//   explicit NontriviallyConstructible(double value)
//     : value_(value) {}
//   double value() const { return value_; }
//   void set_value(double value) { value_ = value; }
//  private:
//   double value_;
// };

typedef ::testing::Types<
  ZeroOrderHold<double>,
  ZeroOrderHold<symbolic::Expression>
  > ZeroOrderHoldTypes;
TYPED_TEST_CASE(ZeroOrderHoldTest, ZeroOrderHoldTypes);

// Tests that the zero-order hold has one input and one output.
TYPED_TEST(ZeroOrderHoldTest, Topology) {
  EXPECT_EQ(1, this->hold_->get_num_input_ports());
  EXPECT_EQ(1, this->context_->get_num_input_ports());

  EXPECT_EQ(1, this->output_->get_num_ports());
  EXPECT_EQ(1, this->hold_->get_num_output_ports());

  EXPECT_FALSE(this->hold_->HasAnyDirectFeedthrough());
}

// Tests that the zero-order hold has discrete state.
TYPED_TEST(ZeroOrderHoldTest, ReservesState) {
  this->CheckReservesState();
}

// Tests that the output is the state.
TYPED_TEST(ZeroOrderHoldTest, Output) {
  this->UpdateState();
  this->hold_->CalcOutput(*this->context_, this->output_.get());
  this->CheckOutput(this->state_update_);
}

// Tests that when the current time is exactly on the sampling period, a update
// is requested in the future.
TYPED_TEST(ZeroOrderHoldTest, NextUpdateTimeMustNotBeCurrentTime) {
  // HACK: Is this part of the public GTest API?
  typedef typename TestFixture::T T;

  // Calculate the next update time.
  this->context_->set_time(0.0);
  UpdateActions<T> actions;
  T next_t = this->hold_->CalcNextUpdateTime(*this->context_, &actions);

  // Check that the time is correct.
  EXPECT_NEAR(0.1, ExtractDoubleOrThrow(next_t), 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<T>& event = actions.events[0];
  EXPECT_EQ(this->GetActionType(), event.action);
}

// Tests that when the current time is between updates, a update is requested
// at the appropriate time in the future.
TYPED_TEST(ZeroOrderHoldTest, NextUpdateTimeIsInTheFuture) {
  typedef typename TestFixture::T T;

  // Calculate the next update time.
  this->context_->set_time(76.32);
  UpdateActions<T> actions;

  // Check that the time is correct.
  T next_t = this->hold_->CalcNextUpdateTime(*this->context_, &actions);
  EXPECT_NEAR(76.4, ExtractDoubleOrThrow(next_t), 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<T>& event = actions.events[0];
  EXPECT_EQ(this->GetActionType(), event.action);
}

// Tests that discrete updates update the state.
TYPED_TEST(ZeroOrderHoldTest, Update) {
  // Fire off an update event.
  this->DispatchUpdate();
  // Check that the state has been updated to the input.
  this->CheckOutput(this->fixed_input_);
}

}  // namespace
}  // namespace systems
}  // namespace drake
