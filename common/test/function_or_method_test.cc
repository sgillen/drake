#include "drake/common/function_or_method.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// Example base with declare-like methods.
class Base {
 public:
  virtual ~Base() {}

  using NoReturn = FunctionOrMethod<Base, false, void, int>;

  NoReturn::Function DeclareNoReturn(NoReturn func) {
    return func.get(this);
  }

  using WithReturn = FunctionOrMethod<Base, true, int>;

  WithReturn::Function DeclareWithReturn(WithReturn func) {
    return func.get(this);
  }
};

// Example implementation.
class Child : public Base {
 public:
  explicit Child(int* side_effect) : side_effect_(side_effect) {}

  void ExampleNoReturn(int x) const {
    *side_effect_ += x;
  }

  int ExampleWithReturn() {
    return *side_effect_ * 2;
  }

  int* side_effect_{};
};

GTEST_TEST(FunctionOrMethodTest, Basic) {
  int value{10};
  Child child(&value);
  // Check method: addition.
  std::function<void(int)> no_return =
      child.DeclareNoReturn(&Child::ExampleNoReturn);
  no_return(1);
  EXPECT_EQ(value, 11);
  // Check function: subtraction.
  no_return = child.DeclareNoReturn([&value](int x) { value -= x; });
  no_return(1);
  EXPECT_EQ(value, 10);

  // // Check method: multiplication.
  std::function<int()> with_return =
      child.DeclareWithReturn(&Child::ExampleWithReturn);
  // EXPECT_EQ(with_return(), 20);
  // // Check function: division.
  with_return = child.DeclareWithReturn([value]() { return value / 2; });
  EXPECT_EQ(with_return(), 5);
  (void)child;
}

}  // namespace
}  // namespace drake
