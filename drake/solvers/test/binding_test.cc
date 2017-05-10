#include "drake/solvers/binding.h"

#include <gtest/gtest.h>

#include "drake/common/test/symbolic_test_util.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
namespace test {

using std::make_shared;
using std::vector;
using std::equal_to;

using internal::BindingBundle;
using internal::CreateBinding;
using symbolic::Variable;
using symbolic::test::VarEqual;

template <typename C>
void ExpectEq(const Binding<C>& lhs, const Binding<C>& rhs) {
  EXPECT_EQ(lhs.constraint(), rhs.constraint());
  EXPECT_EQ(lhs.variables(), rhs.variables());
}
template <typename C>
void ExpectEq(const vector<C>& lhs, const vector<C>& rhs) {
  EXPECT_EQ(lhs.size(), rhs.size());
  for (size_t i = 0; i < lhs.size(); ++i) {
    ExpectEq(lhs[i], rhs[i]);
  }
}

GTEST_TEST(TestBinding, constructBinding) {
  symbolic::Variable x1("x1");
  symbolic::Variable x2("x2");
  symbolic::Variable x3("x3");
  auto bb_con = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding<BoundingBoxConstraint> binding1(
      bb_con,
      {VectorDecisionVariable<2>(x3, x1), VectorDecisionVariable<1>(x2)});
  EXPECT_EQ(binding1.GetNumElements(), 3u);
  VectorDecisionVariable<3> var1_expected(x3, x1, x2);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding1.variables()(i), var1_expected(i));
  }

  // Creates a binding with a single VectorDecisionVariable.
  Binding<BoundingBoxConstraint> binding2(
      bb_con, VectorDecisionVariable<3>(x3, x1, x2));
  EXPECT_EQ(binding2.GetNumElements(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(binding2.variables()(i), var1_expected(i));
  }
}

GTEST_TEST(TestBinding, constructBindingBundle) {
  // Ensure that we have various ways to construct binding bundles
  auto x = VectorDecisionVariable<1>(Variable("x"));
  auto binding = CreateBinding(make_shared<BoundingBoxConstraint>(
      Vector1d(-1), Vector1d(1)), x);
  Binding<Constraint> extra = CreateBinding(binding.constraint(), x);
  vector<Binding<Constraint>> extra_bindings = {extra, extra};

  {
    // Standard case, no extra variables, explicitly specified extra type.
    BindingBundle<BoundingBoxConstraint, Constraint> binding_bundle = {
        binding,
        {extra, extra}
    };
    ExpectEq(binding, binding_bundle.binding);
    ExpectEq(extra_bindings, binding_bundle.extra_bindings);
    EXPECT_EQ(0, binding_bundle.new_vars.size());
    EXPECT_EQ(VarType::CONTINUOUS, binding_bundle.new_vars_type);
  }

  {
    // Implicitly use the same type for extra bindings
    vector<Binding<BoundingBoxConstraint>> extra_bindings = {binding, binding,
                                                           binding};
    BindingBundle<BoundingBoxConstraint> binding_bundle = {
        binding,
        extra_bindings
    };
    ExpectEq(binding, binding_bundle.binding);
    ExpectEq(extra_bindings, binding_bundle.extra_bindings);
    EXPECT_EQ(0, binding_bundle.new_vars.size());
    EXPECT_EQ(VarType::CONTINUOUS, binding_bundle.new_vars_type);
  }

  {
    // Supplying new variables that should be introduced into the program.
    BindingBundle<BoundingBoxConstraint, Constraint> binding_bundle = {
        binding,
        extra_bindings,
        x
    };
    ExpectEq(binding, binding_bundle.binding);
    ExpectEq(extra_bindings, binding_bundle.extra_bindings);
    EXPECT_EQ(x, binding_bundle.new_vars);
    EXPECT_EQ(VarType::CONTINUOUS, binding_bundle.new_vars_type);
  }

  {
    BindingBundle<BoundingBoxConstraint, Constraint> binding_bundle = {
        binding,
        extra_bindings,
        x,
        VarType::BINARY
    };
    ExpectEq(binding, binding_bundle.binding);
    ExpectEq(extra_bindings, binding_bundle.extra_bindings);
    EXPECT_EQ(x, binding_bundle.new_vars);
    EXPECT_EQ(VarType::BINARY, binding_bundle.new_vars_type);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
