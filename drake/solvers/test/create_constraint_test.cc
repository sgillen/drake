#include "drake/solvers/create_constraint.h"

#include <cmath>
#include <limits>
#include <memory>
#include <set>

#include <gtest/gtest.h>

#include "drake/common/test/is_dynamic_castable.h"

namespace drake {
namespace solvers {
namespace internal {
namespace test {

using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::to_string;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {

/*
 * Create vector of decision variables
 */
template <int Rows = Eigen::Dynamic>
VectorDecisionVariable<Rows> CreateVectorDecisionVariable(
    const string& name, int rows = Eigen::Dynamic) {
  VectorDecisionVariable<Rows> out;
  if (Rows == Eigen::Dynamic) {
    DRAKE_DEMAND(rows > 0);
    out.resize(rows);
  }
  for (int i = 0; i < out.rows(); ++i) {
    out(i) = Variable(name + "(" + to_string(i) + ")");
  }
  return out;
}

}  // anonymous namespace

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic1) {
  // Add linear constraint: -10 <= 3 - 5*x0 + 10*x2 - 7*y1 <= 10
  auto x = CreateVectorDecisionVariable<3>("x");
  auto y = CreateVectorDecisionVariable<3>("y");
  const Expression e{3 - 5 * x(0) + 10 * x(2) - 7 * y(1)};
  const double lb{-10};
  const double ub{+10};
  const auto binding = ParseLinearConstraint(e, lb, ub);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);
  const Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - lb).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - ub).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic2) {
  // Add linear constraint: -10 <= x0 <= 10
  // Note that this constraint is a bounding-box constraint which is a sub-class
  // of linear-constraint.
  auto x = CreateVectorDecisionVariable<3>("x");
  const Expression e{x(0)};
  const auto binding = ParseLinearConstraint(e, -10, 10);

  // Check that the constraint in the binding is of BoundingBoxConstraint.
  ASSERT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(binding.constraint()));
  const shared_ptr<BoundingBoxConstraint> constraint_ptr{
      static_pointer_cast<BoundingBoxConstraint>(binding.constraint())};
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - -10).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - 10).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormulaAnd2) {
  // Add linear constraints f1 && f2 && f3 where
  //   f1 := (x0 + 2*x1 >= 3)
  //   f2 := (3*x0 + 4*x1 <= 5)
  //   f3 := (7*x0 + 2*x1 == 9).
  auto x = CreateVectorDecisionVariable<2>("x");
  const Expression e11{x(0) + 2 * x(1)};
  const Expression e12{3};
  const Formula f1{e11 >= e12};
  const Expression e21{3 * x(0) + 4 * x(1)};
  const Expression e22{5};
  const Formula f2{e21 <= e22};
  const Expression e31{7 * x(0) + 2 * x(1)};
  const Expression e32{9};
  const Formula f3{e31 == e32};

  const auto binding = ParseLinearConstraint(f1 && f2 && f3);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  // Checks that we do not have LinearEqualityConstraint.
  EXPECT_FALSE(is_dynamic_castable<LinearEqualityConstraint>(constraint_ptr));
  EXPECT_EQ(constraint_ptr->num_constraints(), 3);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(e11 - e12);
  constraint_set.emplace(e21 - e22);
  constraint_set.emplace(e31 - e32);
  for (int i = 0; i < 3; ++i) {
    if (!std::isinf(lb_in_ctr(i))) {
    EXPECT_EQ(constraint_set.count(Ax(i) - lb_in_ctr(i)), 1);
    }
    if (!std::isinf(ub_in_ctr(i))) {
    EXPECT_EQ(constraint_set.count(Ax(i) - ub_in_ctr(i)), 1);
    }
  }
}

}  // namespace test
}  // namespace internal
}  // namespace solvers
}  // namespace drake
