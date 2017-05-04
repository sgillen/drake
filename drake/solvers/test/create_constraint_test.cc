#include "drake/solvers/create_constraint.h"

#include <cmath>
#include <limits>
#include <set>

#include <gtest/gtest.h>

#include "drake/common/test/is_dynamic_castable.h"

namespace drake {
namespace solvers {
namespace internal {
namespace test {

using std::set;
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
  if (rows != Eigen::Dynamic)
    out.resize(rows);
  DRAKE_DEMAND(out.rows() > 0);
  for (int i = 0; i < out.rows(); ++i) {
    out(i) = Variable(name + "(" + to_string(i) + ")");
  }
  return out;
}

}  // anonymous namespace

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
