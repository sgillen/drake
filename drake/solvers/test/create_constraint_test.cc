#include "drake/solvers/create_constraint.h"

#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace solvers {
namespace internal {
namespace test {

using std::numeric_limits;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::to_string;

using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector4d;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

using symbolic::test::ExprEqual;

namespace {

/*
 * Create vector of decision variables
 */
template <int Rows = Eigen::Dynamic>
VectorDecisionVariable<Rows> CreateVectorDecisionVariable(
    const string& name = "x", int rows = Eigen::Dynamic) {
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

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic3) {
  // Add linear constraints
  //     3 <=  3 - 5*x0 +      + 10*x2        - 7*y1        <= 9
  //   -10 <=                       x2                      <= 10
  //    -7 <= -5 + 2*x0 + 3*x2         + 3*y0 - 2*y1 + 6*y2 <= 12
  //     2 <=                     2*x2                      <= 3
  //     1 <=                                 -   y1        <= 3
  //
  // Note: the second, fourth and fifth rows are actually a bounding-box
  // constraints but we still process the five symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  //         [ 0 0  2 0  0 0]
  //         [ 0 0  0 0 -1 0]
  
  auto x = CreateVectorDecisionVariable<3>("x");
  auto y = CreateVectorDecisionVariable<3>("y");
  Matrix<Expression, 5, 1> M_e;
  Matrix<double, 5, 1> M_lb;
  Matrix<double, 5, 1> M_ub;

  // clang-format off
  M_e  <<  3 - 5 * x(0) + 10 * x(2) - 7 * y(1),
      +x(2),
      -5 + 2 * x(0) + 3 * x(2) + 3 * y(0) - 2 * y(1) + 6 * y(2),
      2 * x(2),
      -y(1);
  M_lb <<  3,
      -10,
      -7,
       2,
      1;
  M_ub << -7,
      10,
      12,
      3,
      3;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = ParseLinearConstraint(M_e, M_lb, M_ub);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  for (int i = 0; i < M_e.size(); ++i) {
    EXPECT_PRED2(ExprEqual, M_e(i) - M_lb(i), Ax(i) - lb_in_ctr(i));
    EXPECT_PRED2(ExprEqual, M_e(i) - M_ub(i), Ax(i) - ub_in_ctr(i));
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic4And5And6) {
  // Check the separate linear constraints:
  // 2  <= 2 * x <= 4
  // 2  <= -2 * x <= 4
  // 1 <= -x <= 3
  // Note: these are bounding box constraint
  const auto x = CreateVectorDecisionVariable<2>("x");

  {
    const Expression e(2 * x(1));
    const auto binding = ParseLinearConstraint(e, 2, 4);
    const auto& constraint = binding.constraint();

    EXPECT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(constraint));
    EXPECT_EQ(binding.variables(), x.segment(1, 1));
    EXPECT_TRUE(CompareMatrices(constraint->lower_bound(), Vector1d(1)));
    EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector1d(2)));
  }

  {
    const Expression e(-2 * x(1));
    const auto binding = ParseLinearConstraint(e, 2, 4);
    const auto& constraint = binding.constraint();

    EXPECT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(constraint));
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
    EXPECT_TRUE(CompareMatrices(constraint->lower_bound(), Vector1d(-2)));
    EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector1d(-1)));
  }

  {
    const Expression e(-x(0));
    const auto binding = ParseLinearConstraint(e, 1, 3);
    const auto& constraint = binding.constraint();

    EXPECT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(constraint));
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(CompareMatrices(constraint->lower_bound(), Vector1d(-3)));
    EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector1d(-1)));
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic7) {
  // Checks the linear constraints
  // 1 <= -2 * x0 <= 3
  // 3 <= 4 * x0 + 2<= 5
  // 2 <= x1 + 2 * (x2 - 0.5*x1) + 3 <= 4;
  // 3 <= -4 * x1 + 3 <= inf
  // Note: these are all bounding box constraints.
  auto x = CreateVectorDecisionVariable<3>();

  Vector4<Expression> e;
  // clang-format off
  e << -2 * x(0),
    4 * x(0) + 2,
    x(1) + 2 * (x(2) - 0.5 * x(1)) + 3,
    -4 * x(1) + 3;
  // clang-format on
  const auto binding = ParseLinearConstraint(
      e, Vector4d(1, 3, 2, 3),
      Vector4d(3, 5, 4, numeric_limits<double>::infinity()));
  const auto& constraint = binding.constraint();

  EXPECT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(constraint));
  EXPECT_EQ(binding.variables(),
            VectorDecisionVariable<4>(x(0), x(0), x(2), x(1)));
  EXPECT_TRUE(CompareMatrices(
      binding.constraint()->lower_bound(),
      Vector4d(-1.5, 0.25, -0.5, -numeric_limits<double>::infinity())));
  EXPECT_TRUE(CompareMatrices(constraint->upper_bound(),
                              Vector4d(-0.5, 0.75, 0.5, 0)));
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic8) {
  // Test the failure cases for adding linear constraint.
  const auto x = CreateVectorDecisionVariable<2>();

  // Non-polynomial.
  EXPECT_THROW(ParseLinearConstraint(sin(x(0)), 1, 2), runtime_error);

  // Non-linear.
  EXPECT_THROW(ParseLinearConstraint(x(0) * x(0), 1, 2), runtime_error);

  // Trivial (and infeasible) case 1 <= 0 <= 2
  EXPECT_THROW(ParseLinearConstraint(x(0) - x(0), 1, 2), runtime_error);
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolic9) {
  // Test trivial constraint with no variables, such as 1 <= 2 <= 3
  auto x = CreateVectorDecisionVariable<2>();

  {
    const auto binding = ParseLinearConstraint(Expression(2), 1, 3);
    const auto& constraint = binding.constraint();
    EXPECT_TRUE(is_dynamic_castable<LinearConstraint>(constraint));
    EXPECT_EQ(constraint->A().rows(), 1);
    EXPECT_EQ(constraint->A().cols(), 0);
  }

  {
    Vector2<Expression> expr;
    expr << 2, x(0);
    const auto binding = ParseLinearConstraint(expr, Vector2d(1, 2),
                                               Vector2d(3, 4));
    const auto& constraint = binding.constraint();
    EXPECT_TRUE(is_dynamic_castable<LinearConstraint>(constraint));
    EXPECT_TRUE(CompareMatrices(constraint->A(), Vector2d(0, 1)));
    EXPECT_TRUE(CompareMatrices(constraint->lower_bound(), Vector2d(-1, 2)));
    EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector2d(1, 4)));
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormulaAnd1) {
  // Add linear constraints
  //
  //   (A*x == b) = |1 2| * |x0| == |5|
  //                |3 4|   |x1|    |6|
  //
  //              = |  x0 + 2*x1 == 5|
  //                |3*x0 + 4*x1 == 6|
  //
  // where A = |1 2|, x = |x0|, b = |5|
  //           |3 4|      |x1|      |6|.
  //
  auto x = CreateVectorDecisionVariable<2>("x");
  Matrix<Expression, 2, 2> A;
  Vector2d b;
  // clang-format off
  A << 1, 2,
       3, 4;
  b << 5,
       6;
  // clang-format on
  const auto binding = ParseLinearConstraint(A * x == b);
  const auto& var_vec{binding.variables()};
  const auto& constraint_ptr = binding.constraint();
  // Checks that we have LinearEqualityConstraint instead of LinearConstraint.
  EXPECT_TRUE(is_dynamic_castable<LinearEqualityConstraint>(constraint_ptr));
  EXPECT_EQ(constraint_ptr->num_constraints(), b.size());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(x(0) + 2 * x(1) - 5);
  constraint_set.emplace(3 * x(0) + 4 * x(1) - 6);
  EXPECT_EQ(constraint_set.count(Ax(0) - lb_in_ctr(0)), 1);
  EXPECT_EQ(constraint_set.count(Ax(0) - ub_in_ctr(0)), 1);
  EXPECT_EQ(constraint_set.count(Ax(1) - lb_in_ctr(1)), 1);
  EXPECT_EQ(constraint_set.count(Ax(1) - ub_in_ctr(1)), 1);
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

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula1) {
  // Add linear constraints
  //    3 - 5*x0 +      + 10*x2        - 7*y1         >= 3
  //                         x2                       >= -10
  //   -5 + 2*x0 + 3*x1         + 3*y0 - 2*y1 + 6*y2  <= -7
  //                       2*x2                       == 2
  //                                   -   y1         >= 1
  //
  // Note: the second, fourth and fifth rows are actually a bounding-box
  // constraints but we still process the five symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  //         [ 0 0  2 0  0 0]
  //         [ 0 0  0 0 -1 0]
  auto x = CreateVectorDecisionVariable<3>("x");
  auto y = CreateVectorDecisionVariable<3>("y");
  Matrix<Formula, 5, 1> M_f;
  // clang-format off
  M_f << (3 - 5 * x(0) + 10 * x(2) - 7 * y(1) >= 3),
         x(2) >= -10,
         -5 + 2 * x(0) + 3 * x(1) + 3 * y(0) - 2 * y(1) + 6 * y(2) <= -7,
         2 * x(2) == 2,
         -y(1) >= 1;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = ParseLinearConstraint(M_f.array());
  const auto& var_vec = binding.variables();
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  for (int i{0}; i < M_f.size(); ++i) {
    if (!std::isinf(lb_in_ctr(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - lb_in_ctr(i));
    }
    if (!std::isinf(ub_in_ctr(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - ub_in_ctr(i));
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
// This test uses operator>= provided for Eigen::Array<Expression> and
// Eigen::Array<double>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula2) {
  // Add linear constraints
  //
  //     M_f = |f1 f2|
  //           |f3 f4|
  //
  // where
  //   f1 =  3 - 5*x0        +10*x2        - 7*y1        >= 3
  //   f2 =                      x2                      >= -10
  //   f3 = -5 + 2*x0 + 3*x1        + 3*y0 - 2*y1 + 6*y2 >= -7
  //   f4 =                    2*x2                      >= 2
  
  auto x = CreateVectorDecisionVariable<3>("x");
  auto y = CreateVectorDecisionVariable<3>("y");
  Matrix<Expression, 2, 2> M_e;
  Matrix<double, 2, 2> M_lb;
  // clang-format off
  M_e << 3 - 5 * x(0) + 10 * x(2) - 7 * y(1),                       x(2),
        -5 + 2 * x(0) +  3 * x(1) + 3 * y(0) - 2 * y(1) + 6 * y(2), 2 * x(2);
  M_lb << 3, -10,
         -7,   2;
  // clang-format on
  Eigen::Array<Formula, 2, 2> M_f{M_e.array() >= M_lb.array()};

  // Check if the binding includes the correct linear constraint.
  const auto binding = ParseLinearConstraint(M_f);
  const auto& var_vec = binding.variables();
  const auto& constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), M_e.rows() * M_e.cols());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  int k{0};
  for (int j{0}; j < M_e.cols(); ++j) {
    for (int i{0}; i < M_e.rows(); ++i) {
      EXPECT_PRED2(ExprEqual, M_e(i, j) - M_lb(i, j), Ax(k) - lb_in_ctr(k));
      ++k;
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula3) {
  // Add linear constraints
  //
  //   (A*x >= b) = |1 2| * |x0| >= |5|
  //                |3 4|   |x1|    |6|
  //
  //              = |  x0 + 2*x1 >= 5|
  //                |3*x0 + 4*x1 >= 6|
  //
  // where A = |1 2|, x = |x0|, b = |5|
  //           |3 4|      |x1|      |6|.
  //
  
  auto x = CreateVectorDecisionVariable<2>("x");
  Matrix<Expression, 2, 2> A;
  Eigen::Vector2d b;
  // clang-format off
  A << 1, 2,
       3, 4;
  b << 5,
       6;
  // clang-format on
  const auto binding = ParseLinearConstraint((A * x).array() >= b.array());
  const auto& var_vec = binding.variables();
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), b.size());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  EXPECT_PRED2(ExprEqual, x(0) + 2 * x(1) - 5, Ax(0) - lb_in_ctr(0));
  EXPECT_PRED2(ExprEqual, 3 * x(0) + 4 * x(1) - 6, Ax(1) - lb_in_ctr(1));
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram,
           AddLinearConstraintSymbolicArrayFormulaException) {
  // Add linear constraints
  //    3 - 5*x0 + 10*x2 - 7*y1 >  3
  //                  x2        > -10
  // Note that this includes strict inequality (>) and results in an exception.
  
  auto x = CreateVectorDecisionVariable<3>("x");
  auto y = CreateVectorDecisionVariable<3>("y");
  Matrix<Formula, 2, 1> M_f;
  // clang-format off
  M_f << (3 - 5 * x(0) + 10 * x(2) - 7 * y(1) > 3),
                              x(2)            > -10;
  // clang-format on
  EXPECT_THROW(ParseLinearConstraint(M_f.array()), runtime_error);
}

}  // namespace test
}  // namespace internal
}  // namespace solvers
}  // namespace drake
