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
GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicArrayFormula1) {
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
GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicArrayFormula2) {
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
GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicArrayFormula3) {
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
GTEST_TEST(testCreateConstraint,
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


GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormula1) {
  
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;

  // x(0) <= 3
  vector<Formula> f;
  f.push_back(x(0) <= 3);
  f.push_back(3 >= x(0));
  f.push_back(x(0) + 2 <= 5);
  f.push_back(4 + x(0) >= 1 + 2 * x(0));
  f.push_back(2 * x(0) + 1 <= 4 + x(0));
  f.push_back(3 * x(0) + x(1) <= 6 + x(0) + x(1));
  for (const auto& fi : f) {
    auto binding = ParseLinearConstraint(fi);
    EXPECT_EQ(++num_bounding_box_constraint,
              prog.bounding_box_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    auto binding = prog.bounding_box_constraints().back();
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->upper_bound(), Vector1d(3)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->lower_bound(),
                                Vector1d(-numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormula2) {
  
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;
  // x0 >= 2
  vector<Formula> f;
  f.push_back(x(0) >= 2);
  f.push_back(2 <= x(0));
  f.push_back(-x(0) <= -2);
  f.push_back(-2 >= -x(0));
  f.push_back(2 + 2 * x(0) >= x(0) + 4);
  f.push_back(3 + 3 * x(0) >= x(0) + 7);
  f.push_back(x(0) + 7 + 2 * x(1) <= 3 * x(0) + 3 + 2 * x(1));
  for (const auto& fi : f) {
    ParseLinearConstraint(fi);
    EXPECT_EQ(++num_bounding_box_constraint,
              prog.bounding_box_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    auto binding = prog.bounding_box_constraints().back();
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->lower_bound(), Vector1d(2)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                                Vector1d(numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormula3) {
  // x(0) + x(2) == 1
  
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_equality_constraint = 0;

  vector<Formula> f;
  f.push_back(x(0) + x(2) == 1);
  f.push_back(x(0) + 2 * x(2) == 1 + x(2));
  for (const auto& fi : f) {
    ParseLinearConstraint(fi);
    EXPECT_EQ(++num_linear_equality_constraint,
              prog.linear_equality_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 0);
    auto binding = prog.linear_equality_constraints().back();
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->lower_bound(), Vector1d(1)));

    VectorX<Expression> expr = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), x(0) + x(2) - 1);
  }
}

GTEST_TEST(testCreateConstraint, ParseLinearConstraintSymbolicFormula4) {
  // x(0) + 2 * x(2) <= 1
  
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_constraint = 0;

  vector<Formula> f;
  f.push_back(x(0) + 2 * x(2) <= 1);
  f.push_back(x(0) + 3 * x(2) <= 1 + x(2));
  f.push_back(-1 <= -x(0) - 2 * x(2));
  f.push_back(-1 - x(2) <= -x(0) - 3 * x(2));
  f.push_back(2 * (x(0) + x(2)) - x(0) <= 1);
  f.push_back(1 >= x(0) + 2 * x(2));
  f.push_back(1 + x(2) >= x(0) + 3 * x(2));
  f.push_back(-x(0) - 2 * x(2) >= -1);
  f.push_back(-x(0) - 3 * x(2) >= -1 - x(2));
  f.push_back(1 >= 2 * (x(0) + x(2)) - x(0));
  for (const auto& fi : f) {
    ParseLinearConstraint(fi);
    EXPECT_EQ(++num_linear_constraint, prog.linear_constraints().size());
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 0);
    auto binding = prog.linear_constraints().back();
    EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                                Vector1d(numeric_limits<double>::infinity())));
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->lower_bound(), Vector1d(-1)));

    VectorX<Expression> expr = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), 1 - x(0) - 2 * x(2));
  }
}

GTEST_TEST(testCreateConstraint,
           AddLinearConstraintSymbolicFormulaAndException) {
  // Add linear constraints:
  //   (x0 + 2*x1 > 3)
  //   (3*x0 + 4*x1 < 5)
  //   (7*x0 + 2*x1 == 9)
  //
  // It includes relational formulas with strict inequalities (> and <). It will
  // throw std::runtime_error.
  
  auto x = prog.NewContinuousVariables(2, "x");
  const Expression e11{x(0) + 2 * x(1)};
  const Expression e12{3};
  const Formula f1{e11 > e12};
  const Expression e21{3 * x(0) + 4 * x(1)};
  const Expression e22{5};
  const Formula f2{e21 < e22};
  const Expression e31{7 * x(0) + 2 * x(1)};
  const Expression e32{9};
  const Formula f3{e31 == e32};
  EXPECT_THROW(ParseLinearConstraint(f1 && f2 && f3), runtime_error);
}


// Tests `AddLinearEqualityConstraint(const symbolic::Formula& f)` method with a
// case where `f` is a linear-equality formula (instead of a conjunction of
// them).
GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint5) {
  
  auto x = CreateVectorDecisionVariable<3>("x");
  // f = (3x₀ + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) + 2 * x(1) + 3 == 5};
  const Binding<LinearEqualityConstraint> binding{
      ParseLinearEqualityConstraint(f)};
  EXPECT_EQ(prog.linear_equality_constraints().size(), 1u);

  const Expression expr_in_added_constraint{
      (binding.constraint()->A() * binding.variables() -
       binding.constraint()->lower_bound())(0)};
  // expr_in_added_constraint should be:
  //    lhs(f) - rhs(f)
  //  = (3x₀ + 2x₁ + 3) - 5
  //  = 3x₀ + 2x₁ - 2.
  EXPECT_PRED2(ExprEqual, expr_in_added_constraint, 3 * x(0) + 2 * x(1) - 2);
  EXPECT_PRED2(ExprEqual, expr_in_added_constraint,
               get_lhs_expression(f) - get_rhs_expression(f));
}

// Tests `AddLinearEqualityConstraint(const symbolic::Formula& f)` method with a
// case where `f` is a conjunction of linear-equality formulas .
GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint6) {
  // Test problem: Ax = b where
  //
  // A = |-3.0  0.0  2.0|  x = |x0|  b = | 9.0|
  //     | 0.0  7.0 -3.0|      |x1|      | 3.0|
  //     | 2.0  5.0  0.0|      |x2|      |-5.0|
  
  auto x = CreateVectorDecisionVariable<3>("x");
  Eigen::Matrix3d A;
  Vector3d b;
  // clang-format off
  A << -3.0, 0.0,  2.0,
        0.0, 7.0, -3.0,
        2.0, 5.0,  0.0;
  b << 9.0,
       3.0,
      -5.0;
  // clang-format on
  const Formula f{A * x == b};
  const Binding<LinearEqualityConstraint> binding{
      ParseLinearEqualityConstraint(f)};
  EXPECT_EQ(prog.linear_equality_constraints().size(), 1u);

  // Checks if AddLinearEqualityConstraint added the constraint correctly.
  const Eigen::Matrix<Expression, 3, 1> exprs_in_added_constraint{
      binding.constraint()->A() * binding.variables() -
      binding.constraint()->lower_bound()};
  const Eigen::Matrix<Expression, 3, 1> expected_exprs{A * x - b};

  // Since a conjunctive symbolic formula uses `std::set` as an internal
  // representation, we need to check if `exprs_in_added_constraint` is a
  // permutation of `expected_exprs`.
  EXPECT_TRUE(is_permutation(
      exprs_in_added_constraint.data(), exprs_in_added_constraint.data() + 3,
      expected_exprs.data(), expected_exprs.data() + 3, ExprEqual));
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is a non-linear equality formula.
GTEST_TEST(testCreateConstraint,
           AddSymbolicLinearEqualityConstraintException1) {
  
  auto x = CreateVectorDecisionVariable<2>("x");
  // f = (3x₀² + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) * x(0) + 2 * x(1) + 3 == 5};
  EXPECT_THROW(ParseLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if a
// conjunctive formula `f` includes a relational formula other than equality.
GTEST_TEST(testCreateConstraint,
           AddSymbolicLinearEqualityConstraintException2) {
  
  auto x = CreateVectorDecisionVariable<2>("x");
  // f = (3x₀ + 2x₁ + 3 >= 5 && 7x₀ - 5x₁ == 0).
  const Formula f{3 * x(0) + 2 * x(1) + 3 >= 5 && 7 * x(0) - 5 * x(1) == 0};
  EXPECT_THROW(ParseLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is neither a linear-equality formula nor a conjunctive formula.
GTEST_TEST(testCreateConstraint,
           AddSymbolicLinearEqualityConstraintException3) {
  
  auto x = CreateVectorDecisionVariable<2>("x");
  // f = (3x₀ + 2x₁ + 3 == 5 || 7x₀ - 5x₁ == 0).
  const Formula f{3 * x(0) + 2 * x(1) + 3 == 5 || 7 * x(0) - 5 * x(1) == 0};
  EXPECT_THROW(ParseLinearEqualityConstraint(f), runtime_error);
}


namespace {
template <typename Derived>
typename enable_if<is_same<typename Derived::Scalar, Expression>::value>::type
CheckAddedSymbolicPositiveSemidefiniteConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<Derived>& V) {
  int num_psd_cnstr = prog->positive_semidefinite_constraints().size();
  int num_lin_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = ParsePositiveSemidefiniteConstraint(V);
  // Check if number of linear equality constraints and positive semidefinite
  // constraints are both incremented by 1.
  EXPECT_EQ(num_psd_cnstr + 1,
            prog->positive_semidefinite_constraints().size());
  EXPECT_EQ(num_lin_eq_cnstr + 1, prog->linear_equality_constraints().size());
  // Check if the returned binding is the correct one.
  EXPECT_EQ(
      binding.constraint().get(),
      prog->positive_semidefinite_constraints().back().constraint().get());
  // Check if the added linear constraint is correct. M is the newly added
  // variables representing the psd matrix.
  const Eigen::Map<const MatrixX<Variable>> M(&binding.variables()(0), V.rows(),
                                              V.cols());
  // The linear equality constraint is only imposed on the lower triangular
  // part of the psd matrix.
  const auto& new_lin_eq_cnstr = prog->linear_equality_constraints().back();
  auto V_minus_M = math::ToSymmetricMatrixFromLowerTriangularColumns(
      new_lin_eq_cnstr.constraint()->A() * new_lin_eq_cnstr.variables() -
      new_lin_eq_cnstr.constraint()->lower_bound());
  EXPECT_EQ(V_minus_M, V - M);
}
}  // namespace

GTEST_TEST(testCreateConstraint, ParsePositiveSemidefiniteConstraint) {
  
  auto X = prog.NewSymmetricContinuousVariables<4>("X");

  auto psd_cnstr = ParsePositiveSemidefiniteConstraint(X).constraint();
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1);
  const auto& new_psd_cnstr = prog.positive_semidefinite_constraints().back();
  EXPECT_EQ(psd_cnstr.get(), new_psd_cnstr.constraint().get());
  Eigen::Map<Eigen::Matrix<Variable, 16, 1>> X_flat(&X(0, 0));
  EXPECT_TRUE(CheckStructuralEquality(X_flat, new_psd_cnstr.variables()));

  // Adds X is psd.
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog,
                                                   Matrix4d::Identity() * X);

  // Adds 2 * X + Identity() is psd.
  CheckAddedSymbolicPositiveSemidefiniteConstraint(
      2.0 * X + Matrix4d::Identity());

  // Adds a linear matrix expression Aᵀ * X + X * A is psd.
  Matrix4d A{};
  // clang-format off
  A << 1, 2, 3, 4,
       0, 1, 2, 3,
       0, 0, 2, 3,
       0, 0, 0, 1;
  // clang-format on
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog,
                                                   A.transpose() * X + X * A);

  // Adds [X.topLeftCorner<2, 2>()  0                        ] is psd
  //      [ 0                     X.bottomRightCorner<2, 2>()]
  Eigen::Matrix<Expression, 4, 4> Y{};
  // clang-format off
  Y << Matrix2d::Identity() * X.topLeftCorner<2, 2>(), Matrix2d::Zero(),
       Matrix2d::Zero(), Matrix2d::Identity() * X.bottomRightCorner<2, 2>();
  // clang-format on
  CheckAddedSymbolicPositiveSemidefiniteConstraint(Y);
}




namespace {
void CheckAddedLinearEqualityConstraintCommon(
    const Binding<LinearEqualityConstraint>& binding,
    const MathematicalProgram& prog, int num_linear_eq_cnstr) {
  // Checks if the number of linear equality constraints get incremented by 1.
  EXPECT_EQ(prog.linear_equality_constraints().size(), num_linear_eq_cnstr + 1);
  // Checks if the newly added linear equality constraint in prog is the same as
  // that returned from AddLinearEqualityConstraint.
  EXPECT_EQ(prog.linear_equality_constraints().back().constraint(),
            binding.constraint());
  // Checks if the bound variables of the newly added linear equality constraint
  // in prog is the same as that returned from AddLinearEqualityConstraint.
  EXPECT_EQ(prog.linear_equality_constraints().back().variables(),
            binding.variables());
}

template <typename DerivedV, typename DerivedB>
void CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<DerivedV>& v,
    const Eigen::MatrixBase<DerivedB>& b) {
  const int num_linear_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = ParseLinearEqualityConstraint(v, b);
  CheckAddedLinearEqualityConstraintCommon(binding, *prog, num_linear_eq_cnstr);
  // Checks if the number of rows in the newly added constraint is the same as
  // the input expression.
  int num_constraints_expected = v.size();

  EXPECT_EQ(binding.constraint()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();

  EXPECT_EQ(flat_V.size(), v.size());
  MatrixX<Expression> v_resize = flat_V;
  v_resize.resize(v.rows(), v.cols());
  for (int i = 0; i < v.rows(); ++i) {
    for (int j = 0; j < v.cols(); ++j) {
      EXPECT_EQ(v_resize(i, j).Expand(), (v(i, j) - b(i, j)).Expand());
    }
  }
}

template <typename DerivedV, typename DerivedB>
void CheckAddedSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<DerivedV>& v,
    const Eigen::MatrixBase<DerivedB>& b) {
  const int num_linear_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = ParseLinearEqualityConstraint(v, b, true);
  CheckAddedLinearEqualityConstraintCommon(binding, *prog, num_linear_eq_cnstr);
  // Checks if the number of rows in the newly added constraint is the same as
  // the input expression.
  int num_constraints_expected = v.rows() * (v.rows() + 1) / 2;
  EXPECT_EQ(binding.constraint()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();
  EXPECT_EQ(math::ToSymmetricMatrixFromLowerTriangularColumns(flat_V), v - b);
}

void CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Expression& e, double b) {
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      prog, Vector1<Expression>(e), Vector1d(b));
}
}  // namespace

GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint1) {
  // Checks the single row linear equality constraint one by one:

  
  auto x = CreateVectorDecisionVariable<3>("x");

  // Checks x(0) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(+x(0), 1);
  // Checks x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(+x(1), 1);
  // Checks x(0) + x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(0) + x(1), 1);
  // Checks x(0) + 2 * x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(0) + 2 * x(1),
                                                         1);
  // Checks 3 * x(0) - 2 * x(1) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      3 * x(0) - 2 * x(1), 2);
  // Checks 2 * x(0) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(2 * x(0), 2);
  // Checks x(0) + 3 * x(2) = 3
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(0) + 3 * x(2),
                                                         3);
  // Checks 2 * x(1) - 3 * x(2) = 4
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      2 * x(1) - 3 * x(2), 4);
  // Checks x(0) + 2 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(0) + 2, 1);
  // Checks x(1) - 2 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(1) - 2, 1);
  // Checks 3 * x(1) + 4 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(3 * x(1) + 4,
                                                         1);
  // Checks x(0) + x(2) + 3 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(x(0) + x(2) + 3,
                                                         1);
  // Checks 2 * x(0) + x(2) - 3 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      2 * x(0) + x(2) - 3, 1);
  // Checks 3 * x(0) + x(1) + 4 * x(2) + 1 = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      3 * x(0) + x(1) + 4 * x(2) + 1, 2);
  // Checks -x(1) = 3
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(-x(1), 3);
  // Checks -(x(0) + 2 * x(1)) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog,
                                                         -(x(0) + 2 * x(1)), 2);

  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      x(0) + 2 * (x(0) + x(2)) + 3 * (x(0) - x(1)), 3);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint2) {
  // Checks adding multiple rows of linear equality constraints.

  
  auto x = CreateVectorDecisionVariable<3>("x");

  // Checks x(1) = 2
  //        x(0) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      Vector2<Expression>(+x(1), +x(0)), Eigen::Vector2d(2, 1));

  // Checks 2 * x(1) = 3
  //        x(0) + x(2) = 4
  //        x(0) + 3 * x(1) + 7 = 1
  Vector3<Expression> v{};
  v << 2 * x(1), x(0) + x(2), x(0) + 3 * x(1) + 7;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      v, Eigen::Vector3d(3, 4, 1));

  // Checks x(0) = 4
  //          1  = 1
  //       -x(1) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      Vector3<Expression>(+x(0), 1, -x(1)), Eigen::Vector3d(4, 1, 2));
}

GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint3) {
  // Checks adding a matrix of linear equality constraints. This matrix is not
  // symmetric.
  
  auto X = prog.NewContinuousVariables<2, 2>("X");

  // Checks A * X = [1, 2; 3, 4], both A * X and B are static sized.
  Eigen::Matrix2d A{};
  A << 1, 3, 4, 2;
  Eigen::Matrix2d B{};
  B << 1, 2, 3, 4;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(A * X, B);

  // Checks A * X = B, with A*X being dynamic sized, and B being static sized.
  MatrixX<Expression> A_times_X = A * X;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(A_times_X, B);

  // Checks A * X = B, with A*X being static sized, and B being dynamic sized.
  Eigen::MatrixXd B_dynamic = B;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(A * X,
                                                         B_dynamic);

  // Checks A * X = B, with both A*X and B being dynamic sized.
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(A_times_X,
                                                         B_dynamic);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicLinearEqualityConstraint4) {
  // Checks adding a symmetric matrix of linear equality constraints.
  
  auto X = prog.NewSymmetricContinuousVariables<2>("X");
  Eigen::Matrix2d A{};
  A << 1, 3, 4, 2;
  Eigen::Matrix2d B{};
  B << 1, 2, 2, 1;
  Eigen::MatrixXd B_dynamic = B;
  Matrix2<Expression> M = A.transpose() * X + X * A;
  MatrixX<Expression> M_dynamic = M;

  // Checks Aᵀ * X + X * A = B, both the left and right hand-side are static
  // sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(M, B);

  // Checks Aᵀ * X + X * A = B, the left hand-side being static sized, while the
  // right hand-side being dynamic sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(M, B_dynamic);

  // Checks Aᵀ * X + X * A = B, the left hand-side being dynamic sized, while
  // the
  // right hand-side being static sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(M_dynamic, B);

  // Checks Aᵀ * X + X * A = B, bot the left and right hand-side are dynamic
  // sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(M_dynamic,
                                                      B_dynamic);

  // Checks Aᵀ * X + X * A = E.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(
      M, Eigen::Matrix2d::Identity());
}

namespace {
bool AreTwoPolynomialsNear(const symbolic::MonomialToCoefficientMap& poly1,
                           const symbolic::MonomialToCoefficientMap& poly2,
                           double tol = numeric_limits<double>::epsilon()) {
  // TODO(hongkai.dai): rewrite this part when we have function to add and
  // subtract two polynomials.
  symbolic::MonomialToCoefficientMap poly_diff;
  poly_diff.reserve(poly1.size() + poly2.size());
  poly_diff.insert(poly1.begin(), poly1.end());
  for (const auto& p2 : poly2) {
    const auto it = poly_diff.find(p2.first);
    if (it == poly_diff.end()) {
      poly_diff.emplace_hint(it, p2.first, -p2.second);
    } else {
      it->second -= p2.second;
    }
  }
  return all_of(poly_diff.begin(), poly_diff.end(), [&tol](const auto& p) {
    return std::abs(symbolic::get_constant_value(p.second)) <= tol;
  });
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog, const Expression& linear_expr,
    const Expression& quadratic_expr) {
  const auto& binding1 =
      ParseLorentzConeConstraint(linear_expr, quadratic_expr);
  const auto& binding2 = prog->lorentz_cone_constraints().back();
  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.variables(), binding2.variables());
  // Now check if the linear and quadratic constraints are parsed correctly.
  const VectorX<Expression> e_parsed =
      binding1.constraint()->A() * binding1.variables() +
      binding1.constraint()->b();
  EXPECT_PRED2(ExprEqual, (e_parsed(0) * e_parsed(0)).Expand(),
               (linear_expr * linear_expr).Expand());
  Expression quadratic_expr_parsed =
      e_parsed.tail(e_parsed.rows() - 1).squaredNorm();
  // Due to the small numerical error, quadratic_expr and quadratic_expr_parsed
  // do not match exactly.So we will compare each term in the two polynomials,
  // and regard them to be equal if the error in the coefficient is sufficiently
  // small.
  const auto& monomial_to_coeff_map_parsed =
      symbolic::DecomposePolynomialIntoMonomial(
          quadratic_expr_parsed, quadratic_expr_parsed.GetVariables());
  const auto& monomial_to_coeff_map = symbolic::DecomposePolynomialIntoMonomial(
      quadratic_expr, quadratic_expr.GetVariables());
  double tol = 100 * numeric_limits<double>::epsilon();
  EXPECT_TRUE(AreTwoPolynomialsNear(monomial_to_coeff_map_parsed,
                                    monomial_to_coeff_map, tol));
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::Matrix<Expression, Eigen::Dynamic, 1>>& e) {
  const auto& binding1 = ParseLorentzConeConstraint(e);
  const auto& binding2 = prog->lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);

  CheckParsedSymbolicLorentzConeConstraint(prog, e(0),
                                           e.tail(e.rows() - 1).squaredNorm());
}

void CheckParsedSymbolicRotatedLorentzConeConstraint(
    MathematicalProgram* prog, const Eigen::Ref<const VectorX<Expression>>& e) {
  const auto& binding1 = ParseRotatedLorentzConeConstraint(e);
  const auto& binding2 = prog->rotated_lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);
}
}  // namespace

class SymbolicLorentzConeTest : public ::testing::Test {
 public:
  SymbolicLorentzConeTest() : prog_(), x_() {
    x_ = CreateVectorDecisionVariable<3>("x");
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<3> x_;
};

TEST_F(SymbolicLorentzConeTest, Test1) {
  // Add Lorentz cone constraint:
  // x is in Lorentz cone
  Matrix<Expression, 3, 1> e;
  e << 1 * x_(0), 1.0 * x_(1), 1.0 * x_(2);
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test2) {
  // Add Lorentz cone constraint:
  // x + [1, 2, 0] is in Lorentz cone.
  Matrix<Expression, 3, 1> e;
  e << x_(0) + 1, x_(1) + 2, +x_(2);
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test3) {
  // Add Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2)]
  // [  - x(0) + 2 * x(2)]    is in Lorentz cone
  // [               x(2)]
  // [  -x(1)            ]
  Matrix<Expression, 4, 1> e;
  // clang-format on
  e << 2 * x_(0) + 3 * x_(2), -x_(0) + 2 * x_(2), +x_(2), -x_(1);
  // clang-format off;
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test4) {
  // Add Lorentz cone constraint:
  // [ 2 * x(0) + 3 * x(1) +            5]
  // [ 4 * x(0)            + 4 * x(2) - 7]
  // [                                 10]
  // [                       2 * x(2)    ]
  Matrix<Expression, 4, 1> e;
  // clang-format off
  e << 2 * x_(0) + 3 * x_(1) + 5,
       4 * x_(0) + 4 * x_(2) - 7,
       10,
       2 * x_(2);
  // clang-format on
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test5) {
  // Add Lorentz cone constraint:
  // [x(0); x(1); x(2); 0] is in the Lorentz cone.
  Vector4<Expression> e;
  e << x_(0), x_(1), x_(2), 0;
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test6) {
  // Add Lorentz cone constraint:
  // [x(0); x(1) + x(2)] is in the Lorentz cone.
  Vector2<Expression> e;
  e << x_(0), x_(1) + x_(2);
  CheckParsedSymbolicLorentzConeConstraint(e);
}

TEST_F(SymbolicLorentzConeTest, Test7) {
  CheckParsedSymbolicLorentzConeConstraint(
      x_(0) + 2, pow(x_(0), 2) + 4 * x_(0) * x_(1) + 4 * pow(x_(1), 2));
}

TEST_F(SymbolicLorentzConeTest, Test8) {
  CheckParsedSymbolicLorentzConeConstraint(
      x_(0) + 2,
      pow(x_(0), 2) - (x_(0) - x_(1)) * (x_(0) + x_(1)) + 2 * x_(1) + 3);
}

TEST_F(SymbolicLorentzConeTest, Test9) {
  CheckParsedSymbolicLorentzConeConstraint(2,
                                           pow(x_(0), 2) + pow(x_(1), 2));
}

TEST_F(SymbolicLorentzConeTest, TestLinearConstraint) {
  // Actually adding linear constraint, that the quadratic expression is
  // actually a constant.
  CheckParsedSymbolicLorentzConeConstraint(x_(0) + 2, 1);
  CheckParsedSymbolicLorentzConeConstraint(x_(0) + 2,
                                           x_(0) - 2 * (0.5 * x_(0) + 1) + 3);
}

TEST_F(SymbolicLorentzConeTest, TestError) {
  // Check the cases to add with invalid quadratic expression.

  // Check polynomial with order no smaller than 2
  EXPECT_THROW(ParseLorentzConeConstraint(2 * x_(0) + 3, pow(x_(1), 3)),
               runtime_error);

  // The quadratic expression is actually affine.
  EXPECT_THROW(ParseLorentzConeConstraint(2 * x_(0), 3 * x_(1) + 2),
               runtime_error);
  EXPECT_THROW(
      ParseLorentzConeConstraint(
          2 * x_(0), x_(1) * x_(1) - (x_(1) - x_(0)) * (x_(1) + x_(0)) -
                         x_(0) * x_(0) + 2 * x_(1) + 3),
      runtime_error);

  // The Hessian matrix is not positive semidefinite.
  EXPECT_THROW(ParseLorentzConeConstraint(2 * x_(0) + 3,
                                              x_(1) * x_(1) - x_(2) * x_(2)),
               runtime_error);
  EXPECT_THROW(
      ParseLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(1) * x_(2)),
      runtime_error);
  EXPECT_THROW(
      ParseLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(0) * x_(2)),
      runtime_error);

  // The quadratic expression is not always non-negative.
  EXPECT_THROW(ParseLorentzConeConstraint(
                   2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) - 1),
               runtime_error);
  EXPECT_THROW(ParseLorentzConeConstraint(
                   2 * x_(0) + 3, pow(2 * x_(0) + 3 * x_(1) + 2, 2) - 1),
               runtime_error);

  // The quadratic expression is a negative constant.
  EXPECT_THROW(ParseLorentzConeConstraint(
                   2 * x_(0) + 3, pow(x_(0), 2) - pow(x_(1), 2) -
                                      (x_(0) + x_(1)) * (x_(0) - x_(1)) - 1),
               runtime_error);

  // The first expression is not actually linear.
  EXPECT_THROW(ParseLorentzConeConstraint(2 * x_(0) * x_(1), pow(x_(0), 2)),
               runtime_error);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicRotatedLorentzConeConstraint1) {
  // Add rotated Lorentz cone constraint:
  // x is in the rotated Lorentz cone constraint.
  
  auto x = CreateVectorDecisionVariable<3>("x");
  Matrix<Expression, 3, 1> e;
  e << +x(0), +x(1), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(e);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicRotatedLorentzConeConstraint2) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 2 * x(2)]
  // [x(0)           ] is in the rotated Lorentz cone
  // [           x(2)]
  
  auto x = CreateVectorDecisionVariable<3>("x");
  Matrix<Expression, 3, 1> e;
  e << x(0) + 2 * x(2), +x(0), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(e);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicRotatedLorentzConeConstraint3) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 1]
  // [x(1) + 2] is in the rotated Lorentz cone
  // [x(2)    ]
  // [x(3) - 1]
  // [-x(1)   ]
  
  auto x = CreateVectorDecisionVariable<4>("x");
  Matrix<Expression, 5, 1> e;
  e << x(0) + 1, x(1) + 2, +x(2), x(3) - 1, -x(1);
  CheckParsedSymbolicRotatedLorentzConeConstraint(e);
}

GTEST_TEST(testCreateConstraint, ParseSymbolicRotatedLorentzConeConstraint4) {
  // Add rotated Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2) + 3]
  // [    x(0) - 4 * x(2)    ] is in the rotated Lorentz cone
  // [           2 * x(2)    ]
  // [3 * x(0)            + 1]
  // [                      4]
  
  auto x = CreateVectorDecisionVariable<4>("x");
  Matrix<Expression, 5, 1> e;
  e << 2 * x(0) + 3 * x(2) + 3, x(0) - 4 * x(2), 2 * x(2), 3 * x(0) + 1, 4;
  CheckParsedSymbolicRotatedLorentzConeConstraint(e);
}

}  // namespace test
}  // namespace internal
}  // namespace solvers
}  // namespace drake
