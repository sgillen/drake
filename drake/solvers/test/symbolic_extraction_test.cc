#include "drake/solvers/symbolic_extraction.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/monomial.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

using std::all_of;
using std::cref;
using std::enable_if;
using std::endl;
using std::is_permutation;
using std::is_same;
using std::make_shared;
using std::map;
using std::move;
using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

using MapVarToIndex = unordered_map<Variable::Id, int>;

namespace drake {
namespace solvers {
namespace internal {
namespace {

GTEST_TEST(SymbolicExtraction, AppendToVector) {
  Variable x("x");
  Variable y("y");
  Variable z("z");
  VectorXDecisionVariable vars_expected(3);
  vars_expected << x, y, z;

  VectorXDecisionVariable vars;
  AppendToVector(x, &vars);
  AppendToVector(y, &vars);
  AppendToVector(z, &vars);
  EXPECT_EQ(vars_expected, vars);
}


// Check expected invariance
void ExpectValidMapVarToIndex(const VectorXDecisionVariable& vars,
                             const MapVarToIndex& map_var_to_index) {
 EXPECT_EQ(vars.size(), map_var_to_index.size());
 for (int i = 0; i < vars.size(); ++i) {
   const auto& var = vars(i);
   EXPECT_EQ(i, map_var_to_index.at(var.get_id()));
 }
}

GTEST_TEST(SymbolicExtraction, ExtractVariables) {
  Variable x("x");
  Variable y("y");
  Expression e = x + y;
  VectorXDecisionVariable vars_expected(2);
  vars_expected << x, y;

  MapVarToIndex map_var_to_index;
  VectorXDecisionVariable vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);

  Variable z("z");
  e += x * (z - y);
  AppendToVector(z, &vars_expected);

  ExtractAndAppendVariablesFromExpression(e, &vars, &map_var_to_index);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);
}

GTEST_TEST(SymbolicExtraction, DecomposeQuadraticExpression) {
  const int num_var = 3;
  Variable x("x");
  Variable y("y");
  Variable z("z");

  VectorXDecisionVariable vars_expected(num_var);
  vars_expected << x, y, z;
  MatrixXd Q_expected(num_var, num_var);
  Q_expected <<
             3, 2, 1,
             4, 6, 5,
             7, 8, 9;
  VectorXd b_expected(num_var);
  b_expected << 10, 11, 12;
  double c_expected = 13;
  Expression e = vars_expected.dot(Q_expected * vars_expected + b_expected) +
      c_expected;

  MapVarToIndex map_var_to_index;
  VectorXDecisionVariable vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  const auto monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, e.GetVariables());
  // Unordered
  MatrixXd Q(num_var, num_var);
  VectorXd b(num_var);
  double c;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(monomial_to_coeff_map,
                                                     map_var_to_index,
                                                     num_var, &Q, &b, &c);
  EXPECT_EQ(vars_expected, vars);
  const double tol = 1e-14;
  EXPECT_TRUE(CompareMatrices(Q_expected, Q, tol));
  EXPECT_TRUE(CompareMatrices(b_expected, b, tol));
  EXPECT_EQ(c_expected, c);
}

}  // anonymous namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
