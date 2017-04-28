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
namespace {

// Check expected invariance
// void ExpectValidMapVarToIndex(const VectorXDecisionVariable& vars,
//                               const MapVarToIndex& map_var_to_index) {
//   EXPECT_EQ(vars.size(), map_var_to_index.size());
//   for (int i = 0; i < vars.size(); ++i) {
//     const auto& var = vars(i);
//     EXPECT_EQ(i, map_var_to_index.at(var.get_id()));
//   }
// }

GTEST_TEST(SymbolicExtraction, AppendToVector) {
//  Variable x("x");
//  Variable y("y");
//  //  Expression e = x + y;
//  VectorXDecisionVariable vars_expected(2);
//  vars_expected << x, y;
//  Variable z("z");
//
//  VectorXDecisionVariable setup(3);
//  setup << vars_expected, z;
//
////  vars_expected.conservativeResize(3);
////  vars_expected(2) = z;
//  // internal::AppendToVector(z, &vars_expected);
//
//  Variable w;
//  std::cout << w.get_name() << " " << w.get_id() << std::endl;
//  w = z;
//  std::cout << w.get_name() << " " << w.get_id() << std::endl;
//
//  vector<Variable> vars2 = {x, y};
//  vars2.resize(3);
//  vars2.push_back(z);
//
//  VectorXd f(2);
//  f << 1, 2;
//  f.conservativeResize(3);
//  f(2) = 3;
//  internal::AppendToVector(3, &f);

  VectorX<string> g(2);
  g << "x", "y";
  internal::AppendToVector("z", &g);
}

//GTEST_TEST(SymbolicExtraction, ExtractAndAppend) {
//  Variable x("x");
//  Variable y("y");
////  Expression e = x + y;
//  VectorXDecisionVariable vars_expected(2);
//  vars_expected << x, y;
//  Variable z("z");
//  vars_expected.conservativeResize(3);
//  vars_expected(2) = z;

  // {
  //   MapVarToIndex map_var_to_index;
  //   VectorXDecisionVariable vars;
  //   std::tie(vars, map_var_to_index) =
  //       internal::ExtractVariablesFromExpression(e);
  //   EXPECT_EQ(vars_expected, vars);
  //   ExpectValidMapVarToIndex(vars, map_var_to_index);
  // }

  // Test appending

  // e += x * (z - y);

  // internal::ExtractAndAppendVariablesFromExpression(e, &vars,
  //                                                   &map_var_to_index);
  // EXPECT_EQ(vars_expected, vars);
  // ExpectValidMapVarToIndex(vars, map_var_to_index);
//}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
