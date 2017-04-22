#include "drake/solvers/create_cost.h"

#include <algorithm>
#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/monomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/symbolic_extract.h"

namespace drake {
namespace solvers {

using std::enable_if;
using std::endl;
using std::find;
using std::is_same;
using std::make_pair;
using std::make_shared;
using std::map;
using std::numeric_limits;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::unordered_map;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {

Binding<QuadraticConstraint> AddQuadraticCostWithMonomialToCoeffMap(
    const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map,
    const VectorXDecisionVariable& vars_vec,
    const unordered_map<Variable::Id, int>& map_var_to_index) {
  // We want to write the expression e in the form 0.5 * x' * Q * x + b' * x + c
  // TODO(hongkai.dai): use a sparse matrix to represent Q and b.
  Eigen::MatrixXd Q(vars_vec.size(), vars_vec.size());
  Eigen::VectorXd b(vars_vec.size());
  double constant_term;
  internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, map_var_to_index, vars_vec.size(), &Q, &b,
      &constant_term);
  // Now add the quadratic constraint 0.5 * x' * Q * x + b' * x
  return Binding<QuadraticConstraint>(CreateQuadraticCost(Q, b), vars_vec);
}

}  // anonymous namespace



std::shared_ptr<QuadraticCost> CreateL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
    return CreateQuadraticCost(2 * A.transpose() * A, -2 * A.transpose() * b);
}

std::shared_ptr<QuadraticCost> CreateQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired) {
  return CreateQuadraticCost(
      2 * Q, -2 * Q * x_desired, -numeric_limits<double>::infinity(),
      numeric_limits<double>::infinity());
}

Binding<QuadraticCost> CreateQuadraticCost(const symbolic::Expression& e) {
// First build an Eigen vector, that contains all the bound variables.
  const symbolic::Variables& vars = e.GetVariables();
  auto p = internal::ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decomposes the expression into coefficients and monomials.
  const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  return AddQuadraticCostWithMonomialToCoeffMap(monomial_to_coeff_map, vars_vec,
                                                map_var_to_index, this);
}

} // namespace solvers
} // namespace drake
