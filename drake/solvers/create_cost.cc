#include "drake/solvers/create_cost.h"

#include "drake/solvers/symbolic_extraction.h"

namespace drake {
namespace solvers {
namespace internal {

using std::make_shared;
using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::unordered_map;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

using internal::DecomposeLinearExpression;
using internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap;
using internal::ExtractAndAppendVariablesFromExpression;
using internal::ExtractVariablesFromExpression;
using internal::SymbolicError;

Binding<LinearCost> ParseLinearCost(const Expression& e) {
  auto p = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& var = p.first;
  const auto& map_var_to_index = p.second;
  Eigen::RowVectorXd c(var.size());
  double constant_term;
  DecomposeLinearExpression(e, map_var_to_index, c, &constant_term);
  // The constant term is ignored now.
  // TODO(hongkai.dai): support adding constant term to the cost.
  return CreateBinding(make_shared<LinearCost>(c), var);
}

namespace {

Binding<QuadraticCost> ParseQuadraticCostWithMonomialToCoeffMap(
    const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map,
    const VectorXDecisionVariable& vars_vec,
    const unordered_map<Variable::Id, int>& map_var_to_index) {
  // We want to write the expression e in the form 0.5 * x' * Q * x + b' * x + c
  // TODO(hongkai.dai): use a sparse matrix to represent Q and b.
  Eigen::MatrixXd Q(vars_vec.size(), vars_vec.size());
  Eigen::VectorXd b(vars_vec.size());
  double constant_term;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, map_var_to_index, vars_vec.size(), &Q, &b,
      &constant_term);
  // Now add the quadratic constraint 0.5 * x' * Q * x + b' * x
  return CreateBinding(make_shared<QuadraticCost>(Q, b), vars_vec);
}

}  // anonymous namespace

Binding<QuadraticCost> ParseQuadraticCost(const Expression& e) {
  // First build an Eigen vector, that contains all the bound variables.
  const symbolic::Variables& vars = e.GetVariables();
  auto p = ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decomposes the expression into coefficients and monomials.
  const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  return ParseQuadraticCostWithMonomialToCoeffMap(monomial_to_coeff_map,
                                                  vars_vec, map_var_to_index);
}

Binding<PolynomialCost> ParsePolynomialCost(const symbolic::Expression& e) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    oss << "Expression" << e << " is not a polynomial. ParsePolynomialCost"
                                " only supports polynomial expression.\n";
    throw runtime_error(oss.str());
  }
  const symbolic::Variables& vars = e.GetVariables();
  const Polynomiald polynomial = e.ToPolynomial();
  vector<Polynomiald::VarType> polynomial_vars(vars.size());
  VectorXDecisionVariable var_vec(vars.size());
  int polynomial_var_count = 0;
  for (const auto& var : vars) {
    polynomial_vars[polynomial_var_count] = var.get_id();
    var_vec[polynomial_var_count] = var;
    ++polynomial_var_count;
  }
  return CreateBinding(make_shared<PolynomialCost>(
                           Vector1<Polynomiald>(polynomial), polynomial_vars),
                       var_vec);
}

Binding<Cost> ParseCost(const symbolic::Expression& e) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    oss << "Expression " << e << " is not a polynomial. ParseCost does not"
        << " support non-polynomial expression.\n";
    throw runtime_error(oss.str());
  }
  const symbolic::Variables& vars = e.GetVariables();
  const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  int total_degree = 0;
  for (const auto& p : monomial_to_coeff_map) {
    total_degree = std::max(total_degree, p.first.total_degree());
  }

  auto e_extracted = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& vars_vec = e_extracted.first;
  const auto& map_var_to_index = e_extracted.second;

  if (total_degree > 2) {
    return ParsePolynomialCost(e);
  } else if (total_degree == 2) {
    return ParseQuadraticCostWithMonomialToCoeffMap(monomial_to_coeff_map,
                                                    vars_vec, map_var_to_index);
  } else {
    // TODO(eric.cousineau): Dispatch to ParseLinearCost?
    Eigen::VectorXd c(vars_vec.size());
    c.setZero();
    for (const auto& p : monomial_to_coeff_map) {
      if (p.first.total_degree() == 1) {
        const Variable::Id var_id = p.first.get_powers().begin()->first;
        DRAKE_DEMAND(is_constant(p.second));
        c(map_var_to_index.at(var_id)) += get_constant_value(p.second);
      }
    }
    return CreateBinding(make_shared<LinearCost>(c), vars_vec);
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
