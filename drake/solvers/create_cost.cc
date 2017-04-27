#include "drake/solvers/create_cost.h"

#include "drake/solvers/symbolic_extraction.h"

namespace drake {
namespace solvers {

using std::make_shared;
using std::numeric_limits;
using std::shared_ptr;
using std::unordered_map;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

using internal::DecomposeLinearExpression;
using internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap;
using internal::ExtractAndAppendVariablesFromExpression;
using internal::ExtractVariablesFromExpression;
using internal::SymbolicError;


shared_ptr<LinearCost> CreateLinearCost(
    const Eigen::Ref<const Eigen::VectorXd>& c) {
  return make_shared<LinearCost>(c.transpose());
}

Binding<LinearCost> CreateLinearCost(const Expression& e) {
  auto p = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& var = p.first;
  const auto& map_var_to_index = p.second;
  Eigen::RowVectorXd c(var.size());
  double constant_term;
  DecomposeLinearExpression(e, map_var_to_index, c, &constant_term);
  // The constant term is ignored now.
  // TODO(hongkai.dai): support adding constant term to the cost.
  return CreateBinding(CreateLinearCost(c), var);
}


shared_ptr<QuadraticCost> CreateQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  return make_shard<QuadraticCost>(Q, b);
}

shared_ptr<QuadraticCost> CreateQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired) {
  return CreateQuadraticCost(2 * Q, -2 * Q * x_desired);
}

namespace {

Binding<QuadraticCost> CreateQuadraticCostWithMonomialToCoeffMap(
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
  return CreateBinding(CreateQuadraticCost(Q, b), vars_vec);
}

}  // anonymous namespace

Binding<QuadraticCost> CreateQuadraticCost(const Expression& e) {
  // First build an Eigen vector, that contains all the bound variables.
  const symbolic::Variables& vars = e.GetVariables();
  auto p = ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decomposes the expression into coefficients and monomials.
  const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  return CreateQuadraticCostWithMonomialToCoeffMap(
    monomial_to_coeff_map, vars_vec, map_var_to_index, this);
}

shared_ptr<QuadraticCost> CreateL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  return CreateQuadraticCost(2 * A.transpose() * A, -2 * A.transpose() * b);
}


}  // namespace solvers
}  // namespace drake
