#include "symbolic_extract.h"

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
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace internal {

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

string SymbolicError::make_string(const symbolic::Expression& e,
                                  const double lb, const double ub,
                                  const std::string& msg) {
  ostringstream oss;
  oss << "Constraint " << lb << " <= " << e << " <= " << ub << " is " << msg
      << ".";
  return oss.str();
}
std::string SymbolicError::make_string(const symbolic::Expression& e,
                                       const std::string& msg) {
  std::ostringstream oss;
  oss << "Constraint " << e << " is " << msg << ".";
  return oss.str();
}
SymbolicError::SymbolicError(const symbolic::Expression& e,
                             const std::string& msg)
    : std::runtime_error{make_string(e, msg)}
{}
SymbolicError::SymbolicError(const symbolic::Expression& e,
                             const double lb,
                             const double ub,
                             const std::string& msg)
    : runtime_error{make_string(e, lb, ub, msg)}
{}

// Given an expression `e`, extract all variables inside `e`, append these
// variables to `vars` if they are not included in `vars` yet.
// @param[in] e  A symbolic expression.
// @param[in,out] vars  As an input, `vars` contain the variables before
// extracting expression `e`. As an output, the variables in `e` that were not
// included in `vars`, will be appended to the end of `vars`.
// @param[in,out] map_var_to_index. map_var_to_index is of the same size as
// `vars`, and map_var_to_index[vars(i).get_id()] = i. This invariance holds
// for map_var_to_index both as the input and as the output.
void ExtractAndAppendVariablesFromExpression(
    const Expression& e, VectorXDecisionVariable* vars,
    unordered_map<Variable::Id, int>* map_var_to_index) {
  DRAKE_DEMAND(static_cast<int>(map_var_to_index->size()) == vars->size());
  for (const Variable& var : e.GetVariables()) {
    if (map_var_to_index->find(var.get_id()) == map_var_to_index->end()) {
      map_var_to_index->emplace(var.get_id(), vars->size());
      vars->conservativeResize(vars->size() + 1, Eigen::NoChange);
      (*vars)(vars->size() - 1) = var;
    }
  }
}

/**
 * Given a vector of linear expressions v, decompose it to
 * \f$ v = A vars + b \f$
 * @param[in] v A vector of linear expressions
 * @param[out] A The matrix containing the linear coefficients.
 * @param[out] b The vector containing all the constant terms.
 * @param[out] vars All variables.
 */
void DecomposeLinearExpression(const Eigen::Ref<const VectorX<Expression>>& v,
                               Eigen::MatrixXd* A, Eigen::VectorXd* b,
                               VectorXDecisionVariable* vars) {
  // 0. Setup map_var_to_index and var_vec.
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.size(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), vars, &map_var_to_index);
  }

  // 2. Construct decompose v as
  // v = A * vars + b
  *A = Eigen::MatrixXd::Zero(v.rows(), vars->rows());
  *b = Eigen::VectorXd::Zero(v.rows());
  for (int i{0}; i < v.size(); ++i) {
    const Expression& e_i{v(i)};
    DecomposeLinearExpression(e_i, map_var_to_index, A->row(i), b->data() + i);
  }
}

// Given an expression `e`, extracts all variables inside `e`.
// @param[in] e A symbolic expression.
// @retval pair pair.first is the variables in `e`. pair.second is the mapping
// from the variable ID to the index in pair.first, such that
// pair.second[pair.first(i).get_id()] = i
pair<VectorXDecisionVariable, unordered_map<Variable::Id, int>>
ExtractVariablesFromExpression(const Expression& e) {
  int var_count = 0;
  const symbolic::Variables var_set = e.GetVariables();
  VectorXDecisionVariable vars(var_set.size());
  unordered_map<Variable::Id, int> map_var_to_index{};
  map_var_to_index.reserve(var_set.size());
  for (const Variable& var : var_set) {
    map_var_to_index.emplace(var.get_id(), var_count);
    vars(var_count++) = var;
  }
  return make_pair(vars, map_var_to_index);
}

void DecomposeQuadraticExpressionWithMonomialToCoeffMap(
    const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map,
    const unordered_map<Variable::Id, int>& map_var_to_index, int num_variables,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c) {
  DRAKE_DEMAND(Q->rows() == num_variables);
  DRAKE_DEMAND(Q->cols() == num_variables);
  DRAKE_DEMAND(b->rows() == num_variables);
  Q->setZero();
  b->setZero();
  *c = 0;
  for (const auto& p : monomial_to_coeff_map) {
    DRAKE_ASSERT(is_constant(p.second));
    DRAKE_DEMAND(!is_zero(p.second));
    const double coefficient = get_constant_value(p.second);
    const symbolic::Monomial& p_monomial = p.first;
    if (p_monomial.total_degree() > 2) {
      ostringstream oss;
      oss << p.first
          << " has order higher than 2, cannot be handled by "
             "DecomposeQuadraticExpressionWithMonomialToCoeffMap"
          << endl;
      throw runtime_error(oss.str());
    }
    const auto& monomial_powers = p_monomial.get_powers();
    if (monomial_powers.size() == 2) {
      // cross terms.
      auto it = monomial_powers.begin();
      const int x1_index = map_var_to_index.at(it->first);
      DRAKE_DEMAND(it->second == 1);
      ++it;
      const int x2_index = map_var_to_index.at(it->first);
      DRAKE_DEMAND(it->second == 1);
      (*Q)(x1_index, x2_index) += coefficient;
      (*Q)(x2_index, x1_index) = (*Q)(x1_index, x2_index);
    } else if (monomial_powers.size() == 1) {
      // Two cases
      // 1. quadratic term a*x^2
      // 2. linear term b*x
      auto it = monomial_powers.begin();
      DRAKE_DEMAND(it->second == 2 || it->second == 1);
      const int x_index = map_var_to_index.at(it->first);
      if (it->second == 2) {
        // quadratic term a * x^2
        (*Q)(x_index, x_index) += 2 * coefficient;
      } else if (it->second == 1) {
        // linear term b * x.
        (*b)(x_index) += coefficient;
      }
    } else {
      // constant term.
      *c += coefficient;
    }
  }
}


}
} // namespace solvers
} // namespace drake




