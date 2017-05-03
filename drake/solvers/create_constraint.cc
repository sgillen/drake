#include "drake/solvers/create_constraint.h"

#include "drake/common/symbolic_formula.h"
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

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

using internal::DecomposeLinearExpression;
using internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap;
using internal::ExtractAndAppendVariablesFromExpression;
using internal::ExtractVariablesFromExpression;
using internal::SymbolicError;

Binding<Constraint> ParseLinearConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  DRAKE_ASSERT(v.rows() == lb.rows() && v.rows() == ub.rows());

  // Setup map_var_to_index and var_vec.
  // such that map_var_to_index[var(i)] = i
  unordered_map<Variable::Id, int> map_var_to_index;
  VectorXDecisionVariable vars(0);
  for (int i = 0; i < v.size(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }

  // Construct A, new_lb, new_ub. map_var_to_index is used here.
  Eigen::MatrixXd A{Eigen::MatrixXd::Zero(v.size(), vars.size())};
  Eigen::VectorXd new_lb{v.size()};
  Eigen::VectorXd new_ub{v.size()};
  // We will determine if lb <= v <= ub is a bounding box constraint, namely
  // x_lb <= x <= x_ub.
  bool is_v_bounding_box = true;
  for (int i = 0; i < v.size(); ++i) {
    double constant_term = 0;
    int num_vi_variables = DecomposeLinearExpression(v(i), map_var_to_index,
                                                     A.row(i), &constant_term);
    if (num_vi_variables == 0 &&
        !(lb(i) <= constant_term && constant_term <= ub(i))) {
      // Unsatisfiable constraint with no variables, such as 1 <= 0 <= 2
      throw SymbolicError(v(i), lb(i), ub(i), "unsatisfiable but called with"
                                              " CreateLinearConstraint");

    } else {
      new_lb(i) = lb(i) - constant_term;
      new_ub(i) = ub(i) - constant_term;
      if (num_vi_variables != 1) {
        is_v_bounding_box = false;
      }
    }
  }
  if (is_v_bounding_box) {
    // If every lb(i) <= v(i) <= ub(i) is a bounding box constraint, then
    // formulate a bounding box constraint x_lb <= x <= x_ub
    VectorXDecisionVariable bounding_box_x(v.size());
    for (int i = 0; i < v.size(); ++i) {
      // v(i) is in the form of c * x
      double x_coeff = 0;
      for (const auto& x : v(i).GetVariables()) {
        const double coeff = A(i, map_var_to_index[x.get_id()]);
        if (coeff != 0) {
          x_coeff += coeff;
          bounding_box_x(i) = x;
        }
      }
      if (x_coeff > 0) {
        new_lb(i) /= x_coeff;
        new_ub(i) /= x_coeff;
      } else {
        const double lb_i = new_lb(i);
        new_lb(i) = new_ub(i) / x_coeff;
        new_ub(i) = lb_i / x_coeff;
      }
    }
    return CreateBinding(make_shared<BoundingBoxConstraint>(new_lb, new_ub),
                         bounding_box_x);
  } else {
    return CreateBinding(make_shared<LinearConstraint>(A, new_lb, new_ub),
                         vars);
  }
}


Binding<Constraint> ParseLinearConstraint(const set<Formula>& formulas) {
  const auto n = formulas.size();

  // Decomposes a set of formulas into a 1D-vector of expressions, `v`, and two
  // 1D-vector of double `lb` and `ub`.
  VectorX<Expression> v{n};
  Eigen::VectorXd lb{n};
  Eigen::VectorXd ub{n};
  int i{0};  // index variable used in the loop
  // After the following loop, we call `AddLinearEqualityConstraint`
  // if `are_all_formulas_equal` is still true. Otherwise, we call
  // `AddLinearConstraint`.  on the value of this Boolean flag.
  bool are_all_formulas_equal{true};
  for (const Formula& f : formulas) {
    if (is_equal_to(f)) {
      // f := (lhs == rhs)
      //      (lhs - rhs == 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = 0.0;
      ub(i) = 0.0;
    } else if (is_less_than_or_equal_to(f)) {
      // f := (lhs <= rhs)
      //      (-∞ <= lhs - rhs <= 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = -numeric_limits<double>::infinity();
      ub(i) = 0.0;
      are_all_formulas_equal = false;
    } else if (is_greater_than_or_equal_to(f)) {
      // f := (lhs >= rhs)
      //      (∞ >= lhs - rhs >= 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = 0.0;
      ub(i) = numeric_limits<double>::infinity();
      are_all_formulas_equal = false;
    } else {
      ostringstream oss;
      oss << "CreateLinearConstraint(const set<Formula>& "
          << "formulas) is called while its argument 'formulas' includes "
          << "a formula " << f
          << " which is not a relational formula using one of {==, <=, >=} "
          << "operators.";
      throw runtime_error(oss.str());
    }
    ++i;
  }
  if (are_all_formulas_equal) {
    return ParseLinearEqualityConstraint(v, lb);
  } else {
    return ParseLinearConstraint(v, lb, ub);
  }
}

Binding<Constraint> ParseLinearConstraint(const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearEqualityConstraint(e1 - e2, 0.0);
  } else if (is_greater_than_or_equal_to(f)) {
    // e1 >= e2  ->  e1 - e2 >= 0  ->  0 <= e1 - e2 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearConstraint(e1 - e2, 0.0,
                                 numeric_limits<double>::infinity());
  } else if (is_less_than_or_equal_to(f)) {
    // e1 <= e2  ->  0 <= e2 - e1  ->  0 <= e2 - e1 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearConstraint(e2 - e1, 0.0,
                                 numeric_limits<double>::infinity());
  }
  if (is_conjunction(f)) {
    return ParseLinearConstraint(get_operands(f));
  }
  ostringstream oss;
  oss << "ParseLinearConstraint is called with a formula "
      << f
      << " which is neither a relational formula using one of {==, <=, >=} "
         "operators nor a conjunction of those relational formulas.";
  throw runtime_error(oss.str());
}


Binding<LinearEqualityConstraint>
ParseLinearEqualityConstraint(const set<Formula>& formulas) {
  const auto n = formulas.size();
  // Decomposes a set of formulas, `{e₁₁ == e₁₂, ..., eₙ₁ == eₙ₂}`
  // into a 1D-vector of expressions, `v = [e₁₁ - e₁₂, ..., eₙ₁ - eₙ₂]`.
  VectorX<symbolic::Expression> v{n};
  int i{0};  // index variable used in the loop
  for (const symbolic::Formula& f : formulas) {
    if (is_equal_to(f)) {
      // f := (lhs == rhs)
      //      (lhs - rhs == 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
    } else {
      ostringstream oss;
      oss << "MathematicalProgram::AddLinearEqualityConstraint(const "
          << "set<Formula>& formulas) is called while its argument 'formulas' "
          << "includes a non-equality formula " << f << ".";
      throw runtime_error(oss.str());
    }
    ++i;
  }
  return ParseLinearEqualityConstraint(v, Eigen::VectorXd::Zero(n));
}

Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearEqualityConstraint(e1 - e2, 0.0);
  }
  if (is_conjunction(f)) {
    return ParseLinearEqualityConstraint(get_operands(f));
  }
  ostringstream oss;
  oss << "MathematicalProgram::AddLinearConstraint is called with a formula "
      << f
      << " which is neither an equality formula nor a conjunction of equality "
         "formulas.";
  throw runtime_error(oss.str());
}

Binding<LinearEqualityConstraint> DoParseLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  DRAKE_DEMAND(v.rows() == b.rows());
  VectorXDecisionVariable vars(0);
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.rows(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }
  // TODO(hongkai.dai): use sparse matrix.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(v.rows(), vars.rows());
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(v.rows());
  for (int i = 0; i < v.rows(); ++i) {
    double constant_term(0);
    DecomposeLinearExpression(v(i), map_var_to_index, A.row(i), &constant_term);
    beq(i) = b(i) - constant_term;
  }
  return CreateBinding(make_shared<LinearEqualityConstraint>(A, beq), vars);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
