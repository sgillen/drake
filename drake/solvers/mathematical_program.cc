#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <cstddef>
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
#include "drake/solvers/symbolic_extraction.h"

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

using internal::DecomposeLinearExpression;
using internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap;
using internal::ExtractAndAppendVariablesFromExpression;
using internal::ExtractVariablesFromExpression;
using internal::SymbolicError;

namespace {

// Solver for simple linear systems of equalities
AttributesSet kLinearSystemSolverCapabilities = kLinearEqualityConstraint;

// Solver for equality-constrained QPs
AttributesSet kEqualityConstrainedQPCapabilities =
    (kQuadraticCost | kLinearCost | kLinearEqualityConstraint);

// Solver for Linear Complementarity Problems (LCPs)
AttributesSet kMobyLcpCapabilities = kLinearComplementarityConstraint;

// Gurobi solver capabilities.
AttributesSet kGurobiCapabilities =
    (kLinearEqualityConstraint | kLinearConstraint | kLorentzConeConstraint |
     kRotatedLorentzConeConstraint | kLinearCost | kQuadraticCost |
     kBinaryVariable);

// Mosek solver capabilities.
AttributesSet kMosekCapabilities =
    (kLinearEqualityConstraint | kLinearConstraint | kLorentzConeConstraint |
     kRotatedLorentzConeConstraint | kLinearCost | kQuadraticCost |
     kPositiveSemidefiniteConstraint | kBinaryVariable);

// Solvers for generic systems of constraints and costs.
AttributesSet kGenericSolverCapabilities =
    (kGenericCost | kGenericConstraint | kQuadraticCost | kQuadraticConstraint |
     kLorentzConeConstraint | kRotatedLorentzConeConstraint | kLinearCost |
     kLinearConstraint | kLinearEqualityConstraint);

// Returns true iff no capabilities are in required and not in available.
bool is_satisfied(AttributesSet required, AttributesSet available) {
  return ((required & ~available) == kNoCapabilities);
}
}  // namespace

enum {
  INITIAL_VARIABLE_ALLOCATION_NUM = 100
};  // not const static int because the VectorXd constructor takes a reference
// to int so it is odr-used (see
// https://gcc.gnu.org/wiki/VerboseDiagnostics#missing_static_const_definition)

MathematicalProgram::MathematicalProgram()
    : num_vars_(0),
      x_initial_guess_(
          static_cast<Eigen::Index>(INITIAL_VARIABLE_ALLOCATION_NUM)),
      solver_result_(0),
      optimal_cost_(numeric_limits<double>::quiet_NaN()),
      required_capabilities_(kNoCapabilities),
      ipopt_solver_(new IpoptSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver<double>()),
      linear_system_solver_(new LinearSystemSolver()),
      equality_constrained_qp_solver_(new EqualityConstrainedQPSolver()),
      gurobi_solver_(new GurobiSolver()),
      mosek_solver_(new MosekSolver()) {}

MatrixXDecisionVariable MathematicalProgram::NewVariables(
    VarType type, int rows, int cols, bool is_symmetric,
    const vector<string>& names) {
  MatrixXDecisionVariable decision_variable_matrix(rows, cols);
  NewVariables_impl(type, names, is_symmetric, decision_variable_matrix);
  return decision_variable_matrix;
}

VectorXDecisionVariable MathematicalProgram::NewVariables(
    VarType type, int rows, const vector<string>& names) {
  return NewVariables(type, rows, 1, false, names);
}

VectorXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, const vector<string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, names);
}

MatrixXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, size_t cols, const vector<string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, cols, false, names);
}

VectorXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, const string& name) {
  vector<string> names(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + "(" + to_string(i) + ")";
  }
  return NewContinuousVariables(rows, names);
}

MatrixXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, size_t cols, const string& name) {
  vector<string> names(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] = name + "(" + to_string(i) + "," + to_string(j) + ")";
      ++count;
    }
  }
  return NewContinuousVariables(rows, cols, names);
}

MatrixXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, size_t cols, const vector<string>& names) {
  return NewVariables(VarType::BINARY, rows, cols, false, names);
}

MatrixXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, size_t cols, const string& name) {
  vector<string> names(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] = name + "(" + to_string(i) + "," + to_string(j) + ")";
      ++count;
    }
  }
  return NewBinaryVariables(rows, cols, names);
}

MatrixXDecisionVariable MathematicalProgram::NewSymmetricContinuousVariables(
    size_t rows, const vector<string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, rows, true, names);
}

MatrixXDecisionVariable MathematicalProgram::NewSymmetricContinuousVariables(
    size_t rows, const string& name) {
  vector<string> names(rows * (rows + 1) / 2);
  int count = 0;
  for (int j = 0; j < static_cast<int>(rows); ++j) {
    for (int i = j; i < static_cast<int>(rows); ++i) {
      names[count] = name + "(" + to_string(i) + "," + to_string(j) + ")";
      ++count;
    }
  }
  return NewVariables(VarType::CONTINUOUS, rows, rows, true, names);
}

VectorXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, const string& name) {
  vector<string> names(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + "(" + to_string(i) + ")";
  }
  return NewVariables(VarType::BINARY, rows, names);
}

namespace {

template <typename To, typename From>
Binding<To> BindingUpcast(const Binding<From>& binding) {
  auto constraint = std::dynamic_pointer_cast<To>(binding.constraint());
  DRAKE_DEMAND(constraint != nullptr);
  return Binding<To>(constraint, binding.variables());
}

}  // anonymous namespace

Binding<Cost> MathematicalProgram::AddCost(const Binding<Cost>& binding) {
  // See AddCost(const Binding<Constraint>&) for explanation
  Cost* cost = binding.constraint().get();
  if (dynamic_cast<QuadraticCost*>(cost)) {
    return AddCost(BindingUpcast<QuadraticCost>(binding));
  } else if (dynamic_cast<LinearCost*>(cost)) {
    return AddCost(BindingUpcast<LinearCost>(binding));
  } else {
    required_capabilities_ |= kGenericCost;
    generic_costs_.push_back(binding);
    return generic_costs_.back();
  }
}

Binding<Cost> MathematicalProgram::AddCost(
    const shared_ptr<Cost>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateBinding(obj, vars));
}

Binding<LinearCost> MathematicalProgram::AddCost(
    const Binding<LinearCost>& binding) {
  required_capabilities_ |= kLinearCost;
  DRAKE_ASSERT(binding.constraint()->num_constraints() == 1 &&
               binding.constraint()->A().cols() ==
                   static_cast<int>(binding.GetNumElements()));
  CheckIsDecisionVariable(binding.variables());
  linear_costs_.push_back(binding);
  return linear_costs_.back();
}

Binding<LinearCost> MathematicalProgram::AddCost(
    const shared_ptr<LinearCost>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateBinding(obj, vars));
}

Binding<LinearCost> MathematicalProgram::AddLinearCost(const Expression& e) {
  return AddCost(CreateLinearCost(e));
}

Binding<LinearCost> MathematicalProgram::AddLinearCost(
    const Eigen::Ref<const Eigen::VectorXd>& c,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateLinearCost(c), vars);
}

Binding<QuadraticCost> MathematicalProgram::AddCost(
    const Binding<QuadraticCost>& binding) {
  required_capabilities_ |= kQuadraticCost;
  DRAKE_ASSERT(binding.constraint()->Q().rows() ==
                   static_cast<int>(binding.GetNumElements()) &&
               binding.constraint()->b().rows() ==
                   static_cast<int>(binding.GetNumElements()));
  CheckIsDecisionVariable(binding.variables());
  quadratic_costs_.push_back(binding);
  return quadratic_costs_.back();
}

Binding<QuadraticCost> MathematicalProgram::AddCost(
    const shared_ptr<QuadraticCost>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateBinding(obj, vars));
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticCost(
    const Expression& e) {
  return AddCost(CreateQuadraticCost(e));
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateQuadraticErrorCost(Q, x_desired), vars);
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(CreateQuadraticCost(Q, b), vars);
}

Binding<PolynomialCost> MathematicalProgram::AddPolynomialCost(
    const Expression& e) {
  return AddCost(CreatePolynomialCost(e));
}

Binding<Cost> MathematicalProgram::AddCost(const Expression& e) {
  return AddCost(CreateCost(e));
}

Binding<Constraint> MathematicalProgram::AddConstraint(
    const Binding<Constraint>& binding) {
  // TODO(eric.cousineau): Use alternative to RTTI.
  // Move kGenericConstraint, etc. to Constraint. Dispatch based on this
  // information. As it is, this causes extra work when we explicitly want a
  // generic constraint.

  // If we get here, then this was possibly a dynamically-simplified
  // constraint. Determine correct container. As last resort, add to generic
  // constraints.
  Constraint* constraint = binding.constraint().get();
  // Check constraints types in reverse order, such that classes that inherit
  // from other classes will not be prematurely added to less specific (or
  // incorrect) container.
  if (dynamic_cast<LinearMatrixInequalityConstraint*>(constraint)) {
    return AddConstraint(
        BindingUpcast<LinearMatrixInequalityConstraint>(binding));
  } else if (dynamic_cast<PositiveSemidefiniteConstraint*>(constraint)) {
    return AddConstraint(
        BindingUpcast<PositiveSemidefiniteConstraint>(binding));
  } else if (dynamic_cast<RotatedLorentzConeConstraint*>(constraint)) {
    return AddConstraint(BindingUpcast<RotatedLorentzConeConstraint>(binding));
  } else if (dynamic_cast<LorentzConeConstraint*>(constraint)) {
    return AddConstraint(BindingUpcast<LorentzConeConstraint>(binding));
  } else if (dynamic_cast<BoundingBoxConstraint*>(constraint)) {
    return AddConstraint(BindingUpcast<BoundingBoxConstraint>(binding));
  } else if (dynamic_cast<LinearEqualityConstraint*>(constraint)) {
    return AddConstraint(BindingUpcast<LinearEqualityConstraint>(binding));
  } else if (dynamic_cast<LinearConstraint*>(constraint)) {
    return AddConstraint(BindingUpcast<LinearConstraint>(binding));
  } else {
    required_capabilities_ |= kGenericConstraint;
    generic_constraints_.push_back(binding);
    return generic_constraints_.back();
  }
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Expression& e, const double lb, const double ub) {
  return AddConstraint(CreateLinearConstraint(e, lb, ub));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  return AddConstraint(CreateLinearConstraint(v, lb, ub));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const set<Formula>& formulas) {
  return AddConstraint(CreateLinearConstraint(formulas));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Formula& f) {
  return AddConstraint(CreateLinearConstraint(f));
}

Binding<Constraint> MathematicalProgram::AddConstraint(
    shared_ptr<Constraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(CreateBinding(con, vars));
}

Binding<LinearConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearConstraint>& binding) {
  required_capabilities_ |= kLinearConstraint;
  // TODO(eric.cousineau): This is a good assertion... But seems out of place,
  // possibly redundant w.r.t. the binding infrastructure.
  DRAKE_ASSERT(binding.constraint()->A().cols() ==
               static_cast<int>(binding.GetNumElements()));
  // TODO(eric.cousineau): Move this and other checks to a generic
  // BindingCheck() (to handle checking for a unique name, or assigning a
  // default name, etc.)
  CheckIsDecisionVariable(binding.variables());
  linear_constraints_.push_back(binding);
  return linear_constraints_.back();
}

Binding<LinearConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<LinearConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(CreateBinding(con, vars));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<LinearConstraint> con = make_shared<LinearConstraint>(A, lb, ub);
  return AddConstraint(CreateLinearConstraint(A, lb, ub), vars);
}

Binding<LinearEqualityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearEqualityConstraint>& binding) {
  required_capabilities_ |= kLinearEqualityConstraint;
  DRAKE_ASSERT(binding.constraint()->A().cols() ==
               static_cast<int>(binding.GetNumElements()));
  linear_equality_constraints_.push_back(binding);
  return linear_equality_constraints_.back();
}

Binding<LinearEqualityConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<LinearEqualityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(CreateBinding(con, vars));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const Expression& e,
                                                 double b) {
  return AddConstraint(CreateLinearEqualityConstraint(e, b));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(
    const set<Formula>& formulas) {
  return AddConstraint(CreateLinearEqualityConstraint(formulas));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const Formula& f) {
  return AddConstraint(CreateLinearEqualityConstraint(f));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
    const Eigen::Ref<const Eigen::VectorXd>& beq,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(CreateLinearEqualityConstraint(Aeq, beq), vars);
}

Binding<BoundingBoxConstraint> MathematicalProgram::AddConstraint(
    const Binding<BoundingBoxConstraint>& binding) {
  required_capabilities_ |= kLinearConstraint;
  DRAKE_ASSERT(binding.constraint()->num_constraints() ==
               binding.GetNumElements());
  bbox_constraints_.push_back(binding);
  return bbox_constraints_.back();
}

Binding<BoundingBoxConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<BoundingBoxConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<BoundingBoxConstraint>(con, vars));
}

Binding<LorentzConeConstraint> MathematicalProgram::AddConstraint(
    const Binding<LorentzConeConstraint>& binding) {
  required_capabilities_ |= kLorentzConeConstraint;
  CheckIsDecisionVariable(binding.variables());
  lorentz_cone_constraint_.push_back(binding);
  return lorentz_cone_constraint_.back();
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 2);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return AddLorentzConeConstraint(A, b, vars);
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Expression& linear_expr, const Expression& quadratic_expr) {
  const auto& quadratic_p = ExtractVariablesFromExpression(quadratic_expr);
  const auto& quadratic_vars = quadratic_p.first;
  const auto& quadratic_var_to_index_map = quadratic_p.second;
  const auto& monomial_to_coeff_map = symbolic::DecomposePolynomialIntoMonomial(
      quadratic_expr, quadratic_expr.GetVariables());
  Eigen::MatrixXd Q(quadratic_vars.size(), quadratic_vars.size());
  Eigen::VectorXd b(quadratic_vars.size());
  double a;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, quadratic_var_to_index_map, quadratic_vars.size(),
      &Q, &b, &a);
  // The constraint that the linear expression v1 satisfying
  // v1 >= sqrt(0.5 * x' * Q * x + b' * x + a), is equivalent to the vector
  // [z; y] being within a Lorentz cone, where
  // z = v1
  // y = [1/sqrt(2) * (R * x + R⁻ᵀb); sqrt(a - 0.5 * bᵀ * Q⁻¹ * a)]
  // R is the matrix satisfying Rᵀ * R = Q

  VectorX<Expression> expr{};

  double constant;  // constant is a - 0.5 * bᵀ * Q⁻¹ * a
  // If Q is strictly positive definite, then use LLT
  Eigen::LLT<Eigen::MatrixXd> llt_Q(Q.selfadjointView<Eigen::Upper>());
  if (llt_Q.info() == Eigen::Success) {
    Eigen::MatrixXd R = llt_Q.matrixU();
    expr.resize(2 + R.rows());
    expr(0) = linear_expr;
    expr.segment(1, R.rows()) =
        1.0 / std::sqrt(2) * (R * quadratic_vars + llt_Q.matrixL().solve(b));
    constant = a - 0.5 * b.dot(llt_Q.solve(b));
  } else {
    // Q is not strictly positive definite.
    // First check if Q is zero.
    const bool is_Q_zero = (Q.array() == 0).all();

    if (is_Q_zero) {
      // Now check if the linear term b is zero. If both Q and b are zero, then
      // add the linear constraint linear_expr >= sqrt(a); otherwise throw a
      // runtime error.
      const bool is_b_zero = (b.array() == 0).all();
      if (!is_b_zero) {
        ostringstream oss;
        oss << "Expression " << quadratic_expr
            << " is not quadratic, cannot call AddLorentzConeConstraint.\n";
        throw runtime_error(oss.str());
      } else {
        if (a < 0) {
          ostringstream oss;
          oss << "Expression " << quadratic_expr
              << " is negative, cannot call AddLorentzConeConstraint.\n";
          throw runtime_error(oss.str());
        }
        Vector2<Expression> expr_constant_quadratic(linear_expr, std::sqrt(a));
        return AddLorentzConeConstraint(expr_constant_quadratic);
      }
    }
    // Q is not strictly positive, nor is it zero. Use LDLT to decompose Q
    // into R * Rᵀ.
    // Question: is there a better way to compute R * x and R⁻ᵀb? The following
    // code is really ugly.
    Eigen::LDLT<Eigen::MatrixXd> ldlt_Q(Q.selfadjointView<Eigen::Upper>());
    if (ldlt_Q.info() != Eigen::Success || !ldlt_Q.isPositive()) {
      ostringstream oss;
      oss << "Expression" << quadratic_expr
          << " does not have a positive semidefinite Hessian. Cannot be called "
             "with AddLorentzConeConstraint.\n";
      throw runtime_error(oss.str());
    }
    Eigen::MatrixXd R1 = ldlt_Q.matrixU();
    for (int i = 0; i < R1.rows(); ++i) {
      for (int j = 0; j < i; ++j) {
        R1(i, j) = 0;
      }
      const double d_sqrt = std::sqrt(ldlt_Q.vectorD()(i));
      for (int j = i; j < R1.cols(); ++j) {
        R1(i, j) *= d_sqrt;
      }
    }
    Eigen::MatrixXd R = R1 * ldlt_Q.transpositionsP();

    expr.resize(2 + R1.rows());
    expr(0) = linear_expr;
    // expr.segment(1, R1.rows()) = 1/sqrt(2) * (R * x + R⁻ᵀb)
    expr.segment(1, R1.rows()) =
        1.0 / std::sqrt(2) *
        (R * quadratic_vars + R.transpose().fullPivHouseholderQr().solve(b));
    constant = a - 0.5 * b.dot(ldlt_Q.solve(b));
  }
  if (constant < 0) {
    ostringstream oss;
    oss << "Expression " << quadratic_expr
        << " is not guaranteed to be non-negative, cannot call it with "
           "AddLorentzConeConstraint.\n";
    throw runtime_error(oss.str());
  }
  expr(expr.rows() - 1) = std::sqrt(constant);
  return AddLorentzConeConstraint(expr);
}

Binding<LorentzConeConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<LorentzConeConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<LorentzConeConstraint>(con, vars));
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<LorentzConeConstraint> constraint =
      make_shared<LorentzConeConstraint>(A, b);
  return AddConstraint(Binding<LorentzConeConstraint>(constraint, vars));
}

Binding<RotatedLorentzConeConstraint> MathematicalProgram::AddConstraint(
    const Binding<RotatedLorentzConeConstraint>& binding) {
  required_capabilities_ |= kRotatedLorentzConeConstraint;
  CheckIsDecisionVariable(binding.variables());
  rotated_lorentz_cone_constraint_.push_back(binding);
  return rotated_lorentz_cone_constraint_.back();
}

Binding<RotatedLorentzConeConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<RotatedLorentzConeConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<RotatedLorentzConeConstraint>(con, vars));
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 3);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return AddRotatedLorentzConeConstraint(A, b, vars);
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<RotatedLorentzConeConstraint> constraint =
      make_shared<RotatedLorentzConeConstraint>(A, b);
  return AddConstraint(constraint, vars);
}

Binding<BoundingBoxConstraint> MathematicalProgram::AddBoundingBoxConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<BoundingBoxConstraint> constraint =
      make_shared<BoundingBoxConstraint>(lb, ub);
  return AddConstraint(Binding<BoundingBoxConstraint>(constraint, vars));
}

Binding<LinearComplementarityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearComplementarityConstraint>& binding) {
  required_capabilities_ |= kLinearComplementarityConstraint;

  // TODO(eric.cousineau): Consider checking bitmask rather than list sizes

  // Linear Complementarity Constraint cannot currently coexist with any
  // other types of constraint or cost.
  // (TODO(ggould-tri) relax this to non-overlapping bindings, possibly by
  // calling multiple solvers.)
  DRAKE_ASSERT(generic_constraints_.empty());
  DRAKE_ASSERT(generic_costs_.empty());
  DRAKE_ASSERT(quadratic_costs_.empty());
  DRAKE_ASSERT(linear_costs_.empty());
  DRAKE_ASSERT(linear_constraints_.empty());
  DRAKE_ASSERT(linear_equality_constraints_.empty());
  DRAKE_ASSERT(bbox_constraints_.empty());
  DRAKE_ASSERT(lorentz_cone_constraint_.empty());
  DRAKE_ASSERT(rotated_lorentz_cone_constraint_.empty());

  linear_complementarity_constraints_.push_back(binding);
  return linear_complementarity_constraints_.back();
}

Binding<LinearComplementarityConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<LinearComplementarityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<LinearComplementarityConstraint>(con, vars));
}

Binding<LinearComplementarityConstraint>
MathematicalProgram::AddLinearComplementarityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& M,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<LinearComplementarityConstraint> constraint =
      make_shared<LinearComplementarityConstraint>(M, q);
  return AddConstraint(constraint, vars);
}

Binding<Constraint> MathematicalProgram::AddPolynomialConstraint(
    const VectorXPoly& polynomials,
    const vector<Polynomiald::VarType>& poly_vars, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  // Polynomials that are actually affine (a sum of linear terms + a
  // constant) can be special-cased.  Other polynomials are treated as
  // generic for now.
  // TODO(ggould-tri) There may be other such special easy cases.
  bool all_affine = true;
  for (int i = 0; i < polynomials.rows(); i++) {
    if (!polynomials[i].IsAffine()) {
      all_affine = false;
      break;
    }
  }
  if (all_affine) {
    Eigen::MatrixXd linear_constraint_matrix =
        Eigen::MatrixXd::Zero(polynomials.rows(), poly_vars.size());
    Eigen::VectorXd linear_constraint_lb = lb;
    Eigen::VectorXd linear_constraint_ub = ub;
    for (int poly_num = 0; poly_num < polynomials.rows(); poly_num++) {
      for (const auto& monomial : polynomials[poly_num].GetMonomials()) {
        if (monomial.terms.size() == 0) {
          linear_constraint_lb[poly_num] -= monomial.coefficient;
          linear_constraint_ub[poly_num] -= monomial.coefficient;
        } else if (monomial.terms.size() == 1) {
          const Polynomiald::VarType term_var = monomial.terms[0].var;
          int var_num = (find(poly_vars.begin(), poly_vars.end(), term_var) -
                         poly_vars.begin());
          DRAKE_ASSERT(var_num < static_cast<int>(poly_vars.size()));
          linear_constraint_matrix(poly_num, var_num) = monomial.coefficient;
        } else {
          DRAKE_ABORT();  // Can't happen (unless isAffine() lied to us).
        }
      }
    }
    if (ub == lb) {
      auto constraint = make_shared<LinearEqualityConstraint>(
          linear_constraint_matrix, linear_constraint_ub);
      return AddConstraint(constraint, vars);
    } else {
      auto constraint = make_shared<LinearConstraint>(
          linear_constraint_matrix, linear_constraint_lb, linear_constraint_ub);
      return AddConstraint(constraint, vars);
    }
  } else {
    auto constraint =
        make_shared<PolynomialConstraint>(polynomials, poly_vars, lb, ub);
    return AddConstraint(constraint, vars);
  }
}

Binding<PositiveSemidefiniteConstraint> MathematicalProgram::AddConstraint(
    const Binding<PositiveSemidefiniteConstraint>& binding) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(math::IsSymmetric(Eigen::Map<const MatrixXDecisionVariable>(
      binding.variables().data(), binding.constraint()->matrix_rows(),
      binding.constraint()->matrix_rows())));
  positive_semidefinite_constraint_.push_back(binding);
  return positive_semidefinite_constraint_.back();
}

Binding<PositiveSemidefiniteConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<PositiveSemidefiniteConstraint> con,
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(math::IsSymmetric(symmetric_matrix_var));
  int num_rows = symmetric_matrix_var.rows();
  // TODO(hongkai.dai): this dynamic memory allocation/copying is ugly.
  VectorXDecisionVariable flat_symmetric_matrix_var(num_rows * num_rows);
  for (int i = 0; i < num_rows; ++i) {
    flat_symmetric_matrix_var.segment(i * num_rows, num_rows) =
        symmetric_matrix_var.col(i);
  }
  positive_semidefinite_constraint_.push_back(
      Binding<PositiveSemidefiniteConstraint>(con, flat_symmetric_matrix_var));
  return positive_semidefinite_constraint_.back();
}

Binding<PositiveSemidefiniteConstraint>
MathematicalProgram::AddPositiveSemidefiniteConstraint(
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  auto constraint =
      make_shared<PositiveSemidefiniteConstraint>(symmetric_matrix_var.rows());
  return AddConstraint(constraint, symmetric_matrix_var);
}

Binding<LinearMatrixInequalityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearMatrixInequalityConstraint>& binding) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(static_cast<int>(binding.constraint()->F().size()) ==
               static_cast<int>(binding.GetNumElements()) + 1);
  linear_matrix_inequality_constraint_.push_back(binding);
  return linear_matrix_inequality_constraint_.back();
}

Binding<LinearMatrixInequalityConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<LinearMatrixInequalityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<LinearMatrixInequalityConstraint>(con, vars));
}

Binding<LinearMatrixInequalityConstraint>
MathematicalProgram::AddLinearMatrixInequalityConstraint(
    const vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto constraint = make_shared<LinearMatrixInequalityConstraint>(F);
  return AddConstraint(constraint, vars);
}

size_t MathematicalProgram::FindDecisionVariableIndex(
    const Variable& var) const {
  auto it = decision_variable_index_.find(var.get_id());
  if (it == decision_variable_index_.end()) {
    ostringstream oss;
    oss << var << " is not a decision variable in the mathematical program, "
                  "when calling GetSolution.\n";
    throw runtime_error(oss.str());
  }
  return it->second;
}

MathematicalProgram::VarType MathematicalProgram::DecisionVariableType(
    const Variable& var) const {
  return decision_variable_type_[FindDecisionVariableIndex(var)];
}

double MathematicalProgram::GetSolution(const Variable& var) const {
  return x_values_[FindDecisionVariableIndex(var)];
}

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  SetDecisionVariableValues(decision_variables_, values);
}

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  DRAKE_ASSERT(values.rows() == variables.rows());
  for (int i = 0; i < values.rows(); ++i) {
    x_values_[FindDecisionVariableIndex(variables(i))] = values(i);
  }
}

void MathematicalProgram::SetDecisionVariableValue(const Variable& var,
                                                   double value) {
  x_values_[FindDecisionVariableIndex(var)] = value;
}

SolutionResult MathematicalProgram::Solve() {
  // This implementation is simply copypasta for now; in the future we will
  // want to tweak the order of preference of solvers based on the types of
  // constraints present.

  if (is_satisfied(required_capabilities_, kLinearSystemSolverCapabilities) &&
      linear_system_solver_->available()) {
    // TODO(ggould-tri) Also allow quadratic objectives whose matrix is
    // Identity: This is the objective function the solver uses anyway when
    // underconstrainted, and is fairly common in real-world problems.
    return linear_system_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_,
                          kEqualityConstrainedQPCapabilities) &&
             equality_constrained_qp_solver_->available()) {
    return equality_constrained_qp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kMosekCapabilities) &&
             mosek_solver_->available()) {
    // TODO(hongkai.dai@tri.global): based on my limited experience, Mosek is
    // faster than Gurobi for convex optimization problem. But we should run
    // a more thorough comparison.
    return mosek_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGurobiCapabilities) &&
             gurobi_solver_->available()) {
    return gurobi_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kMobyLcpCapabilities) &&
             moby_lcp_solver_->available()) {
    return moby_lcp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             snopt_solver_->available()) {
    return snopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             ipopt_solver_->available()) {
    return ipopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             nlopt_solver_->available()) {
    return nlopt_solver_->Solve(*this);
  } else {
    throw runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
