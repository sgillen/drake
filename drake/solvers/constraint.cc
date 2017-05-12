#include "drake/solvers/constraint.h"

#include <cmath>

#include "drake/math/matrix_util.h"

using std::abs;
using std::vector;

namespace drake {
namespace solvers {

void QuadraticConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 Eigen::VectorXd &y) const {
  y.resize(num_constraints());
  y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
}

void QuadraticConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                 AutoDiffVecXd &y) const {
  y.resize(num_constraints());
  y = .5 * x.transpose() * Q_.cast<AutoDiffXd>() * x +
      b_.cast<AutoDiffXd>().transpose() * x;
}

void LorentzConeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  Eigen::VectorXd z = A_ * x + b_;
  y.resize(num_constraints());
  y(0) = z(0);
  y(1) = pow(z(0), 2) - z.tail(z.size() - 1).squaredNorm();
}

void LorentzConeConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                   AutoDiffVecXd &y) const {
  AutoDiffVecXd z = A_.cast<AutoDiffXd>() * x + b_.cast<AutoDiffXd>();
  y.resize(num_constraints());
  y(0) = z(0);
  y(1) = pow(z(0), 2) - z.tail(z.size() - 1).squaredNorm();
}

void RotatedLorentzConeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  Eigen::VectorXd z = A_ * x + b_;
  y.resize(num_constraints());
  y(0) = z(0);
  y(1) = z(1);
  y(2) = z(0) * z(1) - z.tail(z.size() - 2).squaredNorm();
}

void RotatedLorentzConeConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd> &x, AutoDiffVecXd &y) const {
  AutoDiffVecXd z = A_.cast<AutoDiffXd>() * x + b_.cast<AutoDiffXd>();
  y.resize(num_constraints());
  y(0) = z(0);
  y(1) = z(1);
  y(2) = z(0) * z(1) - z.tail(z.size() - 2).squaredNorm();
}

void PolynomialConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::VectorXd &y) const {
  double_evaluation_point_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    double_evaluation_point_[poly_vars_[i]] = x[i];
  }
  y.resize(num_constraints());
  for (size_t i = 0; i < num_constraints(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(double_evaluation_point_);
  }
}

void PolynomialConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                  AutoDiffVecXd &y) const {
  taylor_evaluation_point_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    taylor_evaluation_point_[poly_vars_[i]] = x[i];
  }
  y.resize(num_constraints());
  for (size_t i = 0; i < num_constraints(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_);
  }
}


shared_ptr<Constraint> MakePolynomialConstraint(
    const VectorXPoly& polynomials,
    const vector<Polynomiald::VarType>& poly_vars, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub) {
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
      return make_shared<LinearEqualityConstraint>(linear_constraint_matrix,
                                                   linear_constraint_ub);
    } else {
      return make_shared<LinearConstraint>(
          linear_constraint_matrix, linear_constraint_lb, linear_constraint_ub);
    }
  } else {
    return make_shared<PolynomialConstraint>(polynomials, poly_vars, lb, ub);
  }
}


void LinearConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                              Eigen::VectorXd &y) const {
  y.resize(num_constraints());
  y = A_ * x;
}
void LinearConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                              AutoDiffVecXd &y) const {
  y.resize(num_constraints());
  y = A_.cast<AutoDiffXd>() * x;
}

void BoundingBoxConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  y.resize(num_constraints());
  y = x;
}
void BoundingBoxConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                   AutoDiffVecXd &y) const {
  y.resize(num_constraints());
  y = x;
}

void LinearComplementarityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  y.resize(num_constraints());
  y = (M_ * x) + q_;
}

void LinearComplementarityConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd> &x, AutoDiffVecXd &y) const {
  y.resize(num_constraints());
  y = (M_.cast<AutoDiffXd>() * x) + q_.cast<AutoDiffXd>();
}

bool LinearComplementarityConstraint::DoCheckSatisfied(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const double tol) const {
  // Check: x >= 0 && Mx + q >= 0 && x'(Mx + q) == 0
  Eigen::VectorXd y(num_constraints());
  DoEval(x, y);
  return (x.array() > -tol).all() && (y.array() > -tol).all() &&
         (abs(x.dot(y)) < tol);
}

bool LinearComplementarityConstraint::DoCheckSatisfied(
    const Eigen::Ref<const AutoDiffVecXd> &x,
    const double tol) const {
  AutoDiffVecXd y(num_constraints());
  DoEval(x, y);
  return (x.array() > -tol).all() && (y.array() > -tol).all() &&
         (abs(x.dot(y)) < tol);
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
               num_constraints() * num_constraints());
  Eigen::MatrixXd S(num_constraints(), num_constraints());

  for (int i = 0; i < static_cast<int>(num_constraints()); ++i) {
    S.col(i) = x.segment(i * num_constraints(), num_constraints());
  }

  DRAKE_ASSERT(S.rows() == static_cast<int>(num_constraints()));

  // This uses the lower diagonal part of S to compute the eigen values.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  y = eigen_solver.eigenvalues();
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd&) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &y) const {
  DRAKE_ASSERT(x.rows() == static_cast<int>(F_.size()) - 1);
  Eigen::MatrixXd S = F_[0];
  for (int i = 1; i < static_cast<int>(F_.size()); ++i) {
    S += x(i - 1) * F_[i];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  y = eigen_solver.eigenvalues();
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd&) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    double symmetry_tolerance)
    : Constraint(F.empty() ? 0 : F.front().rows(),
                 F.empty() ? 0 : F.size() - 1),
      F_(F.begin(), F.end()),
      matrix_rows_(F.empty() ? 0 : F.front().rows()) {
  DRAKE_DEMAND(!F.empty());
  set_bounds(Eigen::VectorXd::Zero(matrix_rows_),
             Eigen::VectorXd::Constant(
                 matrix_rows_, std::numeric_limits<double>::infinity()));
  for (const auto& Fi : F) {
    DRAKE_ASSERT(Fi.rows() == matrix_rows_);
    DRAKE_ASSERT(math::IsSymmetric(Fi, symmetry_tolerance));
  }
}
}  // namespace solvers
}  // namespace drake
