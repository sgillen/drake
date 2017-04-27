#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <type_traits>
#include <utility>

#include "drake/common/symbolic_expression.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/function.h"
#include "drake/common/monomial.h"


namespace drake {
namespace solvers {

/** \addtogroup LinearConstraintCreators */
/*@{*/

std::shared_ptr<LinearConstraint> CreateLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub);

Binding<LinearConstraint> CreateLinearConstraint(
    const symbolic::Expression& e, const double lb, const double ub) {
  return CreateLinearConstraint(Vector1<Expression>(e), Vector1<double>(lb),
                             Vector1<double>(ub));
}

Binding<LinearConstraint> CreateLinearConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub);

Binding<LinearConstraint> CreateLinearConstraint(
  const std::set<symbolic::Formula>& formulas);

Binding<LinearConstraint> CreateLinearConstraint(const symbolic::Formula& f);

/*@}*/  // \addtogroup LinearConstraintCreators


/** \addtogroup LinearEqualityConstraintCreators */
/*@{*/

std::shared_ptr<LinearEqualityConstraint> CreateLinearEqualityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
    const Eigen::Ref<const Eigen::VectorXd>& beq);

std::shared_ptr<LinearEqualityConstraint> CreateLinearEqualityConstraint(
    const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq) {
  return CreateLinearEqualityConstraint(a, Vector1d(beq));
}

Binding<LinearEqualityConstraint> CreateLinearEqualityConstraint(
    const symbolic::Expression& e, double b);

Binding<LinearEqualityConstraint>
CreateLinearEqualityConstraint(const std::set<symbolic::Formula>& formulas);

Binding<LinearEqualityConstraint> CreateLinearEqualityConstraint(
    const symbolic::Formula& f);


namespace internal {

Binding<LinearEqualityConstraint>
DoCreateLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b);

}  // namespace internal

namespace detail {

template<typename Derived, typename Scalar>
struct is_matrix_base_of
  : std::integral_constant<
        bool,
        // Is this to prevent an ArrayBase from being implicitly copied?
        std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value &&
        std::is_same<typename DerivedV::Scalar, Scalar>::value &&
        Derived::ColsAtCompileTime == 1
        > {};

template<typename Derived>
struct is_vector
  : std::integral_constant<bool, Derived::ColsAtCompileTime == 1> {};

template<typename Derived, typename Scalar>
struct is_matrix_base_vector_of
  : std::integral_constant<
        bool, 
        is_matrix_base_of<Derived, Scalar>::value &&
        is_vector<Derived>::value
        > {};

template<typename Derived, typename Scalar>
struct is_matrix_base_matrix_of
  : std::integral_constant<
        bool, 
        is_matrix_base_of<Derived, Scalar>::value &&
        !is_vector<Derived>::value
        > {};

}  // namespace detail

template <typename DerivedV, typename DerivedB>
typename std::enable_if<
    is_matrix_base_vector_of<DerivedV, symbolic::Expression>::value &&
    is_matrix_base_vector_of<DerivedB, double>::value,
    Binding<LinearEqualityConstraint>>::type
CreateLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& v,
                            const Eigen::MatrixBase<DerivedB>& b) {
  return internal::DoCreateLinearEqualityConstraint(v, b);
}

template <typename DerivedV, typename DerivedB>
typename std::enable_if<
    is_matrix_base_matrix_of<DerivedV, symbolic::Expression>::value &&
    is_matrix_base_matrix_of<DerivedB, double>::value,
    Binding<LinearEqualityConstraint>>::type
AddLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                            const Eigen::MatrixBase<DerivedB>& B,
                            bool lower_triangle = false) {
  if (lower_triangle) {
    DRAKE_DEMAND(V.rows() == V.cols() && B.rows() == B.cols());
  }
  DRAKE_DEMAND(V.rows() == B.rows() && V.cols() == B.cols());

  // Form the flatten version of V and B, when lower_triangle = false,
  // the flatten version is just to concatenate each column of the matrix;
  // otherwise the flatten version is to concatenate each column of the
  // lower triangular part of the matrix.
  const int V_rows = DerivedV::RowsAtCompileTime != Eigen::Dynamic
                         ? static_cast<int>(DerivedV::RowsAtCompileTime)
                         : static_cast<int>(DerivedB::RowsAtCompileTime);
  const int V_cols = DerivedV::ColsAtCompileTime != Eigen::Dynamic
                         ? static_cast<int>(DerivedV::ColsAtCompileTime)
                         : static_cast<int>(DerivedB::ColsAtCompileTime);

  if (lower_triangle) {
    const int V_triangular_size =
        V_rows != Eigen::Dynamic ? (V_rows + 1) * V_rows / 2 : Eigen::Dynamic;
    int V_triangular_size_dynamic = V.rows() * (V.rows() + 1) / 2;
    Eigen::Matrix<symbolic::Expression, V_triangular_size, 1> flat_lower_V(
        V_triangular_size_dynamic);
    Eigen::Matrix<double, V_triangular_size, 1> flat_lower_B(
        V_triangular_size_dynamic);
    int V_idx = 0;
    for (int j = 0; j < V.cols(); ++j) {
      for (int i = j; i < V.rows(); ++i) {
        flat_lower_V(V_idx) = V(i, j);
        flat_lower_B(V_idx) = B(i, j);
        ++V_idx;
      }
    }
    return CreateLinearEqualityConstraint(flat_lower_V, flat_lower_B);
  } else {
    const int V_size = V_rows != Eigen::Dynamic && V_cols != Eigen::Dynamic
                           ? V_rows * V_cols
                           : Eigen::Dynamic;
    Eigen::Matrix<symbolic::Expression, V_size, 1> flat_V(V.size());
    Eigen::Matrix<double, V_size, 1> flat_B(V.size());
    int V_idx = 0;
    for (int j = 0; j < V.cols(); ++j) {
      for (int i = 0; i < V.rows(); ++i) {
        flat_V(V_idx) = V(i, j);
        flat_B(V_idx) = B(i, j);
        ++V_idx;
      }
    }
    return CreateLinearEqualityConstraint(flat_V, flat_B);
  }
}


/*@}*/  // \addtogroup LinearEqualityConstraintCreators

}  // namespace solvers
}  // namespace drake
