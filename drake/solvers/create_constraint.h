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
namespace internal {

// TODO(eric.cousineau): Use Eigen::Ref more pervasively

Binding<Constraint> ParseLinearConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub);

Binding<Constraint> ParseLinearConstraint(
    const symbolic::Expression& e, const double lb, const double ub) {
  return ParseLinearConstraint(Vector1<symbolic::Expression>(e),
                               Vector1<double>(lb), Vector1<double>(ub));
}

Binding<Constraint> ParseLinearConstraint(const symbolic::Formula& f);

Binding<Constraint> ParseLinearConstraint(
  const std::set<symbolic::Formula>& formulas);


Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b);

Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const symbolic::Expression& e, double b) {
  return ParseLinearEqualityConstraint(Vector1<symbolic::Expression>(e),
                                       Vector1d(b));
}

Binding<LinearEqualityConstraint>
ParseLinearEqualityConstraint(const std::set<symbolic::Formula>& formulas);

Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const symbolic::Formula& f);


namespace detail {

template<typename Derived, typename Scalar>
struct is_eigen_matrix_of
  : std::integral_constant<
        bool,
        // Is this to prevent an ArrayBase from being implicitly copied?
        std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value &&
        std::is_same<typename Derived::Scalar, Scalar>::value
        > {};

template<typename Derived>
struct is_eigen_vector
  : std::integral_constant<bool, Derived::ColsAtCompileTime == 1> {};

template<typename Derived, typename Scalar>
struct is_eigen_matrix_vector_of
  : std::integral_constant<
        bool, 
        detail::is_eigen_matrix_of<Derived, Scalar>::value &&
        detail::is_eigen_vector<Derived>::value
        > {};

template<typename Derived, typename Scalar>
struct is_eigen_matrix_nonvector_of
  : std::integral_constant<
        bool, 
        detail::is_eigen_matrix_of<Derived, Scalar>::value &&
        !detail::is_eigen_vector<Derived>::value
        > {};

/*
 * Determine if two Eigen bases are matrices of Expressions and doubles,
 * to then form an implicit formula.
 */
template<typename DerivedV, typename DerivedB>
struct is_eigen_matrix_formula_pair // explicitly non-vector
  : std::integral_constant<
        bool,
        detail::is_eigen_matrix_nonvector_of<
            DerivedV, symbolic::Expression>::value &&
        detail::is_eigen_matrix_nonvector_of<DerivedB, double>::value
        > {};

/*
 * Determine if two Eigen bases are vectors of Expressions and doubles,
 * to then form an implicit formula.
 */
template<typename DerivedV, typename DerivedB>
struct is_eigen_vector_formula_pair // explicitly vector
  : std::integral_constant<
        bool,
        detail::is_eigen_matrix_vector_of<
            DerivedV, symbolic::Expression>::value &&
        detail::is_eigen_matrix_vector_of<DerivedB, double>::value
        > {};

}  // namespace detail

//template <typename DerivedV, typename DerivedB>
//typename std::enable_if<
//    detail::is_eigen_vector_formula_pair<DerivedV, DerivedB>::value,
//    Binding<LinearEqualityConstraint>>::type
//ParseLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& v,
//                            const Eigen::MatrixBase<DerivedB>& b) {
//  return ParseLinearEqualityConstraint(v, b);
//}

Binding<LinearEqualityConstraint> DoParseLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/*
 * Creates linear equality constraints \f$ v = b \f$, where \p v(i) is a
 * symbolic linear expression. Throws an exception if
 * 1. @p v(i) is a non-linear expression.
 * 2. @p v(i) is a constant.
 * @tparam DerivedV An Eigen Matrix type of Expression. A column vector.
 * @tparam DerivedB An Eigen Matrix type of double. A column vector.
 * @param v v(i) is a linear symbolic expression in the form of
 * <tt> c0 + c1 * x1 + ... + cn * xn </tt> where ci is a constant and @xi is
 * a variable.
 * @param b A vector of doubles.
 * @return The newly created linear equality constraint, together with the
 * bound variables.
 */
template <typename DerivedV, typename DerivedB>
typename std::enable_if<
    detail::is_eigen_vector_formula_pair<DerivedV, DerivedB>::value,
    Binding<LinearEqualityConstraint>>::type
ParseLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                              const Eigen::MatrixBase<DerivedB>& b) {
  return DoParseLinearEqualityConstraint(V, b);
}

/**
 * Adds a linear equality constraint for a matrix of linear expression @p V,
 * such that V(i, j) = B(i, j). If V is a symmetric matrix, then the user
 * may only want to constrain the lower triangular part of V.
 * This function is meant to provide convenience to the user, it incurs
 * additional copy and memory allocation. For faster speed, add each column
 * of the matrix equality in a for loop.
 * @tparam DerivedV An Eigen Matrix type of Expression. The number of columns
 * at compile time should not be 1.
 * @tparam DerivedB An Eigen Matrix type of double.
 * @param V An Eigen Matrix of symbolic expressions. V(i, j) should be a
 * linear expression.
 * @param B An Eigen Matrix of doubles.
 * @param lower_triangle If true, then only the lower triangular part of @p V
 * is constrained, otherwise the whole matrix V is constrained. @default is
 * false.
 * @return The newly added linear equality constraint, together with the
 * bound variables.
 */
template <typename DerivedV, typename DerivedB>
typename std::enable_if<
    detail::is_eigen_matrix_formula_pair<DerivedV, DerivedB>::value,
    Binding<LinearEqualityConstraint>>::type
ParseLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
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
    constexpr int V_triangular_size =
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
    return DoParseLinearEqualityConstraint(flat_lower_V, flat_lower_B);
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
    return DoParseLinearEqualityConstraint(flat_V, flat_B);
  }
}

template <typename Derived>
typename std::enable_if<
    detail::is_eigen_matrix_vector_of<Derived, symbolic::Formula>::value,
    Binding<Constraint>>::type
ParseConstraint(
    const Eigen::MatrixBase<Derived>& e) {
  throw std::runtime_error("Not implemented");
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
