#pragma once

#include <memory>
#include <type_traits>

#include "drake/common/symbolic_expression.h"
#include "drake/solvers/function.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/binding.h"

namespace drake {
namespace solvers {

//
//
//

Binding<LinearCost> CreateAddLinearCost(
    const Expression& e) {
  auto p = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& var = p.first;
  const auto& map_var_to_index = p.second;
  Eigen::RowVectorXd c(var.size());
  double constant_term;
  DecomposeLinearExpression(e, map_var_to_index, c, &constant_term);
  // The constant term is ignored now.
  // TODO(hongkai.dai): support adding constant term to the cost.
  return AddLinearCost(c, var);
}


//
// --- QuadraticCost ---
//

/**
 * Creates a cost of the form x'*Q*x
 */
template<class... Args>
std::shared_ptr<QuadraticCost> CreateQuadraticCost(Args&&... args) {
  return std::make_shared<QuadraticCost>(std::forward<Args>(args)...);
}

/**
 * Creates a cost term of the form | Ax - b |^2.
 */
std::shared_ptr<QuadraticCost> CreateL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/**
 * Creates a cost term of the form (x-x_desired)'*Q*(x-x_desired).
 */
std::shared_ptr<QuadraticCost> CreateQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired);

/**
 * Creates a quadratic cost term
 */
Binding<QuadraticCost> CreateQuadraticCost(const symbolic::Expression& e);


//
// --- FunctionCost ---
//

/**
 * Convert an input of type @tparam F to a FunctionCost object.
 * @tparam F This class should have functions numInputs(), numOutputs and
 * eval(x, y). Check drake::solvrs::detail::FunctionTraits for more details.
 */
template<typename F>
std::shared_ptr<Cost> CreateFunctionCost(F&& f) {
  return std::make_shared<
      FunctionCost<typename std::remove_reference<F>::type>>(
      std::forward<F>(f));
}

// From: drake-distro (git sha: 24452c1)
// /drake/solvers/mathematical_program.h:739
// libstdc++ 4.9 evaluates
// `std::is_convertible<std::unique_ptr<Unrelated>,
// std::shared_ptr<Constraint>>::value`
// incorrectly as `true` so our enable_if overload is not used.
// Provide an explicit alternative for this case.
template<typename A, typename B>
struct is_convertible_workaround
  : std::is_convertible<A, B>
{ };
template<typename A, typename B>
struct is_convertible_workaround<std::unique_ptr<A>, std::shared_ptr<B>>
    : std::is_convertible<A*, B*>
{ };

/**
 * Template condition to check if F is a candidate to be used to construct a
 * FunctionCost object for generic costs.
 * @note Constraint is used to ensure that we do not preclude cost objects
 * that lost their CostShim type somewhere in the process.
 */
template<typename F>
struct is_cost_functor_candidate : std::integral_constant<bool,
    (!is_convertible_workaround<F, Constraint>::value) &&
    (!is_convertible_workaround<F, std::shared_ptr<Constraint>>::value) &&
    (!is_convertible_workaround<F, std::unique_ptr<Constraint>>::value) &&
    (!is_convertible_workaround<F, Binding<Constraint>>::value) &&
    (!is_convertible_workaround<F, symbolic::Expression>::value)>
{ };

// TODO(eric.cousineau): For is_cost_functor_candiate, consider
// changing implementation to simply check if F is callable (after removing
// pointers, decaying, etc.)
// @ref http://stackoverflow.com/a/5117641/7829525

/**
 * Add costs to the optimization program on decision variables as dictated
 * by the Binding constructor.
 * @tparam F it should define functions numInputs, numOutputs and eval. Check
 * drake::solvers::detail::FunctionTraits for more detail.
 */
template <typename F>
typename std::enable_if<
    is_cost_functor_candidate<F>::value,
    std::shared_ptr<Cost>>::type
CreateCost(F&& f) {
  return CreateFunctionCost(std::forward<F>(f));
}

} // namespace solvers
} // namespace drake
