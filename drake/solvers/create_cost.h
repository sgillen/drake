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

} // namespace solvers
} // namespace drake
