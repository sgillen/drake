#pragma once

#include <memory>
#include <type_traits>

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

/**
 * Template condition to check if F is a candidate to be used to construct a
 * FunctionCost object for generic costs.
 * @note Constraint is used to ensure that we do not preclude cost objects
 * that lost their CostShim type somewhere in the process.
 */
template<typename F>
struct is_cost_functor_candidate : std::integral_constant<bool,
    (!std::is_convertible<F, std::shared_ptr<Constraint>>::value) &&
    (!std::is_convertible<F, Binding<Constraint>>::value)>
{ };

// TODO(eric.cousineau): Consider specializing is_cost_functor_candidate if we
// run into this again:
// From: drake-distro (git sha: 24452c1)
// /drake/solvers/mathematical_program.h:739
// libstdc++ 4.9 evaluates
// `std::is_convertible<std::unique_ptr<Unrelated>,
// std::shared_ptr<Constraint>>::value`
// incorrectly as `true` so our enable_if overload is not used.
// Provide an explicit alternative for this case.

} // namespace solvers
} // namespace drake
