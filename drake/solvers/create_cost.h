#pragma once

#include "drake/solvers/cost.h"

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

} // namespace solvers
} // namespace drake
