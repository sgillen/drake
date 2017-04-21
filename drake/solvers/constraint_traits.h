#pragma once

/**
 * @brief Contanis Constraint traits for being incorporated into a container
 * such as MathematicalProgram
 */

#include <vector>

#include <Eigen/Core>

#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {

// TODO(eric.cousineau): Consider using namespace detail

/**
 * Constraint traits for how a given type is to be dispatched into a
 * container such as MathematicalProgram.
 * This enables optional down-casting in the return-type of
 * CreateConstraint(...) functionality, such as the PolynomialConstraint case,
 * where it may be simplified at run-time to a LinearConstraint or a
 * LinearEqualityConstraint.
 * @tparam C Constraint type
 */
template<typename C>
struct constraint_traits {
  /**
   * The type that will ultimately be used to register the constraint in the
   * given container
   */
  using final_type = C;
};

// Specialize for PolynomialConstraint
template<>
struct constraint_traits<PolynomialConstraint>
  : public constraint_traits<Constraint>
{ };

// Ease of use

template<typename C>
using constraint_final_type = constraint_traits<C>::final_type;

template<typename C>
using constraint_final_binding = Binding<constraint_final_type<C>>;

}  // namespace solvers
}  // namespace drake
