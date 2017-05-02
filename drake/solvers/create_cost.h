#pragma once

#include <memory>
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

/*
 * Creates a linear cost term of the form c'*x.
 * @param e A linear symbolic expression.
 * @pre{e is a linear expression c'*x, where each entry of x is a decision
 * variable in the mathematical program}
 * @return The newly created linear constraint, together with the bound
 * variables.
 */
Binding<LinearCost> ParseLinearCost(const symbolic::Expression& e);

/*
 * Creates a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c.
 * Notice that in the optimization program, the constant term `c` in the cost
 * is ignored.
 * @param e A quadratic symbolic expression. Throws a runtime error if the
 * expression is not quadratic.
 * @return The newly created cost together with the bound variables.
 */
Binding<QuadraticCost> ParseQuadraticCost(const symbolic::Expression& e);

/*
 * Creates a cost term in the polynomial form.
 * @param e A symbolic expression in the polynomial form.
 * @return The newly added cost and the bound variables.
 */
Binding<PolynomialCost> ParsePolynomialCost(const symbolic::Expression& e);

/*
 * Creates a cost by dynamically determining the functional form of `e`,
 * @param e A symbolic expression
 * @return The newly created cost and the bound variables.
 */
Binding<Cost> ParseCost(const symbolic::Expression& e);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
