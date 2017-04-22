ConstraintImpl#pragma once

#include "drake/solvers/function.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {

// TODO(eric.cousineau): Remove stopgap, and actually have Constraint and
// Cost be different classes. Consider using some common evaluation base.

/**
 * Stopgap class to provide functionality as constraint, but allow templates to
 * detect a difference from results from CreateConstraint and CreateCost.
 * @tparam C Constraint type to inherit from.
 */
template<typename C>
class CostShim : public C { };

class Cost : public CostShim<Constraint> { };

class LinearCost : public CostShim<LinearConstraint> { };

class QuadraticCost : public CostShim<QuadraticConstraint> { };

/**
 * A cost that may be specified using a callable object
 * @tparam F The function / functor's type
 */
template <typename F>
class FunctionCost : public Cost {
  F const f_;

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionCost)

  // Construct by copying from an lvalue.
  template <typename... Args>
  FunctionCost(const F& f, Args&&... args)
      : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(f) {}

  // Construct by moving from an rvalue.
  template <typename... Args>
  FunctionCost(F&& f, Args&&... args)
      : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(std::forward<F>(f)) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }
};

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

};  // namespace solvers
}  // namespace drake
