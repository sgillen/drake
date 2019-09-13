#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace pydrake {

namespace {
// pybind11 trampoline class to permit overriding virtual functions in Python.
class PySolverInterface : public py::wrapper<solvers::SolverInterface> {
 public:
  using Base = py::wrapper<solvers::SolverInterface>;

  PySolverInterface() : Base() {}

  // The following methods are for the pybind11 trampoline class to permit C++
  // to call the correct Python override. This code path is only activated for
  // Python implementations of the class (whose inheritance will pass through
  // `PySolverInterface`). C++ implementations will use the bindings on the
  // interface below.

  bool available() const override {
    PYBIND11_OVERLOAD_PURE(bool, solvers::SolverInterface, available);
  }

  void Solve(const solvers::MathematicalProgram& prog,
      const optional<Eigen::VectorXd>& initial_guess,
      const optional<solvers::SolverOptions>& solver_options,
      solvers::MathematicalProgramResult* result) const override {
    PYBIND11_OVERLOAD_PURE(void, solvers::SolverInterface, Solve, prog,
        initial_guess, solver_options, result);
  }

  solvers::SolverId solver_id() const override {
    PYBIND11_OVERLOAD_PURE(
        solvers::SolverId, solvers::SolverInterface, solver_id);
  }

  bool AreProgramAttributesSatisfied(
      const solvers::MathematicalProgram& prog) const override {
    PYBIND11_OVERLOAD_PURE(
        bool, solvers::SolverInterface, AreProgramAttributesSatisfied, prog);
  }
};
}  // namespace
}  // namespace pydrake
}  // namespace drake
