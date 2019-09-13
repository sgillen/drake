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

PYBIND11_MODULE(solver_interface, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;

  {
    using Class = SolverInterface;
    constexpr auto& cls_doc = pydrake_doc.drake.solvers.SolverInterface;
    py::class_<Class, PySolverInterface> cls(m, "SolverInterface");
    cls  // BR
         // Adding a constructor permits implementing this interface in Python.
        .def(py::init([]() { return std::make_unique<PySolverInterface>(); }),
            cls_doc.ctor.doc);
    // The following bindings are present to allow Python to call C++
    // implementations of this interface. Python implementations of the
    // interface will call the trampoline implementation methods above.
    cls  // BR
        .def("available", &Class::available, cls_doc.available.doc)
        .def("Solve",
            [](const Class& self, const solvers::MathematicalProgram& prog,
                const optional<Eigen::VectorXd>& initial_guess,
                const optional<solvers::SolverOptions>& solver_options) {
              solvers::MathematicalProgramResult result;
              self.Solve(prog, initial_guess, solver_options, &result);
              return result;
            },
            py::arg("prog"), py::arg("initial_guess"),
            py::arg("solver_options"), cls_doc.Solve.doc)
        .def("solver_id", &Class::solver_id, cls_doc.solver_id.doc)
        .def("AreProgramAttributesSatisfied",
            [](const Class& self, const solvers::MathematicalProgram& prog) {
              return self.AreProgramAttributesSatisfied(prog);
            },
            py::arg("prog"), cls_doc.AreProgramAttributesSatisfied.doc);
  }
}
}  // namespace pydrake
}  // namespace drake
