#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_function.h"

using std::sin;
using std::cos;

namespace drake {
namespace pydrake {

/**
 * Force Eigen to evaluate an autodiff expression. We need this function
 * because, for example, adding two Eigen::AutoDiffXd values produces an
 * Eigen::AutoDiffScalar<Eigen::CWiseBinaryOp> which cannot be returned to
 * python. This just forces an evaluation and conversion to AutoDiffXd which
 * would normally happen automatically in C++.
 */
template <typename Derived>
AutoDiffXd eval(const Eigen::AutoDiffScalar<Derived>& x) {
  return AutoDiffXd(x.value(), x.derivatives());
}

// N.B. This wrap policy is asymmetric for return values only, and should not
// be used for callbacks.
template <typename T>
struct wrap_eval_policy : public wrap_arg_default<T> {
  static auto wrap(T ret) { return eval(ret); }
};

PYBIND11_MODULE(_autodiffutils_py, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  auto wrap_eval = [](auto func) {
    return WrapFunction<wrap_eval_policy>(func);
  };

  py::class_<AutoDiffXd>(m, "AutoDiffXd")
    .def("__init__",
         [](AutoDiffXd& self,
            double value,
            const Eigen::VectorXd& derivatives) {
           new (&self) AutoDiffXd(value, derivatives);
         })
    .def("value", [](const AutoDiffXd& self) {
      return self.value();
    })
    .def("derivatives", [](const AutoDiffXd& self) {
      return self.derivatives();
    })
    .def("sin", [](const AutoDiffXd& self) { return eval(sin(self)); })
    .def("cos", [](const AutoDiffXd& self) { return eval(cos(self)); })
    .def(py::self + py::self, wrap_eval)
    .def(py::self + double(), wrap_eval)
    .def(double() + py::self, wrap_eval)
    .def(py::self - py::self, wrap_eval)
    .def(py::self - double(), wrap_eval)
    .def(double() - py::self, wrap_eval)
    .def(py::self * py::self, wrap_eval)
    .def(py::self * double(), wrap_eval)
    .def(double() * py::self, wrap_eval)
    .def(py::self / py::self, wrap_eval)
    .def(py::self / double(), wrap_eval)
    .def(double() / py::self, wrap_eval)
    .def("__pow__",
         WrapFunction<wrap_eval_policy>(
	    overload_cast<const AutoDiffXd&, int>(&pow)),
         py::is_operator());
}

}  // namespace pydrake
}  // namespace drake
