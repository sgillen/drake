#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

using std::sin;
using std::cos;

namespace drake {
namespace pydrake {
namespace {

/*
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

}  // namespace

PYBIND11_MODULE(_autodiffutils_py, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  py::class_<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff
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
    });
  WrapDef<wrap_eval_policy>(&autodiff)
    .def("sin", [](const AutoDiffXd& self) { return sin(self); });
    // .def("cos", [](const AutoDiffXd& self) { return cos(self); })
    // .def(py::self + py::self)
    // .def(py::self + double())
    // .def(double() + py::self)
    // .def(py::self - py::self)
    // .def(py::self - double())
    // .def(double() - py::self)
    // .def(py::self * py::self)
    // .def(py::self * double())
    // .def(double() * py::self)
    // .def(py::self / py::self)
    // .def(py::self / double())
    // .def(double() / py::self)
    // .def("__pow__",
    //      [](const AutoDiffXd& base, int exponent) {
    //        return pow(base, exponent);
    //      }, py::is_operator());
}

}  // namespace pydrake
}  // namespace drake
