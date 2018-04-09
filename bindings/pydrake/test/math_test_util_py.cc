#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using Eigen::VectorXd;

namespace drake {
namespace pydrake {

namespace {

class AutoDiffContainer {
 public:
  AutoDiffContainer() {
    value_.resize(1, 2);
    value_ <<
        AutoDiffXd(10, (VectorXd(2) << 1, 0).finished()),
        AutoDiffXd(100, (VectorXd(2) << 0, 1).finished());
  }
  MatrixX<AutoDiffXd>& value() { return value_; }
 private:
  MatrixX<AutoDiffXd> value_;
};

}  // namespace

PYBIND11_MODULE(math_test_util, m) {
  // Add utilities to test reference semantics for AutoDiff.
  py::class_<AutoDiffContainer>(m, "AutoDiffContainer")
      .def(py::init())
      .def("value", &AutoDiffContainer::value,
           py_reference_internal);

  m.def("autodiff_increment", [](Eigen::Ref<MatrixX<AutoDiffXd>> value) {
      value.array() += 1;
  });
}

}  // namespace pydrake
}  // namespace drake
