#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

// @file
// Tests both `cpp_types.py` and `cpp_types_pybind.h`.

#include <string>

#include <gtest/gtest.h>

#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
// #include "drake/bindings/pydrake/autodiff_types_pybind.h"
// #include "drake/bindings/pydrake/symbolic_types_pybind.h"

using std::string;

namespace drake {
namespace pydrake {

class CppTypesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    m_ = py::module("__main__");
    globals_ = py::globals();
    py::exec(R"""(
import numpy as np
import ctypes

from pydrake.util.cpptypes import get_type_canonical, get_type_name

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
)""", globals_);
  }

  py::object eval(const string& expr) {
    return py::eval(expr.c_str(), globals_);
  }

  bool Compare(const string& lhs, const string& rhs) {
    return eval(lhs).is(eval(rhs));
  }

  template <typename T>
  bool Compare(const string& rhs) {
    return GetPyType<T>().is(eval(rhs));
  }

  py::scoped_interpreter guard_;
  py::module m_;
  py::dict globals_;
};

TEST_F(CppTypesTest, InPython) {
  // Check registered types.
  ASSERT_TRUE(Compare("get_type_canonical(bool)", "bool"));
  ASSERT_TRUE(Compare("get_type_canonical(str)", "str"));
  ASSERT_TRUE(Compare("get_type_canonical(int)", "int"));
  ASSERT_TRUE(Compare("get_type_canonical(ctypes.c_int32)", "int"));
  ASSERT_TRUE(Compare("get_type_canonical(float)", "float"));
  ASSERT_TRUE(Compare("get_type_canonical(np.double)", "float"));
  ASSERT_TRUE(Compare("get_type_canonical(object)", "object"));
  ASSERT_TRUE(Compare("get_type_canonical(AutoDiffXd)", "AutoDiffXd"));
  ASSERT_TRUE(Compare("get_type_canonical(Expression)", "Expression"));
}

TEST_F(CppTypesTest, InCpp) {
  ASSERT_TRUE(Compare<bool>("bool"));
  ASSERT_TRUE(Compare<std::string>("str"));
  ASSERT_TRUE(Compare<double>("float"));
  ASSERT_TRUE(Compare<float>("np.float32"));
  ASSERT_TRUE(Compare<int>("int"));
  ASSERT_TRUE(Compare<py::object>("object"));
  ASSERT_TRUE(Compare<AutoDiffXd>("AutoDiffXd"));
  ASSERT_TRUE(Compare<symbolic::Expression>("Expression"));
}

}  // namespace pydrake
}  // namespace drake
