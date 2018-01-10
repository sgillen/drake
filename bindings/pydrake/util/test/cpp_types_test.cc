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

from pydrake.util.cpp_types import get_type_canonical, get_type_name

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
)""", globals_);
  }

  template <typename T>
  bool Compare(const string& rhs) {
    return GetPyType<T>().is(py::eval(rhs.c_str(), globals_));
  }

  py::module m_;
  py::dict globals_;
};

TEST_F(CppTypesTest, InPython) {
  // Check pure-Python behavior.
  py::dict locals;
  py::exec(R"""(
pairs = (
    # One-to-one.
    (bool, bool),
    (str, str),
    (int, int),
    (float, float),
    (object, object),
    (AutoDiffXd, AutoDiffXd),
    (Expression, Expression),
    # Aliases:
    (float, np.double),
    (int, ctypes.c_int32),
)

for canonical, alias in pairs:
    pair_str = "{}, {}".format(alias, canonical)
    assert get_type_canonical(alias) is canonical, "Bad pair: " + pair_str
)""", globals_, locals);
  // Sanity check to ensure we've executed our Python code.
  ASSERT_TRUE(!locals["canonical"].is_none());
}

TEST_F(CppTypesTest, InCpp) {
  // Check C++ behavior.
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

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` mutliple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
