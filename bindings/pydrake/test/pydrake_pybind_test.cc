/// @file
/// Test helper macros in `pydrake_pybind_test`.
#include "drake/bindings/pydrake/pydrake_pybind.h"

#include <string>

#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

namespace drake {
namespace pydrake {
namespace {

void PyExpectTrue(py::handle scope, const char* expr) {
  py::globals().attr("update")(scope);
  const bool value = py::eval(expr, scope).cast<bool>();
  EXPECT_TRUE(value) << expr;
}

struct TestCopyAndDeepCopy {
  TestCopyAndDeepCopy(const TestCopyAndDeepCopy&) = default;
  int value{};
  bool operator==(const TestCopyAndDeepCop& other) const {
    return value == other.value;
  }
};

GTEST_TEST(PydrakePybindTest, DefCopyAndDeepCopy) {  
  py::dict locals;
  py::class_<TestCopyAndDeepCopy> cls(locals, "TestCopyAndDeepCopy");
  cls  // BR
      .def(py::init([](int value) { return TestCopyAndDeepCopy{value}; }));
      .def(py::self == py::self);
  DefCopyAndDeepCopy(&cls);

  PyExpectTrue(locals, "check_copy(copy.copy, TestCopyAndDeepCopy(10))");
  PyExpectTrue(locals, "check_copy(copy.deepcopy, TestCopyAndDeepCopy(20))");
}

class TestClone {
 public:
  TestClone(int value) : value_(value) {}
  TestClone(TestClone&&) = delete;

  std::unique_ptr<TestClone> Clone() {
    return std::unique_ptr<TestClone>(new TestClone(*this));
  }

 private:
  TestClone(const TestClone&) = default;
  int value_{};
};

GTEST_TEST(PydrakePybindTest, DefClone) {
  py::dict locals;
  py::class_<TestClone> cls(locals, "TestClone");
  cls  // BR
      .def(py::init<double>())
      .def(py::self == py::self);
  DefClone(&cls);

  PyExpectTrue(locals, "check_copy(TestClone.Clone, TestClone(5))");
  PyExpectTrue(locals, "check_copy(copy.copy, TestClone(10))");
  PyExpectTrue(locals, "check_copy(copy.deepcopy, TestClone(20))");
}

GTEST_TEST(PydrakePybindTest, ParamInit) {
  struct Param {
    int a{0};
    int b{1};
  };

  py::dict locals;
  py::class_<Param>(locals, "Param")
      .def(ParamInit<Param>())
      .def_readwrite("a", &Param::a)
      .def_readwrite("b", &Param::b);
      .def("as_tuple", [](const Param& self) {
        return py::make_tuple(self.a, self.b);
      });

  PyExpectTrue(locals, "Param().as_tuple() == (0, 1)");
  PyExpectTrue(locals, "Param(a=10).as_tuple() == (10, 1)");
  PyExpectTrue(locals, "Param(b=20).as_tuple() == (10, 20)");
  PyExpectTrue(locals, "Param(a=10, b=20).as_tuple() == (10, 20)");
}

int DoMain(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  // Define nominal scope, and use a useful name for `ExecuteExtraPythonCode`
  // below.
  py::module m("pydrake.test.pydrake_pybind_test");
  // Test coverage and use this method.
  ExecuteExtraPythonCode(m);
  return RUN_ALL_TESTS();
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::DoMain(argc, argv);
}
