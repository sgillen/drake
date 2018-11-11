/// @file
/// Tests NumPy user dtypes, using `pybind11`s C++ testing infrastructure.

#include <memory>
#include <string>

#include <pybind11/operators.h>
#include <pybind11/stl.h>

// NOLINTNEXTLINE[build/include]
#include "constructor_stats.h"
// NOLINTNEXTLINE[build/include]
#include "pybind11_tests.h"

#include "drake/bindings/pybind11_ext/numpy_dtypes_user.h"

using std::string;
using std::unique_ptr;

namespace py = pybind11;

namespace {

/*
Goals:

 * Show API
 * Show operator overloads
 * Exercise memory bugs (allocation, etc.)
 * Show implicit / explicit conversions.

The simplest mechanism is to do super simple symbolics.
*/

#define OP_BINARY(op) \
    Symbol operator op(const Symbol& rhs) const {
      Symbol value(*this); value.inplace_binary(op, rhs); return value; \
    }
#define OP_BINARY_WITH_INPLACE(op, iop) \
    Symbol& operator iop(const Symbol& rhs) { \
      return inplace_binary(#op, rhs); \
    } \
    Symbol operator op(const Symbol& rhs) const { \
      Symbol value(*this); value iop rhs; return value; \
    }

// Tests for conversions.
class LengthValue {
 public:
  LengthValue(int value) : value_(value) {}
  int value() const { return value_; }
 private:
  int value_{};
}

// Tests operator overloads.
class Symbol {
 public:
  Symbol() : Symbol("") {}
  Symbol(const Symbol& other) : Symbol(other.str()) {}
  // Implicit conversion.
  Symbol(string str) : str_(new string(str)) {}
  Symbol(double value) : Symbol("float(" + to_string(value) + ")") {}

  const string& str() const { return *str_; }

  operator LengthValue() const { return str_->size(); }

  // Operators.
  OP_BINARY_WITH_INPLACE(+, +=)
  OP_BINARY_WITH_INPLACE(-, -=)
  OP_BINARY_WITH_INPLACE(*, *=)
  OP_BINARY_WITH_INPLACE(/, /=)
  OP_BINARY(==)
  OP_BINARY(!=)
  OP_BINARY(<)
  OP_BINARY(<=)
  OP_BINARY(>)
  OP_BINARY(>=)

 private:
  Symbol& inplace_binary(const char* op, const Symbol& rhs) {
    *str_ = fmt::format("({}) {} ({})", str(), op, rhs.str());
    return *this;
  }

  // Data member to ensure that we do not get segfaults when carrying around
  // `shared_ptr`s, and to ensure that the data is memcpy-moveable.
  // N.B. This is not used for Copy-on-Write optimizations.
  std::shared_ptr<string> str_;
};

}  // namespace

TEST_SUBMODULE(numpy_dtypes_user, m) {
  py::dtype_user<LengthValue> length(m, "LengthValue");
  length  // BR
      .def(py::init<int>());

  py::dtype_user<Symbol> sym(m, "Symbol");
  sym  // BR
      .def(py::init())
      .def(py::init<const string&>())
      .def(py::init<const Symbol&>())
      .def("__repr__",
           [](const Symbol& self) {
             return py::str("<Symbol '{}'>".format(self.str()));
           })
      .def("__str__", [](const Symbol& self) { return self.str(); })
      .def("str", &Symbol::str)
      // Test referecing.
      .def("self_reference",
           [](const Symbol& self) { return &self; },
           py::return_value_policy::reference)
      // Casting.
      .def_loop(py::dtype
      .def_loop(py::dtype_method::explicit_conversion(
          &Symbol::operator LengthValue))
          [](const Symbol


}
