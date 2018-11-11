/// @file
/// Tests NumPy user dtypes, using `pybind11`s C++ testing infrastructure.

#include <memory>
#include <string>

#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/bindings/pybind11_ext/numpy_dtypes_user.h"

using std::string;
using std::to_string;
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

// Tests for conversions.
class LengthValue {
 public:
  LengthValue(int value) : value_(value) {}
  int value() const { return value_; }
 private:
  int value_{};
};

class StrValue {
 public:
  explicit StrValue(const string& value) : value_(new string(value)) {}
  const string& value() const { return *value_; }
 private:
  std::shared_ptr<string> value_{};
};

#define OP_BINARY(op) \
    Symbol operator op(const Symbol& rhs) const { \
      Symbol value(*this); value.inplace_binary(#op, rhs); return value; \
    }
#define OP_BINARY_WITH_INPLACE(op, iop) \
    Symbol& operator iop(const Symbol& rhs) { \
      return inplace_binary(#op, rhs); \
    } \
    Symbol operator op(const Symbol& rhs) const { \
      Symbol value(*this); value iop rhs; return value; \
    }

// Tests operator overloads.
class Symbol {
 public:
  Symbol() : Symbol("") {}
  Symbol(const Symbol& other) : Symbol(other.str()) {}
  // Implicit conversion.
  Symbol(string str) : str_(new string(str)) {}
  Symbol(const StrValue& other) : Symbol(other.value()) {}
  Symbol(double value) : Symbol(fmt::format("float({})", value)) {}

  string str() const { return *str_; }

  // To be explicit.
  operator int() const { return str_->size(); }
  // To be implicit.
  operator LengthValue() const { return str_->size(); }

  template <typename... Args>
  static Symbol format(string pattern, const Args&... args) {
    return Symbol(fmt::format(pattern, args...));
  }

  // Operators.
  OP_BINARY_WITH_INPLACE(+, +=)
  OP_BINARY_WITH_INPLACE(-, -=)
  OP_BINARY_WITH_INPLACE(*, *=)
  OP_BINARY_WITH_INPLACE(/, /=)
  OP_BINARY_WITH_INPLACE(&, &=)
  OP_BINARY_WITH_INPLACE(|, |=)
  OP_BINARY(==)
  OP_BINARY(!=)
  OP_BINARY(<)
  OP_BINARY(<=)
  OP_BINARY(>)
  OP_BINARY(>=)
  OP_BINARY(&&)
  OP_BINARY(||)

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

#undef OP_BINARY
#undef OP_BINARY_WITH_INPLACE

std::ostream& operator<<(std::ostream& os, const Symbol& s) {
  return os << s.str();
}

namespace func {

Symbol abs(const Symbol& s) { return Symbol::format("abs({})", s); }
Symbol cos(const Symbol& s) { return Symbol::format("cos({})", s); }
Symbol sin(const Symbol& s) { return Symbol::format("sin({})", s); }
Symbol pow(const Symbol& a, const Symbol& b) {
  return Symbol::format("({}) ^ ({})", a, b);
}

}  // namespace func

template <typename Class, typename Return>
auto MakeRepr(const string& name, Return (Class::*method)() const) {
  return [name, method](Class* self) {
    return py::str("<{} '{}'>").format(name, (self->*method)());
  };
}

template <typename Class, typename Return>
auto MakeStr(Return (Class::*method)() const) {
  return [method](Class* self) {
    return py::str("{}").format((self->*method)());
  };
}

}  // namespace

PYBIND11_NUMPY_DTYPE_USER(LengthValue);
PYBIND11_NUMPY_DTYPE_USER(StrValue);
PYBIND11_NUMPY_DTYPE_USER(Symbol);

namespace {

PYBIND11_MODULE(numpy_dtypes_user, m) {
  py::dtype_user<LengthValue> length(m, "LengthValue");
  py::dtype_user<StrValue> str(m, "StrValue");
  py::dtype_user<Symbol> sym(m, "Symbol");

  length  // BR
      .def(py::init<int>())
      .def("value", &LengthValue::value)
      .def("__repr__", MakeRepr("LengthValue", &LengthValue::value))
      .def("__str__", MakeStr(&LengthValue::value));

  str  // BR
      .def(py::init<string>())
      .def("value", &StrValue::value)
      .def("__repr__", MakeRepr("StrValue", &StrValue::value))
      .def("__str__", MakeStr(&StrValue::value));

  sym  // BR
      // Nominal definitions.
      .def(py::init())
      .def(py::init<const string&>())
      .def(py::init<double>())
      .def(py::init<const Symbol&>())
      .def("__repr__", MakeRepr("Symbol", &Symbol::str))
      .def("__str__", MakeStr(&Symbol::str))
      .def("str", &Symbol::str)
      // - Test referencing.
      .def("self_reference",
           [](const Symbol& self) { return &self; },
           py::return_value_policy::reference)
      // Casting.
      // - From
      .def_loop(py::dtype_method::explicit_conversion<double, Symbol>())
      .def_loop(py::dtype_method::explicit_conversion<StrValue, Symbol>())
      // - To
      .def_loop(py::dtype_method::explicit_conversion<Symbol, int>())
      .def_loop(py::dtype_method::implicit_conversion<Symbol, LengthValue>())
      // Operators.
      // - Math.
      .def_loop(py::self + py::self)
      .def(py::self += py::self)
      .def_loop(py::self - py::self)
      .def(py::self -= py::self)
      .def_loop(py::self * py::self)
      .def(py::self *= py::self)
      .def_loop(py::self / py::self)
      .def(py::self /= py::self)
      // - Bitwise.
      .def_loop(py::self & py::self)
      .def(py::self &= py::self)
      .def_loop(py::self | py::self)
      .def(py::self |= py::self)
      // - Logical.
      .def_loop(py::self == py::self)
      .def_loop(py::self != py::self)
      .def_loop(py::self < py::self)
      .def_loop(py::self <= py::self)
      .def_loop(py::self > py::self)
      .def_loop(py::self >= py::self)
      // .def_loop(py::self && py::self)
      // .def_loop(py::self || py::self)
      // Explicit UFunc.
      .def_loop("__pow__", &func::pow)
      .def_loop("abs", &func::abs)
      .def_loop("cos", &func::cos)
      .def_loop("sin", &func::sin);
}

}  // namespace
