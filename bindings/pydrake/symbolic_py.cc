#include <map>
#include <string>

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

namespace drake {
namespace pydrake {

using std::map;
using std::string;

int int_cmp(int a, int b) {
  if (a == b)
    return 0;
  else if (a < b)
    return -1;
  else
    return 1;
}

template <typename T>
int hash_cmp(const T& a, const T& b) {
  return int_cmp(std::hash<T>{}(a), std::hash<T>{}(b));
}

// TODO(eric.cousineau): Use py::self for operator overloads?
PYBIND11_MODULE(_symbolic_py, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::symbolic;

  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  // Predeclare all custom dtypes.
  py::dtype_user<Variable> var(m, "Variable");
  py::dtype_user<Expression> expr(m, "Expression");
  py::dtype_user<Formula> formula(m, "Formula");

  var
      .def(py::init<const string&>())
      .def("get_id", &Variable::get_id)
      .def("__str__", &Variable::to_string)
      .def("__repr__",
           [](const Variable& self) {
             return fmt::format("Variable('{}')", self.to_string());
           })
      .def("__hash__",
           [](const Variable& self) { return std::hash<Variable>{}(self); })
      .def("__copy__",
           [](const Variable& self) -> Variable {
             return self;
           })
      // Addition.
      .def_loop(py::self + py::self)
      .def_loop(py::self + double())
      .def_loop(double() + py::self)
      // Subtraction.
      .def_loop(py::self - py::self)
      .def_loop(py::self - double())
      .def_loop(double() - py::self)
      // Multiplication.
      .def_loop(py::self * py::self)
      .def_loop(py::self * double())
      .def_loop(double() * py::self)
      // Division.
      .def_loop(py::self / py::self)
      .def_loop(py::self / double())
      .def_loop(double() / py::self)
      // Pow.
      .def_loop("__pow__",
           [](const Variable& self, double other) { return pow(self, other); })
      .def_loop("__pow__",
           [](const Variable& self, const Variable& other) {
             return pow(self, other);
           })
      .def_loop("__pow__",
           [](const Variable& self, const Expression& other) {
             return pow(self, other);
           })
      // Unary Plus.
      .def(+py::self)  // Not present in NumPy?
      // Unary Minus.
      .def_loop(-py::self)
      // LT(<).
      // Note that while pybind reflects `double < Variable`, NumPy UFunc loops
      // require both orders to be explicitly specified.
      .def_loop(py::self < Expression())
      .def_loop(py::self < py::self)
      .def_loop(py::self < double())
      .def_loop(double() < py::self, false)
      // LE(<=).
      .def_loop(py::self <= Expression())
      .def_loop(py::self <= py::self)
      .def_loop(py::self <= double())
      .def_loop(double() <= py::self, false)
      // GT(>).
      .def_loop(py::self > Expression())
      .def_loop(py::self > py::self)
      .def_loop(py::self > double())
      .def_loop(double() > py::self, false)
      // GE(>=).
      .def_loop(py::self >= Expression())
      .def_loop(py::self >= py::self)
      .def_loop(py::self >= double())
      .def_loop(double() >= py::self, false)
      // EQ(==).
      .def_loop(py::self == Expression())
      .def_loop(py::self == py::self)
      .def_loop(py::self == double())
      // NE(!=).
      .def_loop(py::self != Expression())
      .def_loop(py::self != py::self)
      .def_loop(py::self != double());

  py::class_<Variables>(m, "Variables")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&>())
      .def("size", &Variables::size)
      .def("empty", &Variables::empty)
      .def("__str__", &Variables::to_string)
      .def("__repr__",
           [](const Variables& self) {
             return fmt::format("<Variables \"{}\">", self);
           })
      .def("to_string", &Variables::to_string)
      .def("__hash__",
           [](const Variables& self) { return std::hash<Variables>{}(self); })
      .def("insert",
           [](Variables& self, const Variable& var) { self.insert(var); })
      .def("insert",
           [](Variables& self, const Variables& vars) { self.insert(vars); })
      .def("erase",
           [](Variables& self, const Variable& var) { return self.erase(var); })
      .def("erase", [](Variables& self,
                       const Variables& vars) { return self.erase(vars); })
      .def("include", &Variables::include)
      .def("IsSubsetOf", &Variables::IsSubsetOf)
      .def("IsSupersetOf", &Variables::IsSupersetOf)
      .def("IsStrictSubsetOf", &Variables::IsStrictSubsetOf)
      .def("IsStrictSupersetOf", &Variables::IsStrictSupersetOf)
      .def(py::self == py::self)
      .def(py::self < py::self)
      .def(py::self + py::self)
      .def(py::self + Variable())
      .def(Variable() + py::self)
      .def(py::self - py::self)
      .def(py::self - Variable())
      .def("__cmp__", [](const Variable& a, const Variable& b) {
        // For dict, `PyObject_RichCompare`, to avoid the need for `nonzero`.
        return hash_cmp(a, b);
      });

  m.def("intersect", [](const Variables& vars1, const Variables& vars2) {
    return intersect(vars1, vars2);
  });

  expr
      .def(py::init<>())
      .def(py::init<double>())
      .def(py::init<const Variable&>())
      .def_loop_cast([](double in) -> Expression { return in; }, true)
      .def_loop_cast([](const Variable& in) -> Expression { return in; }, true)
      .def("__str__", &Expression::to_string)
      .def("__repr__",
           [](const Expression& self) {
             return fmt::format("<Expression \"{}\">", self.to_string());
           })
      .def("__copy__",
           [](const Expression& self) -> Expression {
             return self;
           })
      .def("to_string", &Expression::to_string)
      .def("Expand", &Expression::Expand)
      .def("Evaluate", [](const Expression& self) { return self.Evaluate(); })
      // Addition
      .def_loop(py::self + py::self)
      .def_loop(py::self + Variable())
      .def_loop(py::self + double())
      .def_loop(Variable() + py::self)
      .def_loop(double() + py::self)
      .def(py::self += py::self)
      .def(py::self += Variable())
      .def(py::self += double())
      // Subtraction.
      .def_loop(py::self - py::self)
      .def_loop(py::self - Variable())
      .def_loop(py::self - double())
      .def_loop(Variable() - py::self)
      .def_loop(double() - py::self)
      .def(py::self -= py::self)
      .def(py::self -= Variable())
      .def(py::self -= double())
      // Multiplication.
      .def_loop(py::self * py::self)
      .def_loop(py::self * Variable())
      .def_loop(py::self * double())
      .def_loop(Variable() * py::self)
      .def_loop(double() * py::self)
      .def(py::self *= py::self)
      .def(py::self *= Variable())
      .def(py::self *= double())
      // Division.
      .def_loop(py::self / py::self)
      .def_loop(py::self / Variable())
      .def_loop(py::self / double())
      .def_loop(Variable() / py::self)
      .def_loop(double() / py::self)
      .def(py::self /= py::self)
      .def(py::self /= Variable())
      .def(py::self /= double())
      // Unary Plus.
      .def(+py::self)  // Not present in NumPy?
      // Unary Minus.
      .def_loop(-py::self)
      // LT(<).
      // See notes for `Variable` about reversible operations.
      .def_loop(py::self < py::self)
      .def_loop(py::self < Variable())
      .def_loop(py::self < double())
      .def_loop(double() < py::self, false)
      // LE(<=).
      .def_loop(py::self <= py::self)
      .def_loop(py::self <= Variable())
      .def_loop(py::self <= double())
      .def_loop(double() <= py::self, false)
      // GT(>).
      .def_loop(py::self > py::self)
      .def_loop(py::self > Variable())
      .def_loop(py::self > double())
      .def_loop(double() > py::self, false)
      // GE(>=).
      .def_loop(py::self >= py::self)
      .def_loop(py::self >= Variable())
      .def_loop(py::self >= double())
      .def_loop(double() >= py::self, false)
      // EQ(==).
      .def_loop(py::self == py::self)
      .def_loop(py::self == Variable())
      .def_loop(py::self == double())
      // NE(!=)
      .def_loop(py::self != py::self)
      .def_loop(py::self != Variable())
      .def_loop(py::self != double())
      .def("Differentiate", &Expression::Differentiate)
      .def("Jacobian", &Expression::Jacobian);

  // TODO(eric.cousineau): Consider deprecating the aliases in `math`?
  auto math = py::module::import("pydrake.math");
  UfuncMirrorDef<decltype(expr)>(&expr, math)
      // TODO(eric.cousineau): Figure out how to consolidate with the below
      // methods.
      // Pow.
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const double other) { return pow(self, other); })
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const Variable& other) { return pow(self, other); })
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const Expression& other) { return pow(self, other); })
      .def_loop("log", &symbolic::log)
      .def_loop("__abs__", "abs", &symbolic::abs)
      .def_loop("exp", &symbolic::exp)
      .def_loop("sqrt", &symbolic::sqrt)
      // TODO(eric.cousineau): Move `__pow__` here.
      .def_loop("sin", &symbolic::sin)
      .def_loop("cos", &symbolic::cos)
      .def_loop("tan", &symbolic::tan)
      .def_loop("arcsin", "asin", &symbolic::asin)
      .def_loop("arccos", "acos", &symbolic::acos)
      .def_loop("arctan2", "atan2", &symbolic::atan2)
      .def_loop("sinh", &symbolic::sinh)
      .def_loop("cosh", &symbolic::cosh)
      .def_loop("tanh", &symbolic::tanh)
      .def_loop("fmin", "min", &symbolic::min)
      .def_loop("fmax", "max", &symbolic::max)
      .def_loop("ceil", &symbolic::ceil)
      .def_loop("floor", &symbolic::floor);

  // Import aliases.
  // TODO(eric.cousineau): Deprecate, then remove these in lieu of `np.{func}`
  py::exec(R"""(
from pydrake.math import (
    log,
    abs,
    exp,
    pow,
    sqrt,
    sin,
    cos,
    tan,
    asin,
    acos,
    atan2,
    sinh,
    cosh,
    tanh,
    min,
    max,
    ceil,
    floor
)
)""");
  m.def("atan", &symbolic::atan);

  m.def("if_then_else", &symbolic::if_then_else);

  m.def("Jacobian", [](const Eigen::Ref<const VectorX<Expression>>& f,
                       const Eigen::Ref<const VectorX<Variable>>& vars) {
    return Jacobian(f, vars);
  });

  formula
      .def("GetFreeVariables", &Formula::GetFreeVariables)
      .def("EqualTo", &Formula::EqualTo)
      .def("Substitute",
           [](const Formula& self, const Variable& var, const Expression& e) {
             return self.Substitute(var, e);
           })
      .def("Substitute",
           [](const Formula& self, const Variable& var1, const Variable& var2) {
             return self.Substitute(var1, var2);
           })
      .def("Substitute", [](const Formula& self, const Variable& var,
                            const double c) { return self.Substitute(var, c); })
      .def("Substitute",
           [](const Formula& self, const Substitution& s) {
             return self.Substitute(s);
           })
      .def("to_string", &Formula::to_string)
      .def("__str__", &Formula::to_string)
      .def("__repr__",
           [](const Formula& self) {
             return fmt::format("<Formula \"{}\">", self.to_string());
           })
      .def_loop("__eq__", [](const Formula& self,
                        const Formula& other) { return self.EqualTo(other); })
      .def("__ne__", [](const Formula& self,
                        const Formula& other) { return !self.EqualTo(other); })
      .def("__hash__",
           [](const Formula& self) { return std::hash<Formula>{}(self); })
      .def_static("True", &Formula::True)
      .def_static("False", &Formula::False)
      .def("__nonzero__", [](const Formula&) {
        throw py::cast_error("Cannot use `nonzero` on `Formula`");
      })
      .def("__cmp__", [](const Formula& a, const Formula& b) {
        // For dict, `PyObject_RichCompare`, to avoid the need for `nonzero`.
        return hash_cmp(a, b);
      });

  // Cannot hash Symbolic types if we want to disable `nonzero`, which we should.
  // Solution is to wrap `dict` and use a HashProxy key object, HashProxyDict,
  // which defines equality and inequality based on hashing, not rich comparison
  // (__eq__, etc.). We cannot use `__cmp__`, as `__eq__` is used in its place
  // if defined.

  m.def("trigger", []() {
    py::print("triggered");
  });

  // Cannot overload logical operators: http://stackoverflow.com/a/471561
  // Defining custom function for clarity.
  // Could use bitwise operators:
  // https://docs.python.org/2/library/operator.html#operator.__and__
  // However, this may reduce clarity and introduces constraints on order of
  // operations.
  m
      // Hide AND and OR to permit us to make it accept 1 or more arguments in
      // Python (and not have to handle type safety within C++).
      .def("__logical_and",
           [](const Formula& a, const Formula& b) { return a && b; })
      .def("__logical_or",
           [](const Formula& a, const Formula& b) { return a || b; })
      .def("logical_not", [](const Formula& a) { return !a; });

  py::class_<Monomial>(m, "Monomial")
      .def(py::init<const Variable&>())
      .def(py::init<const Variable&, int>())
      .def(py::init<const map<Variable, int>&>())
      .def("degree", &Monomial::degree)
      .def("total_degree", &Monomial::total_degree)
      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__hash__",
           [](const Monomial& self) { return std::hash<Monomial>{}(self); })
      .def(py::self != py::self)
      .def("__str__",
           [](const Monomial& self) { return fmt::format("{}", self); })
      .def("__repr__",
           [](const Monomial& self) {
             return fmt::format("<Monomial \"{}\">", self);
           })
      .def("GetVariables", &Monomial::GetVariables)
      .def("get_powers", &Monomial::get_powers, py_reference_internal)
      .def("ToExpression", &Monomial::ToExpression)
      .def("pow_in_place", &Monomial::pow_in_place, py_reference_internal)
      .def("__pow__",
           [](const Monomial& self, const int p) { return pow(self, p); });

  m.def("MonomialBasis",
        [](const Eigen::Ref<const VectorX<Variable>>& vars, const int degree) {
          return MonomialBasis(Variables{vars}, degree);
        })
      .def("MonomialBasis", [](const Variables& vars, const int degree) {
        return MonomialBasis(vars, degree);
      });

  py::class_<Polynomial>(m, "Polynomial")
      .def(py::init<>())
      .def(py::init<Polynomial::MapType>())
      .def(py::init<const Monomial&>())
      .def(py::init<const Expression&>())
      .def(py::init<const Expression&, const Variables&>())
      .def(py::init([](const Expression& e,
                       const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Polynomial{e, Variables{vars}};
      }))
      .def("indeterminates", &Polynomial::indeterminates)
      .def("decision_variables", &Polynomial::decision_variables)
      .def("Degree", &Polynomial::Degree)
      .def("TotalDegree", &Polynomial::TotalDegree)
      .def("monomial_to_coefficient_map",
           &Polynomial::monomial_to_coefficient_map)
      .def("ToExpression", &Polynomial::ToExpression)
      .def("Differentiate", &Polynomial::Differentiate)
      .def("AddProduct", &Polynomial::AddProduct)
      .def(py::self + py::self)
      .def(py::self + Monomial())
      .def(Monomial() + py::self)
      .def(py::self + double())
      .def(double() + py::self)
      .def(py::self - py::self)
      .def(py::self - Monomial())
      .def(Monomial() - py::self)
      .def(py::self - double())
      .def(double() - py::self)
      .def(py::self * py::self)
      .def(py::self * Monomial())
      .def(Monomial() * py::self)
      .def(py::self * double())
      .def(double() * py::self)
      .def(-py::self)
      .def("EqualTo", &Polynomial::EqualTo)
      // .def(py::self == py::self)
      .def("__eq__", [](const Polynomial& a, const Polynomial& b) {
        using namespace std;
        cerr << "compare:" << endl;
        cerr << "  a: " << a << endl;
        cerr << "  b: " << b << endl;
        cout << "  a == b: " << (a == b) << endl;
        return a == b;
      })
      .def(py::self != py::self)
      .def("__hash__",
           [](const Polynomial& self) { return std::hash<Polynomial>{}(self); })
      .def("__str__",
           [](const Polynomial& self) { return fmt::format("{}", self); })
      .def("__repr__",
           [](const Polynomial& self) {
             return fmt::format("<Polynomial \"{}\">", self);
           })
      .def("__pow__",
           [](const Polynomial& self, const int n) { return pow(self, n); })
      .def("Jacobian", [](const Polynomial& p,
                          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return p.Jacobian(vars);
      });

  py::implicitly_convertible<drake::symbolic::Monomial,
                             drake::symbolic::Polynomial>();
}

}  // namespace pydrake
}  // namespace drake
