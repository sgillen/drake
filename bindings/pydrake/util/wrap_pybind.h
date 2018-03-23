/// @file
/// Defines convenience utilities to wrap pybind11 methods and classes.

#pragma once

#include <string>
#include <utility>

#include "pybind11/pybind11.h"

namespace drake {
namespace pydrake {

/// Defines a function in object `a` and mirrors `def` calls to object `b`.
///
/// @tparam A Type of object `a`
/// @tparam B Type of object `b`
template <typename A, typename B>
class MirrorDef {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MirrorDef);

  MirrorDef(A* a, B* b)
      : a_(a), b_(b) {}

  /// Calls `def` for both `a` and `b`.
  template <typename... Args>
  MirrorDef& def(const char* name, Args&&... args) {
    a_->def(name, std::forward<Args>(args)...);
    b_->def(name, std::forward<Args>(args)...);
    return *this;
  }

 private:
  A* const a_{};
  B* const b_{};
};

/// Mirror ufunc loop definitions from NumPy to `math`
template <typename PyClass>
class UfuncMirrorDef {
 public:
  UfuncMirrorDef(PyClass* cls, py::module math)
    : cls_(cls), math_(math) {}

  template <typename Func>
  UfuncMirrorDef& def_loop(
      const char* cls_name, const char* math_name, const Func& func) {
    cls_->def_loop(cls_name, func);
    math_.def(math_name, func);
    return *this;
  }

  template <typename Func>
  UfuncMirrorDef& def_loop(
      const char* name, const Func& func) {
    return def_loop(name, name, func);
  }

 private:
  PyClass* const cls_{};
  py::module math_;
};

}  // namespace pydrake
}  // namespace drake
