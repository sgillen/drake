#pragma once

#include <pybind11/pybind11.h>

namespace drake {
namespace pydrake {

// `py::keep_alive` is used heavily throughout this code. For more
// information, please see:
// http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive
// Terse notes are added to method bindings to indicate the patient
// (object being kept alive by nurse) and the nurse (object keeping patient
// alive).
// - "Keep alive, ownership" implies that one argument is owned directly by
// one of the other arguments (`self` is included in those arguments, for
// `py::init<>` and class methods).
// - "Keep alive, reference" implies a reference that is lifetime-sensitive
// (something that is not necessarily owned by the other arguments).
// - "Keep alive, transitive" implies a transfer of ownership of owned
// objects from one container to another. (e.g. transfering all `System`s
// from `DiagramBuilder` to `Diagram` when calling
// `DiagramBuilder.Build()`.)
// N.B. `py::return_value_policy::reference_internal` implies
// `py::keep_alive<0, 1>`, which implies
// "Keep alive, reference: `return` keeps `self` alive".

// Aliases for commonly used return value policies.
// `py_reference` is used when `keep_alive` is explicitly used (e.g. for extraction
// methods, like `GetMutableSubsystemState`).
const auto py_reference = pybind11::return_value_policy::reference;
// `py_reference_internal` is used when pointers / lvalue references are returned (no need
// for `keep_alive`, as it is implicit.
const auto py_reference_internal =
    pybind11::return_value_policy::reference_internal;

// TODO(eric.cousineau): pybind11 defaults to C++-like copies when dealing
// with rvalues. We should wrap this into a drake-level binding, so that we
// can default this to `reference` or `reference_internal.`

namespace detail {

template <typename T>
struct wrap_arg {
  using type_in = T;
  static T&& unwrap(type_in&& arg) { return arg; }
  static type_in&& wrap(T&& arg) { return arg; }
};

// Ensure that all reference types are passed as pointers.
template <typename T>
struct wrap_arg<T&> {
  using type_in = T*;
  static T& unwrap(type_in arg) { return *arg; }
  static type_in wrap(T& arg) { return &arg; }
};

template <typename T>
using wrap_arg_in_t = typename wrap_arg<T>::type_in;

}  // namespace detail

// TODO(eric.cousineau): Make this lightweight, if the callback does not have
// capture?
template <typename Return, typename ... Args, typename Func>
auto WrapFunctionImpl(Func&& func) {
  using detail::wrap_arg;
  using detail::wrap_arg_in_t;
  auto func_wrapped =
      [std::forward<Func>(func)](wrap_arg_in_t<Args> args...) {
    return wrap_arg<Return>::wrap(
        func(
            std::forward<Args>(
                wrap_arg<Args>::unwrap(
                    std::forward<wrap_arg_in_t<Args>>(args)))...));
  };
}

template <typename Return, typename Class, typename ... Args>
auto WrapFunction(Return (Class::*method)(Args...)) {
  auto func = [method](Class* self, Args... args) {
    return self->(*method)(std::forward<Args>(args)...);
  };
  return WrapFunctionImpl<Return, Args...>(func);
}

template <typename Return, typename ... Args>
auto WrapFunction(Return (*func)(Args...)) {
  return WrapFunctionImpl<Return, Args...>(func);
}

template <typename Return, typename ... Args>
auto WrapFunction(std::function<Return (Args...)> func) {
  return WrapFunctionImpl<Return, Args...>(std::move(func));
}

}  // namespace pydrake
}  // namespace drake
