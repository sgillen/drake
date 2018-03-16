#pragma once

#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/util/wrap_function.h"

namespace drake {
namespace pydrake {

namespace detail {

template <template <typename> class wrap_arg_policy, typename PyClass>
class wrap_def_impl {
 public:
  using Class = typename PyClass::type;

  wrap_def_impl(PyClass* cls) : cls_{cls} {}

  // Mirror overloads of `py::class_<>::def`.

  // Function overloads.
  template <typename Func, typename... Extra>
  wrap_def_impl& def(const char* name, Func&& func, const Extra&... extra) {
    cls_->def(
        name, WrapFunction<wrap_arg_policy>(std::forward<Func>(func)),
        extra...);
    return *this;
  }

  // Operator overloads.
  template <
      py::detail::op_id id, py::detail::op_type ot,
      typename L, typename R, typename... Extra>
  wrap_def_impl& def(
      const py::detail::op_<id, ot, L, R>&, const Extra&... extra) {
    using op_ = py::detail::op_<id, ot, L, R>;
    using op_traits = typename op_::template info<PyClass>::op;
    cls_->def(
        op_traits::name(),
        WrapFunction<wrap_arg_policy>(&op_traits::execute),
        py::is_operator(), extra...);
    return *this;
  }

 private:
  PyClass* cls_;
};

}  // namespace detail

/**
 * Returns a class which proxies to a pybind11 class_ instance, and
 * defines wrapped operators using `wrap_eval_policy`.
 */
template <template <typename> class wrap_arg_policy, typename PyClass>
auto WrapDef(PyClass* cls) {
  return detail::wrap_def_impl<wrap_arg_policy, PyClass>(cls);
}

}  // namespace pydrake
}  // namespace drake
