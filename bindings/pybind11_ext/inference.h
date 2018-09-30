#pragma once

#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/bindings/pydrake/util/wrap_function.h"

namespace pybind11 {
namespace detail {

using drake::type_pack;

template <typename... A, typename... B>
auto type_pack_concat(type_pack<A...> = {}, type_pack<B...> = {}) {
  return type_pack<A..., B...>{};
}

template <template <typename> class Apply, typename... T>
auto type_pack_apply(type_pack<T...> = {}) {
  return type_pack<Apply<T>...>{};
}

using drake::pydrake::detail::infer_function_info;

}  // namespace detail
}  // namespace pybind11
