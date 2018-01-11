#pragma once

/// @file
/// Provides a mechanism to map C++ types to canonical Python types.

#include <typeinfo>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/util/type_pack.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {
namespace internal {

py::object GetTypeRegistry();

py::object GetPyTypeImpl(const std::type_info& tinfo);

template <typename ... Ts>
void RegisterTypes(py::tuple py_types, type_pack<Ts...> = {}) {
  std::vector<size_t> cpp_types = {typeid(Ts).hash_code()...};
  GetTypeRegistry().attr("register")(cpp_types, py_types);
}

template <typename T, typename = void>
struct get_py_type_impl {
  static py::object run() {
    return GetPyTypeImpl(typeid(T));
  }
};

template <typename T, T Value>
struct get_py_type_impl<std::integral_constant<T, Value>> {
  static py::object run() {
    return py::cast(Value);
  }
};

}  // namespace internal

/// Gets the canonical Python type for a given C++ type.
template <typename T>
inline py::object GetPyType(type_pack<T> = {}) {
  return internal::get_py_type_impl<T>::run();
}

/// Gets the canonical Python types for each C++ type.
template <typename ... Ts>
inline py::tuple GetPyTypes(type_pack<Ts...> = {}) {
  return py::make_tuple(GetPyType<Ts>()...);
}

}  // namespace pydrake
}  // namespace drake
