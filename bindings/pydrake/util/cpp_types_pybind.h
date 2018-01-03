#pragma once

/// @file
/// Helper methods to 

#include <string>
#include <map>

#include <pybind11/pybind11.h>

#include "drake/common/type_util.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {

namespace internal {

// Provides a mechanism to map between Python and C++ types.
class TypeRegistry {
 public:
  TypeRegistry();

  // Gets singleton instance, stored in Python.
  // @note Storing this in Python permits this module to violate ODR, until
  // we have shared library linking.
  static const TypeRegistry& GetPyInstance();

  // Gets the Python type for a given C++ type.
  template <typename T>
  py::handle GetPyType() const {
    return DoGetPyType(typeid(T));
  }

  // Gets the canonical Python type for a given Python type.
  py::handle GetPyTypeCanonical(py::handle py_type) const;

  // Gets the canonical string name for a given Python type.
  py::str GetName(py::handle py_type) const;

 private:
  py::object eval(const std::string& expr) const;

  void exec(const std::string& expr);

  py::handle DoGetPyType(const std::type_info& tinfo) const;

  void Register(
      const std::vector<size_t>& cpp_keys,
      py::tuple py_types, const std::string& name);

  template <typename T>
  void RegisterType(py::tuple py_types,
                const std::string& name_override = {});

  void RegisterCommon();
  void RegisterLiterals();

  class LiteralHelper;
  friend class LiteralHelper;

  py::object globals_;
  py::object locals_;
  std::map<size_t, py::handle> cpp_to_py_;
  py::object py_to_py_canonical_;
  py::dict py_name_;
};

}  // namespace internal

// Gets the canonical Python type for a given C++ type.
template <typename T>
inline py::object GetPyType(type_pack<T> = {}) {
  auto& type_registry = internal::TypeRegistry::GetPyInstance();
  return type_registry.GetPyType<T>();
}

// Gets the canonical Python type for a list of C++ types.
template <typename ... Ts>
inline py::tuple GetPyTypes(type_pack<Ts...> = {}) {
  return py::make_tuple(get_py_type<Ts>()...);
}

/// Gets the canonical string name for a given C++ type.
template <typename T>
inline std::string GetPyName(type_pack<T> = {}) {
  auto& type_registry = internal::TypeRegistry::GetPyInstance();
  return type_registry.GetName(type_registry.GetPyType<T>());
}

/// Gets the canonical string names for a list of C++ types.
template <typename ... Ts>
inline std::vector<std::string> GetPyNames(type_pack<Ts...> = {}) {
  return {get_py_name<Ts>()...};
}

}  // namespace pydrake
}  // namespace drake
