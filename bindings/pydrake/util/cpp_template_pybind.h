#pragma once

#include <string>
#include <utility>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// `GetPyTypes` is implemented specifically for `cpp_template`; to simplify
// dependencies, this is included transitively.
#include "drake/bindings/pydrake/util/cpp_param_pybind.h"

namespace drake {
namespace pydrake {

namespace py = pybind11;

namespace internal {

// See document
inline py::object GetOrInitTemplate(
    py::handle scope, const std::string& name,
    const std::string& template_type, py::tuple create_extra = py::tuple()) {
  const char module_name[] = "pydrake.util.cpp_template";
  py::handle m = py::module::import(module_name);
  return m.attr("get_or_init")(
      scope, name, m.attr(template_type.c_str()), *create_extra);
}

inline void AddInstantiation(
    py::handle py_template, py::handle obj,
    py::tuple param) {
  py_template.attr("add_instantiation")(param, obj);
}

inline std::string GetInstantiationName(
    py::handle py_template, py::tuple param) {
  return py::cast<std::string>(
    py_template.attr("_instantiation_name")(param));
}

}  // namespace internal

/// Adds a template class instantiation.
/// @param scope Parent scope of the template.
/// @param name Name of the template.
/// @param py_class Class instantiation to be added.
/// @param param Parameters for the instantiation.
/// @param default_instantiation_name Name of the default instantiation, which
/// will be set to `py_class` if it does not already exist.
inline py::object AddTemplateClass(
    py::handle scope, const std::string& name,
    py::handle py_class, py::tuple param,
    const std::string& default_instantiation_name = "") {
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateClass");
  internal::AddInstantiation(py_template, py_class, param);
  if (!default_instantiation_name.empty() &&
      !py::hasattr(scope, default_instantiation_name.c_str())) {
    scope.attr(default_instantiation_name.c_str()) = py_class;
  }
  return py_template;
}

/// Provides a temporary, unique name for a class instantiation that
/// will be passed to `AddTemplateClass`.
template <typename T>
std::string TemporaryClassName() {
  return std::string("_TemporaryClassName_") + typeid(T).name();
}

/// Declares a template function.
/// @param scope Parent scope of the template.
/// @param name Name of the template.
/// @param func Function to be added.
/// @param param Parameters for the instantiation.
template <typename Func>
py::object AddTemplateFunction(
    py::handle scope, const std::string& name, Func&& func,
    py::tuple param) {
  // TODO(eric.cousineau): Use `py::sibling` if overloads are needed.
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateFunction");
  py::object py_func = py::cpp_function(
        std::forward<Func>(func),
        py::name(internal::GetInstantiationName(py_template, param).c_str()));
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

/// Declares a template method.
/// @param scope Parent scope of the template. This should be a class.
/// @param name Name of the template.
/// @param func Function to be added.
/// @param param Parameters for the instantiation.
template <typename Func>
py::object AddTemplateMethod(
    py::handle scope, const std::string& name, Func&& func,
    py::tuple param) {
  py::object py_template =
      internal::GetOrInitTemplate(
          scope, name, "TemplateMethod", py::make_tuple(scope));
  py::object py_func = py::cpp_function(
      std::forward<Func>(func),
      py::name(internal::GetInstantiationName(py_template, param).c_str()),
      py::is_method(scope));
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

}  //  namespace pydrake
}  //  namespace drake
