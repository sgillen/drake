#pragma once

/// @file
/// Helpers for defining C++ LCM type serializers.

#include <string>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

template <typename CppType>
py::object BindCppSerializer(const std::string& lcm_package) {
  using systems::lcm::Serializer;
  using systems::lcm::SerializerInterface;
  // Retrieve Python type using C++ name.
  // N.B. Since the LCM type does not supply the package, we need it supplied.
  py::object py_type =
      py::module::import(lcm_package.c_str()).attr(CppType::getTypeName());
  py::module lcm_py = py::module::import("pydrake.systems.lcm");
  auto py_cls = DefineTemplateClass<Serializer<CppType>, SerializerInterface>(
      lcm_py, "CppSerializer", py::make_tuple(py_type));
  py_cls.def(py::init());
  return py_cls;
}

}  // naemspace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
