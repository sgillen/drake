#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

#include <pybind11/eval.h>

const char kModule[] = "pydrake.util.cpp_types";

namespace drake {
namespace pydrake {

namespace internal {

TypeRegistry::TypeRegistry() {
  // Import modules into `locals_`.
  globals_ = py::module::import(kModule).attr("__dict__");
  py_to_py_canonical_ = eval("_StrictMap")();

  RegisterCommon();
  RegisterLiterals();
}

const TypeRegistry& TypeRegistry::GetPyInstance() {
  // TODO: Shift the Pythonic logic / variables into `cpp_types.py`.
  auto m = py::module::import(kModule);
  py::object type_registry_py = m.attr("_type_registry");
  const TypeRegistry* type_registry =
      py::cast<const TypeRegistry*>(type_registry_py);
  return *type_registry;
}

py::object TypeRegistry::DoGetPyType(const std::type_info& tinfo) const {
  // Check if it's a custom-registered type.
  size_t cpp_key = std::type_index(tinfo).hash_code();
  auto iter = cpp_to_py_.find(cpp_key);
  if (iter != cpp_to_py_.end()) {
    return iter->second;
  } else {
    // Get from pybind11-registered types.
    // WARNING: Internal API :(
    auto* info = py::detail::get_type_info(tinfo);
    if (!info) {
      throw std::runtime_error("Unknown type!");
    }
    return py::reinterpret_borrow<py::object>(
        py::handle(reinterpret_cast<PyObject*>(info->type)));
  }
}

py::object TypeRegistry::GetPyTypeCanonical(py::object py_type) const {
  // Since there's no easy / good way to expose C++ type id's to Python,
  // just canonicalize Python types.
  return py_to_py_canonical_.attr("get")(py_type, py_type);
}

py::str TypeRegistry::GetName(py::object py_type) const {
  py::object py_type_fin = GetPyTypeCanonical(py_type);
  py::object out = py_name_.attr("get")(py_type_fin);
  // Assume this is a Python type.
  if (out.is(py::none())) {
    out = eval("_get_type_name")(py_type_fin);
  }
  return out;
}

py::object TypeRegistry::eval(const std::string& expr) const {
  return py::eval(expr, globals_, locals_);
}

void TypeRegistry::Register(
      const std::vector<size_t>& cpp_keys,
      py::tuple py_types, const std::string& name) {
  py::object py_canonical = py_types[0];
  for (size_t cpp_key : cpp_keys) {
    assert(cpp_to_py_.find(cpp_key) == cpp_to_py_.end());
    cpp_to_py_[cpp_key] = py_canonical;
  }
  for (auto py_type : py_types) {
    py_to_py_canonical_.attr("add")(py_type, py_canonical);
  }
  py_name_[py_canonical] = name;
}

template <typename T>
void TypeRegistry::RegisterType(
    py::tuple py_types, const std::string& name_override) {
  std::string name = name_override;
  if (name.empty()) {
    name = py::cast<std::string>(eval("_get_type_name")(py_types[0]));
  }
  Register({type_hash<T>()}, py_types, name);
}

void TypeRegistry::RegisterCommon() {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>(eval("bool,"));
  RegisterType<std::string>(eval("str,"));
  RegisterType<double>(eval("float, np.double, ctypes.c_double"));
  RegisterType<float>(eval("np.float32, ctypes.c_float"));
  RegisterType<int>(eval("int, np.int32, ctypes.c_int32"));
  RegisterType<uint32_t>(eval("np.uint32, ctypes.c_uint32"));
  RegisterType<int64_t>(eval("np.int64, ctypes.c_int64"));
  // For supporting generic Python types.
  RegisterType<py::object>(eval("object"));
}

class TypeRegistry::LiteralHelper {
 public:
  LiteralHelper(TypeRegistry* type_registry)
    : type_registry_(type_registry) {}

  void RegisterLiterals() {
    RegisterSequence(Render(std::integer_sequence<bool, false, true>{}));
    // Register `int` (and `uint` as an alias for positive values).
    constexpr int i_max = 100;
    RegisterSequence(MakeSequence<int, -i_max, -1>());
    RegisterSequence(
        MakeSequence<int, 0, i_max>(),
        {MakeSequence<uint, 0, i_max, int>()});
  }

 private:
  template <typename T>
  struct Sequence {
    std::vector<size_t> keys;
    std::vector<T> values;
  };

  template <typename T, typename IsAliasFor = T, T... Values>
  Sequence<IsAliasFor> Render(std::integer_sequence<T, Values...>) {
    return Sequence<IsAliasFor>{
      {type_hash<std::integral_constant<T, Values>>()...},
      {Values...}};
  }

  template <typename T, T Start, T End, typename IsAliasFor = T>
  Sequence<IsAliasFor> MakeSequence() {
    constexpr T Count = End - Start + 1;
    auto seq = sequence_transform(
        constant_add<T, Start>{}, std::make_integer_sequence<T, Count>{});
    return Render<T, IsAliasFor>(seq);
  }

  template <typename T>
  void RegisterSequence(
      const Sequence<T>& seq,
      std::vector<Sequence<T>> alias_set = {}) {
    for (int i = 0; i < seq.keys.size(); ++i) {
      // Get alias types.
      std::vector<size_t> cpp_keys{seq.keys[i]};
      for (const auto& alias : alias_set) {
        assert(seq.values[i] == alias.values[i]);
        cpp_keys.push_back(alias.keys[i]);
      }
      // Register.
      T value = seq.values[i];
      py::object py_value = py::cast(value);
      type_registry_->Register(
          cpp_keys,
          py::make_tuple(py_value),
          py::str(py_value).cast<std::string>());
    }
  }

  TypeRegistry* type_registry_;
};

void TypeRegistry::RegisterLiterals() {
  // Register a subset of literals.
  LiteralHelper(this).RegisterLiterals();
}

}  // namespace internal

}  // namespace pydrake
}  // namespace drake
