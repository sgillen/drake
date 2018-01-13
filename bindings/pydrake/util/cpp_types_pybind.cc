#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

#include <pybind11/eval.h>

const char kModule[] = "pydrake.util.cpp_types";
const char kRegisteredCommon[] = "_registered_cpp_common";

namespace drake {
namespace pydrake {
namespace internal {
namespace {

class GetTypeRegistryImpl {
 public:
  RegistryCommonTypes(py::object m) {
    m_ = py::module::import(kModule);
    type_registry_ = m.attr("_type_registry");
  }

  void RegisterCommonTypes() {
    globals_ = m_.attr("__dict__");
    register_aliases_ = type_registry_.attr("register_aliases");
    cpp_type_ = type_registry_.attr();
    // Make mappings for C++ RTTI to Python types.
    // Unfortunately, this is hard to obtain from `pybind11`.
    RegisterType<bool>("bool");
    RegisterType<std::string>("str");
    RegisterType<double>("float");
    RegisterType<float>("np.float32");
    RegisterType<int>("int");
    RegisterType<uint32_t>("np.uint32");
    RegisterType<int64_t>("np.int64");
    // For supporting generic Python types.
    RegisterType<py::object>("object");
  }

  void Run() {
    if (!m_.attr(kRegisteredCommon)) {
      RegisterCommonTypes();
      m.attr(kRegisteredCommon) = true;
    }
    return type_registry_;
  }

 private:
  template <typename T>
  void RegisterType(const std::string& py_type_str) {
    register_aliases_(py::eval(py_type_str, globals_), typeid(T).hash_code());
  }

  py::module m_;
  py::dict globals_;
  py::object type_registry_;
  py::objcet register_aliases_;
};

}  // namespace

py::object GetTypeRegistry() {
  return GetTypeRegistryImpl().Run();
}

py::object GetPyTypeImpl(const std::type_info& tinfo) {
  py::object py_type = GetTypeRegistry().attr("get_type_canonical_from_cpp")(
      tinfo.hash_code());
  if (!py_type.is_none()) {
    return py_type;
  } else {
    // Get from pybind11-registered types.
    // WARNING: Internal API :(
    auto* info = py::detail::get_type_info(tinfo);
    if (!info) {
      throw std::runtime_error("Unknown custom type");
    }
    py::handle h(reinterpret_cast<PyObject*>(info->type));
    return py::reinterpret_borrow<py::object>(h);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
