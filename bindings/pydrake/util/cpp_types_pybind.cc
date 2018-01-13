#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

#include <pybind11/eval.h>

const char kModule[] = "pydrake.util.cpp_types";
const char kRegisteredCommon[] = "_registered_cpp_common";

namespace drake {
namespace pydrake {
namespace internal {
namespace {

template <typename T>
py::object GetHash() {
  // Creates a Python object that should uniquely hash for a primitive C++
  // type.
  return py::make_tuple("cpp_type", typeid(T).hash_code());
}

class GetTypeAliasesImpl {
 public:
  GetTypeAliasesImpl() {
    m_ = py::module::import(kModule);
    aliases_ = m_.attr("_aliases");
  }

  py::object Run() {
    if (!py::hasattr(m_, kRegisteredCommon)) {
      RegisterCommonTypes();
      m_.attr(kRegisteredCommon) = true;
    }
    return aliases_;
  }

  void RegisterCommonTypes() {
    globals_ = m_.attr("__dict__");
    register_ = aliases_.attr("register");
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

 private:
  template <typename T>
  void RegisterType(const std::string& py_type_str) {
    // Create an object that is a unique hash.
    register_(py::eval(py_type_str, globals_), GetHash<T>());
  }

  py::module m_;
  py::dict globals_;
  py::object aliases_;
  py::object register_;
};

}  // namespace

py::object GetTypeAliases() {
  return GetTypeAliasesImpl().Run();
}

py::object GetPyTypeImpl(const std::type_info& tinfo) {
  py::object py_type = GetTypeAliases().attr("get_type_canonical_from_cpp")(
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
