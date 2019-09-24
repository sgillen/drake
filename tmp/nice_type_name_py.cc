#include <string>

#include "pybind11/pybind11.h"

#include "drake/common/nice_type_name.h"

namespace drake {

namespace {

// Create an arbitrary type name. For fun.
template <typename T>
class ArbitraryName { };

namespace py = pybind11;

PYBIND11_MODULE(nice_type_name, m) {
  // Try to use codepaths that will excite the same error as in drake#12073.
  m.def("get_arbitrary_type_name", []() {
    using Type = ArbitraryName<std::string>();
    return NiceTypeName::Get<Type>();
  });
}

}  // namespace
}  // namespace drake
