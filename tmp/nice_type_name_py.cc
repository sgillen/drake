#include <string>
#include <regex>
#include <vector>

#include "pybind11/pybind11.h"

#include "drake/common/nice_type_name.h"
#include "drake/common/never_destroyed.h"

namespace drake {

namespace py = pybind11;

namespace {

// Create an arbitrary type name. For fun.
template <typename T>
class ArbitraryName {
 public:
  using Type = ArbitraryName;

  ArbitraryName() {
    using SPair = std::pair<std::regex, std::string>;
    never_destroyed<std::regex> sub;
    sub.access() = std::regex("\\b(class|struct|enum|union) ");
    py::print((void*)&sub);
    // name_ = NiceTypeName::Get<Type>();
  }

  std::string name() const { return name_; }

 private:
  std::string name_;
};

PYBIND11_MODULE(nice_type_name, m) {
  // Try to use codepaths that will excite the same error as in drake#12073.
  {
    using Class = ArbitraryName<std::string>;
    py::class_<Class>(m, "ArbitraryName")
        .def(py::init())
        .def("name", &Class::name);
  }
}

}  // namespace
}  // namespace drake
