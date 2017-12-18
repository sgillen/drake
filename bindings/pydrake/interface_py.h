/// @file
/// Definitions used common to system Python bindings.

#include <pybind11/pybind11.h>

// @note Including `pybind11/eigen.h` slows down compilation quite a bit.
// Only include it when necessary.

namespace pydrake {

// Ensure that all pydrake classes tend towards `shared_ptr`.
template <typename T, typename ... Args>
class drake_class : public pybind11::class_<T, std::shared_ptr<T>, Args...> {
 public:
  using Base = pybind11::class_<T, std::shared_ptr<T>, Args...>;
  using Base::Base;
};

// Return value policies:

// Aliases for commonly used return value policies.
// `return_value_policy::reference` is used when `keep_alive` is explicitly
// used (e.g. for extraction methods, like `GetMutableSubsystemState`).

// `return_value_policy::reference_internal` is used when pointers / lvalue
// references are returned (no need for `keep_alive`, as it is implicit).

}  // namespace pydrake
