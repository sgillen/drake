#pragma once

#include <string>
#include <vector>

#include "drake/common/call_matlab.h"

// Minor extensions to `call_matlab` to enable calling Python.
// N.B. This is NOT meant to replace or be similar to what pybind11 offers for
// its C++ API. Rather, this is a minimum-dependency approach.
// If more complex marhsalling is needed *only for Python*, then pybind11 should
// be used.
// Consider using `Py_IsInitialized()` to determine if a new interpreter should
// be created, and then consider using the `multiprocessing` module for IPC.

namespace drake {
namespace common {

class PythonRemoteVariable;
void ToMatlabArray(const PythonRemoteVariable& var, MatlabArray* matlab_array);

// TODO(eric.cousineau): For multiple things (e.g. multiple indices in
// `getitem`), a Python tuple is needed. Consider making a function,
// `make_tuple`.
// Would need something akin to a cell array in MATLAB.

// forward declaration:
template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args);

class PythonRemoteVariable {
 public:
  PythonRemoteVariable();

  int64_t unique_id() const { return unique_id_; }

  template <typename I>
  PythonRemoteVariable getitem(I index) const {
    return CallPython("getitem", *this, index);
  }

  template <typename I, typename T>
  PythonRemoteVariable setitem(I index, T val) const {
    return CallPython("setitem", *this, index, val);
  }

  // Follow pybind11's example:
  // http://pybind11.readthedocs.io/en/stable/reference.html#_CPPv2NK10object_apiixE6handle
  class ItemAccessor {
   public:
    ItemAccessor(PythonRemoteVariable obj, PythonRemoteVariable index)
      : obj_(obj),
        index_(index) {}
    operator const PythonRemoteVariable&() const {
      return obj_.getitem(index_);
    }
    PythonRemoteVariable operator=(const PythonRemoteVariable& value) {
      obj_.setitem(index_, value);
    }
   private:
    PythonRemoteVariable obj_;
    PythonRemoteVariable index_;
  };

  template <typename ... Types>
  ItemAccessor operator()(Types ... args) const {
    return ItemAccessor(*this, ToPythonTuple(args...));
  }

 private:
  const int64_t unique_id_{};
};

template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args) {
  PythonRemoteVariable output;
  MatlabRPC msg;
  msg.add_lhs(output.unique_id());
  internal::AssembleCallMatlabMsg(&msg, args...);
  msg.set_function_name(function_name);
  internal::PublishCallMatlab(msg);
  return output;
}

template <typename ... Types>
PythonRemoteVariable ToPythonTuple(Args... args) {
  return CallPython("make_tuple", args...);
}

template <typename ... Types>
PythonRemoteVariable ToPythonKwargs(Args... args) {
  return CallPython("make_kwargs", args...);
}

PythonRemoteVariable ToPythonSlice(const std::string& expr) {
  return CallPython("make_slice", expr);
}

/// Creates a new remote variable with the corresponding value set.
template <typename T>
PythonRemoteVariable NewPythonVariable(T value) {
  return CallPython("pass_through", value);
}

}  // namespace common
}  // namespace drake
