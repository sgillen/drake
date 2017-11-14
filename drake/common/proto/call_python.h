#pragma once

#include <string>
#include <vector>

#include "drake/common/proto/call_matlab.h"
#include "drake/common/copyable_unique_ptr.h"

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

template <typename ... Types>
PythonRemoteVariable ToPythonTuple(Types... args);


inline void DelPythonVar(const PythonRemoteVariable& /*var*/) {
  // MatlabRPC msg;
  // ToMatlabArray(var, msg.add_rhs());
  // msg.set_function_name("_client_var_del");
  // internal::PublishCallMatlab(msg);
}

inline void CreatePythonVar(const PythonRemoteVariable& /*var*/) {}

template <typename T>
PythonRemoteVariable NewPythonVariable(T value);


class PythonRemoteVariable {
 public:
  PythonRemoteVariable();

  // TODO: Disable moving???

  ~PythonRemoteVariable() {
    DelPythonVar(*this);
  }

  int64_t unique_id() const { return unique_id_; }

  template <typename I>
  PythonRemoteVariable getitem(I index) const {
    return CallPython("getitem", *this, index);
  }

  template <typename I, typename T>
  PythonRemoteVariable setitem(I index, T val) const {
    return CallPython("setitem", *this, index, val);
  }

  PythonRemoteVariable getattr(const std::string& name) const {
    return CallPython("getattr", *this, name);
  }

  template <typename T>
  PythonRemoteVariable setattr(const std::string& name, T val) const {
    return CallPython("setattr", *this, name, val);
  }

  // Follow pybind11's example:
  // http://pybind11.readthedocs.io/en/stable/reference.html#_CPPv2NK10object_apiixE6handle
  class ItemAccessor {
   public:
    ItemAccessor(PythonRemoteVariable obj, PythonRemoteVariable index);
    ItemAccessor(const ItemAccessor&);
    ~ItemAccessor();

    operator PythonRemoteVariable() const;
    PythonRemoteVariable operator=(const PythonRemoteVariable& value);
    template <typename T>
    PythonRemoteVariable operator=(const T& value) {
      return *this = NewPythonVariable(value);
    }

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
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
PythonRemoteVariable ToPythonTuple(Types... args) {
  return CallPython("make_tuple", args...);
}

template <typename ... Types>
PythonRemoteVariable ToPythonKwargs(Types... args) {
  return CallPython("make_kwargs", args...);
}

inline PythonRemoteVariable ToPythonSlice(const std::string& expr) {
  return CallPython("make_slice", expr);
}

/// Creates a new remote variable with the corresponding value set.
template <typename T>
PythonRemoteVariable NewPythonVariable(T value) {
  return CallPython("pass_through", value);
}

}  // namespace common
}  // namespace drake
