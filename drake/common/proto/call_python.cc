#include "drake/common/proto/call_python.h"

#include "drake/common/proto/matlab_rpc.pb.h"

namespace drake {
namespace common {

static int py_globally_unique_id = 0;

PythonRemoteVariable::PythonRemoteVariable()
    : unique_id_(py_globally_unique_id++)
// TODO(eric.cousineau): Make this consistent with what `call_matlab` has when
// it changes.
{
  CreatePythonVar(*this);
}

struct PythonRemoteVariable::ItemAccessor::Impl {
  PythonRemoteVariable obj;
  PythonRemoteVariable index;
};

PythonRemoteVariable::ItemAccessor::ItemAccessor(
    PythonRemoteVariable obj, PythonRemoteVariable index)
  : impl_(new Impl{obj, index}) {}

PythonRemoteVariable::ItemAccessor::ItemAccessor(
    const ItemAccessor& other)
  : impl_(new Impl(*other.impl_)) {}

PythonRemoteVariable::ItemAccessor::~ItemAccessor() {}

PythonRemoteVariable::ItemAccessor::operator
    PythonRemoteVariable() const {
  return impl_->obj.getitem(impl_->index);
}

PythonRemoteVariable PythonRemoteVariable::ItemAccessor::operator=(const PythonRemoteVariable& value) {
  return impl_->obj.setitem(impl_->index, value);
}

void ToMatlabArray(const PythonRemoteVariable& var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::REMOTE_VARIABLE_REFERENCE);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int64_t);
  int64_t uid = var.unique_id();
  matlab_array->set_data(&uid, num_bytes);
}

}  // namespace common
}  // namespace drake
