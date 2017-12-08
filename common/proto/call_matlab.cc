#include "drake/common/proto/call_matlab.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace common {

static int globally_unique_id = 0;

MatlabRemoteVariable::MatlabRemoteVariable()
    : unique_id_(globally_unique_id++)
// TODO(russt): replace this with a random int64_t, e.g.
// http://stackoverflow.com/questions/7114043/random-number-generation-in-c11-how-to-generate-how-do-they-work
// TODO(russt): david-german-tri recommended a more robust (but more complex)
// solution was to use e.g. [IP address + process id + time].  We decided this
// was sufficient for now.
{}

void ToMatlabArray(const MatlabRemoteVariable& var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::REMOTE_VARIABLE_REFERENCE);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int64_t);
  int64_t uid = var.unique_id();
  matlab_array->set_data(&uid, num_bytes);
}

void ToMatlabArray(double var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(double);
  matlab_array->set_data(&var, num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void ToMatlabArray(int var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::INT);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int);
  matlab_array->set_data(&var, num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::INT);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const MatrixX<bool>>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::LOGICAL);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::CHAR);
  matlab_array->set_shape_type(MatlabArray::VECTOR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(str.length());
  int num_bytes = sizeof(char) * str.length();
  matlab_array->set_data(str.data(), num_bytes);
}

namespace internal {

std::unique_ptr<std::ofstream>
CreateOutputStream(const std::string& filename) {
  // NOTE(russt): This code violates the style-guide by expecting the file to be
  // closed properly at program termination (these streams will be stored as
  // static globals).
  auto raw_output =
      std::make_unique<std::ofstream>(filename.c_str(), std::ios::binary);
  return raw_output;
}

void PublishCallMatlab(const MatlabRPC& message) {
  // TODO(russt): Provide option for setting the filename.
  static auto raw_output = CreateOutputStream("/tmp/matlab_rpc");
  PublishCall(raw_output.get(), message);
}

void PublishCall(
    std::ofstream* praw_output,
    const MatlabRPC& message) {
  DRAKE_DEMAND(praw_output);
  auto& raw_output = *praw_output;

  {  // Defines the lifetime of the CodedOutputStream.
    // Write the size.
    const int size = message.ByteSize();
    DRAKE_ASSERT(sizeof(int) == 4);
    raw_output.write(reinterpret_cast<const char*>(&size), sizeof(int));
    
    message.SerializeToOstream(&raw_output);
    DRAKE_DEMAND(raw_output.good());
  }

  raw_output.flush();
}

}  // namespace internal
}  // namespace common
}  // namespace drake
