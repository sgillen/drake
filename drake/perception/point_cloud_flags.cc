#include "drake/perception/point_cloud_flags.h"

#include <sstream>
#include <vector>

namespace drake {
namespace perception {

namespace {

// Utility for `ToString`.
std::string join(const std::vector<std::string>& elements,
                 const std::string& delim) {
  std::ostringstream os;
  for (size_t i = 0; i < elements.size(); ++i) {
    os << elements[i];
    if (i + 1 < elements.size())
      os << delim;
  }
  return os.str();
}

// Convert a PointCloud's set of fields to a string vector (for use
// with `ToString`).
std::vector<std::string> ToStringVector(
    pc_flags::Fields fields, const pc_flags::DescriptorType& descriptor_type) {
  std::vector<std::string> out;
  if (fields & pc_flags::kXYZs)
    out.push_back("kXYZs");
  if (fields & pc_flags::kDescriptors)
      out.push_back("kDescriptors::" + descriptor_type.name());
  return out;
}

}  // namespace

namespace pc_flags {

std::string ToString(Fields fields, const DescriptorType& descriptor_type) {
  return "(" + join(ToStringVector(fields, descriptor_type), " | ") + ")";
}

}  // namespace pc_flags

}  // namespace drake
}  // namespace perception
