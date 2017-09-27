#pragma once

#include <string>

namespace drake {
namespace perception {

/// Point cloud flags.
namespace pc_flags {

/// Indicates the data the point cloud stores.
enum Field : int {
  /// Inherit other fields. May imply an intersection of all
  /// compatible descriptors.
  kInherit = 0,
  /// XYZ point in Cartesian space.
  kXYZs = 1 << 0,
  /// Descriptors, whose type (and structure) is specified by `DescriptorType`.
  kDescriptors = 1 << 3,
};
typedef int Fields;

/// Describes an descriptor field with a name and the descriptor's size.
///
/// @note This is defined as follows to enable an open set of descriptors, but
/// ensure that these descriptor types are appropriately matched.
/// As `PointCloud` evolves and more algorithms are mapped into Drake,
/// promoting an descriptor field to a proper field should be considered if (a)
/// it is used frequently enough AND (b) if it is often used in conjunction
/// with other fields.
class DescriptorType final {
 public:
  DescriptorType(int size, const std::string& name)
      : size_(size),
        name_(name) {}

  DescriptorType(const DescriptorType& other)
      : DescriptorType(other.size(), other.name()) {}

  // Strictly enforce immutable semantics.
  DescriptorType(const DescriptorType&&) = delete;
  DescriptorType& operator=(const DescriptorType&) = delete;
  DescriptorType& operator=(DescriptorType&&) = delete;

  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  inline bool operator==(const DescriptorType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  inline bool operator!=(const DescriptorType& other) const {
    return !(*this == other);
  }

 private:
  const int size_{};
  const std::string name_;
};

/// No descriptor.
const DescriptorType kDescriptorNone(0, "None");
/// Inherited descriptor.
const DescriptorType kDescriptorInherit(0, "Inherit");
/// Curvature.
const DescriptorType kDescriptorCurvature(1, "Curvature");
/// Point-feature-histogram.
const DescriptorType kDescriptorFPFH(33, "FPFH");

/// Returns a human-friendly representation of a field and descriptor set.
std::string ToString(Fields fields, const DescriptorType& descriptor_type);

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
