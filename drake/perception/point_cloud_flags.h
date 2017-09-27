#pragma once

#include <stdexcept>
#include <string>

namespace drake {
namespace perception {

/// Point cloud flags.
namespace pc_flags {

typedef int BaseFieldT;
/// Indicates the data the point cloud stores.
enum BaseField : BaseFieldT {
  kNone = 0,
  /// Inherit other fields. May imply an intersection of all
  /// compatible descriptors.
  kInherit = 1 << 0,
  /// XYZ point in Cartesian space.
  kXYZs = 1 << 1,
};

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

  DescriptorType(const DescriptorType&) = default;

  // Set up defaults.
  DescriptorType(DescriptorType&&) = delete;
  DescriptorType& operator=(const DescriptorType&) = default;
  DescriptorType& operator=(DescriptorType&&) = default;

  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  inline bool operator==(const DescriptorType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  inline bool operator!=(const DescriptorType& other) const {
    return !(*this == other);
  }

 private:
  int size_{};
  std::string name_;
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


/**
 * Allows combination of these two field types (FieldType and DescriptorType).
 */
class Fields {
 public:
  Fields(BaseFieldT fields)
      : fields_(fields) {}

  Fields(const DescriptorType& descriptor_type)
      : descriptor_type_(descriptor_type) {}

  bool has_fields() const {
    return fields_ != kNone;
  }

  bool has_descriptor() const {
    return descriptor_type_ != kDescriptorNone;
  }

  Fields& operator|=(const Fields& other) {
    fields_ = static_cast<BaseFieldT>(fields_ | other.fields_);
    if (has_descriptor())
      throw std::runtime_error(
          "Cannot have multiple Descriptor flags. "
          "Can only add flags iff (flags.b() == kDescriptorNone).");
    descriptor_type_ = other.descriptor_type_;
    return *this;
  }

  Fields operator|(const Fields& rhs) const {
    return Fields(*this) |= rhs;
  }

  /// Provides intersection.
  Fields operator&(const Fields& rhs) const {

  }

  operator bool() const {
    return fields_ != kNone || descriptor_type_ != kDescriptorNone;
  }

  bool operator==(const Fields& rhs) const {
    return fields_ == rhs.fields_ && descriptor_type_ == rhs.descriptor_type_;
  }

 private:
  BaseFieldT fields_{kNone};
  DescriptorType descriptor_type_{kDescriptorNone};
};

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
