#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

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


/// Implements a point cloud (with contiguous storage), whose main goal is to
/// offer a convenient, synchronized interface to commonly used fields and
/// data types applicable for basic 3D perception.
///
/// This is a mix between the philosophy of PCL (templated interface to
/// provide a compile-time open set, run-time closed set) and VTK (non-templated
/// interface to provide a very free form run-time open set).
/// You may construct one PointCloud which will contain different sets of
/// data, but you cannot change the contained data types after construction.
/// However, you can mutate the data contained within the structure and resize
/// the structure.
///
/// Definitions:
///  - point - An entry in a point cloud (not exclusively an XYZ point).
///  - feature - Abstract representation of local properties (geometric and
/// non-geometric)
///  - descriptor - Concrete representation of a feature.
///  - field - A feature or descriptor described by the point cloud.
///
/// This point cloud class provides the following fields:
///  - xyz - Cartesian XYZ coordinates (float[3]).
///  - descriptor - An descriptor that is run-time defined (float[X]).
///
/// @note "contiguous" here means contiguous in memory. This was chosen to
/// avoid ambiguity between PCL and Eigen, where in PCL "dense" implies that
/// the point cloud corresponds to a cloud with invalid values, and in Eigen
/// "dense" implies contiguous storage.
///
/// @note The accessors / mutators for the point fields of this class returns
/// references to the original Eigen matrices. This implies that they are
/// invalidated whenever memory is reallocated for the values. Given this,
/// minimize the lifetime of these references to be as short as possible.
/// Additionally, algorithms wanting fast access to values should avoid the
/// single point accessors (e.g. `xyz(i)`, `descriptor(i)`).
///
/// @note The definitions presented here for "feature" and "descriptor" are
/// loosely based on their definitions within PCL and Radu Rusu's dissertation:
///   Rusu, Radu Bogdan. "Semantic 3d object maps for everyday manipulation in
///   human living environments." KI-Künstliche Intelligenz 24.4 (2010):
///   345-348.
/// This differs from other definitions, such as having "feature"
/// describe geometric quantities and "descriptor" describe non-geometric
/// quantities which is presented in the following survey paper:
///   Pomerleau, François, Francis Colas, and Roland Siegwart. "A review of
///   point cloud registration algorithms for mobile robotics." Foundations and
///   Trends® in Robotics 4.1 (2015): 1-104.
class PointCloud final {
 public:
  /// Geometric scalar type.
  typedef float T;

  /// Descriptor scalar type.
  typedef T D;

  /// Represents an invalid or uninitialized value.
  static constexpr T kDefaultValue = std::numeric_limits<T>::quiet_NaN();
  static inline bool IsInvalidValue(T value) { return std::isnan(value); }

  /// Constructs a point cloud of a given `new_size`, with the prescribed
  /// `fields`. If `kDescriptors` is one of the fields, then
  /// `descriptor` should included and should not be `kNone`.
  /// @param new_size
  ///   Size of the point cloud after construction.
  /// @param fields
  ///   Fields that the point cloud contains.
  /// @param descriptor
  ///   Descriptor field types. @see DescriptorType
  PointCloud(int new_size, pc_flags::Fields fields = pc_flags::kXYZs,
             const pc_flags::DescriptorType& descriptor_type =
                 pc_flags::kDescriptorNone);

  /// Copies another point cloud's fields and data.
  PointCloud(const PointCloud& other)
      : PointCloud(other, pc_flags::kInherit) {}

  /// Copies another point cloud's fields and data.
  /// @param copy_fields
  ///   Fields to copy. If this is `kInherit`, then `other`s fields will be
  ///   copied.
  /// @param descriptor_type
  ///   Descriptor type to copy. Must
  // Do not define a default argument for `copy_fields` so that this is
  // not ambiguous w.r.t. the copy constructor.
  PointCloud(const PointCloud& other, pc_flags::Fields copy_fields,
             const pc_flags::DescriptorType& descriptor_type =
                 pc_flags::kDescriptorInherit);

  PointCloud& operator=(const PointCloud& other);

  ~PointCloud();

  // TODO(eric.cousineau): Consider locking the point cloud or COW to permit
  // shallow copies.

  /// Returns the fields provided by this point cloud.
  pc_flags::Fields fields() const { return fields_; }

  /// Returns the number of points in this point cloud.
  int size() const { return size_; }

  /// Conservative resize; will maintain existing data, and initialize new
  /// data to their invalid values.
  /// @param new_size
  ///    The new size of the value. If less than the present `size()`, then
  ///    the values will be truncated. If greater than the present `size()`,
  ///    then the new values will be uninitialized if `skip_initialize` is not
  ///    true.
  /// @param skip_initialize
  ///    Do not default-initialize new values.
  void resize(int new_size, bool skip_initialize = false);


  /// @name Geometric Descriptors
  /// @{

  /// Returns if this cloud provides XYZ values.
  bool has_xyzs() const;

  /// Returns access to XYZ values.
  /// This method aborts if this cloud does not provide XYZ values.
  Eigen::Ref<const Matrix3X<T>> xyzs() const;

  /// Returns mutable access to XYZ values.
  /// This method aborts if this cloud does not provide XYZ values.
  Eigen::Ref<Matrix3X<T>> mutable_xyzs();

  /// Returns access to a XYZ values.
  /// This method aborts if this cloud does not provide XYZ values.
  Vector3<T> xyz(int i) const { return xyzs().col(i); }

  /// Returns mutable access to a XYZ values.
  /// This method aborts if this cloud does not provide XYZ values.
  Eigen::Ref<Vector3<T>> mutable_xyz(int i) {
    return mutable_xyzs().col(i);
  }

  ///@}

  /// @name Run-Time Descriptors
  /// @{

  /// Returns if this point cloud provides descriptor values.
  bool has_descriptors() const;

  /// Returns if the point cloud provides a specific descriptor.
  bool has_descriptors(const pc_flags::DescriptorType& descriptor_type) const;

  /// Returns the descriptor type.
  const pc_flags::DescriptorType& descriptor_type() const {
    return descriptor_type_;
  }

  /// Returns access to descriptor values.
  /// This method aborts if this point cloud does not provide descriptor values.
  Eigen::Ref<const MatrixX<D>> descriptors() const;

  /// Returns mutable access to descriptor values.
  /// This method aborts if this point cloud does not provide descriptor values.
  Eigen::Ref<MatrixX<D>> mutable_descriptors();

  /// Returns access to a descriptor values.
  /// This method aborts if this cloud does not provide descriptor values.
  VectorX<T> descriptor(int i) const { return descriptors().col(i); }

  /// Returns mutable access to a descriptor values.
  /// This method aborts if this cloud does not provide descriptor values.
  Eigen::Ref<VectorX<T>> mutable_descriptor(int i) {
    return mutable_descriptors().col(i);
  }

  /// @}

  /// @name Container Manipulation
  /// @{

  /// Copy all points from another point cloud.
  /// @param other
  ///    Other point cloud.
  /// @param fields_in
  ///    Fields to copy. If this is `kInherit`, then both clouds must have the
  ///    exact same fields. Otherwise, both clouds must support the fields
  ///    indicated this parameter.
  /// @param allow_resize
  ///    Permit resizing to the other cloud's size.
  void SetFrom(
      const PointCloud& other,
      pc_flags::Fields fields_in = pc_flags::kInherit,
      bool allow_resize = true);

  // TODO(eric.cousineau): Add indexed version.

  /// Adds `add_size` default-initialized points.
  /// @param new_size
  ///    Number of points to add.
  /// @param skip_initialization
  ///    Do not require that the new values be initialized.
  void Expand(int add_size, bool skip_initialization = false);

  /// @}

  /// @name Fields
  /// @{

  /// Returns if a point cloud has a given set of fields.
  /// @pre If `kDescriptor` is not present in `fields_in`, then
  ///   `descriptor_type` must be `kDescriptorNone`. Otherwise,
  ///   `descriptor_type` must be a valid /// descriptor.
  bool HasFields(pc_flags::Fields fields_in,
                 const pc_flags::DescriptorType& descriptor_type_in =
                     pc_flags::kDescriptorNone) const;

  /// Requires a given set of fields.
  /// @see HasFields for preconditions.
  /// @throws std::runtime_error if this point cloud does not have these
  /// fields.
  void RequireFields(pc_flags::Fields fields_in,
                     const pc_flags::DescriptorType& descriptor_type_in =
                         pc_flags::kDescriptorNone) const;

  /// Returns if a point cloud has exactly a given set of fields.
  /// @see HasFields for preconditions.
  bool HasExactFields(pc_flags::Fields fields_in,
                      const pc_flags::DescriptorType& descriptor_type_in =
                          pc_flags::kDescriptorNone) const;

  /// Requires the exact given set of fields.
  /// @see HasFields for preconditions.
  /// @throws std::runtime_error if this point cloud does not have exactly
  /// these fields.
  void RequireExactFields(pc_flags::Fields field_set,
                          const pc_flags::DescriptorType& descriptor_type =
                              pc_flags::kDescriptorNone) const;

  /// @}

  // TODO(eric.cousineau): Add storage for indices, with SHOT as a motivating
  // example.

  // TODO(eric.cousineau): Add mechanism for handling organized / unorganized
  // point clouds.

 private:
  void SetDefault(int start, int num);

  // Provides PIMPL encapsulation of storage mechanism.
  class Storage;

  // Represents the size of the point cloud.
  int size_;
  // Represents which fields are enabled for this point cloud.
  const pc_flags::Fields fields_{};
  // Represents descriptor type stored.
  // @note If `has_descriptors()` is false, this should be `kDescriptorNone`.
  const pc_flags::DescriptorType descriptor_type_{pc_flags::kDescriptorNone};
  // Owns storage used for the point cloud.
  std::unique_ptr<Storage> storage_;
};

// TODO(eric.cousineau): Consider a way of reinterpret_cast<>ing the array
// data to permit more semantic access to members, PCL-style
// (e.g. point.x, point.r, etc: the reverse of PointCloud<>::getMatrixXfMap()).
// Need to ensure alignments are commensurate. Will only work with
// homogeneous data (possibly with heterogeneous data, if strides can be
// used).

}  // namespace perception
}  // namespace drake
