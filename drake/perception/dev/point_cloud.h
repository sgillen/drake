#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace perception {

/**
 * Describes a feature with a name and the feature's size.
 * @note This is defined as follows to enable an open set of features, but
 * ensure that features are appropriately matched.
 */
class FeatureType {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FeatureType)

  FeatureType(int size, const std::string& name)
    : size_(size),
      name_(name) {}

  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  inline bool operator==(const FeatureType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  inline bool operator!=(const FeatureType& other) const {
    return !(*this == other);
  }
 private:
  int size_;
  std::string name_;
};

/// No feature.
const FeatureType kFeatureNone(0, "None");
/// Inherited feature.
const FeatureType kFeatureInherit(-1, "Inherit");
/// Curvature.
const FeatureType kFeatureCurvature(1, "Curvature");
/// Point-feature-histogram feature.
const FeatureType kFeaturePFH(3, "PFH");

/**
 * Implements a point cloud (with contiguous storage), whose main goal is to
 * offer a convenient, synchronized interface to commonly used capabilities and
 * data types applicable for basic 3D perception.
 *
 * This is a mix between the philosophy of PCL (templated interface to
 * provide a compile-time open set, run-time closed set) and VTK (non-templated
 * interface to provide a very free form run-time open set).
 * You may construct one PointCloud which will contain different sets of
 * data, but you cannot change the contained data types after construction.
 * However, you can mutate the data contained within the structure and resize
 * the structure.
 *
 * This point cloud class provides:
 *  - xyz - Cartesian XYZ coordinates (float[3]).
 *  - color - RGBA (uint8_t[4]). See ImageRgba8U.
 *  - normal - Normals in Cartesian space (float[3]).
 *  - feature - An open-set of features (PFH, SHOT, etc) (float[X]).
 *
 * @note "point" in this case means an entry in the point cloud, not solely
 * an XYZ point.
 *
 * @note "contiguous" here means contiguous in memory. This was chosen to
 * avoid complications with PCL, where "dense" implies that the point cloud
 * corresponds to a depth image, and is indexed accordingly (a grid with
 * column-major storage).
 *
 * @note The accessors / mutators for the point fields of this class returns
 * references to the original Eigen matrices. This implies that they are
 * invalidated whenever memory is reallocated for the values. Given this,
 * minimize the lifetime of these references to be as short as possible.
 * Additionally, algorithms wanting fast access to values should avoid the
 * single point accessors (e.g. `xyz(i), color(i)`).
 */
class PointCloud {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointCloud)

  /// Indicates the data the point cloud stores.
  enum Capability : int {
    /// Inherit other capabilities. May imply an intersection of all
    /// compatible features.
    kInherit = 0,
    /// XYZ point in Cartesian space.
    kXYZs = 1 << 0,
    /// Colors, in RGBA.
    kColors = 1 << 1,
    /// Normals.
    kNormals = 1 << 2,
    /// Features, whose type (and structure) is specified by `FeatureType`.
    kFeatures = 1 << 3,
  };
  typedef int CapabilitySet;

  /// Geometric scalar type (for xyz, normal, etc).
  typedef float T;

  /// Represents an invalid or uninitialized value.
  static constexpr T kDefaultValue = std::numeric_limits<T>::quiet_NaN();
  static inline bool IsInvalidValue(T value) { return std::isnan(value); }

  typedef systems::sensors::ImageTraits<systems::sensors::PixelType::kRgba8U>
          ImageTraits;
  /// Color scalar type. Channels are assumed to be RGBA only.
  typedef ImageTraits::ChannelType C;
  /// Number of channels.
  static constexpr int NC = ImageTraits::kNumChannels;
  static constexpr C kDefaultColor = 0;

  /// Feature scalar type.
  typedef T F;
  /// Index type.
  typedef int Index;
  typedef std::vector<int> Indices;

  /**
   * Constructs a point cloud of a given `new_size`, with the prescribed
   * `capabilities`. If `kFeatures` is one of the capabilities, then
   * `feature` should included and should not be `kNone`.
   * @param new_size
   * @param capabilities
   * @param feature
   */
  PointCloud(Index new_size,
             CapabilitySet capabilities = kXYZs,
             const FeatureType& feature_type = kFeatureNone);

  PointCloud(const PointCloud& other,
             CapabilitySet copy_capabilities = kInherit,
             const FeatureType& feature_type = kFeatureInherit);

  ~PointCloud();

  // TODO(eric.cousineau): Consider locking the point cloud or COW to permit
  // shallow copies.

  /**
   * Returns the capabilities provided by this point cloud.
   */
  CapabilitySet capabilities() const { return capabilities_; }

  /**
   * Returns the number of points in this point cloud.
   */
  Index size() const { return size_; }

  /**
   * Conservative resize; will maintain existing data, and initialize new
   * data to their invalid values.
   * @param new_size
   *    The new size of the value. If less than the present `size()`, then
   *    the values will be truncated. If greater than the present `size()`,
   *    then the new values will be uninitalized if `skip_initialize` is not
   *    true.
   * @param skip_initialize
   *    Do not default-initialize new values.
   */
  void resize(Index new_size, bool skip_initialize = false);


  /// Returns if this cloud provides XYZ points.
  bool has_xyzs() const;

  /// Returns access to XYZ points.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<const Matrix3X<T>> xyzs() const;

  /// Returns mutable access to XYZ points.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<Matrix3X<T>> mutable_xyzs();

  /// Returns access to a XYZ point.
  /// This method aborts if this cloud does not provide XYZ points.
  Vector3<T> xyz(Index i) const { return xyzs().col(i); }

  /// Returns mutable access to a XYZ point.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<Vector3<T>> mutable_xyz(Index i) {
    return mutable_xyzs().col(i);
  }

  /// Returns if this cloud provides color points.
  bool has_colors() const;

  /// Returns access to color points.
  /// This method aborts if this cloud does not provide color points.
  Eigen::Ref<const MatrixNX<NC, C>> colors() const;

  /// Returns mutable access to color points.
  /// This method aborts if this cloud does not provide color points.
  Eigen::Ref<MatrixNX<NC, C>> mutable_colors();

  /// Returns access to a color point.
  /// This method aborts if this cloud does not provide color points.
  VectorN<NC, C> color(Index i) const { return colors().col(i); }

  /// Returns mutable access to a color point.
  /// This method aborts if this cloud does not provide color points.
  Eigen::Ref<VectorN<NC, C>> mutable_color(Index i) {
    return mutable_colors().col(i);
  };


  /// Returns if this cloud provides normal points.
  bool has_normals() const;

  /// Returns access to normal points.
  /// This method aborts if this cloud does not provide normal points.
  Eigen::Ref<const Matrix3X<T>> normals() const;

  /// Returns mutable access to normal points.
  /// This method aborts if this cloud does not provide normal points.
  Eigen::Ref<Matrix3X<T>> mutable_normals();

  /// Returns access to a normal point.
  /// This method aborts if this cloud does not provide normal points.
  Vector3<T> normal(Index i) const { return normals().col(i); }

  /// Returns mutable access to a normal point.
  /// This method aborts if this cloud does not provide normal points.
  Eigen::Ref<Vector3<T>> mutable_normal(Index i) {
    return mutable_normals().col(i);
  }


  /// Returns if this point cloud provides feature points.
  bool has_features() const;

  /// Returns if the point cloud provides a specific feature.
  bool has_features(const FeatureType& feature_type) const;

  /// Returns the feature type.
  const FeatureType& feature_type() const { return feature_type_; }

  /// Returns access to feature points.
  /// This method aborts if this point cloud does not provide feature points.
  Eigen::Ref<const MatrixX<F>> features() const;

  /// Returns mutable access to feature points.
  /// This method aborts if this point cloud does not provide feature points.
  Eigen::Ref<MatrixX<F>> mutable_features();

  /// Returns access to a feature point.
  /// This method aborts if this cloud does not provide feature points.
  Vector3<T> feature(Index i) const { return features().col(i); }

  /// Returns mutable access to a feature point.
  /// This method aborts if this cloud does not provide feature points.
  Eigen::Ref<VectorX<T>> mutable_feature(Index i) {
    return mutable_features().col(i);
  }

  /**
   * Copy all points from another point cloud.
   * @param other
   *    Other point cloud.
   * @param c
   *    Capabilities to copy. If this is `kInherit`, then both clouds
   *    must have the exact same capabilities. Otherwise, both clouds
   *    must support the capabilities indicated by `c`.
   * @param allow_resize
   *    Permit resizing to the other cloud's size.
   */
  void SetFrom(
      const PointCloud& other,
      CapabilitySet c = kInherit,
      bool allow_resize = true);

  // TODO(eric.cousineau): Add indexed version.

  /**
   * Adds `add_size` default-initialized points.
   * @param new_size
   *    Number of points to add.
   * @param skip_initialization
   *    Do not require that the new values be initialized.
   */
  void AddPoints(int add_size, bool skip_initialization = false);

  /**
   * Adds another point cloud to this cloud.
   * @param other
   *    Other point cloud.
   * @param c
   *    Capabilities to copy.
   *    @see SetFrom for how this functions.
   */
  void AddPoints(const PointCloud& other, CapabilitySet c = kInherit);

  /// Returns if a point cloud has a given set of capabilities.
  /// @pre If `kFeature` is not present in `c`, then `feature_type` must be
  /// `kFeatureNone`. Otherwise, `feature_type` must be a valid feature.
  bool HasCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

  /// Requires a given set of capabilities.
  /// @see HasCapabilities for preconditions.
  /// @throws std::runtime_error if this point cloud does not have these
  /// capabilities.
  void RequireCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

  /// Returns if a point cloud has exactly a given set of capabilities.
  /// @see HasCapabilities for preconditions.
  bool HasExactCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

  /// Requires the exact given set of capabilities.
  /// @see HasCapabilities for preconditions.
  /// @throws std::runtime_error if this point cloud does not have exactly
  /// these capabilities.
  void RequireExactCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

 private:
  void SetDefault(int start, int num);

  // Provides PIMPL encapsulation of storage mechanism.
  class Storage;

  // Size of the point cloud.
  int size_;
  // Represents which capabilities are enabled for this point cloud.
  CapabilitySet capabilities_;
  // Feature type stored (if `has_features()` is true; otherwise this should
  // be `kFeatureNone`).
  FeatureType feature_type_;
  // Storage used for the point cloud.
  std::unique_ptr<Storage> storage_;
};

/// Returns a human-friendly representation of a capability and feature set.
std::string ToString(PointCloud::CapabilitySet c, const FeatureType &f);

// TODO(eric.cousineau): Consider a way of reinterpret_cast<>ing the array
// data to permit more semantic access to members, PCL-style.
// Need to ensure alignments are commensurate. Will only work with
// homogeneous data (possibly with heterogeneous data, if strides can be
// used).

}  // namespace perception
}  // namespace drake
