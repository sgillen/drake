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
 *
 * @note This is defined as follows to enable an open set of features, but
 * ensure that features are appropriately matched.
 */
class FeatureType {
 public:
  FeatureType(int size, const std::string& name)
    : size_(size),
      name_(name) {}
  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  bool operator==(const FeatureType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  bool operator!=(const FeatureType& other) const {
    return !(*this == other);
  }
 private:
  const int size_;
  const std::string name_;
};

// Descriptor?
const FeatureType kFeatureNone(0, "None");
const FeatureType kFeatureInherit(-1, "Inherit");
const FeatureType kFeaturePFH(3, "PFH");
// TODO(eric.cousineau): Consider a way of reinterpret_cast<>ing the array
// data (strides taken into account) to permit more semantic access to members,
// PCL-style. Need to ensure alignments are commensurate.
const FeatureType kFeatureSHOT(1334, "SHOT");

/**
 * Provides image coordinates.
 */
struct ImageCoord {
  /// x-coordinate of the pixel.
  int u{};
  /// y-coordinate of the pixel.
  int v{};
};

/**
 * Provides image dimensions and the ability to conver to / from scalar
 * coordinates (similar to `sub2ind` / `ind2sub` in MATLAB).
 */
class ImageDimensions {
 public:
  ImageDimensions(int width, int height)
      : width_(width), height_(height) {
    DRAKE_DEMAND(width_ > 0 && height_ > 0);
  }
  inline int size() const {
    return width_ * height_;
  }
  inline int GetIndex(ImageCoord coord) const {
    DRAKE_ASSERT(coord.u >= 0 && coord.u < width_);
    DRAKE_ASSERT(coord.v >= 0 && coord.v < height_);
    return width_ * coord.v + coord.u;
  }
  inline ImageCoord GetCoord(int index) const {
    DRAKE_ASSERT(index >= 0);
    DRAKE_ASSERT(index < size());
    ImageCoord coord{index % width_, index / width_};
    return coord;
  }
 private:
  int width_;
  int height_;
};

/**
 * Implements a point cloud (with continuous storage).
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
 *  - normals - Normals in Cartesian space (float[3]).
 *  - features - An open-set of features (PFH, SHOT, etc) (float[X]).
 *
 * @note "contiguous" here means contiguous in memory. This was chosen to
 * avoid complications with PCL, where "dense" that the point cloud
 * corresponds to a depth image, and is indexed accordingly (densely along
 * a grid).
 *
 * @note The accessors / mutators for the point fields of this class returns
 * references to the original Eigen matrices. This implies that they are
 * invalidated whenever memory is reallocated for the values. Given this,
 * minimize the lifetime of these references to be as short as possible.
 */
class PointCloud {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointCloud)

  /// Indicates the data the point cloud stores.
  enum Capability : int {
    // Inherit other capabilities. May imply an intersection of all
    // compatible features.
    kInherit = 0,
    // Points in Cartesian space.
    kXYZ = 1 << 0,
    // Color, in RGB
    kColor = 1 << 1,
    // Normals (at each vertex).
    kNormal = 1 << 2,
    /// Must enable features using `EnableFeatures`. If attempting to
    /// construct a point cloud
    kFeature = 1 << 3,
  };
  typedef int CapabilitySet;

  /// Geometric scalar type (e.g. for point, normals.)
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
             CapabilitySet capabilities = kXYZ,
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

  /// Indicates if this point cloud has been sampled from an image, and has
  /// not been resized since then. If so, then `image_dimensions()` may be used.
  bool has_image_dimensions() const;

  /// Sets the image dimensions associated with this point cloud.
  void set_image_dimensions(const ImageDimensions& dim);

  /// Gets the image dimensions associated with this point cloud.
  /// @throws std::exception if no image dimension is associated.
  ImageDimensions image_dimensions() const;

  /// Returns if this point cloud provides XYZ points.
  bool has_xyzs() const;
  Eigen::Ref<const Matrix3X<T>> xyzs() const;
  Eigen::Ref<Matrix3X<T>> mutable_xyzs();
  // For algorithms needing fast access, do NOT use this accessor. Use the
  // entire reference.
  Vector3<T> xyz(Index i) const { return xyzs().col(i); }
  Eigen::Ref<Vector3<T>> mutable_xyz(Index i) {
    return mutable_xyzs().col(i);
  }

  bool has_colors() const;
  Eigen::Ref<const MatrixNX<NC, C>> colors() const;
  Eigen::Ref<MatrixNX<NC, C>> mutable_colors();
  VectorN<NC, C> color(Index i) const { return colors().col(i); }
  Eigen::Ref<VectorN<NC, C>> mutable_color(Index i) {
    return mutable_colors().col(i);
  };

  bool has_normals() const;
  Eigen::Ref<const Matrix3X<T>> normals() const;
  Eigen::Ref<Matrix3X<T>> mutable_normals();
  Vector3<T> normal(Index i) const { return normals().col(i); }
  Eigen::Ref<Vector3<T>> mutable_normal(Index i) {
    return mutable_normals().col(i);
  }

  const FeatureType& feature_type() const { return feature_type_; }
  bool has_features() const;
  bool has_features(const FeatureType& feature_type) const;
  Eigen::Ref<const MatrixX<F>> features() const;
  Eigen::Ref<MatrixX<F>> mutable_features();
  Vector3<T> feature(Index i) const { return features().col(i); }
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
  int size_;
  optional<ImageDimensions> image_dimensions_;

  // Represents which capabilities are enabled for this point cloud.
  CapabilitySet capabilities_;
  // Feature type stored (if `has_features()` is true).
  const FeatureType feature_type_;

  // Provides PIMPL encapsulation of storage mechanism.
  class Storage;
  std::unique_ptr<Storage> storage_;

  void SetDefault(int start, int num);
};

/// Returns a human-friendly representation of the capabilities of a given
/// point cloud.
std::string ToString(PointCloud::CapabilitySet c, const FeatureType &f);

}  // namespace perception
}  // namespace drake
