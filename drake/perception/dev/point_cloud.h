#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace perception {

using Eigen::Matrix3Xd;
using Eigen::Matrix3Xf;

// Sub-sampling?
// Image dimensions?
// Enable permanent "locking" of a point cloud?
// COW semantics?

/*
Design goals:
 - Permit modifications during lifetime, or seal after construction?
   -> This is a storage mechanism. Permit modifications.
 - Permit adding / removing features at run-time? (a la MathematicalProgram?)
   -> That may get complicated.
 - Permit constructing point cloud from data straight away?
   -> Seems like that would be very painful to implement.
 - Permit accessing all items through one index.
    -> Would be painful (a la PCL).
 - Consider storing everything together?

Indexing:
 - How to handle synchronized indexing?

Permit a subset-to-superset (indexing) of features?
 e.g. PointCloud(Point)  -->  PointCloud(Point + Normal)

Allow "RequireCapabilities" to be user-visible, for common error checking.

*/


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
 * Provides image coordinates, and the ability to conver to / from scalar
 * coordinates (a la sub2ind / ind2sub in MATLAB).
 */
struct ImageCoord {
  /// x-coordinate of the pixel.
  int u;
  /// y-coordinate of the pixel.
  int v;
};

class ImageDim {
 public:
  ImageDim(int width, int height)
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
 * Implements a contiguous point cloud.
 *
 * This is a mix between the philosophy of PCL (templated interface to
 * provide a compile-time open set, run-time closed set) and VTK (non-templated
 * interface to provide a very free form run-time open set).
 *
 * @note "contiguous" here means contiguous in memory. This was chosen to
 * avoid complications with PCL, where "dense" that the point cloud
 * corresponds to a depth image, and is indexed accordingly (densely along
 * a grid).
 */
class PointCloud {
 public:

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
    // Others: Curvature?
  };
  typedef int CapabilitySet;

  /// Geometric scalar type (e.g. for point, normals.)
  typedef float T;

  /// Represents an invalid or uninitialized value.
  static constexpr T kDefaultValue = std::numeric_limits<T>::quiet_NaN();
  static inline bool IsInvalidValue(T value) { return std::isnan(value); }

  typedef systems::sensors::ImageTraits<systems::sensors::PixelType::kRgba8U>
          ImageTraits;
  /// Color scalar type. Channels are assumed to be RGB only.
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

  CapabilitySet capabilities() const { return capabilities_; }

  Index size() const { return size_; }

  /// Conservative resize; will maintain existing data, and initialize new
  /// data to their invalid values.
  void resize(Index new_size);

  /// Add `add_size` default-initialized points, starting from old size, to
  /// new size.
  void AddPoints(int add_size, bool skip_initialization = false);

  bool has_xyz() const;
  // Lifetime is only valid as long as point cloud has not been resized.
  // References' lifetimes should be minimal.
  Eigen::Ref<const Matrix3X<T>> xyzs() const;
  Eigen::Ref<Matrix3X<T>> mutable_xyzs();
  // For algorithms needing fast access, do NOT use this accessor. Use the
  // entire reference.
  Vector3<T> xyz(Index i) const { return xyzs().col(i); }
  Eigen::Ref<Vector3<T>> mutable_xyz(Index i) {
    return mutable_xyzs().col(i);
  }

  bool has_color() const;
//  Eigen::Ref<const MatrixNX<NC, C>> colors() const;
//  Eigen::Ref<MatrixNX<NC, C>> mutable_colors();

  bool has_normal() const;
//  Eigen::Ref<const Matrix3X<T>> normals() const;
//  Eigen::Ref<Matrix3X<T>> mutable_normals();

  const FeatureType& feature_type() const { return feature_type_; }
  bool has_feature() const;
  bool has_feature(const FeatureType& feature_type) const;
//  Eigen::Ref<const MatrixX<F>> features() const;
//  Eigen::Ref<MatrixX<F>> mutable_features() const;

  void MergeFrom(const PointCloud& other);

  /// Copy all points from another point cloud.
  void CopyFrom(
      const PointCloud& other,
      CapabilitySet c = kInherit,
      bool allow_subset = false,
      bool allow_resize = true);

//  void CopyFrom(
//      const PointCloud& other,
//      const Indices& indices,
//      CapabilitySet c = kInherit,
//      bool allow_subset = false,
//      bool allow_resize = true);

//  // Transform points/normals given rigid or affine transform.
//  // Note that normals will only have the linear portion applied, not the
//  // translation.
//  void TransformInPlace(const Eigen::Isometry3d& X);
//  void AffineTransformInPlace(const Eigen::Affine3d& T);

//  /// Indicates if this point cloud has been sampled from an image, and has
//  /// not be resized since then. If so, then `image_dim()` may be used.
//  bool has_image_dim() const;
//  /// @throws std::exception if no image dimension is associated.
//  ImageDim image_dim() const;

  /// Requires a given set of capabilities.
  /// @throws std::runtime_error if this point cloud does not have these
  /// capabilities.
  bool HasCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;
  void RequireCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

  bool HasExactCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;
  void RequireExactCapabilities(
      CapabilitySet c,
      const FeatureType& feature_type = kFeatureNone) const;

 private:
  int size_;
  CapabilitySet capabilities_;
  const FeatureType feature_type_;

  class Storage;
  std::unique_ptr<Storage> storage_;

  void SetDefault(int start, int num);
};

}  // namespace perception
}  // namespace drake
