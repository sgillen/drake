#pragma once

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

/*
Philosophies:
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
 private:
  const int size_;
  const std::string name_;
};

// Descriptor?
const FeatureType kFeatureNone(0, "None");
const FeatureType kFeaturePFH(3, "PFH");
const FeatureType kFeatureSHOT(3, "SHOT");
const FeatureType kFeatureFPCS(4, "FPCS");

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
  enum Capabilities {
    // Intersection among available capabilities.
    kAllPossible = 0,
    // Points in Cartesian space.
    kPoints = 1 << 0,
    // Color, in RGB
    kColors = 1 << 1,
    // Normals (at each vertex).
    kNormals = 1 << 2,
    /// Must enable features using `EnableFeatures`. If attempting to
    /// construct a point cloud
    kFeatures = 1 << 3,
    kAll = kPoints | kColors | kNormals | kFeatures,
    // Others: Curvature?
  };

  /// Geometric scalar type (e.g. for point, normals.)
  typedef float T;

  typedef systems::sensors::ImageTraits<systems::sensors::PixelType::kRgb8U>
          ImageTraits;
  /// Color scalar type. Channels are assumed to be RGB only.
  typedef ImageTraits::ChannelType C;
  /// Number of channels.
  static constexpr int NC = ImageTraits::kNumChannels;

  /// Feature scalar type.
  typedef double F;
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
             Capabilities capabilities = kPoints,
             const FeatureType& feature_type = kFeatureNone);

  PointCloud(const PointCloud& other,
             Capabilities copy_capabilities = kAllPossible);

  Capabilities capabilities() const;

  Index size() const;
  void resize(Index new_size);

  void AddItem();

  bool has_points() const;
  // Lifetime is only valid as long as point cloud has not been resized.
  // References' lifetimes should be MINIMAL.
  Eigen::Ref<const Matrix3X<T>> points() const;
  Eigen::Ref<Matrix3X<T>> mutable_points();
  // For algorithms needing fast access, do NOT use this accessor. Use the
  // entire reference.
  Vector3<T> point(Index i) const;
  Eigen::Ref<Vector3<T>> mutable_point(Index i);

  bool has_colors() const;
  Eigen::Ref<const MatrixNX<NC, C>> colors() const;
  Eigen::Ref<MatrixNX<NC, C>> mutable_colors() const;

  bool has_normals() const;
  Eigen::Ref<const Matrix3X<T>> normals() const;
  Eigen::Ref<Matrix3X<T>> mutable_normals() const;

  const FeatureType& feature_type() const;
  bool has_features() const;
  Eigen::Ref<const MatrixX<F>> features() const;
  Eigen::Ref<MatrixX<F>> mutable_features() const;

  void MergeFrom(const PointCloud& other);

  void CopyFrom(
      const PointCloud& other,
      Capabilities c = kAllPossible,
      bool allow_subset = false,
      bool allow_resize = true);

  void CopyFrom(
      const PointCloud& other,
      const Indices& indices,
      Capabilities c = kAllPossible,
      bool allow_subset = false,
      bool allow_resize = true);

  // Transform points/normals given rigid or affine transform.
  // Note that normals will only have the linear portion applied, not the
  // translation.
  void TransformInPlace(const Eigen::Isometry3d& X);
  void AffineTransformInPlace(const Eigen::Affine3d& T);

  /// Indicates if this point cloud has been sampled from an image, and has
  /// not be resized since then. If so, then `image_dim()` may be used.
  bool has_image_dim() const;
  ImageDim image_dim() const;

  void DemandCapabilities(Capabilities expected);

 private:
  // Enable direct access to VTK pointer?
  const FeatureType feature_type_;
  class Internal;
  std::unique_ptr<Internal> internal_;
};

void usage() {
  PointCloud cloud(5, PointCloud::kPoints | PointCloud::kColors);

  cloud.mutable_points().setConstant(1);
  cloud.mutable_colors().setConstant(0.5);

  // Alternative:
  cloud.SetValues({});

  // Add item?
  int n = 3;
  cloud.AddPoints(n);

  // Create point cloud with just normals
  PointCloud normals(0, PointCloud::kNormals);

  filters::NormalEstimation(cloud, &normals);

  // Create point cloud with points and features.
}

}  // namespace perception
}  // namespace drake
