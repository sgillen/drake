#pragma once

#include <memory>

#include <Eigen/Dense>

namespace drake {
namespace perception {

using Eigen::Matrix3Xd;
using Eigen::Matrix3Xf;

// Sub-sampling?
// Image dimensions?
// Enable permanent "locking" of a point cloud?

/**
 * Describes a feature with a name and the feature's size.
 *
 * @note This is defined as follows to enable an open set of features, but
 * ensure that features are appropriately matched.
 */
class Feature {
 public:
  Feature(int size, const std::string& name)
    : size_(size),
      name_(name) {}
  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  bool operator==(const Feature& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
 private:
  const int size_;
  const std::string name_;
};

const Feature PFH(3, "PFH");
const Feature SHOT(3, "SHOT");
const Feature FPCS(4, "FPCS");

/**
 * Implements a contiguous point cloud.
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
    kAllPossible = 0,
    kPoints = 1 << 0,
    kColors = 1 << 1,
    kNormals = 1 << 2,
    /// Must enable features using `EnableFeatures`. If attempting to
    /// construct a point cloud
    kFeatures = 1 << 3,
    kAll = kPoints | kColors | kNormals | kFeatures,
  };

  /// Geometric scalar type (e.g. for point, normals.)
  typedef double T;
  /// Color scalar type. Channels are assumed to be RGB only.
  /// Use the feature
  typedef float C;
  /// Feature scalar type.
  typedef double F;
  /// Index type.
  typedef int Index;

  PointCloud(const Matrix3X<T>& points);
  PointCloud(const Matrix3X<T>& points,
             const Matrix3X<C>& colors);
  PointCloud(const Matrix3X<T>& points);

  PointCloud(const PointCloud& other,
             Capabilities copy_capabilities = kAllPossible);

  PointCloud(Capabilities capabilities,
             Index new_size = 0);

  Capabilities capabilities() const;
  void AddCapabilities(Capabilities c);
  void RemoveCapabilities(Capabilities c);

  Index size() const;
  void resize(Index new_size);

  bool has_points() const;
  Eigen::Ref<const Matrix3X<T>> points() const;
  Eigen::Ref<Matrix3X<T>> mutable_points();
  Eigen::Ref<const Vector3<T>> point(Index i) const;
  Eigen::Ref<Vector3<T>> mutable_point(Index i);

  Index AddPoint(const Vector3<T>& point);
  void RemovePoint(int index);

  bool has_colors() const;
  Eigen::Ref<const Matrix3X<C>> colors() const;
  Eigen::Ref<Matrix3X<C>> mutable_colors() const;

  bool has_normals() const;
  Eigen::Ref<const Matrix3X<T>> normals() const;
  Eigen::Ref<Matrix3X<T>> mutable_normals() const;

  void EnableFeatures(Index feature_size, const std::string& feature_name);
  bool has_features() const;
  Eigen::Ref<const MatrixX<F>> features() const;
  Eigen::Ref<MatrixX<F>> mutable_features() const;

  void MergeInPlace(const PointCloud& other);

  // Bundle items together? Point + color, point + normal, etc.?

 private:
  // Enable direct access to VTK pointer?

  std::unique_ptr<Impl> impl_;
  void RequireCapabilities(Capabilities expected);
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
}

}  // namespace perception
}  // namespace drake
