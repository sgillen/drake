#include "drake/perception/dev/point_cloud.h"

namespace drake {
namespace perception {
namespace {

void usage() {
  PointCloud cloud(5, PointCloud::kXYZ | PointCloud::kColors);

  auto xyzs = cloud.mutable_xyz();
  points.setConstant(1);
  auto colors = cloud.mutable_colors();
  colors.setConstant(0.5);

  // Add item?
  int n = 3;
  cloud.AddPoints(n);

  // Create point cloud with just normals
  PointCloud normals(0, PointCloud::kNormals);

  filters::NormalEstimation(cloud, &normals);

  // Create point cloud with points and features.
}

}  // namespace
}  // namespace perception
}  // namespace drake
