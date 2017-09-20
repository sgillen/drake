#include "drake/perception/dev/point_cloud.h"

namespace drake {
namespace perception {
namespace {

GTEST_TEST(PointCloudTest, Basic) {
  PointCloud cloud(5, PointCloud::kXYZ);

  Matrix3Xd xyzs_expected(3, 5);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;

  cloud.mutable_xyzs() = xyzs_expected;

  EXPECT_TRUE(
    CompareMatrices(
        xyzs_expected,
        cloud.xyzs()));

  // Add item which should be default-initialized.
  int start = cloud.size();
  cloud.AddPoints(1);
  EXPECT_TRUE(
    cloud.mutable_xyz(start).array().isNaN().all());
  // Ensure that we preserve the values.
  EXPECT_TRUE(
    CompareMatrices(
        xyzs_expected,
        cloud.xyzs().middleCols(0, start)));
}

GTEST_TEST(PointCloudTest, Capabilities) {
  // Check basic requirements.
  {
    PointCloud cloud(1, PointCloud::kXYZ);
    EXPECT_TRUE(cloud.HasCapabilities(PointCloud::kXYZ));
    EXPECT_FALSE(cloud.HasCapabilities(PointCloud::kNormal));
  }

  // Check with features.

  // Check with exact capabilities.
  {
    PointCloud cloud(1, PointCloud::kXYZ | PointCloud::kNormal);
    EXPECT_TRUE(cloud.HasCapabilities(PointCloud::kNormal));
    EXPECT_THROW(cloud.HasExactCapabilities(PointCloud::kNormal));
}

}  // namespace
}  // namespace perception
}  // namespace drake
