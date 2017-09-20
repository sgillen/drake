#include "drake/perception/dev/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace perception {
namespace {

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;
  PointCloud cloud(5, PointCloud::kXYZ);

  // Expect the values to be default-initialized.
  EXPECT_TRUE(cloud.xyzs().array().isNaN().all());

  // Set values using the mutable accessor.
  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;

  cloud.mutable_xyzs() = xyzs_expected;
  EXPECT_TRUE(CompareMatrices(xyzs_expected, cloud.mutable_xyzs()));
  EXPECT_TRUE(CompareMatrices(xyzs_expected, cloud.xyzs()));
  // Check element-by-element.
  for (int i = 0; i < count; ++i) {
    const auto xyz_expected = xyzs_expected.col(i);
    EXPECT_TRUE(CompareMatrices(xyz_expected.col(i), cloud.xyz(i)));
    EXPECT_TRUE(CompareMatrices(xyz_expected.col(i), cloud.mutable_xyz(i)));
  }

  // Add item which should be default-initialized.
  int start = cloud.size();
  cloud.AddPoints(1);
  // Check default-initialized.
  EXPECT_TRUE(cloud.mutable_xyz(start).array().isNaN().all());
  // Ensure that we preserve the values.
  EXPECT_TRUE(
    CompareMatrices(xyzs_expected, cloud.xyzs().middleCols(0, start)));
}

GTEST_TEST(PointCloudTest, Capabilities) {
  // Check basic requirements.
  {
    PointCloud cloud(1, PointCloud::kXYZ);
    EXPECT_TRUE(cloud.HasCapabilities(PointCloud::kXYZ));
    EXPECT_NO_THROW(cloud.RequireCapabilities(PointCloud::kXYZ));
    EXPECT_FALSE(cloud.HasCapabilities(PointCloud::kNormal));
    EXPECT_THROW(cloud.RequireCapabilities(PointCloud::kNormal),
                 std::runtime_error);
  }

  // Check with features.

  // Check with exact capabilities.
  {
    PointCloud cloud(1, PointCloud::kXYZ | PointCloud::kNormal);
    EXPECT_TRUE(cloud.HasExactCapabilities(
        PointCloud::kXYZ | PointCloud::kNormal));
    EXPECT_NO_THROW(cloud.RequireExactCapabilities(
            PointCloud::kXYZ | PointCloud::kNormal));
    EXPECT_FALSE(cloud.HasExactCapabilities(PointCloud::kNormal));
    EXPECT_THROW(cloud.RequireExactCapabilities(PointCloud::kNormal),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
