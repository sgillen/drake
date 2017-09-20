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

  auto CheckFields = [count](auto fields_expected,
                         auto mutable_fields, auto fields,
                         auto mutable_field, auto field) {
    PointCloud cloud(count, PointCloud::kXYZ);
    // Expect the values to be default-initialized.
    EXPECT_TRUE(mutable_fields(cloud).array().isNaN().all());

    // Set values using the mutable accessor.
    mutable_fields(cloud) = fields_expected;
    EXPECT_TRUE(CompareMatrices(fields_expected, mutable_fields(cloud)));
    EXPECT_TRUE(CompareMatrices(fields_expected, fields(cloud)));
    // Check element-by-element.
    for (int i = 0; i < count; ++i) {
      const auto field_expected = fields_expected.col(i);
      EXPECT_TRUE(CompareMatrices(field_expected, field(cloud, i)));
      EXPECT_TRUE(CompareMatrices(field_expected, mutable_field(cloud, i)));
    }

    // Add item which should be default-initialized.
    int start = cloud.size();
    cloud.AddPoints(1);
    // Check default-initialized.
    EXPECT_TRUE(mutable_field(cloud, start).array().isNaN().all());
    // Ensure that we preserve the values.
    EXPECT_TRUE(
      CompareMatrices(fields_expected, fields(cloud).middleCols(0, start)));
  };

  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(xyzs_expected,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });
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

  // Check invalid capabilities.
  {
    EXPECT_THROW(PointCloud(1, 0), std::runtime_error);
    EXPECT_THROW(PointCloud(1, 100), std::runtime_error);
    EXPECT_THROW(PointCloud(1, -100), std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
