#include "drake/perception/dev/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

using Eigen::Matrix4Xf;

typedef Eigen::Matrix<uint8_t, 4, Eigen::Dynamic> Matrix4Xb;

namespace drake {
namespace perception {
namespace {

template <typename T, typename XprType>
struct is_default_impl {
  static bool run(const XprType& xpr) {
    return xpr.array().isNaN().all();
  }
};

template <typename XprType>
struct is_default_impl<uint8_t, XprType> {
  static bool run(const XprType& xpr) {
    return (xpr.array() == 0).all();
  }
};

template <typename T, typename XprType>
bool is_default(const XprType& xpr) {
  return is_default_impl<T, XprType>::run(xpr);
};

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;

  auto CheckFields = [count](auto fields_expected, PointCloud::CapabilitySet c,
                             auto mutable_fields, auto fields,
                             auto mutable_field, auto field) {
    PointCloud cloud(count, c);
    typedef decltype(fields_expected) XprType;
    typedef typename XprType::Scalar T;

    // Expect the values to be default-initialized.
    EXPECT_TRUE(is_default<T>(mutable_fields(cloud)));

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
    int last = cloud.size();
    cloud.AddPoints(1);
    // Check default-initialized.
    EXPECT_TRUE(is_default<T>(mutable_field(cloud, last)));
    // Ensure that we preserve the values.
    EXPECT_TRUE(
      CompareMatrices(fields_expected, fields(cloud).middleCols(0, last)));
  };

  // Points.
  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(xyzs_expected, PointCloud::kXYZ,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });

  // Colors.
  Matrix4Xb colors_expected(4, count);
  colors_expected.transpose() <<
    1, 2, 3, 4,
    10, 20, 30, 40,
    110, 120, 130, 140,
    5, 6, 7, 8,
    150, 160, 170, 180;
  CheckFields(colors_expected, PointCloud::kColor,
              [](PointCloud& cloud) { return cloud.mutable_colors(); },
              [](PointCloud& cloud) { return cloud.colors(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_color(i); },
              [](PointCloud& cloud, int i) { return cloud.color(i); });
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
