#include "drake/perception/dev/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Matrix3Xf;
using Eigen::Matrix4Xf;

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;


namespace drake {
namespace perception {
namespace {

typedef Eigen::Matrix<uint8_t, 4, Eigen::Dynamic> Matrix4XU;

template <typename T>
struct check_helper {
  template <typename XprType>
  static bool IsDefault(const XprType& xpr) {
    return xpr.array().isNaN().all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult Compare(const XprTypeA& a, const XprTypeB& b) {
    return CompareMatrices(a, b);
  }
};

template <>
struct check_helper<uint8_t> {
  template <typename XprType>
  static bool IsDefault(const XprType& xpr) {
    return (xpr.array() == PointCloud::kDefaultColor).all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult Compare(const XprTypeA& a, const XprTypeB& b) {
    // Do not use `CompareMatrices`, as it is not geared for unsigned values.
    return (a.array() == b.array()).all() ?
           AssertionSuccess() : AssertionFailure();
  }
};

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;

  auto CheckFields = [count](auto fields_expected, PointCloud::CapabilitySet c,
                             const ExtraType& extra_type,
                             auto mutable_fields, auto fields,
                             auto mutable_field, auto field) {
    PointCloud cloud(count, c, extra_type);
    EXPECT_EQ(count, cloud.size());
    typedef decltype(fields_expected) XprType;
    typedef typename XprType::Scalar T;

    // Shim normal comparison to use unsigned-type friendly comparators.
    auto CompareMatrices = [](const auto& a, const auto& b) {
      return check_helper<T>::Compare(a, b);
    };

    // Expect the values to be default-initialized.
    EXPECT_TRUE(check_helper<T>::IsDefault(mutable_fields(cloud)));

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
    EXPECT_EQ(count + 1, cloud.size());
    // Check default-initialized.
    EXPECT_TRUE(check_helper<T>::IsDefault(field(cloud, last)));
    // Ensure that we preserve the values.
    EXPECT_TRUE(
        CompareMatrices(fields_expected, fields(cloud).middleCols(0, last)));

    // Resize to a size smaller.
    int small_size = 3;
    cloud.resize(small_size);
    EXPECT_EQ(small_size, cloud.size());
    EXPECT_TRUE(
        CompareMatrices(fields_expected.middleCols(0, small_size),
                        fields(cloud)));

    // Resize to a size larger.
    int large_size = 6;
    cloud.resize(large_size);
    EXPECT_EQ(large_size, cloud.size());
    EXPECT_TRUE(
        CompareMatrices(fields_expected.middleCols(0, small_size),
                        fields(cloud).middleCols(0, small_size)));
    EXPECT_TRUE(
        check_helper<T>::IsDefault(
            fields(cloud).middleCols(small_size, large_size - small_size)));
  };

  // TODO(eric.cousineau): Iterate through all field combinations.

  // Points.
  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(xyzs_expected, PointCloud::kXYZs, kExtraNone,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });

  // Colors.
  Matrix4XU colors_expected(4, count);
  colors_expected.transpose() <<
    1, 2, 3, 4,
    10, 20, 30, 40,
    110, 120, 130, 140,
    5, 6, 7, 8,
    150, 160, 170, 180;
  CheckFields(colors_expected, PointCloud::kColors, kExtraNone,
              [](PointCloud& cloud) { return cloud.mutable_colors(); },
              [](PointCloud& cloud) { return cloud.colors(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_color(i); },
              [](PointCloud& cloud, int i) { return cloud.color(i); });

  // Normals.
  Matrix3Xf normals_expected(3, count);
  normals_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(normals_expected, PointCloud::kNormals, kExtraNone,
              [](PointCloud& cloud) { return cloud.mutable_normals(); },
              [](PointCloud& cloud) { return cloud.normals(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_normal(i); },
              [](PointCloud& cloud, int i) { return cloud.normal(i); });

  // Extras (Curvature).
  Matrix3Xf extras_expected(3, count);
  extras_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(extras_expected, PointCloud::kExtras, kExtraPFH,
              [](PointCloud& cloud) { return cloud.mutable_extras(); },
              [](PointCloud& cloud) { return cloud.extras(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_extra(i); },
              [](PointCloud& cloud, int i) { return cloud.extra(i); });
}

GTEST_TEST(PointCloudTest, Capabilities) {
  // Check human-friendly formatting.
  EXPECT_EQ("(kXYZs | kNormals | kExtras::Curvature)",
            ToString(PointCloud::kXYZs | PointCloud::kNormals |
                     PointCloud::kExtras, kExtraCurvature));

  // Check zero-size.
  {
    PointCloud cloud(0, PointCloud::kXYZs | PointCloud::kNormals);
    EXPECT_EQ(0, cloud.size());
  }

  // Check basic requirements.
  {
    PointCloud cloud(1, PointCloud::kXYZs);
    EXPECT_TRUE(cloud.HasCapabilities(PointCloud::kXYZs));
    EXPECT_NO_THROW(cloud.RequireCapabilities(PointCloud::kXYZs));
    EXPECT_FALSE(cloud.HasCapabilities(PointCloud::kNormals));
    EXPECT_THROW(cloud.RequireCapabilities(PointCloud::kNormals),
                 std::runtime_error);
  }

  // Check with exact capabilities.
  {
    PointCloud cloud(1, PointCloud::kXYZs | PointCloud::kNormals);
    EXPECT_TRUE(cloud.HasExactCapabilities(
        PointCloud::kXYZs | PointCloud::kNormals));
    EXPECT_NO_THROW(cloud.RequireExactCapabilities(
            PointCloud::kXYZs | PointCloud::kNormals));
    EXPECT_FALSE(cloud.HasExactCapabilities(PointCloud::kNormals));
    EXPECT_THROW(cloud.RequireExactCapabilities(PointCloud::kNormals),
                 std::runtime_error);
  }

  // Check invalid capabilities.
  {
    EXPECT_THROW(PointCloud(1, 0), std::runtime_error);
    EXPECT_THROW(PointCloud(1, 100), std::runtime_error);
    EXPECT_THROW(PointCloud(1, -100), std::runtime_error);
  }

  // Check with extras.
  {
    PointCloud cloud(1, PointCloud::kExtras, kExtraCurvature);
    EXPECT_TRUE(cloud.has_extras());
    EXPECT_TRUE(cloud.has_extras(kExtraCurvature));
    EXPECT_FALSE(cloud.has_extras(kExtraPFH));

    // Negative tests for `has_extras`.
    PointCloud simple_cloud(1, PointCloud::kXYZs);
    EXPECT_FALSE(simple_cloud.has_extras());
    EXPECT_FALSE(simple_cloud.has_extras(kExtraCurvature));

    // Negative tests for construction.
    EXPECT_THROW(PointCloud(1, PointCloud::kExtras, kExtraNone),
                 std::runtime_error);
    EXPECT_THROW(PointCloud(1, PointCloud::kXYZs, kExtraCurvature),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
