#include "drake/perception/dev/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

using Eigen::Matrix4Xf;

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;


namespace drake {
namespace perception {
namespace {

typedef Eigen::Matrix<PointCloud::C, 4, Eigen::Dynamic> Matrix4XC;

template <typename T>
struct check_helper {
  template <typename XprType>
  static bool is_default(const XprType& xpr) {
    return xpr.array().isNaN().all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult compare(const XprTypeA& a, const XprTypeB& b) {
    return CompareMatrices(a, b);
  }
};

template <>
struct check_helper<PointCloud::C> {
  template <typename XprType>
  static bool is_default(const XprType& xpr) {
    const PointCloud::C def = PointCloud::kDefaultColor;
    return (xpr.array() == def).all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult compare(const XprTypeA& a, const XprTypeB& b) {
    // Do not use `CompareMatrices`, as it is not geared for unsigned values.
    return (a.array() == b.array()).all() ?
           AssertionSuccess() : AssertionFailure();
  }
};

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;

  auto CheckFields = [count](auto fields_expected, PointCloud::CapabilitySet c,
                             const FeatureType& feature_type,
                             auto mutable_fields, auto fields,
                             auto mutable_field, auto field) {
    PointCloud cloud(count, c, feature_type);
    EXPECT_EQ(count, cloud.size());
    typedef decltype(fields_expected) XprType;
    typedef typename XprType::Scalar T;

    // Shim normal comparison to use unsigned-type friendly comparators.
    auto CompareMatrices = [](const auto& a, const auto& b) {
      return check_helper<T>::compare(a, b);
    };

    // Expect the values to be default-initialized.
    EXPECT_TRUE(check_helper<T>::is_default(mutable_fields(cloud)));

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
    EXPECT_TRUE(check_helper<T>::is_default(field(cloud, last)));
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
        check_helper<T>::is_default(
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
  CheckFields(xyzs_expected, PointCloud::kXYZ, kFeatureNone,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });

  // Colors.
  Matrix4XC colors_expected(4, count);
  colors_expected.transpose() <<
    1, 2, 3, 4,
    10, 20, 30, 40,
    110, 120, 130, 140,
    5, 6, 7, 8,
    150, 160, 170, 180;
  CheckFields(colors_expected, PointCloud::kColor, kFeatureNone,
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
  CheckFields(normals_expected, PointCloud::kNormal, kFeatureNone,
              [](PointCloud& cloud) { return cloud.mutable_normals(); },
              [](PointCloud& cloud) { return cloud.normals(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_normal(i); },
              [](PointCloud& cloud, int i) { return cloud.normal(i); });

  // Features (PFH).
  Matrix3Xf features_expected(3, count);
  features_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(features_expected, PointCloud::kFeature, kFeaturePFH,
              [](PointCloud& cloud) { return cloud.mutable_features(); },
              [](PointCloud& cloud) { return cloud.features(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_feature(i); },
              [](PointCloud& cloud, int i) { return cloud.feature(i); });
}

GTEST_TEST(PointCloudTest, Capabilities) {
  // Check human-friendly formatting.
  EXPECT_EQ("(kXYZ | kNormal | kFeature::PFH)",
            ToString(PointCloud::kXYZ | PointCloud::kNormal |
                     PointCloud::kFeature, kFeaturePFH));

  // Check zero-size.
  {
    PointCloud cloud(0, PointCloud::kXYZ | PointCloud::kNormal);
    EXPECT_EQ(0, cloud.size());
  }

  // Check basic requirements.
  {
    PointCloud cloud(1, PointCloud::kXYZ);
    EXPECT_TRUE(cloud.HasCapabilities(PointCloud::kXYZ));
    EXPECT_NO_THROW(cloud.RequireCapabilities(PointCloud::kXYZ));
    EXPECT_FALSE(cloud.HasCapabilities(PointCloud::kNormal));
    EXPECT_THROW(cloud.RequireCapabilities(PointCloud::kNormal),
                 std::runtime_error);
  }

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

  // Check with features.
  {
    PointCloud cloud(1, PointCloud::kFeature, kFeaturePFH);
    EXPECT_TRUE(cloud.has_features());
    EXPECT_TRUE(cloud.has_features(kFeaturePFH));
    EXPECT_FALSE(cloud.has_features(kFeatureSHOT));

    // Negative tests for `has_features`.
    PointCloud simple_cloud(1, PointCloud::kXYZ);
    EXPECT_FALSE(simple_cloud.has_features());
    EXPECT_FALSE(simple_cloud.has_features(kFeaturePFH));

    // Negative tests for construction.
    EXPECT_THROW(PointCloud(1, PointCloud::kFeature, kFeatureNone),
                 std::runtime_error);
    EXPECT_THROW(PointCloud(1, PointCloud::kXYZ, kFeaturePFH),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
