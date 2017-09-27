#include "drake/perception/point_cloud_flags.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

namespace drake {
namespace perception {

namespace pcf = pc_flags;

namespace {

GTEST_TEST(PointCloudFlagsTest, Basic) {
  // Check human-friendly formatting.
  {
    std::ostringstream os;
    os << (pcf::kXYZs | pcf::kDescriptorCurvature);
    EXPECT_EQ("(kXYZs | kDescriptors::Curvature)", os.str());
  }

  // Check basics.
  pcf::Fields lhs = pcf::kXYZs;
  pcf::Fields rhs = pcf::kXYZs;
  EXPECT_EQ(lhs, rhs);
  EXPECT_TRUE(lhs & pcf::kXYZs);
  EXPECT_FALSE(lhs & pcf::kDescriptorCurvature);
  lhs |= pcf::kDescriptorFPFH;
  EXPECT_NE(lhs, rhs);
  EXPECT_TRUE(lhs & pcf::kDescriptorFPFH);

  // Check implicit conversion.
  EXPECT_EQ(pcf::Fields(pcf::kXYZs), pcf::kXYZs);
  EXPECT_EQ(pcf::Fields(pcf::kNone), pcf::Fields(pcf::kDescriptorNone));
}

GTEST_TEST(PointCloudTest, Fields) {

  // Check zero-size.
  {
    PointCloud cloud(0, pcf::kXYZs);
    EXPECT_EQ(0, cloud.size());
  }

  // Check basic requirements.
  {
    PointCloud cloud(1, pcf::kXYZs);
    EXPECT_TRUE(cloud.has_xyzs());
    EXPECT_TRUE(cloud.HasFields(pcf::kXYZs));
    EXPECT_NO_THROW(cloud.RequireFields(pcf::kXYZs));
    EXPECT_FALSE(cloud.HasFields(pcf::kDescriptorFPFH));
    EXPECT_THROW(cloud.RequireFields(pcf::kDescriptorFPFH),
                 std::runtime_error);
  }

  // Check with exact fields.
  {
    PointCloud cloud(1, pcf::kXYZs | pcf::kDescriptorCurvature);
    EXPECT_TRUE(cloud.HasExactFields(
        pcf::kXYZs | pcf::kDescriptorCurvature));
    EXPECT_NO_THROW(cloud.RequireExactFields(
        pcf::kXYZs | pcf::kDescriptorCurvature));
    EXPECT_FALSE(cloud.HasExactFields(pcf::kXYZs));
    EXPECT_THROW(cloud.RequireExactFields(pcf::kXYZs),
                 std::runtime_error);
  }

  // Check invalid fields.
  {
    EXPECT_THROW(PointCloud(1, 0), std::runtime_error);
    EXPECT_THROW(PointCloud(1, 100), std::runtime_error);
    EXPECT_THROW(PointCloud(1, -100), std::runtime_error);
  }

  // Check with descriptors.
  {
    PointCloud cloud(1, pcf::kDescriptorCurvature);
    EXPECT_FALSE(cloud.has_xyzs());
    EXPECT_TRUE(cloud.has_descriptors());
    EXPECT_TRUE(cloud.has_descriptors(pcf::kDescriptorCurvature));
    EXPECT_FALSE(cloud.has_descriptors(pcf::kDescriptorFPFH));

    // Negative tests for `has_descriptors`.
    PointCloud simple_cloud(1, pcf::kXYZs);
    EXPECT_FALSE(simple_cloud.has_descriptors());
    EXPECT_FALSE(simple_cloud.has_descriptors(pcf::kDescriptorCurvature));

    // Negative tests for construction.
    EXPECT_THROW(PointCloud(1, pcf::kNone),
                     std::runtime_error);
    EXPECT_THROW(PointCloud(1, pcf::kDescriptorNone),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
