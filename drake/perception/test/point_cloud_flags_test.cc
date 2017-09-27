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
    EXPECT_EQ("(kXYZs | kDescriptorCurvature)", os.str());
  }

  // Check basics.
  pcf::Fields lhs = pcf::kXYZs;
  pcf::Fields rhs = pcf::kXYZs;
  EXPECT_EQ(lhs, rhs);
  EXPECT_TRUE(lhs.has(pcf::kXYZs));
  EXPECT_FALSE(lhs.has(pcf::kDescriptorCurvature));
  lhs |= pcf::kDescriptorFPFH;
  EXPECT_NE(lhs, rhs);
  EXPECT_TRUE(lhs.has(pcf::kDescriptorFPFH));

  // Check implicit conversion.
  EXPECT_EQ(pcf::Fields(pcf::kXYZs), pcf::kXYZs);
  EXPECT_EQ(pcf::Fields(pcf::kNone), pcf::Fields(pcf::kDescriptorNone));
}

}  // namespace
}  // namespace perception
}  // namespace drake
