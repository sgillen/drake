#include "drake/systems/sensors/camera_info.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {
// This is because there is a precision difference between Ubuntu and Mac.
const double kTolerance = 1e-12;

const int kWidth = 640;
const int kHeight = 480;
const double kFx = 554.25625842204079;  // In pixels.
const double kFy = 579.41125496954282;  // In pixels.
const double kCx = kWidth * 0.5;
const double kCy = kHeight * 0.5;
const double kVerticalFov = 0.78539816339744828;  // 45.0 degrees.
const double kHorizontalFovFx = 2 * atan(kWidth / (2 * kFx));
const double kHorizontalFovFy = 2 * atan(kWidth / (2 * kFy));

void Verify(
    const Eigen::Matrix3d& expected,
    const Eigen::Vector2d& expected_fov,
    const CameraInfo& dut) {
  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(expected(0, 0), dut.focal_x(), kTolerance);
  EXPECT_NEAR(expected(1, 1), dut.focal_y(), kTolerance);
  EXPECT_NEAR(expected(0, 2), dut.center_x(), kTolerance);
  EXPECT_NEAR(expected(1, 2), dut.center_y(), kTolerance);
  EXPECT_NEAR(expected_fov(0), dut.fov_x(), kTolerance);
  EXPECT_NEAR(expected_fov(1), dut.fov_y(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected, dut.intrinsic_matrix(),
                              kTolerance));
}

GTEST_TEST(TestCameraInfo, ConstructionTest) {
  const Eigen::Matrix3d expected(
      (Eigen::Matrix3d() << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());
  Eigen::Vector2d expected_fov(kHorizontalFovFx, kVerticalFov);

  CameraInfo dut(kWidth, kHeight, kFx, kFy, kCx, kCy);
  Verify(expected, expected_fov, dut);
}

// The focal lengths become identical with this constructor.
GTEST_TEST(TestCameraInfo, ConstructionWithFovTest) {
  /////   fov_x = 2 * atan(width / height * tan(fov_y / 2)).
  const Eigen::Matrix3d expected(
      (Eigen::Matrix3d() << kFy, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());
  Eigen::Vector2d expected_fov(kHorizontalFovFy, kVerticalFov);

  CameraInfo dut(kWidth, kHeight, kVerticalFov);
  Verify(expected, expected_fov, dut);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
